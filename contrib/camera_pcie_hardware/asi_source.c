// Anyloop source device for ZWO ASI cameras.
// Captures 8-bit mono frames via the ASI SDK and puts them in the pipeline as
// T_MATRIX_UCHAR so center_of_mass and udp_sink can consume them directly.
//
// Plugin URI example:
//   {"uri": "file:/home/fsl/anyloop_levfork/anyloop/build/asi_source.so", "params": {...}}
//
// Parameters:
//   "camera_name" (string): name substring to match (default: "ASI290MM")
//   "width"       (int|"auto"): ROI width; auto sizes it to the beam
//   "height"      (int|"auto"): ROI height; auto sizes it to the beam
//   "start_x"     (int|"auto"): ROI left edge; auto centers it on the beam
//   "start_y"     (int|"auto"): ROI top edge;  auto centers it on the beam
//   "exposure"    (int):    exposure time in microseconds (default: 1000)
//   "gain"        (int|"auto"): sensor gain; auto picks the brightest
//                 non-clipping gain (see below)
//   "bandwidth"   (int):    USB bandwidth %, 40–100 (default: 80)
//   "high_speed"  (int):    1 = 10-bit ADC fast readout, 0 = 12-bit (default: 0)
//   "stall_timeout_ms" (int): how long a blocking frame wait may last before
//                 the stream is declared stalled. The loop is frame-paced, so
//                 a healthy wait is ~1 frame period; anything much longer
//                 means the camera stopped streaming. Default 0 = auto:
//                 3 frame periods, measured at runtime (1 ms at 3788 fps).
//                 Until the first rate estimate (~0.5 s in), a conservative
//                 500 ms is used (the first frame after capture start
//                 routinely takes >200 ms).
//   "die_on_stall" (int): what to do when a stall is declared. 1 (default) =
//                 abort the run immediately: proc() returns an error, which
//                 is fatal for this device, so the run dies cleanly instead
//                 of continuing with a multi-hundred-ms hole in the data
//                 (DAC frozen, error accumulating). 0 = try to recover in
//                 place: stop/restart capture up to 3 times and keep going.
//
// AUTO MODE
// "auto" on width/height/start_x/start_y/gain means MEASURE IT AT STARTUP, not
// "use the built-in default". It replaces the manual probe-and-derive loop the
// configs used to carry (probe_frame.json -> find_roi.py -> paste numbers),
// which went stale every time the beam moved, was refocused, or the source
// brightness changed.
//
// Any of those five set to "auto" costs one startup probe, in this order:
//
//  1. ROI (width/height/start_x/start_y). A full-sensor capture is averaged
//     over auto_probe_frames, the background is taken as the frame median, and
//     the beam's centroid and per-axis sigma come from windowed moments
//     iterated about the brightest pixel. The ROI is then sized to
//     +/-auto_roi_sigma sigma on each axis (rounded up to the 8 px / 2 px the
//     sensor requires, floored at auto_roi_min) and centred on the centroid.
//     Only the parameters actually set to "auto" are taken from the probe; a
//     pinned width with an "auto" start still gets centred on the measured
//     beam. If no beam is found the probe falls back to the old behaviour --
//     256 px, sensor-centred -- and says so loudly.
//  2. Gain, at the FINAL ROI (a probe at the full sensor sees a different
//     readout and a different pixel mix, and the configs have been bitten by
//     exactly that before). A binary search over the camera's gain range picks
//     the LARGEST gain that keeps the saturated-pixel fraction at or under
//     auto_sat_limit and the 99.9th-percentile pixel at or under
//     auto_peak_target, i.e. the most signal available without clipping the
//     core the centroid is computed from. If even the minimum gain clips, the
//     minimum is used and warned about -- the source is too bright and needs
//     an optical or exposure fix.
//
// Exposure is deliberately NOT auto: it sets the frame rate, and therefore the
// loop period, the delay in FRAMES and every controller constant derived from
// them. Changing it is a re-identification, not a calibration.
//
//   "auto_probe_frames" (int):  frames averaged per probe step (default 8)
//   "auto_roi_sigma"  (double): ROI half-size in beam sigmas (default 4.0)
//   "auto_roi_min"    (int):    smallest auto ROI edge, px (default 32)
//   "auto_sat_limit"  (double): fraction of pixels at/over 254 that auto gain
//                     will tolerate (default 0.001)
//   "auto_peak_target" (int):   99.9th-percentile pixel value auto gain aims
//                     to stay under (default 230)

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>
#include "anyloop.h"
#include "logging.h"
#include "timing.h"
#include "xalloc.h"
#include "ASICamera2.h"

// How many proc() calls per latency-diagnostic report. At ~815 Hz this is a
// report roughly every ~2.5 s.
#define ASI_DIAG_WINDOW 2000

// How many proc() calls per aylp_timing_publish(). Shorter than the diagnostic
// window because a controller doing an auto-delay estimate has only the startup
// hold to work with: at 471 Hz this publishes about twice a second.
#define ASI_TIMING_WINDOW 256


struct asi_source_data {
	int camera_id;
	int opened;
	int capturing;
	int width;
	int height;
	long buf_size;
	gsl_matrix_uchar *matrix;

	// Latency diagnostics. The -p profiler only times how long proc() takes
	// to pull an already-buffered frame; it can't see how *old* that frame
	// is. The SDK free-runs the camera and queues frames, so comparing the
	// camera's production rate against the loop rate — and watching how many
	// frames we drain per call — reveals the queue backlog / stale-frame
	// latency that the profiler misses.
	struct timespec diag_t0;   // start of the current reporting window
	long diag_calls;           // proc() calls in this window
	long diag_frames;          // frames the camera produced this window
	int  diag_max_drained;     // worst single-call drain this window
	long diag_blocked;         // calls that found an empty queue and blocked

	// Stream-stall recovery bookkeeping. The camera firmware / SDK
	// occasionally stops delivering frames mid-stream with no USB-level
	// error (established 2026-07-16: dmesg is silent at stall moments); a
	// capture restart brings it back. Count them and report at fini.
	long stall_recoveries;
	int  stall_timeout_ms;     // configured value; 0 = auto (3 frames)
	int  cur_timeout_ms;       // effective wait right now
	int  die_on_stall;         // 1 = abort run on stall; 0 = restart capture

	// Loop-timing channel (see libaylp/timing.h). This device is the only
	// one that can separate the loop's work from its waiting: the wait
	// happens inside this proc(), so the interval from this proc()
	// RETURNING to its next call is the whole downstream pipeline --
	// centroiding, control, DAC write, sinks -- with no frame wait mixed
	// in. Averaged over a window and published for whoever needs the
	// transport delay in frames.
	struct timespec t_exit;    // when the last proc() returned
	struct timespec t_entry;   // when the last proc() was called
	int  have_exit;            // t_exit/t_entry are populated
	double tw_compute;         // sum of downstream compute times, s
	double tw_period;          // sum of proc-to-proc intervals, s
	long tw_calls;             // calls in the current timing window
	double compute_mean;       // last published compute time, s (for logs)
};


// ---- startup auto-configuration --------------------------------------------
// Everything in this section runs once, from init, before the loop exists. It
// may take a second and allocate a sensor-sized buffer; none of it is on the
// per-frame path.

// Probe reads are not frame-paced by anything, so they get a generous wait: a
// full-sensor frame is far slower than the operating ROI, and the first frame
// after a capture start routinely takes >200 ms.
#define ASI_PROBE_TIMEOUT_MS 2000

/** Beam as measured from a probe capture. Pixel units, relative to the probe
 * ROI's own origin. */
struct asi_beam {
	double cy, cx;		// centroid
	double sy, sx;		// per-axis rms width (sigma)
	double bg, peak;	// background (frame median) and peak, counts
	int found;
};

/** Exposure health of a capture, in the two terms auto gain cares about. */
struct asi_expo {
	double sat_frac;	// fraction of pixels at or over 254
	int p999;		// 99.9th-percentile pixel value
};

/** Which parameters were given as "auto", plus the knobs that shape them. */
struct asi_auto_cfg {
	int width, height, start_x, start_y, gain;	// 1 = "auto"
	int probe_frames;
	double roi_sigma;
	int roi_min;
	double sat_limit;
	int peak_target;
};


/** Pull `skip + frames` frames, accumulating the last `frames` of them into
 * `acc` (per-pixel sum, may be null) and `hist` (pooled 256-bin histogram of
 * every individual frame, may be null).
 *
 * The skip matters: ASIGetVideoData returns the OLDEST queued frame, so
 * immediately after a capture start or a gain change the first frames out of
 * the queue were still taken under the previous settings.
 *
 * `acc` is what beam finding wants (averaging kills the read noise), `hist` is
 * what exposure measurement wants (saturation is a per-frame property; it
 * averages away).
 */
static int asi_probe_grab(int id, unsigned char *buf, long n_px,
	int skip, int frames, uint32_t *acc, uint64_t *hist)
{
	if (acc) memset(acc, 0, (size_t)n_px * sizeof *acc);
	if (hist) memset(hist, 0, 256 * sizeof *hist);
	// Empty the queue first. It can hold far more than `skip` frames --
	// probes are slow and frames keep arriving while one is being reduced --
	// and every one of them was taken under the previous settings.
	if (skip) while (ASIGetVideoData(id, buf, n_px, 0) == ASI_SUCCESS) {}
	for (int i = 0; i < skip + frames; i++) {
		ASI_ERROR_CODE err = ASIGetVideoData(id, buf, n_px,
			ASI_PROBE_TIMEOUT_MS);
		if (err != ASI_SUCCESS) {
			log_error("asi_source: probe frame %d of %d failed (%d)",
				i + 1, skip + frames, err);
			return -1;
		}
		if (i < skip) continue;
		if (acc) for (long p = 0; p < n_px; p++) acc[p] += buf[p];
		if (hist) for (long p = 0; p < n_px; p++) hist[buf[p]]++;
	}
	return 0;
}


/** Locate the beam in an averaged probe frame and measure its width.
 *
 * The background is the frame MEDIAN: on a sensor-sized probe the beam is a
 * small minority of the pixels, so the median is the dark/sky level and is
 * immune to the beam itself, to hot pixels, and to a handful of dead columns.
 *
 * Centroid and second moments are then taken over a window that starts around
 * the brightest pixel and is re-sized to the beam over a few iterations. A
 * window is necessary in both directions: taken over the whole sensor, a
 * background gradient or a second spot drags the centroid; taken too tight, the
 * moment truncates and the ROI comes out too small. The gate is only 5% of the
 * peak above background, so the truncation bias on a Gaussian is a couple of
 * percent -- deliberately much lower than a "find the spot" gate, because this
 * number is a WIDTH, not a detection.
 */
static struct asi_beam asi_find_beam(const uint32_t *acc, int frames,
	int w, int h)
{
	struct asi_beam b = {0};
	long n = (long)w * h;
	uint64_t hist[256] = {0};
	double peak = 0.0;
	long peak_p = 0;
	for (long p = 0; p < n; p++) {
		double v = acc[p] / (double)frames;
		if (v > peak) { peak = v; peak_p = p; }
		int q = (int)(v + 0.5);
		hist[q > 255 ? 255 : q]++;
	}
	uint64_t half = (uint64_t)n / 2, cum = 0;
	int bgq = 0;
	while (bgq < 255 && (cum += hist[bgq]) < half) bgq++;
	b.bg = bgq;
	b.peak = peak;
	double sig = peak - b.bg;
	// 10 counts of contrast is a few times the read noise of an averaged
	// frame; below that there is nothing here worth centring an ROI on.
	if (sig < 10.0) return b;
	double gate = b.bg + (0.05 * sig > 3.0 ? 0.05 * sig : 3.0);

	double cy = peak_p / w, cx = peak_p % w;
	double sy = 0.0, sx = 0.0;
	int half_y = 64, half_x = 64;
	for (int it = 0; it < 4; it++) {
		int y0 = (int)(cy - half_y), y1 = (int)(cy + half_y);
		int x0 = (int)(cx - half_x), x1 = (int)(cx + half_x);
		if (y0 < 0) y0 = 0;
		if (x0 < 0) x0 = 0;
		if (y1 > h - 1) y1 = h - 1;
		if (x1 > w - 1) x1 = w - 1;
		double s = 0.0, my = 0.0, mx = 0.0;
		for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++) {
			double v = acc[(long)y * w + x] / (double)frames;
			if (v < gate) continue;
			double g = v - b.bg;
			s += g; my += g * y; mx += g * x;
		}
		if (s <= 0.0) return b;
		cy = my / s; cx = mx / s;
		double vy = 0.0, vx = 0.0;
		for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++) {
			double v = acc[(long)y * w + x] / (double)frames;
			if (v < gate) continue;
			double g = v - b.bg;
			vy += g * (y - cy) * (y - cy);
			vx += g * (x - cx) * (x - cx);
		}
		sy = sqrt(vy / s); sx = sqrt(vx / s);
		// re-window at 4 sigma for the next pass, bounded so one bad
		// iteration can't swallow the sensor or collapse to a pixel
		int ny = (int)ceil(4.0 * sy), nx = (int)ceil(4.0 * sx);
		half_y = ny < 8 ? 8 : (ny > 256 ? 256 : ny);
		half_x = nx < 8 ? 8 : (nx > 256 ? 256 : nx);
	}
	if (!(sy > 0.0) || !(sx > 0.0) || !isfinite(sy) || !isfinite(sx))
		return b;
	// De-bias the gate. A second moment taken only over pixels above a
	// fraction g of the peak is short of the true sigma by a factor that
	// follows from the Gaussian itself: the gate is a disk of radius
	// c*sigma with c^2 = 2 ln(1/g), and integrating the weighted x^2 over
	// that disk gives sigma_meas^2 / sigma^2 = [2 - (c^2+2) g] / [2 (1-g)].
	// For the 5% gate that is 0.918, i.e. an ROI 8% too small, which is a
	// real bite out of the wings the centroid needs. This assumes the beam
	// is roughly Gaussian; a beam that isn't gets a slightly generous ROI,
	// which is the harmless direction.
	double gfrac = (gate - b.bg) / sig;
	if (gfrac > 1e-3 && gfrac < 0.5) {
		double c2 = 2.0 * log(1.0 / gfrac);
		double f2 = (2.0 - (c2 + 2.0) * gfrac) / (2.0 * (1.0 - gfrac));
		if (f2 > 0.1 && f2 <= 1.0) {
			double f = sqrt(f2);
			sy /= f;
			sx /= f;
		}
	}
	b.cy = cy; b.cx = cx;
	b.sy = sy; b.sx = sx;
	b.found = 1;
	return b;
}


/** Measure saturation and the bright tail of the current settings. */
static int asi_measure_expo(int id, unsigned char *buf, long n_px, int frames,
	struct asi_expo *out)
{
	uint64_t hist[256];
	// 4 skipped frames clears anything queued under the previous gain
	if (asi_probe_grab(id, buf, n_px, 4, frames, NULL, hist)) return -1;
	uint64_t n = (uint64_t)n_px * frames;
	uint64_t sat = hist[254] + hist[255];
	uint64_t want = n - n / 1000;	// 99.9th percentile
	uint64_t cum = 0;
	int q = 0;
	while (q < 255 && (cum += hist[q]) < want) q++;
	out->sat_frac = n ? (double)sat / (double)n : 0.0;
	out->p999 = q;
	return 0;
}


/** Measure the beam at the full sensor and resolve every "auto" ROI field
 * from it. Leaves capture stopped and the ROI format untouched by the caller's
 * standards -- the caller sets the operating ROI afterwards either way.
 *
 * `probe_gains` is tried in order until a beam is found: with an auto gain
 * there is no trustworthy starting point, so the search starts at the darkest
 * setting (a clipped, bloomed frame measures a width that isn't the beam's)
 * and only opens up if that shows nothing.
 *
 * Returns -1 only on a camera/SDK failure. A probe that simply finds no beam
 * returns 0 and leaves the caller's values alone (the old 256 px sensor-centred
 * defaults), having said so.
 */
static int asi_probe_roi(int id, const struct asi_auto_cfg *ac,
	long sensor_w, long sensor_h, const long *probe_gains, int n_gains,
	int *width, int *height, int *start_x, int *start_y)
{
	int pw = (int)(sensor_w & ~7L), ph = (int)(sensor_h & ~1L);
	long n_px = (long)pw * ph;
	unsigned char *buf = xmalloc((size_t)n_px);
	uint32_t *acc = xmalloc((size_t)n_px * sizeof *acc);
	struct asi_beam b = {0};
	int rc = -1;

	ASI_ERROR_CODE err = ASISetROIFormat(id, pw, ph, 1, ASI_IMG_RAW8);
	if (err == ASI_SUCCESS) err = ASISetStartPos(id, 0, 0);
	if (err != ASI_SUCCESS) {
		log_error("asi_source: could not set the %dx%d probe ROI (%d)",
			pw, ph, err);
		goto out;
	}
	err = ASIStartVideoCapture(id);
	if (err != ASI_SUCCESS) {
		log_error("asi_source: probe capture failed to start (%d)", err);
		goto out;
	}
	for (int g = 0; g < n_gains && !b.found; g++) {
		err = ASISetControlValue(id, ASI_GAIN, probe_gains[g], ASI_FALSE);
		if (err != ASI_SUCCESS) {
			log_error("asi_source: could not set probe gain %ld (%d)",
				probe_gains[g], err);
			ASIStopVideoCapture(id);
			goto out;
		}
		// 4 skipped frames clears the queue of pre-change exposures
		if (asi_probe_grab(id, buf, n_px, 4, ac->probe_frames, acc, NULL)) {
			ASIStopVideoCapture(id);
			goto out;
		}
		b = asi_find_beam(acc, ac->probe_frames, pw, ph);
		if (b.found)
			log_info("asi_source: beam probe at gain %ld: centre "
				"(%.2f,%.2f), sigma (%.2f,%.2f) px [y,x], "
				"background %.1f, peak %.1f counts",
				probe_gains[g], b.cy, b.cx, b.sy, b.sx,
				b.bg, b.peak);
		else
			log_warn("asi_source: beam probe at gain %ld found no "
				"beam (background %.1f, peak %.1f counts)",
				probe_gains[g], b.bg, b.peak);
	}
	ASIStopVideoCapture(id);
	rc = 0;
	if (!b.found) {
		log_warn("asi_source: auto ROI could not find a beam on the "
			"full sensor; falling back to the %dx%d sensor-centred "
			"default. Is the source on and in the field?",
			*width, *height);
		goto out;
	}

	if (ac->width) {
		int w = (int)ceil(2.0 * ac->roi_sigma * b.sx);
		if (w < ac->roi_min) w = ac->roi_min;
		w = (w + 7) & ~7;		// the sensor wants a multiple of 8
		if (w > pw) w = pw;
		*width = w;
	}
	if (ac->height) {
		int h = (int)ceil(2.0 * ac->roi_sigma * b.sy);
		if (h < ac->roi_min) h = ac->roi_min;
		h = (h + 1) & ~1;		// ... and an even height
		if (h > ph) h = ph;
		*height = h;
	}
	// Centre on the beam, then clamp into the sensor and align. Both the
	// clamp and the alignment mask only ever move the origin down, so the
	// far edge cannot be pushed back out of the sensor.
	if (ac->start_x) {
		int x = (int)lround(b.cx) - *width / 2;
		if (x + *width > pw) x = pw - *width;
		if (x < 0) x = 0;
		*start_x = x & ~7;
	}
	if (ac->start_y) {
		int y = (int)lround(b.cy) - *height / 2;
		if (y + *height > ph) y = ph - *height;
		if (y < 0) y = 0;
		*start_y = y & ~1;
	}
	log_info("asi_source: auto ROI -> %dx%d @ (%d,%d); beam sits at "
		"in-frame (%.2f,%.2f) [y,x] of %.1f,%.1f centre",
		*width, *height, *start_x, *start_y,
		b.cy - *start_y, b.cx - *start_x,
		(*height - 1) / 2.0, (*width - 1) / 2.0);
	if (ac->width || ac->height) {
		// The clearance test the configs used to do by hand: an ROI
		// that cuts the wings biases the centroid toward the frame
		// centre, which the controller reads as real pointing motion.
		double need_y = ac->roi_sigma * b.sy, need_x = ac->roi_sigma * b.sx;
		double have_y = fmin(b.cy - *start_y, *start_y + *height - b.cy);
		double have_x = fmin(b.cx - *start_x, *start_x + *width - b.cx);
		if (have_y < need_y || have_x < need_x)
			log_warn("asi_source: auto ROI is clipped by the sensor "
				"edge: %.1f/%.1f px of clearance [y,x] against "
				"the %.1f/%.1f wanted. The beam is too close to "
				"the edge, or too wide, for %G sigma.",
				have_y, have_x, need_y, need_x, ac->roi_sigma);
	}
out:
	xfree(acc);
	xfree(buf);
	return rc;
}


/** Pick the largest gain that clips neither the saturation limit nor the
 * bright-tail target. Brightness is monotone in gain, so this bisects instead
 * of walking a ladder: ~10 measurements over a 0-600 range.
 *
 * Must be called with capture RUNNING at the operating ROI: readout mode and
 * the pixels in frame both move the numbers, and a gain chosen at the full
 * sensor has been wrong here before.
 *
 * Returns the chosen gain. *ok is 1 when a non-clipping gain was found, 0 when
 * even the minimum gain clips, and -1 when the search could not be carried out.
 */
static long asi_auto_gain(int id, unsigned char *buf, long n_px,
	const struct asi_auto_cfg *ac, long gmin, long gmax, int *ok)
{
	long lo = gmin, hi = gmax, best = gmin;
	int found = 0;
	*ok = -1;
	while (lo <= hi) {
		long mid = lo + (hi - lo) / 2;
		if (ASISetControlValue(id, ASI_GAIN, mid, ASI_FALSE)
				!= ASI_SUCCESS) {
			log_error("asi_source: could not set gain %ld during "
				"the auto-gain search", mid);
			return found ? best : gmin;
		}
		struct asi_expo e;
		if (asi_measure_expo(id, buf, n_px, ac->probe_frames, &e))
			return found ? best : gmin;
		int pass = e.sat_frac <= ac->sat_limit && e.p999 <= ac->peak_target;
		log_info("asi_source: auto gain %ld: p99.9 %d counts, %.3f%% "
			"saturated -> %s", mid, e.p999, 1e2 * e.sat_frac,
			pass ? "headroom left" : "clipping");
		if (pass) {
			best = mid;
			found = 1;
			lo = mid + 1;
		} else {
			hi = mid - 1;
		}
	}
	*ok = found;
	return found ? best : gmin;
}


int asi_source_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct asi_source_data *data = self->device_data;

	// Loop-timing window. Taken before the frame wait: the interval since
	// the last RETURN is downstream work only, the interval since the last
	// CALL is the frame period.
	struct timespec t_in;
	clock_gettime(CLOCK_MONOTONIC, &t_in);
	if (LIKELY(data->have_exit)) {
		data->tw_compute += (t_in.tv_sec - data->t_exit.tv_sec)
			+ 1e-9 * (t_in.tv_nsec - data->t_exit.tv_nsec);
		data->tw_period += (t_in.tv_sec - data->t_entry.tv_sec)
			+ 1e-9 * (t_in.tv_nsec - data->t_entry.tv_nsec);
		data->tw_calls++;
	}
	data->t_entry = t_in;
	if (UNLIKELY(data->tw_calls >= ASI_TIMING_WINDOW)) {
		data->compute_mean = data->tw_compute / data->tw_calls;
		aylp_timing_publish(data->tw_period / data->tw_calls,
			data->compute_mean, (size_t)data->tw_calls);
		data->tw_compute = 0.0;
		data->tw_period = 0.0;
		data->tw_calls = 0;
	}

	// The ASI SDK buffers frames continuously after ASIStartVideoCapture, and
	// ASIGetVideoData returns the OLDEST queued frame. If the control loop runs
	// slower than the camera, frames pile up and we end up acting on stale
	// data — a fixed dead-time in the loop that drives oscillation regardless
	// of gain. Drain the queue non-blocking (0 ms timeout) so we keep only the
	// freshest frame, then fall back to a blocking read if nothing was queued.
	int got = 0;
	int drained = 0;   // frames pulled non-blocking this call (queue depth)
	while (ASIGetVideoData(
			data->camera_id, data->matrix->data, data->buf_size, 0
		) == ASI_SUCCESS) {
		got = 1;
		drained++;
	}
	if (!got) {
		// Blocking wait for the next frame. The loop is frame-paced, so
		// a healthy wait is ~1 frame period; cur_timeout_ms (~3 frame
		// periods by default) expiring means the stream stopped.
		struct timespec st0, st1;
		clock_gettime(CLOCK_MONOTONIC, &st0);
		ASI_ERROR_CODE err = ASIGetVideoData(
			data->camera_id, data->matrix->data, data->buf_size,
			data->cur_timeout_ms
		);
		if (UNLIKELY(err == ASI_ERROR_TIMEOUT)) {
			// one grace re-read before declaring a stall, so a
			// millisecond scheduler hiccup of the SDK's worker
			// thread doesn't trigger a needless capture restart
			err = ASIGetVideoData(data->camera_id,
				data->matrix->data, data->buf_size,
				data->cur_timeout_ms);
		}
		if (UNLIKELY(err == ASI_ERROR_TIMEOUT)) {
			// Genuine stall (2026-07-16: camera firmware / SDK
			// stops streaming with the USB link clean; it never
			// comes back on its own).
			if (data->die_on_stall) {
				log_error("asi_source: stream stalled (no frame "
					"for 2x %d ms); die_on_stall is set, "
					"aborting run", data->cur_timeout_ms);
				return -1;
			}
			// Otherwise stop/restart capture and retry, up to 3
			// times, before declaring the run dead. The pipeline
			// is blocked meanwhile: the DAC holds its last command
			// for the whole gap and the loop resumes onto whatever
			// error accumulated.
			for (int try = 1; try <= 3; try++) {
				log_warn("asi_source: stream stalled (no frame "
					"for 2x %d ms); restarting capture "
					"(attempt %d/3)", data->cur_timeout_ms,
					try);
				ASIStopVideoCapture(data->camera_id);
				err = ASIStartVideoCapture(data->camera_id);
				if (err != ASI_SUCCESS) continue;
				// first frame after a restart takes a while;
				// give it a generous window
				err = ASIGetVideoData(data->camera_id,
					data->matrix->data, data->buf_size,
					500);
				if (err == ASI_SUCCESS) break;
			}
			if (err == ASI_SUCCESS) {
				data->stall_recoveries++;
				clock_gettime(CLOCK_MONOTONIC, &st1);
				double dead = (st1.tv_sec - st0.tv_sec)
					+ (st1.tv_nsec - st0.tv_nsec) / 1e9;
				log_warn("asi_source: stream recovered; "
					"~%.3f s of frames lost (stall #%ld "
					"this run)", dead,
					data->stall_recoveries);
			}
		}
		if (UNLIKELY(err != ASI_SUCCESS)) {
			log_error("ASIGetVideoData error %d", err);
			return -1;
		}
	}

	// --- latency diagnostic ------------------------------------------------
	// frames the camera actually produced since the last call: the non-blocking
	// drain count when the queue was non-empty, else the single frame we blocked
	// for. Accumulate per window, then report loop vs camera rate and backlog.
	data->diag_calls++;
	data->diag_frames += got ? drained : 1;
	if (drained > data->diag_max_drained) data->diag_max_drained = drained;
	if (!got) data->diag_blocked++;
	if (data->diag_calls >= ASI_DIAG_WINDOW) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		double dt = (now.tv_sec - data->diag_t0.tv_sec)
			+ (now.tv_nsec - data->diag_t0.tv_nsec) / 1e9;
		if (dt > 0.0) {
			double loop_hz = data->diag_calls / dt;
			double cam_hz  = data->diag_frames / dt;
			double mean_drained = (double)data->diag_frames
				/ data->diag_calls;
			// mean_drained > 1 with cam_hz > loop_hz => the camera
			// outruns the loop and frames sit queued (stale-frame
			// latency); we keep the newest, but the freshest frame
			// still carries ~1/cam_hz + exposure of pipeline latency.
			// blocked% high => loop outruns the camera: no backlog,
			// latency is just one exposure+readout+USB transfer.
			log_info("asi latency: loop %.0f Hz | camera %.0f Hz | "
				"drained/call mean %.2f max %d | blocked %.0f%% | "
				"frame period %.2f ms | downstream compute "
				"%.3f ms",
				loop_hz, cam_hz, mean_drained,
				data->diag_max_drained,
				100.0 * data->diag_blocked / data->diag_calls,
				cam_hz > 0 ? 1000.0 / cam_hz : 0.0,
				1e3 * data->compute_mean);
			// auto stall timeout: 3 frame periods at the measured
			// camera rate, at least 1 ms (the SDK takes int ms)
			if (data->stall_timeout_ms == 0 && cam_hz > 0.0) {
				int t = (int)(3000.0 / cam_hz + 0.999);
				data->cur_timeout_ms = t > 1 ? t : 1;
			}
		}
		data->diag_t0 = now;
		data->diag_calls = 0;
		data->diag_frames = 0;
		data->diag_max_drained = 0;
		data->diag_blocked = 0;
	}
	state->matrix_uchar = data->matrix;
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = (uint64_t)data->height;
	state->header.log_dim.x = (uint64_t)data->width;
	clock_gettime(CLOCK_MONOTONIC, &data->t_exit);
	data->have_exit = 1;
	return 0;
}


int asi_source_fini(struct aylp_device *self)
{
	struct asi_source_data *data = self->device_data;
	if (!data) return 0;
	log_info("asi_source: %ld stream stall(s) recovered by capture "
		"restart this run", data->stall_recoveries);
	if (data->capturing) ASIStopVideoCapture(data->camera_id);
	if (data->opened) ASICloseCamera(data->camera_id);
	if (data->matrix) xfree_type(gsl_matrix_uchar, data->matrix);
	xfree(data);
	return 0;
}


int asi_source_init(struct aylp_device *self)
{
	self->proc = &asi_source_proc;
	self->fini = &asi_source_fini;
	self->device_data = xcalloc(1, sizeof(struct asi_source_data));
	struct asi_source_data *data = self->device_data;
	data->camera_id = -1;

	// parameter defaults
	const char *camera_name = "ASI290MM";
	int width     = 256;
	int height    = 256;
	int start_x   = -1;   // <0 → center on sensor (the no-beam fallback)
	int start_y   = -1;
	long exposure = 1000;  // µs
	long gain     = 100;
	long bandwidth = 80;
	long high_speed = 0;  // 1 = 10-bit ADC fast readout
	data->die_on_stall = 1;
	// "auto" on any of these means "measure it at startup"; see the header
	int auto_width = 0, auto_height = 0;
	int auto_start_x = 0, auto_start_y = 0, auto_gain = 0;
	int probe_frames = 8;
	double roi_sigma = 4.0;
	int roi_min = 32;
	double sat_limit = 0.001;
	int peak_target = 230;

	if (!self->params) {
		log_error("No params object found");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		int is_auto = json_object_is_type(val, json_type_string)
			&& !strcmp(json_object_get_string(val), "auto");
		if (key[0] == '_') {
			// comment key
		} else if (!strcmp(key, "camera_name")) {
			camera_name = json_object_get_string(val);
		} else if (!strcmp(key, "width")) {
			if (is_auto) auto_width = 1;
			else width = json_object_get_int(val);
		} else if (!strcmp(key, "height")) {
			if (is_auto) auto_height = 1;
			else height = json_object_get_int(val);
		} else if (!strcmp(key, "start_x")) {
			if (is_auto) auto_start_x = 1;
			else start_x = json_object_get_int(val);
		} else if (!strcmp(key, "start_y")) {
			if (is_auto) auto_start_y = 1;
			else start_y = json_object_get_int(val);
		} else if (!strcmp(key, "exposure")) {
			exposure = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "gain")) {
			if (is_auto) auto_gain = 1;
			else gain = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "auto_probe_frames")) {
			probe_frames = json_object_get_int(val);
		} else if (!strcmp(key, "auto_roi_sigma")) {
			roi_sigma = json_object_get_double(val);
		} else if (!strcmp(key, "auto_roi_min")) {
			roi_min = json_object_get_int(val);
		} else if (!strcmp(key, "auto_sat_limit")) {
			sat_limit = json_object_get_double(val);
		} else if (!strcmp(key, "auto_peak_target")) {
			peak_target = json_object_get_int(val);
		} else if (!strcmp(key, "bandwidth")) {
			bandwidth = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "high_speed")) {
			high_speed = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "die_on_stall")) {
			data->die_on_stall = json_object_get_int(val) ? 1 : 0;
		} else if (!strcmp(key, "stall_timeout_ms")) {
			data->stall_timeout_ms = (int)json_object_get_int64(val);
			if (data->stall_timeout_ms < 0) {
				log_error("stall_timeout_ms must be >= 0 "
					"(0 = auto)");
				return -1;
			}
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	// effective wait until the first frame-rate estimate refines it (auto
	// mode tightens to 3 measured frame periods after ~0.5 s). The very
	// first frame after ASIStartVideoCapture routinely takes >200 ms
	// (measured 2026-07-17), so start generous — same window the restart
	// path uses — or every run opens with a spurious capture restart.
	data->cur_timeout_ms = data->stall_timeout_ms > 0
		? data->stall_timeout_ms : 500;

	if (width <= 0 || height <= 0) {
		log_error("width and height must be positive (got %dx%d)",
			width, height);
		return -1;
	}
	if (width % 8 != 0) {
		log_error("width must be a multiple of 8 (got %d)", width);
		return -1;
	}
	if (probe_frames < 1 || probe_frames > 256) {
		log_error("auto_probe_frames must be in [1,256] (got %d)",
			probe_frames);
		return -1;
	}
	if (!(roi_sigma > 0.0) || !isfinite(roi_sigma)) {
		log_error("auto_roi_sigma must be positive (got %G)", roi_sigma);
		return -1;
	}
	if (roi_min < 8) {
		log_error("auto_roi_min must be >= 8 (got %d)", roi_min);
		return -1;
	}
	if (!(sat_limit >= 0.0) || sat_limit > 1.0 || peak_target < 16
			|| peak_target > 255) {
		log_error("auto gain needs 0 <= auto_sat_limit <= 1 and "
			"16 <= auto_peak_target <= 255 (got %G, %d)",
			sat_limit, peak_target);
		return -1;
	}

	// find camera by name substring
	int n_cams = ASIGetNumOfConnectedCameras();
	if (n_cams <= 0) {
		log_error("No ASI cameras found");
		return -1;
	}
	log_info("Found %d ASI camera(s)", n_cams);

	long sensor_w = 0, sensor_h = 0;
	for (int i = 0; i < n_cams; i++) {
		ASI_CAMERA_INFO info;
		ASIGetCameraProperty(&info, i);
		log_info("  [%d] %s (ID %d, %ldx%ld)",
			i, info.Name, info.CameraID, info.MaxWidth, info.MaxHeight
		);
		if (data->camera_id < 0 && strstr(info.Name, camera_name)) {
			data->camera_id = info.CameraID;
			sensor_w = info.MaxWidth;
			sensor_h = info.MaxHeight;
		}
	}
	if (data->camera_id < 0) {
		log_error("No camera matching \"%s\" found", camera_name);
		return -1;
	}

	ASI_ERROR_CODE err;
	err = ASIOpenCamera(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIOpenCamera failed (%d)", err);
		return -1;
	}
	data->opened = 1;
	err = ASIInitCamera(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIInitCamera failed (%d)", err);
		return -1;
	}

	// Everything except gain is settled before any probe: exposure,
	// bandwidth and readout mode all move the numbers a probe measures, so
	// they have to be the operating ones while it runs.
	err = ASISetControlValue(data->camera_id, ASI_BANDWIDTHOVERLOAD,
		bandwidth, ASI_FALSE);
	if (err == ASI_SUCCESS)
		err = ASISetControlValue(data->camera_id, ASI_EXPOSURE,
			exposure, ASI_FALSE);
	if (err == ASI_SUCCESS)
		err = ASISetControlValue(data->camera_id, ASI_HIGH_SPEED_MODE,
			high_speed, ASI_FALSE);
	if (err != ASI_SUCCESS) {
		log_error("ASISetControlValue failed (%d)", err);
		return -1;
	}

	struct asi_auto_cfg ac = {
		.width = auto_width, .height = auto_height,
		.start_x = auto_start_x, .start_y = auto_start_y,
		.gain = auto_gain,
		.probe_frames = probe_frames, .roi_sigma = roi_sigma,
		.roi_min = roi_min, .sat_limit = sat_limit,
		.peak_target = peak_target,
	};

	// gain range, needed for both the probe ladder and the auto search
	long gain_min = 0, gain_max = 0, gain_def = gain;
	int have_gain_caps = 0;
	int n_ctrl = 0;
	if (ASIGetNumOfControls(data->camera_id, &n_ctrl) == ASI_SUCCESS) {
		for (int i = 0; i < n_ctrl; i++) {
			ASI_CONTROL_CAPS caps;
			if (ASIGetControlCaps(data->camera_id, i, &caps)
					!= ASI_SUCCESS) continue;
			if (caps.ControlType != ASI_GAIN) continue;
			gain_min = caps.MinValue;
			gain_max = caps.MaxValue;
			gain_def = caps.DefaultValue;
			have_gain_caps = 1;
			break;
		}
	}
	if (auto_gain && !have_gain_caps) {
		log_error("asi_source: gain is \"auto\" but the camera did not "
			"report a gain range");
		return -1;
	}

	if (auto_width || auto_height || auto_start_x || auto_start_y) {
		// With an explicit gain there is exactly one right setting to
		// probe at. With "auto" there is none yet, so try dark first
		// and open up only if nothing is there.
		long probe_gains[3];
		int n_probe = 0;
		if (auto_gain) {
			probe_gains[n_probe++] = gain_min;
			if (gain_def > gain_min) probe_gains[n_probe++] = gain_def;
			if (gain_max > gain_def) probe_gains[n_probe++] = gain_max;
		} else {
			probe_gains[n_probe++] = gain;
		}
		if (asi_probe_roi(data->camera_id, &ac, sensor_w, sensor_h,
				probe_gains, n_probe, &width, &height,
				&start_x, &start_y))
			return -1;
		if (width % 8 != 0 || height % 2 != 0) {
			log_error("asi_source: auto ROI produced an unusable "
				"%dx%d", width, height);
			return -1;
		}
	}

	// center ROI on the sensor if the start position was neither given nor
	// resolved by a probe
	if (start_x < 0)
		start_x = (int)((sensor_w - width)  / 2) & ~7;  // align to 8px
	if (start_y < 0)
		start_y = (int)((sensor_h - height) / 2) & ~1;  // align to 2px
	if (start_x < 0 || start_y < 0
			|| (long)start_x + width > sensor_w
			|| (long)start_y + height > sensor_h) {
		log_error("ROI %dx%d @ (%d,%d) is outside sensor %ldx%ld",
			width, height, start_x, start_y, sensor_w, sensor_h);
		return -1;
	}

	err = ASISetROIFormat(data->camera_id, width, height, 1, ASI_IMG_RAW8);
	if (err != ASI_SUCCESS) {
		log_error("ASISetROIFormat failed (%d) for %dx%d RAW8", err, width, height);
		return -1;
	}
	err = ASISetStartPos(data->camera_id, start_x, start_y);
	if (err != ASI_SUCCESS) {
		log_error("ASISetStartPos failed (%d) for start=(%d,%d)",
			err, start_x, start_y
		);
		return -1;
	}
	err = ASISetControlValue(data->camera_id, ASI_GAIN,
		auto_gain ? gain_min : gain, ASI_FALSE);
	if (err != ASI_SUCCESS) {
		log_error("ASISetControlValue failed for gain (%d)", err);
		return -1;
	}

	err = ASIStartVideoCapture(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIStartVideoCapture failed (%d)", err);
		return -1;
	}
	data->capturing = 1;

	if (auto_gain) {
		long n_px = (long)height * width;
		unsigned char *buf = xmalloc((size_t)n_px);
		int ok = -1;
		gain = asi_auto_gain(data->camera_id, buf, n_px, &ac,
			gain_min, gain_max, &ok);
		xfree(buf);
		if (ok < 0) {
			log_error("asi_source: the auto-gain search could not "
				"be completed; refusing to start on an "
				"unmeasured gain");
			return -1;
		}
		if (!ok)
			log_warn("asi_source: auto gain: even the minimum gain "
				"%ld clips (over %G%% of pixels saturated or "
				"p99.9 over %d). Using it anyway; the centroid "
				"is being computed on a flattened core until "
				"the source is dimmed or the exposure is cut.",
				gain_min, 1e2 * sat_limit, peak_target);
		err = ASISetControlValue(data->camera_id, ASI_GAIN, gain,
			ASI_FALSE);
		if (err != ASI_SUCCESS) {
			log_error("asi_source: could not set the chosen gain "
				"%ld (%d)", gain, err);
			return -1;
		}
		log_info("asi_source: auto gain -> %ld (of %ld..%ld)",
			gain, gain_min, gain_max);
	}

	log_info("Camera %d ready: %dx%d RAW8 @ (%d,%d), exp=%ldµs gain=%ld bw=%ld%%",
		data->camera_id, width, height, start_x, start_y, exposure, gain, bandwidth
	);
	// Downstream devices work in centroid units normalized over the frame,
	// and with an auto ROI the frame size is not in anyone's config. Publish
	// it (see libaylp/timing.h).
	aylp_frame_geometry_publish((size_t)height, (size_t)width);
	aylp_timing_publish_exposure(1e-6 * (double)exposure);

	// allocate the frame buffer (gsl_matrix_uchar is contiguous when tda==size2)
	data->width    = width;
	data->height   = height;
	data->buf_size = (long)height * width;
	data->matrix   = xmalloc_type(gsl_matrix_uchar, (size_t)height, (size_t)width);
	if (data->matrix->tda != (size_t)width) {
		log_error("ASI frame buffer is unexpectedly non-contiguous "
			"(tda=%zu width=%d)", data->matrix->tda, width);
		return -1;
	}

	// start the first latency-diagnostic window
	clock_gettime(CLOCK_MONOTONIC, &data->diag_t0);

	self->type_in   = AYLP_T_ANY;
	self->units_in  = AYLP_U_ANY;
	self->type_out  = AYLP_T_MATRIX_UCHAR;
	self->units_out = AYLP_U_COUNTS;
	return 0;
}
