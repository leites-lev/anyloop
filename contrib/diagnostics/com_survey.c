// com_survey -- validate and tune anyloop:fit_com from a real open-loop
// capture, and report the beam/camera numbers the rest of the tuning depends
// on.
//
// Capture with contrib/calibration-scripts/configurations/com_survey.json (one minute, no tracker, no loop, no
// DAC), then run this over the resulting .aylp file. The fit parameters are
// derived from that capture and accepted only when the ROI, saturation and
// live-frame gates all pass.
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>

#include "anyloop.h"
#include "fit_com.h"

#define HDR 40
#define MAXLAG 1024

static double now_s(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + 1e-9*t.tv_nsec;
}
static int cmpd(const void *a, const void *b)
{ double x = *(const double*)a, y = *(const double*)b; return x<y?-1:(x>y); }

struct series { double *v; size_t n, cap; };
static void ser_add(struct series *s, double x)
{
	if (s->n == s->cap) {
		s->cap = s->cap ? s->cap*2 : 4096;
		s->v = realloc(s->v, s->cap*sizeof(double));
	}
	s->v[s->n++] = x;
}
static double ser_mean(const struct series *s)
{
	double m = 0; for (size_t i = 0; i < s->n; i++) m += s->v[i];
	return s->n ? m/s->n : 0.0;
}
static double ser_rms_about_mean(const struct series *s)
{
	double m = ser_mean(s), q = 0;
	for (size_t i = 0; i < s->n; i++) q += (s->v[i]-m)*(s->v[i]-m);
	return s->n ? sqrt(q/s->n) : 0.0;
}
// rms of the frame-to-frame difference / sqrt(2): the high-frequency content,
// which is what a steering loop is actually being asked to reject, and where
// fabricated motion shows up most clearly.
static double ser_hf(const struct series *s)
{
	if (s->n < 2) return 0.0;
	double q = 0;
	for (size_t i = 1; i < s->n; i++) {
		double d = s->v[i]-s->v[i-1];
		q += d*d;
	}
	return sqrt(q/(2.0*(s->n-1)));
}

int main(int argc, char **argv)
{
	const char *path = 0;
	const char *json_path = 0;
	const char *trace_path = 0;
	double fs = 3788.0, row_time = 0.0;
	double exposure_us = 0.0, max_exposure_us = 0.0, camera_gain = 0.0;
	double pwm_hz = 0.0, pwm_duty = 1.0;
	size_t max_frames = 0;
	size_t skip_frames = 0;
	size_t o_fitwin = 0;
	static const char *usage =
		"usage: %s CAPTURE.aylp [--fs HZ] [--row-time S] "
		"[--skip-frames N] [--max-frames N] [--json RESULT.json] [--trace TRACE.bin]\n"
		"    [--exposure-us US] [--max-exposure-us US] [--gain G]\n"
		"    [--pwm-hz HZ] [--pwm-duty FRACTION]\n"
		"    --fit-window PX      fit_com window size\n";
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--fs") && i+1 < argc) fs = atof(argv[++i]);
		else if (!strcmp(argv[i], "--row-time") && i+1 < argc)
			row_time = atof(argv[++i]);
		else if (!strcmp(argv[i], "--max-frames") && i+1 < argc)
			max_frames = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--skip-frames") && i+1 < argc)
			skip_frames = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--json") && i+1 < argc)
			json_path = argv[++i];
		else if (!strcmp(argv[i], "--trace") && i+1 < argc)
			trace_path = argv[++i];
		else if (!strcmp(argv[i], "--exposure-us") && i+1 < argc)
			exposure_us = atof(argv[++i]);
		else if (!strcmp(argv[i], "--max-exposure-us") && i+1 < argc)
			max_exposure_us = atof(argv[++i]);
		else if (!strcmp(argv[i], "--gain") && i+1 < argc)
			camera_gain = atof(argv[++i]);
		else if (!strcmp(argv[i], "--pwm-hz") && i+1 < argc)
			pwm_hz = atof(argv[++i]);
		else if (!strcmp(argv[i], "--pwm-duty") && i+1 < argc)
			pwm_duty = atof(argv[++i]);
		else if (!strcmp(argv[i], "--fit-window") && i+1 < argc)
			o_fitwin = strtoull(argv[++i], 0, 0);
		else if (argv[i][0] != '-') path = argv[i];
		else { fprintf(stderr, usage, argv[0]); return 2; }
	}
	if (!path) { fprintf(stderr, usage, argv[0]); return 2; }
	if (pwm_hz < 0.0 || pwm_duty <= 0.0 || pwm_duty > 1.0) {
		fprintf(stderr, "invalid PWM frequency/duty\n"); return 2;
	}
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); return 1; }

	unsigned char hdr[HDR];
	if (fread(hdr, 1, HDR, f) != HDR) {
		fprintf(stderr, "%s: too short to hold one frame header\n", path);
		return 1;
	}
	uint64_t H, W;
	memcpy(&H, hdr+8, 8);
	memcpy(&W, hdr+16, 8);
	uint8_t type = hdr[6];
	if (type != AYLP_T_MATRIX_UCHAR) {
		fprintf(stderr, "%s: first frame is type %u, expected "
			"MATRIX_UCHAR (%u). Capture with com_survey.json.\n",
			path, type, (unsigned)AYLP_T_MATRIX_UCHAR);
		return 1;
	}
	if (H < 8 || W < 8 || H > 4096 || W > 4096) {
		fprintf(stderr, "%s: implausible frame size %lux%lu\n", path,
			(unsigned long)H, (unsigned long)W);
		return 1;
	}
	fseek(f, 0, SEEK_END);
	long long bytes = ftell(f);
	fseek(f, 0, SEEK_SET);
	size_t fsz = HDR + H*W;
	size_t total_frames = (size_t)(bytes / (long long)fsz);
	if (skip_frames >= total_frames) {
		fprintf(stderr, "%s: skip exceeds capture length\n", path); return 1;
	}
	size_t nframes = total_frames - skip_frames;
	if (max_frames && nframes > max_frames) nframes = max_frames;
	if (nframes < 100) {
		fprintf(stderr, "%s: only %zu frames; need at least 100\n",
			path, nframes);
		return 1;
	}

	printf("=================================================================\n");
	printf(" com_survey: %s\n", path);
	printf(" %zu frames of %lux%lu at fs %.0f Hz  (%.1f s of data)\n",
		nframes, (unsigned long)H, (unsigned long)W, fs, nframes/fs);
	printf("=================================================================\n");

	gsl_matrix_uchar *img = gsl_matrix_uchar_alloc(H, W);
	unsigned char *raw = malloc(fsz);

	// ---------------- pass 1: camera and beam characterisation -------
	double bg_sum = 0, bg_sq = 0; size_t bg_n = 0;
	size_t nsat = 0, npix_tot = 0;
	size_t sig_n = 0;
	double flux_sum = 0;
	double cy_sum = 0, cx_sum = 0;
	size_t sample_every = nframes/2000 ? nframes/2000 : 1;
	size_t sample_cap = nframes/sample_every + 2, namp = 0, nsigma = 0;
	double *amp_samples = malloc(sample_cap*sizeof(double));
	double *flux_samples = malloc(sample_cap*sizeof(double));
	double *sigma_samples = malloc(sample_cap*sizeof(double));
	// running background sigma, so the moment cut below has something to
	// use on the very first sampled frame
	double bg_sd = 0.5;
	fseek(f, (long long)skip_frames*fsz, SEEK_SET);
	for (size_t k = 0; k < nframes; k++) {
		if (fread(raw, 1, fsz, f) != fsz) { nframes = k; break; }
		if (k % sample_every) continue;
		const unsigned char *px = raw + HDR;
		// background from the frame border, peak from anywhere
		double bs = 0; size_t bn = 0; unsigned peak = 0;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				unsigned char v = px[i*W+j];
				if (i==0 || j==0 || i==H-1 || j==W-1) {
					bs += v; bn++;
					bg_sum += v; bg_sq += (double)v*v; bg_n++;
				}
				if (v > peak) peak = v;
				if (v >= 255) nsat++;
				npix_tot++;
			}
		double bg = bn ? bs/bn : 0.0;
		if (bg_n > 64) {
			double m = bg_sum/bg_n;
			double v = bg_sq/bg_n - m*m;
			bg_sd = v > 0.25 ? sqrt(v) : 0.5;
		}
		amp_samples[namp++] = peak - bg;
		// flux-weighted centroid and radial second moment above bg
		double sw = 0, sy = 0, sx = 0;
		double cut = 5.0*(bg_sd > 0.5 ? bg_sd : 0.5);
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				double v = (double)px[i*W+j] - bg;
				if (v <= cut) continue;
				sw += v; sy += v*i; sx += v*j;
			}
		if (sw <= 0) continue;
		flux_samples[nsigma] = sw;
		double cy = sy/sw, cx = sx/sw, sr2 = 0;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				double v = (double)px[i*W+j] - bg;
				if (v <= cut) continue;
				double dy = i-cy, dx = j-cx;
				sr2 += v*(dy*dy+dx*dx);
			}
		double frame_sigma = sqrt(sr2/sw/2.0);
		sig_n++;
		sigma_samples[nsigma++] = frame_sigma;
		flux_sum += sw; cy_sum += cy; cx_sum += cx;
	}
	double bg_mean = bg_n ? bg_sum/bg_n : 0;
	bg_sd = bg_n ? sqrt(bg_sq/bg_n - bg_mean*bg_mean) : 0;
	qsort(amp_samples, namp, sizeof(double), cmpd);
	qsort(sigma_samples, nsigma, sizeof(double), cmpd);
	qsort(flux_samples, nsigma, sizeof(double), cmpd);
	double amp_p10 = namp ? amp_samples[(size_t)(0.10*(namp-1))] : 0;
	double amp_p50 = namp ? amp_samples[namp/2] : 0;
	double amp_p90 = namp ? amp_samples[(size_t)(0.90*(namp-1))] : 0;
	double amp_p99 = namp ? amp_samples[(size_t)(0.99*(namp-1))] : 0;
	double moment_cut = fmax(2.0, ceil(5.0*bg_sd));
	double min_amplitude;
	double pwm_period = 0.0, pwm_dark_flux = 100.0, pwm_bright_flux = 1000.0;
	size_t pwm_full_start = 0, pwm_end_margin = 0;
	bool integrate_cycle = pwm_hz > 0.0
		&& exposure_us*1e-6 >= 0.95/pwm_hz;
	if (pwm_hz > 0.0 && !integrate_cycle && nsigma > 10) {
		double off = 1.0-pwm_duty;
		size_t idark = (size_t)(fmin(0.95, fmax(0.01, 0.8*off))*(nsigma-1));
		size_t ibright = (size_t)(fmin(0.99, off+0.20*pwm_duty)*(nsigma-1));
		pwm_dark_flux = 1.10*flux_samples[idark];
		pwm_bright_flux = flux_samples[ibright];
		if (pwm_bright_flux <= pwm_dark_flux)
			pwm_bright_flux = 1.5*pwm_dark_flux;
		pwm_period = fs/pwm_hz;
		double dark_frames = off*pwm_period;
		double exposure_frames = exposure_us*1e-6*fs;
		pwm_full_start = (size_t)ceil(dark_frames)+1;
		pwm_end_margin = (size_t)ceil(exposure_frames)+1;
		size_t ibeam = (size_t)(fmin(0.99, off+0.10*pwm_duty)*(namp-1));
		min_amplitude = fmax(5.0*bg_sd, 0.25*amp_samples[ibeam]);
	} else {
		min_amplitude = fmax(5.0*bg_sd, 0.25*amp_p10);
	}
	if (min_amplitude < 2.0) min_amplitude = 2.0;
	// The upper-decile amplitude describes illuminated frames and is not
	// pulled down by scheduled or sprinkled dark frames.
	double amp_mean = amp_p90;
	// Median width rejects both dark-frame noise and rare clipped/smeared fits.
	double sigma = nsigma ? sigma_samples[nsigma/2] : 2.0;
	double flux_mean = sig_n ? flux_sum/sig_n : 0;
	double cy_mean = sig_n ? cy_sum/sig_n : H/2.0;
	double cx_mean = sig_n ? cx_sum/sig_n : W/2.0;
	double sat_frac = npix_tot ? (double)nsat/npix_tot : 0;

	printf("\n--- camera and beam -------------------------------------------\n");
	printf("  background        %6.2f counts, sigma %.2f\n", bg_mean, bg_sd);
	printf("  beam amplitude    p50 %6.1f  p90 %6.1f  p99 %6.1f counts"
	       " above background\n", amp_p50, amp_p90, amp_p99);
	printf("  beam sigma        %6.2f px   (FWHM %.2f px)\n",
		sigma, 2.3548*sigma);
	printf("  mean position     (%.2f, %.2f) px\n", cy_mean, cx_mean);
	printf("  total flux        %.0f counts\n", flux_mean);
	printf("  moment cut        %.1f counts above frame background\n", moment_cut);
	printf("  minimum amplitude %.1f counts above frame background\n", min_amplitude);
	if (pwm_period > 0.0)
		printf("  PWM %.3f frames; dark/bright flux %.0f / %.0f; full %zu..period-%zu\n",
			pwm_period, pwm_dark_flux, pwm_bright_flux,
			pwm_full_start, pwm_end_margin);
	else if (integrate_cycle)
		printf("  PWM phase filter disabled: exposure integrates a complete cycle\n");
	printf("  saturated pixels  %.4f%% %s\n", 100*sat_frac,
		sat_frac > 1e-3 ? "  <-- TOO MANY, reduce exposure or gain"
		: "(ok)");

	int sugg_thresh = (int)ceil(bg_mean + 5*bg_sd);
	if (sugg_thresh < 0) sugg_thresh = 0;
	if (sugg_thresh > 255) sugg_thresh = 255;
	size_t sugg_win = (size_t)(2*ceil(4*sigma)+1);
	if (sugg_win > H) sugg_win = H;
	if (sugg_win > W) sugg_win = W;
	double half_win = 0.5*(sugg_win - 1);
	bool beam_clipped = cy_mean - half_win < 0 || cy_mean + half_win >= H ||
		cx_mean - half_win < 0 || cx_mean + half_win >= W;
	printf("\n  suggested threshold   %d      (background + 5 sigma)\n",
		sugg_thresh);
	printf("  suggested sigma_init  %.2f\n", sigma);
	printf("  suggested window      %zu x %zu   (+/-4 sigma)\n",
		sugg_win, sugg_win);

	double suggested_exposure = exposure_us;
	double suggested_gain = camera_gain;
	if (exposure_us > 0.0 && amp_p90 > 1.0) {
		const double target = 180.0;
		double scale = target/amp_p90;
		// The upper tail protects against clipping even when p90 itself is
		// in range. This matters for a source whose bright phase varies.
		if (sat_frac > 1e-3 && amp_p99 > 230.0 && 220.0/amp_p99 < scale)
			scale = 220.0/amp_p99;
		double cap = max_exposure_us > 0.0 ? max_exposure_us : exposure_us;
		if (scale >= 1.0) {
			// Add photons with exposure first. Gain is allowed to rise only
			// after exposure reaches the no-frame-rate-loss ceiling.
			suggested_exposure = exposure_us*scale;
			if (suggested_exposure > cap) suggested_exposure = cap;
			double remaining = scale*exposure_us/suggested_exposure;
			if (remaining > 1.0)
				suggested_gain += 200.0*log10(remaining);
		} else if (camera_gain > 0.0) {
			// Once gain has been added, remove it before shortening exposure.
			suggested_gain += 200.0*log10(scale);
		} else {
			suggested_exposure = exposure_us*scale;
		}
		if (suggested_exposure > cap) suggested_exposure = cap;
		if (suggested_exposure < 20.0) suggested_exposure = 20.0;
		if (suggested_gain < 0.0) suggested_gain = 0.0;
		if (suggested_gain > 200.0) suggested_gain = 200.0;
		// Avoid reacting to harmless measurement scatter near the target.
		if (amp_p90 >= 140.0 && amp_p90 <= 220.0 && sat_frac <= 1e-3) {
			suggested_exposure = exposure_us;
			suggested_gain = camera_gain;
		}
		suggested_exposure = 10.0*lround(suggested_exposure/10.0);
		suggested_gain = 5.0*lround(suggested_gain/5.0);
		printf("  camera (bright p90)   exposure %.0f -> %.0f us, gain %.0f -> %.0f\n",
			exposure_us, suggested_exposure, camera_gain, suggested_gain);
		printf("                         target 180 counts; exposure capped at %.0f us"
		       " to preserve frame rate\n", cap);
	}

	// ---------------- pass 2: validate fit_com over every frame -------
	char fitcfg[700];
	// Validation and actuator calibration must tolerate commanded travel.
	// The stationary 4-sigma footprint is only a clearance diagnostic; use
	// the entire ROI unless the caller explicitly requests another window.
	sugg_win = o_fitwin ? o_fitwin : (H < W ? H : W);
	snprintf(fitcfg, sizeof fitcfg,
		"{\"window_height\":%zu,\"window_width\":%zu,\"sigma_init\":%.3f,"
		"\"sigma_min\":%.3f,\"sigma_max\":%.3f,\"min_amplitude\":%.2f,"
		"\"max_step\":8.0,\"max_iter\":12,\"max_us\":300,"
		"\"robust_k\":2.5,\"robust_iter\":1,\"fit_slope\":false,"
		"\"moment_output\":true,\"moment_cut\":%.3f,"
		"\"pwm_period_frames\":%.6f,\"pwm_dark_flux\":%.3f,"
		"\"pwm_bright_flux\":%.3f,\"pwm_full_start\":%zu,"
		"\"pwm_full_end_margin\":%zu,"
		"\"row_time\":%g,\"reacquire_after\":300}",
		sugg_win, sugg_win, sigma, sigma*0.3 > 0.3 ? sigma*0.3 : 0.3,
		sigma*4, min_amplitude, moment_cut, pwm_period, pwm_dark_flux,
		pwm_bright_flux, pwm_full_start, pwm_end_margin, row_time);
	printf("\n--- tracker configuration used -------------------------------\n");
	printf("  fit_com  window %zux%zu  sigma_init %.2f\n",
		sugg_win, sugg_win, sigma);

	struct aylp_device fit_dev = {0};
	json_object *fp = json_tokener_parse(fitcfg);
	fit_dev.params = fp;
	if (fit_com_init(&fit_dev)) { fprintf(stderr, "fit_com init failed\n"); return 1; }
	struct aylp_fit_com_data *fd = fit_dev.device_data;
	FILE *trace = trace_path ? fopen(trace_path, "wb") : 0;
	if (trace_path && !trace) { perror(trace_path); return 1; }

	struct series fy = {0}, fx = {0}, rres = {0};
	struct series raw_y = {0}, raw_x = {0}, fit_raw_dy = {0},
		fit_raw_dx = {0}, raw_flux = {0};
	double *track_y = malloc(nframes*sizeof(double));
	double *track_x = malloc(nframes*sizeof(double));
	unsigned char *track_kind = calloc(nframes, 1); /* 1 complete, 2 inferred */
	double *ftime = malloc(nframes*sizeof(double));
	size_t frej = 0, flost = 0;
	double *step_samples = malloc(nframes*sizeof(double));
	size_t nstep = 0, reject_streak = 0, max_reject_streak = 0;
	double prev_live_y = 0.0, prev_live_x = 0.0;
	bool have_prev_live = false;
	// fit_com sizes its window on the first frame it sees, so the residual
	// map's extent is not known until then -- allocate the ring lazily.
	size_t nlag = 0;
	double *ring = 0;
	int lags[] = {1, 10, 100, 1000};
	double lag_num[4] = {0}, lag_da[4] = {0}, lag_db[4] = {0};
	size_t lag_n[4] = {0};

	fseek(f, (long long)skip_frames*fsz, SEEK_SET);
	struct aylp_state st = {0};
	size_t used = 0;
	for (size_t k = 0; k < nframes; k++) {
		if (fread(raw, 1, fsz, f) != fsz) break;
		memcpy(img->data, raw+HDR, H*W);

		st.matrix_uchar = img; st.header.status = 0;
		double t0 = now_s();
		fit_com_proc(&fit_dev, &st);
		ftime[used] = 1e6*(now_s()-t0);
		int fr = (st.header.status & AYLP_FRAME_REJECTED) != 0;
		if (fr) {
			frej++;
			reject_streak++;
			if (reject_streak > max_reject_streak)
				max_reject_streak = reject_streak;
		} else {
			reject_streak = 0;
		}
		if (st.header.status & AYLP_BEAM_LOST) flost++;

		used++;

		if (k < 200) continue;	// let both settle
		double fcy = (fd->last_y+1)/2*(H-1);
		double fcx = (fd->last_x+1)/2*(W-1);
		if (!fr && !fd->inferred_last) {
			if (have_prev_live)
				step_samples[nstep++] = hypot(fcy-prev_live_y, fcx-prev_live_x);
			prev_live_y = fcy; prev_live_x = fcx; have_prev_live = true;
		}
		ser_add(&fy, fcy);
		ser_add(&fx, fcx);
		track_y[k] = fcy; track_x[k] = fcx;
		if (!fr) track_kind[k] = fd->inferred_last ? 2 : 1;
		if (trace) {
			unsigned char flags = (fr ? 1 : 0) | (fd->inferred_last ? 2 : 0)
				| ((st.header.status & AYLP_BEAM_LOST) ? 4 : 0);
			fwrite(&fcy, sizeof fcy, 1, trace);
			fwrite(&fcx, sizeof fcx, 1, trace);
			fwrite(&flags, sizeof flags, 1, trace);
			continue;
		}
		if (!fr) ser_add(&rres, fd->last_rms);

		// Independent truth estimate from the raw pixels.  This does not use
		// fit_com's centre, window, amplitude, residual, or validity flag.
		// A border mean supplies the per-frame background and a fixed 2-DN
		// excess cut suppresses read noise without changing with brightness.
		double border = 0.0; size_t border_n = 0;
		const unsigned char *px = raw + HDR;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++)
				if (i == 0 || j == 0 || i == H-1 || j == W-1) {
					border += px[i*W+j]; border_n++;
				}
		double frame_bg = border_n ? border/border_n : 0.0;
		double rw = 0.0, ry = 0.0, rx = 0.0;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				double v = (double)px[i*W+j] - frame_bg;
				if (v <= 2.0) continue;
				rw += v; ry += v*i; rx += v*j;
			}
		if (!fr && !fd->inferred_last && rw > 100.0) {
			double rcy = ry/rw, rcx = rx/rw;
			ser_add(&raw_y, rcy); ser_add(&raw_x, rcx);
			ser_add(&fit_raw_dy, fcy-rcy);
			ser_add(&fit_raw_dx, fcx-rcx);
			ser_add(&raw_flux, rw);
		}

		// residual-map autocorrelation at several lags: does the part
		// of the beam the gaussian cannot explain stay put, or boil?
		// The device only keeps residuals for the core box it actually
		// iterates on, and re-plans that box every frame; a frame whose
		// box came out a different size is not comparable to the ring's
		// and is skipped rather than folded in.
		if (!ring) {
			nlag = fd->core_h*fd->core_stride;
			if (!nlag) continue;
			ring = calloc(MAXLAG*nlag, sizeof(double));
		}
		if (fd->core_h*fd->core_stride != nlag) continue;
		double *slot = ring + (k % MAXLAG)*nlag;
		memcpy(slot, fd->resid, nlag*sizeof(double));
		for (int L = 0; L < 4; L++) {
			size_t lag = (size_t)lags[L];
			if (k < 200 + lag || lag >= MAXLAG) continue;
			double *old = ring + ((k-lag) % MAXLAG)*nlag;
			double sa=0, sb=0, saa=0, sbb=0, sab=0;
			for (size_t i = 0; i < nlag; i++) {
				sa += slot[i]; sb += old[i];
				saa += slot[i]*slot[i]; sbb += old[i]*old[i];
				sab += slot[i]*old[i];
			}
			double na = nlag;
			double va = saa/na - (sa/na)*(sa/na);
			double vb = sbb/na - (sb/na)*(sb/na);
			double cv = sab/na - (sa/na)*(sb/na);
			if (va > 1e-12 && vb > 1e-12) {
				lag_num[L] += cv/sqrt(va*vb);
				lag_n[L]++;
			}
			lag_da[L] += va; lag_db[L] += vb;
		}
	}
	fclose(f);
	if (trace) {
		fclose(trace);
		fit_com_fini(&fit_dev);
		json_object_put(fp);
		free(fy.v); free(fx.v); free(rres.v);
		free(ftime); free(step_samples); free(ring); free(raw);
		free(amp_samples); free(flux_samples); free(sigma_samples);
		free(track_y); free(track_x); free(track_kind);
		gsl_matrix_uchar_free(img);
		printf("wrote %zu tracker records to %s\n", used > 200 ? used-200 : 0,
			trace_path);
		return 0;
	}

	// ---------------- report -----------------------------------------
	double resid = ser_mean(&rres);
	double rel_resid = amp_mean > 0 ? resid/amp_mean : 0;
	double corr[4];
	for (int L = 0; L < 4; L++)
		corr[L] = lag_n[L] ? lag_num[L]/lag_n[L] : 0.0;
	// normalise by lag 1 so per-frame photon noise, which decorrelates
	// instantly and is not speckle, does not masquerade as boiling
	double persist = corr[0] > 0.05 ? corr[2]/corr[0] : 0.0;
	if (persist > 1.0) persist = 1.0;
	if (persist < 0.0) persist = 0.0;

	printf("\n--- gaussian model fit (fit_com) ------------------------------\n");
	printf("  residual rms      %6.3f counts\n", resid);
	printf("  relative residual %6.4f   (residual / amplitude)\n", rel_resid);
	printf("      < 0.010  beam is gaussian to within noise\n");
	printf("      > 0.035  substantial departure from gaussian\n");

	printf("\n--- is the departure frozen or boiling? -----------------------\n");
	printf("  residual-map correlation:  lag1 %.3f  lag10 %.3f  "
		"lag100 %.3f  lag1000 %.3f\n", corr[0], corr[1], corr[2], corr[3]);
	printf("  persistence (lag100/lag1)  %.3f   "
		"(1.0 = frozen, 0.0 = boiling)\n", persist);
	if (corr[2] > 0.01 && corr[3] > 0.01 && corr[2] > corr[3]) {
		double tau = 100.0*log(corr[0]/corr[3])/log(corr[0]/corr[2]);
		(void)tau;
	}

	qsort(ftime, used, sizeof(double), cmpd);
	qsort(step_samples, nstep, sizeof(double), cmpd);
	double step_p999 = nstep
		? step_samples[(size_t)(0.999*(nstep-1))] : 0.0;
	double suggested_max_step = fmax(2.0, 1.5*step_p999);
	if (suggested_max_step > 0.25*(H < W ? H : W))
		suggested_max_step = 0.25*(H < W ? H : W);
	size_t normal_pwm_gap = pwm_period > 0.0
		? (size_t)ceil((1.0-pwm_duty)*pwm_period)+pwm_full_start
			+pwm_end_margin : 0;
	size_t suggested_reacquire = max_reject_streak
		+ (max_reject_streak/2 > 3 ? max_reject_streak/2 : 3);
	if (suggested_reacquire < normal_pwm_gap+3)
		suggested_reacquire = normal_pwm_gap+3;
	if (suggested_reacquire < 10) suggested_reacquire = 10;
	json_object_object_add(fp, "max_step",
		json_object_new_double(suggested_max_step));
	json_object_object_add(fp, "reacquire_after",
		json_object_new_int64((int64_t)suggested_reacquire));
	double period_us = 1e6/fs;
	printf("\n--- cost per frame (this machine) -----------------------------\n");
	printf("  fit_com  p50 %6.1f us  p99 %6.1f us  max %6.1f us   "
		"(%.0f%% of the %.0f us period at p99)\n",
		ftime[used/2], ftime[(size_t)(used*0.99)], ftime[used-1],
		100*ftime[(size_t)(used*0.99)]/period_us, period_us);
	printf("  motion p99.9 %.3f px/frame -> max_step %.3f px\n",
		step_p999, suggested_max_step);
	printf("  longest rejected streak %zu frames -> reacquire_after %zu frames\n",
		max_reject_streak, suggested_reacquire);

	printf("\n--- fit_com tracking result -----------------------------------\n");
	double fit_hf = ser_hf(&fy);
	printf("  %-9s %-12s %-12s %-11s %-9s %s\n", "", "rms y (px)",
		"rms x (px)", "hf y (px)", "rejected", "beam_lost");
	printf("  %-9s %-12.4f %-12.4f %-11.4f %-9.2f%% %zu\n", "fit_com",
		ser_rms_about_mean(&fy), ser_rms_about_mean(&fx), fit_hf,
		100.0*frej/used, flost);
	printf("  hf = rms of the frame-to-frame difference / sqrt(2): the\n"
		"       high-frequency content, which is what the loop must\n"
		"       reject and where fabricated motion shows up first.\n");

	double dy_mean = ser_mean(&fit_raw_dy), dx_mean = ser_mean(&fit_raw_dx);
	double dy_rms = ser_rms_about_mean(&fit_raw_dy);
	double dx_rms = ser_rms_about_mean(&fit_raw_dx);
	double max_abs_dy = 0.0, max_abs_dx = 0.0;
	double fm = ser_mean(&raw_flux), fv = 0.0, fdy = 0.0, fdx = 0.0;
	for (size_t i = 0; i < fit_raw_dy.n; i++) {
		double ady = fabs(fit_raw_dy.v[i] - dy_mean);
		double adx = fabs(fit_raw_dx.v[i] - dx_mean);
		if (ady > max_abs_dy) max_abs_dy = ady;
		if (adx > max_abs_dx) max_abs_dx = adx;
		double df = raw_flux.v[i] - fm;
		fv += df*df;
		fdy += df*(fit_raw_dy.v[i] - dy_mean);
		fdx += df*(fit_raw_dx.v[i] - dx_mean);
	}
	double bright_y = fv > 0.0 ? fdy/fv*fm : 0.0;
	double bright_x = fv > 0.0 ? fdx/fv*fm : 0.0;
	struct series infer_dy = {0}, infer_dx = {0}, hold_dy = {0}, hold_dx = {0},
		predict_dy = {0}, predict_dx = {0};
	size_t prev = (size_t)-1, next = 0;
	size_t prev2 = (size_t)-1;
	for (size_t k = 200; k < used; k++) {
		if (track_kind[k] == 1) { prev2 = prev; prev = k; continue; }
		if (track_kind[k] != 2 || prev == (size_t)-1) continue;
		if (next <= k) {
			next = k+1;
			while (next < used && track_kind[next] != 1) next++;
		}
		if (next >= used) continue;
		double a = (double)(k-prev)/(double)(next-prev);
		double ty = track_y[prev] + a*(track_y[next]-track_y[prev]);
		double tx = track_x[prev] + a*(track_x[next]-track_x[prev]);
		ser_add(&infer_dy, track_y[k]-ty);
		ser_add(&infer_dx, track_x[k]-tx);
		ser_add(&hold_dy, track_y[prev]-ty);
		ser_add(&hold_dx, track_x[prev]-tx);
		if (prev2 != (size_t)-1) {
			double dt = (double)(prev-prev2);
			double py = track_y[prev] + (k-prev)*(track_y[prev]-track_y[prev2])/dt;
			double px = track_x[prev] + (k-prev)*(track_x[prev]-track_x[prev2])/dt;
			ser_add(&predict_dy, py-ty); ser_add(&predict_dx, px-tx);
		}
	}
	double infer_y_bias = ser_mean(&infer_dy), infer_x_bias = ser_mean(&infer_dx);
	double infer_y_rms = ser_rms_about_mean(&infer_dy);
	double infer_x_rms = ser_rms_about_mean(&infer_dx);
	double hold_y_rms = ser_rms_about_mean(&hold_dy);
	double hold_x_rms = ser_rms_about_mean(&hold_dx);
	printf("\n--- independent raw-pixel tracking check ----------------------\n");
	printf("  compared %zu complete frames against border-subtracted raw centroid\n",
		fit_raw_dy.n);
	printf("  fit - raw bias       y %+7.3f px   x %+7.3f px\n", dy_mean, dx_mean);
	printf("  disagreement RMS     y %7.3f px   x %7.3f px\n", dy_rms, dx_rms);
	printf("  disagreement max     y %7.3f px   x %7.3f px\n",
		max_abs_dy, max_abs_dx);
	printf("  brightness coupling  y %+7.3f px / fractional flux   "
		"x %+7.3f\n", bright_y, bright_x);
	printf("  inferred shutter frames %zu, checked against bracketing complete frames\n",
		infer_dy.n);
	printf("  inference bias       y %+7.3f px   x %+7.3f px\n",
		infer_y_bias, infer_x_bias);
	printf("  inference error RMS  y %7.3f px   x %7.3f px\n",
		infer_y_rms, infer_x_rms);
	printf("  causal hold RMS      y %7.3f px   x %7.3f px\n",
		hold_y_rms, hold_x_rms);
	printf("  causal velocity RMS  y %7.3f px   x %7.3f px\n",
		ser_rms_about_mean(&predict_dy), ser_rms_about_mean(&predict_dx));
	for (double a=0.25; a<=0.75; a+=0.25) {
		struct series by={0}, bx={0};
		for(size_t i=0;i<infer_dy.n;i++) {
			ser_add(&by, hold_dy.v[i]+a*(infer_dy.v[i]-hold_dy.v[i]));
			ser_add(&bx, hold_dx.v[i]+a*(infer_dx.v[i]-hold_dx.v[i]));
		}
		printf("  blend %.2f RMS       y %7.3f px   x %7.3f px\n", a,
			ser_rms_about_mean(&by),ser_rms_about_mean(&bx));
		free(by.v); free(bx.v);
	}

	// ---------------- fit_com acceptance -----------------------------
	printf("\n=================================================================\n");
	const char *pick; const char *why;
	double fit_rejected = (double)frej/used;
	bool raw_agrees = fit_raw_dy.n > used/10 && dy_rms < 0.5 && dx_rms < 0.5
		&& fabs(bright_y) < 0.5 && fabs(bright_x) < 0.5;
	// The PWM path is a causal hold, not a non-causal reconstruction. Its
	// error naturally grows with real open-loop jitter, so a fixed absolute
	// RMS limit rejects the noisiest (and most important) valid captures.
	// Require it to be no worse than the independently computed causal-hold
	// baseline instead; raw-centroid agreement above still catches fabricated
	// motion on the complete frames that train the hold.
	bool inference_agrees = pwm_period <= 0.0
		|| (infer_dy.n > used/20
			&& infer_y_rms <= hold_y_rms+0.25
			&& infer_x_rms <= hold_x_rms+0.25
			&& fabs(infer_y_bias) < 0.75
			&& fabs(infer_x_bias) < 0.75);
	// A rolling-shutter frame that crosses a PWM edge is intentionally held:
	// at 80% duty the 20% off interval plus row-sliced transitions measured
	// about 32% unusable frames.  Independent centroid agreement below still
	// prevents this wider live-fraction allowance from admitting false motion.
	const double max_rejected = 0.40;
	if (beam_clipped) {
		pick = "FAIL";
		why = "the measured beam/window reaches outside the ROI. Re-centre\n"
		"  or enlarge the ROI before using fit_com.";
	} else if (sat_frac > 1e-3) {
		pick = "FAIL";
		why = "the beam is saturating, which destroys sub-pixel accuracy\n"
		"  for fit_com. Reduce exposure or gain and re-run this survey.";
	} else if (fit_rejected > max_rejected) {
		pick = "FAIL";
		why = "fit_com rejected more than 40% of frames. Fix\n"
		"  centring, illumination, or tracker gates before driving.";
	} else if (!raw_agrees) {
		pick = "FAIL";
		why = "fit_com does not agree with the independent raw-pixel\n"
		"  centroid or its error depends on brightness. Do not drive.";
	} else if (!inference_agrees) {
		pick = "FAIL";
		why = "shutter-frame inferred centres do not agree with the\n"
		"  trajectory interpolated between complete frames. Do not drive.";
	} else {
		pick = "fit_com";
		why = "fit_com passed ROI, saturation, live-frame, independent\n"
		"  raw-centroid, and brightness-coupling gates.";
	}
	printf(" FIT_COM RESULT:  %s\n", pick);
	printf("=================================================================\n");
	printf("  %s\n", why);


	if (json_path) {
		json_object *result = json_object_new_object();
		json_object_object_add(result, "recommendation",
			json_object_new_string(!strcmp(pick, "fit_com") ? "fit_com" :
				"none"));
		json_object_object_add(result, "fit_com", json_object_get(fp));
		json_object_object_add(result, "frames",
			json_object_new_int64((int64_t)nframes));
		json_object_object_add(result, "fs_hz", json_object_new_double(fs));
		json_object_object_add(result, "beam_sigma_px",
			json_object_new_double(sigma));
		json_object_object_add(result, "beam_amplitude_counts",
			json_object_new_double(amp_mean));
		json_object_object_add(result, "raw_disagreement_rms_y_px",
			json_object_new_double(dy_rms));
		json_object_object_add(result, "raw_disagreement_rms_x_px",
			json_object_new_double(dx_rms));
		json_object_object_add(result, "brightness_coupling_y_px",
			json_object_new_double(bright_y));
		json_object_object_add(result, "brightness_coupling_x_px",
			json_object_new_double(bright_x));
		json_object_object_add(result, "amplitude_p50_counts",
			json_object_new_double(amp_p50));
		json_object_object_add(result, "amplitude_p90_counts",
			json_object_new_double(amp_p90));
		json_object_object_add(result, "amplitude_p99_counts",
			json_object_new_double(amp_p99));
		json_object *camera = json_object_new_object();
		json_object_object_add(camera, "exposure",
			json_object_new_int((int)lround(suggested_exposure)));
		json_object_object_add(camera, "gain",
			json_object_new_int((int)lround(suggested_gain)));
		json_object_object_add(result, "camera", camera);
		json_object_object_add(result, "saturation_fraction",
			json_object_new_double(sat_frac));
		json_object_object_add(result, "beam_clipped",
			json_object_new_boolean(beam_clipped));
		json_object *position = json_object_new_object();
		json_object_object_add(position, "y",
			json_object_new_double(cy_mean));
		json_object_object_add(position, "x",
			json_object_new_double(cx_mean));
		json_object_object_add(result, "beam_position", position);
		json_object_object_add(result, "fit_rejected_fraction",
			json_object_new_double((double)frej/used));
		json_object_object_add(result, "fit_hf_px",
			json_object_new_double(fit_hf));
		json_object_object_add(result, "step_p999_px",
			json_object_new_double(step_p999));
		json_object_object_add(result, "max_rejected_streak_frames",
			json_object_new_int64((int64_t)max_reject_streak));
		if (json_object_to_file_ext(json_path, result,
				JSON_C_TO_STRING_PRETTY) < 0) {
			fprintf(stderr, "cannot write JSON result %s\n", json_path);
			json_object_put(result);
			return 1;
		}
		printf("\n  Machine-readable result written to %s\n", json_path);
		json_object_put(result);
	}

	fit_com_fini(&fit_dev);
	json_object_put(fp);
	free(fy.v); free(fx.v); free(rres.v);
	free(ftime); free(step_samples); free(ring); free(raw);
	free(amp_samples); free(flux_samples); free(sigma_samples);
	gsl_matrix_uchar_free(img);
	return 0;
}
