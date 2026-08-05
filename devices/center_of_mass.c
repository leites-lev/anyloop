#include <math.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

#include "anyloop.h"
#include "logging.h"
#include "thread_pool.h"
#include "center_of_mass.h"
#include "xalloc.h"


// TODO: optimize? for example, if the matrix is contiguous and 8x8 in size, the
// compiler can do some really nice optimizations here. but for variable sizes,
// and non-contiguous matrices ... it gets a little harder.
// With -O3, on my old laptop, I timed that this could do 10 million 8x8
// matrices in 4.302 s, whereas the naive solution with contiguous memory took
// 2.447 s. So even with this naive non-contiguous solution, we still get at
// worst 0.4 μs per matrix, which is ... not the worst. That's 43 μs for a 10x10
// grid of submatrices, with no multithreading. GCC is quite intelligent.
/** Center of mass task for gsl_matrix_uchar.
* Will write (y,x) coords (in that order) of the center of mass of the uchar
* matrix at src->mat to dst[0] and dst[1], subtracting src->threshold from
* each pixel (clamping to zero) before accumulating. */
void com_mat_uchar(struct aylp_com_src *src, double *dst)
{
	double y = 0.0, x = 0.0, s = 0.0;
	// take weighted average
	for (size_t i=0; i < src->mat.size1; i++) {
		for (size_t j=0; j < src->mat.size2; j++) {
			unsigned char raw = src->mat.data[i*src->mat.tda + j];
			unsigned char el = raw > src->threshold
				? raw - src->threshold : 0;
			y += i*el;
			x += j*el;
			s += el;
		}
	}
	// if all pixels are at or below threshold, output centre
	if (!s) { dst[0] = 0.0; dst[1] = 0.0; return; }
	// set final values, scaling from 0:size-1 (given by y/s or x/s) to -1:1
	dst[0] = -1.0 + 2*y/(s*(src->mat.size1-1));
	dst[1] = -1.0 + 2*x/(s*(src->mat.size2-1));
}


// This is nice and fast, but doesn't work for us, because we want to find the
// com of a slice of non-contiguous memory....
// TODO: actually, it seems like contiguous matrices are one of the most likely
// scenarios in practice, e.g. if we're reading from a camera. Consider
// switching to this at runtime!
// /** Hardcoded 8x8 center of mass task so the compiler can optimize it.
// * Will look at 64 uchars at src, and write the output as two doubles in dst
// * in (y,x) order. */
// void com_8x8(unsigned char *src, double *dst)
// {
// 	double y = 0.0, x = 0.0, s = 0.0;
// 	for (char i = 0; i < 8; i++) {
// 		for (char j = 0; j < 8; j++) {
// 			y += i * src->data[8*i+j];
// 			x += j * src->data[8*i+j];
// 			s += src->data[8*i+j];
// 		}
// 	}
// 	dst[0] = y / s;
// 	dst[1] = x / s;
// }


int center_of_mass_init(struct aylp_device *self)
{
	int err;
	self->device_data = xcalloc(1, sizeof(struct aylp_center_of_mass_data));
	struct aylp_center_of_mass_data *data = self->device_data;

	// default params
	data->region_height = 0;
	data->region_width = 0;
	data->thread_count = 1;
	data->threshold = 0;
	data->min_peak = 0;	// 0 => any nonzero sum counts as a beam
	data->ref_cut = 0.0;		// 0 => no partial-beam test
	data->ref_warmup = 200;
	data->ref_rate = 0.01;
	data->ref_floor = 0.25;
	data->had_beam = true;	// so the first loss logs
	data->track = false;
	data->init_y = -1;	// <0 => acquire from the brightest pixel
	data->init_x = -1;
	data->reacquire_after = 30;
	data->acquire_seconds = 0.0;	// no acquisition phase by default
	data->acquire_height = 0;	// 0 => the whole image
	data->acquire_width = 0;
	// parse parameters
	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
			// keys starting with _ are comments
		} else if (!strcmp(key, "region_height")) {
			data->region_height = json_object_get_uint64(val);
			log_trace("region_height = %zu", data->region_height);
		} else if (!strcmp(key, "region_width")) {
			data->region_width = json_object_get_uint64(val);
			log_trace("region_width = %zu", data->region_width);
		} else if (!strcmp(key, "thread_count")) {
			data->thread_count = json_object_get_uint64(val);
			if (data->thread_count == 0) {
				log_error("Correcting 0 threads to 1 thread");
				data->thread_count = 1;
			}
			log_trace("thread_count = %zu", data->thread_count);
		} else if (!strcmp(key, "threshold")) {
			data->threshold = (unsigned char)json_object_get_int(val);
			log_trace("threshold = %u", data->threshold);
		} else if (!strcmp(key, "min_peak")) {
			data->min_peak = (unsigned char)json_object_get_int(val);
			log_trace("min_peak = %u", data->min_peak);
		} else if (!strcmp(key, "ref_cut")) {
			data->ref_cut = json_object_get_double(val);
			if (data->ref_cut < 0.0 || data->ref_cut > 1.0) {
				log_error("ref_cut must be in [0, 1]; it is a "
					"fraction of the frame's typical row"
				);
				return -1;
			}
			log_trace("ref_cut = %G", data->ref_cut);
		} else if (!strcmp(key, "ref_warmup")) {
			data->ref_warmup = json_object_get_uint64(val);
			if (!data->ref_warmup) {
				log_error("ref_warmup must be nonzero");
				return -1;
			}
			log_trace("ref_warmup = %zu", data->ref_warmup);
		} else if (!strcmp(key, "ref_rate")) {
			data->ref_rate = json_object_get_double(val);
			if (data->ref_rate <= 0.0 || data->ref_rate > 1.0) {
				log_error("ref_rate must be in (0, 1]");
				return -1;
			}
			log_trace("ref_rate = %G", data->ref_rate);
		} else if (!strcmp(key, "ref_floor")) {
			data->ref_floor = json_object_get_double(val);
			if (data->ref_floor <= 0.0 || data->ref_floor >= 1.0) {
				log_error("ref_floor must be in (0, 1)");
				return -1;
			}
			log_trace("ref_floor = %G", data->ref_floor);
		} else if (!strcmp(key, "track")) {
			data->track = json_object_get_boolean(val);
			log_trace("track = %d", data->track);
		} else if (!strcmp(key, "init_y")) {
			data->init_y = json_object_get_int64(val);
			log_trace("init_y = %ld", data->init_y);
		} else if (!strcmp(key, "init_x")) {
			data->init_x = json_object_get_int64(val);
			log_trace("init_x = %ld", data->init_x);
		} else if (!strcmp(key, "reacquire_after")) {
			data->reacquire_after = json_object_get_uint64(val);
			if (!data->reacquire_after) {
				log_error("reacquire_after must be nonzero");
				return -1;
			}
			log_trace("reacquire_after = %zu", data->reacquire_after);
		} else if (!strcmp(key, "acquire_seconds")) {
			data->acquire_seconds = json_object_get_double(val);
			if (data->acquire_seconds < 0.0) {
				log_error("acquire_seconds must be >= 0");
				return -1;
			}
			log_trace("acquire_seconds = %G", data->acquire_seconds);
		} else if (!strcmp(key, "acquire_height")) {
			data->acquire_height = json_object_get_uint64(val);
			log_trace("acquire_height = %zu", data->acquire_height);
		} else if (!strcmp(key, "acquire_width")) {
			data->acquire_width = json_object_get_uint64(val);
			log_trace("acquire_width = %zu", data->acquire_width);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}
	if (!data->region_height || !data->region_width) {
		log_error("You must provide nonzero region_height and "
			"region_width params"
		);
		return -1;
	}
	if ((data->init_y < 0) != (data->init_x < 0)) {
		log_error("Provide both init_y and init_x, or neither");
		return -1;
	}
	if (data->min_peak && !data->track) {
		log_warn("min_peak only applies in track mode; ignoring it");
		data->min_peak = 0;
	}
	if (data->min_peak && data->min_peak <= data->threshold) {
		// a pixel has to clear `threshold` to weigh anything at all, so a
		// min_peak at or below it can never reject a frame the old code
		// would have accepted -- it is silently doing nothing
		log_warn("min_peak %u is at or below threshold %u, so it can "
			"never reject a frame; raise it above the background "
			"peak of a beamless frame", data->min_peak,
			data->threshold
		);
	}
	if (data->ref_cut > 0.0 && !data->track) {
		log_warn("ref_cut only applies in track mode; ignoring it");
		data->ref_cut = 0.0;
	}
	if (data->ref_cut > 0.0) {
		if (data->region_height < 4) {
			// with three rows or fewer there is no meaningful
			// profile to compare against, and after ref_floor
			// trims the weak ones there would be nothing left
			log_error("ref_cut needs region_height >= 4 to have a "
				"row profile to compare; it is %zu",
				data->region_height
			);
			return -1;
		}
		data->ref = xcalloc(data->region_height, sizeof(double));
		data->rows = xcalloc(data->region_height, sizeof(double));
		data->rho = xcalloc(data->region_height, sizeof(double));
	}

	if (data->track) {
		// the tracking window is a single region by definition, so there
		// is nothing to hand out to a thread pool
		if (data->thread_count > 1) {
			log_warn("track mode uses one region; "
				"ignoring thread_count = %zu", data->thread_count
			);
			data->thread_count = 1;
		}
		self->proc = &center_of_mass_proc_track;
		self->fini = &center_of_mass_fini;
	} else if (data->thread_count > 1) {
		// start threads
		data->threads = xmalloc(data->thread_count * sizeof(pthread_t));
		for (size_t t = 0; t < data->thread_count; t++) {
			err = pthread_create(&data->threads[t],
				0, task_runner, &data->queue
			);
			if (err) {
				log_error("Couldn't create pthread: %s",
					strerror(err)
				);
				return -1;
			}
		}
		log_info("Started %zu threads", data->thread_count);
		self->proc = &center_of_mass_proc_threaded;
		self->fini = &center_of_mass_fini_threaded;
	} else {
		// no threading
		self->proc = &center_of_mass_proc;
		self->fini = &center_of_mass_fini;
	}

	// set types and units
	self->type_in = AYLP_T_MATRIX_UCHAR;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_VECTOR;
	self->units_out = AYLP_U_MINMAX;
	return 0;
}


int center_of_mass_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_center_of_mass_data *data = self->device_data;
	size_t max_y = state->matrix_uchar->size1;
	size_t max_x = state->matrix_uchar->size2;
	size_t y_subap_count = max_y / data->region_height;
	size_t x_subap_count = max_x / data->region_width;
	size_t subap_count = y_subap_count * x_subap_count;
	if (UNLIKELY(!subap_count)) {
		log_error("Refusing to process zero subapertures; "
			"region size is %zu by %zu but image is %zu by %zu",
			data->region_height, data->region_width, max_y, max_x
		);
		return -1;
	}
	// allocate the com vector if needed
	if (UNLIKELY(!data->com || data->com->size < subap_count*2)) {
		xfree_type(gsl_vector, data->com);
		data->com = xmalloc_type(gsl_vector, subap_count*2);
	}

	size_t n = 0;
	for (size_t i=0; i < y_subap_count; i++) {
		for (size_t j=0; j < x_subap_count; j++) {
			double y = 0.0, x = 0.0, s = 0.0;
			// inlined version of com_mat_uchar basically
			for (size_t l=0; l < data->region_height; l++) {
				for (size_t m=0; m < data->region_width; m++) {
					unsigned char raw;
					raw = state->matrix_uchar->data[
						(i*data->region_height + l)
						* state->matrix_uchar->tda
						+ j*data->region_width + m
					];
					unsigned char el = raw > data->threshold
						? raw - data->threshold : 0;
					y += l*el;
					x += m*el;
					s += el;
				}
			}
			if (!s) {
				data->com->data[2*n] = 0.0;
				data->com->data[2*n+1] = 0.0;
			} else {
				data->com->data[2*n] = -1.0
					+ 2*y/(s*(data->region_height-1));
				data->com->data[2*n+1] = -1.0
					+ 2*x/(s*(data->region_width-1));
			}
			n += 1;
		}
	}

	// zero-copy update of pipeline state
	state->vector = data->com;
	// housekeeping on the header
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = data->com->size;
	state->header.log_dim.x = 1;
	return 0;
}


int center_of_mass_fini(struct aylp_device *self)
{
	struct aylp_center_of_mass_data *data = self->device_data;
	// The reject rate is the number that decides whether gating chopped
	// frames is workable at all, and it is the one thing the rate-limited
	// per-episode warnings cannot tell you. Every held frame hands the loop
	// the previous frame's error again, so the integrator keeps winding on
	// stale data -- a few percent is free, a third is added loop delay.
	if (data && data->track && data->n_held) {
		log_info("center_of_mass: held %zu of %zu frames (%.2f%%) over "
			"%zu episodes", data->n_held, data->n_frames,
			100.0 * data->n_held / data->n_frames, data->n_episodes
		);
	}
	if (data) {
		xfree(data->ref);
		xfree(data->rows);
		xfree(data->rho);
	}
	xfree(self->device_data);
	return 0;
}


/** Place the tracking window on the brightest pixel of the whole image.
* Used to acquire on the first frame, and to recover after the beam has been
* lost for reacquire_after frames. Note that this locks onto whatever is
* brightest — if a stray reflection outpeaks the beam, pass init_y/init_x. */
static bool acquire_window(
	struct aylp_center_of_mass_data *data, gsl_matrix_uchar *img
) {
	unsigned char best = 0;
	size_t by = img->size1 / 2;
	size_t bx = img->size2 / 2;
	for (size_t i = 0; i < img->size1; i++) {
		for (size_t j = 0; j < img->size2; j++) {
			unsigned char v = img->data[i*img->tda + j];
			if (v > best) { best = v; by = i; bx = j; }
		}
	}
	// The brightest pixel of a beamless frame is just the loudest speck of
	// read noise, and it lands somewhere new every frame. Moving the window
	// there is worse than not moving at all: if the beam returns where it
	// left, an unmoved window is already on it. Park at the image centre
	// only when we have never had a placement to keep.
	if (data->min_peak && best < data->min_peak) {
		if (!data->acquired) {
			data->win_y = img->size1 / 2;
			data->win_x = img->size2 / 2;
		}
		return false;
	}
	data->win_y = by;
	data->win_x = bx;
	return true;
}


/** Slide the window centre so that a win_h by win_w window lies fully inside the
* image. Callers must already have checked that the window is no bigger than the
* image. */
static void clamp_window(
	struct aylp_center_of_mass_data *data,
	size_t win_h, size_t win_w, size_t max_y, size_t max_x
) {
	size_t half_y = win_h / 2;
	size_t half_x = win_w / 2;
	if (data->win_y < half_y) data->win_y = half_y;
	if (data->win_x < half_x) data->win_x = half_x;
	if (data->win_y > max_y - win_h + half_y)
		data->win_y = max_y - win_h + half_y;
	if (data->win_x > max_x - win_w + half_x)
		data->win_x = max_x - win_w + half_x;
}


static double com_monotonic_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + 1e-9 * t.tv_nsec;
}


/** Result of the partial-beam test, for logging. */
struct com_cut {
	bool cut;	// some rows are lit and others are not
	double lo;	// dimmest significant row, relative to the reference
	double mid;	// the frame's typical row, relative to the reference
	size_t n_sig;	// rows that carried enough reference to be judged
};

/** Is this frame's beam cut across rows?
*
* Divides the frame's row profile by the learned reference profile. On a whole
* beam every row comes back at the same ratio -- whatever the frame's brightness
* happens to be relative to the reference, uniformly across rows -- so the
* comparison below is between the dimmest row and the frame's own typical row,
* and the frame's overall level cancels. On a cut beam the ratios split into two
* groups, near the frame's level where the shutter let light through and near
* zero where it did not, and the dimmest row falls far under the typical one.
*
* The median is the typical row rather than the mean or the max: on a frame cut
* clean in half the mean sits halfway between the two groups and the max is one
* noisy row, while the median stays with whichever group holds more rows, which
* is all this needs to see a gap. */
static struct com_cut com_check_cut(
	struct aylp_center_of_mass_data *data, size_t win_h
) {
	struct com_cut r = {0};
	double ref_max = 0.0;
	for (size_t i = 0; i < win_h; i++)
		if (data->ref[i] > ref_max) ref_max = data->ref[i];
	if (ref_max <= 0.0)
		return r;	// no reference yet; nothing to say
	double floor = data->ref_floor * ref_max;

	// ratios of the rows the reference says should carry real signal,
	// insertion-sorted as they are collected. win_h is a window height --
	// tens of rows at most -- so this is a handful of compares, and it keeps
	// the median exact rather than approximating it
	for (size_t i = 0; i < win_h; i++) {
		if (data->ref[i] < floor)
			continue;
		double v = data->rows[i] / data->ref[i];
		size_t k = r.n_sig++;
		while (k > 0 && data->rho[k-1] > v) {
			data->rho[k] = data->rho[k-1];
			k--;
		}
		data->rho[k] = v;
	}
	// three rows is the fewest that can show a dim one against a typical
	// one; below that the test would be comparing a row to itself
	if (r.n_sig < 3)
		return r;

	r.lo = data->rho[0];
	r.mid = data->rho[r.n_sig / 2];
	// mid <= 0 means every significant row lost its light while the window
	// as a whole still has some -- the beam is not where the reference says
	// it is, which is not a frame to steer on either
	r.cut = r.mid <= 0.0 || r.lo < data->ref_cut * r.mid;
	return r;
}


/** Fold this frame's row profile into the reference.
*
* Before the bootstrap finishes this takes the row-wise maximum, because the
* source may already be chopping while it runs: a cut only ever removes light,
* so each row's largest value over enough frames is its uncut value, while a
* mean would learn a blend of whole and cut beams and sit too low. Afterwards it
* is an EMA over accepted frames only, which follows slow drift in power, focus
* and alignment without letting cut frames pull it down toward themselves. */
static void com_update_ref(struct aylp_center_of_mass_data *data, size_t win_h)
{
	if (!data->ref_ready) {
		for (size_t i = 0; i < win_h; i++)
			if (data->rows[i] > data->ref[i])
				data->ref[i] = data->rows[i];
		if (++data->ref_seen >= data->ref_warmup) {
			data->ref_ready = true;
			log_info("center_of_mass: reference profile learned "
				"from %zu frames; partial-beam gate is now "
				"active", data->ref_seen
			);
		}
		return;
	}
	for (size_t i = 0; i < win_h; i++)
		data->ref[i] += data->ref_rate * (data->rows[i] - data->ref[i]);
}


/** Throw the learned reference away.
*
* Called when the window re-acquires: the beam has been gone long enough that
* the window is being replaced from scratch, and a profile learned at the old
* position describes a beam that may not be there any more. Relearning costs
* ref_warmup frames; keeping a stale reference costs every frame after. */
static void com_reset_ref(struct aylp_center_of_mass_data *data)
{
	if (!data->ref)
		return;
	for (size_t i = 0; i < data->region_height; i++)
		data->ref[i] = 0.0;
	data->ref_seen = 0;
	data->ref_ready = false;
}


/** Seconds between beam-lost/beam-back log lines. */
#define COM_LOG_INTERVAL 1.0

/** True at most once per COM_LOG_INTERVAL, so the beam-lost and beam-back
* transitions can be logged without flooding.
*
* These fire once per loss episode, which is the right cadence for a beam that
* occasionally drops out and completely the wrong one for a chopped source: a
* flux gate rejecting alternate frames produces an episode every few frames, and
* at loop rate that is hundreds of log lines a second going to a terminal in the
* middle of the control loop. The suppressed episodes are still counted, and
* fini reports the totals, so nothing is lost but the noise. */
static bool com_log_ok(struct aylp_center_of_mass_data *data)
{
	double now = com_monotonic_s();
	if (now - data->last_log_t < COM_LOG_INTERVAL)
		return false;
	data->last_log_t = now;
	return true;
}


/** Center of mass over a single window that follows the beam.
*
* Each frame the sum is taken over a region_height by region_width box centred
* on the previous frame's center of mass, so anything outside that box — a stray
* reflection elsewhere on the sensor, say — never enters the sum. The image
* itself is untouched, so a udp_sink placed ahead of this device still shows the
* whole frame.
*
* The output is normalized across the *whole image*, not the window. This is the
* important part: the window chases the beam, so a window-relative coordinate
* would sit near zero no matter where the beam actually was, and the loop would
* have no error signal to act on. Normalizing to the image keeps the setpoint at
* the image centre and keeps the error-per-pixel — hence the loop gain —
* independent of the window size.
*/
int center_of_mass_proc_track(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_center_of_mass_data *data = self->device_data;
	gsl_matrix_uchar *img = state->matrix_uchar;
	size_t max_y = img->size1;
	size_t max_x = img->size2;
	if (UNLIKELY(data->region_height > max_y
			|| data->region_width > max_x)) {
		log_error("Tracking window is %zu by %zu but image is only "
			"%zu by %zu", data->region_height, data->region_width,
			max_y, max_x
		);
		return -1;
	}
	// allocate the com vector if needed; track mode emits a single y,x pair
	if (UNLIKELY(!data->com || data->com->size < 2)) {
		xfree_type(gsl_vector, data->com);
		data->com = xmalloc_type(gsl_vector, 2);
	}
	if (UNLIKELY(!data->acquired)) {
		bool placed;
		if (data->init_y >= 0) {
			data->win_y = (size_t)data->init_y;
			data->win_x = (size_t)data->init_x;
			placed = true;
		} else {
			placed = acquire_window(data, img);
		}
		// Staying unacquired until a real beam shows up also keeps the
		// acquisition phase honest: its clock must start when there is
		// something to acquire, or it expires against a dark sensor and
		// narrows the window onto noise.
		if (placed) {
			data->acquired = true;
			data->acquiring = data->acquire_seconds > 0.0;
			data->acquire_t0 = com_monotonic_s();
			log_info("center_of_mass: acquired window at (%zu,%zu)%s",
				data->win_y, data->win_x,
				data->acquiring
					? "; starting wide acquisition phase" : ""
			);
		}
	}
	// During the acquisition phase the sum runs over a wider window (the whole
	// image by default). The point is that a wide window is flux-weighted over
	// everything in it, so the centroid is dragged toward whichever spot carries
	// the most light, rather than toward whichever spot the brightest-pixel scan
	// happened to hit first in raster order. When the phase ends we narrow onto
	// wherever that centroid settled.
	size_t win_h = data->region_height;
	size_t win_w = data->region_width;
	if (UNLIKELY(data->acquiring)) {
		win_h = data->acquire_height ? data->acquire_height : max_y;
		win_w = data->acquire_width ? data->acquire_width : max_x;
		if (win_h > max_y) win_h = max_y;
		if (win_w > max_x) win_w = max_x;
	}
	clamp_window(data, win_h, win_w, max_y, max_x);
	size_t org_y = data->win_y - win_h/2;
	size_t org_x = data->win_x - win_w/2;

	// The row profile is only kept when the partial-beam test is armed and
	// the window is at its configured size. During the acquisition phase the
	// window is a different (wider) shape, so a profile taken then does not
	// describe the same thing and must not be mixed into the reference.
	bool profile = data->ref_cut > 0.0 && !data->acquiring;

	double y = 0.0, x = 0.0, s = 0.0;
	unsigned char peak = 0;
	for (size_t l = 0; l < win_h; l++) {
		double row = 0.0;
		for (size_t m = 0; m < win_w; m++) {
			unsigned char raw = img->data[
				(org_y + l) * img->tda + org_x + m
			];
			if (raw > peak) peak = raw;
			unsigned char el = raw > data->threshold
				? raw - data->threshold : 0;
			// accumulate in image coordinates, not window ones
			y += (org_y + l)*el;
			x += (org_x + m)*el;
			s += el;
			row += el;
		}
		// win_h == region_height whenever `profile` is set, so this
		// cannot run off the end of the array
		if (profile) data->rows[l] = row;
	}

	// "Is the beam in frame?" is a question about brightness, and a nonzero
	// sum does not answer it: sensor read noise running a few counts above
	// `threshold` gives a perfectly nonzero sum whose centroid is noise and
	// wanders the full width of the window. Only a real peak separates a
	// beam from a dark frame -- with min_peak 0 this is exactly the old
	// s != 0 test, so configs that do not set it are unaffected.
	//
	// The three tests fail for different reasons and the log line has to say
	// which, so keep them apart rather than folding them into one bool: an
	// empty sum means nothing cleared `threshold`, a low peak means the
	// window holds no bright pixel, and a split row profile with a healthy
	// peak means the beam is only partly there -- a rolling shutter that
	// caught a chopped source mid-cycle. Reporting the wrong one sends you
	// tuning the wrong parameter.
	//
	// The partial-beam test is skipped until the reference has been learned,
	// so the first ref_warmup frames go through on the brightness tests
	// alone. That is deliberate: a gate is worth less than the reference it
	// would be guessing at, and at loop rate the warmup is milliseconds.
	struct com_cut cut = {0};
	const char *why = 0;
	if (UNLIKELY(s == 0.0))
		why = "sum";
	else if (UNLIKELY(peak < data->min_peak))
		why = "peak";
	else if (profile && data->ref_ready) {
		cut = com_check_cut(data, win_h);
		if (UNLIKELY(cut.cut))
			why = "cut";
	}
	bool beam = !why;
	// only whole frames teach the reference what a whole beam looks like
	if (profile && beam)
		com_update_ref(data, win_h);

	data->n_frames++;
	if (LIKELY(beam)) {
		double abs_y = y/s, abs_x = x/s;
		data->last_y = -1.0 + 2*abs_y/(max_y - 1);
		data->last_x = -1.0 + 2*abs_x/(max_x - 1);
		// recentre the window for the next frame
		data->win_y = (size_t)(abs_y + 0.5);
		data->win_x = (size_t)(abs_x + 0.5);
		if (UNLIKELY(!data->had_beam)) {
			if (com_log_ok(data))
				log_info("center_of_mass: beam back after %zu "
					"frames (peak %u, dimmest row %.2f of "
					"typical); resuming updates", data->lost,
					peak, cut.mid > 0.0 ? cut.lo/cut.mid : 1.0
				);
			data->had_beam = true;
		}
		data->lost = 0;
		state->header.status &= (aylp_status)~AYLP_BEAM_LOST;
	} else {
		// No beam: hold the last good output rather than emitting a
		// centroid we do not believe. Reporting the noise centroid would
		// feed the loop a fictitious error; reporting (0,0) reads
		// downstream as "perfectly centred" and would let the integrator
		// park and then lurch when the beam reappears. Holding is the
		// only option that leaves the loop where the beam last was.
		// If the beam stays gone the window is stranded and can never
		// find it again, so fall back to a full-image re-acquire.
		data->n_held++;
		if (UNLIKELY(data->had_beam)) {
			data->n_episodes++;
			if (com_log_ok(data)) {
				if (!strcmp(why, "sum"))
					log_warn("center_of_mass: no beam in frame "
						"(no pixel in the window is above "
						"threshold %u; window peak %u); "
						"holding centroid (%.4f,%.4f)",
						data->threshold, peak,
						data->last_y, data->last_x
					);
				else if (!strcmp(why, "peak"))
					log_warn("center_of_mass: no beam in frame "
						"(window peak %u < min_peak %u); "
						"holding centroid (%.4f,%.4f)",
						peak, data->min_peak,
						data->last_y, data->last_x
					);
				else
					log_warn("center_of_mass: beam cut across "
						"rows (dimmest of %zu rows is %.2f "
						"of typical, under ref_cut %G; "
						"peak %u is fine); holding centroid "
						"(%.4f,%.4f)", cut.n_sig,
						cut.mid > 0.0 ? cut.lo/cut.mid : 0.0,
						data->ref_cut, peak,
						data->last_y, data->last_x
					);
			}
			data->had_beam = false;
		}
		if (++data->lost >= data->reacquire_after) {
			state->header.status |= AYLP_BEAM_LOST;
			if (data->lost == data->reacquire_after) {
				log_warn("center_of_mass: no signal in window "
					"for %zu frames; re-acquiring",
					data->lost
				);
				// re-enter the wide phase: whatever stranded the
				// window will likely strand it again if we drop
				// straight back to the narrow one
				data->acquiring = data->acquire_seconds > 0.0;
				data->acquire_t0 = com_monotonic_s();
				// the reference describes a beam at the old
				// window position, which is exactly what we
				// have just stopped believing in
				com_reset_ref(data);
			}
			acquire_window(data, img);
		}
	}
	// end of the acquisition phase: narrow onto wherever the centroid settled
	if (UNLIKELY(data->acquiring)) {
		double elapsed = com_monotonic_s() - data->acquire_t0;
		if (elapsed >= data->acquire_seconds) {
			data->acquiring = false;
			log_info("center_of_mass: acquisition done after %.2f s; "
				"narrowing window from %zu by %zu to %zu by %zu, "
				"centred on (%zu,%zu)", elapsed, win_h, win_w,
				data->region_height, data->region_width,
				data->win_y, data->win_x
			);
		}
	}
	data->com->data[0] = data->last_y;
	data->com->data[1] = data->last_x;

	// zero-copy update of pipeline state
	state->vector = data->com;
	// Tell the rest of the pipeline whether this centroid is a fresh
	// measurement or last frame's held over. Everything downstream sees an
	// ordinary vector either way -- holding is invisible in the data, which
	// is the point of holding -- so without this flag an integrator has no
	// way to tell that it is winding on the same error over and over.
	if (LIKELY(beam))
		state->header.status &= ~(aylp_status)AYLP_NO_SIGNAL;
	else
		state->header.status |= AYLP_NO_SIGNAL;
	// housekeeping on the header
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = 2;
	state->header.log_dim.x = 1;
	return 0;
}


int center_of_mass_proc_threaded(
	struct aylp_device *self, struct aylp_state *state
)
{
	struct aylp_center_of_mass_data *data = self->device_data;
	size_t max_y = state->matrix_uchar->size1;
	size_t max_x = state->matrix_uchar->size2;
	size_t y_subap_count = max_y / data->region_height;
	size_t x_subap_count = max_x / data->region_width;
	// TODO: we assume one task per subaperture, but this might not be ideal
	// for all subaperture counts and sizes. It may be that the overhead
	// associated with processing one task can be comparable to the cost of
	// calculating on one subaperture, and we should be creating tasks that
	// process more than one subaperture at a time.
	size_t n_tasks = y_subap_count * x_subap_count;
	if (!n_tasks) {
		log_error("Refusing to process zero subapertures; "
			"region size is %zu by %zu but image is %zu by %zu",
			data->region_height, data->region_width, max_y, max_x
		);
		return -1;
	}
	// It's unfortunately quite ugly that we malloc here, but it's the
	// simplest fast solution I can think of to the issue of not knowing the
	// size of state->matrix_uchar when init() is run. Remember that `data`
	// is `calloc`ed so we are guaranteed to malloc on first proc.
	if (data->n_tasks < n_tasks) {
		// we *could* realloc instead of this free/malloc combo, but I
		// expect free/malloc to be faster since we don't care about
		// keeping the old memory (https://stackoverflow.com/a/39562813)
		xfree(data->tasks);
		data->tasks = xmalloc(n_tasks * sizeof(struct aylp_task));
		data->n_tasks = n_tasks;
		// allocate the com_src inputs for the tasks
		xfree(data->com_srcs);
		data->com_srcs = xmalloc(n_tasks * sizeof(struct aylp_com_src));
	}
	// allocate the com vector if needed
	if (!data->com || data->com->size < n_tasks*2) {
		xfree_type(gsl_vector, data->com);
		data->com = xmalloc_type(gsl_vector, n_tasks*2);
	}

	// start assigning tasks
	size_t t = 0;
	for (size_t i=0; i < y_subap_count; i++) {
		for (size_t j=0; j < x_subap_count; j++) {
			// set source data
			data->com_srcs[t].mat = gsl_matrix_uchar_submatrix(
				state->matrix_uchar,
				i * data->region_height,
				j * data->region_width,
				data->region_height,
				data->region_width
			).matrix;
			data->com_srcs[t].threshold = data->threshold;
			// set task
			data->tasks[t] = (struct aylp_task){
				.func = (void(*)(void*,void*))com_mat_uchar,
				.src = &data->com_srcs[t],
				.dst = (void *)(data->com->data+2*t),
				.next_task = 0
			};
			task_enqueue(&data->queue, &data->tasks[t]);
			t += 1;
		}
	}
	// wait for threads to finish
	while (data->queue.tasks_processing) {
		sched_yield();	// (is there a better function to call?)
	}
	// zero-copy update of pipeline state
	state->vector = data->com;
	// housekeeping on the header
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = data->com->size;
	state->header.log_dim.x = 1;
	return 0;
}


int center_of_mass_fini_threaded(struct aylp_device *self)
{
	struct aylp_center_of_mass_data *data = self->device_data;
	shut_queue(&data->queue);
	for (size_t t = 0; t < data->thread_count; t++) {
		pthread_join(data->threads[t], 0);
	}
	xfree(data->threads);
	xfree(data->tasks);
	xfree(data->com_srcs);
	xfree(self->device_data);
	return 0;
}
