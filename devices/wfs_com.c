#include <math.h>
#include <string.h>
#include <time.h>

#include "anyloop.h"
#include "logging.h"
#include "wfs_com.h"
#include "xalloc.h"


static double wfs_monotonic_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + 1e-9 * t.tv_nsec;
}


// index into data->ref / data->ref_set for subaperture (r,c)
static inline size_t subap_idx(struct aylp_wfs_com_data *data,
	size_t r, size_t c
) {
	return r * data->subap_cols + c;
}


// Place the window centre on the brightest pixel of the whole image. Used on
// first acquisition and after AYLP_BEAM_LOST recovery. Like center_of_mass,
// this locks onto whatever is brightest -- pass init_y/init_x if a stray
// reflection can outpeak the real beam.
static void acquire_window(
	struct aylp_wfs_com_data *data, gsl_matrix_uchar *img
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
	data->win_y = by;
	data->win_x = bx;
}


// Slide the window centre so the win_h by win_w extended window (subaperture
// grid plus the search margin on every side) lies fully inside the image.
static void clamp_window(struct aylp_wfs_com_data *data,
	size_t max_y, size_t max_x
) {
	size_t half_y = data->win_h / 2;
	size_t half_x = data->win_w / 2;
	if (data->win_y < half_y) data->win_y = half_y;
	if (data->win_x < half_x) data->win_x = half_x;
	if (data->win_y > max_y - data->win_h + half_y)
		data->win_y = max_y - data->win_h + half_y;
	if (data->win_x > max_x - data->win_w + half_x)
		data->win_x = max_x - data->win_w + half_x;
}


// (Re)acquire: place the window, reset every subaperture's reference
// template (the old ones describe pixel content at the old location and are
// meaningless at the new one), and flag the coming frame as a warm-up frame
// so seeding templates isn't mistaken for a lost signal.
static void reacquire(struct aylp_wfs_com_data *data, gsl_matrix_uchar *img)
{
	if (data->init_y >= 0 && !data->acquired) {
		data->win_y = (size_t)data->init_y;
		data->win_x = (size_t)data->init_x;
	} else {
		acquire_window(data, img);
	}
	clamp_window(data, img->size1, img->size2);
	memset(data->ref_set, 0, data->n_subaps * sizeof(bool));
	data->abs_y = (double)data->win_y;
	data->abs_x = (double)data->win_x;
	data->last_y = -1.0 + 2.0*data->abs_y/((double)img->size1 - 1.0);
	data->last_x = -1.0 + 2.0*data->abs_x/((double)img->size2 - 1.0);
	data->acquired = true;
	data->just_acquired = true;
	data->t_acquire = wfs_monotonic_s();
	data->lost = 0;
	log_info("wfs_com: acquired window at (%zu,%zu)", data->win_y, data->win_x);
}


int wfs_com_init(struct aylp_device *self)
{
	self->device_data = xcalloc(1, sizeof(struct aylp_wfs_com_data));
	struct aylp_wfs_com_data *data = self->device_data;

	// defaults
	data->threshold = 0;
	data->search_radius = 2;
	data->ref_beta = 0.02;
	data->ref_beta_init = 0.0;	// <= 0 disables the schedule
	data->ref_beta_tau = 2.0;
	data->min_confidence = 0.5;
	data->flux_floor = 1.0;
	data->min_valid_subaps = 0;	// 0 => derive from n_subaps at the end
	data->row_time = 0.0;		// disabled by default
	data->rolling_shutter = false;
	data->max_row_shear = 0.0;	// disabled by default
	data->init_y = -1;
	data->init_x = -1;
	data->reacquire_after = 10;

	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
		} else if (!strcmp(key, "subap_height")) {
			data->subap_h = json_object_get_uint64(val);
		} else if (!strcmp(key, "subap_width")) {
			data->subap_w = json_object_get_uint64(val);
		} else if (!strcmp(key, "subap_rows")) {
			data->subap_rows = json_object_get_uint64(val);
		} else if (!strcmp(key, "subap_cols")) {
			data->subap_cols = json_object_get_uint64(val);
		} else if (!strcmp(key, "threshold")) {
			data->threshold = (unsigned char)json_object_get_int(val);
		} else if (!strcmp(key, "search_radius")) {
			data->search_radius = json_object_get_uint64(val);
		} else if (!strcmp(key, "ref_beta")) {
			data->ref_beta = json_object_get_double(val);
		} else if (!strcmp(key, "ref_beta_init")) {
			data->ref_beta_init = json_object_get_double(val);
		} else if (!strcmp(key, "ref_beta_tau")) {
			data->ref_beta_tau = json_object_get_double(val);
		} else if (!strcmp(key, "min_confidence")) {
			data->min_confidence = json_object_get_double(val);
		} else if (!strcmp(key, "flux_floor")) {
			data->flux_floor = json_object_get_double(val);
		} else if (!strcmp(key, "min_valid_subaps")) {
			data->min_valid_subaps = json_object_get_uint64(val);
		} else if (!strcmp(key, "row_time")) {
			data->row_time = json_object_get_double(val);
			if (data->row_time > 0.0) data->rolling_shutter = true;
		} else if (!strcmp(key, "rolling_shutter")) {
			data->rolling_shutter = json_object_get_boolean(val);
		} else if (!strcmp(key, "max_row_shear")) {
			data->max_row_shear = json_object_get_double(val);
		} else if (!strcmp(key, "init_y")) {
			data->init_y = json_object_get_int64(val);
		} else if (!strcmp(key, "init_x")) {
			data->init_x = json_object_get_int64(val);
		} else if (!strcmp(key, "reacquire_after")) {
			data->reacquire_after = json_object_get_uint64(val);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	if (!data->subap_h || !data->subap_w
			|| !data->subap_rows || !data->subap_cols) {
		log_error("wfs_com: subap_height, subap_width, subap_rows and "
			"subap_cols must all be nonzero");
		return -1;
	}
	if ((data->init_y < 0) != (data->init_x < 0)) {
		log_error("Provide both init_y and init_x, or neither");
		return -1;
	}
	if (!data->search_radius
			|| data->search_radius > AYLP_WFS_COM_MAX_SEARCH_RADIUS) {
		log_error("wfs_com: search_radius must be in [1, %d]",
			AYLP_WFS_COM_MAX_SEARCH_RADIUS);
		return -1;
	}
	if (!(data->ref_beta > 0.0 && data->ref_beta <= 1.0)) {
		log_error("wfs_com: ref_beta must be in (0, 1]");
		return -1;
	}
	if (data->ref_beta_init > 0.0 && data->ref_beta_tau <= 0.0) {
		log_error("wfs_com: ref_beta_tau must be positive when "
			"ref_beta_init > 0");
		return -1;
	}
	if (data->ref_beta_init > 1.0) {
		log_error("wfs_com: ref_beta_init must be <= 1");
		return -1;
	}
	if (!data->reacquire_after) {
		log_error("wfs_com: reacquire_after must be nonzero");
		return -1;
	}

	data->n_subaps = data->subap_rows * data->subap_cols;
	data->ext_h = data->subap_h + 2*data->search_radius;
	data->ext_w = data->subap_w + 2*data->search_radius;
	data->win_h = data->subap_rows*data->subap_h + 2*data->search_radius;
	data->win_w = data->subap_cols*data->subap_w + 2*data->search_radius;
	if (!data->min_valid_subaps)
		data->min_valid_subaps = (data->n_subaps + 3) / 4;	// ~25%
	if (data->min_valid_subaps > data->n_subaps) {
		log_error("wfs_com: min_valid_subaps (%zu) exceeds the "
			"subaperture count (%zu)", data->min_valid_subaps,
			data->n_subaps);
		return -1;
	}

	data->ref = xcalloc(data->n_subaps * data->subap_h * data->subap_w,
		sizeof(double));
	data->ref_backup = xcalloc(
		data->n_subaps * data->subap_h * data->subap_w, sizeof(double));
	data->ref_set = xcalloc(data->n_subaps, sizeof(bool));
	data->ext = xcalloc(data->ext_h * data->ext_w, sizeof(double));
	data->row_sum_w = xcalloc(data->subap_rows, sizeof(double));
	data->row_sum_wdy = xcalloc(data->subap_rows, sizeof(double));
	data->row_sum_wdx = xcalloc(data->subap_rows, sizeof(double));
	data->com = xmalloc_type(gsl_vector, 2);

	self->type_in = AYLP_T_MATRIX_UCHAR;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_VECTOR;
	self->units_out = AYLP_U_MINMAX;
	self->proc = &wfs_com_proc;
	self->fini = &wfs_com_fini;

	log_info("wfs_com: %zux%zu subapertures of %zux%zu px, search radius "
		"%zu, window %zux%zu", data->subap_rows, data->subap_cols,
		data->subap_h, data->subap_w, data->search_radius,
		data->win_h, data->win_w);
	return 0;
}


int wfs_com_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_wfs_com_data *data = self->device_data;
	gsl_matrix_uchar *img = state->matrix_uchar;
	size_t max_y = img->size1, max_x = img->size2;

	if (UNLIKELY(data->win_h > max_y || data->win_w > max_x)) {
		log_error("wfs_com: window is %zux%zu but image is only "
			"%zux%zu", data->win_h, data->win_w, max_y, max_x);
		return -1;
	}
	if (UNLIKELY(!data->acquired)) reacquire(data, img);

	// Updates made while matching are speculative until the frame-level
	// validity and rolling-shear gates have seen every subaperture. Preserve
	// established templates so a rejected frame cannot teach the tracker the
	// very distortion it rejected. The acquisition warm-up must still seed.
	size_t ref_elems = data->n_subaps * data->subap_h * data->subap_w;
	bool refs_speculative = !data->just_acquired;
	if (refs_speculative)
		memcpy(data->ref_backup, data->ref, ref_elems * sizeof(double));

	// NLMS-style step schedule (see wfs_com.h): fast right after
	// (re)acquisition while templates are still settling, relaxing to
	// the steady-state ref_beta with time constant ref_beta_tau.
	double ref_beta_cur = data->ref_beta;
	if (data->ref_beta_init > 0.0) {
		double dt = wfs_monotonic_s() - data->t_acquire;
		ref_beta_cur = data->ref_beta
			+ (data->ref_beta_init - data->ref_beta)
				* exp(-dt / data->ref_beta_tau);
	}

	size_t org_y = data->win_y - data->win_h/2;
	size_t org_x = data->win_x - data->win_w/2;
	size_t R = data->search_radius;
	size_t grid = 2*R + 1;
	size_t sh = data->subap_h, sw = data->subap_w;

	double sum_w = 0.0, sum_wdy = 0.0, sum_wdx = 0.0;
	size_t valid_count = 0;
	memset(data->row_sum_w, 0, data->subap_rows * sizeof(double));
	memset(data->row_sum_wdy, 0, data->subap_rows * sizeof(double));
	memset(data->row_sum_wdx, 0, data->subap_rows * sizeof(double));

	for (size_t r = 0; r < data->subap_rows; r++) {
		for (size_t c = 0; c < data->subap_cols; c++) {
			size_t idx = subap_idx(data, r, c);
			double *ref = data->ref + idx*sh*sw;
			size_t ext_top = org_y + r*sh;
			size_t ext_left = org_x + c*sw;

			// extract + threshold the extended (margin-padded) block
			for (size_t i = 0; i < data->ext_h; i++) {
				for (size_t j = 0; j < data->ext_w; j++) {
					unsigned char raw = img->data[
						(ext_top+i)*img->tda + ext_left+j
					];
					unsigned char el = raw > data->threshold
						? raw - data->threshold : 0;
					data->ext[i*data->ext_w + j] = (double)el;
				}
			}
			// flux of the nominal (unshifted) core cell, used for
			// the flux gate and as an SNR weight
			double flux = 0.0;
			for (size_t i = 0; i < sh; i++)
				for (size_t j = 0; j < sw; j++)
					flux += data->ext[(R+i)*data->ext_w + (R+j)];

			if (UNLIKELY(!data->ref_set[idx])) {
				// seed the reference template from the core
				// cell; no shift estimate is possible yet
				if (flux >= data->flux_floor) {
					for (size_t i = 0; i < sh; i++)
						for (size_t j = 0; j < sw; j++)
							ref[i*sw+j] = data->ext[
								(R+i)*data->ext_w + (R+j)
							];
					data->ref_set[idx] = true;
				}
				continue;
			}
			if (flux < data->flux_floor) continue;	// gone dark

			// exhaustive integer-pixel normalized cross-correlation
			double ncc[AYLP_WFS_COM_MAX_GRID][AYLP_WFS_COM_MAX_GRID];
			double energy_r = 0.0;
			for (size_t i = 0; i < sh*sw; i++)
				energy_r += ref[i]*ref[i];
			double best = -2.0;
			size_t best_sy = R, best_sx = R;
			for (size_t sy = 0; sy < grid; sy++) {
				for (size_t sx = 0; sx < grid; sx++) {
					double corr = 0.0, energy_c = 0.0;
					for (size_t i = 0; i < sh; i++) {
						for (size_t j = 0; j < sw; j++) {
							double v = data->ext[
								(sy+i)*data->ext_w + sx+j
							];
							corr += v * ref[i*sw+j];
							energy_c += v*v;
						}
					}
					double n = corr
						/ sqrt(energy_c*energy_r + 1e-9);
					ncc[sy][sx] = n;
					if (n > best) {
						best = n; best_sy = sy; best_sx = sx;
					}
				}
			}
			double confidence = best < 0.0 ? 0.0 : best;
			if (confidence < data->min_confidence) continue;

			// Full 2D (non-separable) quadratic peak fit over the 3x3
			// neighborhood around the integer peak, skipped (left at
			// the integer estimate) if the peak sits on the edge of
			// the search range -- widen search_radius if that happens
			// often. Fitting y and x independently (the previous
			// approach: two separate 1D parabolic fits) ignores the
			// cross (row*col) term of the correlation surface, which
			// is a well-documented source of systematic "peak
			// locking" bias in correlation trackers. This fits
			// f(i,j) = A i^2 + B j^2 + C i j + D i + E j + F by
			// ordinary least squares over the 9 samples (closed form,
			// since the 3x3 regular grid's design matrix is fixed and
			// its Gram matrix inverts analytically -- see the derived
			// coefficient formulas below) and solves for the true
			// vertex of that surface, coupling term included.
			double dy_frac = 0.0, dx_frac = 0.0;
			if (best_sy > 0 && best_sy < grid-1
					&& best_sx > 0 && best_sx < grid-1) {
				double fm1m1 = ncc[best_sy-1][best_sx-1];
				double fm10  = ncc[best_sy-1][best_sx];
				double fm1p1 = ncc[best_sy-1][best_sx+1];
				double f0m1  = ncc[best_sy][best_sx-1];
				double f0p1  = ncc[best_sy][best_sx+1];
				double fp1m1 = ncc[best_sy+1][best_sx-1];
				double fp10  = ncc[best_sy+1][best_sx];
				double fp1p1 = ncc[best_sy+1][best_sx+1];

				double row_m1 = fm1m1+fm10+fm1p1;
				double row_p1 = fp1m1+fp10+fp1p1;
				double col_m1 = fm1m1+f0m1+fp1m1;
				double col_p1 = fm1p1+f0p1+fp1p1;
				double S0 = row_m1+row_p1 + (f0m1+ncc[best_sy][best_sx]+f0p1);
				double Sii = row_m1+row_p1;
				double Sjj = col_m1+col_p1;
				double Si = row_p1-row_m1;
				double Sj = col_p1-col_m1;
				double Sij = fp1p1-fp1m1-fm1p1+fm1m1;

				double F = (5*S0 - 3*Sii - 3*Sjj)/9.0;
				double A = (S0 - Sjj - 3*F)/2.0;
				double B = (S0 - Sii - 3*F)/2.0;
				double C = Sij/4.0;
				double D = Si/6.0;
				double E = Sj/6.0;

				// Guard: a genuine correlation peak fits a concave
				// (negative-definite) surface. A < 0 with a positive
				// determinant confirms that (and, since det=4AB-C^2,
				// implies B < 0 too); anything else is a degenerate
				// or saddle-shaped local fit, which happens near very
				// low-confidence/noisy matches -- fall back to the
				// integer estimate rather than solving a nonsense
				// vertex.
				double det = 4*A*B - C*C;
				if (A < 0.0 && det > 1e-12) {
					dy_frac = (-2*B*D + C*E) / det;
					dx_frac = (-2*A*E + C*D) / det;
					if (dy_frac < -1.0) dy_frac = -1.0;
					if (dy_frac > 1.0) dy_frac = 1.0;
					if (dx_frac < -1.0) dx_frac = -1.0;
					if (dx_frac > 1.0) dx_frac = 1.0;
				}
			}

			double dy = ((double)best_sy - (double)R) + dy_frac;
			double dx = ((double)best_sx - (double)R) + dx_frac;
			double weight = confidence * flux;
			sum_w += weight;
			sum_wdy += weight * dy;
			sum_wdx += weight * dx;
			data->row_sum_w[r] += weight;
			data->row_sum_wdy[r] += weight * dy;
			data->row_sum_wdx[r] += weight * dx;
			valid_count++;

			// Blend the SUB-PIXEL-aligned candidate into the
			// reference template (bilinear interpolation using the
			// same dy_frac/dx_frac as the shift estimate) -- frozen
			// for subapertures that didn't clear the confidence gate
			// above, so a distorted/bad match can't corrupt its own
			// template.
			//
			// Using only the integer-shift-aligned candidate here
			// (dropping dy_frac/dx_frac) leaves up to +-0.5 px of
			// residual sub-pixel misalignment in every blend. For a
			// target that drifts continuously by less than a pixel
			// per frame -- exactly the vibration/pointing case this
			// device targets -- that residual is never corrected by
			// a whole-pixel best_sy/best_sx move, so the EWMA slowly
			// reshapes the template to match the drifting target
			// instead of ever reporting the drift as a shift: the
			// tracker silently locks up. Interpolating to the full
			// sub-pixel estimate before blending removes that
			// systematic leak. The +-1 fallback offsets are safe
			// because dy_frac/dx_frac are only ever nonzero when
			// best_sy/best_sx are strictly interior to the search
			// grid (see the refinement guard above), so best_sy+-1
			// and best_sx+-1 stay in bounds.
			int oy = (dy_frac >= 0.0) ? 1 : -1;
			int ox = (dx_frac >= 0.0) ? 1 : -1;
			double wy = fabs(dy_frac), wx = fabs(dx_frac);
			for (size_t i = 0; i < sh; i++) {
				size_t y0 = best_sy + i;
				size_t y1 = wy > 0.0 ? (size_t)((long)y0 + oy) : y0;
				for (size_t j = 0; j < sw; j++) {
					size_t x0 = best_sx + j;
					size_t x1 = wx > 0.0
						? (size_t)((long)x0 + ox) : x0;
					double v00 = data->ext[y0*data->ext_w + x0];
					double v01 = data->ext[y0*data->ext_w + x1];
					double v10 = data->ext[y1*data->ext_w + x0];
					double v11 = data->ext[y1*data->ext_w + x1];
					double v = v00*(1-wy)*(1-wx) + v01*(1-wy)*wx
						+ v10*wy*(1-wx) + v11*wy*wx;
					ref[i*sw+j] = (1.0-ref_beta_cur)*ref[i*sw+j]
						+ ref_beta_cur*v;
				}
			}
		}
	}

	// Per-row-group weighted-mean shift, computed regardless of row_time:
	// used both for the shear-rejection gate below and, when enabled,
	// as the rolling-shutter regression's data points. If row-groups
	// disagree by more than max_row_shear pixels in y or x, subapertures
	// on different rows genuinely saw the target at different positions
	// within this one frame's readout -- exactly the rolling-shutter
	// signature -- and the whole frame is rejected below rather than
	// blended into one (distorted) global shift.
	double row_y_min = 0.0, row_y_max = 0.0, row_x_min = 0.0, row_x_max = 0.0;
	bool shear_rejected_frame = false;
	if (data->max_row_shear > 0.0) {
		size_t n_rows_seen = 0;
		for (size_t r = 0; r < data->subap_rows; r++) {
			double w = data->row_sum_w[r];
			if (w <= 0.0) continue;
			double y_r = data->row_sum_wdy[r] / w;
			double x_r = data->row_sum_wdx[r] / w;
			if (!n_rows_seen || y_r < row_y_min) row_y_min = y_r;
			if (!n_rows_seen || y_r > row_y_max) row_y_max = y_r;
			if (!n_rows_seen || x_r < row_x_min) row_x_min = x_r;
			if (!n_rows_seen || x_r > row_x_max) row_x_max = x_r;
			n_rows_seen++;
		}
		shear_rejected_frame = n_rows_seen >= 2
			&& (row_y_max - row_y_min > data->max_row_shear
				|| row_x_max - row_x_min > data->max_row_shear);
der.units = self->units_out;
	state->header	}

	if (UNLIKELY(data->just_acquired)) {
		// warm-up frame: every subaperture just seeded its template
		// and has nothing to report yet. Not a lost signal.
		data->just_acquired = false;
		state->header.status &= (aylp_status)~AYLP_FRAME_REJECTED;
	} else if (valid_count < data->min_valid_subaps || sum_w <= 0.0) {
		// Too few confident subapertures this frame -- e.g. a chop-off
		// frame, a transient occlusion, or genuine signal loss. Hold
		// the last valid output rather than reporting a fake position,
		// same rationale as anyloop:center_of_mass.
		state->header.status &= (aylp_status)~AYLP_FRAME_REJECTED;
		if (refs_speculative)
			memcpy(data->ref, data->ref_backup, ref_elems * sizeof(double));
		if (++data->lost >= data->reacquire_after) {
			state->header.status |= AYLP_BEAM_LOST;
			if (data->lost == data->reacquire_after) {
				log_warn("wfs_com: fewer than %zu confident "
					"subapertures for %zu frames; re-acquiring",
					data->min_valid_subaps, data->lost);
			}
			reacquire(data, img);
		}
	} else if (shear_rejected_frame) {
		// Rolling-shutter shear rejection: hold the last valid output
		// instead of reporting a shear-distorted position. Deliberately
		// NOT counted toward data->lost/reacquire_after -- see
		// max_row_shear's comment in wfs_com.h: the geometry hasn't
		// changed, only this frame's readout timing, so a sustained
		// rolling-shutter condition (e.g. continuous vibration) must not
		// force a repeated window/template reset.
		//
		// Reaching this branch means the frame PASSED the
		// min_valid_subaps gate above, i.e. the beam was demonstrably
		// present and well-matched -- so the consecutive-loss counter
		// resets here exactly as it does on a normally tracked frame.
		// Merely leaving it frozen would let low-signal frames
		// interleaved with shear-rejected ones accumulate toward
		// reacquire_after across frames where the beam was repeatedly
		// seen just fine, eventually firing a spurious reacquire.
		data->lost = 0;
		if (refs_speculative)
			memcpy(data->ref, data->ref_backup, ref_elems * sizeof(double));
		data->shear_rejected++;
		state->header.status |= AYLP_FRAME_REJECTED;
		log_debug("wfs_com: rejected frame for rolling-shutter shear "
			"(row spread y=%.2f x=%.2f px > max_row_shear %.2f px; "
			"%zu total)", row_y_max - row_y_min, row_x_max - row_x_min,
			data->max_row_shear, data->shear_rejected);
	} else {
		state->header.status &= (aylp_status)~AYLP_FRAME_REJECTED;
		double global_dy = sum_wdy / sum_w;
		double global_dx = sum_wdx / sum_w;
		if (data->rolling_shutter) {
			// Rolling-shutter correction: rather than blending every
			// subaperture's shift together regardless of row (which
			// mixes measurements taken at genuinely different instants
			// into one smeared number), first collapse to one
			// (weighted) shift per row group, then fit a line against
			// each row group's own capture time and report the
			// position at a fixed reference time (the window's
			// vertical centre) instead of at the blended, ill-defined
			// "average" instant. Reduces to the plain weighted average
			// above whenever fewer than 2 row groups have data
			// (nothing to fit a line through).
			double sw_t = 0.0, swt = 0.0, swy = 0.0, swx = 0.0;
			size_t n_rows_seen = 0;
			for (size_t r = 0; r < data->subap_rows; r++) {
				double w = data->row_sum_w[r];
				if (w <= 0.0) continue;
				double t = (double)r*sh + (double)sh/2.0;
				sw_t += w; swt += w*t;
				swy += data->row_sum_wdy[r];	// already weight*dy
				swx += data->row_sum_wdx[r];	// already weight*dx
				n_rows_seen++;
			}
			if (n_rows_seen >= 2 && sw_t > 0.0) {
				double t_mean = swt / sw_t;
				double y_mean = swy / sw_t;
				double x_mean = swx / sw_t;
				double s_tt = 0.0, s_ty = 0.0, s_tx = 0.0;
				for (size_t r = 0; r < data->subap_rows; r++) {
					double w = data->row_sum_w[r];
					if (w <= 0.0) continue;
					double t = (double)r*sh + (double)sh/2.0;
					double dt = t - t_mean;
					double y_r = data->row_sum_wdy[r] / w;
					double x_r = data->row_sum_wdx[r] / w;
					s_tt += w * dt * dt;
					s_ty += w * dt * (y_r - y_mean);
					s_tx += w * dt * (x_r - x_mean);
				}
				double t_ref = (double)(data->subap_rows*sh) / 2.0;
				if (s_tt > 1e-12) {
					double b_y = s_ty / s_tt;
					double b_x = s_tx / s_tt;
					global_dy = y_mean + b_y*(t_ref - t_mean);
					global_dx = x_mean + b_x*(t_ref - t_mean);
				} else {
					global_dy = y_mean;
					global_dx = x_mean;
				}
			}
		}
		data->abs_y += global_dy;
		data->abs_x += global_dx;
		// Derive the next frame's integer window placement from the
		// sub-pixel position. Only snap abs_y/abs_x back to that
		// integer if clamping near an image edge actually had to move
		// it -- otherwise leave abs_y/abs_x at full sub-pixel
		// precision; rounding it to the window's integer centre every
		// frame would throw away exactly the accuracy the parabolic
		// refinement above exists to produce.
		// Guard against casting a negative double to size_t (which
		// would wrap around to a huge value and clamp to the WRONG
		// edge below) if a still-noisy weighted average ever pushes
		// the position outside the image before clamp_window() gets
		// a chance to pull it back in.
		if (data->abs_y < 0.0) data->abs_y = 0.0;
		if (data->abs_y > (double)(max_y - 1)) data->abs_y = (double)(max_y - 1);
		if (data->abs_x < 0.0) data->abs_x = 0.0;
		if (data->abs_x > (double)(max_x - 1)) data->abs_x = (double)(max_x - 1);
		size_t new_win_y = (size_t)llround(data->abs_y);
		size_t new_win_x = (size_t)llround(data->abs_x);
		data->win_y = new_win_y;
		data->win_x = new_win_x;
		clamp_window(data, max_y, max_x);
		if (data->win_y != new_win_y) data->abs_y = (double)data->win_y;
		if (data->win_x != new_win_x) data->abs_x = (double)data->win_x;

		data->last_y = -1.0 + 2.0*data->abs_y/((double)max_y - 1.0);
		data->last_x = -1.0 + 2.0*data->abs_x/((double)max_x - 1.0);
		data->lost = 0;
		state->header.status &= (aylp_status)~AYLP_BEAM_LOST;
	}

	data->com->data[0] = data->last_y;
	data->com->data[1] = data->last_x;
	state->vector = data->com;
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = 2;
	state->header.log_dim.x = 1;
	return 0;der.units = self->units_out;
	state->header
}


int wfs_com_fini(struct aylp_device *self)
{
	struct aylp_wfs_com_data *data = self->device_data;
	xfree(data->ref);
	xfree(data->ref_backup);
	xfree(data->ref_set);
	xfree(data->ext);
	xfree(data->row_sum_w);
	xfree(data->row_sum_wdy);
	xfree(data->row_sum_wdx);
	if (data->com) xfree_type(gsl_vector, data->com);
	xfree(self->device_data);
	return 0;
}
