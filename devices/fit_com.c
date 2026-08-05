#include <math.h>
#include <string.h>

#include "anyloop.h"
#include "logging.h"
#include "fit_com.h"
#include "xalloc.h"

#define NP AYLP_FIT_COM_NP


// Slide the window so it lies fully inside the image.
static void clamp_window(struct aylp_fit_com_data *data,
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


// Cost (weighted sum of squared residuals) of parameter vector p, and
// optionally the Gauss-Newton normal equations for it.
//
// The residual is model-minus-observation, so the step solves
// (JtJ + lambda diag) d = -Jtr.
static double fit_eval(struct aylp_fit_com_data *data, gsl_matrix_uchar *img,
	size_t org_y, size_t org_x, const double *p,
	double *JtJ, double *Jtr, bool need_jac, double *resid_out
) {
	double sig = p[AYLP_FIT_P_SIGMA];
	double amp = p[AYLP_FIT_P_AMP];
	double bg = p[AYLP_FIT_P_BG];
	double sig2 = sig*sig;
	double inv2s2 = 1.0/(2.0*sig2);
	// exp(-(d+1)^2/2s^2) = exp(-d^2/2s^2) * exp(-(2d+1)/2s^2), and the
	// second factor itself advances by a constant ratio. So a gaussian
	// sampled on a unit grid costs two multiplies per pixel instead of a
	// transcendental -- which is the whole inner loop here. Walking OUTWARD
	// from the pixel nearest the centre keeps every ratio <= 1, so nothing
	// can overflow the way a left-to-right sweep would for a narrow sigma.
	double cstep = exp(-2.0*inv2s2);
	double cost = 0.0;
	if (need_jac) {
		memset(JtJ, 0, NP*NP*sizeof(double));
		memset(Jtr, 0, NP*sizeof(double));
	}
	for (size_t i = 0; i < data->win_h; i++) {
		size_t row = org_y + i;
		double t = (double)row - data->ref_row;
		double yc = p[AYLP_FIT_P_Y0] + p[AYLP_FIT_P_SY]*t;
		double xc = p[AYLP_FIT_P_X0] + p[AYLP_FIT_P_SX]*t;
		double dy = (double)row - yc;
		double ey = exp(-dy*dy*inv2s2);
		double *rrow = resid_out ? resid_out + i*data->win_w : 0;
		const unsigned char *irow = img->data + row*img->tda + org_x;
		const double *wrow = data->weights + i*data->win_w;

		// index of the sample nearest the centre, clamped into the row
		double jc_f = xc - (double)org_x;
		long jc = (long)llround(jc_f);
		if (jc < 0) jc = 0;
		if (jc >= (long)data->win_w) jc = (long)data->win_w - 1;
		double dc = (double)(org_x + (size_t)jc) - xc;
		double gc = exp(-dc*dc*inv2s2);

		for (int dir = 0; dir < 2; dir++) {
			// dir 0 walks right from jc, dir 1 walks left from jc-1
			long j = dir ? jc-1 : jc;
			if (dir && j < 0) continue;
			if (!dir && j >= (long)data->win_w) continue;
			double g, ratio, dx;
			if (!dir) {
				g = gc; dx = dc;
				ratio = exp(-(2.0*dc + 1.0)*inv2s2);
			} else {
				dx = dc - 1.0;
				ratio = exp((2.0*dc - 1.0)*inv2s2);
				g = gc*ratio;
				ratio *= cstep;
			}
			for (; dir ? (j >= 0) : (j < (long)data->win_w);
					j += dir ? -1 : 1) {
				double q = ey*g;
				double r2 = dy*dy + dx*dx;
				double model = bg + amp*q;
				double res = model - (double)irow[j];
				double w = wrow[j];
				cost += w*res*res;
				if (rrow) rrow[j] = res;
				if (need_jac) {
					double aq = amp*q;
					double jac[NP];
					jac[AYLP_FIT_P_Y0] = aq*dy/sig2;
					jac[AYLP_FIT_P_X0] = aq*dx/sig2;
					jac[AYLP_FIT_P_SY] = jac[AYLP_FIT_P_Y0]*t;
					jac[AYLP_FIT_P_SX] = jac[AYLP_FIT_P_X0]*t;
					jac[AYLP_FIT_P_SIGMA] = aq*r2/(sig2*sig);
					jac[AYLP_FIT_P_AMP] = q;
					jac[AYLP_FIT_P_BG] = 1.0;
					for (size_t a = 0; a < NP; a++) {
						if (!data->active[a]) continue;
						Jtr[a] += w*jac[a]*res;
						for (size_t b = a; b < NP; b++) {
							if (!data->active[b]) continue;
							JtJ[a*NP+b] += w*jac[a]*jac[b];
						}
					}
				}
				// advance the recurrence one sample outward
				g *= ratio;
				ratio *= cstep;
				dx += dir ? -1.0 : 1.0;
			}
		}
	}
	if (need_jac) {	// mirror the upper triangle
		for (size_t a = 0; a < NP; a++)
			for (size_t b = a+1; b < NP; b++)
				JtJ[b*NP+a] = JtJ[a*NP+b];
	}
	return cost;
}


// Cholesky solve of the (symmetric positive definite) damped normal
// equations, in place on a copy. Returns false if not positive definite,
// which the caller answers by raising the damping.
static bool solve_step(const double *JtJ, const double *Jtr,
	const bool *active, double lambda, double *step
) {
	double A[NP*NP];
	memcpy(A, JtJ, sizeof(A));
	for (size_t a = 0; a < NP; a++) {
		if (active[a]) {
			// Marquardt scaling: damp by the diagonal, not by the
			// identity, so parameters with wildly different natural
			// scales (a background in counts against a slope in
			// px/row) are damped comparably.
			double d = A[a*NP+a];
			A[a*NP+a] = d + lambda*(d > 0.0 ? d : 1.0);
		} else {
			// Frozen parameter: unit row/column, zero rhs => zero step.
			for (size_t b = 0; b < NP; b++) A[a*NP+b] = A[b*NP+a] = 0.0;
			A[a*NP+a] = 1.0;
		}
	}
	double L[NP*NP];
	memset(L, 0, sizeof(L));
	for (size_t a = 0; a < NP; a++) {
		for (size_t b = 0; b <= a; b++) {
			double s = A[a*NP+b];
			for (size_t k = 0; k < b; k++) s -= L[a*NP+k]*L[b*NP+k];
			if (a == b) {
				if (s <= 1e-300) return false;
				L[a*NP+a] = sqrt(s);
			} else {
				L[a*NP+b] = s/L[b*NP+b];
			}
		}
	}
	double z[NP];
	for (size_t a = 0; a < NP; a++) {
		double s = active[a] ? -Jtr[a] : 0.0;
		for (size_t k = 0; k < a; k++) s -= L[a*NP+k]*z[k];
		z[a] = s/L[a*NP+a];
	}
	for (size_t a = NP; a-- > 0; ) {
		double s = z[a];
		for (size_t k = a+1; k < NP; k++) s -= L[k*NP+a]*step[k];
		step[a] = s/L[a*NP+a];
	}
	return true;
}


static void clamp_params(struct aylp_fit_com_data *data, double *p,
	size_t max_y, size_t max_x
) {
	if (!(p[AYLP_FIT_P_SIGMA] >= data->sigma_min))
		p[AYLP_FIT_P_SIGMA] = data->sigma_min;
	if (p[AYLP_FIT_P_SIGMA] > data->sigma_max)
		p[AYLP_FIT_P_SIGMA] = data->sigma_max;
	if (!(p[AYLP_FIT_P_AMP] >= 0.0)) p[AYLP_FIT_P_AMP] = 0.0;
	if (!(p[AYLP_FIT_P_BG] >= 0.0)) p[AYLP_FIT_P_BG] = 0.0;
	if (p[AYLP_FIT_P_BG] > 255.0) p[AYLP_FIT_P_BG] = 255.0;
	if (!(p[AYLP_FIT_P_Y0] >= 0.0)) p[AYLP_FIT_P_Y0] = 0.0;
	if (p[AYLP_FIT_P_Y0] > (double)(max_y-1)) p[AYLP_FIT_P_Y0] = (double)(max_y-1);
	if (!(p[AYLP_FIT_P_X0] >= 0.0)) p[AYLP_FIT_P_X0] = 0.0;
	if (p[AYLP_FIT_P_X0] > (double)(max_x-1)) p[AYLP_FIT_P_X0] = (double)(max_x-1);
	// A slope steep enough to smear the beam across the whole window in one
	// readout is not a measurement, it is a diverging fit.
	double smax = (double)max_y;
	for (int k = AYLP_FIT_P_SY; k <= AYLP_FIT_P_SX; k++) {
		if (!(p[k] > -smax)) p[k] = -smax;
		if (p[k] > smax) p[k] = smax;
	}
}


// Seed the parameter vector from the image itself: brightest pixel (or the
// configured init point) for position, window border for background, and the
// flux-weighted second moment for sigma. Slopes start at zero.
static void acquire(struct aylp_fit_com_data *data, gsl_matrix_uchar *img)
{
	size_t max_y = img->size1, max_x = img->size2;
	if (data->init_y >= 0) {
		data->win_y = (size_t)data->init_y;
		data->win_x = (size_t)data->init_x;
	} else {
		unsigned char best = 0;
		size_t by = max_y/2, bx = max_x/2;
		for (size_t i = 0; i < max_y; i++) {
			for (size_t j = 0; j < max_x; j++) {
				unsigned char v = img->data[i*img->tda + j];
				if (v > best) { best = v; by = i; bx = j; }
			}
		}
		data->win_y = by; data->win_x = bx;
	}
	clamp_window(data, max_y, max_x);
	size_t org_y = data->win_y - data->win_h/2;
	size_t org_x = data->win_x - data->win_w/2;

	// background from the window border, peak from the interior
	double bsum = 0.0; size_t bn = 0;
	unsigned char peak = 0; size_t py = data->win_y, px = data->win_x;
	for (size_t i = 0; i < data->win_h; i++) {
		for (size_t j = 0; j < data->win_w; j++) {
			unsigned char v = img->data[(org_y+i)*img->tda + org_x+j];
			if (i == 0 || j == 0 || i == data->win_h-1
					|| j == data->win_w-1) {
				bsum += v; bn++;
			}
			if (v > peak) { peak = v; py = org_y+i; px = org_x+j; }
		}
	}
	double bg = bn ? bsum/(double)bn : 0.0;
	if (data->init_y >= 0) { py = (size_t)data->init_y; px = (size_t)data->init_x; }

	// sigma from the radial second moment: for a 2D gaussian,
	// <r^2> = 2 sigma^2 over the flux above background.
	// Taken LOCALLY, over a box a few sigma_init wide around the peak. Over
	// the whole window a stray reflection inflates it badly (its distance
	// enters as r^2), the initial model is then broad enough to span both
	// blobs, and the fit settles between them -- at which point residuals
	// are large at BOTH and no amount of robust reweighting can tell which
	// one is the beam. Seeding narrow and on the peak keeps the stray
	// firmly in the tail, where the reweighting can discard it.
	double rad = 3.0*data->sigma_init;
	double sw = 0.0, sr2 = 0.0;
	for (size_t i = 0; i < data->win_h; i++) {
		double dy = (double)(org_y+i) - (double)py;
		if (fabs(dy) > rad) continue;
		for (size_t j = 0; j < data->win_w; j++) {
			double dx = (double)(org_x+j) - (double)px;
			if (fabs(dx) > rad) continue;
			double v = (double)img->data[(org_y+i)*img->tda + org_x+j] - bg;
			if (v <= 0.0) continue;
			sw += v; sr2 += v*(dy*dy + dx*dx);
		}
	}
	double sigma = sw > 0.0 ? sqrt(sr2/sw/2.0) : data->sigma_init;
	if (!(sigma >= data->sigma_min)) sigma = data->sigma_init;
	if (sigma > data->sigma_max) sigma = data->sigma_max;

	data->p[AYLP_FIT_P_Y0] = (double)py;
	data->p[AYLP_FIT_P_X0] = (double)px;
	data->p[AYLP_FIT_P_SY] = 0.0;
	data->p[AYLP_FIT_P_SX] = 0.0;
	data->p[AYLP_FIT_P_SIGMA] = sigma;
	data->p[AYLP_FIT_P_AMP] = (double)peak - bg > 0.0 ? (double)peak - bg : 1.0;
	data->p[AYLP_FIT_P_BG] = bg;
	data->lambda = 1e-3;
	data->acquired = true;
	data->lost = 0;
	log_info("fit_com: acquired at (%.2f,%.2f), sigma %.2f, amp %.1f, bg %.1f",
		data->p[AYLP_FIT_P_Y0], data->p[AYLP_FIT_P_X0], sigma,
		data->p[AYLP_FIT_P_AMP], bg);
}


int fit_com_init(struct aylp_device *self)
{
	self->device_data = xcalloc(1, sizeof(struct aylp_fit_com_data));
	struct aylp_fit_com_data *data = self->device_data;
	self->fini = &fit_com_fini;

	// defaults
	data->window_h = 0;		// 0 => whole image
	data->window_w = 0;
	data->init_y = -1;
	data->init_x = -1;
	data->max_iter = 10;
	data->tol = 1e-4;
	data->robust_k = 2.5;
	data->robust_iter = 1;
	data->sigma_init = 2.0;
	data->sigma_min = 0.5;
	data->sigma_max = 20.0;
	data->min_amplitude = 5.0;
	data->max_residual = 0.0;	// 0 => disabled
	data->reacquire_after = 10;
	data->row_time = 0.0;
	bool fit_slope = true;

	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
		} else if (!strcmp(key, "window_height")) {
			data->window_h = json_object_get_uint64(val);
		} else if (!strcmp(key, "window_width")) {
			data->window_w = json_object_get_uint64(val);
		} else if (!strcmp(key, "init_y")) {
			data->init_y = json_object_get_int64(val);
		} else if (!strcmp(key, "init_x")) {
			data->init_x = json_object_get_int64(val);
		} else if (!strcmp(key, "max_iter")) {
			data->max_iter = json_object_get_uint64(val);
		} else if (!strcmp(key, "tol")) {
			data->tol = json_object_get_double(val);
		} else if (!strcmp(key, "robust_k")) {
			data->robust_k = json_object_get_double(val);
		} else if (!strcmp(key, "robust_iter")) {
			data->robust_iter = json_object_get_uint64(val);
		} else if (!strcmp(key, "sigma_init")) {
			data->sigma_init = json_object_get_double(val);
		} else if (!strcmp(key, "sigma_min")) {
			data->sigma_min = json_object_get_double(val);
		} else if (!strcmp(key, "sigma_max")) {
			data->sigma_max = json_object_get_double(val);
		} else if (!strcmp(key, "min_amplitude")) {
			data->min_amplitude = json_object_get_double(val);
		} else if (!strcmp(key, "max_residual")) {
			data->max_residual = json_object_get_double(val);
		} else if (!strcmp(key, "reacquire_after")) {
			data->reacquire_after = json_object_get_uint64(val);
		} else if (!strcmp(key, "row_time")) {
			data->row_time = json_object_get_double(val);
		} else if (!strcmp(key, "fit_slope")) {
			fit_slope = json_object_get_boolean(val);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	if ((data->init_y < 0) != (data->init_x < 0)) {
		log_error("Provide both init_y and init_x, or neither");
		return -1;
	}
	if (!data->max_iter) {
		log_error("fit_com: max_iter must be nonzero");
		return -1;
	}
	if (!isfinite(data->tol) || data->tol < 0.0) {
		log_error("fit_com: tol must be finite and non-negative");
		return -1;
	}
	if (!isfinite(data->robust_k) || data->robust_k < 0.0) {
		log_error("fit_com: robust_k must be finite and non-negative");
		return -1;
	}
	if (!isfinite(data->sigma_min) || !isfinite(data->sigma_max)
			|| data->sigma_min <= 0.0
			|| data->sigma_max <= data->sigma_min) {
		log_error("fit_com: need 0 < sigma_min < sigma_max");
		return -1;
	}
	if (!isfinite(data->sigma_init) || data->sigma_init <= 0.0) {
		log_error("fit_com: sigma_init must be positive");
		return -1;
	}
	if (!isfinite(data->min_amplitude) || data->min_amplitude < 0.0) {
		log_error("fit_com: min_amplitude must be finite, non-negative");
		return -1;
	}
	if (!isfinite(data->max_residual) || data->max_residual < 0.0) {
		log_error("fit_com: max_residual must be finite, non-negative");
		return -1;
	}
	if (!isfinite(data->row_time) || data->row_time < 0.0) {
		log_error("fit_com: row_time must be finite and non-negative");
		return -1;
	}
	if (!data->reacquire_after) {
		log_error("fit_com: reacquire_after must be nonzero");
		return -1;
	}
	if (data->window_h && data->window_h < 3) {
		log_error("fit_com: window_height must be at least 3");
		return -1;
	}
	if (data->window_w && data->window_w < 3) {
		log_error("fit_com: window_width must be at least 3");
		return -1;
	}

	for (size_t a = 0; a < NP; a++) data->active[a] = true;
	data->active[AYLP_FIT_P_SY] = fit_slope;
	data->active[AYLP_FIT_P_SX] = fit_slope;
	data->lambda = 1e-3;

	self->type_in = AYLP_T_MATRIX_UCHAR;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_VECTOR;
	self->units_out = AYLP_U_MINMAX;
	self->proc = &fit_com_proc;
	data->com = xmalloc_type(gsl_vector, 2);

	log_info("fit_com: %s model, window %zux%zu (0 = whole frame), "
		"max_iter %zu", fit_slope ? "sheared (row-dependent centre)"
		: "static", data->window_h, data->window_w, data->max_iter);
	return 0;
}


int fit_com_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_fit_com_data *data = self->device_data;
	gsl_matrix_uchar *img = state->matrix_uchar;
	if (UNLIKELY(!img || !img->data || img->size1 < 3 || img->size2 < 3
			|| img->tda < img->size2)) {
		log_error("fit_com: invalid image matrix or dimensions");
		return -1;
	}
	size_t max_y = img->size1, max_x = img->size2;

	// (re)size the window and scratch to this image
	size_t want_h = data->window_h ? data->window_h : max_y;
	size_t want_w = data->window_w ? data->window_w : max_x;
	if (want_h > max_y) want_h = max_y;
	if (want_w > max_x) want_w = max_x;
	if (UNLIKELY(want_h != data->win_h || want_w != data->win_w)) {
		data->win_h = want_h;
		data->win_w = want_w;
		xfree(data->weights);
		xfree(data->resid);
		xfree(data->resid_alt);
		data->weights = xcalloc(data->win_h*data->win_w, sizeof(double));
		data->resid = xcalloc(data->win_h*data->win_w, sizeof(double));
		data->resid_alt = xcalloc(data->win_h*data->win_w, sizeof(double));
		data->acquired = false;
	}
	// A fixed epoch in IMAGE coordinates, so the instant the reported
	// position belongs to does not wander as the window tracks the beam --
	// a moving epoch reads downstream as a wobbling loop delay.
	data->ref_row = 0.5*((double)max_y - 1.0);

	if (UNLIKELY(!data->acquired)) acquire(data, img);
	clamp_window(data, max_y, max_x);
	size_t org_y = data->win_y - data->win_h/2;
	size_t org_x = data->win_x - data->win_w/2;
	size_t npix = data->win_h * data->win_w;

	// Warm start from the previous frame: between frames the beam moves far
	// less than its own width, so the fit begins inside the basin and
	// converges in a couple of iterations.
	double p[NP];
	memcpy(p, data->p, sizeof(p));
	for (size_t k = 0; k < npix; k++) data->weights[k] = 1.0;

	double JtJ[NP*NP], Jtr[NP], step[NP], trial[NP];
	double *resid = data->resid, *resid_alt = data->resid_alt;
	double cost = fit_eval(data, img, org_y, org_x, p, 0, 0, false, resid);
	double lambda = data->lambda;
	size_t it = 0;
	bool converged = false;
	for (; it < data->max_iter; it++) {
		// IRLS: after a few plain iterations, downweight pixels the model
		// cannot explain -- a stray reflection, a hot pixel, clipping.
		// Crucially the model already contains the motion, so a sheared
		// beam fits and is NOT treated as an outlier.
		if (data->robust_k > 0.0 && it >= data->robust_iter) {
			double ss = 0.0;
			for (size_t k = 0; k < npix; k++)
				ss += resid[k]*resid[k];
			double scale = sqrt(ss/(double)npix);
			if (scale > 1e-9) {
				// Tukey biweight, not Huber. Huber only tapers as
				// cut/|r| and never reaches zero, so a bright
				// stray reflection -- which is not a sparse
				// outlier but hundreds of pixels of coherent
				// support -- keeps enough leverage to drag the
				// centre several px. A redescending weight
				// excludes it outright once the fit is seated on
				// the real beam, which acquisition guarantees by
				// starting from the brightest pixel.
				double cut = data->robust_k*scale;
				size_t kept = 0;
				for (size_t k = 0; k < npix; k++) {
					double u = resid[k]/cut;
					if (fabs(u) >= 1.0) { data->weights[k] = 0.0; continue; }
					double m = 1.0 - u*u;
					data->weights[k] = m*m;
					kept++;
				}
				// Never let reweighting starve the fit of data.
				if (kept < 4*AYLP_FIT_COM_NP)
					for (size_t k = 0; k < npix; k++)
						data->weights[k] = 1.0;
			}
		}
		// One pass: cost at the current point under the current weights,
		// the normal equations, and the residuals -- all from the same
		// walk over the window rather than three separate ones.
		cost = fit_eval(data, img, org_y, org_x, p, JtJ, Jtr, true, resid);
		bool stepped = false;
		for (int attempt = 0; attempt < 8; attempt++) {
			if (!solve_step(JtJ, Jtr, data->active, lambda, step)) {
				lambda *= 10.0;
				continue;
			}
			for (size_t a = 0; a < NP; a++)
				trial[a] = p[a] + (data->active[a] ? step[a] : 0.0);
			clamp_params(data, trial, max_y, max_x);
			// Trial residuals go to the spare buffer, so an accepted
			// step just swaps pointers instead of costing another
			// full pass to re-derive what it already computed.
			double tc = fit_eval(data, img, org_y, org_x, trial,
				0, 0, false, resid_alt);
			if (tc < cost) {
				double rel = (cost - tc)/(cost > 0.0 ? cost : 1.0);
				memcpy(p, trial, sizeof(p));
				double *sw = resid; resid = resid_alt; resid_alt = sw;
				cost = tc;
				lambda = lambda > 1e-8 ? lambda*0.3 : lambda;
				stepped = true;
				if (rel < data->tol) converged = true;
				break;
			}
			lambda *= 10.0;
			if (lambda > 1e12) break;
		}
		if (!stepped || converged) break;
	}
	data->resid = resid;
	data->resid_alt = resid_alt;
	data->lambda = lambda < 1e-8 ? 1e-8 : (lambda > 1.0 ? 1.0 : lambda);
	data->n_iter_last = it + 1;

	// unweighted rms residual, the honest goodness-of-fit
	double ss = 0.0;
	for (size_t k = 0; k < npix; k++) ss += resid[k]*resid[k];
	double rms = sqrt(ss/(double)npix);

	bool ok = isfinite(p[AYLP_FIT_P_Y0]) && isfinite(p[AYLP_FIT_P_X0])
		&& p[AYLP_FIT_P_AMP] >= data->min_amplitude
		&& p[AYLP_FIT_P_SIGMA] > data->sigma_min
		&& p[AYLP_FIT_P_SIGMA] < data->sigma_max
		&& (data->max_residual <= 0.0 || rms <= data->max_residual);

	if (LIKELY(ok)) {
		memcpy(data->p, p, sizeof(p));
		data->last_rms = rms;
		data->vel_y = data->row_time > 0.0
			? p[AYLP_FIT_P_SY]/data->row_time : 0.0;
		data->vel_x = data->row_time > 0.0
			? p[AYLP_FIT_P_SX]/data->row_time : 0.0;
		data->last_y = -1.0 + 2.0*p[AYLP_FIT_P_Y0]/((double)max_y - 1.0);
		data->last_x = -1.0 + 2.0*p[AYLP_FIT_P_X0]/((double)max_x - 1.0);
		data->win_y = (size_t)llround(p[AYLP_FIT_P_Y0]);
		data->win_x = (size_t)llround(p[AYLP_FIT_P_X0]);
		clamp_window(data, max_y, max_x);
		data->lost = 0;
		state->header.status &= (aylp_status)~AYLP_FRAME_REJECTED;
		state->header.status &= (aylp_status)~AYLP_BEAM_LOST;
	} else {
		// The model could not describe this frame. Hold the last valid
		// output rather than publish a diverged fit, and say so.
		state->header.status |= AYLP_FRAME_REJECTED;
		if (++data->lost >= data->reacquire_after) {
			state->header.status |= AYLP_BEAM_LOST;
			if (data->lost == data->reacquire_after) {
				log_warn("fit_com: fit failed for %zu frames "
					"(amp %.1f < %.1f? sigma %.2f, rms %.1f); "
					"re-acquiring", data->lost,
					p[AYLP_FIT_P_AMP], data->min_amplitude,
					p[AYLP_FIT_P_SIGMA], rms);
			}
			data->acquired = false;
		}
	}

	data->com->data[0] = data->last_y;
	data->com->data[1] = data->last_x;
	state->vector = data->com;
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = 2;
	state->header.log_dim.x = 1;
	return 0;
}


int fit_com_fini(struct aylp_device *self)
{
	struct aylp_fit_com_data *data = self->device_data;
	if (data) {
		xfree(data->weights);
		xfree(data->resid);
		xfree(data->resid_alt);
		if (data->com) xfree_type(gsl_vector, data->com);
	}
	xfree(self->device_data);
	return 0;
}
