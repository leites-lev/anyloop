#include <time.h>
#include <math.h>
#include <string.h>
#include <gsl/gsl_vector.h>

#include "anyloop.h"
#include "logging.h"
#include "xalloc.h"
#include "fsp.h"


// ---- small dense linear algebra on dim <= AYLP_FSP_MAX_DIM ------------------
// Everything here runs at init and once per adaptation tick, never in the hot
// per-sample path (which uses the O(n_modes) AR recursion directly), so plain
// triple loops on a <=16x16 matrix are fine.

// build the block-diagonal AR(2) state matrix A (dim x dim) from the per-mode
// coefficients: each 2x2 block is [[a1_i, a2_i], [1, 0]]
static void fsp_build_A(const struct aylp_fsp_axis *ax, double *A)
{
	size_t D = ax->dim;
	memset(A, 0, D * D * sizeof(double));
	for (size_t i = 0; i < ax->n_modes; i++) {
		size_t b = 2 * i;
		A[(b + 0) * D + (b + 0)] = ax->a1[i];
		A[(b + 0) * D + (b + 1)] = ax->a2[i];
		A[(b + 1) * D + (b + 0)] = 1.0;
	}
}

// C = A * B (all D x D, row-major)
static void fsp_mm(const double *A, const double *B, double *C, size_t D)
{
	for (size_t i = 0; i < D; i++) for (size_t j = 0; j < D; j++) {
		double s = 0.0;
		for (size_t k = 0; k < D; k++) s += A[i*D+k] * B[k*D+j];
		C[i*D+j] = s;
	}
}

// C = A * B^T (all D x D, row-major)
static void fsp_mmT(const double *A, const double *B, double *C, size_t D)
{
	for (size_t i = 0; i < D; i++) for (size_t j = 0; j < D; j++) {
		double s = 0.0;
		for (size_t k = 0; k < D; k++) s += A[i*D+k] * B[j*D+k];
		C[i*D+j] = s;
	}
}

// Solve the stationary Kalman gain L for one axis by iterating the predicted-
// covariance Riccati recursion to convergence. The measurement is the sum of
// the modal positions (C picks out the even-indexed "x_i(k)" states), plus
// white noise of variance r; the process noise Q is diagonal with q_i on each
// position state. Returns 0 on success.
static int fsp_solve_gain(struct aylp_fsp_axis *ax)
{
	size_t D = ax->dim;
	if (D == 0 || D > AYLP_FSP_MAX_DIM) return -1;
	double A[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double P[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double Pf[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double AP[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double Pn[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	fsp_build_A(ax, A);

	// Q (diagonal, q_i on position states) doubles as the initial P
	memset(P, 0, D * D * sizeof(double));
	for (size_t i = 0; i < ax->n_modes; i++)
		P[(2*i) * D + (2*i)] = ax->q[i] > 0.0 ? ax->q[i] : 1e-9;

	double PCt[AYLP_FSP_MAX_DIM];		// P C^T
	double CP[AYLP_FSP_MAX_DIM];		// C P
	double Kf[AYLP_FSP_MAX_DIM];		// filter gain wrt predicted state
	double prev_trace = 0.0;
	const size_t MAXIT = 2000;
	for (size_t it = 0; it < MAXIT; it++) {
		// S = C P C^T + r ; PCt[r] = sum_j P[r][2j] ; CP[c] = sum_i P[2i][c]
		double S = ax->r > 0.0 ? ax->r : 1e-9;
		for (size_t rI = 0; rI < D; rI++) {
			double pc = 0.0, cp = 0.0;
			for (size_t i = 0; i < ax->n_modes; i++) {
				pc += P[rI*D + 2*i];
				cp += P[(2*i)*D + rI];
			}
			PCt[rI] = pc;
			CP[rI] = cp;
		}
		for (size_t i = 0; i < ax->n_modes; i++) S += PCt[2*i];
		if (!isfinite(S) || S <= 0.0) return -1;
		for (size_t rI = 0; rI < D; rI++) Kf[rI] = PCt[rI] / S;
		// Pf = P - Kf * CP   (filtered covariance)
		for (size_t i = 0; i < D; i++) for (size_t j = 0; j < D; j++)
			Pf[i*D+j] = P[i*D+j] - Kf[i] * CP[j];
		// Pn = A Pf A^T + Q
		fsp_mm(A, Pf, AP, D);
		fsp_mmT(AP, A, Pn, D);
		for (size_t i = 0; i < ax->n_modes; i++)
			Pn[(2*i)*D + (2*i)] += ax->q[i] > 0.0 ? ax->q[i] : 1e-9;
		// convergence on the trace
		double tr = 0.0;
		for (size_t i = 0; i < D; i++) tr += Pn[i*D+i];
		memcpy(P, Pn, D * D * sizeof(double));
		if (it > 3 && fabs(tr - prev_trace) <= 1e-12 * (1.0 + fabs(tr)))
			break;
		prev_trace = tr;
	}
	// final gain L = P C^T / (C P C^T + r)
	double S = ax->r > 0.0 ? ax->r : 1e-9;
	for (size_t rI = 0; rI < D; rI++) {
		double pc = 0.0;
		for (size_t i = 0; i < ax->n_modes; i++) pc += P[rI*D + 2*i];
		PCt[rI] = pc;
	}
	for (size_t i = 0; i < ax->n_modes; i++) S += PCt[2*i];
	if (!isfinite(S) || S <= 0.0) return -1;
	for (size_t rI = 0; rI < D; rI++) ax->L[rI] = PCt[rI] / S;
	return 0;
}

// (re)compute the AR(2) coefficients of every mode from its f/zeta at rate fs
static void fsp_build_modes(struct aylp_fsp_axis *ax, double fs)
{
	double Ts = 1.0 / fs;
	for (size_t i = 0; i < ax->n_modes; i++) {
		double z = ax->zeta[i];
		if (z < 0.0) z = 0.0;
		if (z > 0.999) z = 0.999;
		double w = 2.0 * M_PI * ax->f[i] * Ts;
		double rpole = exp(-z * w);
		ax->a1[i] = 2.0 * rpole * cos(w * sqrt(1.0 - z*z));
		ax->a2[i] = -rpole * rpole;
		// stationary variance gain (see fsp.h); guard the (1 -/+ a1
		// - a2) factors, which approach 0 as the pole nears z = 1
		double d1 = 1.0 - ax->a1[i] - ax->a2[i];
		double d2 = 1.0 + ax->a1[i] - ax->a2[i];
		double d3 = 1.0 + ax->a2[i];
		if (d1 < 1e-12) d1 = 1e-12;
		if (d2 < 1e-12) d2 = 1e-12;
		if (d3 < 1e-12) d3 = 1e-12;
		ax->Gv[i] = (1.0 - ax->a2[i]) / (d3 * d1 * d2);
	}
}

// design the shared command-filter biquad (RBJ cookbook low-pass, Q=1/sqrt2)
static void fsp_build_cmdlp(struct aylp_fsp_data *data)
{
	double w0 = 2.0 * M_PI * data->cmd_fc / data->fs;
	// Q = 1/sqrt2 (Butterworth): alpha = sin(w0)/(2Q) = sin(w0)/sqrt2
	double alpha = sin(w0) * M_SQRT1_2;
	double cw = cos(w0);
	double a0 = 1.0 + alpha;
	data->lp_b0 = (1.0 - cw) / 2.0 / a0;
	data->lp_b1 = (1.0 - cw) / a0;
	data->lp_b2 = (1.0 - cw) / 2.0 / a0;
	data->lp_a1 = -2.0 * cw / a0;
	data->lp_a2 = (1.0 - alpha) / a0;
}

// (re)compute one axis's per-mode pre-compensation for the command filter:
// evaluate H(e^{j2pi f_i/fs}) at each mode center and store the extra
// prediction steps n_i = round(-arg H / omega_i) that cancel the filter's
// phase lag at that line, plus the capped gain boost min(1/|H|, 3) that
// cancels its droop. Called at init and on every adaptation tick (f_i wander
// moves the compensation point). See fsp.h for why this is a roll-forward
// and NOT a quadrature rotation.
static void fsp_build_comp(struct aylp_fsp_axis *ax,
	const struct aylp_fsp_data *data)
{
	ax->max_steps = data->delay;
	for (size_t i = 0; i < ax->n_modes; i++) {
		if (data->cmd_fc <= 0.0) {
			ax->comp_n[i] = 0;
			ax->comp_g[i] = 1.0;
			continue;
		}
		double w = 2.0 * M_PI * ax->f[i] / data->fs;
		// H(z) at z = e^{jw}: (b0 + b1 z^-1 + b2 z^-2)/(1 + a1 z^-1
		// + a2 z^-2), evaluated with real/imag parts
		double c1 = cos(w), s1 = -sin(w);		// z^-1
		double c2 = cos(2*w), s2 = -sin(2*w);		// z^-2
		double nr = data->lp_b0 + data->lp_b1*c1 + data->lp_b2*c2;
		double ni = data->lp_b1*s1 + data->lp_b2*s2;
		double dr = 1.0 + data->lp_a1*c1 + data->lp_a2*c2;
		double di = data->lp_a1*s1 + data->lp_a2*s2;
		double dd = dr*dr + di*di;
		double hr = (nr*dr + ni*di) / dd;
		double hi = (ni*dr - nr*di) / dd;
		double mag = hypot(hr, hi);
		double g = mag > 1e-9 ? 1.0/mag : 3.0;
		if (g > 3.0) g = 3.0;
		double th = -atan2(hi, hr);	// phase advance (rad, >= 0)
		size_t n = (size_t)(th/w + 0.5);
		ax->comp_n[i] = n;
		ax->comp_g[i] = g;
		if (data->delay + n > ax->max_steps)
			ax->max_steps = data->delay + n;
	}
}


// ---- parameter parsing helpers ---------------------------------------------

// pull a JSON array of doubles into dst (up to max); returns count
static size_t fsp_get_darray(struct json_object *val, double *dst, size_t max)
{
	if (!json_object_is_type(val, json_type_array)) return 0;
	size_t n = json_object_array_length(val);
	if (n > max) n = max;
	for (size_t i = 0; i < n; i++)
		dst[i] = json_object_get_double(
			json_object_array_get_idx(val, i)
		);
	return n;
}

// parse the per-axis mode/plant params out of a nested JSON object
static int fsp_parse_axis(struct aylp_fsp_axis *ax, struct json_object *obj)
{
	double q_scalar = -1.0;
	size_t nf = 0, nz = 0, nq = 0;
	json_object_object_foreach(obj, key, val) {
		if (key[0] == '_') {
		} else if (!strcmp(key, "K") || !strcmp(key, "plant_gain")) {
			ax->K = json_object_get_double(val);
		} else if (!strcmp(key, "r")) {
			ax->r = json_object_get_double(val);
		} else if (!strcmp(key, "freqs")) {
			nf = fsp_get_darray(val, ax->f, AYLP_FSP_MAX_MODES);
		} else if (!strcmp(key, "zeta")) {
			nz = fsp_get_darray(val, ax->zeta, AYLP_FSP_MAX_MODES);
		} else if (!strcmp(key, "q")) {
			if (json_object_is_type(val, json_type_array))
				nq = fsp_get_darray(val, ax->q,
					AYLP_FSP_MAX_MODES);
			else
				q_scalar = json_object_get_double(val);
		} else {
			log_warn("fsp: unknown axis parameter \"%s\"", key);
		}
	}
	ax->n_modes = nf;
	ax->dim = 2 * nf;
	if (!nf) {
		log_error("fsp: each axis needs a non-empty \"freqs\" array.");
		return -1;
	}
	if (ax->dim > AYLP_FSP_MAX_DIM) {
		log_error("fsp: at most %d modes per axis.", AYLP_FSP_MAX_MODES);
		return -1;
	}
	// fill defaults: zeta 0.002 (lightly damped line), q from the scalar or
	// 1e-5 -- q is the DRIVE variance in (normalized error units)^2 per
	// sample and must be physically scaled against r; q/r near 1 puts the
	// whole loop in the waterbed-amplifying regime (see doc/devices/fsp.md)
	for (size_t i = 0; i < nf; i++) {
		if (i >= nz) ax->zeta[i] = 0.002;
		if (i >= nq) ax->q[i] = q_scalar > 0.0 ? q_scalar : 1e-5;
	}
	if (ax->r <= 0.0) ax->r = 1.0;
	if (ax->K == 0.0) {
		log_error("fsp: each axis needs a nonzero plant gain \"K\".");
		return -1;
	}
	return 0;
}


int fsp_init(struct aylp_device *self)
{
	self->proc = &fsp_proc;
	self->fini = &fsp_fini;
	self->device_data = xcalloc(1, sizeof(struct aylp_fsp_data));
	struct aylp_fsp_data *data = self->device_data;

	// defaults
	data->delay = 5;		// ~2 ms at ~2310 Hz, matches the loop
	data->delay_frac = 0.0;
	data->fs = 2310.0;
	data->clamp = 1.0;
	data->start_delay = 0.0;
	data->ramp = 10.0;
	data->adapt_period = 0.0;	// fixed FSP unless asked
	data->adapt_df_max = 0.5;	// Hz per update
	data->adapt_tau = 5.0;
	data->cmd_fc = 0.0;		// raw minimum-variance command unless asked
	data->broad_order = 0;		// modal-only unless explicitly identified
	data->broad_mu = 0.03;

	if (!self->params) {
		log_error("fsp: no params object found.");
		return -1;
	}
	struct json_object *axy = NULL, *axx = NULL;
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
		} else if (!strcmp(key, "type")) {
			const char *s = json_object_get_string(val);
			if (!strcmp(s, "vector")) data->type = AYLP_T_VECTOR;
			else log_error("fsp: unrecognized type: %s", s);
		} else if (!strcmp(key, "units")) {
			data->units = aylp_units_from_string(
				json_object_get_string(val));
		} else if (!strcmp(key, "delay")) {
			data->delay = json_object_get_uint64(val);
		} else if (!strcmp(key, "delay_frac")) {
			data->delay_frac = json_object_get_double(val);
		} else if (!strcmp(key, "fs")) {
			data->fs = json_object_get_double(val);
		} else if (!strcmp(key, "clamp")) {
			data->clamp = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "start_delay")) {
			data->start_delay = json_object_get_double(val);
			if (data->start_delay < 0) data->start_delay = 0;
		} else if (!strcmp(key, "ramp")) {
			data->ramp = json_object_get_double(val);
			if (data->ramp < 0) data->ramp = 0;
		} else if (!strcmp(key, "adapt_period")) {
			data->adapt_period = json_object_get_double(val);
		} else if (!strcmp(key, "adapt_df_max")) {
			data->adapt_df_max = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "adapt_tau")) {
			data->adapt_tau = json_object_get_double(val);
		} else if (!strcmp(key, "cmd_fc")) {
			data->cmd_fc = json_object_get_double(val);
		} else if (!strcmp(key, "broad_order")) {
			data->broad_order = json_object_get_uint64(val);
		} else if (!strcmp(key, "broad_mu")) {
			data->broad_mu = json_object_get_double(val);
		} else if (!strcmp(key, "y") || !strcmp(key, "axis_y")) {
			axy = val;
		} else if (!strcmp(key, "x") || !strcmp(key, "axis_x")) {
			axx = val;
		} else {
			log_warn("fsp: unknown parameter \"%s\"", key);
		}
	}

	if (data->type != AYLP_T_VECTOR) {
		log_error("fsp: type must be \"vector\".");
		return -1;
	}
	if (data->delay < 1) {
		log_error("fsp: delay must be >= 1 sample.");
		return -1;
	}
	if (data->delay_frac < 0.0 || data->delay_frac >= 1.0) {
		log_error("fsp: delay_frac must satisfy 0 <= delay_frac < 1.");
		return -1;
	}
	if (data->fs <= 0.0) {
		log_error("fsp: fs must be > 0.");
		return -1;
	}
	if (data->broad_order > 4096) {
		log_error("fsp: broad_order must be <= 4096.");
		return -1;
	}
	if (data->broad_order && (data->broad_mu <= 0.0
			|| data->broad_mu >= 2.0)) {
		log_error("fsp: broad_mu must satisfy 0 < broad_mu < 2.");
		return -1;
	}
	if (!axy || !axx) {
		log_error("fsp: both \"y\" and \"x\" axis objects are required.");
		return -1;
	}
	if (fsp_parse_axis(&data->axis[0], axy)) return -1;
	if (fsp_parse_axis(&data->axis[1], axx)) return -1;

	if (data->cmd_fc > 0.0) {
		if (data->cmd_fc >= data->fs / 2.0) {
			log_error("fsp: cmd_fc must be < fs/2.");
			return -1;
		}
		fsp_build_cmdlp(data);
		log_info("fsp: command robustness low-pass at %G Hz, per-mode "
			"phase/gain pre-compensated", data->cmd_fc);
	}

	for (int a = 0; a < 2; a++) {
		struct aylp_fsp_axis *ax = &data->axis[a];
		fsp_build_modes(ax, data->fs);
		fsp_build_comp(ax, data);
		if (fsp_solve_gain(ax)) {
			log_error("fsp: Riccati solve failed on %s axis; check "
				"q/r and mode params.", a == 0 ? "y" : "x");
			return -1;
		}
		// One extra command is retained for fractional-delay interpolation.
		ax->ucmd = xcalloc(data->delay + 1, sizeof(double));
		if (data->broad_order) {
			data->broad_hist_len = data->broad_order + data->delay + 1;
			ax->broad_hist = xcalloc(data->broad_hist_len,
				sizeof(double));
			ax->broad_w = xcalloc(data->broad_order, sizeof(double));
			ax->broad_w_next = xcalloc(data->broad_order,
				sizeof(double));
			ax->broad_xbuf = xcalloc(data->broad_order, sizeof(double));
		}
		ax->r_ewma = ax->r;
		for (size_t i = 0; i < ax->n_modes; i++) {
			// q_ewma tracks the mode's STATE energy; seed it at the
			// stationary energy implied by the configured drive q
			ax->q_ewma[i] = ax->q[i] * ax->Gv[i];
			ax->demod_re[i] = ax->demod_im[i] = 0.0;
			ax->demod_ph[i] = 0.0;
		}
		log_info("fsp: %s axis, %zu modes, K=%G, predicting %zu samples "
			"ahead%s", a == 0 ? "y" : "x", ax->n_modes, ax->K,
			data->delay,
			data->adapt_period > 0 ? ", adaptive" : " (fixed)");
	}
	if (data->broad_order)
		log_info("fsp: full-band %zu-state disturbance predictor, horizon "
			"%zu+%.3G frames, NLMS mu %G", data->broad_order,
			data->delay, data->delay_frac, data->broad_mu);

	self->type_in = AYLP_T_VECTOR;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_UNCHANGED;
	self->units_out = data->units;
	return 0;
}


// slow adaptation tick: refresh q_i (from projected innovation energy) and r
// (from the broadband innovation floor), optionally nudge each center freq
// toward the locally demodulated line, then rebuild coeffs and the gain.
static void fsp_adapt(struct aylp_fsp_data *data)
{
	for (int a = 0; a < 2; a++) {
		struct aylp_fsp_axis *ax = &data->axis[a];
		for (size_t i = 0; i < ax->n_modes; i++) {
			// process noise tracks the mode's recent energy, with a
			// floor so a quiet line can come back. q_ewma is STATE
			// energy; the Riccati wants the DRIVE variance, smaller
			// by the mode's stationary variance gain Gv (~1e4 for a
			// sharp line). Feeding state energy in directly inflates
			// q/r by that factor and re-creates the ~3x waterbed the
			// fsp_sim.py study measured.
			double qn = ax->q_ewma[i] / ax->Gv[i];
			if (qn < 1e-12) qn = 1e-12;
			ax->q[i] = qn;
			// bounded frequency retune from the demod phasor angle
			if (data->adapt_df_max > 0.0) {
				double mag = hypot(ax->demod_re[i],
					ax->demod_im[i]);
				if (mag > 1e-9) {
					double ang = atan2(ax->demod_im[i],
						ax->demod_re[i]);
					// ang is the residual phase over the
					// window; convert to a small df, capped
					double df = ang / (2.0 * M_PI)
						* (1.0 / data->adapt_tau);
					if (df > data->adapt_df_max)
						df = data->adapt_df_max;
					if (df < -data->adapt_df_max)
						df = -data->adapt_df_max;
					ax->f[i] += df;
					if (ax->f[i] < 0.1) ax->f[i] = 0.1;
					if (ax->f[i] > 0.45 * data->fs)
						ax->f[i] = 0.45 * data->fs;
				}
			}
			ax->demod_re[i] = ax->demod_im[i] = 0.0;
		}
		if (ax->r_ewma > 1e-12) ax->r = ax->r_ewma;
		fsp_build_modes(ax, data->fs);
		fsp_build_comp(ax, data);
		if (fsp_solve_gain(ax))
			log_warn("fsp: adaptive Riccati solve failed on %s "
				"axis; keeping previous gain.",
				a == 0 ? "y" : "x");
	}
	log_debug("fsp: adapted (re-identified modes and gain).");
}


int fsp_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_fsp_data *data = self->device_data;
	gsl_vector *s = state->vector;

	struct timespec tp;
	int err = clock_gettime(CLOCK_MONOTONIC, &tp);
	if (err) {
		log_error("fsp: couldn't get time: %s", strerror(err));
		return -1;
	}
	double now = tp.tv_sec + 1e-9 * tp.tv_nsec;
	if (UNLIKELY(!data->t0)) data->t0 = now;

	if (UNLIKELY(!data->res_v || data->res_v->size != s->size)) {
		if (data->res_v) xfree_type(gsl_vector, data->res_v);
		data->res_v = xmalloc_type(gsl_vector, s->size);
	}
	gsl_vector *r = data->res_v;
	data->n_elem = s->size;

	// closed-loop authority: 0 during the startup hold, then a linear ramp
	// from 0 to 1 over `ramp` seconds so the handover is bumpless
	bool in_hold = (now - data->t0) < data->start_delay;
	double frac;
	if (in_hold) {
		frac = 0.0;
	} else {
		if (UNLIKELY(!data->t_close)) {
			data->t_close = now;
			data->t_adapt = now;
		}
		double up = now - data->t_close;
		frac = (data->ramp > 0.0 && up < data->ramp)
			? up / data->ramp : 1.0;
		if (UNLIKELY(frac > 0.0 && !data->closed)) {
			data->closed = true;
			log_info("fsp: loop closing; blending command in "
				"over %G s", data->ramp);
		}
	}

	double beta = 0.0;	// EWMA weight for adaptation stats
	if (data->adapt_period > 0.0 && data->adapt_tau > 0.0)
		beta = 1.0 - exp(-1.0 / (data->adapt_tau * data->fs));

	size_t delay = data->delay;
	size_t ring_len = delay + 1;
	size_t slot_older = data->uhead;	// u(k-delay-1)
	size_t slot = (slot_older + 1) % ring_len;	// u(k-delay)
	if (data->broad_order) {
		data->broad_head = (data->broad_head + 1) % data->broad_hist_len;
		data->broad_seen++;
	}

	for (size_t j = 0; j < s->size; j++) {
		// only elements 0 (y) and 1 (x) are controlled; pass any extras
		if (j > 1) { r->data[j * r->stride] = 0.0; continue; }
		struct aylp_fsp_axis *ax = &data->axis[j];
		double e = s->data[j * s->stride];

		// --- Smith-predictor core: reconstruct the disturbance by
		// removing our own delayed plant contribution ---
		// Integer delay after a first-order Thiran all-pass fractional
		// delay. Unlike linear interpolation, the Thiran model has unity
		// magnitude through Nyquist and therefore does not leave a false
		// high-frequency command residue in the Smith reconstruction.
		double u_old = ax->ucmd[slot];
		double phi_meas = e - ax->K * u_old;

		// Full compound-disturbance observer (Kulcsar/Petit/Meimon):
		// identify the delay-step conditional mean of the reconstructed
		// disturbance. In scalar form the stationary Kalman/Wiener observer
		// is an FIR state-space realization; NLMS tracks slow spectrum drift
		// without a large online covariance matrix.
		double broad_hat = 0.0;
		if (data->broad_order) {
			size_t P = data->broad_order, H = data->broad_hist_len;
			ax->broad_hist[data->broad_head] = phi_meas;
			if (data->broad_seen >= H) {
				size_t idx = (data->broad_head + H - delay) % H;
				double pred = 0.0, energy = 1e-12;
				for (size_t i = 0; i < P; i++) {
					double v = ax->broad_hist[idx];
					ax->broad_xbuf[i] = v;
					pred += ax->broad_w[i] * v;
					energy += v * v;
					idx = idx ? idx - 1 : H - 1;
				}
				double pe = phi_meas - pred;
				if (isfinite(pe)) {
					double step = data->broad_mu * pe / energy;
					for (size_t i = 0; i < P; i++)
						ax->broad_w[i] += step * ax->broad_xbuf[i];
				} else {
					memset(ax->broad_w, 0, P * sizeof(double));
				}
				// Identify the adjacent delay+1 predictor. Their weighted
				// combination is the conditional mean at the fractional
				// Bode-fit horizon.
				idx = (data->broad_head + H - delay - 1) % H;
				pred = 0.0; energy = 1e-12;
				for (size_t i = 0; i < P; i++) {
					double v = ax->broad_hist[idx];
					ax->broad_xbuf[i] = v;
					pred += ax->broad_w_next[i] * v;
					energy += v * v;
					idx = idx ? idx - 1 : H - 1;
				}
				pe = phi_meas - pred;
				if (isfinite(pe)) {
					double step = data->broad_mu * pe / energy;
					for (size_t i = 0; i < P; i++)
						ax->broad_w_next[i] += step
							* ax->broad_xbuf[i];
				} else {
					memset(ax->broad_w_next, 0, P * sizeof(double));
				}
				idx = data->broad_head;
				double broad_next = 0.0;
				for (size_t i = 0; i < P; i++) {
					broad_hat += ax->broad_w[i]
						* ax->broad_hist[idx];
					broad_next += ax->broad_w_next[i]
						* ax->broad_hist[idx];
					idx = idx ? idx - 1 : H - 1;
				}
				broad_hat = (1.0 - data->delay_frac) * broad_hat
					+ data->delay_frac * broad_next;
				if (!isfinite(broad_hat)) broad_hat = 0.0;
			}
		}

		// --- Kalman filter: predict one step, then correct ---
		// xpred = A xhat (per-mode AR recursion); Cxpred = sum positions
		double xpred[AYLP_FSP_MAX_DIM];
		double Cxpred = 0.0;
		for (size_t i = 0; i < ax->n_modes; i++) {
			double p0 = ax->a1[i]*ax->xhat[2*i]
				+ ax->a2[i]*ax->xhat[2*i+1];
			double p1 = ax->xhat[2*i];
			xpred[2*i] = p0;
			xpred[2*i+1] = p1;
			Cxpred += p0;
		}
		double innov = phi_meas - Cxpred;
		if (!isfinite(innov)) {
			// bad sample: reset the estimate, output 0
			memset(ax->xhat, 0, ax->dim * sizeof(double));
			ax->ucmd[slot_older] = 0.0;
			r->data[j * r->stride] = 0.0;
			continue;
		}
		for (size_t d = 0; d < ax->dim; d++)
			ax->xhat[d] = xpred[d] + ax->L[d] * innov;

		// --- adaptation statistics (cheap, per sample) ---
		if (beta > 0.0) {
			ax->r_ewma += beta * (innov*innov - ax->r_ewma);
			for (size_t i = 0; i < ax->n_modes; i++) {
				double amp = ax->xhat[2*i]*ax->xhat[2*i];
				ax->q_ewma[i] += beta * (amp - ax->q_ewma[i]);
				// quadrature demod of the reconstructed
				// disturbance at the nominal line, to sense
				// frequency drift
				ax->demod_ph[i] += 2.0*M_PI*ax->f[i]/data->fs;
				if (ax->demod_ph[i] > M_PI)
					ax->demod_ph[i] -= 2.0*M_PI;
				ax->demod_re[i] += beta * (phi_meas
					* cos(ax->demod_ph[i]) - ax->demod_re[i]);
				ax->demod_im[i] += beta * (phi_meas
					* sin(ax->demod_ph[i]) - ax->demod_im[i]);
			}
		}

		// --- delay-step prediction: run the AR mean forward, harvesting
		// mode i's (boosted) contribution at its own step count
		// delay + comp_n[i], so the command filter's phase lag at that
		// line nets out to ~0; the minimum-variance command cancels the
		// predicted sum ---
		double p_now[AYLP_FSP_MAX_DIM];
		memcpy(p_now, ax->xhat, ax->dim * sizeof(double));
		double phi_hat = 0.0;
		for (size_t step = 1; step <= ax->max_steps; step++) {
			for (size_t i = 0; i < ax->n_modes; i++) {
				double p0 = ax->a1[i]*p_now[2*i]
					+ ax->a2[i]*p_now[2*i+1];
				p_now[2*i+1] = p_now[2*i];
				p_now[2*i] = p0;
				if (step == delay + ax->comp_n[i])
					phi_hat += ax->comp_g[i] * p0;
			}
		}

		// The full-band predictor and modal predictor estimate the same
		// disturbance. Select, do not sum, to avoid double cancellation.
		double cancel_hat = data->broad_order ? broad_hat : phi_hat;
		double u = -frac * cancel_hat / ax->K;
		// command robustness low-pass (DF2T biquad); see fsp.h
		if (data->cmd_fc > 0.0) {
			double uf = data->lp_b0*u + ax->lp_z1;
			ax->lp_z1 = data->lp_b1*u - data->lp_a1*uf + ax->lp_z2;
			ax->lp_z2 = data->lp_b2*u - data->lp_a2*uf;
			u = uf;
		}
		if (!isfinite(u)) {
			u = 0.0;
			ax->lp_z1 = ax->lp_z2 = 0.0;
		}
		if (u > data->clamp) u = data->clamp;
		if (u < -data->clamp) u = -data->clamp;

		// Apply the fractional part of the plant delay before the integer
		// command ring. H(z)=(a+z^-1)/(1+a*z^-1), with DC group delay f.
		double a_frac = (1.0 - data->delay_frac)
			/ (1.0 + data->delay_frac);
		double u_frac = a_frac*u + ax->frac_x1 - a_frac*ax->frac_y1;
		ax->frac_x1 = u;
		ax->frac_y1 = u_frac;
		ax->ucmd[slot_older] = u_frac;
		r->data[j * r->stride] = u;
	}

	// advance the shared command-delay ring once per frame
	data->uhead = (data->uhead + 1) % ring_len;
	data->n_seen += 1;

	// slow re-identification
	if (data->adapt_period > 0.0 && data->t_close
			&& (now - data->t_adapt) >= data->adapt_period) {
		fsp_adapt(data);
		data->t_adapt = now;
	}

	state->vector = r;
	return 0;
}


int fsp_fini(struct aylp_device *self)
{
	struct aylp_fsp_data *data = self->device_data;
	for (int a = 0; a < 2; a++) {
		xfree(data->axis[a].ucmd);
		xfree(data->axis[a].broad_hist);
		xfree(data->axis[a].broad_w);
		xfree(data->axis[a].broad_w_next);
		xfree(data->axis[a].broad_xbuf);
	}
	if (data->res_v) xfree_type(gsl_vector, data->res_v);
	xfree(data);
	return 0;
}
