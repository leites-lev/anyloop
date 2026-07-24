#include <time.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
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
// Riccati gain solve, refactored into resumable pieces (begin / iterate /
// finalize) so the adaptation can spread it a few iterations per frame while
// init still runs it synchronously. The solve state (A, P, iteration count,
// last trace) lives on the axis so it survives between frames.
#define AYLP_FSP_RICCATI_MAXIT 2000
// Riccati iterations run per frame while a re-identification solve is in
// progress. ~16 D<=16 iterations is a few microseconds -- far below the frame
// budget -- so the solve (typically a few hundred iterations) finishes in tens
// of milliseconds without any single frame stalling.
#define AYLP_FSP_ADAPT_ITERS 16

// snapshot A and seed P = Q, then reset the iteration counter.
static int fsp_solve_gain_begin(struct aylp_fsp_axis *ax)
{
	size_t D = ax->dim;
	if (D == 0 || D > AYLP_FSP_MAX_DIM) return -1;
	fsp_build_A(ax, ax->adapt_A);
	// Q (diagonal, q_i on position states) doubles as the initial P
	memset(ax->adapt_P, 0, D * D * sizeof(double));
	for (size_t i = 0; i < ax->n_modes; i++)
		ax->adapt_P[(2*i) * D + (2*i)] = ax->q[i] > 0.0 ? ax->q[i] : 1e-9;
	ax->adapt_it = 0;
	ax->adapt_prev_trace = 0.0;
	return 0;
}

// advance the fixed-point iteration by up to `niter` steps.
// returns 1 = converged (or hit the iteration cap), 0 = more work to do,
// -1 = numeric failure. Does not touch ax->L (see finalize).
static int fsp_solve_gain_iterate(struct aylp_fsp_axis *ax, size_t niter)
{
	size_t D = ax->dim;
	double *A = ax->adapt_A;
	double *P = ax->adapt_P;
	double Pf[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double AP[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double Pn[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double PCt[AYLP_FSP_MAX_DIM];		// P C^T
	double CP[AYLP_FSP_MAX_DIM];		// C P
	double Kf[AYLP_FSP_MAX_DIM];		// filter gain wrt predicted state
	for (size_t step = 0; step < niter; step++) {
		if (ax->adapt_it >= AYLP_FSP_RICCATI_MAXIT) return 1;
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
		size_t it = ax->adapt_it++;
		if (it > 3 && fabs(tr - ax->adapt_prev_trace)
				<= 1e-12 * (1.0 + fabs(tr)))
			return 1;
		ax->adapt_prev_trace = tr;
	}
	return 0;
}

// final gain L = P C^T / (C P C^T + r) from the converged covariance.
static int fsp_solve_gain_finalize(struct aylp_fsp_axis *ax)
{
	size_t D = ax->dim;
	double *P = ax->adapt_P;
	double PCt[AYLP_FSP_MAX_DIM];
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

// synchronous solve for the init path: run the whole iteration in one call.
static int fsp_solve_gain(struct aylp_fsp_axis *ax)
{
	if (fsp_solve_gain_begin(ax)) return -1;
	int done;
	while ((done = fsp_solve_gain_iterate(ax,
			AYLP_FSP_RICCATI_MAXIT)) == 0)
		;
	if (done < 0) return -1;
	return fsp_solve_gain_finalize(ax);
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

// design one axis's burst-guard band-pass (RBJ constant-peak band-pass) at
// the parasitic-loop regeneration frequency fs/(2*(delay+delay_frac)), the
// frequency where any plant-model mismatch rings (~310-340 Hz on this bench).
// Q = 1.5 spans the measured 250-450 Hz hump.
static void fsp_build_guard(struct aylp_fsp_axis *ax, double fs)
{
	double f0 = fs / (2.0 * ((double)ax->delay + ax->delay_frac));
	if (f0 > 0.45 * fs) f0 = 0.45 * fs;
	ax->gd_f0 = f0;
	double w0 = 2.0 * M_PI * f0 / fs;
	double alpha = sin(w0) / (2.0 * 1.5);
	double a0 = 1.0 + alpha;
	ax->gd_b0 = alpha / a0;		// b1 = 0
	ax->gd_b2 = -alpha / a0;
	ax->gd_a1 = -2.0 * cos(w0) / a0;
	ax->gd_a2 = (1.0 - alpha) / a0;
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
	ax->max_steps = ax->delay;
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
		if (ax->delay + n > ax->max_steps)
			ax->max_steps = ax->delay + n;
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
	// sentinels: inherit the global delay/delay_frac unless the axis sets them
	ax->delay = 0;
	ax->delay_frac = -1.0;
	json_object_object_foreach(obj, key, val) {
		if (key[0] == '_') {
		} else if (!strcmp(key, "K") || !strcmp(key, "plant_gain")) {
			ax->K = json_object_get_double(val);
		} else if (!strcmp(key, "delay")) {
			ax->delay = json_object_get_uint64(val);
		} else if (!strcmp(key, "delay_frac")) {
			ax->delay_frac = json_object_get_double(val);
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


static void fsp_adapt(struct aylp_fsp_data *data);

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
	data->broad_lp = 0;		// raw broadband observer unless asked
	data->broad_freeze_closed = true;
	data->trip_frames = 8;
	// burst guard defaults: on. Quiet-bench 250-450 Hz envelope is ~0.003
	// normalized; the floor keeps the bar at 4 * 0.008 = 0.032 (~0.5 px),
	// well below the multi-px bursts and well above ambient.
	data->guard_ratio = 4.0;
	data->guard_floor = 0.008;
	data->guard_hold = 0.25;
	data->guard_ramp = 1.0;
	data->guard_tick = 10.0;
	// stall-gap patch default: on. The response is proportional to the
	// frames missed (pad + continue, no authority change), so the trigger
	// can sit just above normal cadence jitter: 2 ms is ~7 frames at
	// 3788 Hz, and it also catches the ~0.3-0.6 s camera-stall hole
	// (see fsp.h).
	data->gap_trip = 0.002;

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
		} else if (!strcmp(key, "broad_lp")) {
			data->broad_lp = json_object_get_uint64(val);
		} else if (!strcmp(key, "broad_freeze_closed")) {
			data->broad_freeze_closed = json_object_get_boolean(val);
		} else if (!strcmp(key, "trip_error")) {
			data->trip_error = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "trip_command")) {
			data->trip_command = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "trip_frames")) {
			data->trip_frames = json_object_get_uint64(val);
		} else if (!strcmp(key, "guard_ratio")) {
			data->guard_ratio = json_object_get_double(val);
		} else if (!strcmp(key, "guard_floor")) {
			data->guard_floor = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "guard_hold")) {
			data->guard_hold = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "guard_ramp")) {
			data->guard_ramp = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "guard_tick")) {
			data->guard_tick = fabs(json_object_get_double(val));
		} else if (!strcmp(key, "gap_trip")) {
			data->gap_trip = json_object_get_double(val);
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
	if (data->broad_lp) {
		if (!data->broad_order) {
			log_error("fsp: broad_lp requires broad_order > 0.");
			return -1;
		}
		if (data->broad_lp < 3 || data->broad_lp > 63) {
			log_error("fsp: broad_lp must be 3..63 taps.");
			return -1;
		}
		if (!(data->broad_lp & 1)) {
			data->broad_lp += 1;
			log_warn("fsp: broad_lp must be odd for an integer "
				"group delay; using %zu", data->broad_lp);
		}
	}
	data->broad_gd = data->broad_lp ? (data->broad_lp - 1) / 2 : 0;
	if ((data->trip_error > 0.0 || data->trip_command > 0.0)
			&& !data->trip_frames) {
		log_error("fsp: trip_frames must be >= 1 when a trip is enabled.");
		return -1;
	}
	if (!axy || !axx) {
		log_error("fsp: both \"y\" and \"x\" axis objects are required.");
		return -1;
	}
	if (fsp_parse_axis(&data->axis[0], axy)) return -1;
	if (fsp_parse_axis(&data->axis[1], axx)) return -1;

	// resolve per-axis delays: unset values inherit the global ones
	for (int a = 0; a < 2; a++) {
		struct aylp_fsp_axis *ax = &data->axis[a];
		if (!ax->delay) ax->delay = data->delay;
		if (ax->delay_frac < 0.0) ax->delay_frac = data->delay_frac;
		if (ax->delay_frac >= 1.0) {
			log_error("fsp: %s axis delay_frac must satisfy "
				"0 <= delay_frac < 1.", a == 0 ? "y" : "x");
			return -1;
		}
	}

	if (data->cmd_fc > 0.0) {
		if (data->cmd_fc >= data->fs / 2.0) {
			log_error("fsp: cmd_fc must be < fs/2.");
			return -1;
		}
		fsp_build_cmdlp(data);
		log_info("fsp: command robustness low-pass at %G Hz, per-mode "
			"phase/gain pre-compensated", data->cmd_fc);
	}

	if (data->guard_ratio > 0.0) {
		// fast envelope ~10 ms (rides the burst onset), baseline ~10 s
		data->guard_beta_fast = 1.0 - exp(-1.0 / (0.010 * data->fs));
		data->guard_beta_slow = 1.0 - exp(-1.0 / (10.0 * data->fs));
	}
	// stall-gap DC pad: ~0.5 s EWMA -- slow enough to average the
	// vibration lines out, fast enough to track intra-run drift
	data->gap_dc_beta = 1.0 - exp(-1.0 / (0.5 * data->fs));

	for (int a = 0; a < 2; a++) {
		struct aylp_fsp_axis *ax = &data->axis[a];
		fsp_build_modes(ax, data->fs);
		fsp_build_comp(ax, data);
		if (data->guard_ratio > 0.0) fsp_build_guard(ax, data->fs);
		if (fsp_solve_gain(ax)) {
			log_error("fsp: Riccati solve failed on %s axis; check "
				"q/r and mode params.", a == 0 ? "y" : "x");
			return -1;
		}
		// One extra command is retained for fractional-delay interpolation.
		ax->ucmd = xcalloc(ax->delay + 1, sizeof(double));
		if (data->broad_order) {
			ax->broad_hist_len = data->broad_order + ax->delay
				+ data->broad_gd + 2;
			ax->broad_hist = xcalloc(ax->broad_hist_len,
				sizeof(double));
			ax->broad_w = xcalloc(data->broad_order, sizeof(double));
			ax->broad_w_next = xcalloc(data->broad_order,
				sizeof(double));
			ax->broad_xbuf = xcalloc(data->broad_order, sizeof(double));
			if (data->broad_lp)
				ax->broad_lpbuf = xcalloc(data->broad_lp,
					sizeof(double));
		}
		ax->r_ewma = ax->r;
		for (size_t i = 0; i < ax->n_modes; i++) {
			// q_ewma tracks the mode's STATE energy; seed it at the
			// stationary energy implied by the configured drive q
			ax->q_ewma[i] = ax->q[i] * ax->Gv[i];
			ax->demod_re[i] = ax->demod_im[i] = 0.0;
			ax->demod_ph[i] = 0.0;
		}
		log_info("fsp: %s axis, %zu modes, K=%G, predicting %zu+%.3G "
			"samples ahead%s", a == 0 ? "y" : "x", ax->n_modes, ax->K,
			ax->delay, ax->delay_frac,
			data->adapt_period > 0 ? ", adaptive" : " (fixed)");
	}
	if (data->broad_order)
		log_info("fsp: full-band %zu-state disturbance predictor, horizon "
			"y %zu+%.3G / x %zu+%.3G frames, NLMS mu %G",
			data->broad_order,
			data->axis[0].delay, data->axis[0].delay_frac,
			data->axis[1].delay, data->axis[1].delay_frac,
			data->broad_mu);
	if (data->broad_lp)
		log_info("fsp: observer band-limit: %zu-tap boxcar prefilter "
			"(first null %.0f Hz), +%zu frames folded into the broad "
			"horizon", data->broad_lp, data->fs / (double)data->broad_lp,
			data->broad_gd);
	else if (data->broad_order)
		log_warn("fsp: observer band-limit DISABLED (broad_lp = 0): the "
			"NLMS can learn/chase HF command echo from K/delay "
			"mismatch (see the 2026-07-22 380 Hz ring)");
	if (data->trip_error > 0.0 || data->trip_command > 0.0)
		log_info("fsp: latched safety trip: error=%G command=%G for %zu "
			"frames", data->trip_error, data->trip_command,
			data->trip_frames);
	if (data->guard_ratio > 0.0)
		log_info("fsp: burst guard on: y %.0f Hz / x %.0f Hz detectors, "
			"trigger %Gx over max(baseline, %G), hold %G s + ramp "
			"%G s, ticker every %G s", data->axis[0].gd_f0,
			data->axis[1].gd_f0, data->guard_ratio,
			data->guard_floor, data->guard_hold, data->guard_ramp,
			data->guard_tick);
	else
		log_warn("fsp: burst guard DISABLED (guard_ratio <= 0)");
	if (data->gap_trip > 0.0)
		log_info("fsp: stall-gap patch on: frame gap > %G ms pads the "
			"histories with the held command and continues",
			1e3 * data->gap_trip);
	else
		log_warn("fsp: stall-gap patch DISABLED (gap_trip <= 0)");

	// --- RT hygiene (2026-07-21): lock memory and warm the adaptation path.
	// Under SCHED_FIFO the first re-identification after the loop closes was
	// bursting ~2.8 ms/axis (two back-to-back frame gaps ~8 s in) because its
	// code, stack, and workspace pages were cold / not resident. Lock the
	// process into RAM (no demand-paging faults on the RT loop) and run the
	// exact runtime adapt+solve sequence once here, off the critical path, so
	// every page it touches is already warm when adapt_period first fires.
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
		log_warn("fsp: mlockall failed (%s); page faults may still jitter "
			"the loop -- run with privilege / raised memlock rlimit",
			strerror(errno));
	else
		log_info("fsp: locked process memory (mlockall) against RT page "
			"faults");
	if (data->adapt_period > 0.0) {
		fsp_adapt(data);	// build_modes/comp + solve_begin, both axes
		for (int a = 0; a < 2; a++) {
			struct aylp_fsp_axis *ax = &data->axis[a];
			while (fsp_solve_gain_iterate(ax, AYLP_FSP_RICCATI_MAXIT)
					== 0)
				;
			fsp_solve_gain_finalize(ax);	// restores the init gain
			ax->adapt_solving = false;
		}
		log_info("fsp: warmed adaptation path (first re-identification "
			"pre-touched)");
	}

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
		// Kick off the resumable Riccati solve instead of running it
		// here: fsp_proc advances it a few iterations per frame so it
		// never bursts. The modes/comp above take effect immediately;
		// the previous gain keeps running until the new one converges
		// (a few tens of ms), a negligible mismatch since the modes
		// moved at most adapt_df_max Hz.
		if (fsp_solve_gain_begin(ax)) {
			log_warn("fsp: adaptive Riccati init failed on %s "
				"axis; keeping previous gain.",
				a == 0 ? "y" : "x");
			ax->adapt_solving = false;
		} else {
			ax->adapt_solving = true;
		}
	}
	log_debug("fsp: re-identification started (gain solve amortized).");
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
	if (data->tripped) frac = 0.0;

	// --- stall-gap patch: a proc-to-proc gap beyond gap_trip means the
	// source dropped frames (scheduler hiccup) or stalled outright
	// (camera stream stall + capture restart). The DAC held the last
	// command throughout, so patch each history with what actually
	// happened, sized to the frames missed, and continue -- no hold, no
	// ramp, no authority change (see fsp.h). ---
	if (UNLIKELY(data->gap_trip > 0.0 && data->t_last > 0.0
			&& now - data->t_last >= data->gap_trip)) {
		double gap_s = now - data->t_last;
		size_t miss = (size_t)(gap_s * data->fs + 0.5);
		miss = miss > 1 ? miss - 1 : 1;	// this frame isn't missed
		data->gap_events++;
		log_warn("fsp: %.1f ms frame gap (~%zu frames; source %s); "
			"padding histories with the held command and continuing "
			"(gap event %zu)", 1e3 * gap_s, miss,
			gap_s > 0.1 ? "stall" : "hiccup", data->gap_events);
		for (size_t a = 0; a < 2; a++) {
			struct aylp_fsp_axis *ax = &data->axis[a];
			size_t ring_len = ax->delay + 1;
			// The DAC held the last commanded u (r gets the
			// pre-frac-filter u): run the fractional all-pass and
			// the Smith ring forward on that constant input for
			// the missed frames (capped: past ring_len + settling
			// more iterations change nothing)
			double u_held = ax->frac_x1;
			double a_frac = (1.0 - ax->delay_frac)
				/ (1.0 + ax->delay_frac);
			size_t n = miss < ring_len + 16 ? miss : ring_len + 16;
			for (size_t k = 0; k < n; k++) {
				double u_frac = a_frac * u_held + ax->frac_x1
					- a_frac * ax->frac_y1;
				ax->frac_x1 = u_held;
				ax->frac_y1 = u_frac;
				ax->ucmd[ax->uhead] = u_frac;
				ax->uhead = (ax->uhead + 1) % ring_len;
			}
			// Propagate the modal state through the gap: the AR
			// recursion advances each mode's phase exactly as the
			// model says it should. It is a contraction, so this
			// is safe at any gap length; cap the work at 2 s of
			// frames, beyond which sharp lines have decayed.
			size_t np = miss < (size_t)(2.0 * data->fs)
				? miss : (size_t)(2.0 * data->fs);
			for (size_t k = 0; k < np; k++) {
				for (size_t i = 0; i < ax->n_modes; i++) {
					double p0 = ax->a1[i] * ax->xhat[2*i]
						+ ax->a2[i] * ax->xhat[2*i+1];
					ax->xhat[2*i+1] = ax->xhat[2*i];
					ax->xhat[2*i] = p0;
				}
			}
			// Pad the NLMS history with the slow DC estimate of
			// the disturbance: phases across the hole are
			// unknowable, but this keeps the prediction carrying
			// the DC correction straight through (no release), and
			// real frames refill the tap window in ~H/fs.
			if (data->broad_order) {
				size_t H = ax->broad_hist_len;
				size_t nh = miss < H ? miss : H;
				for (size_t k = 0; k < nh; k++) {
					ax->broad_head = (ax->broad_head + 1)
						% H;
					ax->broad_hist[ax->broad_head] =
						ax->phi_dc;
				}
				// pad the observer prefilter ring the same way
				size_t nl = miss < data->broad_lp
					? miss : data->broad_lp;
				for (size_t k = 0; k < nl; k++) {
					ax->broad_lpbuf[ax->broad_lphead] =
						ax->phi_dc;
					ax->broad_lphead = (ax->broad_lphead
						+ 1) % data->broad_lp;
				}
				ax->broad_seen += nh;
				// pause training until the fabricated samples
				// leave the tap window -- unless they are a
				// negligible fraction of it (a few-frame drop
				// trains straight through)
				if (nh > 8) ax->broad_fab = H;
			}
		}
	}
	data->t_last = now;

	double beta = 0.0;	// EWMA weight for adaptation stats
	if (data->adapt_period > 0.0 && data->adapt_tau > 0.0)
		beta = 1.0 - exp(-1.0 / (data->adapt_tau * data->fs));

	for (size_t j = 0; j < s->size; j++) {
		// only elements 0 (y) and 1 (x) are controlled; pass any extras
		if (j > 1) { r->data[j * r->stride] = 0.0; continue; }
		struct aylp_fsp_axis *ax = &data->axis[j];
		double e = s->data[j * s->stride];

		// per-axis delay bookkeeping (each axis is visited exactly once
		// per frame, so advancing here keeps per-frame cadence)
		size_t delay = ax->delay;
		size_t ring_len = delay + 1;
		size_t slot_older = ax->uhead;	// u(k-delay-1)
		size_t slot = (slot_older + 1) % ring_len;	// u(k-delay)
		if (data->broad_order) {
			ax->broad_head = (ax->broad_head + 1)
				% ax->broad_hist_len;
			ax->broad_seen++;
		}

		// Learn the ordinary open-loop operating point.  Use the same slow
		// timescale as the other statistics; attenuation_test provides 500 s
		// of genuine open-loop data, so the preceding zeroed START phase has
		// completely washed out before closing.
		if (in_hold) {
			double tau = data->adapt_tau > 0.0 ? data->adapt_tau : 5.0;
			double b = 1.0 - exp(-1.0 / (tau * data->fs));
			ax->trip_center += b * (e - ax->trip_center);
		}

		// --- burst guard: band-pass the raw error at this axis's
		// regeneration frequency, envelope-detect, and shed authority
		// while a burst is alive (see fsp.h). g_gain multiplies this
		// axis's closed-loop authority; suppress freezes learning so
		// the NLMS/adaptation never trains on the ringing. ---
		double g_gain = 1.0;
		bool suppress = false;
		bool over = false;
		if (data->guard_ratio > 0.0) {
			double bp = ax->gd_b0 * e + ax->gd_z1;
			ax->gd_z1 = -ax->gd_a1 * bp + ax->gd_z2;
			ax->gd_z2 = ax->gd_b2 * e - ax->gd_a2 * bp;
			if (!isfinite(bp)) {
				bp = 0.0;
				ax->gd_z1 = ax->gd_z2 = 0.0;
			}
			ax->gd_env += data->guard_beta_fast
				* (bp * bp - ax->gd_env);
			double base = ax->gd_base;
			double fl = data->guard_floor * data->guard_floor;
			if (base < fl) base = fl;
			double thr = data->guard_ratio * data->guard_ratio
				* base;
			over = ax->gd_env > thr;
			if (!ax->guard_active && frac > 0.0 && over) {
				ax->guard_active = true;
				ax->guard_ramping = false;
				ax->guard_t_trip = now;
				ax->guard_events++;
				data->guard_events++;
				if (now - ax->guard_t_log > 2.0) {
					ax->guard_t_log = now;
					log_warn("fsp: burst guard tripped on "
						"%s axis (event %zu): %.0f Hz "
						"envelope %.3G > %.3G; holding "
						"authority at 0 for %G s, then "
						"ramping back over %G s",
						j == 0 ? "y" : "x",
						ax->guard_events, ax->gd_f0,
						sqrt(ax->gd_env), sqrt(thr),
						data->guard_hold,
						data->guard_ramp);
				}
			} else if (!ax->guard_active && frac > 0.0 && !over) {
				// learn the quiet baseline only outside bursts
				ax->gd_base += data->guard_beta_slow
					* (ax->gd_env - ax->gd_base);
			}
		}
		// hold/ramp re-entry, applied outside the detector gate so a
		// stall-gap trip re-enters the same way even when the band-pass
		// detector is disabled (guard_ratio <= 0)
		if (ax->guard_active) {
			double ts = now - ax->guard_t_trip;
			if (ts <= data->guard_hold) {
				g_gain = 0.0;
			} else if (over) {
				// still ringing after the hold: back to
				// zero authority. If recovery had begun
				// this is a fresh regeneration (a new
				// event); a ring that merely outlives
				// the hold extends the same event.
				if (ax->guard_ramping) {
					ax->guard_events++;
					data->guard_events++;
					ax->guard_ramping = false;
				}
				ax->guard_t_trip = now;
				g_gain = 0.0;
			} else {
				double rmp = data->guard_ramp > 0.0
					? data->guard_ramp : 1e-9;
				ax->guard_ramping = true;
				g_gain = (ts - data->guard_hold) / rmp;
				if (g_gain >= 1.0) {
					g_gain = 1.0;
					ax->guard_active = false;
				}
			}
			if (g_gain < 1.0) {
				suppress = true;
				ax->guard_frames++;
			}
		}
		// stall-gap bookkeeping: count down the frames until the
		// fabricated (padded) samples have left the NLMS tap window
		if (UNLIKELY(ax->broad_fab)) {
			ax->broad_fab--;
			suppress = true;
		}

		// --- Smith-predictor core: reconstruct the disturbance by
		// removing our own delayed plant contribution ---
		// Integer delay after a first-order Thiran all-pass fractional
		// delay. Unlike linear interpolation, the Thiran model has unity
		// magnitude through Nyquist and therefore does not leave a false
		// high-frequency command residue in the Smith reconstruction.
		double u_old = ax->ucmd[slot];
		double phi_meas = e - ax->K * u_old;
		// slow DC estimate of the reconstructed disturbance, used to
		// pad the NLMS history across a stall gap
		ax->phi_dc += data->gap_dc_beta * (phi_meas - ax->phi_dc);

		// Full compound-disturbance observer (Kulcsar/Petit/Meimon):
		// identify the delay-step conditional mean of the reconstructed
		// disturbance. In scalar form the stationary Kalman/Wiener observer
		// is an FIR state-space realization; NLMS tracks slow spectrum drift
		// without a large online covariance matrix.
		double broad_hat = 0.0;
		if (data->broad_order) {
			size_t P = data->broad_order, H = ax->broad_hist_len;
			// Observer band-limit (see fsp.h): boxcar the raw phi so
			// the NLMS never sees (nor learns to chase) the HF command
			// echo; its exact integer group delay broad_gd is added to
			// the prediction horizon below, so in-band timing holds.
			double phi_bl = phi_meas;
			if (data->broad_lp) {
				ax->broad_lpbuf[ax->broad_lphead] = phi_meas;
				ax->broad_lphead = (ax->broad_lphead + 1)
					% data->broad_lp;
				double sum = 0.0;
				for (size_t i = 0; i < data->broad_lp; i++)
					sum += ax->broad_lpbuf[i];
				phi_bl = sum / (double)data->broad_lp;
			}
			size_t bd = delay + data->broad_gd;
			ax->broad_hist[ax->broad_head] = phi_bl;
			if (ax->broad_seen >= H) {
				size_t idx = (ax->broad_head + H - bd) % H;
				double pred = 0.0, energy = 1e-12;
				for (size_t i = 0; i < P; i++) {
					double v = ax->broad_hist[idx];
					ax->broad_xbuf[i] = v;
					pred += ax->broad_w[i] * v;
					energy += v * v;
					idx = idx ? idx - 1 : H - 1;
				}
				double pe = phi_bl - pred;
				bool train = (!data->broad_freeze_closed
					|| in_hold) && !suppress;
				if (train && isfinite(pe)) {
					double step = data->broad_mu * pe / energy;
					for (size_t i = 0; i < P; i++)
						ax->broad_w[i] += step * ax->broad_xbuf[i];
				} else if (!isfinite(pe)) {
					memset(ax->broad_w, 0, P * sizeof(double));
				}
				// Identify the adjacent delay+1 predictor. Their weighted
				// combination is the conditional mean at the fractional
				// Bode-fit horizon.
				idx = (ax->broad_head + H - bd - 1) % H;
				pred = 0.0; energy = 1e-12;
				for (size_t i = 0; i < P; i++) {
					double v = ax->broad_hist[idx];
					ax->broad_xbuf[i] = v;
					pred += ax->broad_w_next[i] * v;
					energy += v * v;
					idx = idx ? idx - 1 : H - 1;
				}
				pe = phi_bl - pred;
				if (train && isfinite(pe)) {
					double step = data->broad_mu * pe / energy;
					for (size_t i = 0; i < P; i++)
						ax->broad_w_next[i] += step
							* ax->broad_xbuf[i];
				} else if (!isfinite(pe)) {
					memset(ax->broad_w_next, 0, P * sizeof(double));
				}
				idx = ax->broad_head;
				double broad_next = 0.0;
				for (size_t i = 0; i < P; i++) {
					broad_hat += ax->broad_w[i]
						* ax->broad_hist[idx];
					broad_next += ax->broad_w_next[i]
						* ax->broad_hist[idx];
					idx = idx ? idx - 1 : H - 1;
				}
				broad_hat = (1.0 - ax->delay_frac) * broad_hat
					+ ax->delay_frac * broad_next;
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
			// bad sample: reset the estimate, output 0; still advance
			// this axis's command ring to keep the delay line in step
			memset(ax->xhat, 0, ax->dim * sizeof(double));
			ax->ucmd[slot_older] = 0.0;
			ax->uhead = (ax->uhead + 1) % ring_len;
			r->data[j * r->stride] = 0.0;
			continue;
		}
		for (size_t d = 0; d < ax->dim; d++)
			ax->xhat[d] = xpred[d] + ax->L[d] * innov;

		// --- adaptation statistics (cheap, per sample); frozen while
		// the burst guard is active so the ringing never contaminates
		// the identified model ---
		if (beta > 0.0 && !suppress) {
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
		double u = -frac * g_gain * cancel_hat / ax->K;
		// A static centroid offset is normal and may require substantial DC
		// command to remove.  Trip only when the magnitude grows beyond its
		// learned open-loop level; motion toward zero is successful control,
		// not a fault.
		bool over_error = data->trip_error > 0.0
			&& fabs(e) > fabs(ax->trip_center) + data->trip_error;
		bool over_command = data->trip_command > 0.0
			&& fabs(u) > data->trip_command;
		if (frac > 0.0 && (over_error || over_command)) {
			ax->trip_count++;
		} else {
			ax->trip_count = 0;
		}
		if (!data->tripped && ax->trip_count >= data->trip_frames) {
			data->tripped = true;
			log_error("fsp: SAFETY TRIP latched on %s axis: e=%G "
				"(open baseline=%G), requested u=%G; output held at "
				"zero until restart", j == 0 ? "y" : "x", e,
				ax->trip_center, u);
		}
		if (data->tripped) u = 0.0;
		// command robustness low-pass (DF2T biquad); see fsp.h
		if (data->cmd_fc > 0.0) {
			double uf = data->lp_b0*u + ax->lp_z1;
			ax->lp_z1 = data->lp_b1*u - data->lp_a1*uf + ax->lp_z2;
			ax->lp_z2 = data->lp_b2*u - data->lp_a2*uf;
			u = uf;
		}
		if (data->tripped) {
			u = 0.0;
			ax->lp_z1 = ax->lp_z2 = 0.0;
		}
		if (!isfinite(u)) {
			u = 0.0;
			ax->lp_z1 = ax->lp_z2 = 0.0;
		}
		if (u > data->clamp) u = data->clamp;
		if (u < -data->clamp) u = -data->clamp;

		// Apply the fractional part of the plant delay before the integer
		// command ring. H(z)=(a+z^-1)/(1+a*z^-1), with DC group delay f.
		double a_frac = (1.0 - ax->delay_frac)
			/ (1.0 + ax->delay_frac);
		double u_frac = a_frac*u + ax->frac_x1 - a_frac*ax->frac_y1;
		ax->frac_x1 = u;
		ax->frac_y1 = u_frac;
		ax->ucmd[slot_older] = u_frac;
		ax->uhead = (ax->uhead + 1) % ring_len;
		r->data[j * r->stride] = u;
	}

	data->n_seen += 1;
	if (data->t_close) data->n_closed += 1;

	// burst-guard ticker: a periodic terminal line with the activation
	// count and the fraction of closed-loop frames spent at reduced
	// authority, so a marginal run is visible while it happens
	if (data->guard_ratio > 0.0 && data->guard_tick > 0.0
			&& data->t_close) {
		if (UNLIKELY(!data->t_tick)) data->t_tick = data->t_close;
		if (now - data->t_tick >= data->guard_tick) {
			data->t_tick = now;
			size_t gy = data->axis[0].guard_events;
			size_t gx = data->axis[1].guard_events;
			double held = 100.0 * (double)(data->axis[0].guard_frames
				+ data->axis[1].guard_frames)
				/ (2.0 * (double)data->n_closed);
			log_info("fsp guard: %zu activations (y %zu / x %zu), "
				"authority shed %.2f%% of the closed loop so far",
				data->guard_events, gy, gx, held);
		}
	}

	// slow re-identification, amortized: start a new solve every
	// adapt_period, then advance any in-progress solve a few iterations per
	// frame so the Riccati never bursts (a single-frame solve stalled frame
	// delivery ~7 ms and logged as a periodic "source hiccup" -- see fsp.h).
	// Don't start a new one while the previous is still converging.
	if (data->adapt_period > 0.0 && data->t_close
			&& !data->axis[0].adapt_solving
			&& !data->axis[1].adapt_solving
			&& (now - data->t_adapt) >= data->adapt_period) {
		fsp_adapt(data);
		data->t_adapt = now;
	}
	for (int a = 0; a < 2; a++) {
		struct aylp_fsp_axis *ax = &data->axis[a];
		if (!ax->adapt_solving) continue;
		int done = fsp_solve_gain_iterate(ax, AYLP_FSP_ADAPT_ITERS);
		if (done == 0) continue;	// more iterations next frame
		if (done < 0 || fsp_solve_gain_finalize(ax))
			log_warn("fsp: adaptive Riccati solve failed on %s "
				"axis; keeping previous gain.",
				a == 0 ? "y" : "x");
		ax->adapt_solving = false;
	}

	state->vector = r;
	return 0;
}


int fsp_fini(struct aylp_device *self)
{
	struct aylp_fsp_data *data = self->device_data;
	if (data->guard_ratio > 0.0 && data->n_closed)
		log_info("fsp guard final: %zu activations (y %zu / x %zu) "
			"over %zu closed-loop frames; recurring activations "
			"mean K/delay no longer match the plant -- re-run the "
			"bodes at the current coarse bias before trusting an "
			"attenuation number", data->guard_events,
			data->axis[0].guard_events, data->axis[1].guard_events,
			data->n_closed);
	if (data->gap_events)
		log_warn("fsp: %zu frame gap(s) patched this run (source "
			"stalled or hiccuped; check asi_source recovery lines)",
			data->gap_events);
	for (int a = 0; a < 2; a++) {
		xfree(data->axis[a].ucmd);
		xfree(data->axis[a].broad_hist);
		xfree(data->axis[a].broad_w);
		xfree(data->axis[a].broad_w_next);
		xfree(data->axis[a].broad_xbuf);
		xfree(data->axis[a].broad_lpbuf);
	}
	if (data->res_v) xfree_type(gsl_vector, data->res_v);
	xfree(data);
	return 0;
}
