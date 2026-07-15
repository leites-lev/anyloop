#ifndef AYLP_DEVICES_FSP_H_
#define AYLP_DEVICES_FSP_H_

#include <stdbool.h>
#include <time.h>

#include "anyloop.h"

// Adaptive Filtered Smith Predictor / adaptive LQG for tip-tilt beam
// stabilization. This is a COMPLETE controller: it takes the [y, x] error
// (center-of-mass output) and emits the [y, x] command, so it REPLACES both
// anyloop:kalman_filter and anyloop:pid in the pipeline (running it alongside
// either double-compensates the loop delay and will pump lines). It sits where
// pid used to: after center_of_mass and any logging sink, before clamp.
//
// Structure (per axis, independent; index 0 = y, 1 = x, matching pid's
// [y,x] convention). The plant seen by the loop is a gain K plus a transport
// delay of `delay + delay_frac` samples (camera + compute + DAC ZOH):
//
//     e(k) = phi(k) + K * ((1-f)u(k-delay) + f*u(k-delay-1))
//
// where phi is the disturbance (bench vibration) the beam would see open-loop,
// and u is our command. The three pieces, following Kulcsar 2006 / Petit 2008
// / Meimon 2010 / Correia 2010 (optimal/adaptive control for AO):
//
//  1. SMITH-PREDICTOR CORE. We know K and delay, so we reconstruct the
//     disturbance from the measured error by removing our own delayed
//     contribution:  phi_meas(k) = e(k) - K * u(k - delay).  This is the
//     internal plant model that takes the delay out of the loop's
//     characteristic equation. During the open-loop startup hold (command
//     forced to 0) phi_meas is exactly the raw open-loop disturbance, which
//     is the cleanest data to identify the vibration modes on.
//
//  2. DISTURBANCE MODEL + KALMAN FILTER. phi is modeled as a sum of `n_modes`
//     narrowband AR(2) vibration modes (Meimon 2010's tip-tilt model), each a
//     damped oscillator driven by white noise:
//         x_i(k) = a1_i x_i(k-1) + a2_i x_i(k-2) + nu_i,   var(nu_i)=q_i
//         a1_i =  2 exp(-2 pi z_i f_i Ts) cos(2 pi f_i Ts sqrt(1 - z_i^2))
//         a2_i = -exp(-4 pi z_i f_i Ts)
//     The stacked state X = [x_i(k), x_i(k-1)]_i is estimated from phi_meas by
//     a Kalman filter with a STATIONARY gain L, precomputed at init by
//     iterating the Riccati recursion to convergence. The q_i/r ratio is the
//     LQG tuning knob (process vs measurement noise): larger q_i = more
//     aggressive tracking of that line.
//
//  3. delay-STEP PREDICTION + MINIMUM-VARIANCE CONTROL. The command issued now
//     lands `delay` samples from now, so we project the modal state forward by
//     running the AR recursion `delay` steps (the mean prediction, zero future
//     noise) and cancel the predicted disturbance:
//         phi_hat(k+delay|k) = C A^delay Xhat(k|k)
//         u(k) = -phi_hat(k+delay|k) / K
//     This is what "gets around" the delay on the PREDICTABLE (modal) part of
//     phi; the unpredictable broadband floor and anything above the loop's
//     achievable bandwidth are NOT helped (a fundamental limit, not an
//     implementation one).
//
// ADAPTATION (Petit/Meimon): the vibration lines wander (~0.5 Hz/run on this
// bench) and vary several-fold in amplitude within hours, so a fixed model
// detunes. Every `adapt_period` seconds we re-identify from buffered data:
// update each mode's process-noise q_i from its recent innovation energy,
// update the measurement noise r from the broadband innovation floor,
// optionally nudge each mode center frequency f_i toward the locally measured
// line (bounded by `adapt_df_max`), rebuild the AR coefficients, and recompute
// the stationary gain L. Set adapt_period <= 0 for a fixed (non-adaptive) FSP.
//
// COMMAND ROBUSTNESS FILTER (the "F" in FSP; cmd_fc > 0): the raw
// minimum-variance command injects the estimate's broadband/noise content onto
// the beam at frequencies where prediction is worthless, which is what showed
// up as a ~3x waterbed at 120-400 Hz in the fsp_sim.py study. A 2nd-order
// low-pass at cmd_fc rolls that authority off. A plain filter would also lag
// the in-band line cancellation, so each mode is PRE-COMPENSATED at its
// center frequency: mode i's AR state is rolled forward n_i EXTRA samples,
// n_i = round(-arg H(f_i) / omega_i), so the filter's phase lag at that line
// nets out to ~0, and its contribution is boosted by min(1/|H(f_i)|, 3) to
// undo the gain droop. (Do NOT "fix" this back to a quadrature rotation of
// the state: extracting the quadrature divides by sin(omega_i), which
// amplifies the white part of the Kalman state ~1/sin(omega_i) -- ~65x for a
// 5.6 Hz mode at 2310 Hz -- straight into the command; the fsp_sim.py study
// measured the closed loop getting WORSE that way. Rolling through the stable
// AR dynamics is a contraction: same phase advance, no noise amplification.)
// Recomputed on every adaptation tick when f_i moves. cmd_fc = 0 disables
// (raw minimum-variance command).

#define AYLP_FSP_MAX_MODES 8
#define AYLP_FSP_MAX_DIM (2 * AYLP_FSP_MAX_MODES)

// per-axis controller state
struct aylp_fsp_axis {
	size_t n_modes;
	size_t dim;			// = 2 * n_modes
	double K;			// signed plant gain (error units per command
					// unit); sign must make the loop negative
					// feedback -- verify with a push test
	// per-mode nominal parameters (also the adaptation targets)
	double f[AYLP_FSP_MAX_MODES];		// center frequency (Hz)
	double zeta[AYLP_FSP_MAX_MODES];	// damping ratio (0..1)
	double q[AYLP_FSP_MAX_MODES];		// process noise variance
	double r;				// measurement noise variance
	// derived AR(2) coefficients per mode
	double a1[AYLP_FSP_MAX_MODES];
	double a2[AYLP_FSP_MAX_MODES];
	// stationary variance gain of each mode: var(state)/var(drive) =
	// (1-a2)/((1+a2)(1-a1-a2)(1+a1-a2)); ~1/(4 pi zeta f Ts) for light
	// damping, i.e. HUGE for sharp lines. Converts between the state
	// energy the adaptation measures and the drive variance q the Riccati
	// solve needs -- confusing the two inflates q/r by orders of magnitude
	// and puts the loop in the waterbed-amplifying regime (see _note on
	// scaling in fsp.c / doc/devices/fsp.md).
	double Gv[AYLP_FSP_MAX_MODES];
	// filtered modal state estimate Xhat(k|k), laid out per mode as
	// [x_i(k), x_i(k-1)]
	double xhat[AYLP_FSP_MAX_DIM];
	// stationary Kalman gain (column, length dim)
	double L[AYLP_FSP_MAX_DIM];
	// command delay line of length `delay + 1`, also retaining the adjacent
	// sample needed by the fractional-delay plant model
	double *ucmd;
	// adaptation accumulators (per mode): EWMA of innovation energy
	// projected onto the mode, and a quadrature demodulator for the local
	// frequency estimate
	double q_ewma[AYLP_FSP_MAX_MODES];
	double demod_re[AYLP_FSP_MAX_MODES];
	double demod_im[AYLP_FSP_MAX_MODES];
	double demod_ph[AYLP_FSP_MAX_MODES];	// running reference phase (rad)
	double r_ewma;				// EWMA innovation variance floor
	// command robustness filter: per-mode pre-compensation at the mode
	// center -- extra prediction steps n_i = round(-arg H(f_i)/omega_i)
	// cancelling the filter's phase lag, and a capped gain boost
	// min(1/|H(f_i)|, 3) cancelling its droop -- plus the biquad state
	// (direct form II transposed)
	size_t comp_n[AYLP_FSP_MAX_MODES];
	double comp_g[AYLP_FSP_MAX_MODES];
	size_t max_steps;	// = delay + max_i comp_n[i]
	double lp_z1, lp_z2;
	// Optional full-band disturbance observer. This is the scalar FIR
	// realization of the delayed Wiener/Kalman predictor: it learns the
	// minimum-variance delay-step estimate directly from the Smith-
	// reconstructed disturbance, including the broadband continuum omitted
	// by the small bank of AR(2) vibration modes.
	double *broad_hist;
	double *broad_w;
	double *broad_w_next;
	double *broad_xbuf;
	// First-order Thiran all-pass state for the fractional command delay:
	// y(k) = a*u(k) + u(k-1) - a*y(k-1).
	double frac_x1, frac_y1;
	size_t trip_count;
};

struct aylp_fsp_data {
	// param type: only "vector" is meaningful
	aylp_type type;
	// units to output (command units, e.g. minmax)
	aylp_units units;
	// loop transport delay in samples (camera + compute + DAC ZOH)
	size_t delay;
	// Fractional remainder of the measured transport delay. The plant uses a
	// first-order Thiran all-pass (unit magnitude, correct low-frequency group
	// delay); the full-band observer blends adjacent horizon predictions.
	double delay_frac;
	// sample rate (Hz) used to turn f/zeta into AR coefficients; should
	// match the loop rate
	double fs;
	// output magnitude limit (command units)
	double clamp;
	// seconds to hold the command at 0 (loop open) at startup while the
	// Kalman filter converges on the clean open-loop disturbance
	double start_delay;
	// seconds over which the command blends 0 -> full authority after the
	// hold, so the handover is bumpless
	double ramp;
	// adaptation cadence (s); <= 0 disables adaptation (fixed FSP)
	double adapt_period;
	// max per-update center-frequency correction (Hz); 0 freezes f
	double adapt_df_max;
	// EWMA time constant for the adaptation statistics (s)
	double adapt_tau;
	// command robustness low-pass cutoff (Hz); 0 disables. See the header
	// comment: rolls off command authority where prediction is worthless,
	// with per-mode phase/gain pre-compensation so the lines still cancel.
	double cmd_fc;
	// Full-band predictor order and NLMS identification step. order=0 keeps
	// the original modal-only controller. When enabled it supplies the
	// command prediction; the modal observer continues to identify/log but
	// is not added again (which would double-count the same disturbance).
	size_t broad_order;
	double broad_mu;
	// Identification is safest under a known zero command. When true, NLMS
	// weights freeze as soon as the startup hold ends.
	bool broad_freeze_closed;
	// Latched safety trip. While authority is nonzero, either excessive
	// measured error or requested command for trip_frames consecutive samples
	// opens the loop (zero command) until process restart.
	double trip_error;
	double trip_command;
	size_t trip_frames;
	bool tripped;
	size_t broad_hist_len;
	size_t broad_head;
	size_t broad_seen;
	// shared biquad coefficients for the command filter (normalized)
	double lp_b0, lp_b1, lp_b2, lp_a1, lp_a2;

	// per-axis controllers: [0] = y (element 0), [1] = x (element 1)
	struct aylp_fsp_axis axis[2];
	// shared ring write index for the per-axis command delay lines
	size_t uhead;

	// timing / handover bookkeeping
	double t0;		// CLOCK_MONOTONIC of first proc (s)
	double t_close;		// time the loop first closed (s); 0 until then
	double t_adapt;		// time of last adaptation (s)
	bool closed;		// true once past start_delay (logged once)
	size_t n_seen;		// frames processed since start

	// number of pipeline elements the last proc saw (for the passthrough
	// guard when the vector isn't 2-wide)
	size_t n_elem;
	// result vector
	gsl_vector *res_v;
};

// initialize fsp device
int fsp_init(struct aylp_device *self);

// process fsp device once per loop
int fsp_proc(struct aylp_device *self, struct aylp_state *state);

// close fsp device when loop exits
int fsp_fini(struct aylp_device *self);

#endif
