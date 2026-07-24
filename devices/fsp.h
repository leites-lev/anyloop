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
//
// BURST GUARD (guard_ratio > 0, on by default): any mismatch between the
// configured plant model (K, delay) and the true plant leaks the command back
// into the Smith reconstruction, closing a parasitic loop whose phase hits
// -180 deg at f = fs / (2 * (delay + delay_frac)) -- ~310-340 Hz here. When
// the margin at that frequency goes negative (K drifts with the coarse bias /
// alignment; see the 2026-07-16 "310 Hz spiral" and the 2026-07-17 run-15
// rerun), the loop emits intermittent BURSTS of ringing there that the 500 s
// PSD averages into a broad 250-450 Hz hump and a ruined RMS. The guard runs
// a per-axis band-pass at that frequency on the raw error, tracks a fast
// envelope against a slow baseline (with an absolute floor so a quiet bench
// can't lower the bar to noise), and on detection: cuts that axis's authority
// to zero, freezes NLMS training and the adaptation statistics (so the
// predictor does not learn the ringing), holds guard_hold seconds, then ramps
// authority back over guard_ramp seconds. Every activation is counted and a
// ticker line is printed every guard_tick seconds of closed-loop time. This
// keeps a marginal run alive; the FIX for recurring activations is
// re-measuring K/delay at the current operating point.
//
// STALL-GAP HANDLING (gap_trip > 0, on by default): if the source drops
// frames (a scheduler hiccup) or stalls outright (the ASI camera stream
// stall + capture restart, a ~0.3-0.6 s hole), the DAC holds the last
// command while the disturbance keeps moving, and every history in the
// predictor ends up spanning the hole. Neither re-entering at full
// authority onto stale state (burst seed) nor cutting authority (releases
// the DC correction: the FSM snaps to bias and the accumulated drift
// re-appears as a multi-px excursion -- observed 2026-07-17) is right.
// Instead the gap is simply PATCHED, sized to the frames actually missed:
// the Smith command ring is padded with the held command (exact -- that is
// what the plant received), the modal state is propagated through the gap
// with the AR recursion (the model knows how phases advance), and the NLMS
// input history is padded with a slow EWMA of the reconstructed disturbance
// -- so the prediction carries the DC correction straight through the hole
// and recovers full AC prediction as real frames refill the tap window
// (~broad_hist_len/fs, ~140 ms). No hold, no ramp, no authority change: a
// 2-frame drop costs 2 frames; a capture restart costs the hole plus the
// refill. NLMS training pauses only while fabricated samples meaningfully
// pollute the tap window (skipped entirely for drops of a few frames).
// Gap events are counted separately from burst events: bursts mean
// "re-measure K", gaps mean "the camera stalled".

#define AYLP_FSP_MAX_MODES 8
#define AYLP_FSP_MAX_DIM (2 * AYLP_FSP_MAX_MODES)

// per-axis controller state
struct aylp_fsp_axis {
	size_t n_modes;
	size_t dim;			// = 2 * n_modes
	double K;			// signed plant gain (error units per command
					// unit); sign must make the loop negative
					// feedback -- verify with a push test
	// Per-axis transport delay in samples. The two axes' plants differ
	// (bode 2026-07-16: x 5.62 vs y 6.37 frames at 3788 Hz), so each axis
	// may set its own "delay"/"delay_frac" inside its object; unset values
	// inherit the global ones. Resolved at init (0 / <0 = inherit).
	size_t delay;
	double delay_frac;
	// per-axis command ring and full-band observer bookkeeping -- lengths
	// depend on the axis delay so they cannot be shared
	size_t uhead;			// ring write index into ucmd
	size_t broad_hist_len;		// = broad_order + delay + broad_gd + 2
	size_t broad_head;
	size_t broad_seen;
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
	// Amortized re-identification solve state. The Riccati gain solve is
	// spread a few iterations per frame (fsp_proc) instead of run in one
	// burst -- the burst stalled frame delivery ~7 ms every adapt_period and
	// showed up as a periodic "source hiccup". adapt_A / adapt_P persist
	// across frames while adapt_solving is set; the previous gain keeps
	// running until the new one converges.
	bool adapt_solving;
	size_t adapt_it;			// current Riccati iteration
	double adapt_prev_trace;		// trace of P at the last iteration
	double adapt_A[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
	double adapt_P[AYLP_FSP_MAX_DIM * AYLP_FSP_MAX_DIM];
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
	size_t max_steps;	// = ax->delay + max_i comp_n[i]
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
	// observer prefilter ring: the last broad_lp raw phi samples (see
	// broad_lp in aylp_fsp_data)
	double *broad_lpbuf;
	size_t broad_lphead;
	// First-order Thiran all-pass state for the fractional command delay:
	// y(k) = a*u(k) + u(k-1) - a*y(k-1).
	double frac_x1, frac_y1;
	// Open-loop operating-point estimate used by the safety detector.  The
	// beam may have a large legitimate static centroid offset, so an absolute
	// error threshold would trip as soon as the loop starts.  During the
	// startup hold this tracks that offset; closed-loop safety compares error
	// magnitude with the learned open-loop magnitude plus trip_error.
	double trip_center;
	size_t trip_count;
	// Burst guard (see header comment). Band-pass biquad at this axis's
	// regeneration frequency fs/(2*(delay+delay_frac)), DF2T state, fast
	// envelope and slow baseline of the band POWER, and the recovery state
	// machine: guard_gain multiplies this axis's authority (0 during the
	// hold, 0->1 over guard_ramp afterward).
	double gd_f0;			// detector center (Hz), for logging
	double gd_b0, gd_b2, gd_a1, gd_a2;	// band-pass coeffs (b1 = 0)
	double gd_z1, gd_z2;
	double gd_env;			// fast EWMA of bp^2
	double gd_base;			// slow EWMA of bp^2 (quiet frames only)
	bool guard_active;
	bool guard_ramping;		// recovery ramp has begun since the trip
	double guard_t_trip;		// time of the latest trigger (s)
	double guard_t_log;		// last per-event log line (rate limit)
	size_t guard_events;		// activations since start
	size_t guard_frames;		// frames spent at reduced authority
	// stall-gap bookkeeping: slow EWMA of the reconstructed disturbance,
	// used to pad the NLMS history across a frame gap (so the prediction
	// carries the DC correction straight through the hole), and a
	// countdown of frames until fabricated samples have left the tap
	// window (training pauses while it is nonzero).
	double phi_dc;
	size_t broad_fab;
};

struct aylp_fsp_data {
	// param type: only "vector" is meaningful
	aylp_type type;
	// units to output (command units, e.g. minmax)
	aylp_units units;
	// loop transport delay in samples (camera + compute + DAC ZOH); the
	// GLOBAL DEFAULT -- each axis object may override with its own
	// "delay"/"delay_frac" (see aylp_fsp_axis)
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
	// Observer band-limit (broad_lp > 0, odd): boxcar prefilter length on the
	// reconstructed disturbance feeding the full-band observer. The NLMS is
	// otherwise broadband to Nyquist, and any K/delay model error leaks the
	// command back into its input as a predictable high-frequency echo that
	// it LEARNS and chases into a self-sustained ring (2026-07-22: ~380 Hz
	// x-axis limit cycle, 4.8 px rms, ignited ~1 min after close with zero
	// external trigger, even with K/delay set from same-day bodes -- the
	// mismatch is amplitude-dependent, so no static tune removes it). The
	// boxcar zeroes the observer's loop gain at fs/broad_lp and attenuates
	// the whole regeneration band; being linear-phase with exactly integer
	// group delay (broad_lp-1)/2, that delay is simply ADDED to the broad
	// prediction horizon, so in-band cancellation timing is unchanged
	// (passband droop at 30 Hz is <1%). 0 disables (raw broadband observer).
	size_t broad_lp;
	size_t broad_gd;	// = (broad_lp-1)/2, derived at init
	// Identification is safest under a known zero command. When true, NLMS
	// weights freeze as soon as the startup hold ends.
	bool broad_freeze_closed;
	// Latched safety trip. While authority is nonzero, either error magnitude
	// exceeding the learned open-loop magnitude by trip_error, or an excessive
	// requested command, for trip_frames consecutive samples opens the loop
	// (zero command) until process restart.
	double trip_error;
	double trip_command;
	size_t trip_frames;
	bool tripped;
	// Burst guard tuning (see header comment). guard_ratio is an AMPLITUDE
	// ratio: trigger when the band envelope exceeds guard_ratio times the
	// quiet baseline (or the guard_floor, whichever is larger; floor is in
	// normalized error units). <= 0 disables the guard.
	double guard_ratio;
	double guard_floor;
	double guard_hold;	// s at zero authority after a trigger
	double guard_ramp;	// s to ramp authority back to 1
	double guard_tick;	// s between ticker lines; 0 silences the ticker
	double guard_beta_fast;	// EWMA weights derived at init
	double guard_beta_slow;
	double t_tick;		// last ticker print (s)
	size_t guard_events;	// total activations, both axes
	// Stall-gap handling (see header comment). Patch histories when the
	// wall-clock gap between consecutive proc calls exceeds gap_trip
	// seconds; <= 0 disables.
	double gap_trip;
	double gap_dc_beta;	// EWMA weight for the per-axis phi_dc estimate
	double t_last;		// time of the previous proc call (s)
	size_t gap_events;	// stall gaps detected since start
	size_t n_closed;	// frames processed since the loop first closed
	// shared biquad coefficients for the command filter (normalized)
	double lp_b0, lp_b1, lp_b2, lp_a1, lp_a2;

	// per-axis controllers: [0] = y (element 0), [1] = x (element 1)
	struct aylp_fsp_axis axis[2];

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
