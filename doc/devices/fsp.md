anyloop:fsp
===========

Types and units: `[T_VECTOR, U_ANY] -> [T_UNCHANGED, U_command]`.

Adaptive **Filtered Smith Predictor** / adaptive LQG for tip-tilt beam
stabilization. Unlike `anyloop:kalman_filter` (a predictor that pre-processes
the error for a downstream `pid`), this is a **complete controller**: error in,
command out. It **replaces both `kalman_filter` and `pid`** — running it with
either double-compensates the loop delay and pumps the vibration lines. Place
it where `pid` used to sit: after `center_of_mass` and any logging sink, before
`clamp`.

The plant seen by the loop is a gain `K` plus a possibly fractional transport
delay (camera latency + compute + DAC ZOH): `e(k) = phi(k) +
K*((1-f)*u(k-delay) + f*u(k-delay-1))`, where `f=delay_frac` and `phi` is the
open-loop disturbance. Three stages, after
Kulcsár 2006, Petit 2008, Meimon 2010, Correia 2010:

1. **Smith-predictor core.** Reconstruct the disturbance by removing our own
   delayed plant contribution: `phi_meas(k) = e(k) - K*u(k-delay)`. This takes
   the delay out of the loop's characteristic equation.
2. **Disturbance observer.** The modal path models `phi` as a sum of `n_modes`
   narrowband damped oscillators (`freqs`/`zeta`), each driven by white noise
   of variance `q`. A stationary Kalman gain (precomputed by iterating the
   Riccati recursion at init, and again on each adaptation tick) estimates the
   modal state from `phi_meas`. The `q`/`r` ratio is the LQG tuning knob.
   With `broad_order > 0`, the controller instead uses an identified
   full-band Wiener/Kalman FIR realization, covering both the broadband
   continuum and vibration peaks.
3. **delay-step prediction + minimum-variance control.** Roll the modal state
   forward `delay` steps and cancel it: `u(k) = -phi_hat(k+delay|k)/K`.

This is the one in-loop mechanism that beats the delay-limited crossover on the
**predictable** (modal) part of the disturbance. It does nothing for the
broadband floor or above-bandwidth content — a fundamental limit.

Adaptation (`adapt_period > 0`): every `adapt_period` s, refresh each mode's
`q` from its recent estimated energy, `r` from the innovation floor, nudge each
center frequency toward the locally demodulated line (capped at `adapt_df_max`
Hz/update, sized to the ~0.5 Hz/run wander), rebuild coefficients, and recompute
the gain. Set `adapt_period <= 0` for a fixed FSP while first validating.

Parameters
----------
- `type`: must be `"vector"`.
- `units`: output units (e.g. `"minmax"`).
- `delay`: loop transport delay in samples. The latest Bode fit is 1.81 ms,
  or 4.18 frames at 2310 Hz; use 4 for the discrete controller.
- `delay_frac`: fractional remainder of the transport delay. The Smith plant
  uses a first-order Thiran all-pass, preserving command magnitude while
  matching the fractional group delay; the full-band observer blends its
  adjacent horizon predictions. Use 0.18 with `delay=4` for the 1.81 ms Bode
  fit at 2310 Hz.
- `fs`: loop rate (Hz); must match, so AR coefficients land on the right digital
  frequencies.
- `clamp`: command magnitude limit.
- `start_delay`, `ramp`: hold the command at 0 for `start_delay` s (Kalman
  converges on the clean open-loop error), then blend to full authority over
  `ramp` s.
- `adapt_period`, `adapt_df_max`, `adapt_tau`: adaptation cadence, per-update
  frequency-correction cap, and EWMA time constant.
- `cmd_fc`: optional 2nd-order low-pass on the command with per-mode
  roll-forward phase compensation (0 = off, the default). Leave it off: the
  `fsp_sim.py` study showed the waterbed does **not** come from HF command
  noise, so the filter costs prediction horizon without buying anything. The
  real waterbed fix is the q/r scaling below.
- `broad_order`, `broad_mu`: enable the full compound-disturbance observer.
  The observer learns the delay-step conditional mean directly from the
  Smith-reconstructed disturbance using an order-`broad_order` FIR
  realization of the scalar Wiener/Kalman predictor, updated by NLMS with
  step `broad_mu`. This supplies the command instead of the modal estimate
  (the two must not be summed). Set `broad_order` to 0 for the original
  modal-only controller. Attenuation12 uses order 128 and `broad_mu=0.005`;
  attenuation11 showed that 512 taps at 0.03 preserved more high-frequency
  coefficient noise than the additional prediction accuracy justified.
- `broad_freeze_closed`: freeze full-band identification when the startup hold
  ends (default true). This is required for safe feedback operation: identify
  while command is known to be zero, then apply a fixed observer. Continuous
  closed-loop NLMS can identify leaked command as disturbance when `K` or the
  delay model is imperfect and thereby create positive feedback.
- `trip_error`, `trip_command`, `trip_frames`: latched safety limits. During
  the startup hold FSP learns each axis's ordinary open-loop operating point.
  After closing, it trips if error magnitude exceeds the learned open-loop
  magnitude by `trip_error`, or if requested (pre-clamp) command exceeds
  `trip_command`, for `trip_frames` consecutive samples. FSP then commands
  zero until restart. This magnitude-envelope test permits successful motion
  from a static offset toward zero without mistaking the offset for runaway.
  The current configs use 0.05 error units, 0.65 command units, and 8 frames.
- `y`, `x`: per-axis objects (element 0 = y, element 1 = x), each with:
  - `K`: **signed** command→error gain. Wrong sign = positive feedback =
    runaway; verify with a push test.
  - `r`: measurement-noise variance.
  - `freqs`: mode center frequencies (Hz), up to `AYLP_FSP_MAX_MODES` (8).
  - `zeta`: per-mode damping (~0.002 for a sharp line, ~0.3 for a broadband
    hump). Defaults to 0.002.
  - `q`: per-mode process-noise (**drive**) variance (array), or a scalar for
    all. Defaults to 1e-5. NOT the mode's visible energy: drive variance =
    state energy / Gv, where Gv ≈ 1/(4π·zeta·f·Ts) is huge for sharp lines.

Tuning: q/r is the waterbed dial (`fsp_sim.py` on the measured attenuation10 PSD)
---------------------------------------------------------------------------------
The rejection transfer is `R(ω) = 1 − e^{−jωd}·T(ω)`, with `T` the
measurement→command transfer. A conservation law identical to the feedback
Bode integral applies (`∫ ln|R| dω = 0` for stable causal `T`): in-band
rejection must be paid for out of band. The q/r ratio positions the design on
that tradeoff curve:

- **q/r ~ 1** (mis-scaled): the estimator tracks *everything*, `|T| ≈ 1` far
  out of band, and the delay phase mismatch alone makes
  `|R| = 2|sin(ωd/2)| ≈ 2–4` — a ~3× waterbed at 64–400 Hz. The Kalman gain
  also inflates (close-spaced mode pairs fight over the shared measurement —
  same pathology as the pid notes' "keep oscillators >2 Hz apart").
- **q/r ~ 1e-5** (physical): waterbed ~×1.5 — the same as the classical
  pid+lead loop — with the 10–30 Hz band rejected ~2× harder (sim: x 0.060 px
  vs 0.137 measured). Total RMS lands **at** the classical loop's
  (x 0.241 vs 0.244, y 0.161 vs 0.151 px): the conservation law means the FSP
  *redistributes* error (deeper lines, same waterbed) rather than shrinking
  the total on this disturbance mix.
- **q/r ~ 3e-6**: waterbed ~×1.35 at softer in-band rejection.

Corollary: don't judge the FSP by total RMS alone — its value over the
classical loop is *where* the residual sits (line depth vs broadband), plus
single-knob retuning. For a smaller total, the delay itself must shrink (QPD
front-end); no in-loop tuning escapes the conservation law.

The adaptation refreshes q from the measured mode state energy **divided by
Gv** (the stationary variance gain) — feeding state energy in directly
re-creates the q/r ~ 1 regime and its ×3 waterbed.

Validate with `contrib/attenuation_fsp.json` (open-vs-closed A/B) and compare
the closed RMS-about-mean and the 120–400 Hz band ratio against
`attenuation10_lead_fc64` (x 0.244 px / ×1.57, y 0.151 px / ×1.32).
