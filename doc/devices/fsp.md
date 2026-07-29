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

The plant seen by the loop is a gain `K`, an optional Bode-fitted
frequency-shaping biquad `H(z)`, and a possibly fractional transport delay
(camera latency + compute + DAC ZOH): `e = phi + K*H(z)*D(z)*u`, where `D`
contains `delay`/`delay_frac` and `phi` is the open-loop disturbance. Three
stages, after
Kulcsár 2006, Petit 2008, Meimon 2010, Correia 2010:

1. **Smith-predictor core.** Reconstruct the disturbance by removing our own
   delayed plant contribution: `phi_meas = e - K*H(z)*D(z)*u`. This takes the
   identified plant out of the disturbance estimate. When `plant_b`/`plant_a`
   are omitted, `H(z)=1` and this is the original scalar-gain model.
2. **Disturbance observer.** The modal path models `phi` as a sum of `n_modes`
   narrowband damped oscillators (`freqs`/`zeta`), each driven by white noise
   of variance `q`. A stationary Kalman gain (precomputed by iterating the
   Riccati recursion at init, and again on each adaptation tick) estimates the
   modal state from `phi_meas`. The `q`/`r` ratio is the LQG tuning knob.
   With `broad_order > 0`, the controller instead uses an identified
   full-band Wiener/Kalman FIR realization, covering both the broadband
   continuum and vibration peaks.
3. **delay-step prediction + minimum-variance control.** Roll the modal state
   forward `delay` steps and cancel it. With a shaped plant, the requested
   actuator-space correction is passed through the matched stable inverse:
   `u = H^-1(z)*(-phi_hat/K)`. Requiring both `H` and `H^-1` to be stable keeps
   this causal and prevents the fit from hiding an unstable plant inversion.
4. **Optional drift/transient split.** With `drift_tau > 0`, a slow EWMA state
   carries DC/random-walk pointing drift and the modal/FIR predictor sees only
   the remaining vibration. With `transient_sigma > 0`, an unexpectedly large
   prediction innovation freezes learning and cross-fades from predictive
   cancellation to a bounded proportional recentering servo. After the
   innovation remains quiet for `transient_hold`, the predictor is blended
   back over `transient_ramp`. This gives bumps a direct feedback path without
   permanently placing a second controller around the predictive loop.

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
- `delay`: loop transport delay in samples. Fit it from the full-band Bode
  phase slope plus the one hidden bode frame (2026-07-16 @ 3788 Hz: x 5.62,
  y 6.37 frames), NOT from the step-departure latency, which understates the
  effective delay. Beware the units when the frame rate changes: latency in ms
  can fall while the delay in FRAMES rises. Each axis object may set its own
  `delay`/`delay_frac`, overriding this global value (the two plants differ).
- `delay_frac`: fractional remainder of the transport delay. The Smith plant
  uses a first-order Thiran all-pass, preserving command magnitude while
  matching the fractional group delay; the full-band observer blends its
  adjacent horizon predictions. Per-axis override allowed as with `delay`.
- `fs`: loop rate (Hz); must match, so AR coefficients land on the right digital
  frequencies.
- `clamp`: command magnitude limit — the symmetric shorthand for
  `[-clamp, +clamp]`. Default 1.0.
- `clamp_min`, `clamp_max`: the actual output bounds, for an **asymmetric**
  limit. Either one overrides the corresponding side of `clamp`, in any config
  order. Use these when the actuator's reachable range is not centred on the
  command origin: a mirror spanning −10…+10 V driven from a +2.5 V bias at
  1 V/command-unit is `clamp_min: -12.5, clamp_max: 7.5`. A symmetric limit
  there would either give up half the negative range or let the loop request a
  voltage the hardware cannot produce.
  - The window must contain 0 (init fails otherwise): the `start_delay` hold,
    the `ramp` blend, the burst guard and the non-finite fallback all drive the
    command to zero, so a window excluding it would make those unrepresentable.
    Zero exactly on a boundary is allowed.
  - **Put the limit here, not in a downstream `anyloop:clamp` stage.** The
    clamped command is also what enters the fractional-delay filter and command
    ring, so bounding it here keeps this device's plant model identical to what
    the actuator received. A clamp *after* `fsp` would cut the command without
    `fsp` knowing, and that divergence is the command-echo mismatch that drives
    regeneration. A downstream `clamp` stage matched exactly to these bounds is
    fine as a backstop — just never tighter.
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
- `wiener_file`: optional offline initialization for the full-band predictor.
  The UTF-8 text file must contain exactly `broad_order` non-comment rows,
  indexed from zero, with columns
  `index y_w y_w_next x_w x_w_next`. Blank lines and lines beginning with `#`
  are ignored. The offline solve must use the same `broad_order`, per-axis
  horizon, `drift_tau`, and `broad_lp` preprocessing as the runtime. After
  loading, `broad_freeze_closed` and `broad_mu` determine whether the solution
  stays fixed or continues adapting.
- `broad_lp`: observer band-limit (odd boxcar taps on the NLMS input; 0 = off).
  The full-band observer is otherwise sensitive out to Nyquist, and any K/delay
  model error leaks command echo into its input as a *predictable* HF signal
  that closed-loop NLMS learns and chases into a self-sustained ring at the
  regeneration frequency (2026-07-22: a ~380 Hz x-axis limit cycle, 4.8 px rms,
  ignited ~1 min after close with same-day bode-matched K/delay — the residual
  mismatch is amplitude-dependent, so no static tune removes it). A `broad_lp`
  boxcar has its first null at `fs/broad_lp` and attenuates the whole
  regeneration band; being linear-phase with exactly integer group delay
  `(broad_lp-1)/2`, that delay is folded into the broad prediction horizon so
  in-band cancellation timing is unchanged (11 taps at 3788 Hz: null 344 Hz,
  <1 % droop at 30 Hz). This removes at the root what the burst guard only
  reacts to; verified 180 s with zero ring vs. a latched limit cycle without it.
- `drift_tau`: slow-drift EWMA time constant in seconds. `<= 0` preserves the
  original compound predictor. When enabled, the drift estimate is cancelled
  separately as constant over the command horizon and is subtracted before
  the boxcar/modal/FIR vibration observers. Start around 1--5 s: shorter values
  transfer more low-frequency motion out of the predictor, while longer values
  leave more of it in the learned vibration model.
- `transient_sigma`, `transient_floor`: enable large-event recovery and set its
  innovation threshold to
  `max(transient_sigma * quiet_innovation_rms, transient_floor)`, in normalized
  error units. `transient_sigma <= 0` disables the path. The quiet variance is
  learned with time constant `transient_tau` and is not updated during events.
- `transient_kp`: proportional gain for the fallback command
  `u = -(drift_hat + transient_kp * error) / K`; it must satisfy
  `0 < transient_kp < 1`. Preserving `drift_hat` prevents a bump from releasing
  the existing slow pointing correction. Keeping the feedback gain below one
  makes the scalar fixed-delay fallback stable while the clamp bounds command.
- `transient_tau`, `transient_hold`, `transient_ramp`: quiet innovation-variance
  time constant, required quiet time after the most recent threshold crossing,
  and bumpless cross-fade time back to predictive cancellation. NLMS and modal
  adaptation statistics are frozen throughout the event and return ramp.

A conservative first hardware trial (all values remain opt-in unless included):

```
"drift_tau":       2.0,
"transient_sigma": 6.0,
"transient_floor": 0.03,
"transient_kp":    0.25,
"transient_tau":   5.0,
"transient_hold":  0.10,
"transient_ramp":  0.25
```

Set `transient_floor` from a stable run rather than blindly keeping the example:
it should sit above ordinary single-frame centroid scatter but below the event
size that needs direct recovery. Count/logged activations should be rare.

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
- `guard_ratio`, `guard_floor`, `guard_hold`, `guard_ramp`, `guard_tick`:
  non-latching **burst guard** (default ON: 4 / 0.008 / 0.25 s / 1 s / 10 s;
  `guard_ratio <= 0` disables). Any mismatch between the configured plant
  model (`K`, `delay`) and the true plant leaks command back into the Smith
  reconstruction, closing a parasitic loop whose phase crosses −180° at
  `fs / (2·(delay + delay_frac))` (~316 Hz y / ~337 Hz x at 3788 Hz). When
  the margin there goes negative — `K` drifts with the coarse bias and
  alignment — the loop emits intermittent ring bursts at that frequency
  (the 2026-07-16 "310 Hz spiral"; the 2026-07-17 run-15 rerun, where the
  bursts averaged into a broad 250–450 Hz PSD hump and tripled the closed
  RMS while the code was identical to the good morning run). Per axis, the
  guard band-passes the raw error at the regeneration frequency (Q 1.5),
  tracks a ~10 ms envelope against a ~10 s quiet baseline, and triggers when
  the envelope exceeds `guard_ratio` × max(baseline, `guard_floor`). On a
  trigger the axis's authority is cut to 0, NLMS training and the adaptation
  statistics freeze (so the ringing is never learned), the cut holds for
  `guard_hold` s (extended while the ring persists), then authority ramps
  back over `guard_ramp` s; a burst returning mid-ramp counts as a new
  event. Every `guard_tick` s of closed-loop time a ticker line reports the
  activation count and the percentage of frames spent at reduced authority,
  and a final total is logged at shutdown. The guard keeps a marginal run
  alive; **recurring activations mean `K`/`delay` no longer match the plant
  at the current operating point — re-run the Bode fits before trusting the
  attenuation numbers.**
- `gap_trip`: **stall-gap guard** (seconds; default 0.05, `<= 0` disables).
  If the wall-clock gap between consecutive frames exceeds this — e.g. the
  ASI camera stream stalled and asi_source restarted capture, a ~0.3–0.6 s
  hole during which the DAC held the last command while the error kept
  moving — every predictor history now spans the hole and re-entering at
  full authority onto stale state is exactly the transient that seeds a
  regeneration burst. An authority *cut* is wrong too: the command carries
  the loop's DC correction, and zeroing it snaps the FSM back to bias,
  re-exposing the accumulated drift as a multi-px excursion that the ramp
  then walks back slowly (observed 2026-07-17: ~6 px for ~1.3 s). Instead,
  each axis **keeps commanding the value the DAC held through the gap**
  (zero bump at resume) for `guard_hold` s while the predictor state is
  rebuilt — modal state zeroed (mode phases rotated unpredictably), NLMS
  input history reset so it never predicts or trains across the hole, Smith
  command ring rewritten with the held command — then **blends from the
  held command to the live controller command** over `guard_ramp` s.
  Learning stays frozen until the blend completes; the safety-trip latch
  overrides the blend. Works even with the burst detector disabled. Gap
  events are counted and logged separately from burst events: **bursts mean
  re-measure `K`; gaps mean the camera stalled** (check the asi_source
  recovery lines).
- `y`, `x`: per-axis objects (element 0 = y, element 1 = x), each with:
  - `K`: **signed** command→error gain. Wrong sign = positive feedback =
    runaway; verify with a push test.
  - `plant_b`, `plant_a`: optional three-coefficient numerator and denominator
    of the per-axis Bode residual `H(z)=(b0+b1*z^-1+b2*z^-2)/
    (a0+a1*z^-1+a2*z^-2)`. Both arrays must be supplied together. FSP
    normalizes `a0`, verifies that all poles and zeros lie inside the unit
    circle, uses `H` in the Smith reconstruction, and uses the matched `H^-1`
    on the cancellation command. Omit both for the original `H=1` behavior.
    Run 16 coefficients are fits to `fsm_bodex_716`/`fsm_bodey_716` after
    removing the run-15 `K`, transport delay, and the Bode helper's hidden
    frame; they cut 5–300 Hz complex plant-model RMS error from 8.80% to
    4.39% on x and 8.31% to 3.46% on y.
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
