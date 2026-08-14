# Steering, pulsed COM, and frame-gap session — 2026-08-13

This record supersedes the 2026-08-12 memory wherever the camera mode,
tracker, frame rate, gain, delay, or current run configuration differs.

## Current continuous and 1000-Hz-integrated setup

- Camera: ASI290MM, 384x384 ROI at `(1032, 276)`, gain 0, USB bandwidth 100,
  high-speed mode, measured about 471 Hz (2.12 ms/frame). The validated 1-kHz
  run uses a 1000-us exposure; the continuous and 1-Hz commissioning configs
  use 2000 us.
- Tracker: thresholded `center_of_mass`, tracking window 243x243, threshold 1,
  min peak 3, initialized at `(178,177)`. Output is normalized over the full
  384-pixel image, so the conversion is `(384-1)/2 = 191.5 px/unit`.
- FSP plant values in the current configuration: global delay `2+0.36`
  frames, `Ky=0.152174`, `Kx=0.261818`. These are operational values, not a
  clean new closed-loop identification.
- Normal fast clamp is +/-0.5 V relative to the learned DC operating point.
  Commands near 1 V can therefore be legitimate when the learned DC bias is
  near +0.5 V. Transient mode is disabled (`transient_sigma=0`).
- Broadband controller is the existing 512-tap NLMS: `broad_order=512`,
  steady `broad_mu=0.03`, startup schedule 0.5 -> 0.03 over 60 s, and
  `broad_lp=5`. This was enabled after finding that 84% of Y and 89% of X
  open-loop variance was below 10 Hz while `broad_order=0` had left that
  broadband motion untreated.

## Continuous-beam results and interpretation

- Modal/drift-only steering reduced total normalized RMS from about 0.143 open
  to 0.0334 late closed (~4.3x total-error reduction) and nearly eliminated
  mean offset. Its jitter-only improvement was smaller (~1.6x).
- With the 512-tap NLMS enabled, the live viewer eventually showed about
  3.8-3.9 px RMS per axis and ~5.3 px combined over recent 60 s windows, with
  mean offset near zero. Early adaptation temporarily traded sub-1-Hz
  improvement for extra 3-100-Hz energy; meaningful scoring must wait until
  `mu` approaches 0.03.
- The live viewer reports per-axis RMS about the mean over its recent display
  window. It does not report two-axis total RMS and it excludes mean offset.
  This explains why a viewer value near 4 px can coexist with a larger
  combined or longer-window number.
- Clamp-warning spam is not the same as true rail occupancy. `trip_command`
  checks absolute requested command, while the real fast clamp is centered on
  learned DC. In the modal-only run true relative rail occupancy was tiny; in
  the broadband run it became material (roughly 2-4% near the +/-0.5-V fast
  envelope), showing that the NLMS demanded more authority.
- Passive closed-loop error/command data cannot uniquely identify gain and
  delay here. ARX fits selected an impossible zero-delay command term because
  the controller makes current command correlated with current error. Fits
  produced model-dependent gains/delays, so no K/delay change was accepted.
  Use an independent small PRBS/multisine/probe for defensible identification.

## Frame-gap diagnosis

- `gap_trip=0.002` was invalid at 471 Hz because a normal frame is 2.12 ms.
  Current configs use 0.0032 s: normal frames pass, a one-frame miss at about
  4.24 ms is detected.
- Measured gaps are real late camera deliveries, usually 4-8.3 ms and often
  exactly 8.1-8.3 ms (the ASI SDK timeout signature), not long stream stalls.
  ASI diagnostics normally report 470-471 Hz and no capture restart.
- A major launcher bug was fixed: launching the whole process under
  `chrt -f 80 taskset -c 2` made every ASI/libusb worker inherit FIFO 80 and
  CPU 2, forcing camera workers to compete with the loop. The recorded runner
  now starts normally, then promotes/pins only the main anyloop thread.
- Promoting/separating camera worker threads did not remove the remaining
  gaps and was reverted. Three cameras and USB Wi-Fi share the xHCI controller;
  USB/controller contention remains the strongest untested external cause.
- `udp_sink` now ignores loopback `ECONNREFUSED` when an optional viewer is
  absent, preventing synchronous log spam in the real-time path.
- Even a run with 391 gap events lost/padded only 782 of 671451 frames (~0.12%,
  0.17% wall time). Such data remains useful for RMS, centering and low-frequency
  rejection, but not precise high-frequency phase/delay claims.

## Run recording

- `contrib/steering/tools/run_steering_recorded.sh [label]` records the console log and creates
  a per-run record containing the starting config, code/config diff, timestamps,
  exit status, and associated AYLP outputs.
- Error and command streams are both recorded. File sinks rotate earlier files,
  so do not infer chronology from suffix alone; use modification timestamps and
  the run record/log.

## Measured results by optical waveform

These are measured live results, not simulations. Comparisons are within-run
open/closed A/B measurements wherever available; numbers from different camera
modes should not be compared as though their pixel noise floors were identical.

| beam | camera/tracking condition | measured open -> closed result | conclusion |
|---|---|---|---|
| 100 Hz, 80% duty | validated transferred live configuration | Y jitter 3.804 -> 3.048 px (19.9% lower); X jitter 2.810 -> 1.639 px (41.7% lower); combined RMS 4.736 -> 3.497 px (26.2% lower); median radial error 3.688 -> 2.077 px (43.7% lower) | Stable useful attenuation, but mean radial offset worsened 0.263 -> 0.498 px. Score windows had no rejected frames, guard trips, or beam loss. |
| 1000 Hz, 20% duty, 1 ms exposure | 384x384, measured 471 Hz; exposure integrates a complete optical cycle | Y jitter 7.107 -> 4.650 px (34.6% lower); X jitter 7.404 -> 6.340 px (14.4% lower); combined RMS 10.642 -> 7.863 px (26.1% lower); mean radial offset 2.817 -> 0.126 px (95.5% lower) | The integrated 1-kHz beam behaves like a continuous beam for COM. It gave useful but axis-asymmetric jitter rejection and nearly eliminated offset. |
| 1 Hz, 20% duty | 384x384, measured 471 Hz; brightness-gated thresholded COM; only illuminated frames scored | Settled +/-0.3-V window: total RMS 19.73 -> 8.77 px (2.25x lower), combined jitter 12.89 -> 8.39 px (1.54x lower), with strong recentering | Promising commissioning result. End-of-file beam-loss samples are excluded because the beam was intentionally turned off. |

The 100-Hz result is the archived matched A/B associated with the validated
configuration (`backup_live_validated_40pct_x_20pct_y.json`); “40% x / 20% y”
in that filename refers to the controller attenuation result/label, not the
optical waveform duty cycle. The source condition was 100 Hz at 80% duty.

The exact 1000-Hz A/B is preserved in `steering_ab_1000hz_20pct.json`. Its
open/closed score windows contained 6123/11775 frames and zero rejected frames.
That particular short A/B used `broad_order=0`. Broadband control was then
tested separately on the same 1-ms, 471-Hz integrated 1-kHz optical setup: the
preserved run `data/steering_runs/last_night_2026-08-12_to_2026-08-13/02-05-00_beam-1000hz-20pct_controller-nlms512/config_used.json`
has `broad_order=512`, `broad_mu=0.03`, and `broad_lp=5`. Therefore “the 1-kHz
beam was only tested modal-only” is false; only the compact A/B JSON is
modal-only. The later broadband run is the source of the approximately
3.8-3.9 px per-axis / 5.3 px combined late live-view result discussed above,
with the frame-gap caveats in this record.
The live push tests also measured configured plant magnitudes `Kx=0.261818`
and `Ky=0.152174` normalized-error units per volt (about 50.14 and 29.14 px/V),
with fit R-squared 0.996 and 0.973 respectively. Those gains were measured
cleanly; the current `2+0.36`-frame delay remains an operational estimate rather
than an equally clean independent identification.

## Canonical run configurations and provenance

As of the final audit on 2026-08-13, each canonical run JSON contains the latest
most successful configuration that was actually exercised for that waveform:

| canonical run | preserved source/evidence | controller now encoded |
|---|---|---|
| `run_100hz_80pct.json` | `backup_live_validated_40pct_x_20pct_y.json` | The validated modal/drift-only run: 112x112, 400 us, 1457 Hz, global delay 2.298 frames, Y delay 2.664 frames, +/-0.5 V, `broad_order=0`. Its measured result was 41.7% X and 19.9% Y jitter attenuation. NLMS was not tested on this waveform, so it was not invented here. |
| `run_1000hz_20pct.json` | `data/steering_runs/last_night_2026-08-12_to_2026-08-13/02-05-00_beam-1000hz-20pct_controller-nlms512/config_used.json` | The later live 1-ms/471-Hz integrated-beam run: 512-tap NLMS, `mu` 0.5 -> 0.03 over 60 s, `broad_lp=5`, +/-0.5 V, 25-s open hold and 20-s precenter delay. The late viewer reached about 3.8-3.9 px RMS per axis and about 5.3 px combined; frame-gap caveats still apply. The earlier exact short A/B (26.1% combined RMS reduction and 95.5% offset reduction) was modal-only. |
| `run_1hz_20pct.json` | final +/-0.3-V sparse-beam commissioning run | 384x384, 2 ms/471 Hz, `min_peak=100`, `ref_cut=0`, `reacquire_after=600`, hole-aware 512-tap NLMS with 0.10 -> 0.03 step, and +/-0.3 V. Illuminated-frame total RMS was 19.73 -> 8.77 px (2.25x lower); intentional beam-off samples are excluded. |

Calibration suites may replace measured camera/COM, gain, and delay fields after
a fully passing calibration. They do not replace the waveform's chosen NLMS
order, step schedule, low-pass length, authority, or commissioning timeline.

## 1 Hz, 20% duty-cycle COM lessons

The active configuration is:

`contrib/steering/configurations/run_1hz_20pct.json`

The source waveform is external; the JSON configures camera, tracking,
controller and recording only.

At 471 Hz a 20%-duty 1-Hz source gives roughly 94 bright frames followed by
about 377 dark frames. During darkness COM must hold the last valid centroid;
the resulting step-like viewer trace is sample-and-hold, not continuous
measurement.

### Failed COM approaches

1. `threshold=1`, `min_peak=3`, no partial-frame gate accepted marginal rolling-
   shutter transition frames. A transition centroid could jump Y to about -0.5
   normalized, then be held through all ~376 dark frames. The error was mainly Y
   because the rolling shutter cuts along sensor rows.
2. `ref_cut=0.5`, `ref_floor=0.25`, warmup 200 badly over-rejected: only 73/6059
   frames were valid and 3473 carried `BEAM_LOST`.
3. A relaxed core gate (`ref_cut=0.2`, `ref_floor=0.75`, warmup 50) still accepted
   only 5.8% instead of ~20% and asserted beam loss. The reason is structural:
   the beam moves while invisible for 0.8 s, so a full returning beam is shifted
   relative to the stored row profile and looks like a cut beam.

### Working COM settings

Use brightness separation rather than a stored row profile:

```json
"threshold": 1,
"min_peak": 100,
"ref_cut": 0.0,
"reacquire_after": 600
```

Observed full bright frames had peaks mostly 166-255; marginal transitions were
commonly 3-55. This configuration produced 18.0% valid frames (expected 20%),
82.0% `NO_SIGNAL`, and zero `BEAM_LOST`. Typical seconds contained 94-95 valid
frames, almost exactly the theoretical bright count.

`NO_SIGNAL` during the 80% dark interval is correct and must not be described as
beam loss. A real integration bug was fixed in `fsp.c`: FSP now routes both
`AYLP_FRAME_REJECTED` and `AYLP_NO_SIGNAL` through its measurement-hole path, so
held centroids cannot train observers or update control state as hundreds of
identical fresh measurements. `test_fsp_dark` passes after this change.

## 1 Hz modest closed-loop commissioning

The copied 1-Hz config initially remained open because it contained
`start_delay=2500 s`. It now uses:

- `start_delay=20 s`
- `precenter_delay=10 s`, `precenter_ramp=10 s`
- loop ramp 10 s
- 512-tap NLMS retained
- sparse-data startup `broad_mu_init=0.10`, steady `broad_mu=0.03`
- `dark_train_min_real=0.15`, `dark_fab_frames=500`,
  `dark_predict_max=400`

The +/-0.2-V fast-authority trial was stable:

- bright-frame total RMS: 21.65 -> 12.96 px (~1.67x)
- most improvement was recentering
- Y jitter: 10.19 -> 10.07 px; X jitter: 4.71 -> 4.32 px
- commands spent much of the closed interval at the modest envelope

The scored part of the next +/-0.3-V trial was a clear improvement:

| metric | open bright frames | late closed bright frames | result |
|---|---:|---:|---:|
| total RMS | 19.73 px | 8.77 px | 2.25x lower |
| Y mean | +12.96 px | +2.55 px | strongly recentered |
| X mean | -7.42 px | +0.17 px | essentially zero |
| Y jitter RMS | 11.79 px | 7.39 px | 37% lower |
| X jitter RMS | 5.22 px | 3.98 px | 24% lower |
| combined jitter | 12.89 px | 8.39 px | 1.54x lower |

In that score window the +/-0.3-V run had about 18.3% valid illuminated frames,
zero beam-loss events, zero guard activations, and NLMS was still converging
(`mu` about 0.043). Beam-loss samples recorded while the source was being
turned off at the end are an operator action, not a tracking failure, and are
excluded from the performance result.

Because the clamp is relative to learned DC, applied absolute command ranges can
reach roughly +/-0.6 V in the +/-0.3-V run. This is expected and transient mode
remains disabled.

## Operational rules retained

1. Always distinguish `NO_SIGNAL`, `BEAM_LOST`, distorted-frame rejection, and
   timing gaps. They imply different fixes.
2. For low-duty pulsed light, score only genuinely bright frames when measuring
   optical pointing. Also report the held all-frame series separately because it
   describes what downstream consumers see.
3. Do not tune a position-sensitive reference profile across long invisible
   intervals when the beam can move during the hole.
4. Do not train adaptive control on held centroids. Status propagation is part
   of the controller, not merely logging metadata.
5. Evaluate NLMS only after its startup step has decayed; early spectral trades
   can differ substantially from steady behavior.
6. Raise authority incrementally and measure jitter separately from mean-offset
   improvement. A lower total RMS can otherwise be mistaken for better dynamic
   rejection when it is only recentering.
7. Do not infer plant gain/delay from naive passive closed-loop regression.
8. Preserve exact run configuration, console diagnostics, error and command
   streams together.
