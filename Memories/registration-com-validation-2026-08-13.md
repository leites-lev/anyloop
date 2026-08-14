# Registration COM validation — 2026-08-13

## Outcome

The optional `center_of_mass` registration path is now ROI- and beam-agnostic
and is validated offline against all 16 repository camera time-series captures
(112, 256, 320, and 384 pixel ROIs, including 1 Hz data). The implementation is
in `devices/center_of_mass.c`; offline coverage is in
`devices/center_of_mass_test.c`.

The tracker uses brightness-affine robust image translation rather than assuming
a Gaussian spot or fixed beam diameter. Sampling capacity comes from ROI size,
noise and conditioning are measured from each keyframe, and the robust residual
scale is estimated per frame.

## Important fixes

- Removed fixed ROI/sample assumptions; feature storage and sample budgets derive
  from the configured window.
- Increased stable feature support to `4*ceil(sqrt(candidate_count))`: normally
  432 samples at 112 pixels and 1520 at 384 pixels.
- Use keyframe gradients so rolling-shutter/exposure edges appearing only in the
  destination cannot steer the translation fit.
- Fit per-frame gain and offset as nuisance parameters and use Tukey-robust
  residual weighting.
- Corrected the robust refinement to solve from the actual residual rather than
  applying the full displacement twice.
- Retain the original keyframe while it has sufficient two-axis overlap;
  unnecessary rolling-keyframe chains had integrated small errors into tens of
  pixels of false motion.
- The independent background-subtracted centroid tether now uses a robust border
  background and is disabled when beam signal touches the ROI border. A clipped
  centroid is not an absolute position measurement.
- When correspondence is genuinely lost, reacquire a keyframe from a valid,
  unclipped centroid; otherwise hold the previous result.

## Offline tests

### Known translations injected into real frames

For each of the 16 camera files, select its highest-flux real illuminated frame,
retain its sensor noise/beam shape/reflections/saturation, and inject a closed
subpixel trajectory (`dy=4 sin(t)`, `dx=3 sin(2t)`) plus varying global gain and
background. Check RMS, recovered translation gain, maximum error, and terminal
closed-loop bias.

Final results:

- Per-axis RMS range: **0.0042–0.0403 px**.
- Combined radial RMS range: **0.0080–0.0425 px**.
- Median radial RMS: **0.0128 px**.
- Pooled radial RMS: **0.0164 px**.
- Total radial spread across files: **0.0345 px**.
- Worst file: 1 Hz `2026-08-13T00-50-45_camera_capture.aylp`, **0.0425 px radial RMS**.
- Best file: `2026-08-13T00-12-06_camera_capture.aylp`, **0.0080 px radial RMS**.
- Recovered gains across the earlier final sweep were essentially unity; closed
  trajectories returned with negligible terminal bias.

### Real-frame forward/backward and composition consistency

On a bright consecutive triple from every camera file, independently measure
A→B/B→A and B→C/C→B, then compare A→B→C with direct A→C. This requires no claim
that apparent camera motion is physical beam motion.

Final results over 16 files:

- Forward/backward closure RMS: **0.110 / 0.092 px**.
- Worst forward/backward closure: **0.197 px**.
- Three-frame composition RMS: **0.360 px**.
- Worst composition error: **0.652 px**.
- Gates: each inverse closure below **0.25 px** and composition below **0.75 px**.
  Composition is looser because three actual frames can evolve non-rigidly; the
  production tracker measures frames against its retained keyframe rather than
  dead-reckoning through pairwise increments.

### Recorded movement COM

The recorded `0 V` movement file contains 500/500 `NO_SIGNAL` samples, so it
cannot honestly provide an actuator-position baseline. The `+0.5 V` file has
500/500 valid samples: mean `0.3015/-0.1728`, RMS spread `0.0349/0.0461`, and
first-half to second-half drift `0.0064/0.0498`. The offline test validates these
flags, finite output, resolved offset, stability, and drift without inventing an
axis/sign assertion from the beamless file.

All eight Meson suites passed after the final tracker changes.

## Predictive-controller implication

Tracker measurement error (pooled radial RMS about 0.016 px) is negligible
relative to the control residual, so additional COM precision is not the main
control limitation. The recorded 2.1 ms-delay linear-prediction bound is:

- X: `0.69 → 0.40 px RMS` ideal residual (about 42% reduction).
- Y: `0.27 → 0.15 px RMS` ideal residual (about 44% reduction).

Previous live results reached about `0.91 → 0.59 px` on X with prediction plus
modal cancellation. Prediction hurt Y by pumping a line, so Y passthrough near
0.23 px was safer. A realistic optimized target is therefore approximately
`0.4–0.6 px RMS` X, `0.15–0.23 px RMS` Y, or `0.45–0.64 px` combined radial RMS.
Focus subsequent optimization on latency compensation, axis-specific predictor
blending, vibration-line cancellation, and avoiding broadband noise/waterbed
amplification rather than further tracker precision.
