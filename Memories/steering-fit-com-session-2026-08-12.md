# Steering and fit_com session — 2026-08-12

## Decisions retained

- Steering uses `fit_com` only. `wfs_com` is not part of the registered test
  suite or the live steering pipeline.
- The current camera mode is ASI290MM, 112x112 ROI at `(960, 524)`, 400 us
  exposure, gain 120, bandwidth 100, with a measured loop rate of 1457 Hz.
  Changing ROI changes frame rate and therefore requires delay revalidation.
- Illumination tuning increases exposure first, but only to the point before it
  reduces the frame rate/increases loop delay; gain is increased afterward.
- `fit_com` uses whole-frame 112x112 tracking, `sigma_init=11.799`,
  `sigma_min=3.54`, `sigma_max=47.197`, `min_amplitude=5`, 12 iterations,
  300 us cap, robust moment output, and PWM/rolling-shutter inference.
- Automatic pre-centering learns the slow `drift_hat`, waits 3 s, ramps the DC
  correction for 10 s with up to +/-0.6 V, and keeps that bias when the fast
  loop closes. Fast authority is +/-0.1 V relative to the learned DC bias.
- Current plant values are `Ky=0.45`, delay `2+0.664` frames and `Kx=0.25368`,
  delay `2+0.298` frames. Re-measure gain after optical realignment; preserve
  the delay unless the ROI, exposure, tracker cost, or camera rate changes.

## Verified runs

The 95 s automatic-centering run used 138,415 frames. It moved the initial
6.76 px radial offset to 0.49 px before fast closure. Over the 60 s closed
window the vector-mean radial offset was 0.073 px and jitter RMS was 2.36 px Y,
1.74 px X. It had zero guard activations, zero beam losses, and 11 rejected
frames.

The matched A/B run on 2026-08-12 used 201,100 frames at 1457 Hz:

| metric | open 60 s | closed 60 s | improvement |
|---|---:|---:|---:|
| Y jitter RMS | 3.804 px | 3.048 px | 19.9% |
| X jitter RMS | 2.810 px | 1.639 px | 41.7% |
| combined RMS | 4.736 px | 3.497 px | 26.2% |
| median radial error | 3.688 px | 2.077 px | 43.7% |
| mean radial offset | 0.263 px | 0.498 px | -89.6% (worse) |

Both one-minute score windows had zero rejected frames. The controller had zero
guard activations and no beam loss; 14 frames in the complete 138 s acquisition
were unmeasurable and held safely. The result is stable but does not meet a 2x
combined-RMS target; Y rejection and slow closed-loop centering remain the main
limitations.

Reproduce the calculation with:

```sh
python3 contrib/analyze_steering_ab.py steering_par_err.aylp \
  --fs 1457 --pixel-scale 55.5 --open-start 13 --closed-start 78
```
