anyloop:fit_com
================

Types and units: `[T_MATRIX_UCHAR, U_ANY] -> [T_VECTOR, U_MINMAX]`.

A **joint spatio-temporal beam fit**. Output convention (normalized to the
whole image, `[-1, 1]`, 0 = image centre) matches `anyloop:center_of_mass` and
`anyloop:wfs_com` exactly, so it is a drop-in replacement upstream of
`anyloop:fsp`/`anyloop:pid`.

Why
---

A rolling-shutter frame does not have *a* beam position. Each row is exposed at
a different instant, so if the beam moves during readout the frame is sheared,
and any single position extracted from it is ill-defined — there are as many
answers as there are rows.

`center_of_mass` ignores this and reports the first moment of the smear.
`wfs_com` measures a position and then tries to undo the shear afterwards, by
fitting a line through 2–3 row-group means — two parameters from three points,
with no redundancy, so the fitted slope is mostly noise (see
`doc/devices/wfs_com.md`, which documents that correction measuring net-harmful
and being disabled by default).

This device puts the motion **inside the model** and fits it directly:

```
I(row,col) = bg + amp * exp(-[(row - y(row))^2 + (col - x(row))^2] / 2 sigma^2)

  y(row) = y0 + slope_y * (row - ref_row)
  x(row) = x0 + slope_x * (row - ref_row)
```

Seven parameters — `y0, x0, slope_y, slope_x, sigma, amp, bg` — against every
pixel in the window. On a 20x20 window that is 400 observations for 7 unknowns,
against the row regression's 3 for 2. The same physical quantity, estimated
with two orders of magnitude more data.

The consequence that matters: **shear stops being a defect to detect and reject,
and becomes the signal that identifies the slope.** Nothing is ever discarded
for moving. That removes an entire failure mode, because a shear threshold is
necessarily a *velocity* threshold — shear is proportional to speed — so it
preferentially drops frames at the fast part of every oscillation and keeps the
turning points. Those dropouts are correlated with the disturbance phase, which
means the loop runs open (holding its last command) exactly when the beam is
moving fastest, and any adaptive predictor downstream identifies its model from
a phase-biased sample.

Measured against `wfs_com` on the same synthetic sheared scene (3 px @ 60 Hz,
32x32 ROI, fs 3788):

| | raw RMS | debiased RMS | bias | us/frame |
|---|---|---|---|---|
| `fit_com` | 0.0043 px | 0.0043 px | (-0.000, -0.000) | 62 |
| `wfs_com` | 0.6602 px | 0.1099 px | (+0.456, +0.465) | 13 |

The bias column is the second structural difference. `wfs_com` measures
displacement relative to a reference captured at acquisition, so whatever
offset the beam had at that instant is absorbed and reported as zero. This
device solves for absolute position every frame, so there is no anchor to
absorb anything and no acquisition-time offset to carry.

Algorithm
---------

Per frame:

1. Place the window (whole image by default) around the previous position.
2. Warm-start the parameter vector from the previous frame. Between frames the
   beam moves far less than its own width, so the solver begins inside the
   basin and typically converges in one or two iterations.
3. Levenberg–Marquardt with Marquardt (diagonal) scaling, so parameters with
   very different natural scales — a background in counts against a slope in
   px/row — are damped comparably. The damping is carried between frames.
4. From the second iteration, reweight pixels by a **Tukey biweight** on the
   residuals: a stray reflection or hot pixel is excluded outright. Because the
   model already contains the motion, a *sheared* beam fits and is never
   mistaken for an outlier.
5. Gate the result. A fit whose amplitude has collapsed, whose sigma has hit a
   bound, or whose residual exceeds `max_residual` is rejected: the last valid
   output is held and `AYLP_FRAME_REJECTED` asserted. After `reacquire_after`
   consecutive failures, `AYLP_BEAM_LOST` is asserted and the parameters are
   re-seeded from the image.

Acquisition seeds position from the brightest pixel (or `init_y`/`init_x`),
background from the window border, and sigma from a **local** radial second
moment taken over a box a few `sigma_init` wide. That locality matters: over
the whole window a stray reflection inflates the sigma seed (its distance
enters as r²), the initial model is then broad enough to span both blobs, and
the fit settles between them — at which point residuals are large at *both* and
no amount of robust reweighting can tell which one is the beam. Seeding narrow
and on the peak keeps the stray in the tail where the reweighting can discard
it. Measured: a stray 7 px away pulls the centre 2.41 px without this, 0.27 px
with it.

Performance
-----------

The inner loop evaluates a gaussian per pixel, which would be a transcendental
per pixel per pass. Since `exp(-(d+1)²/2s²) = exp(-d²/2s²)·exp(-(2d+1)/2s²)`
and the second factor itself advances by a constant ratio, a gaussian sampled
on a unit grid costs two multiplies per pixel instead. The recurrence walks
*outward* from the sample nearest the centre so every ratio stays ≤ 1 and
nothing can overflow, which a left-to-right sweep would do for a narrow sigma.
Trial steps write residuals to a spare buffer and swap on acceptance rather
than re-walking the window.

Together with a bounded iteration count these took the 32x32 case from 1463
us/frame — over five times the 264 us budget at fs 3788 — to 62 us. **Set
`max_iter` and a `window_*` deliberately**: they are the two knobs that bound
worst-case latency, and jitter in measurement latency feeds straight into
`fsp`'s fixed-delay model.

What this does **not** do
--------------------------

- **Fit a beam that isn't gaussian.** This is the real exposure, and it is
  different in kind from a correlation tracker's. `wfs_com` correlates against
  a *learned* template, so it does not care what shape the beam is, only that
  the shape is stable. This device assumes the shape. Where the beam is
  speckled, clipped, or strongly astigmatic, the model is wrong everywhere
  rather than at identifiable outlier pixels, and the result is a systematic
  bias in `y0`/`x0` that robust weighting cannot remove. Check the residual map
  on real frames before trusting it: if residuals are noise-like the model
  fits; if they show structure that moves, they do not.
- **Survive two comparable blobs in the window.** Robust weighting has a
  breakdown point. The defence is the window — keep it tight enough to exclude
  known stray reflections rather than relying on the weighting.
- **Guarantee a fixed iteration count.** LM is iterative. `max_iter` bounds it,
  but the cost genuinely varies frame to frame.
- **Resolve shear finer than the model.** The motion model is linear in row
  index — constant velocity within one readout. Genuinely curved intra-frame
  motion is not captured, and neither is a bottom-up or column-wise rolling
  shutter without changing the axis convention.

Parameters
----------

- `window_height`, `window_width` (default 0 = whole image): fit window,
  tracked on the previous position. Smaller is faster and excludes distant
  artifacts; must comfortably contain the beam (±4 sigma is a good target).
- `init_y`, `init_x` (optional): initial position and window placement. Unlike
  `wfs_com`, these only seed the search — the reported position is absolute, so
  no acquisition-time offset is absorbed.
- `sigma_init` (default 2.0): starting beam width, and the scale of the local
  box used to seed sigma at acquisition. Set it near the true value.
- `sigma_min`, `sigma_max` (defaults 0.5, 20.0): bounds. Hitting a bound
  rejects the frame, so they double as a sanity gate.
- `min_amplitude` (default 5.0): fitted amplitude below this means no beam.
- `max_residual` (default 0, disabled): rms residual in counts above which the
  frame is rejected. Set it from the residual you actually measure, plus
  headroom, or leave it off.
- `max_iter` (default 10): LM iteration cap. Bounds worst-case latency.
- `tol` (default 1e-4): relative cost improvement at which to stop.
- `robust_k` (default 2.5): Tukey cutoff in residual sigmas. 0 disables
  reweighting. Smaller rejects more aggressively.
- `robust_iter` (default 1): plain iterations before reweighting begins.
- `fit_slope` (default true): fit the intra-frame motion. False freezes both
  slopes at zero, reducing this to a static 5-parameter beam fit.
- `row_time` (default 0): seconds per image row of readout. **The position fit
  does not need it** — the slope is fitted in px/row and the reported position
  is the model evaluated at the ROI centre row, both independent of it. It
  converts the fitted slope to a physical velocity and fixes the epoch the
  reported position belongs to, which is what a delay-modelling controller
  downstream needs. A wrong value cannot corrupt the position.
- `reacquire_after` (default 10): consecutive failed fits before
  `AYLP_BEAM_LOST` and a re-seed.

The reference epoch is the ROI centre row, fixed in **image** coordinates, so
the instant the reported position belongs to does not wander as the window
tracks the beam. A moving epoch reads downstream as a wobbling loop delay.

Diagnostics kept in the device data, useful when logging: `vel_y`/`vel_x`
(px/s, zero unless `row_time` is set), `last_rms`, and `n_iter_last`.
