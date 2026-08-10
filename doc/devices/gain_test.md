anyloop:gain_test
=================

Types and units: `[T_VECTOR, U_ANY] -> [T_VECTOR, U_MINMAX]`.

Measures the plant's DC gain with a staircase. The device sits where a
controller (`anyloop:pid`, `anyloop:fsp`) normally sits — sensor upstream, DAC
stage downstream — and walks one output element from `low` to `high` in `step`
increments, holding each level for `dwell` seconds. The trailing `settle_frac`
of every level is averaged into one settled response, and a least-squares line
through the (command, settled response) pairs is the gain.

That gain is the same quantity `bode_plot` reports as `fit_K` and
`latency_test` as `gain`, measured from the same point in the loop — but at DC,
in about half a minute, and with the whole command range visible at once
instead of one small-signal amplitude. Use it to re-check a `K` between sweeps,
or to find where the actuator stops being linear.

What the staircase buys over a two-level step:

- **compression is visible**, as curvature in the per-level residuals. The FSM
  on this rig loses a few percent of gain above ~50 mV, and the `_K_warning` in
  `contrib/attenuation_par_fsp.json` is a long note about how that bit an
  earlier run. The largest residual is reported as a fraction of the response
  span, and the residual panel of the PDF shows its shape.
- **`fit_range`** re-fits the small-signal region the closed loop actually
  lives in. If that slope and the full-range slope differ, the full-range one
  is not the number to put in a controller.
- **`updown`** walks back down again, which measures hysteresis — and is the
  only way to separate gain from beam drift (see below).

Stages: park at `bias` for `warmup` s so the sensor can acquire; glide to `low`
over `ramp` s; run the staircase; glide back to `bias`; then fit, log, write
the files and raise `AYLP_DONE`. Both glides are deliberate — a tracked
`center_of_mass` window only follows motion that stays inside it, so jumping
straight to `low` can lose the beam before the test starts, and jumping home at
the end throws the actuator for no reason.

Sizing the sweep to the sensor
------------------------------

**The useful range is set by the camera, not by the DAC.** With the gain this
rig has (Kx 4.05, Ky 3.646 CoM units/V) and a 64×64 ROI (31.5 px per unit), one
volt moves the beam about 128 px — twice the ROI. The beam leaves a 64×64 ROI
at roughly ±0.25 V and leaves a 20×20 `center_of_mass` tracking window at about
±0.08 V *per step*, so both the span and the increment matter:

    span:  |V| < (ROI/2 - beam radius) / (K * pixel_scale)
    step:  step * K * pixel_scale < (window/2 - beam radius)

The shipped configs use ±0.12 V in 12 mV steps: ±15 px of travel with ~17 px of
margin, and 1.5 px per step against a settled centroid noise hundreds of times
smaller. Re-derive both if the ROI, the optics or `K` change.

Levels that break those bounds anyway are recorded but excluded from the fit,
rather than quietly flattening the slope:

- a settled `|response|` past `resp_max` — the beam is running out of the
  sensor;
- a settled window whose samples are all bit-identical — a tracked
  `center_of_mass` that has lost the beam holds its last coordinate, which
  otherwise reads as a beautifully quiet level.

Two unusable levels in a row end the sweep early *once it has been in range* —
a sweep that starts wider than the sensor walks into range and its good middle
is kept, while one that leaves range at the top stops instead of driving the
beam further off.

Drift, and why it needs `updown`
--------------------------------

The beam on this rig walks several px per hour and the small-signal gain itself
wanders a few percent over one, so a slope measured over 30 s carries whatever
drift happened during those 30 s. The device fits a linear-in-time term
alongside the gain to separate them — but **that fit is only possible on an
up-and-down sweep**. In a plain rising staircase the command and the clock are
the same axis: gain and drift are perfectly collinear and no fit can tell one
from the other. The device checks for that and says so rather than inventing a
split. With `updown`, the down branch repeats every command at a later time,
which breaks the tie; the gap between the plain and the drift-carrying slope is
then the amount of drift that had been landing in the gain.

Output
------

Everything is logged. `output_file` also writes a PDF (response vs command with
the fit, plus a residual panel in px) and, alongside it, a `.dat` carrying the
fit results in its header and one row per level: command in units and in volts,
settled mean, standard deviation, standard error, sample count, timestamp,
residual, validity and branch. Set `output_file` to `""` to log the numbers and
write nothing.

Parameters
----------

- `index_cmd` (int) (optional)
  - Output element to drive. Default 1 (x, for a `[y, x]` command vector).
- `index_err` (int) (optional)
  - Input element to watch. Default 1 (x, for a `center_of_mass` `[y, x]`).
- `out_size` (int) (optional)
  - Output vector length; every other element is held at 0. Default 2.
- `low`, `high`, `step` (float) (optional)
  - Staircase endpoints and increment, in command units. Defaults -0.1, 0.1,
    0.01. `(high - low)` need not be a whole number of steps; the last level is
    clamped to `high`. Note these are *command* units — with a negative DAC
    `scale` the sweep descends in volts.
- `bias` (float) (optional)
  - Command held before and after the sweep, and the centre `fit_range` is
    measured about. Default 0.
- `dwell` (float) (optional)
  - Seconds at each level. Default 1.0.
- `settle_frac` (float) (optional)
  - Trailing fraction of each dwell that is averaged. Default 0.5. The leading
    part is thrown away, so it has to cover the plant's settling.
- `warmup` (float) (optional)
  - Seconds parked at `bias` before the sweep. Default 5. Must exceed the
    sensor's own acquisition time.
- `ramp` (float) (optional)
  - Seconds to glide into and out of the staircase. Default 2.
- `updown` (bool) (optional)
  - Sweep back down after reaching `high`, measuring each level twice and
    reporting the mean up/down offset as hysteresis. Doubles the run. Default
    false. Required for the drift fit.
- `resp_max` (float) (optional)
  - A settled `|response|` past this marks the level unusable. Default 0.9,
    i.e. the very edge of a `U_MINMAX` sensor.
- `fit_range` (float) (optional)
  - Also fit the levels within this distance of `bias`, for a small-signal
    gain. Default 0 (off).
- `volts_per_unit` (float) (optional)
  - The DAC `scale` of the driven channel, used only for reporting: it converts
    the fit into units/V, px/V and mV/px, and carries the sign of the optical
    path. Default 1.
- `pixel_scale` (float) (optional)
  - Pixels per response unit, `(dim - 1)/2` for a `center_of_mass` output —
    31.5 at a 64×64 ROI. Default 1. Same convention as `attenuation_test`.
- `output_file` (string) (optional)
  - PDF path; the `.dat` is written alongside it. Default `"gain_test.pdf"`.
    `""` writes nothing.
- `config` (string) (optional)
  - Free text copied into the `.dat` header — the operating point, biases,
    camera settings, why the run was made.

Example
-------

See `contrib/conf_gain_par_x.json` and `contrib/conf_gain_par_y.json` for
complete pipelines (`asi_source → center_of_mass → gain_test → parport_dac`).

```json
{
 "uri": "anyloop:gain_test",
 "params": {
  "index_cmd": 1, "index_err": 1, "out_size": 2,
  "low": -0.12, "high": 0.12, "step": 0.012,
  "dwell": 1.0, "settle_frac": 0.5,
  "fit_range": 0.05,
  "volts_per_unit": 1.0, "pixel_scale": 31.5,
  "output_file": "fsm_gain_x.pdf"
 }
}
```

Tests
-----

`tests/test_gain_test.c` drives the device against a simulated plant of known
gain and checks that the reported slope is that gain — including under
compression, sensor saturation and beam drift. `ninja -C build test` runs it
with no hardware.
