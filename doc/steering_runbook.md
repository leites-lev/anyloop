Steering runbook
================

Every profile in `contrib/steering/configurations/` carries the four steps at
the top of the file, in order. This is what they mean.

For a new beam the whole sequence is: `--on`, gain test, check the fit, run.
Nothing else needs editing — the camera measures its own ROI and gain,
`center_of_mass` its threshold, gates and window, and `fsp` its `fs`, transport
delay and delay bias, all at startup.

1. Isolate cores
----------------

None

2. Calibrate for a new beam
---------------------------

None

The 100 Hz / 80% profile uses its own manifest:

BEFORE A CLOSED RUN, MEASURE K AT THIS OPERATING POINT. K is in units normalized over the frame and the ROI is auto, so a K taken at a different frame size, focus or alignment is simply wrong. Gain only, leaving the delay on auto:
  sudo chrt -f 80 taskset -c 2 python3 contrib/calibration-scripts/tools/run_calibration_suite.py contrib/steering/configurations/support/calibrate_100hz_80pct.json --skip-delay
That runs support/calibrate_100hz_80pct_gain_x.json then _gain_y.json (mirror parked, one axis driven at a time) and writes Kx/Ky into this file's x.K / y.K. CHECK THE HEADERS BEFORE BELIEVING IT: '# fit slope ... R2' with R2 >= 0.97 and 30/30 levels; the 2026-08-13 pair on the 1000 Hz profile came in at R2 0.9962 (x) and 0.9732 (y), so anything much below that is a bad run, not a bad plant. Drop --skip-delay to re-measure the delay by PRBS as well -- that writes a MEASURED delay over the "auto" setting on the fsp stage, which is an upgrade, not a regression.

Step 2 updates **every** steering profile, not just the one named in the
command: `K` and `delay_ident_ms` describe the physical plant, and all four
profiles run on the same bench. Running a gain config on its own, outside the
suite, writes only `run_1000hz_20pct.json`.

3 and 4. Run
------------

`run_steering.py` parks the FSM and runs the ROI recenter pass before starting
(a no-op while the ROI is self-sizing), then launches the loop pinned and
realtime. `run_steering_recorded.sh` does the same and preserves the run's
outputs, config and console log under `data/steering_runs/`.

What is *not* automatic
-----------------------

- **The sign of K.** `K` is stored positive and the sign lives in
  `parport_dac`'s `scale`; the gain test's `.dat` reports the *signed* slope.
  After any optics change, compare it against the last good run — a flipped
  sign is positive feedback into a fixed delay.
- **`delay_ident_ms`** is on `"auto"`: the sensor path is *modelled*
  (`exposure/2 + 2.5 frames`, the constant taken from this bench's own Bode
  fits) rather than identified. It matches those fits to within 4%, but it
  cannot see actuator lag. Step 2 without `--skip-delay` measures the delay by
  PRBS and writes a real number into every profile; do that before a run whose
  attenuation number you intend to quote.
