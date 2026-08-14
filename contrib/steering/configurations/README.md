# Steering configurations

Paths below are relative to this directory; the commands are meant to be run
from the repository root.

## Active configuration

`steering_par_fsp.json` is the primary `--live`
configuration. For now it is kept identical to
`run_1000hz_20pct.json`.

## Recorded-run profiles

- `run_1000hz_20pct.json` — 1000 Hz beam, 20% optical duty
- `run_100hz_80pct.json` — 100 Hz beam, 80% optical duty
- `run_1hz_20pct.json` — 1 Hz beam, 20% optical duty

Each carries its runbook as the first four top-level keys, in the order they
are run: isolate cores, calibrate for a new beam, run, run-and-archive. What
each step does — and what to check before closing the loop — is in
`doc/steering_runbook.md`; superseded bench numbers are in
`doc/steering_bench_history.md`.

## `support/`

Everything a profile depends on but is not launched directly:

- `calibrate_1000hz_20pct.json`, `calibrate_100hz_80pct.json` — calibration
  manifests, run through
  `contrib/calibration-scripts/tools/run_calibration_suite.py`
- `calibrate_*_gain_{x,y}.json` — per-axis DC gain runs. They carry
  `write_config`, so a passing fit installs `K` into the run profile itself
- `calibrate_*_{x,y}.json` — per-axis PRBS delay runs
- `com_survey_*.json` — beam/centroid surveys
- `movement_*.json` — actuator movement checks
- `attenuation_par_fsp.json` — closed-loop attenuation measurement

## Launching

```
sudo python3 contrib/steering/tools/run_steering.py 1000hz_20pct
contrib/steering/tools/run_steering_recorded.sh --1000 --label <label>
```

The first parks the FSM and recenters the ROI before starting; the second also
preserves the run's outputs under `data/steering_runs/`.

## Other configurations

The constant-beam, pulsed-beam, WFS-COM, and push-event configs are preserved
under `../../legacy/legacy_steering_profiles/`. New runs should use the named
profiles above.
