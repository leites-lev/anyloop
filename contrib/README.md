# Contributed tools and configurations

This directory contains reusable configurations and utilities. Recorded output
does not belong here; run data is stored under `data/`.

## Layout

- `steering/` — current steering configs, profiles, and launchers
- `calibration-scripts/` — calibration configs and their runners
- `anyloop-examples/` — small pipeline, API, and plugin examples
- `data-analysis-tools/` — offline analysis, fitting, and analysis regressions
- `monitoring-scripts/` — Julia live/file monitors
- `camera_pcie_hardware/` — optional hardware backends and bench tools
- `diagnostics/` — diagnostic, replay, pipeline-check, and benchmark programs
- `legacy/legacy_steering_profiles/` — superseded steering configurations
- `legacy/outdated_scripts/` — retired experiments and hardware scripts; do
  not use these for new runs

## Common entry points

Run a recorded steering profile from the repository root:

```sh
contrib/steering/tools/run_steering_recorded.sh --1000
contrib/steering/tools/run_steering_recorded.sh --100
contrib/steering/tools/run_steering_recorded.sh --1
```

The primary steering config is
`steering/configurations/steering_par_fsp.json`. It currently
matches the 1000 Hz, 20% duty profile. See that directory's README for details.

Run the standard calibration manifest with:

```sh
sudo chrt -f 80 taskset -c 2 python3 \
  contrib/calibration-scripts/tools/run_calibration_suite.py \
  contrib/calibration-scripts/configurations/conf_gain_delay_par_xy.json
```
