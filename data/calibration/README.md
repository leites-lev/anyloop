# Calibration data — start here

## Layout

- `runs/` — future calibration runs. Each run gets a timestamped folder with
  its manifest, exact result, logs, full-frame probe, and raw camera capture.
- `sessions/2026-08-12_night_to_2026-08-13_morning/` — existing camera/CoM
  commissioning data grouped by purpose and renamed with recording timestamps.
- `historical_soundcard/` — small, older soundcard channel/latency measurements;
  unrelated to the current camera/FSM optical calibration.
- `run_staging/` — temporary files while calibration is active. It should be
  empty between calibration runs.

## Current historical session

The session contains approximately 2.8 GB:

- `com_tracker_survey/` — 2.6 GB of raw camera captures plus compact analysis.
- `full_frame_beam_probes/` — 203 MB of beam-location/recentering captures.
- `fsm_movement_check/` — two tiny CoM captures at 0 V and x = 0.5 V.

The compact survey result says `recommendation: none`, `beam_clipped: true`,
and a 42.95% fit rejection rate. It is failed/diagnostic evidence, not a valid
current calibration. Raw `.aylp` captures may be deleted if reanalysis of that
failed session is not needed; current steering operation does not read them.

Run a new organized CoM calibration with:

```text
contrib/com_survey_run.sh
```

It creates `runs/YYYY-MM-DDTHH-MM-SS_com_tracker_survey/` automatically.
