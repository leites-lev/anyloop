# Experimental data index

All generated measurements belong under `data/`; they should not be stored at
the repository root.

## Current organization

- `steering_runs/SESSION/HH-MM-SS_beam-BEAM_config-CONFIG_LABEL/` — recorded
  steering runs. Session folders clearly separate last night from today; each
  run name shows local time, optical beam, selected config, and run label.
  Each complete run contains `config_used.json`, `run_manifest.txt`,
  `console.log`, `changes.patch`, `measured_error.aylp`, and, when produced,
  `actuator_command.aylp`, observer data, transient events, and `results.json`.
- `steering_runs/*/unmanifested_attempts/` — older pulsed steering attempts and
  unindexed outputs that predate per-run manifests. Their names include the
  known recording date/profile and explicitly say `unmanifested` when an exact
  start time cannot be recovered.
- `calibration/` — timestamped calibration sessions organized by purpose,
  plus a dedicated staging directory for active calibration. See its README.
- `legacy/root_rollovers/` — older numbered steering rollovers whose exact
  settings were not archived with the data.
- `run_staging/` — temporary output while a run is active. It should be empty
  between runs; the wrapper moves completed output into `steering_runs/`.
- `imports/` — the human-readable 2026-08-12 FSL transfer, with clearly named
  original archive and extracted copy. See its README before deleting either.
- `legacy/` — optional unindexed historical captures, not used by current
  results or configs. Its README explains exactly what can be deleted.

## Main configs

The active configs remain under `contrib/` because scripts refer to them there:

- `contrib/steering_par_fsp.json` — primary live config; currently an exact
  copy of the canonical 1000 Hz, 20% profile.
- `contrib/pulsed_steering/run_1hz_20pct.json` — 1 Hz, 20% duty run config.
- `contrib/pulsed_steering/run_100hz_80pct.json` — 100 Hz, 80% duty run config.
- `contrib/pulsed_steering/run_1000hz_20pct.json` — 1000 Hz, 20% duty run config.
- `contrib/attenuation_par_fsp.json` — long attenuation/reference test.
- `contrib/conf_bode_par_{x,y}.json` and `_small` variants — plant calibration.

Historical explanatory comments inside these JSON files are not measurements
of the current setup. The archived `config_used.json` inside a run directory is the
authority for what that run actually used.

Run and archive a selected steering profile with:

```text
contrib/run_steering_recorded.sh --live
contrib/run_steering_recorded.sh --1000
contrib/run_steering_recorded.sh --100
contrib/run_steering_recorded.sh --1
```

The equivalent general form is `--config live|1000|100|1`. Add `--label NAME`
for an explanatory suffix. Pulsed profiles supply the beam automatically; use
`--beam continuous` (or another accurate label) with `--live`. Use
`--session last_night_YYYY-MM-DD_to_YYYY-MM-DD` when continuing after midnight.

## Best comparable results from 2026-08-13

All pixel values below use the correct 384-pixel CoM scale of 191.5 px per
normalized unit. Open is 2–22 s; early closed is 40–340 s.

| Run | Controller | Open vector RMS | Closed vector RMS | Rejection |
| --- | --- | ---: | ---: | ---: |
| `last_night.../01-32-02_beam-1000hz-20pct_controller-modal_baseline` | modal only | 10.23 px | 6.66 px | 34.9% |
| `last_night.../02-05-00_beam-1000hz-20pct_controller-nlms512` | modal + 512-tap predictor | 11.93 px | 7.48 px | 37.3% |
| same run, 300–600 s | modal + 512-tap predictor | 11.93 px reference | 6.66 px | 44.1% |

The last window reaches 6.06 px when only fresh illuminated CoM samples are
used. It is the best overnight value, but not a strict simultaneous A/B result.
The 1000 Hz / 20% optical-duty A/B result is 10.64 px open to 7.86 px closed
(26.1% rejection); see its `steering_ab_1000hz_20pct.json`.

Do not compare pulsed and non-pulsed results without also reporting held-frame
fraction. Threshold CoM repeats the last centroid while the beam is dark.
