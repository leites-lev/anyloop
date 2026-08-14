# Steering runs — start here

Runs are grouped first by human session, then named by recording time, beam,
and controller/config. This avoids mixing after-midnight work with daytime work.

## Sessions

- [`last_night_2026-08-12_to_2026-08-13/`](last_night_2026-08-12_to_2026-08-13/README.md)
  — overnight 1000 Hz/20% integrated-beam and 1 Hz/20% sparse-beam tests.
- [`today_2026-08-13/`](today_2026-08-13/README.md)
  — afternoon data whose launcher did not preserve its beam or config.

## Naming convention

```text
SESSION/
  HH-MM-SS_beam-BEAM_config-CONFIG_LABEL/
```

New runs record the full ISO local timestamp, session, beam, profile, source
config, commit, completion time, exit status, and file list in
`run_manifest.txt`.

## Standard files inside a documented run

- `run_manifest.txt` — when/beam/config/status and generated-file inventory.
- `config_used.json` — immutable exact config used for that run.
- `results.json` — comparable analysis metrics, when available.
- `measured_error.aylp` and `actuator_command.aylp` — primary binary data.
- `observer_weights.dat`, `observer_trace.dat`, `transient_events.csv`.
- `console.log` and `changes.patch` — diagnostics and code/config provenance.

Folders containing `unknown` or `unmanifested` are retained for evidence but
must not be used to rank configurations.
