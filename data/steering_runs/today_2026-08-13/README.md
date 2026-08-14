# Today: 2026-08-13 daytime

Two root-level recordings were recovered after the overnight cleanup:

| Time | Beam | Config | Available data | Status |
| --- | --- | --- | --- | --- |
| 14:33:23 | Unknown | Unknown | error, command, weights, transient log | Unrecorded; not rankable |
| 14:56:45 | Unknown | Unknown | error, command, observer trace | Unrecorded; not rankable |
| 15:03:51 | Unknown | Unknown | observer weights only | Recovered orphan; not loadable as validated weights |

These folders deliberately say `beam-unknown_config-unknown`. Assigning them a
waveform or controller from timestamps alone would be misleading. Future runs
through `contrib/run_steering_recorded.sh` record both fields automatically.
