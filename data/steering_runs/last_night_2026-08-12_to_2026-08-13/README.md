# Last night: 2026-08-12 evening to 2026-08-13 morning

## Best results

| Beam | Controller | Run | Open → closed vector RMS | Rejection |
| --- | --- | --- | ---: | ---: |
| 1000 Hz, 20%, integrated by 1 ms exposure | Modal only | `01-32-02_beam-1000hz-20pct_controller-modal_baseline` | 10.23 → 6.66 px | 34.9% |
| 1000 Hz, 20%, integrated by 1 ms exposure | 512-tap NLMS | `02-05-00_beam-1000hz-20pct_controller-nlms512` | 11.93 → 7.48 px early | 37.3% |
| same | same, later 300–600 s | same run | 11.93 reference → 6.66 px | 44.1% |

The 512-tap run reaches 6.06 px using fresh illuminated CoM samples only. The
later window is the best overnight result, but it is not a simultaneous A/B.

## Run-by-run map

| Time | Beam | Camera | Controller/config | Status |
| --- | --- | --- | --- | --- |
| 01:30:27 | 1000 Hz, 20% integrated | 384×384, 1000 us, 471 Hz | modal, broadband off | 88 s checkout |
| 01:32:02 | 1000 Hz, 20% integrated | same | modal, broadband off | valid baseline |
| 02:05:00 | 1000 Hz, 20% integrated | same | 512-tap NLMS, LP=5 | valid principal run |
| 02:48:44 | 1 Hz, 20% sparse | 384×384, 2000 us, 471 Hz | 512-tap NLMS | only 50 s |
| 02:50:15 | 1 Hz, 20% sparse | same | 512-tap NLMS | crashed, exit 139 |
| 02:50:38 | 1 Hz, 20% sparse | same | start delay 2500 s | open-loop diagnostic |
| 02:56:48 | 1 Hz, 20% sparse | same | start delay 2500 s | only 10 s |

`unmanifested_attempts/` contains additional pulsed files that predate reliable
one-run/one-manifest recording. They are grouped by known waveform but not
ranked as individual runs.
