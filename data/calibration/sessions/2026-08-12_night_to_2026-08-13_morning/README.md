# Camera and CoM calibration session: 2026-08-12 night

This work was recorded from 2026-08-12 21:42 through 2026-08-13 00:51.

| Folder | Purpose | Verdict |
| --- | --- | --- |
| `com_tracker_survey/` | Compare/derive CoM tracker parameters from camera frames | Failed diagnostic: clipped beam, 42.95% rejected, no recommendation |
| `full_frame_beam_probes/` | Locate the beam on the full ASI290MM sensor and choose ROI | Historical recentering inputs |
| `fsm_movement_check/` | Compare CoM at 0 V and an x-axis 0.5 V command | Small diagnostic pair |

Filenames contain the local recording date and time. The survey's authoritative
compact output is `com_tracker_survey/2026-08-13T00-02-51_analysis_result.json`.
The `raw_captures/` directory contains the large files that dominate disk use.
