# steering_tuned runs, 2026-08-03

Hand-copied out of `dac_cmd_check.aylp` while the loop was running, BEFORE
file_sink learned not to truncate. Kept because the runs they came from were
destroyed by subsequent restarts.

- `dac_cmd_2026-08-03_run1.aylp` — 58075 frames (~144 s @ 402 Hz). The ONLY
  surviving copy of that run; the live file was truncated by a restart minutes
  after this was taken.
- `dac_cmd_2026-08-03_run3_partial_prefix.aylp` — a PREFIX of a later run
  (11.85 MB copied while it was still being written), not the whole thing. The
  complete version of that run survives as a rotated `dac_cmd_check.NNNN.aylp`
  in the repo root.

Contents: `anyloop:file_sink` output taken after `pid` and `clamp`, so each
record is a 40 byte AYLP header plus two doubles — the [y, x] DAC command in
minmax units, NOT the centroid. 56 bytes/frame. There is no camera data in these:
nothing was recording frames at the time.

Measured loop rate over these runs: ~400 Hz (401.4 Hz averaged over a 764 s run).
