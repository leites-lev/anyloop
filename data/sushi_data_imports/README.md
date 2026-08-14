# Imported transfer from 2026-08-12

This folder contains one transfer in two forms:

- `2026-08-12_fsl_transfer_original.zip` — original untouched 5.3 GB archive.
- `2026-08-12_fsl_transfer_extracted/` — readable extracted copy containing:
  - `calibration/` — gain, delay, ladder, and PRBS measurements/plots.
  - `configs/` — configs shipped with the transfer.
  - `current_validation/` — CoM survey and steering validation capture.
  - `offline_run/` — two large ten-minute open-loop CoM recordings.
  - `code/` — source bundle and originating commit identifier.

Keeping both forms costs approximately 9.9 GB. Once the extracted tree has
been backed up or verified to your satisfaction, the original ZIP is optional.
Conversely, the extracted tree can be recreated from the ZIP. Do not delete
both unless the transfer is no longer needed.
