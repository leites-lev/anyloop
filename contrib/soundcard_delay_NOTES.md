# Soundcard L→y delay measurement — investigation notes (2026-07-22)

Goal: measure the transport delay of the **soundcard Left channel → FSM y-axis →
camera → CoM** path (the AC actuator in the dual-actuator plan).

**Status: NOT SOLVED.** The delay could not be measured because the in-loop
`aylp_alsa` output under-drives the FSM compared to `aplay`, especially above
~20–30 Hz, so there is no broadband coherence to fit a delay to. Root-cause the
under-drive first; then the multitone phase-slope method below should work.

---

## What IS established

1. **LEFT channel (command vector[0]) drives y. RIGHT is not connected** (confirmed
   by the user). Verified: `aplay` playing L=20 Hz / R=30 Hz while `steering_fsp`
   held the beam open-loop; FFT of the y CoM in `spikehunt_err.aylp` showed 20 Hz
   in **y at 8×** the x response (24× the noise floor); 30 Hz (R) at the floor.
   → For any y drive use `aylp_alsa` `scale:[1,0]` (element 0 = L).

2. **The L→y coupling is broadband and strong** — measured with `aplay`
   (true 48 kHz sine), multitone L=20/40/80/150 Hz @0.2 each, beam held by
   `steering_fsp`: y-response **SNR 41 / 41 / 19 / 6** (a clean low-pass, all in y).
   So it is NOT a narrowband actuator.

---

## Methods tried and why each failed

### A. `latency_test` (settled-step) — CANNOT work on an AC-coupled actuator
- It calibrates on the *settled* step, but the soundcard is AC-coupled so a DC
  step decays back to baseline before the settle window → step/σ ≈ 1 → the 8σ
  gate aborts before timing any departure.
- Evidence: `Bode_and_testruns/latency_soundcard.dat` (period 0.15 s: step/σ 0.1);
  fresh L→y run at period 0.02 s: step −0.018, σ 0.018, step/σ 1.0 → abort.
- The departure (first-motion) *time* would be valid; only the gate blocks it.
  Would need a peak-referenced "AC mode" added to `latency_test`.

### B. White-noise cross-correlation (added `test_source` `kind:"noise"`) — SNR too low
- Idea: white command → command↔error cross-correlation = impulse response;
  onset = transport delay, peak = group delay. In-loop so command & error share
  the frame clock (`delay_err.aylp` before inject, `delay_cmd.aylp` after).
- Result: **coherence ~5%.** White noise spreads amplitude 0.3 over ~1900 Hz, so
  per-frequency energy is buried under the beam's low-frequency drift (rms 0.035).
  GCC-PHAT found no peak; the transfer-function phase slope gave an unphysical
  *negative* delay. Plain xcorr showed a broad bipolar bump (peak ~16 fr) that is
  the bandpass-smeared response, not a clean delay.
- Config: `conf_delay_noise_soundcard.json`. Scripts (scratchpad):
  `xcorr_delay.py`, `xcorr_delay2.py`, `gcc_delay.py`.

### C. Bode swept-sine phase slope (`conf_bode_soundcard_y.json`) — low coherence
- `bode_plot` element 0, sweep 12–300 Hz, aylp_alsa L. Result: fit K≈0.009,
  τ negative, phase residual 37°, magnitude 750% non-flat, most points flagged
  LOW QUALITY. The time-shared sweep gives only ~30 cycles/point → under-integrates
  → drift dominates. (The method is sound; the SNR per point is not there.)

### D. In-loop multitone phase slope (added `test_source` multitone) — blocked by under-drive
- Idea (the right one for a modest-authority AC actuator): play several
  well-separated tones **simultaneously** in-loop so each integrates over the whole
  record; DFT command & error at each tone; slope of unwrapped phase(H) vs freq
  = −2π·τ. Analysis is in cycles/sample so it's loop-rate-independent.
- Config: `conf_delay_multitone_soundcard.json`. Script: `multitone_delay.py`.
- **Blocked:** in-loop response SNR is far below `aplay` (see next section), and
  collapses above ~30 Hz, so there is no usable phase-vs-frequency slope
  (residuals 36–72°, τ jumps 12–43 frames run-to-run = noise).

---

## THE BLOCKER: `aylp_alsa` in-loop under-drive (needs root-cause)

Same card (hw:0,0), same mixer, near-identical digital amplitude (0.17–0.2 per
tone), yet the beam responds far less to the in-loop path than to `aplay`:

| tone  | aplay SNR | in-loop SNR (`aylp_alsa`) |
|-------|-----------|---------------------------|
| 20 Hz | 41        | 15  (~2× weak)            |
| 40 Hz | 41        | 1   (dead)                |
| 80 Hz | 19        | 1   (dead)                |
| 150 Hz| 6         | 0.7 (dead)                |

So the in-loop path has a steep low-pass cliff around ~20–30 Hz that `aplay` does
not. This must be fixed for any in-loop delay measurement — and it also matters for
the actual dual-actuator loop (the soundcard would have little authority in-loop).

Facts / partial findings:
- `latency_frames:4` additionally under-drove ~**24×**: a 4-sample queue underruns
  the card's **32-frame minimum period**; `set_swparams` sets stop_threshold =
  boundary, i.e. **xrun detection is disabled**, so the underruns are silent
  (xruns=0) while the analog output collapses.
- `latency_frames:32` (0.667 ms queue, one full period, 0 xruns) fixed the
  underrun but **NOT** the ≥40 Hz cliff → the cliff is a separate issue.
- Both threaded and legacy `aylp_alsa` modes write **one held command value per
  proc** (ZOH at the loop rate). ZOH rolloff at ≤150 Hz vs a 3788 Hz loop is
  negligible (sinc≈1), so ZOH alone does not explain the cliff.

Hypotheses to test (for the next attempt):
1. Threaded feeder timing: the feeder refills the 32-frame queue ~once per period
   (~1.5 kHz), sampling `shared_cmd` at moments that may alias/average the
   command's HF variation. Try **non-threaded low-latency** (`process_lowlat`) and
   **legacy** fill modes and re-measure the 40 Hz SNR.
2. Look for a place where `shared_cmd`/the mmap fill smooths or drops updates
   (mutex, poll cadence, `avail`-limited partial fills).
3. Ground-truth with the card's **loopback capture** (card0 has a capture device):
   drive a 40 Hz in-loop tone, capture the actual analog-out, and compare its
   amplitude/spectrum to the `aplay` 40 Hz tone — isolates plugin vs physics.
4. Scope the line-out while driving in-loop 40 Hz vs aplay 40 Hz.

Once the in-loop output matches `aplay` broadband, run
`conf_delay_multitone_soundcard.json` (tones e.g. 20/40/80/150 or a wider set) and
`multitone_delay.py`; the phase slope gives the delay. Remember to **subtract the
soundcard queue latency** (plugin logs `queue N fr = X ms`; 0.667 ms ≈ 2.5 loop
frames at 3788 Hz) to isolate the FSM+camera+CoM path delay.

---

## Files created this session
- Code (built): `devices/test_source.c/.h` — added `kind:"noise"` (+`seed`) and a
  multitone for `kind:"sine"` (`frequencies` [Hz] array + `fs`).
- Configs (`contrib/`): `conf_determine_ychan.json`, `conf_bode_soundcard_y.json`,
  `conf_delay_noise_soundcard.json`, `conf_delay_multitone_soundcard.json`.
- Data (repo root): `delay_err.aylp`, `delay_cmd.aylp` (last multitone run),
  `spikehunt_err.aylp` (aplay determination/response), `fsm_bode_soundcard_y.{dat,pdf}`.
- Analysis scripts live in the session scratchpad (ephemeral): `multitone_delay.py`,
  `xcorr_delay.py`, `xcorr_delay2.py`, `gcc_delay.py`, `analyze_chan.py`.

## Reproduction notes
- Run soundcard configs under `sudo -n` (rt_prio/mlockall); `sudo -n` is passwordless
  here. `spikehunt_cmd.aylp`/`delay_*.aylp` may be left root-owned — `sudo -n chown`
  or `rm` them before re-runs (a plain run gets "Permission denied" on the file_sink).
- Mixer must be: Auto-Mute **Disabled**, Line Out + Headphone **on**, Master 0 dB
  (already set). `contrib/play_tone.sh <hz> <amp>` drives **both** channels; for
  single-channel tests generate a stereo WAV with the other channel zeroed.
- Beam ROI 32×32 at start_x 272 / start_y 392 (beam ~x290,y408 on sensor); loop
  ~3788 Hz at exp 200.
