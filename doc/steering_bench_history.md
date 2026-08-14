Steering bench history
======================

Notes lifted out of the steering profiles on 2026-08-14. Every one describes a
bench state that has been superseded — a different camera, ROI, exposure or
frame rate — or a parameter the loop now measures for itself (`asi_source`'s
ROI and gain, `fsp`'s `fs`, `delay` and `delay_auto_bias`). None of it is a
live setting.

It is kept because the reasoning and the numbers are evidence: when a fresh
measurement disagrees with one of these, the disagreement is the finding.

The live configuration and its runbook are in
`contrib/steering/configurations/` — each profile carries `_run`, `_archive`,
`_perf_mode` and `_calibrate_gain` at the top.


## camera (asi_source)

### `_roi_2026_08_07_reprobe2`

REPROBED AGAIN 2026-08-07 19:42 -- the bench is still being worked on and the beam has moved and changed since the 19:20 probe: full-sensor position (446,1115) -> (436,1107), and sigma 5.21 -> 9.33 px (10.12 y / 8.54 x), i.e. back out of focus. start 1076/404 lands it at in-frame (32.2, 31.3) against the 31.5 centre. Gates re-derived to match: sigma_init 9.33, sigma_min/max at 0.3x/4x. SOURCE STATE AT THIS PROBE: 24.8% of frames are dark and saturation reaches 24% of pixels at gain 0 (gain cannot go lower; exposure is not available as a lever because 400 us sets fs 2310 and the delay in FRAMES). The dark frames are a REGULAR ~46 Hz CHOP: 119 streaks in 2.6 s, every one of them 13 frames (6 ms) long. That is well under reacquire_after 300, so they route to fit_com's hold path rather than tripping BEAM_LOST -- but a quarter of all frames carry a held error sample, which the loop will feel.

### `_roi_2026_08_07`

REPROBED 2026-08-07 19:20 at the OPERATING ROI (6000-frame 64x64 capture, gain ladder 0/10/25/50). THE BEAM CHANGED DURING THE SESSION -- someone refocused and realigned it: sigma 8.2 -> 5.21 px (37% tighter), peak brightness roughly doubled, and the centroid moved ~10 px in x. That is also the resolution of the 'K drifted 39%/hour' puzzle: the optical path was NOT fixed while K was being measured, and refocusing changes the lever arm at the sensor, so the decline was real rather than an artefact. start 1092/416 puts the beam at in-frame (31.0, 33.3) against the 31.5 centre, on the ASI grid (x multiple of 4, y multiple of 2). Derived from a capture at start 1084/414 that found the beam at (33.04, 41.29). MEASURE THE ROI FROM A CAPTURE AT THE OPERATING ROI, never from a full-sensor probe: on 2026-08-07 the two disagreed by 10 px in both axes, so find_roi.py's full-sensor arithmetic needs confirming against a cropped frame before it is trusted.

### `_camera_note`

ASI290MM (IMX290, 1936x1096) as of 2026-08-06, back from the ASI662MM (IMX662, 1920x1080) this file ran on 2026-08-02..08-05. EVERY MEASURED NUMBER IN THIS FILE NOW PREDATES TWO CAMERA SWAPS AND AN ROI CHANGE and must be re-taken: fs (both the 3788 Hz of the 290 at 32x32/exp50/bw100 and anything measured on the 662 are wrong here -- 64x64 at exposure 400 is a different frame rate again, and fs sets the loop period, K's units, the delay in FRAMES and broad_lp all at once), delay, K, and the pixel scale. Re-run the bodes before trusting a run.

### `_roi_note`

ROI 64x64 at start_x 644 / start_y 318, centring full-sensor (676, 350) -- the beam position every ASI662MM config agreed on, carried over unverified. THE 662 AND THE 290 ARE DIFFERENT SENSORS ON A DIFFERENT MOUNT: re-probe with contrib/calibration-scripts/configurations/probe_frame.json and re-derive with contrib/calibration-scripts/tools/find_roi.py before the first real run. The beam also walks several px per hour, so re-derive again once |park| exceeds ~2 px.

### `_roi_80x80_2026_08_11`

ROI enlarged 64x64 -> 80x80 for the measured sigma 8.62 px beam: +/-4 sigma needs about 69 px and failed the 64x64 clearance gate. This ROI changes camera readout rate. The old fs=2310 and delay in frames below are provisional until measured at this exact 80x80 mode; do not close steering from the old values.

### `_gain_2026_08_07`

gain must now be 0: at the current source brightness every higher setting saturates the core. Measured amp p5/p50/p95 and saturation p99: gain 0 -> 18/137/255, 1.66%; gain 10 -> 4/198/255, 2.12%; gain 25 -> 4/242/255, 2.32%; gain 50 -> 14/254/255, 3.61%. At gain 50 the beam is saturated in over half of all frames, against p50 109 at the same gain at 18:07. Exposure is NOT available as the lever -- 400 us sets fs 2310 and therefore the delay in FRAMES.

### `_bandwidth_2026_08_07`

bandwidth 50 -> 100. THIS FILE WAS RUNNING AT ROUGHLY HALF THE FRAME RATE IT CLAIMED. USB bandwidth costs fps close to linearly (measured 2026-07-17: bw 80 gave 3031 Hz against bw 100's 3788 at 32x32), so at 50 this config could not have been at the fs 2310 its fsp stage is configured for -- and fs sets the loop period, the delay in FRAMES and broad_lp all at once, so every one of them was wrong here. attenuation_par_fsp.json was already at 100, as were both bode sweeps, which is the condition the delay and K above were measured under. Re-confirm the achieved rate from the run log before trusting a result.


## center_of_mass

### `_provenance`

MEASURED 2026-08-06 by contrib/calibration-scripts/tools/com_survey_run.sh over a 60 s / 138600-frame capture at 64x64, exp 400, gain 0, fs 2310 Hz, with the ROI auto-centred first (find_roi put it at 1088/406, beam landing at (28.39, 29.95) against a 31.5 centre -- the residual is the 4-px/2-px ASI ROI grid, not an error): beam sigma 6.55 px / FWHM 15.43, amplitude 213.9 over background 1.74, saturation 2.68%. window 55 = +/-4 sigma; sigma_min/max and min_amplitude are 0.3x/4x/0.1x of measured, which is how com_survey derives its own suggestions. RE-RUN contrib/calibration-scripts/tools/com_survey_run.sh AND RE-DERIVE after any refocus or source change -- these are beam properties. An EARLIER set (sigma 6.31, window 53) was taken while the beam was being moved by hand and is superseded.

### `_replaced`

Replaced this center_of_mass block, kept for reversibility: {"region_height":20,"region_width":20,"threshold":5,"_min_peak":"hold the centroid instead of updating it when no beam is in frame; 40 against a measured beamless window peak of 5-6 and a saturating beam -- see steering_tuned.json for the full derivation","min_peak":40,"track":true,"_beam_loss":"Ten consecutive frames with no above-threshold signal assert AYLP_BEAM_LOST. FSP holds zero only while it remains set and resumes after reacquisition.","reacquire_after":10}

### `_fit_com_delay_hold`

NOT LIVE: the original measured candidate took 516.6 us p50 / 691.5 us p99 (0.243/0.326 frame at 471 Hz). The optimized moment-only candidate below takes 110.9 us p50 / 128.0 us p99 (0.052/0.060 frame) at 384x384 with a 243 window; stride-4 changed replay PSD bands by <=1.5% and synthetic position RMS from 0.027 to 0.046 px. Candidate: {"uri":"anyloop:fit_com","params":{"window_height":243,"window_width":243,"init_y":178,"init_x":177,"sigma_init":30.144,"sigma_min":9.043,"sigma_max":120.575,"min_amplitude":23.0,"max_step":11.2584,"fit_gaussian":false,"moment_output":true,"moment_col_stride":4,"moment_cut":2.0,"reacquire_after":37}}. Recheck measured loop delay when enabling it.


## fsp

### `_delay_warning`

RE-FIT 2026-07-31 from fsm_bodex_par2.dat / fsm_bodey_par2.dat (the 12:16 and 12:23 full sweeps): x 2.763 fr (global, tau 0.4654 ms at fs 3788.25), y 2.624 fr (override, tau 0.4288 ms at fs 3788.29). Both are full-band 5-300 Hz phase slopes and both include the one hidden bode frame. Against the 07-30 pair (2.717 / 2.577) that is +0.046 / +0.047 fr -- about 12 us, which at 300 Hz is ~1.3 deg against phase residuals of 1.71 / 2.69 deg, so the two sweeps AGREE and this is a re-confirmation, not a drift. Kept the newer/longer values because an over-modeled delay merely costs rejection while UNDER-modeling is the documented failure mode (the 2026-07-16 run under-modeled by 1.6-2.3 fr and spiraled at ~310 Hz).
UNLIKE K, delay must NEVER be taken from a *_small*.dat: those sweeps span 10-30 Hz, where the plant accumulates ~7 deg of phase, so their loop_frames outputs (1.09-2.02 fr, scattered run to run) are fitting noise. Only a full 5-300 Hz sweep fits tau. Re-fit if the ROI, exposure or bandwidth changes the frame rate -- delay in FRAMES moves with fs even when the delay in ms does not.

### `_K_warning`

ALWAYS TAKE K FROM THE SMALL-SIGNAL PLATEAU, and never straight off the fit line of a +-100 mV sweep. The loop commands ~6 mV rms / ~20 mV peak; a +-100 mV sweep drives an order of magnitude harder, and the FSM compresses by ~4% at that level. Current values, 2026-07-31: Kx 4.05, Ky 3.646.
Evidence, per axis, per amplitude (<=30 Hz plateau):
  x 07-30: 100 mV 3.815 / 50 mV 3.971 (ratio 1.041)   -- compressing
  x 07-31: 100 mV 4.050 / 50 mV 4.092 / 20 mV 4.056 / 8 mV 4.020 (ratio 1.003) -- flat
  y 07-30: 100 mV 3.664 / 50 mV 3.655 (ratio 0.998)   -- flat
  y 07-31: 100 mV 3.523 / 50 mV 3.662 / 20 mV 3.646 / 8 mV 3.668 (ratio 1.039) -- compressing
Two things follow. (1) The compression is REAL but it moves between axes day to day, so you cannot decide once which axis needs the small sweep -- always run the small sweep, or at minimum read the second (half-amplitude) pass of the full one. On 07-31 the raw fit lines say Kx 4.050 and Ky 3.510; the second is 3.7% low and would have under-modeled y. (2) The small-signal gain itself wanders a few percent over an hour: seven +-20 mV x sweeps read 4.08, 3.97, 4.35, 4.35, 4.36, 4.07, 4.06, each file internally consistent across both of its passes, so it is the plant/beam moving, not measurement noise. y drifted 3.706 -> 3.695 -> 3.645 -> 3.646 the same morning.
Since u = -phi_hat/K, configuring K BELOW the truth OVER-commands. If a run shows guard trips or ~690 Hz command activity, raise K rather than lowering it. Re-measure on any re-centre: K is bias- and ROI-dependent, and every number above was taken at offset 0 with ROI 1816/684, which is where this file runs.

### `_delay_2026_08_07`

MEASURED 2026-08-07 18:34-18:56 on the ASI290MM at 64x64 / exp 400 / gain 50, ROI 1084/414, fs_measured 2309.9 Hz, parport DAC, from fsm_bodex_par5/par6.dat and fsm_bodey_par5.dat. Sweeps 5-200 Hz, 60 pts, 49 cycles in 7 segments; K is the <=30 Hz weighted plateau and the delay is the full-band weighted phase slope, both refit offline after excluding sweep points whose integration window overlapped a logged beam outage (the bench was being worked on -- 9 to 1198 outage events per run). x delay = 3.018 fr (tau 0.8735 ms, phase resid 5.82 deg). DELAY IS COUPLED TO fit_com's SETTINGS, not just the transport. fit_com runs between the frame arriving and the command being written, so ITS COMPUTE TIME IS INSIDE THIS NUMBER. These delays were measured with max_iter 12 / max_us 0, profiling 0.1736 ms/frame (x) and 0.1233 ms/frame (y). THIS FILE STILL RUNS max_iter 4 / max_us 25, which profiled 0.045 ms/frame -- about 0.30 frames cheaper. Either match the sweep's fit_com settings here (preferred: at 4 iterations the fit under-reports fast beam motion by 32% at 200 Hz, so it costs rejection as well as delay accuracy), or subtract the profiled mean cost difference from the delay above. Do not mix. A bounded max_us ~200 us with max_iter 12 is the compromise for a loop: fit_com's per-frame cost ranges 0.002-0.27 ms uncapped, i.e. the loop delay jitters by up to ~0.6 frames, which a fixed-delay model cannot represent. RE-CHECK broad_lp AGAINST THIS DELAY -- it was not changed here. Regeneration sits at fs/(2*(delay+delay_frac)) = 2310/(2*3.018) = 383 Hz, where the 5-tap boxcar (null at fs/5 = 462 Hz) gives only -13.7 dB, against the -19.2 dB it gave at the fs 3788 / 2.763 fr it was tuned for. Re-derive it. CAMERA AND TRACKER IN THIS FILE ARE STALE AND WILL LOSE THE BEAM. It still has start_x/start_y 1088/416, gain 0 and min_amplitude 21.39 -- exactly the combination that failed on 2026-08-07 18:03, where the chopped source's dim phase fell under the gate and fit_com re-acquired several times a second WITH THE MIRROR PARKED. The sweep that produced the numbers above ran ROI 1084/414, gain 50, min_amplitude 10, sigma_init 8.2, window 0. Bring those over before closing the loop; see conf_bode_par_x.json's dated notes.

### `_fs`

3788 CONFIRMED on this transport: the par bodes measured 3788.32 (x) and 3786.38 (y) against the 3788 nominal, so the camera is still the bottleneck and the DAC adds nothing to the loop period. Re-confirm if ROI/exposure/bandwidth change, since delay in FRAMES depends on it.

### `_fs_2026_08_07`

fs 3788 -> 2310. 3788 was the 32x32/exp50 rate; this file runs 64x64/exp400, measured 2309.9 Hz by both sweeps. Delay in FRAMES is meaningless against the wrong fs, so this moves with the delay above.

### `_fs_1000hz_20pct`

PROVISIONAL 1000 Hz ceiling from the 1000 us exposure. Read the achieved loop rate from asi_source, put that measured value here, and remeasure both axis delays before enabling actuation.

### `_broad_lp_2026_08_07`

broad_lp 5 -> 7, re-derived for the new delay. The parasitic-loop regeneration frequency is fs/(2*(delay+delay_frac)), which the fs 3788 / 2.763 fr of the parport-era tuning put at ~685 Hz, where a 5-tap boxcar (null at fs/5) gave -19.2 dB. At fs 2310 and 3.018 fr the regeneration has moved DOWN to 382.7 Hz, where 5 taps give only -13.7 dB -- the filter no longer covers the band it exists to cover. Boxcar stopband at 382.7 Hz against group-delay cost, computed over the odd tap counts fsp accepts (fsp.c:1417 silently increments an even broad_lp, so 6 is not selectable even though its null would land almost exactly on 383 Hz):
   N= 5: -13.7 dB, group delay 2.0 fr, 30 Hz droop -0.06 dB
   N= 7: -17.2 dB, group delay 3.0 fr, 30 Hz droop -0.12 dB   <-- chosen
   N= 9: -13.0 dB, group delay 4.0 fr, 30 Hz droop -0.19 dB
   N=11: -20.3 dB, group delay 5.0 fr, 30 Hz droop -0.29 dB
   N=13: -22.9 dB, group delay 6.0 fr, 30 Hz droop -0.41 dB
7 restores approximately the stopband depth this filter was designed to have, for ONE extra frame of group delay. It is deliberately not 11 or 13: the deeper stopband costs prediction horizon, and PAR-6 already tested 9 against 5 and found it not worth keeping (net y -2% / x +10%), so horizon is the scarcer resource here. Group delay is folded into the broad horizon automatically (fsp.c:1423). RE-DERIVE AGAIN whenever the delay or fs changes -- the regeneration band moves with both.

### `_clamp_bringup_2026_07_30`

+-1 remains the operational bound: 0.27 V already sweeps the beam out of the 32x32 ROI, while +-1 provides about 59 px of authority against a +-16 px window.

### `_clamp_restored_2026_08_01`

The continuous predictor remains at +-1 V; full range is granted only to the detected-event recovery state.


## fsp — y axis

### `_K`

SMALL-SIGNAL PLATEAU 3.646 +/- 0.011 (fsm_bodey_par_small5.dat, +-20 mV, <=30 Hz, 2026-07-31 11:43), CONFIRMED by the half-amplitude pass of the newest full sweep: fsm_bodey_par2.dat reads 3.523 at +-100 mV but 3.662 at +-50 mV. So the 3.51 in that file's fit line is a LARGE-SIGNAL number and must not be copied here -- y compresses ~4% above ~50 mV, and the loop lives at ~20 mV. Consistent small-signal reads: 3.668 at 12 mV, 3.646 at 20 mV, 3.662 at 50 mV. Was the 0.66 piplate placeholder.

### `_delay`

MEASURED 2+0.624 = 2.624 fr (fsm_bodey_par2.dat, 2026-07-31 12:16, full-band 5-300 Hz phase slope, tau 0.4288 ms at fs 3788.29, one hidden bode frame already added), rounded to 0.62. Was 2.577 from fsm_bodey_par.dat on 07-30 -- a +0.047 fr shift, at the edge of the 2.69 deg phase residual, so treat it as a re-confirmation rather than a real change. Taking the newer/longer of the two is the safe direction: under-modeled delay is the documented failure mode.

### `_delay_2026_08_07`

MEASURED 2026-08-07 18:34-18:56 on the ASI290MM at 64x64 / exp 400 / gain 50, ROI 1084/414, fs_measured 2309.9 Hz, parport DAC, from fsm_bodex_par5/par6.dat and fsm_bodey_par5.dat. Sweeps 5-200 Hz, 60 pts, 49 cycles in 7 segments; K is the <=30 Hz weighted plateau and the delay is the full-band weighted phase slope, both refit offline after excluding sweep points whose integration window overlapped a logged beam outage (the bench was being worked on -- 9 to 1198 outage events per run). y delay = 3.127 fr (tau 0.9206 ms, phase resid 3.06 deg). Same fit_com coupling as the global delay.

### `_K_2026_08_07`

MEASURED 2026-08-07 18:34-18:56 on the ASI290MM at 64x64 / exp 400 / gain 50, ROI 1084/414, fs_measured 2309.9 Hz, parport DAC, from fsm_bodex_par5/par6.dat and fsm_bodey_par5.dat. Sweeps 5-200 Hz, 60 pts, 49 cycles in 7 segments; K is the <=30 Hz weighted plateau and the delay is the full-band weighted phase slope, both refit offline after excluding sweep points whose integration window overlapped a logged beam outage (the bench was being worked on -- 9 to 1198 outage events per run). Ky is NOT a direct reading at this timestamp -- only one y sweep survived (its second pass ran with no beam at all). It is Kx x 1.4087, the drift-immune ratio: the y sweep sat between two x sweeps, so x interpolates to y's exact time at the same 0.15 V drive, giving Kx 0.4394 against Ky 0.6190 measured. This assumes the drift is COMMON-MODE optical (focus/alignment/lever arm), which is what someone working at the bench produces. If one axis drifted mechanically on its own, the ratio does not carry -- re-measure y directly. *** THE GAIN WAS DRIFTING WHEN THIS WAS MEASURED: -39% PER HOUR. *** Eight pass-level K values across 45 minutes fit a pure time-drift model to 3.5% rms; K ran 0.5794 (18:10) -> 0.5027 (18:37) -> 0.4054 (18:54). The value above is the LAST and therefore LOWEST measurement, and it carries a timestamp for a reason: if the drift continued, the true K is now lower still, which is the SAFE direction (configured K above truth under-commands). If the bench work ended and alignment recovered, the true K is HIGHER than this and the configured value OVER-COMMANDS -- the documented failure mode. RE-MEASURE BEFORE ANY UNATTENDED RUN. Corollary, and it retires an old warning: across 0.05-0.30 V the amplitude dependence fits to +1.8% per halving, consistent with zero (bound ~4%). The FSM is LINEAR over that range. Every 'NONLINEAR/CLIPPING' verdict bode_plot emitted this session was drift aliased onto amplitude, because bode_plot always runs full amplitude first and half amplitude ~3 min later, confounding the two perfectly.

### `_K_raised_2026_08_07`

K RAISED TO THE TOP OF THE MEASURED RANGE ON PURPOSE. The session measured Kx between 0.405 (18:54) and 0.579 (18:10) and cannot say which end is real. Since u = -phi_hat/K, the realised loop gain is K_true/K_cfg: configuring 0.405 bounds the worst case at 1.43 (43% OVER-command into a 3-frame delay), while configuring 0.579 bounds it at 0.70 (under-corrects, stable, costs rejection). For an unattended run that is not a close call, and it is what this file's own _K_warning has always said: raise K rather than lower it. Ky = 1.4087 x Kx, the drift-immune cross-axis ratio (the y sweep sat between two x sweeps, so x interpolates to y's exact time at matched drive). *** BOTH VALUES PREDATE THE REFOCUS DESCRIBED ON THE CAMERA STAGE -- RE-MEASURE BEFORE TRUSTING EITHER. *** They are here so the file is self-consistent, not because they describe the current optics.


## fsp — x axis

### `_K`

4.05, and it is now the SAME number from every amplitude: fsm_bodex_par2.dat (2026-07-31 12:23, +-100 mV) fits 4.0500 +/- 0.0037 with its +-50 mV pass at 4.092, and fsm_bodex_par_small7.dat (+-20 mV) fits 4.056 +/- 0.023 with its +-8 mV pass at 4.020. Ratio between the 100 and 50 mV passes is 1.003, i.e. x's compression is GONE -- on 07-30 the same pair read 3.815/3.971 (ratio 1.041). So the 3.787 that PAR-1..3 ran was not a large-signal artefact of a plant that also reads 4.05 small-signal; the plant itself moved. See _K_warning. Was the 0.915 piplate placeholder.

### `_delay`

x inherits the global 2+0.76 = 2.763 fr (fsm_bodex_par2.dat, 2026-07-31 12:23, full-band 5-300 Hz phase slope, tau 0.4654 ms at fs 3788.25, one hidden bode frame already added). Was 2.717 from fsm_bodex_par.dat on 07-30 -- +0.046 fr, comparable to the 1.71 deg phase residual, so a re-confirmation more than a change; the newer/longer value is also the safe side.

### `_K_2026_08_07`

MEASURED 2026-08-07 18:34-18:56 on the ASI290MM at 64x64 / exp 400 / gain 50, ROI 1084/414, fs_measured 2309.9 Hz, parport DAC, from fsm_bodex_par5/par6.dat and fsm_bodey_par5.dat. Sweeps 5-200 Hz, 60 pts, 49 cycles in 7 segments; K is the <=30 Hz weighted plateau and the delay is the full-band weighted phase slope, both refit offline after excluding sweep points whose integration window overlapped a logged beam outage (the bench was being worked on -- 9 to 1198 outage events per run). Kx = 0.4054 +/- 0.0055 at 18:54, the weighted mean of fsm_bodex_par6's two passes (0.4051 at 0.30 V, 0.4061 at 0.15 V, 3 min apart, agreeing to 0.2%). *** THE GAIN WAS DRIFTING WHEN THIS WAS MEASURED: -39% PER HOUR. *** Eight pass-level K values across 45 minutes fit a pure time-drift model to 3.5% rms; K ran 0.5794 (18:10) -> 0.5027 (18:37) -> 0.4054 (18:54). The value above is the LAST and therefore LOWEST measurement, and it carries a timestamp for a reason: if the drift continued, the true K is now lower still, which is the SAFE direction (configured K above truth under-commands). If the bench work ended and alignment recovered, the true K is HIGHER than this and the configured value OVER-COMMANDS -- the documented failure mode. RE-MEASURE BEFORE ANY UNATTENDED RUN. Corollary, and it retires an old warning: across 0.05-0.30 V the amplitude dependence fits to +1.8% per halving, consistent with zero (bound ~4%). The FSM is LINEAR over that range. Every 'NONLINEAR/CLIPPING' verdict bode_plot emitted this session was drift aliased onto amplitude, because bode_plot always runs full amplitude first and half amplitude ~3 min later, confounding the two perfectly. SIGN RESOLVED 2026-08-07: scale is [-1, 1] = [ch0 y, ch1 x] and CONFIRMED CORRECT on the bench. That matches what both bode sweeps measured a positive in-phase K with (x at +1.0, y at -1.0), which is the convention fsp's u = -phi_hat/K assumes. The earlier [1, -1] would have been positive feedback on both axes. If this ever needs changing again, fix it in the parport_dac scale, never by flipping K.

### `_K_raised_2026_08_07`

K RAISED TO THE TOP OF THE MEASURED RANGE ON PURPOSE. The session measured Kx between 0.405 (18:54) and 0.579 (18:10) and cannot say which end is real. Since u = -phi_hat/K, the realised loop gain is K_true/K_cfg: configuring 0.405 bounds the worst case at 1.43 (43% OVER-command into a 3-frame delay), while configuring 0.579 bounds it at 0.70 (under-corrects, stable, costs rejection). For an unattended run that is not a close call, and it is what this file's own _K_warning has always said: raise K rather than lower it. Ky = 1.4087 x Kx, the drift-immune cross-axis ratio (the y sweep sat between two x sweeps, so x interpolates to y's exact time at matched drive). *** BOTH VALUES PREDATE THE REFOCUS DESCRIBED ON THE CAMERA STAGE -- RE-MEASURE BEFORE TRUSTING EITHER. *** They are here so the file is self-consistent, not because they describe the current optics.


## parport_dac

### `_scale_sign_2026_08_07`

SIGN FLIPPED ON BOTH AXES 2026-08-07: was [1, -1] (y +1, x -1), now [-1, 1] (y -1, x +1). Order is [ch0, ch1] = [y, x]. Both bode sweeps measure a POSITIVE, in-phase K -- phase at 5 Hz is +0.037 rad (x, scale +1.0) and +0.028 rad (y, scale -1.0), i.e. ~0 not ~pi -- so these are the signs that make the plant gain positive, which is what fsp's u = -phi_hat/K assumes. The old signs would have been POSITIVE FEEDBACK on both axes. A camera remount flipping image parity in both axes explains the change. THIS IS INFERRED FROM SWEEP PHASE, NOT FROM A PUSH TEST -- confirm with a push test before closing the loop, and if it disagrees fix it HERE, never by flipping the sign of K.
