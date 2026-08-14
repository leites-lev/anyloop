# Registration COM runtime and A/B comparison — 2026-08-13

The registration tracker was optimized while preserving all numerical tests.
The normal path now caches keyframe samples/gradients, reuses interpolated
residuals, uses linear-time median selection, accumulates only the unique
symmetric normal-matrix terms, and skips the full centroid scan after the
registration origin is established.

All eight Meson suites pass. Replay of all 13 available camera captures also
passes. Steady-state timing was approximately 42–43 us median for 256x256,
53 us for 320x320, and 64 us for 384x384; observed p99 values were below
91 us. Keyframe installation/reacquisition is a separate one-time slow path
(roughly 7–15 ms in these debug/replay measurements) and must not be used as
the steady-state delay estimate.

Paired replay used identical `threshold=1`, `min_peak=100`, tracking windows,
and accepted illuminated frames for registration on/off. Across 16,198 frames:

- combined position RMS: 6.23 px with registration versus 15.58 px without,
  a 60.0% reduction;
- frame-to-frame/HF RMS: 0.090 px versus 1.981 px, a 95.5% reduction;
- every frame accepted by registration was also accepted by the plain tracker;
- individual combined-RMS improvement ranged from 13% to 90%.

This is evidence that ordinary centroid motion is strongly contaminated by
illumination/shape/exposure artifacts in these captures, not proof that every
removed excursion was nonphysical. Injected-motion recovery remains the
absolute accuracy check (about 0.016 px pooled radial RMS and unity gain).

Canonical steering profiles now set `registration=true` after replay
validation. A live A/B run still needs to measure the complete camera-to-DAC
delay with registration enabled. The old inline JSON notes saying registration
was disabled because it diverged after 17 seconds are stale: that was the
pre-fix local-keyframe implementation.

The 1000-Hz registration profiles use automatic COM setup for `threshold`,
`min_peak`, and `init_y`/`init_x`; `reacquire_after` remains an explicit 1000
frames.
Auto threshold uses the 95th-percentile border background; auto min peak starts
three counts above it and tracks 10% of the accepted mean signal; auto init
uses the brightest adequate frame.  Auto reacquisition remains available for
future profiles, but these 1000-Hz profiles deliberately retain the fixed
1000-frame interval.
