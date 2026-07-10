anyloop:center_of_mass
======================

Types and units: `[T_MATRIX_UCHAR, U_ANY] -> [T_VECTOR, U_MINMAX]`.

This device breaks up an image into one or more regions, and calculates the
center-of-mass coordinate of that image. For example, this device might be used
with only one region to determine the center-of-mass coordinates of a beam on a
camera, which can then be used to control a tip-tilt mirror to recenter said
beam. This device is also used with many regions for getting error signals from
a wavefront sensor.

An example configuration for a wavefront sensor with 8x8-pixel subapertures:

```json
{
  "uri": "anyloop:center_of_mass",
    "params": {
      "region_height": 8,
      "region_width": 8,
      "thread_count": 1
    }
}
```

Pipeline data is replaced with a vector of interleaved center-of-mass y and x
coordinates (a vector of length 2N, where N is the number of regions of
interest). For example, if the input has four regions of interest, the output
will be [y1,x1,y2,x2,y3,x3,y4,x4] where each y,x is from -1 to 1, where 0 means
perfectly centered in the region of interest. It is assumed that the input is
written in order of increasing x coordinate, then increasing y coordinate.

Parameters
----------

- `region_height` (integer) (required)
  - Height of each region to find the center of mass of. The image will be split
    up into regions of this height, from the top going down. Excess data will be
    ignored. Set this to 0 to set the region height to the logical height of the
    whole image.
- `region_width` (integer) (required)
  - Width of each region to find the center of mass of. The image will be split
    up into regions of this width, from left to right. Excess data will be
    ignored. Set this to 0 to set the region width to the logical height of the
    whole image.
- `thread_count` (integer) (optional)
  - Number of threads to use for the calculation. Set this to 1 (default) for no
    multithreading. Ignored when `track` is set.
- `track` (boolean) (optional)
  - Confine the sum to a single `region_height` by `region_width` window centred
    on the previous frame's center of mass. Defaults to false.
- `init_y`, `init_x` (integer) (optional)
  - Initial window centre, in image pixels. Must be given together. If omitted,
    the window is acquired from the brightest pixel of the first frame.
- `reacquire_after` (integer) (optional)
  - Consecutive frames of zero signal inside the window before re-acquiring from
    the brightest pixel of the whole image. Defaults to 10.
- `acquire_seconds` (float) (optional)
  - Run with a wide acquisition window for this long before narrowing to
    `region_height`/`region_width`. Defaults to 0 (no acquisition phase). Also
    re-entered whenever the window re-acquires after losing the beam.
- `acquire_height`, `acquire_width` (integer) (optional)
  - Size of the acquisition window. Default 0, meaning the whole image.

Tracking window
---------------

Center of mass is a flux-weighted average, so anything bright in the region
contributes. A stray reflection alongside the beam does two things: it pulls the
centroid off the beam by a fixed offset, and — the one that matters — it
attenuates the response to real beam motion by `S_beam / (S_beam + S_reflection)`,
silently scaling your loop gain.

Setting `track` confines the sum to a window that follows the beam, so a
reflection outside that window never enters the sum. The image is passed down the
pipeline untouched, so a `udp_sink` placed *before* this device still shows the
whole frame, reflection and all.

```json
{
  "uri": "anyloop:center_of_mass",
    "params": {
      "region_height": 25,
      "region_width": 25,
      "threshold": 20,
      "track": true
    }
}
```

The output is normalized across the **whole image**, not the window. This is what
makes the mode usable in a control loop: the window chases the beam, so a
window-relative coordinate would sit near zero no matter where the beam actually
was, and the loop would have no error signal to act on. Because the normalization
is to the image, the setpoint stays the image centre and the error per pixel of
beam motion — hence the loop gain — does not change with window size. You can
resize the window freely without retuning the PID.

Behaviour worth knowing:

- **Acquisition.** By default the window lands on the brightest pixel of the
  first frame. That scan keeps the *first* maximum it meets in raster order, so
  when the beam and a reflection both saturate at 255 the one nearer the top of
  the frame wins — which may well be the reflection.

  `acquire_seconds` fixes this without hardcoding a position: for that long the
  sum runs over a wide window (the whole image by default), where the centroid is
  flux-weighted across everything present and therefore drawn toward whichever
  spot carries the most *light*. When the phase ends, the window narrows onto
  wherever that centroid settled, and because it re-centres every frame it then
  walks onto the dominant spot.

  This works **iff the beam carries more total flux than the reflection**. Peak
  brightness is irrelevant once both saturate; what matters is the integral above
  `threshold`. At an even flux split the window stalls between the two spots, and
  past that it converges onto the reflection. When the beam does not dominate,
  use `init_y`/`init_x` — that is deterministic and needs no acquisition phase.
- **How close the beam may get.** The window reaches `region_height/2` from its
  centre. Contamination returns once the reflection's above-threshold pixels
  reach inside that, i.e. when the beam-to-reflection separation drops below
  roughly `region/2 + reflection_radius`. Size the window as small as the beam's
  per-frame motion and spot size allow.
- **Loss of signal.** If every pixel in the window is at or below `threshold`,
  the last valid output is held rather than reporting `(0, 0)` — which downstream
  reads as "perfectly centred", not "no signal", and would let an integrator park
  and then lurch when the beam reappears. After `reacquire_after` such frames the
  window re-acquires from the brightest pixel of the whole image, so a stranded
  window can recover. Note that this can lock onto a reflection if the beam is
  genuinely gone.
- **Edges.** The window is clamped to lie inside the image, so a beam pinned
  against an edge reports a coordinate near ±1 rather than reading out of bounds.

Without `track`, the device behaves exactly as before: the region grid starts at
the top-left of the image and tiles across it, and the output is normalized per
region.

