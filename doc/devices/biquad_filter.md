anyloop:biquad_filter
=====================

Types and units: `[T_VECTOR|T_MATRIX, U_ANY] -> [T_UNCHANGED, U_UNCHANGED]`.

A second-order IIR filter (one "biquad" section) applied independently to every
element of the pipeline state. Use it ahead of `anyloop:pid` to keep loop gain
off a mechanical resonance: a `lowpass` rolls off everything above its corner,
and a `notch` cuts a single frequency.

The filter is a Direct Form I difference equation,

```
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
```

with one set of past samples kept per element. The coefficients are computed
once at init from `f0`, `q`, and `fs` using the bilinear transform. Frequency
pre-warping is built in (the coefficients come from cos/sin of the *digital*
frequency `w0 = 2*pi*f0/fs`), so the feature lands on `f0` even when `f0` is a
large fraction of Nyquist — important here, since 80 Hz at fs=333 Hz is ~0.48
Nyquist and a naive design would land noticeably low.

Note that `fs` is assumed constant. If the loop rate jitters badly (check with
`-p`), the realized corner/notch frequency will wander proportionally.

Parameters
----------

- `type` (string) (required)
  - "vector" or "matrix", matching the pipeline data.
- `mode` (string) (optional)
  - "lowpass" or "notch". Defaults to "lowpass".
- `f0` (float) (required)
  - Corner frequency (lowpass) or notch center (notch), in Hz. Must be in
    (0, fs/2).
- `q` (float) (optional)
  - Quality factor. For a lowpass, ~0.707 is maximally flat (the default); for
    a notch, set it near the resonance's own Q (width = f0/Q) so the notch is
    about as wide as the peak it cancels.
- `fs` (float) (required)
  - Sample rate in Hz that the coefficients are designed for (your loop rate).
- `units` (string) (optional)
  - Output units. Defaults to null (unchanged from input).
