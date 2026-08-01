#!/usr/bin/env python3
"""Fit FSP freqs/zeta/q from an open-loop CoM AYLP recording.

The preferred input is the full-rate pre-FSP error record written by
attenuation_par_fsp.json. With --config, attenuation_test's start_delay and
open_time select the genuinely open-loop interval automatically. The script is
dependency-free at the Python level; it uses the same system GSL library as
Anyloop for fast FFTs.

This is output-only modal identification. K and delay must still come from the
separate command->centroid Bode tests.
"""

import argparse
import ctypes
import ctypes.util
import json
import math
import mmap
import statistics
import struct
import sys
from array import array
from pathlib import Path


AYLP_HEADER = 40
AYLP_MAGIC = 0x504C5941
AYLP_VECTOR = 1 << 2


def pipeline_stage(config, uri):
    for stage in config.get("pipeline", []):
        if stage.get("uri") == uri:
            return stage.get("params", {})
    return None


def selection_from_config(config):
    fsp = pipeline_stage(config, "anyloop:fsp")
    if fsp is None:
        raise ValueError("config has no anyloop:fsp stage")
    test = pipeline_stage(config, "anyloop:attenuation_test")
    if test is None:
        return float(fsp["fs"]), 0.0, None
    return (float(fsp["fs"]), float(test.get("start_delay", 0.0)),
            float(test["open_time"]))


def read_aylp_window(path, start, stop):
    """Read only a selected frame interval from a fixed-size AYLP vector."""
    with path.open("rb") as stream:
        header = stream.read(AYLP_HEADER)
        if (len(header) != AYLP_HEADER
                or struct.unpack_from("<I", header)[0] != AYLP_MAGIC):
            raise ValueError(f"{path}: invalid AYLP header")
        kind = header[6]
        ny, nx = struct.unpack_from("<QQ", header, 8)
        count = ny * nx
        if kind != AYLP_VECTOR or count < 2:
            raise ValueError(f"{path}: expected a two-element AYLP vector")
        frame_bytes = AYLP_HEADER + count * 8
        byte_count = path.stat().st_size
        if byte_count % frame_bytes:
            raise ValueError(f"{path}: truncated or variable-sized AYLP stream")
        frames = byte_count // frame_bytes
        stop = min(stop, frames)
        if start < 0 or stop <= start:
            raise ValueError("selected open-loop window is outside the recording")
        mapped = mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ)
        axes = (array("d"), array("d"))
        for frame in range(start, stop):
            y, x = struct.unpack_from("<dd", mapped,
                                      frame * frame_bytes + AYLP_HEADER)
            axes[0].append(y)
            axes[1].append(x)
        mapped.close()
    return axes


def gsl_fft_functions():
    name = ctypes.util.find_library("gsl")
    if not name:
        raise RuntimeError("libgsl is required (Anyloop itself also links GSL)")
    library = ctypes.CDLL(name)
    transform = library.gsl_fft_real_radix2_transform
    transform.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
                          ctypes.c_size_t]
    transform.restype = ctypes.c_int
    unpack = library.gsl_fft_halfcomplex_radix2_unpack
    unpack.argtypes = [ctypes.POINTER(ctypes.c_double),
                       ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
                       ctypes.c_size_t]
    unpack.restype = ctypes.c_int
    return transform, unpack


def largest_power_of_two(value):
    return 1 << (int(value).bit_length() - 1)


def welch_psd(values, fs, segment_s):
    length = largest_power_of_two(min(len(values), round(segment_s * fs)))
    if length < 256:
        raise ValueError("open interval is too short for modal fitting")
    step = length // 2
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (length - 1))
              for i in range(length)]
    window_energy = sum(value * value for value in window)
    bins = length // 2 + 1
    segment_psds = []
    transform, unpack = gsl_fft_functions()
    segments = 0
    for start in range(0, len(values) - length + 1, step):
        mean = sum(values[start:start + length]) / length
        packed = array("d", ((values[start+i] - mean) * window[i]
                             for i in range(length)))
        spectrum = array("d", [0.0]) * (2 * length)
        packed_ptr = (ctypes.c_double * length).from_buffer(packed)
        spectrum_ptr = (ctypes.c_double * (2 * length)).from_buffer(spectrum)
        if transform(packed_ptr, 1, length) or \
                unpack(packed_ptr, spectrum_ptr, 1, length):
            raise RuntimeError("GSL FFT failed")
        scale = 1.0 / (fs * window_energy)
        segment_psd = array("d", [0.0]) * bins
        for index in range(bins):
            real = spectrum[2*index]
            imag = spectrum[2*index+1]
            power = (real*real + imag*imag) * scale
            if 0 < index < bins - 1:
                power *= 2.0
            segment_psd[index] = power
        segment_psds.append(segment_psd)
        segments += 1
    if not segments:
        raise ValueError("open interval contains no complete Welch segment")
    frequency = [index * fs / length for index in range(bins)]
    # Median Welch rejects an isolated accidental touch during the nominally
    # quiet open phase without letting the controller's closed phase leak in.
    psd = [statistics.median(segment[index] for segment in segment_psds)
           for index in range(bins)]
    return frequency, psd, segments


def running_median(values, width):
    width = max(3, int(width) | 1)
    half = width // 2
    result = []
    for index in range(len(values)):
        lo = max(0, index-half)
        hi = min(len(values), index+half+1)
        result.append(statistics.median(values[lo:hi]))
    return result


def peak_candidates(frequency, psd, fmin, fmax, prominence_db, min_sep,
                    candidate_limit):
    df = frequency[1] - frequency[0]
    lo = max(1, math.ceil(fmin / df))
    hi = min(len(frequency)-1, math.floor(fmax / df)+1)
    logp = [10.0 * math.log10(value + 1e-300) for value in psd]
    floor = running_median(logp[lo-1:hi+1], round(2.0 / df))
    candidates = []
    for index in range(lo, hi):
        prominence = logp[index] - floor[index-(lo-1)]
        if (prominence >= prominence_db and psd[index] >= psd[index-1]
                and psd[index] >= psd[index+1]):
            candidates.append((prominence, psd[index], index))
    candidates.sort(reverse=True)
    retained = []
    for candidate in candidates:
        if all(abs(frequency[candidate[2]] - frequency[other[2]]) >= min_sep
               for other in retained):
            retained.append(candidate)
        if len(retained) >= candidate_limit:
            break
    return retained


def positive_linear_fit(shape, values):
    count = len(shape)
    sx = sum(shape)
    sy = sum(values)
    sxx = sum(value*value for value in shape)
    sxy = sum(x*y for x, y in zip(shape, values))
    determinant = count*sxx - sx*sx
    amplitude = (count*sxy - sx*sy) / determinant if determinant else 0.0
    baseline = (sy - amplitude*sx) / count
    return max(baseline, 1e-300), max(amplitude, 0.0)


def fit_lorentzian(frequency, psd, peak_index, fit_width, zeta_min,
                   zeta_max):
    """Fit B + A/(1+((f-f0)/gamma)^2), where zeta ~= gamma/f0."""
    df = frequency[1] - frequency[0]
    seed = frequency[peak_index]
    lo = max(0, math.ceil((seed-fit_width)/df))
    hi = min(len(frequency), math.floor((seed+fit_width)/df)+1)
    f, p = frequency[lo:hi], psd[lo:hi]
    if len(f) < 7:
        raise ValueError(f"insufficient bins around {seed:g} Hz")
    gamma_lo = max(0.5*df, zeta_min*seed)
    gamma_hi = max(gamma_lo*1.01, min(fit_width, zeta_max*seed))
    gammas = [gamma_lo * (gamma_hi/gamma_lo)**(i/59.0) for i in range(60)]
    best = None
    for offset in range(-4, 5):
        centre = seed + 0.25*offset*df
        for gamma in gammas:
            shape = [1.0/(1.0+((value-centre)/gamma)**2) for value in f]
            baseline, amplitude = positive_linear_fit(shape, p)
            score = sum((math.log(baseline+amplitude*x)
                         - math.log(max(y, 1e-300)))**2
                        for x, y in zip(shape, p)) / len(p)
            if best is None or score < best[0]:
                best = score, centre, gamma, baseline, amplitude
    _, centre, gamma, baseline, amplitude = best
    return (centre, gamma/max(centre, df), math.pi*amplitude*gamma,
            baseline, gamma <= gamma_lo*1.02)


def stationary_gain(frequency, zeta, fs):
    omega = 2.0 * math.pi * frequency / fs
    radius = math.exp(-zeta * omega)
    a1 = 2.0*radius*math.cos(omega*math.sqrt(max(0.0, 1-zeta*zeta)))
    a2 = -radius*radius
    d1 = max(1e-12, 1.0-a1-a2)
    d2 = max(1e-12, 1.0+a1-a2)
    d3 = max(1e-12, 1.0+a2)
    return (1.0-a2)/(d3*d1*d2)


def integrate(values, df, lo, hi):
    if hi <= lo:
        return 0.0
    return sum(values[lo+1:hi-1])*df \
        + 0.5*df*(values[lo]+values[hi-1])


def fit_axis(values, fs, args):
    frequency, psd, averages = welch_psd(values, fs, args.segment_s)
    candidates = peak_candidates(
        frequency, psd, args.fmin, args.fmax, args.prominence_db,
        args.min_sep_hz, args.max_modes*3)
    fits = []
    for prominence, _, index in candidates:
        centre, zeta, variance, baseline, unresolved = fit_lorentzian(
            frequency, psd, index, args.fit_width_hz,
            args.zeta_min, args.zeta_max)
        if variance > 0.0:
            fits.append({"frequency": centre, "zeta": zeta,
                         "variance": variance, "prominence_db": prominence,
                         "baseline": baseline, "unresolved": unresolved})
    fits.sort(key=lambda item: item["variance"], reverse=True)
    fits = fits[:args.max_modes]
    fits.sort(key=lambda item: item["frequency"])

    residual = list(psd)
    for item in fits:
        gamma = item["zeta"]*item["frequency"]
        amplitude = item["variance"]/(math.pi*gamma)
        for index, value in enumerate(frequency):
            shape = 1.0/(1.0+((value-item["frequency"])/gamma)**2)
            residual[index] = max(0.0, residual[index]-amplitude*shape)
    df = frequency[1]-frequency[0]
    lo = max(0, math.ceil(args.noise_fmin/df))
    hi = min(len(frequency), math.floor(args.noise_fmax/df)+1)
    residual_variance = max(integrate(residual, df, lo, hi), 1e-300)
    for item in fits:
        gv = stationary_gain(item["frequency"], item["zeta"], fs)
        item["q"] = item["variance"]/gv/residual_variance
    # A long spectrum contains occasional locally prominent noise maxima. They
    # fit vanishing q/r and should not consume the controller's eight states.
    fits = [item for item in fits if item["q"] >= args.min_q_ratio]
    return fits, residual_variance, df, averages


def main():
    parser = argparse.ArgumentParser(
        description="Fit FSP freqs, zeta and q/r from open-loop AYLP data.")
    parser.add_argument("aylp", type=Path, help="full-rate pre-FSP [y,x] error record")
    parser.add_argument("--config", required=True, type=Path,
                        help="attenuation config used to record the data")
    parser.add_argument("--start-s", type=float,
                        help="override open-window start from attenuation_test")
    parser.add_argument("--duration-s", type=float,
                        help="override open-window duration from attenuation_test")
    parser.add_argument("--segment-s", type=float, default=90.0)
    parser.add_argument("--fmin", type=float, default=4.0)
    parser.add_argument("--fmax", type=float, default=150.0)
    parser.add_argument("--noise-fmin", type=float, default=1.0)
    parser.add_argument("--noise-fmax", type=float, default=400.0)
    parser.add_argument("--prominence-db", type=float, default=4.0)
    parser.add_argument("--min-q-r", dest="min_q_ratio", type=float,
                        default=1e-7,
                        help="discard fitted modes below this q/r (default 1e-7)")
    parser.add_argument("--min-sep-hz", type=float, default=0.75)
    parser.add_argument("--fit-width-hz", type=float, default=1.0)
    parser.add_argument("--zeta-min", type=float, default=0.0005)
    parser.add_argument("--zeta-max", type=float, default=0.30)
    parser.add_argument("--max-modes", type=int, default=8)
    parser.add_argument("--output", type=Path,
                        help="also write the fitted JSON fragment here")
    args = parser.parse_args()
    if (args.segment_s <= 0 or args.fmin <= 0 or args.fmax <= args.fmin
            or args.noise_fmin < 0 or args.noise_fmax <= args.noise_fmin
            or args.prominence_db <= 0 or args.min_sep_hz <= 0
            or args.min_q_ratio < 0
            or args.fit_width_hz <= 0 or args.zeta_min <= 0
            or args.zeta_max <= args.zeta_min
            or not 1 <= args.max_modes <= 8):
        parser.error("invalid fitting limits")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    fs, auto_start, auto_duration = selection_from_config(config)
    start_s = auto_start if args.start_s is None else args.start_s
    duration_s = auto_duration if args.duration_s is None else args.duration_s
    if duration_s is None:
        parser.error("config has no attenuation_test; provide --duration-s")
    start = round(start_s*fs)
    requested_stop = start + round(duration_s*fs)
    try:
        axes = read_aylp_window(args.aylp, start, requested_stop)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"# open window {start_s:g}..{start_s+duration_s:g} s, "
          f"{len(axes[0])} frames at {fs:g} Hz", file=sys.stderr)
    result = {}
    for axis_name, values in zip(("y", "x"), axes):
        fits, residual, resolution, averages = fit_axis(values, fs, args)
        if not fits:
            raise SystemExit(f"no qualifying {axis_name}-axis modes found")
        result[axis_name] = {
            "r": 1.0,
            "freqs": [round(item["frequency"], 3) for item in fits],
            "zeta": [round(item["zeta"], 6) for item in fits],
            "q": [float(f"{item['q']:.6g}") for item in fits],
        }
        print(f"# {axis_name}: resolution {resolution:.4g} Hz, {averages} "
              f"Welch segments, residual variance {residual:.4g}",
              file=sys.stderr)
        for item in fits:
            note = " [zeta resolution-limited]" if item["unresolved"] else ""
            print(f"#   {item['frequency']:8.3f} Hz  "
                  f"zeta={item['zeta']:.5g}  q/r={item['q']:.4g}  "
                  f"power={item['variance']:.4g}  "
                  f"prom={item['prominence_db']:.1f} dB{note}",
                  file=sys.stderr)
    rendered = json.dumps(result, indent=2)+"\n"
    print(rendered, end="")
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
        print(f"# wrote {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
