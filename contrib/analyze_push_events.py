#!/usr/bin/env python3
"""Summarize and compare FSP push-event CSV logs.

The FSP writes normalized centroid error.  For the 32-pixel ROI used by the
parallel-DAC push configuration, one normalized unit is 15.5 pixels.
"""

import argparse
import collections
import csv
import json
import math
import statistics
import struct
from pathlib import Path


FIELDS = (
    ("error_settle_s", "settle s", 1.0),
    ("innovation_quiet_s", "forcing s", 1.0),
    ("recovery_s", "recovery s", 1.0),
    ("peak_abs_error", "peak px", None),
    ("error_rms", "event RMS px", None),
    ("peak_abs_command", "peak cmd", 1.0),
    ("peak_abs_integral", "peak I cmd", 1.0),
)

AYLP_HEADER = 40
AYLP_MAGIC = 0x504C5941
AYLP_VECTOR = 1 << 2


def read_aylp_vectors(path: Path):
    """Read a double-vector AYLP stream as a pair of per-axis lists."""
    axes = [[], []]
    with path.open("rb") as stream:
        frame = 0
        while True:
            header = stream.read(AYLP_HEADER)
            if not header:
                break
            if len(header) != AYLP_HEADER:
                raise ValueError(f"{path}: truncated header at frame {frame}")
            magic = struct.unpack_from("<I", header)[0]
            kind = header[6]
            ny, nx = struct.unpack_from("<QQ", header, 8)
            count = ny * nx
            if magic != AYLP_MAGIC or kind != AYLP_VECTOR or count < 2:
                raise ValueError(
                    f"{path}: frame {frame} is not a two-element AYLP vector"
                )
            payload = stream.read(count * 8)
            if len(payload) != count * 8:
                raise ValueError(f"{path}: truncated payload at frame {frame}")
            y, x = struct.unpack_from("<dd", payload)
            axes[0].append(y)
            axes[1].append(x)
            frame += 1
    if not axes[0]:
        raise ValueError(f"{path}: no frames")
    return axes


def fsp_params(path: Path):
    config = json.loads(path.read_text(encoding="utf-8"))
    for stage in config.get("pipeline", []):
        if stage.get("uri") == "anyloop:fsp":
            return stage.get("params", {})
    raise ValueError(f"{path}: no anyloop:fsp stage")


def axis_plant(params, axis):
    axis_params = params.get(axis, {})
    delay = int(axis_params.get("delay", params.get("delay", 0)))
    frac = float(axis_params.get("delay_frac", params.get("delay_frac", 0.0)))
    gain = float(axis_params["K"])
    freqs = [float(value) for value in axis_params.get("freqs", [])]
    plant_b = axis_params.get("plant_b", params.get("plant_b"))
    plant_a = axis_params.get("plant_a", params.get("plant_a"))
    return delay, frac, gain, freqs, plant_b, plant_a


def thiran_delay(values, delay, frac):
    """Apply the same fractional all-pass and integer delay as FSP's echo."""
    a = (1.0 - frac) / (1.0 + frac)
    x1 = y1 = 0.0
    fractional = []
    for value in values:
        out = a * value + x1 - a * y1
        x1, y1 = value, out
        fractional.append(out)
    if delay <= 0:
        return fractional
    return [0.0] * delay + fractional[:-delay]


def biquad(values, b, a):
    if not b or not a:
        return values
    if len(b) != 3 or len(a) != 3:
        raise ValueError("plant_b and plant_a must each contain three values")
    b0, b1, b2 = map(float, b)
    _, a1, a2 = map(float, a)
    z1 = z2 = 0.0
    result = []
    for value in values:
        out = b0 * value + z1
        z1 = b1 * value - a1 * out + z2
        z2 = b2 * value - a2 * out
        result.append(out)
    return result


def reconstruct_disturbance(error, command, delay, frac, gain, plant_b, plant_a):
    echo = biquad(thiran_delay(command, delay, frac), plant_b, plant_a)
    return [measured - gain * applied for measured, applied in zip(error, echo)]


def detrend(values):
    n = len(values)
    if n < 2:
        return list(values)
    centre = (n - 1) / 2.0
    mean = sum(values) / n
    denom = sum((i - centre) ** 2 for i in range(n))
    slope = sum((i - centre) * (value - mean)
                for i, value in enumerate(values)) / denom
    return [value - mean - slope * (i - centre)
            for i, value in enumerate(values)]


def spectral_power(values, fs, frequency):
    """Hann-windowed single-frequency power using an oscillator recurrence."""
    n = len(values)
    omega = 2.0 * math.pi * frequency / fs
    cw, sw = math.cos(omega), math.sin(omega)
    c, s = 1.0, 0.0
    real = imag = 0.0
    for i, value in enumerate(values):
        window = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / max(n - 1, 1))
        sample = window * value
        real += sample * c
        imag -= sample * s
        c, s = c * cw - s * sw, s * cw + c * sw
    return real * real + imag * imag


def local_frequency(values, fs, nominal, search_hz):
    duration = len(values) / fs
    step = max(0.02, 1.0 / max(8.0 * duration, 1.0))
    lo = max(step, nominal - search_hz)
    hi = min(0.45 * fs, nominal + search_hz)
    frequencies = []
    powers = []
    frequency = lo
    while frequency <= hi + 0.5 * step:
        frequencies.append(frequency)
        powers.append(spectral_power(values, fs, frequency))
        frequency += step
    peak = max(range(len(powers)), key=powers.__getitem__)
    fitted = frequencies[peak]
    # Quadratic interpolation of log power removes most grid quantization.
    if 0 < peak < len(powers) - 1 and min(powers[peak-1:peak+2]) > 0.0:
        left, mid, right = map(math.log, powers[peak-1:peak+2])
        denom = left - 2.0 * mid + right
        if denom:
            fitted += 0.5 * (left - right) / denom * step
    floor = statistics.median(powers) if powers else 0.0
    prominence = powers[peak] / max(floor, 1e-30)
    return fitted, prominence


def solve_linear(matrix, vector):
    """Small dense Gaussian solve with pivoting and a tiny ridge."""
    n = len(vector)
    augmented = [list(matrix[i]) + [vector[i]] for i in range(n)]
    scale = max((abs(matrix[i][i]) for i in range(n)), default=1.0)
    ridge = max(scale, 1.0) * 1e-12
    for i in range(n):
        augmented[i][i] += ridge
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        value = augmented[col][col]
        if abs(value) < 1e-30:
            continue
        for item in range(col, n + 1):
            augmented[col][item] /= value
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            for item in range(col, n + 1):
                augmented[row][item] -= factor * augmented[col][item]
    return [augmented[i][n] for i in range(n)]


def joint_modal_fit(values, fs, frequencies):
    """Fit all excited frequencies jointly; return coverage and residual."""
    signal = detrend(values)
    count = 2 * len(frequencies)
    if not count:
        return 0.0, signal
    gram = [[0.0] * count for _ in range(count)]
    rhs = [0.0] * count
    basis_rows = []
    for index, value in enumerate(signal):
        t = index / fs
        basis = []
        for frequency in frequencies:
            phase = 2.0 * math.pi * frequency * t
            basis.extend((math.cos(phase), math.sin(phase)))
        basis_rows.append(basis)
        for i in range(count):
            rhs[i] += basis[i] * value
            for j in range(i, count):
                gram[i][j] += basis[i] * basis[j]
    for i in range(count):
        for j in range(i):
            gram[i][j] = gram[j][i]
    coefficients = solve_linear(gram, rhs)
    residual = []
    total2 = error2 = 0.0
    for value, basis in zip(signal, basis_rows):
        predicted = sum(c * b for c, b in zip(coefficients, basis))
        residual.append(value - predicted)
        total2 += value * value
        error2 += (value - predicted) ** 2
    coverage = 1.0 - error2 / total2 if total2 > 0.0 else 0.0
    return coverage, residual


def strongest_residual(residual, fs, configured, max_hz, exclusion_hz):
    # Downsample before the broad residual scan; this keeps pure-Python runtime
    # practical without affecting the sub-200-Hz modal band.
    block = max(1, int(fs / max(2.5 * max_hz, 1.0)))
    reduced = [sum(residual[i:i+block]) / len(residual[i:i+block])
               for i in range(0, len(residual), block)]
    reduced_fs = fs / block
    duration = len(reduced) / reduced_fs
    step = max(0.10, 1.0 / max(4.0 * duration, 1.0))
    candidates = []
    frequency = step
    while frequency <= min(max_hz, 0.45 * reduced_fs):
        if all(abs(frequency - known) > exclusion_hz for known in configured):
            candidates.append((spectral_power(reduced, reduced_fs, frequency), frequency))
        frequency += step
    if not candidates:
        return float("nan"), 0.0
    power, frequency = max(candidates)
    floor = statistics.median(item[0] for item in candidates)
    return frequency, power / max(floor, 1e-30)


def clustered_residuals(peaks, tolerance_hz):
    """Cluster one residual peak per event without counting an event twice."""
    clusters = []
    for event, frequency, prominence in sorted(peaks, key=lambda item: item[1]):
        eligible = [cluster for cluster in clusters
                    if abs(frequency - statistics.median(
                        item[1] for item in cluster)) <= tolerance_hz
                    and all(item[0] != event for item in cluster)]
        if eligible:
            cluster = min(eligible, key=lambda items: abs(
                frequency - statistics.median(item[1] for item in items)))
            cluster.append((event, frequency, prominence))
        else:
            clusters.append([(event, frequency, prominence)])
    return clusters


def frequency_test(rows, error_path, command_path, config_path,
                   window_s, search_hz, tolerance_hz, min_events,
                   min_prominence, residual_prominence, max_frequency):
    """Validate configured modes on command-subtracted push ring-downs."""
    if "start_s" not in rows[0]:
        raise ValueError("frequency testing requires the start_s CSV column")
    params = fsp_params(config_path)
    fs = float(params.get("fs", 0.0))
    if not math.isfinite(fs) or fs <= 0.0:
        raise ValueError(f"{config_path}: FSP fs must be positive")
    errors = read_aylp_vectors(error_path)
    commands = read_aylp_vectors(command_path)
    sample_count = min(*(len(axis) for axis in errors),
                       *(len(axis) for axis in commands))
    if max(*(len(axis) for axis in errors), *(len(axis) for axis in commands)) \
            != sample_count:
        print(f"warning: stream lengths differ; using first {sample_count} frames")

    axis_data = {}
    for axis_index, axis_name in enumerate(("y", "x")):
        delay, frac, gain, freqs, plant_b, plant_a = axis_plant(
            params, axis_name)
        if not freqs:
            raise ValueError(f"{config_path}: FSP {axis_name}.freqs is empty")
        disturbance = reconstruct_disturbance(
            errors[axis_index][:sample_count],
            commands[axis_index][:sample_count], delay, frac, gain,
            plant_b, plant_a)
        axis_data[axis_name] = (disturbance, freqs)

    print("\nModal-frequency validation on reconstructed disturbance")
    print(f"error={error_path} command={command_path} fs={fs:g} Hz "
          f"window={window_s:g} s")
    measurements = collections.defaultdict(list)
    residual_peaks = collections.defaultdict(list)
    coverages = collections.defaultdict(list)
    usable_events = collections.Counter()
    window_frames = max(16, round(window_s * fs))

    for row_index, row in enumerate(rows):
        axis_name = row["axis"]
        if axis_name not in axis_data:
            continue
        start = round(float(row["start_s"]) * fs)
        stop = min(start + window_frames, sample_count)
        if start < 0 or stop - start < max(16, window_frames // 2):
            print(f"  {axis_name} event {row.get('event', row_index + 1)}: "
                  "SKIP (insufficient recorded samples)")
            continue
        disturbance, configured = axis_data[axis_name]
        signal = detrend(disturbance[start:stop])
        event_id = f"{axis_name}:{row.get('event', row_index + 1)}"
        usable_events[axis_name] += 1
        fitted = []
        mode_text = []
        for mode_index, nominal in enumerate(configured):
            frequency, prominence = local_frequency(
                signal, fs, nominal, search_hz)
            measurements[(axis_name, mode_index)].append(
                (event_id, frequency, prominence))
            if prominence >= min_prominence:
                matches = abs(frequency - nominal) <= tolerance_hz
                if matches:
                    fitted.append(frequency)
                mode_text.append(
                    f"{frequency:.2f}Hz({prominence:.1f}x"
                    f"{'' if matches else ', shifted'})")
        coverage, residual = joint_modal_fit(signal, fs, fitted)
        coverages[axis_name].append(coverage)
        residual_frequency, residual_ratio = strongest_residual(
            residual, fs, fitted, max_frequency, tolerance_hz)
        if math.isfinite(residual_frequency) \
                and residual_ratio >= residual_prominence:
            residual_peaks[axis_name].append(
                (event_id, residual_frequency, residual_ratio))
        modes = ", ".join(mode_text) if mode_text else "none excited"
        residual_note = (f"{residual_frequency:.2f}Hz/{residual_ratio:.1f}x"
                         if math.isfinite(residual_frequency) else "none")
        print(f"  {event_id}: modes {modes}; fit {100.0 * coverage:.1f}%; "
              f"residual {residual_note}")

    failed = False
    incomplete = False
    print("\nConfigured-mode stability (frequency is amplitude-independent)")
    print("axis  nominal Hz  excited n  median Hz   MAD Hz  result")
    for axis_name in ("y", "x"):
        configured = axis_data[axis_name][1]
        for mode_index, nominal in enumerate(configured):
            excited = [(frequency, prominence)
                       for _, frequency, prominence
                       in measurements[(axis_name, mode_index)]
                       if prominence >= min_prominence]
            frequencies = [item[0] for item in excited]
            if len(frequencies) < min_events:
                incomplete = True
                print(f"{axis_name:>4} {nominal:11.3f} {len(frequencies):10d}"
                      f" {'-':>10} {'-':>8}  UNTESTED")
                continue
            centre = statistics.median(frequencies)
            mad = statistics.median(abs(value - centre)
                                    for value in frequencies)
            good = abs(centre - nominal) <= tolerance_hz \
                and mad <= tolerance_hz
            failed |= not good
            print(f"{axis_name:>4} {nominal:11.3f} {len(frequencies):10d} "
                  f"{centre:10.3f} {mad:8.3f}  "
                  f"{'PASS' if good else 'FAIL'}")

    print("\nModel coverage and repeatable unmodeled lines")
    for axis_name in ("y", "x"):
        coverage = coverages[axis_name]
        if coverage:
            print(f"  {axis_name}: median configured-mode coverage "
                  f"{100.0 * statistics.median(coverage):.1f}% over "
                  f"{usable_events[axis_name]} usable events")
        repeatable = []
        for cluster in clustered_residuals(
                residual_peaks[axis_name], tolerance_hz):
            if len(cluster) >= min_events:
                centre = statistics.median(item[1] for item in cluster)
                prominence = statistics.median(item[2] for item in cluster)
                repeatable.append((centre, len(cluster), prominence))
        if repeatable:
            failed = True
            for frequency, count, prominence in repeatable:
                print(f"  {axis_name}: FAIL repeatable residual "
                      f"{frequency:.2f} Hz in {count} events "
                      f"(median prominence {prominence:.1f}x); consider "
                      "adding/replacing a configured mode")
        else:
            print(f"  {axis_name}: no repeatable strong residual line")

    result = "FAIL" if failed else "INCOMPLETE" if incomplete else "PASS"
    print(f"\nFrequency-test result: {result}")
    return result


def load(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    # Logs written before selectable integral recovery did not have this
    # column; treating it as zero keeps historical P/modal comparisons valid.
    for row in rows:
        row.setdefault("peak_abs_integral", "0")
        row.setdefault("shadow_promoted", "0")
        row.setdefault("shadow_error_ratio", "nan")
    required = {"axis", "recovery", *(name for name, _, _ in FIELDS)}
    if not rows:
        raise ValueError(f"{path}: no completed push events")
    missing = required - rows[0].keys()
    if missing:
        raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
    return rows


def median(rows, field, scale):
    return statistics.median(float(row[field]) * scale for row in rows)


def summarize(path: Path, rows, pixel_scale):
    print(f"\n{path} ({len(rows)} completed axis-events)")
    promoted = [row for row in rows if int(row["shadow_promoted"])]
    if promoted:
        ratios = [float(row["shadow_error_ratio"]) for row in promoted]
        print(f"shadow model promoted in {len(promoted)} event(s); median "
              f"held-out RMS ratio {statistics.median(ratios):.3f}")
    else:
        print("shadow model promotions: 0")
    print("axis      recovery  n  " + "  ".join(f"{label:>12}" for _, label, _ in FIELDS))
    recoveries = sorted({row["recovery"] for row in rows})
    for axis in ("y", "x", "all"):
        axis_rows = rows if axis == "all" else [row for row in rows if row["axis"] == axis]
        for recovery in (*recoveries, "all"):
            selected = axis_rows if recovery == "all" else [
                row for row in axis_rows if row["recovery"] == recovery
            ]
            if not selected:
                continue
            values = []
            for field, _, fixed_scale in FIELDS:
                scale = pixel_scale if fixed_scale is None else fixed_scale
                values.append(median(selected, field, scale))
            print(
                f"{axis:>4} {recovery:>13} {len(selected):2d}  "
                + "  ".join(f"{value:12.4g}" for value in values)
            )


def compare_recoveries(rows, pixel_scale):
    print("\nMedian ratio modal / proportional (below 1 is better)")
    print("axis  " + "  ".join(f"{label:>12}" for _, label, _ in FIELDS))
    for axis in ("y", "x", "all"):
        selected = rows if axis == "all" else [row for row in rows if row["axis"] == axis]
        p = [row for row in selected if row["recovery"] == "proportional"]
        m = [row for row in selected if row["recovery"] == "modal"]
        if not p or not m:
            continue
        ratios = []
        for field, _, fixed_scale in FIELDS:
            scale = pixel_scale if fixed_scale is None else fixed_scale
            pv = median(p, field, scale)
            mv = median(m, field, scale)
            ratios.append(mv / pv if pv else float("nan"))
        print(f"{axis:>4}  " + "  ".join(f"{value:12.3f}" for value in ratios))


def compare(a_path, a_rows, b_path, b_rows, pixel_scale):
    print(f"\nMedian ratio {b_path.name} / {a_path.name} (below 1 is better)")
    print("axis  " + "  ".join(f"{label:>12}" for _, label, _ in FIELDS))
    for axis in ("y", "x", "all"):
        a = a_rows if axis == "all" else [row for row in a_rows if row["axis"] == axis]
        b = b_rows if axis == "all" else [row for row in b_rows if row["axis"] == axis]
        if not a or not b:
            continue
        ratios = []
        for field, _, fixed_scale in FIELDS:
            scale = pixel_scale if fixed_scale is None else fixed_scale
            av = median(a, field, scale)
            bv = median(b, field, scale)
            ratios.append(bv / av if av else float("nan"))
        print(f"{axis:>4}  " + "  ".join(f"{value:12.3f}" for value in ratios))


def main():
    parser = argparse.ArgumentParser(
        description="Summarize one push-event log or compare two matched trials."
    )
    parser.add_argument("csv", nargs="+", type=Path, help="one or two FSP transient CSV files")
    parser.add_argument("--pixel-scale", type=float, default=15.5)
    parser.add_argument("--frequency-test", action="store_true",
                        help="validate configured modes on push ring-downs")
    parser.add_argument("--error-aylp", type=Path,
                        help="full-rate pre-FSP [y,x] error stream")
    parser.add_argument("--command-aylp", type=Path,
                        help="full-rate applied [y,x] command stream")
    parser.add_argument("--config", type=Path,
                        help="JSON config used for the push run")
    parser.add_argument("--window-s", type=float, default=2.0,
                        help="ring-down window beginning at event detection")
    parser.add_argument("--search-hz", type=float, default=1.5,
                        help="search half-width around each configured mode")
    parser.add_argument("--tolerance-hz", type=float, default=0.5,
                        help="allowed median frequency error and event MAD")
    parser.add_argument("--min-events", type=int, default=3,
                        help="required excited pushes per mode")
    parser.add_argument("--min-prominence", type=float, default=3.0,
                        help="minimum local line/background power ratio")
    parser.add_argument("--residual-prominence", type=float, default=5.0,
                        help="minimum residual line/background power ratio")
    parser.add_argument("--max-frequency", type=float, default=200.0,
                        help="highest frequency searched for omitted modes")
    parser.add_argument("--strict", action="store_true",
                        help="exit nonzero unless frequency test passes")
    args = parser.parse_args()
    if len(args.csv) > 2:
        parser.error("provide one log to summarize or two logs to compare")
    if args.frequency_test:
        missing = [name for name, value in (
            ("--error-aylp", args.error_aylp),
            ("--command-aylp", args.command_aylp),
            ("--config", args.config)) if value is None]
        if missing:
            parser.error("--frequency-test requires " + ", ".join(missing))
        if len(args.csv) != 1:
            parser.error("--frequency-test accepts exactly one event CSV")
        if args.window_s <= 0.0 or args.search_hz <= 0.0 \
                or args.tolerance_hz <= 0.0 or args.min_events <= 0 \
                or args.min_prominence <= 0.0 \
                or args.residual_prominence <= 0.0 \
                or args.max_frequency <= 0.0:
            parser.error("frequency-test numeric options must be positive")

    loaded = [(path, load(path)) for path in args.csv]
    for path, rows in loaded:
        summarize(path, rows, args.pixel_scale)
        compare_recoveries(rows, args.pixel_scale)
    if len(loaded) == 2:
        compare(*loaded[0], *loaded[1], args.pixel_scale)
    if args.frequency_test:
        result = frequency_test(
            loaded[0][1], args.error_aylp, args.command_aylp, args.config,
            args.window_s, args.search_hz, args.tolerance_hz,
            args.min_events, args.min_prominence,
            args.residual_prominence, args.max_frequency)
        if args.strict and result != "PASS":
            raise SystemExit(2)


if __name__ == "__main__":
    main()
