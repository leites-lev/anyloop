#!/usr/bin/env python3
"""Summarize matched open/closed windows in a two-vector AYLP recording."""

import argparse
import json
import math
import mmap
import statistics
import struct
from pathlib import Path

HEADER = 40
MAGIC = 0x504C5941
VECTOR = 1 << 2


def read_window(path, first, last, pixel_scale):
    values = []
    rejected = 0
    with path.open("rb") as stream:
        header = stream.read(HEADER)
        if len(header) != HEADER or struct.unpack_from("<I", header)[0] != MAGIC:
            raise ValueError(f"{path}: invalid AYLP header")
        ny, nx = struct.unpack_from("<QQ", header, 8)
        count = ny * nx
        if header[6] != VECTOR or count < 2:
            raise ValueError(f"{path}: expected a two-element vector")
        record = HEADER + 8 * count
        size = path.stat().st_size
        if size % record:
            raise ValueError(f"{path}: truncated or variable-size stream")
        frames = size // record
        if first < 0 or last > frames or last <= first:
            raise ValueError(f"{path}: requested window is outside recording")
        mapped = mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ)
        for frame in range(first, last):
            offset = frame * record
            rejected += bool(mapped[offset + 5])
            y, x = struct.unpack_from("<dd", mapped, offset + HEADER)
            values.append((pixel_scale * y, pixel_scale * x))
        mapped.close()
    ys = [value[0] for value in values]
    xs = [value[1] for value in values]
    mean_y, mean_x = statistics.fmean(ys), statistics.fmean(xs)
    rms_y = math.sqrt(statistics.fmean((value - mean_y) ** 2 for value in ys))
    rms_x = math.sqrt(statistics.fmean((value - mean_x) ** 2 for value in xs))
    radii = sorted(math.hypot(y, x) for y, x in values)
    return {
        "frames": len(values), "rejected_frames": rejected,
        "mean_y_px": mean_y, "mean_x_px": mean_x,
        "mean_radial_offset_px": math.hypot(mean_y, mean_x),
        "jitter_y_rms_px": rms_y, "jitter_x_rms_px": rms_x,
        "combined_rms_px": math.sqrt(statistics.fmean(
            y * y + x * x for y, x in values)),
        "median_radial_error_px": radii[len(radii) // 2],
    }


def improvement(open_value, closed_value):
    if open_value == 0.0:
        return None
    return 100.0 * (open_value - closed_value) / open_value


def analyze(path, fs, pixel_scale, open_start, open_seconds,
            closed_start, closed_seconds):
    opened = read_window(path, round(open_start * fs),
                         round((open_start + open_seconds) * fs), pixel_scale)
    closed = read_window(path, round(closed_start * fs),
                         round((closed_start + closed_seconds) * fs), pixel_scale)
    keys = ("jitter_y_rms_px", "jitter_x_rms_px", "combined_rms_px",
            "median_radial_error_px", "mean_radial_offset_px")
    return {"open": opened, "closed": closed,
            "improvement_percent": {
                key: improvement(opened[key], closed[key]) for key in keys}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument("--fs", type=float, required=True)
    parser.add_argument("--pixel-scale", type=float, required=True)
    parser.add_argument("--open-start", type=float, required=True)
    parser.add_argument("--open-seconds", type=float, default=60.0)
    parser.add_argument("--closed-start", type=float, required=True)
    parser.add_argument("--closed-seconds", type=float, default=60.0)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    result = analyze(args.recording, args.fs, args.pixel_scale,
                     args.open_start, args.open_seconds,
                     args.closed_start, args.closed_seconds)
    output = json.dumps(result, indent=2) + "\n"
    if args.json:
        args.json.write_text(output, encoding="utf-8")
    print(output, end="")


if __name__ == "__main__":
    main()
