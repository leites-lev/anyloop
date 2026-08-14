#!/usr/bin/env python3
import importlib.util
import math
import struct
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "analyze_steering_ab", ROOT / "contrib/data-analysis-tools/analyze_steering_ab.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

header = bytearray(40)
struct.pack_into("<I", header, 0, module.MAGIC)
header[6] = module.VECTOR
struct.pack_into("<QQ", header, 8, 2, 1)

with tempfile.NamedTemporaryFile() as stream:
    # Two seconds open at 4 Hz, then two seconds closed. Alternating values
    # make the means zero and the exact RMS ratio two-to-one.
    for amplitude in (2.0, 1.0):
        for frame in range(8):
            sign = 1.0 if frame % 2 else -1.0
            stream.write(header)
            stream.write(struct.pack("<dd", sign * amplitude, 0.0))
    stream.flush()
    result = module.analyze(Path(stream.name), 4.0, 1.0, 0.0, 2.0, 2.0, 2.0)

assert math.isclose(result["open"]["jitter_y_rms_px"], 2.0)
assert math.isclose(result["closed"]["jitter_y_rms_px"], 1.0)
assert math.isclose(
    result["improvement_percent"]["combined_rms_px"], 50.0)
assert result["open"]["rejected_frames"] == 0
