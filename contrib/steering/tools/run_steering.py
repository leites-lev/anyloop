#!/usr/bin/env python3
"""Park, recenter the camera ROI, then start a pulsed steering run."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
PROFILES = ROOT / "contrib" / "steering" / "configurations"
ANYLOOP = ROOT / "build" / "anyloop"
PARK = ROOT / "contrib" / "calibration-scripts" / "configurations" / "conf_parport_dac_park.json"
FIND_ROI = ROOT / "contrib" / "calibration-scripts" / "tools" / "find_roi.py"
RUNS = {
	"live": PROFILES / "steering_par_fsp.json",
	"100hz_80pct": PROFILES / "run_100hz_80pct.json",
    "1000hz_20pct": PROFILES / "run_1000hz_20pct.json",
    "1hz_20pct": PROFILES / "run_1hz_20pct.json",
}


def main():
    parser = argparse.ArgumentParser(
        description="Park the FSM, recenter the ROI, then start steering")
    parser.add_argument("waveform", choices=RUNS)
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds to settle at 0 V before ROI capture")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    run_config = RUNS[args.waveform]
    for path in (ANYLOOP, PARK, FIND_ROI, run_config):
        if not path.is_file():
            raise SystemExit(f"required file not found: {path}")
    if not args.dry_run and os.geteuid() != 0:
        raise SystemExit(
            "The MMIO DAC backend requires root. Re-run as:\n\n  "
            f"sudo chrt -f 80 taskset -c 2 {sys.executable} "
            f"{Path(__file__).resolve()} {args.waveform}")

    park = [str(ANYLOOP), "-p", str(PARK)]
    recenter = [
        sys.executable, str(FIND_ROI), "--settle", str(args.settle),
        "--ref-config", str(run_config), "--write",
        "--config", str(run_config),
    ]
    run = [str(ANYLOOP), "-p", str(run_config)]
    for label, command in (("park FSM", park),
                           ("recenter ROI", recenter),
                           ("start steering", run)):
        print(f"[{label}] {' '.join(command)}", flush=True)
        if not args.dry_run:
            subprocess.run(command, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
