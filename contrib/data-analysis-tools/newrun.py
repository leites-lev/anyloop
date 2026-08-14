#!/usr/bin/env python3
"""Launch an anyloop config into a fresh, auto-numbered run directory.

anyloop writes every output to the literal path in the config, so re-running a
config silently overwrites the previous run. That is not hypothetical: on
2026-07-31 a ~29 s re-run destroyed PAR-5's 360 MB error record, its command
record, its learned weights and its convergence trace, because `fsp`'s fini
rewrites the weights/trace on ANY exit including an early Ctrl-C. Only the
.dat/.pdf, written at normal completion, survived.

This wrapper takes the config, allocates the next run number, and rewrites
every OUTPUT path to sit inside runs/<config-stem>/<NNN>/. Nothing outside that
directory is touched, so a re-run can never clobber an earlier one.

  python3 contrib/data-analysis-tools/newrun.py contrib/steering/configurations/support/attenuation_par_fsp.json -n "PAR-6: mu 0.3"
  python3 contrib/data-analysis-tools/newrun.py contrib/steering/configurations/steering_par_fsp.json --dry-run

The resolved config, a metadata file (timestamp, git commit, dirty state, the
exact command) and the full log are written alongside the data, so a run
directory is self-describing after the fact.
"""
import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
RUNS = os.path.join(REPO, "runs")

# Keys whose values are paths anyloop WRITES. wiener_file is deliberately
# absent: it is an INPUT (an offline Wiener solution to load) and must keep
# pointing wherever the caller aimed it.
OUT_KEYS = {"filename", "output_file", "wiener_out", "wiener_trace"}


def walk(obj, fn):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k in OUT_KEYS and isinstance(v, str):
                obj[k] = fn(v)
            else:
                walk(v, fn)
    elif isinstance(obj, list):
        for v in obj:
            walk(v, fn)


def next_index(stem):
    """One past the highest run seen, counting both run dirs and the legacy
    numbered files still sitting in the repo root, so an existing series (e.g.
    attenuation_par_fsp1..5.dat) continues rather than restarting at 1."""
    seen = {0}
    d = os.path.join(RUNS, stem)
    if os.path.isdir(d):
        for e in os.listdir(d):
            if e.isdigit():
                seen.add(int(e))
    for e in os.listdir(REPO):
        m = re.fullmatch(re.escape(stem) + r"(\d+)\.(dat|pdf)", e)
        if m:
            seen.add(int(m.group(1)))
    return max(seen) + 1


def git(*a):
    try:
        return subprocess.run(["git", "-C", REPO] + list(a),
                              capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("-n", "--note", default="", help="why this run exists")
    ap.add_argument("--cpu", default="2", help="taskset core (default 2)")
    ap.add_argument("--prio", default="80", help="chrt FIFO priority")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("extra", nargs="*", help="extra args passed to anyloop")
    args = ap.parse_args()

    cfg_path = os.path.abspath(args.config)
    stem = os.path.splitext(os.path.basename(cfg_path))[0]
    idx = next_index(stem)
    run_dir = os.path.join(RUNS, stem, "%03d" % idx)

    with open(cfg_path) as f:
        cfg = json.load(f)

    moved = []

    def relocate(p):
        base = os.path.basename(p)
        root, ext = os.path.splitext(base)
        # the directory carries the run number now, so drop it from the name:
        # atten_par_err6.aylp -> atten_par_err.aylp
        root = re.sub(r"\d+$", "", root)
        new = os.path.join(run_dir, root + ext)
        moved.append((p, os.path.relpath(new, REPO)))
        return new

    walk(cfg, relocate)

    print("run directory : %s" % os.path.relpath(run_dir, REPO))
    print("config        : %s" % os.path.relpath(cfg_path, REPO))
    for old, new in moved:
        print("  %-28s -> %s" % (old, new))
    if not moved:
        print("  (no output paths found -- nothing to relocate)")

    if args.dry_run:
        print("\n--dry-run: nothing created, nothing launched")
        return 0

    os.makedirs(run_dir, exist_ok=False)
    resolved = os.path.join(run_dir, "config.json")
    with open(resolved, "w") as f:
        json.dump(cfg, f, indent=1)
    shutil.copy2(cfg_path, os.path.join(run_dir, "config.original.json"))

    cmd = (["sudo", "chrt", "-f", args.prio, "taskset", "-c", args.cpu,
            os.path.join(REPO, "build", "anyloop"), "-p", resolved]
           + args.extra)

    with open(os.path.join(run_dir, "meta.txt"), "w") as f:
        f.write("run        : %s %03d\n" % (stem, idx))
        f.write("started    : %s\n" % datetime.datetime.now().isoformat(" ", "seconds"))
        f.write("note       : %s\n" % (args.note or "(none)"))
        f.write("config     : %s\n" % os.path.relpath(cfg_path, REPO))
        f.write("git commit : %s\n" % git("rev-parse", "HEAD"))
        f.write("git dirty  : %s\n" % ("yes" if git("status", "--porcelain") else "no"))
        f.write("command    : %s\n" % " ".join(cmd))

    # symlink for analysis scripts that want "whatever ran last"
    link = os.path.join(RUNS, stem, "latest")
    try:
        if os.path.islink(link) or os.path.exists(link):
            os.remove(link)
        os.symlink("%03d" % idx, link)
    except OSError:
        pass

    log = os.path.join(run_dir, "run.log")
    print("\nlaunching: %s\n" % " ".join(cmd))
    with open(log, "w") as lf:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True,
                             bufsize=1)
        try:
            for line in p.stdout:
                sys.stdout.write(line)
                lf.write(line)
        except KeyboardInterrupt:
            # Ctrl-C reaches anyloop too; let it run fini and drain its output
            # so the weights/trace it writes on the way out are logged.
            for line in p.stdout:
                sys.stdout.write(line)
                lf.write(line)
        rc = p.wait()

    with open(os.path.join(run_dir, "meta.txt"), "a") as f:
        f.write("finished   : %s\n" % datetime.datetime.now().isoformat(" ", "seconds"))
        f.write("exit code  : %d\n" % rc)
    print("\noutputs in %s" % os.path.relpath(run_dir, REPO))
    return rc


if __name__ == "__main__":
    sys.exit(main())
