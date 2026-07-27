#!/usr/bin/env python3
"""Find asi_source start_x/start_y that centre the beam in each config's ROI.

Captures a full-sensor probe frame (beam parked at bias), locates the beam
centre of mass, then for every contrib/*.json that has an asi_source device
prints the start_x/start_y that put the beam at frame centre for that config's
width/height. See doc/devices/fsp.md and the roi-crop-follows-beam memory.

Usage:
    contrib/find_roi.py                 # wait 30 s to settle, capture, compute all
    contrib/find_roi.py --settle 0      # capture immediately (beam already settled)
    contrib/find_roi.py --no-capture    # reuse an existing probe_full.aylp
    contrib/find_roi.py --exposure 300  # override probe exposure (µs)
    contrib/find_roi.py --write         # ALSO edit start_x/start_y into the configs

Notes / gotchas (why the arithmetic is what it is):
  * Frame centre is (dim-1)/2, NOT dim/2: center_of_mass normalises px to
    -1..+1 as -1 + 2*px/(dim-1), so norm 0 = px 15.5 on a 32-px frame.
  * ASI ROI grid: start_x must be a multiple of 8, start_y a multiple of 2.
  * The operating exposure saturates the beam and shifts its CoM a few px vs a
    dim exposure -- probe near the loop's exposure so you centre where the loop
    actually sees the beam. Run only after the coarse channels settle (~30 s).
"""
import argparse, glob, json, os, re, struct, subprocess, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBE_CFG = os.path.join(ROOT, "contrib", "probe_frame.json")
ANYLOOP = os.path.join(ROOT, "build", "anyloop")


def capture(exposure):
    """Run probe_frame.json (optionally with an exposure override) -> aylp path."""
    cfg = json.load(open(PROBE_CFG))
    params = cfg["pipeline"][0]["params"]
    if exposure is not None:
        params["exposure"] = exposure
    # find where the probe writes its frames
    out = "probe_full.aylp"
    for st in cfg["pipeline"]:
        if "file_sink" in st.get("uri", ""):
            out = st["params"]["filename"]
    out = out if os.path.isabs(out) else os.path.join(ROOT, out)
    tmp = os.path.join(ROOT, ".find_roi_probe.json")
    json.dump(cfg, open(tmp, "w"))
    try:
        print(f"# capturing (exp={params['exposure']}us) -- give the beam ~30 s to park...",
              file=sys.stderr)
        subprocess.run([ANYLOOP, tmp], cwd=ROOT, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        os.remove(tmp)
    return out


def read_frames(path):
    """Parse an .aylp file: 40-byte header (magic u32, 4x u8, log_dim y/x u64 at
    offset 8), then y*x uint8 pixels, repeated per frame."""
    d = open(path, "rb").read()
    off, fr = 0, []
    while off + 40 <= len(d):
        y, x = struct.unpack_from("<QQ", d, off + 8)
        o, n = off + 40, y * x
        if o + n > len(d):
            break
        fr.append(np.frombuffer(d, np.uint8, n, o).reshape(y, x))
        off = o + n
    if not fr:
        sys.exit(f"no frames parsed from {path}")
    return np.mean(fr, axis=0)


def beam_com(img):
    """Background-subtracted 50%-of-peak centroid, with a sanity check."""
    sub = img - np.median(img)
    sub[sub < 0] = 0
    ys, xs = np.nonzero(sub >= sub.max() * 0.5)
    w = sub[ys, xs]
    t = w.sum()
    cx, cy = (xs * w).sum() / t, (ys * w).sum() / t
    H, W = img.shape
    near_centre = abs(cx - (W - 1) / 2) < 20 and abs(cy - (H - 1) / 2) < 20
    if img.max() < 60:
        print(f"# WARNING: dim frame (max px {img.max():.0f}) -- beam may be missing",
              file=sys.stderr)
    if near_centre and img.max() < 80:
        print("# WARNING: CoM near sensor centre on a dim frame -- likely background,"
              " not the beam", file=sys.stderr)
    return cx, cy, img.max()


def align(v, a):
    """Nearest multiple of a."""
    return int(round(v / a)) * a


def roi_start(bx, by, w, h, sw, sh):
    sx = align(bx - (w - 1) / 2, 8)          # start_x on 8-px grid
    sy = align(by - (h - 1) / 2, 2)          # start_y on 2-px grid
    sx = max(0, min(sx, sw - w))
    sy = max(0, min(sy, sh - h))
    return sx, sy


def asi_params(cfg):
    """Return the asi_source params dict of a config, or None."""
    for st in cfg.get("pipeline", []):
        if "asi_source" in st.get("uri", ""):
            return st["params"]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-capture", action="store_true",
                    help="reuse existing probe_full.aylp instead of grabbing a new one")
    ap.add_argument("--exposure", type=int, default=None,
                    help="probe exposure in µs (default: whatever probe_frame.json has)")
    ap.add_argument("--settle", type=float, default=30.0,
                    help="seconds to wait for the FSM to park before capturing (default 30; "
                         "the beam drifts for ~30 s after the hold starts). Use 0 to skip.")
    ap.add_argument("--aylp", default=None, help="path to an existing .aylp probe file")
    ap.add_argument("--write", action="store_true",
                    help="edit the computed start_x/start_y back into each config")
    args = ap.parse_args()

    if args.aylp:
        path = args.aylp
    elif args.no_capture:
        path = os.path.join(ROOT, "probe_full.aylp")
    else:
        if args.settle > 0:
            print(f"# settling: waiting {args.settle:.0f} s for the beam to park "
                  "(make sure the FSM is held at bias)...", file=sys.stderr)
            time.sleep(args.settle)
        path = capture(args.exposure)

    img = read_frames(path)
    bx, by, mx = beam_com(img)
    H, W = img.shape
    print(f"# beam CoM: x={bx:.1f}  y={by:.1f}   (sensor {W}x{H}, peak px {mx:.0f})")
    print(f"# {'config':28s} {'WxH':>9s}  start_x  start_y   beam-in-ROI")

    for f in sorted(glob.glob(os.path.join(ROOT, "contrib", "*.json"))):
        try:
            cfg = json.load(open(f))
        except (ValueError, OSError):
            continue
        p = asi_params(cfg)
        if not p or "width" not in p or "height" not in p:
            continue
        w, h = p["width"], p["height"]
        if w >= W and h >= H:        # the full-sensor probe config itself
            continue
        sx, sy = roi_start(bx, by, w, h, W, H)
        print(f"  {os.path.basename(f):28s} {w:4d}x{h:<4d}  {sx:7d}  {sy:7d}"
              f"   ({bx - sx:.1f}, {by - sy:.1f})")

        if args.write:
            s = open(f).read()
            had = re.search(r'"start_x"', s)
            s2 = re.sub(r'("start_x"\s*:\s*)-?\d+', rf'\g<1>{sx}', s)
            s2 = re.sub(r'("start_y"\s*:\s*)-?\d+', rf'\g<1>{sy}', s2)
            if not had:
                # insert after the height line, matching its indentation
                m = re.search(r'([ \t]*)"height"\s*:\s*\d+,', s2)
                if m:
                    ind = m.group(1)
                    ins = f'\n{ind}"start_x": {sx},\n{ind}"start_y": {sy},'
                    s2 = s2[:m.end()] + ins + s2[m.end():]
            if s2 != s:
                open(f, "w").write(s2)

    if args.write:
        print("# --write: start_x/start_y updated in the configs above")
    else:
        print("# (print-only; re-run with --write to edit the configs)")


if __name__ == "__main__":
    main()
