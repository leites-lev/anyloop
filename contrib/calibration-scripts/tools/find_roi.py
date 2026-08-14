#!/usr/bin/env python3
"""Find asi_source start_x/start_y that centre the beam in each config's ROI.

Captures a full-sensor probe frame (beam parked at bias), locates the beam
centre of mass, then for every contrib/*.json that has an asi_source device
prints the start_x/start_y that put the beam at frame centre for that config's
width/height. See doc/devices/fsp.md and the roi-crop-follows-beam memory.

Usage:
    contrib/calibration-scripts/tools/find_roi.py                 # wait 30 s to settle, capture, compute all
    contrib/calibration-scripts/tools/find_roi.py --settle 0      # capture immediately (beam already settled)
    contrib/calibration-scripts/tools/find_roi.py --no-capture    # reuse the staged probe capture
    contrib/calibration-scripts/tools/find_roi.py --exposure 300  # override probe exposure (µs)
    contrib/calibration-scripts/tools/find_roi.py --camera ASI290MM   # probe a different camera
    contrib/calibration-scripts/tools/find_roi.py --write         # ALSO edit start_x/start_y into the configs

Notes / gotchas (why the arithmetic is what it is):
  * WHICH CAMERA THE PROBE USES is taken from --ref-config (default
    contrib/steering/configurations/steering_par_fsp.json), not from probe_frame.json, so the ROI is
    always computed for the sensor the loop actually runs on. --camera
    overrides it. The full-sensor width/height are then read back from the
    camera itself (asi_source logs every connected camera's MaxWidth/MaxHeight
    before it matches one), because they are a property of the sensor and
    hardcoding them is how you end up probing an ASI290MM's 1936x1096 on an
    ASI662MM's 1920x1080. probe_frame.json's own width/height are only a
    fallback for when enumeration fails.
  * The ROI grid below was measured on the ASI290MM. It is a property of the
    ASI SDK's ROI handling rather than of the sensor, so it is expected to hold
    on the 662 as well -- but that has NOT been re-measured. If exact centring
    matters on a new camera, re-run the ASISetStartPos/ASIGetStartPos check.
  * Frame centre is (dim-1)/2, NOT dim/2: center_of_mass normalises px to
    -1..+1 as -1 + 2*px/(dim-1), so norm 0 = px 15.5 on a 32-px frame.
  * ASI ROI grid: start_x snaps to a multiple of 4, start_y to a multiple of 2.
    MEASURED on the ASI290MM 2026-07-31, not assumed -- ASISetStartPos followed
    by ASIGetStartPos (which exists in libASICamera2.so but is absent from
    contrib/camera_pcie_hardware/ASICamera2.h, so nothing here had ever read the value back):
    1832->1832, 1833/1834/1835->1832, 1836->1836, 1838->1836, 1840->1840, and
    start_y 663->662, 665->664. 1836 is NOT a multiple of 8 and is accepted
    verbatim, so the grid is 4. This file used to align x to 8, which threw
    away half the achievable precision for nothing: worst-case x residual was
    +-4 px and is now +-2. ASISetStartPos returns ASI_SUCCESS even when it
    silently snaps, so a wrong value is invisible without the read-back.
  * THE RESIDUAL CANNOT BE DRIVEN TO ZERO BY CROPPING. start_x is an integer on
    a 4-px grid and the beam CoM is fractional, so the best a crop can do is
    +-2 px in x and +-1 px in y. Exact centring needs the beam moved onto the
    grid (an FSM/DAC bias nudge -- but K is bias-dependent and must be re-fit
    after, see the Kx 3.97-4.36 wander) or a setpoint offset in
    center_of_mass, which has no such parameter today. The offset column below
    tells you how much is left; it is reported relative to frame centre, so a
    perfectly centred beam reads (0.0, 0.0).
  * The operating exposure saturates the beam and shifts its CoM a few px vs a
    dim exposure -- probe near the loop's exposure so you centre where the loop
    actually sees the beam. Run only after the coarse channels settle (~30 s).
"""
import argparse, copy, glob, json, os, re, struct, subprocess, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
PROBE_CFG = os.path.join(
    ROOT, "contrib", "configs", "calibration", "probe_frame.json")
REF_CFG = os.path.join(
    ROOT, "contrib", "configs", "steering", "steering_par_fsp.json")
ANYLOOP = os.path.join(ROOT, "build", "anyloop")

# a camera_name no real camera can match, so asi_source enumerates the lot and
# then bails -- see list_cameras()
ENUM_NAME = "__aylp_enumerate_only__"
CAM_LINE = re.compile(r"\[(\d+)\]\s+(\S.*?)\s+\(ID\s+\d+,\s*(\d+)x(\d+)\)")


def asi_stage(cfg):
    """Return the asi_source pipeline stage of a config, or None."""
    for st in cfg.get("pipeline", []):
        if "asi_source" in st.get("uri", ""):
            return st
    return None


def camera_params_from_config(path):
    """Camera parameters requested by a config, or None if it has no camera."""
    try:
        st = asi_stage(json.load(open(path)))
    except (ValueError, OSError) as e:
        print(f"# WARNING: cannot read {os.path.basename(path)}: {e}",
              file=sys.stderr)
        return None
    return (st or {}).get("params")


def list_cameras():
    """[(name, max_w, max_h)] for every connected ASI camera.

    asi_source logs the full camera list before it tries to match a name, so
    asking for a name nothing can match is a cheap way to read the list -- the
    run fails, which is the point, and no camera is ever opened or streamed.
    """
    try:
        cfg = json.load(open(PROBE_CFG))
    except (ValueError, OSError):
        return []
    st = asi_stage(cfg)
    if st is None:
        return []
    st = copy.deepcopy(st)
    st["params"]["camera_name"] = ENUM_NAME
    tmp = os.path.join(ROOT, ".find_roi_enum.json")
    json.dump({"pipeline": [st]}, open(tmp, "w"))
    try:
        p = subprocess.run([ANYLOOP, tmp], cwd=ROOT, capture_output=True,
                           text=True)
    except OSError as e:
        print(f"# WARNING: cannot run anyloop to enumerate cameras: {e}",
              file=sys.stderr)
        return []
    finally:
        os.remove(tmp)
    return [(m.group(2), int(m.group(3)), int(m.group(4)))
            for m in CAM_LINE.finditer(p.stdout + p.stderr)]


def sensor_size(camera):
    """Full-sensor (w, h) of the first connected camera matching `camera`.

    Matching is the same substring rule asi_source itself uses, so what this
    resolves to is what the probe will actually open. Returns None if nothing
    matches, which the caller treats as "fall back to probe_frame.json".
    """
    cams = list_cameras()
    if not cams:
        return None
    for name, w, h in cams:
        if camera in name:
            print(f"# camera: {name} (sensor {w}x{h})", file=sys.stderr)
            return w, h
    print(f"# WARNING: no connected camera matches \"{camera}\" -- saw "
          + ", ".join(f"{n} ({w}x{h})" for n, w, h in cams), file=sys.stderr)
    return None


def capture(exposure, gain=None, camera=None, sensor=None):
    """Run probe_frame.json (with overrides applied) -> aylp path."""
    os.makedirs(os.path.join(ROOT, "data/calibration/run_staging"), exist_ok=True)
    cfg = json.load(open(PROBE_CFG))
    params = asi_stage(cfg)["params"]
    if exposure is not None:
        params["exposure"] = exposure
    if gain is not None:
        params["gain"] = gain
    if camera is not None:
        params["camera_name"] = camera
    if sensor is not None:
        params["width"], params["height"] = sensor
        # a full-sensor probe must not inherit a crop from the config it was
        # last used with; asi_source auto-centres when these are absent
        params.pop("start_x", None)
        params.pop("start_y", None)
    # find where the probe writes its frames
    out = "data/calibration/run_staging/probe_full.aylp"
    for st in cfg["pipeline"]:
        if "file_sink" in st.get("uri", ""):
            out = st["params"]["filename"]
    out = out if os.path.isabs(out) else os.path.join(ROOT, out)
    tmp = os.path.join(ROOT, ".find_roi_probe.json")
    json.dump(cfg, open(tmp, "w"))
    try:
        print(f"# capturing (exp={params['exposure']}us, gain={params['gain']}) "
              "-- give the beam ~30 s to park...",
              file=sys.stderr)
        probe = subprocess.run([ANYLOOP, tmp], cwd=ROOT, text=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
        if probe.returncode:
            # A failed camera mode used to surface only as CalledProcessError,
            # hiding the ASI SDK's actual reason and making an automatic ROI
            # checkpoint impossible to diagnose.
            if probe.stdout:
                print(probe.stdout, file=sys.stderr, end="")
            probe.check_returncode()
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


def dominant_component(mask):
    """Return y/x indices for the largest 8-connected True component.

    A full sensor contains occasional hot pixels and readout artefacts.  Taking
    the centroid of every half-peak pixel lets those unrelated pixels pull the
    answer away from a dim but spatially coherent spot.
    """
    todo = mask.copy()
    height, width = todo.shape
    largest = []
    for y, x in zip(*np.nonzero(todo)):
        if not todo[y, x]:
            continue
        todo[y, x] = False
        stack = [(int(y), int(x))]
        component = []
        while stack:
            py, px = stack.pop()
            component.append((py, px))
            for ny in range(max(0, py - 1), min(height, py + 2)):
                for nx in range(max(0, px - 1), min(width, px + 2)):
                    if todo[ny, nx]:
                        todo[ny, nx] = False
                        stack.append((ny, nx))
        if len(component) > len(largest):
            largest = component
    if not largest:
        return np.array([], dtype=int), np.array([], dtype=int)
    points = np.asarray(largest)
    return points[:, 0], points[:, 1]


def beam_com(img):
    """Locate the dominant spatially coherent half-peak spot."""
    background = float(np.median(img))
    sub = np.maximum(img - background, 0)
    raw_peak_signal = float(sub.max())
    if raw_peak_signal <= 0:
        return 0.0, 0.0, float(img.max()), {
            "valid": False, "support": 0, "threshold_support": 0,
            "dominance": 0.0, "peak_signal": raw_peak_signal,
            "background": background,
        }

    # Find the brightest 3x3 neighbourhood first.  Basing the threshold on the
    # raw global maximum lets one hot pixel hide a dim, resolved beam.
    padded = np.pad(sub, 1)
    smooth = sum(padded[dy:dy + img.shape[0], dx:dx + img.shape[1]]
                 for dy in range(3) for dx in range(3))
    py, px = np.unravel_index(np.argmax(smooth), smooth.shape)
    neighbourhood = sub[max(0, py - 1):py + 2, max(0, px - 1):px + 2]
    peak_signal = float(neighbourhood.max())
    mask = sub >= peak_signal * 0.5
    threshold_support = int(mask.sum())
    # A near-uniform image has no localisable spot and would make the Python
    # component walk needlessly expensive.
    if threshold_support > img.size // 4:
        return 0.0, 0.0, float(img.max()), {
            "valid": False, "support": 0,
            "threshold_support": threshold_support, "dominance": 0.0,
            "peak_signal": peak_signal, "background": background,
        }

    ys, xs = dominant_component(mask)
    support = len(xs)
    if support:
        weights = sub[ys, xs]
        total = weights.sum()
        cx = float((xs * weights).sum() / total)
        cy = float((ys * weights).sum() / total)
        span_x = int(xs.max() - xs.min() + 1)
        span_y = int(ys.max() - ys.min() + 1)
    else:
        cx = cy = 0.0
        span_x = span_y = 0
    dominance = support / threshold_support if threshold_support else 0.0
    # Area alone is not a safe coherence test: a well-focused beam can have
    # fewer than 25 half-peak pixels while still spanning several rows and
    # columns. Accept that compact case only when one component owns at least
    # half of all selected pixels. The wider/speckled case retains the older
    # 25-pixel / 35% rule. Both branches exclude isolated hot pixels and thin
    # readout streaks through the 2-D span requirement.
    resolved = span_x >= 3 and span_y >= 3
    compact = support >= 9 and dominance >= 0.40
    extended = support >= 25 and dominance >= 0.35
    valid = resolved and (compact or extended)
    return cx, cy, float(img.max()), {
        "valid": valid, "support": support,
        "threshold_support": threshold_support, "dominance": dominance,
        "peak_signal": peak_signal, "background": background,
    }


def align(v, a):
    """Nearest multiple of a."""
    return int(round(v / a)) * a


def roi_start(bx, by, w, h, sw, sh):
    sx = align(bx - (w - 1) / 2, 4)          # start_x on 4-px grid (measured)
    sy = align(by - (h - 1) / 2, 2)          # start_y on 2-px grid (measured)
    sx = max(0, min(sx, sw - w))
    sy = max(0, min(sy, sh - h))
    return sx, sy


def asi_params(cfg):
    """Return the asi_source params dict of a config, or None."""
    st = asi_stage(cfg)
    return st["params"] if st else None


def com_window(cfg):
    """Tracking-window (h, w) from the config's COM tracker, or None.

    The window centres on the beam and spans (region-1)/2 either side, so an
    off-centre park eats into the clearance against the ROI edge. Running out
    is not a soft failure: it truncates the spot and desensitises the centroid,
    which is what halved PAR-4's y rejection in every band.
    """
    for st in cfg.get("pipeline", []):
        if "center_of_mass" in st.get("uri", ""):
            p = st.get("params", {})
            if "region_height" in p and "region_width" in p:
                return p["region_height"], p["region_width"]
        if st.get("uri") == "anyloop:fit_com":
            p = st.get("params", {})
            height = p.get("window_height", 0)
            width = p.get("window_width", 0)
            if height and width:
                return height, width
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-capture", action="store_true",
                    help="reuse existing probe_full.aylp instead of grabbing a new one")
    ap.add_argument("--exposure", type=int, default=None,
                    help="probe exposure in µs (default: reference config)")
    ap.add_argument("--gain", type=int, default=None,
                    help="probe gain (default: reference config)")
    ap.add_argument("--settle", type=float, default=30.0,
                    help="seconds to wait for the FSM to park before capturing (default 30; "
                         "the beam drifts for ~30 s after the hold starts). Use 0 to skip.")
    ap.add_argument("--camera", default=None,
                    help="camera_name substring to probe with (default: whatever "
                         "--ref-config asks for)")
    ap.add_argument("--ref-config", default=REF_CFG,
                    help="config whose camera the probe should match "
                         f"(default {os.path.relpath(REF_CFG, ROOT)})")
    ap.add_argument("--aylp", default=None, help="path to an existing .aylp probe file")
    ap.add_argument("--write", action="store_true",
                    help="edit the computed start_x/start_y back into each config")
    ap.add_argument("--config", action="append", dest="configs", default=[],
                    help="limit reporting/writes to this config (repeatable; "
                         "default is every contrib JSON)")
    args = ap.parse_args()

    if args.aylp:
        path = args.aylp
    elif args.no_capture:
        path = os.path.join(ROOT, "data/calibration/run_staging/probe_full.aylp")
    else:
        ref_params = camera_params_from_config(args.ref_config) or {}
        camera = args.camera or ref_params.get("camera_name")
        if camera is None:
            sys.exit(f"no camera_name in {args.ref_config} and none given -- "
                     "pass --camera")
        if not args.camera:
            print(f"# camera \"{camera}\" taken from "
                  f"{os.path.relpath(args.ref_config, ROOT)}", file=sys.stderr)
        sensor = sensor_size(camera)
        if sensor is None:
            print("# WARNING: falling back to probe_frame.json's width/height "
                  "-- if they are not this sensor's full size the CoM will be "
                  "measured on a crop and every ROI below will be wrong",
                  file=sys.stderr)
        if args.settle > 0:
            print(f"# settling: waiting {args.settle:.0f} s for the beam to park "
                  "(make sure the FSM is held at bias)...", file=sys.stderr)
            time.sleep(args.settle)
        exposure = (args.exposure if args.exposure is not None
                    else ref_params.get("exposure"))
        gain = args.gain if args.gain is not None else ref_params.get("gain")
        path = capture(exposure, gain, camera, sensor)

    img = read_frames(path)
    bx, by, mx, detection = beam_com(img)
    H, W = img.shape
    if args.write and not detection["valid"]:
        sys.exit(f"refusing --write: probe signal {detection['peak_signal']:.1f} "
                 f"DN above background, dominant half-peak support "
                 f"{detection['support']} px ({detection['dominance']:.0%} of "
                 "selected pixels); no spatially coherent beam was found")
    if not detection["valid"]:
        print("# WARNING: no spatially coherent beam was found; do not use the "
              "reported position", file=sys.stderr)
    print(f"# beam CoM: x={bx:.1f}  y={by:.1f}   (sensor {W}x{H}, peak px {mx:.0f})")
    print(f"# dominant half-peak support: {detection['support']} / "
          f"{detection['threshold_support']} px "
          f"({detection['dominance']:.0%}); signal "
          f"{detection['peak_signal']:.1f} DN above background")
    print(f"# offset = beam position minus frame centre (dim-1)/2, in ROI px:"
          f" 0.0 means the loop sees zero error with the beam parked.")
    print(f"# x is limited to +-2.0 by the 4-px ASI grid, y to +-1.0 by the"
          f" 2-px grid -- see the module docstring.")
    print(f"# {'config':28s} {'WxH':>9s}  start_x  start_y"
          f"   offset(x,y)  margin")

    files = args.configs or glob.glob(
        os.path.join(ROOT, "contrib", "configs", "**", "*.json"),
        recursive=True)
    files = [f if os.path.isabs(f) else os.path.join(ROOT, f) for f in files]
    for f in sorted(set(files)):
        try:
            cfg = json.load(open(f))
        except (ValueError, OSError):
            continue
        p = asi_params(cfg)
        if not p or "width" not in p or "height" not in p:
            continue
        w, h = p["width"], p["height"]
        # A config on "auto" sizes and centres its own ROI from a beam probe at
        # startup, so there is nothing here to compute or write: it would be a
        # stale number by the time the loop ran, and it is the number the loop
        # would then use INSTEAD of measuring. Leave it alone and say so.
        if not isinstance(w, int) or not isinstance(h, int):
            print(f"  {os.path.basename(f):28s} {'auto':>9s}"
                  f"   {'-':>7s}  {'-':>7s}   (self-sizing ROI, skipped)")
            continue
        if w >= W and h >= H:        # the full-sensor probe config itself
            continue
        sx, sy = roi_start(bx, by, w, h, W, H)
        ox = (bx - sx) - (w - 1) / 2
        oy = (by - sy) - (h - 1) / 2
        win = com_window(cfg)
        if win:
            rh, rw = win
            # clearance between the tracking window's furthest edge and the
            # ROI's; negative means the window is running off the sensor crop
            mx = (w - 1) / 2 - (abs(ox) + (rw - 1) / 2)
            my = (h - 1) / 2 - (abs(oy) + (rh - 1) / 2)
            m = min(mx, my)
            marg = f"{m:5.1f}px" + ("  CLIPS!" if m < 0 else
                                    "  TIGHT" if m < 1.0 else "")
        else:
            marg = "     n/a"
        print(f"  {os.path.basename(f):28s} {w:4d}x{h:<4d}  {sx:7d}  {sy:7d}"
              f"   ({ox:+.1f},{oy:+.1f})  {marg}")

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
