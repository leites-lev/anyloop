#!/usr/bin/env python3
"""Find the current disturbance lines and write them into the bode configs' avoid_freqs.

Records the PARKED-beam CoM time series (open loop, FSM held at its bias, NO
injection), computes the per-axis power spectrum, detects the spectral lines in
the bode band, and writes them into each bode config's avoid_freqs -- the x-axis
lines (CoM x) into conf_bode_x*.json, the y-axis lines (CoM y) into y*.json.

Why: a bench line sitting inside a bode sweep point biases |H| coherently -- the
lock-in segments do NOT average it out, so deeper cycles don't help; the only fix
is to dodge the line. bode_plot.c nudges any sweep point within avoid_df of an
avoid_freqs entry off the line. The lines wander ~0.5 Hz/run, so refresh this list
before a careful bode (especially the deep-K sweeps).

Usage:
    contrib/find_avoid.py                 # settle 30 s, record 40 s, print lines
    contrib/find_avoid.py --write         # ALSO write avoid_freqs into the configs
    contrib/find_avoid.py --duration 60   # record longer (finer/less-noisy spectrum)
    contrib/find_avoid.py --settle 0      # beam already parked; record immediately
    contrib/find_avoid.py --aylp rec.aylp # analyse an existing recording, no capture
    contrib/find_avoid.py --no-park       # don't add a piplate stanza (FSM left as-is)

Notes / gotchas:
  * CoM output is the vector [y, x] (y = element 0 = ch0, x = element 1 = ch1).
  * We only look in [fmin, fmax] ~= the bode band; sub-5 Hz drift (coarse-channel
    settling) is high-passed out, so a missed settle mostly costs low-freq junk we
    ignore anyway -- but --settle 30 still gives a cleaner spectrum.
  * .aylp per-frame layout: 40-byte header (magic u32; version/status/type/units
    u8 at 4..7; log_dim y,x u64 at 8,16; pitch y,x f64 at 24,32) then y*x data
    elements. A CoM VECTOR is 2 float64s.
"""
import argparse, copy, glob, json, os, re, struct, subprocess, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ANYLOOP = os.path.join(ROOT, "build", "anyloop")
AYLP_MAGIC = 0x504C5941

# type flags from libaylp/anyloop.h (element byte size per data type)
T_VECTOR, T_MATRIX = 1 << 2, 1 << 3
T_MATRIX_UCHAR, T_MATRIX_USHORT = 1 << 5, 1 << 6

# which configs each axis' lines get written into
X_CONFIGS = ["conf_bode_x.json", "conf_bode_x_gain.json"]
Y_CONFIGS = ["conf_bode_y.json", "conf_bode_y_gain.json"]


def elem_dtype(t):
    if t & T_MATRIX_UCHAR:  return np.uint8, 1
    if t & T_MATRIX_USHORT: return np.uint16, 2
    return np.float64, 8            # VECTOR / MATRIX are doubles


def build_record_config(ref_path, out_aylp, count, park):
    """Copy the asi_source + center_of_mass (+ parked piplate) of a bode config and
    append file_sink + stop_after_count, so the recording sees the exact sensor the
    bode does but with no injection."""
    cfg = json.load(open(ref_path))
    stages = []
    for st in cfg["pipeline"]:
        uri = st.get("uri", "")
        if "asi_source" in uri or "center_of_mass" in uri:
            stages.append(copy.deepcopy(st))
    # record the CoM vector right after center_of_mass
    stages.append({"uri": "anyloop:file_sink", "params": {"filename": out_aylp}})
    if park:
        # hold the FSM at its bias: copy the bode's piplate but zero every scale so
        # only the offsets drive -> all channels parked, beam still, pure disturbance
        for st in cfg["pipeline"]:
            if "piplate_bridge" in st.get("uri", ""):
                pp = copy.deepcopy(st)
                pp["params"]["scale"] = [0] * len(pp["params"].get("scale", [0]))
                stages.append(pp)
                break
    stages.append({"uri": "anyloop:stop_after_count", "params": {"count": count}})
    cfg["pipeline"] = stages
    return cfg


def capture(ref_path, settle, duration, fs, park):
    out = os.path.join(ROOT, "avoid_probe.aylp")
    count = int(round((settle + duration) * fs))
    cfg = build_record_config(ref_path, out, count, park)
    tmp = os.path.join(ROOT, ".find_avoid_probe.json")
    json.dump(cfg, open(tmp, "w"))
    try:
        print(f"# recording {settle+duration:.0f} s ({count} frames @ {fs:.0f} Hz)"
              f" -- FSM parked, no injection...", file=sys.stderr)
        subprocess.run([ANYLOOP, tmp], cwd=ROOT, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        os.remove(tmp)
    return out


def read_com_series(path):
    """Parse an .aylp of CoM vectors -> (y[], x[]) arrays over frames."""
    d = open(path, "rb").read()
    off, ys, xs = 0, [], []
    while off + 40 <= len(d):
        magic = struct.unpack_from("<I", d, off)[0]
        if magic != AYLP_MAGIC:
            break
        t = d[off + 6]
        ly, lx = struct.unpack_from("<QQ", d, off + 8)
        dt, esz = elem_dtype(t)
        n = ly * lx
        o = off + 40
        if o + n * esz > len(d):
            break
        v = np.frombuffer(d, dt, n, o).astype(float)
        if n >= 2:                 # CoM = [y, x]
            ys.append(v[0]); xs.append(v[1])
        off = o + n * esz
    if len(ys) < 64:
        sys.exit(f"too few CoM samples parsed from {path} ({len(ys)})")
    return np.asarray(ys), np.asarray(xs)


def welch_psd(sig, fs, seg_seconds):
    """Hann-windowed Welch PSD (50% overlap). Returns (freqs, psd)."""
    sig = np.asarray(sig, float)
    sig = sig - sig.mean()
    L = int(min(len(sig), max(256, round(seg_seconds * fs))))
    win = np.hanning(L)
    step = L // 2
    acc, k = None, 0
    for s in range(0, len(sig) - L + 1, step):
        seg = sig[s:s + L] * win
        p = np.abs(np.fft.rfft(seg)) ** 2
        acc = p if acc is None else acc + p
        k += 1
    if k == 0:                     # signal shorter than one segment
        seg = np.pad(sig, (0, L - len(sig))) * win
        acc = np.abs(np.fft.rfft(seg)) ** 2
        k = 1
    return np.fft.rfftfreq(L, 1.0 / fs), acc / k


def _running_median(a, w):
    w = max(3, w | 1)
    half = w // 2
    pad = np.pad(a, half, mode="edge")
    return np.array([np.median(pad[i:i + w]) for i in range(len(a))])


def find_lines(f, psd, fmin, fmax, prom_db, min_sep, max_lines):
    """Detect spectral lines: local maxima whose log-PSD exceeds a running-median
    floor by prom_db. Sub-bin refined by parabolic interpolation, merged within
    min_sep, capped to the strongest max_lines."""
    band = (f >= fmin) & (f <= fmax)
    fb, pb = f[band], psd[band]
    if len(fb) < 5:
        return []
    df = fb[1] - fb[0]
    logp = 10.0 * np.log10(pb + 1e-30)
    floor = _running_median(logp, int(round(2.0 / df)))   # ~2 Hz median floor
    excess = logp - floor
    cand = []
    for i in range(1, len(fb) - 1):
        if excess[i] >= prom_db and logp[i] >= logp[i - 1] and logp[i] >= logp[i + 1]:
            y0, y1, y2 = logp[i - 1], logp[i], logp[i + 1]
            den = y0 - 2 * y1 + y2
            delta = 0.5 * (y0 - y2) / den if den != 0 else 0.0
            delta = max(-1.0, min(1.0, delta))
            cand.append((fb[i] + delta * df, excess[i]))
    cand.sort(key=lambda c: -c[1])                 # strongest first
    kept = []
    for fc, ex in cand:
        if all(abs(fc - kf) >= min_sep for kf, _ in kept):
            kept.append((fc, ex))
        if len(kept) >= max_lines:
            break
    return sorted(round(fc, 2) for fc, _ in kept)


def write_avoid(config_names, freqs, fmin, fmax):
    """Replace ONLY the in-band [fmin,fmax] avoid_freqs with the freshly measured
    lines, keeping any existing out-of-band entries (e.g. the 58/84 Hz lines a full
    5-300 Hz sweep still needs but this <=30 Hz pass never measured). Merging, not
    clobbering, so the tool is safe on both the deep-K and the full configs."""
    written = []
    for name in config_names:
        path = os.path.join(ROOT, "contrib", name)
        if not os.path.isfile(path):
            continue
        s = open(path).read()
        m = re.search(r'("avoid_freqs"\s*:\s*)\[([^\]]*)\]', s)
        if not m:
            continue
        old = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?', m.group(2))]
        keep = [v for v in old if v < fmin or v > fmax]     # out-of-band survivors
        merged = sorted(set(round(v, 2) for v in keep) | set(freqs))
        arr = "[" + ", ".join(f"{v:g}" for v in merged) + "]"
        s2 = s[:m.start()] + m.group(1) + arr + s[m.end():]
        if s2 != s:
            open(path, "w").write(s2)
            written.append(name)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="conf_bode_x_gain.json",
                    help="config to copy asi_source/center_of_mass/piplate from")
    ap.add_argument("--settle", type=float, default=30.0,
                    help="seconds recorded then DISCARDED so the coarse channels park (default 30)")
    ap.add_argument("--duration", type=float, default=40.0,
                    help="seconds of usable recording after the settle (default 40)")
    ap.add_argument("--fs", type=float, default=None,
                    help="sample rate Hz (default: the ref config's bode sample_rate, or 3788)")
    ap.add_argument("--seg-seconds", type=float, default=6.0,
                    help="Welch segment length in s (RBW ~= 1/this; default 6 -> ~0.17 Hz)")
    ap.add_argument("--fmin", type=float, default=4.5, help="line search low edge Hz")
    ap.add_argument("--fmax", type=float, default=31.0, help="line search high edge Hz")
    ap.add_argument("--prominence", type=float, default=6.0,
                    help="dB a peak must clear the local median floor to count (default 6)")
    ap.add_argument("--min-sep", type=float, default=0.4, help="min Hz between lines")
    ap.add_argument("--max-lines", type=int, default=14,
                    help="cap per axis (too many nudged points distort the grid)")
    ap.add_argument("--aylp", default=None, help="analyse this existing recording; skip capture")
    ap.add_argument("--no-park", action="store_true",
                    help="do not add a piplate stanza (leave the FSM wherever it is)")
    ap.add_argument("--write", action="store_true",
                    help="write avoid_freqs into the bode configs (default: print only)")
    args = ap.parse_args()

    ref_path = os.path.join(ROOT, "contrib", args.ref)
    fs = args.fs
    if fs is None:
        try:
            cfg = json.load(open(ref_path))
            fs = next(st["params"]["sample_rate"] for st in cfg["pipeline"]
                      if "bode_plot" in st.get("uri", ""))
        except (StopIteration, KeyError, ValueError, OSError):
            fs = 3788.0

    if args.aylp:
        path = args.aylp
        skip = 0
    else:
        path = capture(ref_path, args.settle, args.duration, fs, not args.no_park)
        skip = int(round(args.settle * fs))

    y, x = read_com_series(path)
    y, x = y[skip:], x[skip:]      # drop the settle transient
    print(f"# analysed {len(x)} samples ({len(x)/fs:.1f} s) @ {fs:.0f} Hz, "
          f"band {args.fmin}-{args.fmax} Hz", file=sys.stderr)

    out = {}
    for axis, sig in (("x", x), ("y", y)):
        f, p = welch_psd(sig, fs, args.seg_seconds)
        out[axis] = find_lines(f, p, args.fmin, args.fmax, args.prominence,
                               args.min_sep, args.max_lines)
        print(f"# {axis}-axis lines ({len(out[axis])}): "
              + ", ".join(f"{v:g}" for v in out[axis]))

    if args.write:
        wx = write_avoid(X_CONFIGS, out["x"], args.fmin, args.fmax)
        wy = write_avoid(Y_CONFIGS, out["y"], args.fmin, args.fmax)
        for n in wx: print(f"# wrote x lines -> contrib/{n}")
        for n in wy: print(f"# wrote y lines -> contrib/{n}")
    else:
        print("# (print-only; re-run with --write to update the configs)")


if __name__ == "__main__":
    main()
