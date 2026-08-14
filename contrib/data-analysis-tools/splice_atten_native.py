#!/usr/bin/env python3
"""Rebuild a run's attenuation PDF in attenuation_test's OWN format, using a
different run's open phase as the reference.

Motivation: PAR-5's open phase is unusable (a mount bump at t~200 s pinned the
beam against the ROI edge for the rest of the open window), so every rejection
number attenuation_test derived from it is void. Its CLOSED phase is clean and
is the best y on record. This writes a spliced .dat -- PAR-3's psd_open columns
beside PAR-5's psd_closed columns, attenuation recomputed -- and then runs
attenuation_test's own plot script on it, so the output is byte-for-byte the
same LAYOUT as every other attenuation PDF: three graph rows (PSD, attenuation
in dB, error variance per 5 Hz band) per axis, plus the config / per-band RMS
page.

The plot script is extracted from devices/attenuation_test.c at run time rather
than copied, so this cannot drift out of sync with the C source.

Why PAR-3 and not PAR-4 (which is the same day): PAR-5's own clean pre-bump
window (t=110-200 s) read y 0.456 / x 0.339 px, against PAR-3's y 0.465 /
x 0.312 and PAR-4's y 0.553 / x 0.423. Closer in ENVIRONMENT beats closer in
time when the denominator is an environment measurement; PAR-4 would flatter
PAR-5's rejection ratio by 15-40%.

    python3 contrib/data-analysis-tools/splice_atten_native.py            # PAR-3 open + PAR-5 closed
    python3 contrib/data-analysis-tools/splice_atten_native.py 3 5 out.pdf
"""
import ast
import os
import subprocess
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))


def extract_plot_script(dest):
    """Pull PLOT_SCRIPT out of attenuation_test.c so the layout always matches
    whatever the C device would have produced."""
    src = open(os.path.join(REPO, "devices", "attenuation_test.c")).read()
    i = src.index("static const char *PLOT_SCRIPT =")
    j = src.index(";\n", i)
    out = []
    for line in src[i + len("static const char *PLOT_SCRIPT ="):j].splitlines():
        s = line.strip().rstrip(";").strip()
        if s.startswith('"'):
            out.append(ast.literal_eval(s))
    open(dest, "w").write("".join(out))
    return dest


def read_meta(path):
    """Return (comment lines, numeric array)."""
    com = [l.rstrip("\n") for l in open(path) if l.startswith("#")]
    return com, np.loadtxt(path)


def get(com, prefix):
    for l in com:
        b = l[1:].strip()
        if b.startswith(prefix):
            return b
    return None


def main():
    n_open = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    n_closed = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    out_pdf = (sys.argv[3] if len(sys.argv) > 3
               else "attenuation_par_fsp%d_open%d.pdf" % (n_closed, n_open))

    po_path = os.path.join(REPO, "attenuation_par_fsp%d.dat" % n_open)
    pc_path = os.path.join(REPO, "attenuation_par_fsp%d.dat" % n_closed)
    com_o, do = read_meta(po_path)
    com_c, dc = read_meta(pc_path)

    n_elem = (dc.shape[1] - 1) // 3
    f = dc[:, 0]                       # closed run's grid is authoritative
    fo = do[:, 0]

    # The two runs measured fs a hair apart (3788.11 vs 3788.22), so their bin
    # centres differ by up to ~12% of a bin. Interpolate the open PSD onto the
    # closed run's grid rather than assuming they line up.
    cols = [f]
    for e in range(n_elem):
        open_psd = np.interp(f, fo, do[:, 1 + 3 * e])
        closed_psd = dc[:, 2 + 3 * e]
        att = 10.0 * np.log10((closed_psd + 1e-300) / (open_psd + 1e-300))
        cols += [open_psd, closed_psd, att]
    arr = np.column_stack(cols)

    # --- header, in attenuation_test's exact shape -----------------------
    ps = float(get(com_c, "pixel_scale=").split("=", 1)[1])
    labels = "y,x"
    hdr = get(com_c, "fs_open=")
    fs_closed = [t for t in hdr.split() if t.startswith("fs_closed=")][0]
    fs_open = [t for t in get(com_o, "fs_open=").split()
               if t.startswith("fs_open=")][0]
    lines = []
    lines.append("# %s %s nfft=4096 open_s=500 closed_s=500 labels=%s"
                 % (fs_open, fs_closed, labels))
    lines.append("# %s" % get(com_c, "start="))
    lines.append("# pixel_scale=%g" % ps)

    # stat lines: open half from the reference run, closed half from this one
    so = {int(l.split()[2]): l.split()[3:7] for l in com_o
          if l.startswith("# stat")}
    sc = {int(l.split()[2]): l.split()[3:7] for l in com_c
          if l.startswith("# stat")}
    for e in range(n_elem):
        lines.append("# stat %d %s %s %s %s"
                     % (e, so[e][0], so[e][1], sc[e][2], sc[e][3]))

    lines.append("# config SPLICED FIGURE -- open phase from PAR-%d, closed "
                 "phase from PAR-%d." % (n_open, n_closed))
    lines.append("# config PAR-%d's own open phase is VOID: a mount bump at "
                 "t~200 s pinned the beam against the ROI edge for the rest "
                 "of the open window (its raw open figures were y 2.51 / "
                 "x 6.70 px). Only its CLOSED phase is used here."
                 % n_closed)
    lines.append("# config Reference chosen for ENVIRONMENT match, not "
                 "recency: PAR-5's clean pre-bump window (t=110-200 s) read "
                 "y 0.456 / x 0.339 px, vs PAR-3 y 0.465 / x 0.312 and PAR-4 "
                 "y 0.553 / x 0.423. PAR-4 is the same day but 21%/25% "
                 "larger, which would flatter the rejection ratio.")
    lines.append("# config The 'mean -> mean' figures on page 2 mix runs by "
                 "construction (open mean is PAR-%d's, closed is PAR-%d's) "
                 "and are not a within-run drift measurement. The RMS-about-"
                 "mean pair IS the comparison of interest." % (n_open, n_closed))
    lines.append("# config Open PSD interpolated onto the closed run's "
                 "frequency grid (the two runs measured fs 3788.11 vs "
                 "3788.22, up to ~12%% of a bin apart).")
    lines.append("# freq_Hz" + "".join(
        " psd_open_%d psd_closed_%d atten_dB_%d" % (e, e, e)
        for e in range(n_elem)))

    dat_path = os.path.splitext(out_pdf)[0] + ".dat"
    with open(dat_path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
        for row in arr:
            fh.write(" ".join("%g" % v for v in row) + "\n")

    script = extract_plot_script("/tmp/atten_splice_plot_%d.py" % os.getpid())
    r = subprocess.run([sys.executable, script, dat_path, out_pdf, labels],
                       capture_output=True, text=True)
    os.unlink(script)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        return 1

    print("wrote %s  and  %s" % (out_pdf, dat_path))
    for e, lab in enumerate(labels.split(",")):
        m = (f >= 1) & (f < 400)
        o = ps * np.sqrt(np.trapz(arr[m, 1 + 3 * e], f[m]))
        c = ps * np.sqrt(np.trapz(arr[m, 2 + 3 * e], f[m]))
        print("  %s  open(PAR-%d) %.4f px -> closed(PAR-%d) %.4f px   %.1fx"
              % (lab, n_open, o, n_closed, c, o / c))
    return 0


if __name__ == "__main__":
    sys.exit(main())
