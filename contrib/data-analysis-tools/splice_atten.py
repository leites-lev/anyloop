#!/usr/bin/env python3
"""Rebuild the PAR-5 attenuation figure against a reference open loop.

PAR-5's OWN open phase is unusable: a ~15 px mount disturbance at t~200 s
pinned the beam against the ROI edge for the rest of the open window, so its
open PSD -- and every rejection number attenuation_test derived from it -- is
meaningless. Its CLOSED phase is clean. This figure keeps PAR-5's closed PSD
and substitutes PAR-3's open PSD as the reference.

What licenses the substitution: PAR-5's pre-disturbance window (t=110-200 s,
before the bump) measured y 0.456 / x 0.339 px rms against PAR-3's full open
phase at y 0.465 / x 0.312 px -- the same environment to within ~8%.

Everything is read from the attenuation_test .dat files, so every PSD comes
from the same estimator on the same 4096-point grid.

CAVEAT that must travel with any comparison here: the runs did NOT score the
same window. PAR-3 ran settle_time 120 s, so its closed PSD covers t=720-1220
while the observer was still converging; PAR-4 and PAR-5 ran settle_time 600 s
and scored t=1200-1700. PAR-3's closed trace is therefore pessimistic relative
to the other two, and is included as historical context, not as a fair peer.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

PXU = 15.5
# dataviz reference palette: neutral for the baseline, categorical slots 1-3
C_OPEN = "#8a8a80"
C_RUN = [(3, "PAR-3", "#2a78d6"), (4, "PAR-4", "#eb6834"), (5, "PAR-5", "#1baf7a")]
BANDS = [(1, 10), (10, 30), (30, 60), (60, 150), (150, 250), (250, 400)]
COL = {"y": (1, 2), "x": (4, 5)}          # (psd_open, psd_closed) per axis


def load(n):
    a = np.array([l.split() for l in open(f"attenuation_par_fsp{n}.dat")
                  if not l.startswith("#") and l.strip()], float)
    return a[:, 0], a


def stats(n):
    out = {}
    for l in open(f"attenuation_par_fsp{n}.dat"):
        if l.startswith("# stat"):
            p = l.split()
            out["yx"[int(p[2])]] = (float(p[3]) * PXU, float(p[4]) * PXU,
                                    float(p[5]) * PXU, float(p[6]) * PXU)
    return out


def brms(f, p, lo, hi):
    m = (f >= lo) & (f < hi)
    return np.sqrt(np.trapz(p[m], f[m])) * PXU


# Open reference: PAR-3. PAR-4 is the same day as PAR-5, but its open is
# 21% (y) / 25% (x) LARGER than PAR-5's own clean pre-bump window
# (t=110-200 s, y 0.456 / x 0.339 px), whereas PAR-3 matches it closely
# (y 0.465 / x 0.312) -- closer in environment beats closer in time, and
# using PAR-4 would flatter PAR-5's rejection ratio. Both are tabulated
# in the sensitivity block either way.
REF, REF_LBL = 3, "PAR-3"      # open reference
ALT, ALT_LBL = 4, "PAR-4"      # sensitivity check against the same-day open
f3, d3 = load(REF)
dalt = load(ALT)[1]
data = {lbl: load(n)[1] for n, lbl, _ in C_RUN}

pdf_path = "attenuation_par_fsp5_spliced.pdf"
with PdfPages(pdf_path) as pdf:
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))
    fig.suptitle(
        "PAR-5 closed loop vs %s reference open loop\n" % REF_LBL +
        "PAR-5's own open phase is discarded (mount disturbance pinned the "
        "beam at the ROI edge, t$\\approx$200-600 s)", fontsize=11)
    for col, ax in enumerate("yx"):
        io, ic = COL[ax]
        po = d3[:, io]
        a = axes[0][col]
        a.loglog(f3[1:], po[1:] * PXU**2, color=C_OPEN, lw=2.2,
                 label="open (%s reference)" % REF_LBL)
        for n_, name, c in C_RUN:
            a.loglog(f3[1:], data[name][1:, ic] * PXU**2, color=c, lw=1.3,
                     label=f"{name} closed")
        a.set_xlim(1, 400)
        a.set_title(f"{ax} axis  —  power spectral density", fontsize=10)
        a.set_ylabel("PSD  (px$^2$/Hz)")
        a.grid(True, which="both", alpha=0.16, lw=0.5)
        a.legend(fontsize=8, framealpha=0.9)

        b = axes[1][col]
        b.axhline(0, color="#555", lw=1)
        for n_, name, c in C_RUN:
            with np.errstate(divide="ignore", invalid="ignore"):
                att = 10 * np.log10(data[name][:, ic] / po)
            b.semilogx(f3[1:], att[1:], color=c, lw=1.3, label=name)
        b.set_xlim(1, 400)
        b.set_ylim(-40, 20)
        b.set_title(f"{ax} axis  —  attenuation vs {REF_LBL} open "
                    "(negative = rejection)", fontsize=10)
        b.set_xlabel("frequency (Hz)")
        b.set_ylabel("closed / open  (dB)")
        b.grid(True, which="both", alpha=0.16, lw=0.5)
        b.legend(fontsize=8, framealpha=0.9)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    pdf.savefig(fig)
    plt.close(fig)

    fig, a = plt.subplots(figsize=(11, 8.5))
    a.axis("off")
    txt = ("PAR-5 SPLICED BAND BUDGET   (RMS in px; open = %s reference)\n"
           % REF_LBL) + ("=" * 74) + "\n\n"
    for ax in "yx":
        io, ic = COL[ax]
        po = d3[:, io]
        txt += f"--- {ax} axis ---\n"
        txt += (f"{'band (Hz)':>12}{'open':>10}" +
                "".join(f"{n:>10}" for _, n, _ in C_RUN) +
                f"{'PAR-5 rej':>12}\n")
        for lo, hi in BANDS + [(1, 400)]:
            o = brms(f3, po, lo, hi)
            row = f"{f'{lo}-{hi}':>12}{o:>10.4f}"
            for n_, n, _ in C_RUN:
                row += f"{brms(f3, data[n][:, ic], lo, hi):>10.4f}"
            c5 = brms(f3, data["PAR-5"][:, ic], lo, hi)
            row += f"{o/c5:>11.1f}x"
            txt += row + ("\n" if (lo, hi) != (1, 400) else "   <- TOTAL\n")
        txt += "\n"
    txt += ("SENSITIVITY TO THE OPEN REFERENCE (total 1-400 Hz)\n"
            "   axis   open ref     open px   PAR-5 closed   rejection\n")
    for lbl, dd in ((REF_LBL, d3), (ALT_LBL, dalt)):
        for ax in "yx":
            io, ic = COL[ax]
            o = brms(f3, dd[:, io], 1, 400)
            c = brms(f3, data["PAR-5"][:, ic], 1, 400)
            txt += "   %-6s %-12s %8.4f   %10.4f   %8.1fx\n" % (ax, lbl, o, c, o / c)
    txt += ("\nTIME-DOMAIN RMS from each run's own '# stat' line "
            "(open mean / open rms / closed rms, px):\n")
    for n_, n, _ in C_RUN:
        s = stats(n_)
        txt += (f"   {n}:  y park {s['y'][0]:+6.2f}  open {s['y'][1]:6.4f}  "
                f"closed {s['y'][3]:6.4f}   |   "
                f"x park {s['x'][0]:+6.2f}  open {s['x'][1]:6.4f}  "
                f"closed {s['x'][3]:6.4f}\n")
    txt += ("\nCAVEATS\n"
            " * OPEN REFERENCE IS PAR-3, chosen for ENVIRONMENT match rather "
            "than recency: PAR-5's own\n   clean pre-bump window (t=110-200 s) "
            "read y 0.456 / x 0.339 px, against PAR-3's y 0.465 /\n   x 0.312 "
            "and PAR-4's y 0.553 / x 0.423. PAR-4 is the same day but its open "
            "is 21%% (y) /\n   25%% (x) larger, which would flatter PAR-5's "
            "ratio. The PAR-4-referenced numbers are in\n   the sensitivity "
            "block above -- quote the pair, the choice moves the headline "
            "15-40%%.\n"
            " * PAR-3 scored t=720-1220 s (settle_time 120) while still "
            "converging; PAR-4 and PAR-5\n   scored t=1200-1700 (settle_time "
            "600). PAR-3's closed trace is pessimistic, not a fair peer.\n"
            " * PAR-4 ran with the beam parked +5.95 px, which pushed the "
            "20x20 CoM window off the\n   32x32 ROI edge and desensitized the "
            "y centroid ~2x. Its y trace is not trustworthy.\n"
            " * PAR-5's own open PSD is discarded here; its '# stat' open "
            "figures (y 2.51 / x 6.70 px)\n   include the mount disturbance "
            "and must not be quoted.\n"
            " * PAR-5's raw error/command records, weights and convergence "
            "trace were overwritten at\n   14:12 by a re-run using the same "
            "output filenames. Only the .dat/.pdf survive.\n")
    a.text(0.02, 0.98, txt, family="monospace", fontsize=8, va="top")
    pdf.savefig(fig)
    plt.close(fig)

print("wrote", pdf_path, "\n")
for ax in "yx":
    io, ic = COL[ax]
    o = brms(f3, d3[:, io], 1, 400)
    line = f"  {ax}  open({REF_LBL} ref) {o:.4f} px |"
    for n_, n, _ in C_RUN:
        c = brms(f3, data[n][:, ic], 1, 400)
        line += f"  {n} {c:.4f} ({o/c:.1f}x)"
    print(line)
