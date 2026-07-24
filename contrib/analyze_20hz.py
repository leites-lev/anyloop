#!/usr/bin/env python3
"""Measure the soundcard->FSM->camera delay from the 20 Hz drive/response
recordings written by conf_latency_20hz.json.

Reads the drive (command, element 0) and response (CoM y, element 0), both one
sample per camera frame, and finds the time lag between them by cross-correlation
(searched over one drive period, so it is unambiguous for a delay < one period).

CAVEAT: at a single frequency the measured lag = TOTAL phase / (2*pi*f), which
includes the AC-coupling phase LEAD -- so the 20 Hz lag slightly UNDER-estimates
(or, if the coupling corner is high, can even invert) the true transport delay.
For a clean delay use the phase slope over a sweep (conf_bode_soundcard_x.json).
"""
import sys, struct
import numpy as np

HDR = 40  # magic u32 + 4xu8 + log_dim(2xu64=16) + pitch(2xdouble=16)


def read_vec(path):
    d = open(path, "rb").read()
    off, rows = 0, []
    while off + HDR <= len(d):
        y, x = struct.unpack_from("<QQ", d, off + 8)
        n = y * x
        if n == 0 or off + HDR + 8 * n > len(d):
            break
        rows.append(struct.unpack_from("<%dd" % n, d, off + HDR))
        off += HDR + 8 * n
    return np.array(rows)


def main():
    fs = float(sys.argv[3]) if len(sys.argv) > 3 else 3788.0
    resp = read_vec(sys.argv[1] if len(sys.argv) > 1 else "sc20_resp.aylp")
    cmd = read_vec(sys.argv[2] if len(sys.argv) > 2 else "sc20_cmd.aylp")
    n = min(len(resp), len(cmd))
    print(f"# frames: resp {len(resp)}, cmd {len(cmd)}, using {n}; fs={fs:.0f} Hz")
    skip = int(1.5 * fs)  # drop acquire + settle
    y = resp[skip:n, 0].astype(float)
    u = cmd[skip:n, 0].astype(float)
    y -= y.mean(); u -= u.mean()

    # dominant drive frequency (sanity + phase method)
    U = np.fft.rfft(u); f = np.fft.rfftfreq(len(u), 1 / fs)
    k = 1 + np.argmax(np.abs(U[1:])); fd = f[k]
    resp_amp = np.abs(np.fft.rfft(y)[k]) / len(y) * 2
    print(f"# drive freq {fd:.2f} Hz; response amp at drive {resp_amp:.4f} "
          f"(0 => that channel not moving this axis)")

    # cross-correlation over one drive period
    per = int(round(fs / fd))
    cc = np.array([np.dot(u[:len(u) - L], y[L:]) for L in range(per)])
    lag = int(np.argmax(cc))
    # express as signed (a lag near a full period = small negative)
    lag_signed = lag if lag <= per // 2 else lag - per
    print(f"# xcorr peak lag {lag} fr ({lag_signed} signed) "
          f"= {lag_signed / fs * 1e3:.3f} ms")

    # phase method (same info, continuous)
    ph = np.angle(np.fft.rfft(y)[k] / np.fft.rfft(u)[k])  # response - drive
    delay_ph = (-ph) / (2 * np.pi * fd)                    # seconds, in (-T/2,T/2]
    print(f"# phase(resp-drive) {np.degrees(ph):+.1f} deg -> "
          f"delay {delay_ph * 1e3:+.3f} ms  (<-- report this)")
    print(f"#   in frames: {delay_ph * fs:+.2f}")


if __name__ == "__main__":
    main()
