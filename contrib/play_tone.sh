#!/usr/bin/env bash
# Play a precise sine tone to the sound card for scope / FSM-drive testing.
#
# Usage:  contrib/play_tone.sh <freq_hz> <amplitude> [seconds]
#   freq_hz    : tone frequency in Hz (e.g. 50, 1000, 2.5)
#   amplitude  : 0..1 fraction of full scale (0.8 gave ~3 Vpp / +-1.5 V on
#                this host's line-out at Master 0 dB; 1.0 ~ 3.75 Vpp)
#   seconds    : play duration; omit or "inf" to loop until Ctrl-C
#
# Env overrides:
#   ALSA_DEV   : ALSA device (default hw:0,0)
#   RATE       : sample rate (default 48000; ALC3234 max is 48000)
#
# Notes:
#  * Both channels (L=R) are driven simultaneously.
#  * The loop-forever mode generates an integer number of cycles so it repeats
#    seamlessly (no click at the wrap).
#  * This plays a TRUE 48 kHz sine -- unlike the anyloop test_source->aylp_alsa
#    path, whose waveform is sampled at the loop rate. Use this for measuring
#    the card (corner, voltage, linearity); use an anyloop config only when you
#    specifically want to exercise the aylp_alsa plugin in the real signal chain.
set -euo pipefail

FREQ=${1:?usage: play_tone.sh <freq_hz> <amplitude 0-1> [seconds|inf]}
AMP=${2:?usage: play_tone.sh <freq_hz> <amplitude 0-1> [seconds|inf]}
DUR=${3:-inf}
DEV=${ALSA_DEV:-hw:0,0}
RATE=${RATE:-48000}

WAV=$(mktemp --suffix=.wav)
APID=""
cleanup() { rm -f "$WAV"; }
# Ctrl-C / kill: stop the running aplay AND break the loop. aplay is run in the
# background and we `wait` on it, so this trap fires immediately (a trap can't
# run while a foreground child holds the terminal, which is why the naive
# `while true; do aplay; done` just restarted aplay after every Ctrl-C).
stop() { trap - INT TERM; echo; echo "stopped."; [ -n "$APID" ] && kill "$APID" 2>/dev/null || true; cleanup; exit 130; }
trap cleanup EXIT
trap stop INT TERM

# base target ~5 s per generated chunk when looping
BASE=5
python3 - "$WAV" "$RATE" "$BASE" "$FREQ" "$AMP" "$DUR" <<'PY'
import wave, struct, math, sys
path, rate, base, f, amp, dur = sys.argv[1:7]
rate=int(rate); base=float(base); f=float(f); amp=max(-1.0,min(1.0,float(amp)))
if dur == "inf":
    cycles=max(1, round(f*base))          # integer cycles -> seamless loop
    n=max(1, round(cycles*rate/f))
else:
    n=int(rate*float(dur))
w=wave.open(path,'wb'); w.setnchannels(2); w.setsampwidth(2); w.setframerate(rate)
buf=bytearray()
for i in range(n):
    v=int(amp*32767*math.sin(2*math.pi*f*i/rate))
    buf+=struct.pack('<hh', v, v)
w.writeframes(bytes(buf)); w.close()
PY

echo "Playing ${FREQ} Hz at amplitude ${AMP} FS on ${DEV} @ ${RATE} Hz (both channels). Ctrl-C to stop."
if [ "$DUR" = inf ]; then
	while true; do
		aplay -D "$DEV" "$WAV" 2>/dev/null &
		APID=$!
		wait "$APID" || true
	done
else
	aplay -D "$DEV" "$WAV" &
	APID=$!
	wait "$APID" || true
fi
