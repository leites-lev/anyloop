#!/bin/bash
# Run the com_survey capture with the ROI re-centred on the beam FIRST, then
# analyse it. This is the entry point -- do not run contrib/calibration-scripts/configurations/com_survey.json by
# hand unless you know the ROI is already centred.
#
# Why the centring step is not optional: every number the survey reports is a
# property of the beam as it lands on the sensor, and an off-centre ROI clips
# the wings. A clipped beam silently corrupts sigma, the window suggestion and
# the gaussian residual all at once -- measured 2026-08-06, a 19 px x-offset
# put flux in column 0, made position estimators disagree by 5-15 px, and
# inflated the relative residual. Nothing in the output says "clipped"; it just
# reports a smaller, less gaussian beam. The only defence is to re-centre every
# time, which costs one full-sensor probe.
#
# Usage:
#   contrib/calibration-scripts/tools/com_survey_run.sh              # beam already parked and settled
#   contrib/calibration-scripts/tools/com_survey_run.sh 30           # wait 30 s for the FSM to settle first
#
# The settle argument goes to find_roi.py --settle. Use it when the coarse
# channels have just moved: com_survey.json expects the FSM parked at its
# normal bias, and drift during settling reads as beam motion in the survey.
set -euo pipefail

cd "$(dirname "$0")/../../.."
SETTLE="${1:-0}"
CFG=contrib/calibration-scripts/configurations/com_survey.json
STAGING=data/calibration/run_staging
STAMP=$(date +%Y-%m-%dT%H-%M-%S)
RECORD="data/calibration/runs/${STAMP}_com_tracker_survey"
OUT="$STAGING/com_survey.aylp"
mkdir -p "$STAGING" "$RECORD"

echo "=== 1/3  centring the ROI on the beam (settle ${SETTLE}s) ==="
# --write edits start_x/start_y into every config with an asi_source, not just
# this one, so the survey and the loop configs stay on the same ROI. start_x
# snaps to a multiple of 4 and start_y to a multiple of 2 in the ASI SDK, so
# the residual offset cannot be driven below about +-2 px in x by cropping.
sudo python3 contrib/calibration-scripts/tools/find_roi.py --settle "$SETTLE" --write

echo
echo "=== 2/3  capturing ==="
LOG="$RECORD/capture_console.log"
sudo chrt -f 80 taskset -c 2 ./build/anyloop "$CFG" 2>&1 | tee "$LOG" \
	| grep -E "Camera 0 ready|stall\(s\)" || true

# Take fs from the LAST reading, never the first: the camera reports ~1849 Hz
# for its first second while it settles and 2310 Hz from the second second on,
# so a short capture or a first-line grep reads ~20% low and mis-scales every
# frequency in the analysis.
FS=$(grep -o "loop [0-9]* Hz" "$LOG" | tail -1 | tr -dc '0-9')
if [ -z "$FS" ]; then
	echo "could not read the achieved frame rate from the run; aborting" >&2
	exit 1
fi
echo "achieved fs: ${FS} Hz"

echo
echo "=== 3/3  analysing ==="
./build/com_survey "$OUT" --fs "$FS" \
	> "$RECORD/analysis_result.json" \
	2> "$RECORD/analyzer.log"
mv "$OUT" "$RECORD/camera_capture.aylp"
if [ -f "$STAGING/probe_full.aylp" ]; then
	mv "$STAGING/probe_full.aylp" "$RECORD/full_frame_probe.aylp"
fi
{
	printf 'recorded_at=%s\n' "$(date --iso-8601=seconds)"
	printf 'calibration=com_tracker_survey\nsource_config=%s\n' "$CFG"
	printf 'measured_frame_rate_hz=%s\n' "$FS"
} > "$RECORD/calibration_manifest.txt"
echo "calibration archived in $RECORD"
