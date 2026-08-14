#!/bin/bash
# Run one steering profile while preserving a timestamped diagnostic record.
set -euo pipefail

cd "$(dirname "$0")/../../.."
mkdir -p data/run_staging

usage() {
	cat <<'EOF'
Usage: contrib/steering/tools/run_steering_recorded.sh [CONFIG] [OPTIONS]

Config selection (default: --live):
  --live             Primary live steering config
  --1000             1000 Hz, 20% optical-duty config
  --100              100 Hz, 80% optical-duty config
  --1                1 Hz, 20% optical-duty config
  -c, --config NAME  NAME is live, 1000, 100, or 1
  --beam NAME        Beam label for --live (default: unspecified)
  --session NAME     Session folder (default: today_YYYY-MM-DD)
  -l, --label NAME   Additional descriptive run label

Examples:
  contrib/steering/tools/run_steering_recorded.sh --1000
  contrib/steering/tools/run_steering_recorded.sh --config 100 --label retest
  contrib/steering/tools/run_steering_recorded.sh --live --beam continuous
EOF
}

PROFILE=live
LABEL=
BEAM=
SESSION=
while [ "$#" -gt 0 ]; do
	case "$1" in
		--live) PROFILE=live ;;
		--1000) PROFILE=1000 ;;
		--100) PROFILE=100 ;;
		--1) PROFILE=1 ;;
		-c|--config)
			[ "$#" -ge 2 ] || { echo "$1 requires a value" >&2; usage >&2; exit 2; }
			PROFILE=$2
			shift
			;;
		-l|--label)
			[ "$#" -ge 2 ] || { echo "$1 requires a value" >&2; usage >&2; exit 2; }
			LABEL=$2
			shift
			;;
		--beam)
			[ "$#" -ge 2 ] || { echo "$1 requires a value" >&2; usage >&2; exit 2; }
			BEAM=$2
			shift
			;;
		--session)
			[ "$#" -ge 2 ] || { echo "$1 requires a value" >&2; usage >&2; exit 2; }
			SESSION=$2
			shift
			;;
		-h|--help) usage; exit 0 ;;
		*) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
	esac
	shift
done

case "$PROFILE" in
	live)
		CONFIG=contrib/steering/configurations/steering_par_fsp.json
		OUTPUT_SUFFIX=_1000hz_20pct
		PROFILE_NAME=live_1000hz_20pct
		DEFAULT_BEAM=1000hz-20pct
		DEFAULT_LABEL=primary
		;;
	1000|1000hz|1000hz_20pct)
		PROFILE=1000
		CONFIG=contrib/steering/configurations/run_1000hz_20pct.json
		OUTPUT_SUFFIX=_1000hz_20pct
		PROFILE_NAME=1000hz_20pct
		DEFAULT_BEAM=1000hz-20pct
		DEFAULT_LABEL=standard
		;;
	100|100hz|100hz_80pct)
		PROFILE=100
		CONFIG=contrib/steering/configurations/run_100hz_80pct.json
		OUTPUT_SUFFIX=_100hz_80pct
		PROFILE_NAME=100hz_80pct
		DEFAULT_BEAM=100hz-80pct
		DEFAULT_LABEL=standard
		;;
	1|1hz|1hz_20pct)
		PROFILE=1
		CONFIG=contrib/steering/configurations/run_1hz_20pct.json
		OUTPUT_SUFFIX=_1hz_20pct
		PROFILE_NAME=1hz_20pct
		DEFAULT_BEAM=1hz-20pct
		DEFAULT_LABEL=standard
		;;
	*) echo "unknown config profile: $PROFILE" >&2; usage >&2; exit 2 ;;
esac
[ -f "$CONFIG" ] || { echo "config not found: $CONFIG" >&2; exit 1; }
[ -n "$LABEL" ] || LABEL=$DEFAULT_LABEL
[ -n "$BEAM" ] || BEAM=$DEFAULT_BEAM
[ -n "$SESSION" ] || SESSION="today_$(date +%Y-%m-%d)"

STAMP=$(date +%H-%M-%S)
SAFE_LABEL=$(printf '%s' "$LABEL" | tr -cs 'A-Za-z0-9._-' '_')
SAFE_BEAM=$(printf '%s' "$BEAM" | tr -cs 'A-Za-z0-9._-' '_')
SAFE_SESSION=$(printf '%s' "$SESSION" | tr -cs 'A-Za-z0-9._-' '_')
RECORD="data/steering_runs/${SAFE_SESSION}/${STAMP}_beam-${SAFE_BEAM}_config-${PROFILE_NAME}_${SAFE_LABEL}"
mkdir -p "$RECORD"
LOG="$RECORD/console.log"
cp "$CONFIG" "$RECORD/config_used.json"
git diff --no-ext-diff -- "$CONFIG" devices/fsp.c \
	devices/fsp.h devices/udp_sink.c devices/udp_sink.h \
	contrib/steering/tools/run_steering_recorded.sh > "$RECORD/changes.patch" || true
{
	printf 'run_name=%s\nrecorded_at=%s\n' "${PROFILE_NAME}_${SAFE_LABEL}" "$(date --iso-8601=seconds)"
	printf 'session=%s\nbeam=%s\nprofile=%s\nsource_config=%s\n' \
		"$SAFE_SESSION" "$SAFE_BEAM" "$PROFILE_NAME" "$CONFIG"
	printf 'commit=%s\n' "$(git rev-parse HEAD 2>/dev/null || printf unknown)"
} > "$RECORD/run_manifest.txt"

finish_record() {
	status=$?
	{
		printf 'finished=%s\nexit_status=%s\n' \
			"$(date --iso-8601=seconds)" "$status"
		find . -maxdepth 1 -type f -name 'steering_par_*.aylp' \
			-newer "$RECORD/config.json" -printf 'output=%f bytes=%s\n' | sort
	} >> "$RECORD/run_manifest.txt"
	# A run directory must contain the data it describes.  Moving the active
	# outputs also prevents file_sink's numbered rollover scheme from leaving
	# anonymous recordings at the repository root.
	for item in \
		"data/run_staging/steering_par_err${OUTPUT_SUFFIX}.aylp:measured_error.aylp" \
		"data/run_staging/steering_par_cmd${OUTPUT_SUFFIX}.aylp:actuator_command.aylp" \
		"data/run_staging/steering_par_weights${OUTPUT_SUFFIX}.dat:observer_weights.dat" \
		"data/run_staging/steering_par_wtrace${OUTPUT_SUFFIX}.dat:observer_trace.dat" \
		"data/run_staging/steering_par_transient${OUTPUT_SUFFIX}.csv:transient_events.csv"
	do
		source_name=${item%%:*}
		dest_name=${item#*:}
		if [ -f "$source_name" ] && [ "$source_name" -nt "$RECORD/config.json" ]; then
			mv "$source_name" "$RECORD/$dest_name"
			printf 'data_file=%s\n' "$dest_name" >> "$RECORD/run_manifest.txt"
		fi
	done
}
trap finish_record EXIT

echo "recording diagnostics to $LOG"
echo "run label: $LABEL ($RECORD)"
echo "config profile: $PROFILE ($CONFIG)"
# Do not launch the whole process under chrt/taskset. ASI creates libusb and
# frame-delivery worker threads during initialization; those threads inherit
# the creator's scheduler and affinity. Launching under FIFO/CPU2 therefore
# put every camera worker at FIFO 80 on the same core as the loop, which caused
# the frame gaps this tuning was meant to prevent. Start normally, allow the
# SDK workers to be created with ordinary scheduling/all-core affinity, then
# promote and pin only the main anyloop thread (Linux scheduler operations on
# a PID affect that thread only unless their explicit all-tasks option is used).
sudo bash -c '
	config=$1
	stdbuf -oL -eL ./build/anyloop -p "$config" &
	loop_pid=$!
	sleep 1
	if kill -0 "$loop_pid" 2>/dev/null; then
		taskset -pc 2 "$loop_pid" >&2
		chrt -f -p 80 "$loop_pid"
	fi
	wait "$loop_pid"
' steering-runner "$CONFIG" 2>&1 | tee "$LOG"
