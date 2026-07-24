#!/bin/bash
# Run the end-to-end latency test 5 times on x then 5 times on y, back-to-back,
# automatically labeled run1..run5 (x) and yrun1..yrun5 (y) in the results file
# (Bode_and_testruns/latency_test_results.dat -- both axes append to the same
# file). Each run also reports the plant DC gain (px per unit minmax command,
# same units as bode_plot's fit_K), so this measures Kx/Ky as well as latency.
# Each run holds the fine channels at bias for 30 s first (piplate start_delay
# in the config) so the coarse channels stabilize.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
CONF_X="$REPO/contrib/conf_latency_test.json"
CONF_Y="$REPO/contrib/conf_latency_y.json"

set_label() {
	# $1 = config path, $2 = label
	python3 - "$1" "$2" <<'PYEOF'
import re, sys
p, label = sys.argv[1], sys.argv[2]
c = open(p).read()
c = re.sub(r'"label":\s*"[^"]*"', '"label":      "%s"' % label, c)
open(p, 'w').write(c)
PYEOF
}

run_set() {
	# $1 = config path, $2 = label prefix (e.g. "run" or "yrun")
	local conf="$1" prefix="$2" i
	for i in 1 2 3 4 5; do
		set_label "$conf" "$prefix$i"
		echo "=== $prefix$i ==="
		timeout 150 "$REPO/build/anyloop" "$conf" 2>&1 \
			| grep -E "acquisition done|calibrated|edges measured|plant DC gain|departure \(first|50% crossing:|sample interval|median departure|ERROR|WARN"
	done
}

echo "########## X AXIS (5 runs) ##########"
run_set "$CONF_X" "run"
echo "########## Y AXIS (5 runs) ##########"
run_set "$CONF_Y" "yrun"

# restore neutral labels
set_label "$CONF_X" single
set_label "$CONF_Y" yrun5
