#!/bin/bash
# Run the end-to-end latency test 5 times back-to-back, automatically labeled
# run1..run5 in the results file (Bode_and_testruns/latency_test_results.dat).
# Each run holds the fine channels at bias for 30 s first (piplate start_delay
# in conf_latency_test.json) so the coarse channels stabilize.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
CONF="$REPO/contrib/conf_latency_test.json"
for i in 1 2 3 4 5; do
	python3 - "$CONF" "run$i" <<'PYEOF'
import re, sys
p, label = sys.argv[1], sys.argv[2]
c = open(p).read()
c = re.sub(r'"label":\s*"[^"]*"', '"label":      "%s"' % label, c)
open(p, 'w').write(c)
PYEOF
	echo "=== run $i ==="
	timeout 150 "$REPO/build/anyloop" "$CONF" 2>&1 \
		| grep -E "acquisition done|calibrated|edges measured|departure \(first|50% crossing:|sample interval|median departure|ERROR|WARN"
done
python3 - "$CONF" single <<'PYEOF'
import re, sys
p, label = sys.argv[1], sys.argv[2]
c = open(p).read()
c = re.sub(r'"label":\s*"[^"]*"', '"label":      "%s"' % label, c)
open(p, 'w').write(c)
PYEOF
