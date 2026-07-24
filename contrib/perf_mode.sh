#!/usr/bin/env bash
#
# perf_mode.sh -- toggle the low-latency system tuning for the anyloop loop.
#
# Fixes the 5-50 ms camera frame-delivery jitter traced 2026-07-21 to OS power
# management (see the asi-usb3 memory + doc): the loop blocks waiting on a frame,
# the cores idle into deep C-states at powersave frequency, and waking the USB
# IRQ / ASI SDK worker thread stacks into milliseconds. Turning this ON took the
# mid-run frame drops from ~10700 to ~20 over a 140 s run.
#
# What --on does (all runtime sysfs, reversible, NOT persistent across reboot):
#   1. CPU governor -> performance on every core (no freq-ramp latency)
#   2. disable CPU idle C-states with >15 us exit latency (keep POLL/C1/C1E)
#   3. pin the xhci USB IRQ(s) to CPU $IRQ_CPU, off the loop core + its sibling
#   4. OFFLINE the loop core's hyperthread SIBLING so nothing contends for the
#      shared execution units mid-measurement (the runtime equivalent of
#      isolating the physical core; taskset+chrt alone leave the sibling live)
# --off restores this box's defaults: powersave, all C-states, IRQ on all CPUs,
# sibling back online.
#
# NOT settable at runtime (boot-time kernel cmdline only) -- see --grub-hint:
#   isolcpus / nohz_full / rcu_nocbs take the periodic scheduler tick, RCU
#   callbacks, and general load-balancing off the loop core for good. A shell
#   cannot apply these live; they need a GRUB edit + reboot. --on offlines the
#   sibling as the best runtime approximation, but the boot params are the
#   persistent, complete fix.
#
# Usage:
#   contrib/perf_mode.sh --on          # apply the runtime tuning
#   contrib/perf_mode.sh --off         # revert to defaults
#   contrib/perf_mode.sh --status      # show current state
#   contrib/perf_mode.sh --grub-hint   # print the boot-param line to add
#
# Then launch the loop pinned + realtime on the reserved core:
#   sudo chrt -f 80 taskset -c 2 ./build/anyloop contrib/steering_fsp.json
#
# Uses sudo for the privileged writes, so run it as your normal user.

set -euo pipefail

LOOP_CPU=2          # core you should taskset the loop onto (kept clear of the IRQ)
IRQ_CPU=3           # park the xhci USB IRQ here, off the loop core + sibling
OFF_GOVERNOR=powersave   # this box's default; change if yours differs
STATE_FILE=/run/perf_mode.offlined   # sibling CPUs we offlined, so --off can restore them

# the loop core's hyperthread sibling(s): everything in its thread_siblings_list
# except LOOP_CPU itself. Offlined in --on so the physical core is ours alone.
# NOTE: once a sibling is offline the kernel drops it from thread_siblings_list,
# so --on records what it offlined in STATE_FILE and --off restores from there.
loop_siblings() {
	local list c
	list=$(tr ',' ' ' < /sys/devices/system/cpu/cpu$LOOP_CPU/topology/thread_siblings_list)
	for c in $list; do
		[ "$c" != "$LOOP_CPU" ] && echo "$c"
	done
	return 0   # never fail: an all-offline list would else exit 1 and, under
	           # set -e/pipefail, abort every caller
}

# the sibling set to act on: the live list UNION whatever --on recorded, so it is
# stable even after the sibling is offlined (and thus dropped from the live list).
target_siblings() {
	{
		loop_siblings
		if [ -f "$STATE_FILE" ]; then cat "$STATE_FILE"; fi
	} | sort -un | grep -E '^[0-9]+$' || true
}

irq_mask() { printf '%x' $(( 1 << $1 )); }          # CPU index -> hex affinity mask
all_cpus_mask() { printf '%x' $(( (1 << $(nproc)) - 1 )); }
xhci_irqs() { awk -F: '/xhci/ {gsub(/ /,"",$1); print $1}' /proc/interrupts; }

set_governor() {
	# skip offline CPUs: their scaling_governor node is present but EBUSY, which
	# would abort under set -e (matters when a sibling is offlined by --on)
	for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
		local cpudir=${c%/cpufreq/scaling_governor}
		[ -f "$cpudir/online" ] && [ "$(cat "$cpudir/online")" = 0 ] && continue
		echo "$1" | sudo tee "$c" >/dev/null
	done
}

status() {
	echo "governor:     $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor) (cpu0)"
	echo -n "C-states on:  "
	for s in /sys/devices/system/cpu/cpu0/cpuidle/state*/; do
		[ "$(cat "$s/disable")" = 0 ] && echo -n "$(cat "$s/name") "
	done
	echo
	for irq in $(xhci_irqs); do
		echo "xhci IRQ $irq: affinity $(cat /proc/irq/$irq/smp_affinity)"
	done
	echo -n "loop core $LOOP_CPU sibling(s):"
	for s in $(target_siblings); do
		echo -n " cpu$s=$(cat /sys/devices/system/cpu/cpu$s/online 2>/dev/null || echo '?')"
	done
	echo " (0 = offlined, good)"
	echo -n "boot isolation:  isolcpus/nohz_full/rcu_nocbs "
	if grep -qE 'isolcpus|nohz_full|rcu_nocbs' /proc/cmdline; then
		echo "PRESENT"
	else
		echo "ABSENT (run --grub-hint for the persistent fix)"
	fi
}

on() {
	set_governor performance
	# disable any idle state whose exit latency is > 15 us (drops C3..C10,
	# keeps POLL/C1/C1E at <=10 us)
	for s in /sys/devices/system/cpu/cpu*/cpuidle/state*/; do
		lat=$(cat "$s/latency" 2>/dev/null || echo 0)
		if [ "$lat" -gt 15 ]; then echo 1 | sudo tee "$s/disable" >/dev/null; fi
	done
	m=$(irq_mask "$IRQ_CPU")
	for irq in $(xhci_irqs); do echo "$m" | sudo tee /proc/irq/$irq/smp_affinity >/dev/null; done
	# offline the loop core's HT sibling(s) so the physical core is ours alone.
	# record the full set first (union of live + any prior record), so a repeat
	# --on does not clobber it once the sibling is already offline, and --off can
	# always find what to bring back.
	local sibs
	sibs=$(target_siblings)
	printf '%s\n' $sibs | sudo tee "$STATE_FILE" >/dev/null
	for s in $sibs; do
		[ "$(cat /sys/devices/system/cpu/cpu$s/online)" = 1 ] || continue
		echo 0 | sudo tee /sys/devices/system/cpu/cpu$s/online >/dev/null
		echo "offlined sibling cpu$s (shares the physical core with cpu$LOOP_CPU)"
	done
	echo "perf mode ON"
	status
	echo
	echo "now launch the loop pinned + realtime:"
	echo "  sudo chrt -f 80 taskset -c $LOOP_CPU ./build/anyloop <config.json>"
}

off() {
	# bring back the sibling core(s) we offlined FIRST -- restoring governor /
	# C-states below must see them online (an offline CPU's cpufreq node is EBUSY).
	# Sources: STATE_FILE (an offline sibling is gone from thread_siblings_list) and
	# the live list, as a fallback if --on was never run this boot.
	for s in $(target_siblings); do
		[ "$(cat /sys/devices/system/cpu/cpu$s/online)" = 0 ] || continue
		echo 1 | sudo tee /sys/devices/system/cpu/cpu$s/online >/dev/null
		echo "re-onlined sibling cpu$s"
	done
	sudo rm -f "$STATE_FILE"
	set_governor "$OFF_GOVERNOR"
	# re-enable every idle state
	for s in /sys/devices/system/cpu/cpu*/cpuidle/state*/disable; do
		echo 0 | sudo tee "$s" >/dev/null
	done
	m=$(all_cpus_mask)
	for irq in $(xhci_irqs); do echo "$m" | sudo tee /proc/irq/$irq/smp_affinity >/dev/null; done
	echo "perf mode OFF (restored defaults)"
	status
}

grub_hint() {
	local sibs cores
	sibs=$(target_siblings | paste -sd,)
	cores="$LOOP_CPU${sibs:+,$sibs}"
	cat <<EOF
Boot-time isolation (persistent, survives reboot) -- NOT settable by this script.
Isolate the whole physical core: loop cpu $LOOP_CPU + sibling(s) $sibs.

1. Edit /etc/default/grub, add to GRUB_CMDLINE_LINUX_DEFAULT:
     isolcpus=$cores nohz_full=$cores rcu_nocbs=$cores
2. sudo update-grub    (Debian/Ubuntu) then reboot.
3. Verify:  cat /sys/devices/system/cpu/isolated   # should show $cores

With this in place cpu $cores leave the general scheduler entirely, so you no
longer need --on to offline the sibling -- but --on's governor / C-state / IRQ
tuning still applies. Launch unchanged:
   sudo chrt -f 80 taskset -c $LOOP_CPU ./build/anyloop <config.json>
EOF
}

case "${1:-}" in
	--on)        on ;;
	--off)       off ;;
	--status)    status ;;
	--grub-hint) grub_hint ;;
	*)
		sed -n '3,40p' "$0" | sed 's/^# \{0,1\}//'
		exit 1
		;;
esac
