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
#   3. park EVERY movable device IRQ off the loop core + its sibling, then pin
#      the xhci USB IRQ(s) specifically to CPU $IRQ_CPU. Added 2026-08-14: the
#      parport DAC write is ~52 bit-banged MMIO stores held inline on the loop
#      thread (~14-57 us per frame depending on parport_dac's delay_ns), so any
#      unrelated IRQ landing on the loop core stretches a frame. Pinning only
#      xhci left every other device free to land there.
#   4. OFFLINE the loop core's hyperthread SIBLING so nothing contends for the
#      shared execution units mid-measurement (the runtime equivalent of
#      isolating the physical core; taskset+chrt alone leave the sibling live)
#
# NOTE the parallel card itself has no IRQ to pin: anyloop unbinds parport_pc
# and bit-bangs the BAR directly, keeping port interrupts masked, so its PCI
# IRQ line never has a handler. There is no separate parport thread either --
# "giving the parport its own core" means making the LOOP core exclusive, which
# is what steps 3 and 4 do.
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
#   contrib/camera_pcie_hardware/perf_mode.sh --on          # apply the runtime tuning
#   contrib/camera_pcie_hardware/perf_mode.sh --off         # revert to defaults
#   contrib/camera_pcie_hardware/perf_mode.sh --status      # show current state
#   contrib/camera_pcie_hardware/perf_mode.sh --grub-hint   # print the boot-param line to add
#
# Then launch the loop pinned + realtime on the reserved core:
#   sudo chrt -f 80 taskset -c 2 ./build/anyloop \
#     contrib/steering/configurations/steering_par_fsp.json
#
# Uses sudo for the privileged writes, so run it as your normal user.

set -euo pipefail

LOOP_CPU=2          # core you should taskset the loop onto (kept clear of the IRQ)
IRQ_CPU=3           # park the xhci USB IRQ here, off the loop core + sibling
OFF_GOVERNOR=powersave   # this box's default; change if yours differs
STATE_FILE=/run/perf_mode.offlined   # sibling CPUs we offlined, so --off can restore them
IRQ_STATE_FILE=/run/perf_mode.irqaffinity  # "irq oldmask" lines, so --off can restore them

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

# Every CPU the kernel knows about, online or not. nproc counts only ONLINE
# CPUs, so it shrinks once --on offlines the sibling -- using it to build an
# "all CPUs" mask would silently drop the top core from the mask.
possible_cpus() { awk -F- '{print $NF + 1}' /sys/devices/system/cpu/possible; }
possible_mask() { printf '%x' $(( (1 << $(possible_cpus)) - 1 )); }

# The loop's physical core: LOOP_CPU plus its hyperthread sibling(s).
loop_core_mask() {
	local m=$(( 1 << LOOP_CPU )) s
	for s in $(target_siblings); do m=$(( m | (1 << s) )); done
	printf '%x' "$m"
}

# Everything EXCEPT the loop's physical core -- where all movable device IRQs
# get parked, so no unrelated interrupt can land on the loop core mid-frame.
# The parport DAC write is a bit-banged sequence of ~52 MMIO stores held
# inline on the loop thread; an IRQ landing in the middle stretches the frame.
# (SPI is synchronous, so a stretched frame is not corrupted -- this is about
# loop jitter, not about the DAC missing writes.)
irq_isolate_mask() {
	printf '%x' $(( 0x$(possible_mask) & ~0x$(loop_core_mask) ))
}

# Park every IRQ that will accept it. Per-CPU and chained interrupts (timer,
# IPIs, some managed MSI queues) reject affinity writes with EIO or EROFS --
# that is expected and not an error, so failures are counted, not fatal.
isolate_irqs() {
	local mask=$1 irq old moved=0 pinned=0
	: | sudo tee "$IRQ_STATE_FILE" >/dev/null
	for irq in /proc/irq/[0-9]*; do
		[ -f "$irq/smp_affinity" ] || continue
		old=$(cat "$irq/smp_affinity" 2>/dev/null) || continue
		if echo "$mask" | sudo tee "$irq/smp_affinity" >/dev/null 2>&1; then
			echo "${irq##*/} $old" | sudo tee -a "$IRQ_STATE_FILE" >/dev/null
			moved=$((moved + 1))
		else
			pinned=$((pinned + 1))
		fi
	done
	echo "parked $moved IRQ(s) off the loop core (mask $mask); $pinned are per-CPU and cannot move"
	# new IRQs registered later inherit this
	echo "$mask" | sudo tee /proc/irq/default_smp_affinity >/dev/null 2>&1 || true
}

restore_irqs() {
	[ -f "$IRQ_STATE_FILE" ] || return 0
	local irq old n=0
	while read -r irq old; do
		[ -n "${irq:-}" ] || continue
		echo "$old" | sudo tee "/proc/irq/$irq/smp_affinity" >/dev/null 2>&1 || true
		n=$((n + 1))
	done < "$IRQ_STATE_FILE"
	sudo rm -f "$IRQ_STATE_FILE"
	echo "$(all_cpus_mask)" | sudo tee /proc/irq/default_smp_affinity >/dev/null 2>&1 || true
	echo "restored affinity on $n IRQ(s)"
}

# How many IRQs could still fire on the loop core -- 0 is the goal.
irqs_on_loop_core() {
	local irq aff n=0 lm
	lm=0x$(loop_core_mask)
	for irq in /proc/irq/[0-9]*; do
		[ -f "$irq/smp_affinity" ] || continue
		aff=$(tr -d ',' < "$irq/smp_affinity" 2>/dev/null) || continue
		[ -n "$aff" ] || continue
		if (( 0x$aff & lm )); then n=$((n + 1)); fi
	done
	echo "$n"
}

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
	echo "IRQs on loop core: $(irqs_on_loop_core) still able to fire on cpu$LOOP_CPU/sibling (0 = good)"
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
	# Park EVERY movable IRQ off the loop's physical core first, then put the
	# xhci one specifically on IRQ_CPU. Order matters: the blanket pass would
	# otherwise overwrite the xhci pin with the generic mask.
	isolate_irqs "$(irq_isolate_mask)"
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
	restore_irqs
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
