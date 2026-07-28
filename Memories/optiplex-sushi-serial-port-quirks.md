---
name: optiplex-sushi-serial-port-quirks
description: "This PC (sushi, OptiPlex 7060) hardware quirks - serial IRQ permanently dead, direct-register access required; Secure Boot state flips and decides whether ioperm/PCI BAR mmap work at all (off again 2026-07-27)"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 2acbc6c6-772a-487d-ad3c-20b215d6c6f2
  modified: 2026-07-27T15:00:00.000Z
---

Machine "sushi" = Dell OptiPlex 7060 (i7-8700, Debian, kernel 6.19). Serial findings (verified July 2026):

- **ttyS0 (0x3F8)**: real 16550A in Dell's SCH555x-family EC-SuperIO (device ID 0xC9 rev A1 at config port 0x2E, SMSC 0x55/0xAA unlock). Data path works perfectly (99.7% wire limit, 115200 max, baud_base 115200, no clock multiplier found). **IRQ delivery is permanently broken**: EC never forwards serial IRQs to host on any line (scanned 3–11); UART LDN7 reg 0x0F ("config select", coreboot writes 0x02) is hard read-only; BIOS Serial1 setting already Com1. Unfixable in software.
- Usable modes: (1) driver polled mode `irq=0` via TIOCSSERIAL — works both directions but ~331 B/s, ~48 ms RTT, resets at reboot; (2) **direct register access** (ioperm/iopl + outb at RT priority) — full 11,483 B/s, ~87 µs/byte, the path for real-time use.
- **ttyS1 is Intel AMT Serial-over-LAN** (PCI 00:16.3), no physical connector; its 16550 clone never fires the RX character-timeout interrupt (needs rx_trig_bytes=1 for small reads). Don't use for hardware.
- **Secure Boot decides whether any user-space register access works, and its state HAS FLIPPED before** — found ENABLED on 2026-07-27 (it had been disabled earlier in July), user disabled it again the same day. Under Secure Boot, lockdown=integrity: root can neither `ioperm`/`outb` nor mmap a PCI BAR through sysfs (mmap returns EPERM, dmesg logs "Lockdown: direct PCI access is restricted"), which kills the mmio fast path of anyloop's parport_dac and everything in [[parallel-card-dac81404-plan]]. **Read `/sys/kernel/security/lockdown` carefully: the BRACKETED word is the ACTIVE mode**, so `none [integrity] confidentiality` means locked down and `[none] integrity confidentiality` means free — trivially misread. Verify with `mokutil --sb-state`. It cannot be lowered at runtime; fix is F2 → Secure Boot → Disable. No LUKS anywhere; old Windows/BitLocker boot entry is stale (NVMe removed). apt has pre-existing broken dependencies (superiotool uninstallable; built from coreboot source instead).
- SCHED_FIFO wakeup jitter measured: p99 3.8 µs under load, max 6.8 µs; idle spikes ~32 µs from C-states (cap via /dev/cpu_dma_latency).
- Test tools in ~/uart-tests. Related: [[daqc2-uart-dac-project]].
