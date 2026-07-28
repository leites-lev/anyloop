---
name: daqc2-uart-dac-project
description: "Ongoing project - custom EFM8 firmware for Pi-Plates DAQC2 to make a UART-to-analog converter, driven from this PC's serial port"
metadata: 
  node_type: memory
  type: project
  originSessionId: 2acbc6c6-772a-487d-ad3c-20b215d6c6f2
  modified: 2026-07-24T13:44:41.060Z
---

User is building a UART→analog output device by reflashing the Pi-Plates DAQC2plate (as of July 2026). Key facts:

- DAQC2's micro is an **EFM8LB12F64E-C-QFN32** (Silicon Labs Laser Bee): 4 internal 12-bit DACs (200 ksps/ch, 2.6 µs settling), 2 UARTs (UART1 has the fast baud generator, RX max SYSCLK/8 = 9 Mbaud @72 MHz), programmed via 2-wire C2 (C2CK=reset pin, C2D=P3.0). Flash lock cleared by device erase — stock firmware unrecoverable after erase (one-way door).
- Plan: erase + custom firmware; protocol chosen = 1 byte/frame: values 0–250 = sample (scaled), 251–254 = select channel 0–3, 255 = nop; PC resends channel-select every ~100 bytes for self-healing. Optional 4-bit-delta packing doubles rate.
- Flashing options: Arduino C2 flasher or Raspberry Pi GPIO bit-bang (c2prog); add UART bootloader (AN945) in custom firmware afterward.
- PC sender: direct-register RT sender at 0x3F8 (ioperm/iopl + outb, SCHED_FIFO) — benchmarked 11,483 B/s = 99.7% of 115200 wire limit. Test tools live in ~/uart-tests (rtuart, txdrain, directuart, sources included).
- CAUTION not yet resolved with user: PC COM port is RS-232 voltage levels (±V) — needs a MAX3232-class transceiver before the EFM8's 3.3 V UART pins. See [[optiplex-sushi-serial-port-quirks]].
- Alternative under consideration (July 2026): parallel PCIe card + SPI DAC route — see [[parallel-card-dac81404-plan]] (16-bit, 4ch, bipolar ranges, potentially much faster, no irreversible EFM8 erase).
