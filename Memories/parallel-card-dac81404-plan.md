---
name: parallel-card-dac81404-plan
description: "Fast analog-out plan B: StarTech PEX1P2 (AX99100) parallel PCIe card bit-banging SPI to BP-DAC81404EVM, plus verified option to restrap the card into a hardware SPI master"
metadata: 
  node_type: memory
  type: project
  originSessionId: 074d0286-07a7-4d12-b074-dc4e8466407d
  modified: 2026-07-24T14:11:15.249Z
---

Alternative to the DAQC2 reflash (as of July 2026): StarTech PEX1P2 1-port parallel PCIe card driving a TI BP-DAC81404EVM (quad 16-bit DAC, ±5/±10/±20 V ranges, 50 MHz SPI) over direct wires. **Card installed and the DAC is working as of 2026-07-29** — Plan A (MMIO bit-bang) drives real voltage out of `DAC_VOUT_A`, tracking the commanded value. User has all power supplies covered.

**Card facts (verified from AX99100 datasheet, July 2026):**
- PEX1P2 uses ASIX AX99100. Chip personality is set by strap resistors on pins 54/56/58 (8 modes). Parallel port exists only in modes 2S1P(001)/2MP1P(010); SPI master only in 2S1SPI(110)/2MP1SPI(100) — mutually exclusive, same package pins (Port 4). In parallel modes, PCI function 3 (SPI) doesn't enumerate at all — not software-enableable.
- Registers are available via **memory BARs**, so no outb needed: mmap `/sys/bus/pci/devices/.../resourceN` and use plain stores. Posted MMIO writes ~100–300 ns vs ~1–2 µs non-posted I/O writes.

**Plan A (unmodified card): MMIO bit-bang SPI**
- SCLK/SDIN/SYNC on data pins D0–D2 + GND; optional SDO readback on ACK (DB25 pin 10). ~50 writes per 24-bit frame → est. 60–200 kS/s.
- Needs: DB25 male breakout (card has no internal header), dupont wires, 33–100 Ω series R on SCLK, leads <20 cm. EVM IOVDD = 3.3 V (AX99100 is 3.3 V logic — verify swing with meter before connecting; 5 V would violate DAC abs-max at 3.3 V IOVDD). (Note added 2026-07-28: the chip's own datasheet names these pins SYNC/LDAC, no "Z" — the EVM schematic's `DAC_SYNCZ` net label is the board's own active-low notation, not the chip pin name.)
- **LDAC (pin 13), CLR (pin 16) and RST (pin 32) all sit at IOVDD — not ground, not open** (corrected 2026-07-28 from the earlier "LDAC tied low, CLR inactive"). **As built 2026-07-29: no wires on any of them; the EVM's own 10 kΩ pull-ups hold all three at 3.3 V, and J2 is removed so LDAC is not pulled to ground.** So the wanted state is reached by the board, not by added wiring — nothing to fit, just don't refit J2/J3 or disturb the pull-ups. SLASEH2A Table 6-1 says "Connect to IOVDD if unused" for LDAC and CLR; it says nothing for RST, but all three are active-low inputs with no documented *internal* pull-up, which is why those external 10 kΩ are load-bearing. LDAC low is only harmless while every channel is in asynchronous mode. CLR: a glitch low clears buffer *and* active registers on all four channels to zero/midscale (8.3.3.3), and `skip_unchanged` hides it because the driver only re-sends on a code change. RST is worst: tRSTW is 20 ns and a POR restores every register default (device powered down, reference off, ranges 0-5 V, FSDO 0), which the driver never re-checks — every frame after that is silently ignored until anyloop restarts. No hardware reset line is needed since `reset_at_init` does a SOFT-RESET via TRIGGER. Re-meter all three at ≈3.3 V after any rework — the EVM jumper wiring is in SLAU825, which is not saved locally.
- **DB25 as built:** D0→SCLK, D1→SDIN, D2→SYNC (EVM net `DAC_SYNCZ`), GND on 18–25 — the driver's `pin_sclk`/`pin_sdin`/`pin_sync` defaults 0/1/2, so no overrides in config. The breakout's screw terminals have plastic covers on them; keep them on, since the 4.94 V control pins sit right beside the 3.3 V data pins that run to the DAC.
- Software: unbind parport_pc, find BARs via lspci -v, reuse ioperm-era RT recipe (SCHED_FIFO + /dev/cpu_dma_latency) from ~/uart-tests. DAC81404 powers up with outputs off — init must write range-select + power-up registers.

**Plan B (restrap card to hardware SPI, verified in datasheet):**
- Move strap pull-ups 2S1P(001)→2S1SPI(110): fine-pitch resistor rework, forfeits parallel port. SPI master then: 42 MHz SCLK, modes 0–3, SS[2:0], TX DMA up to 64 KB with "fragmentation" = automatic per-frame CS framing (ideal for streaming 24-bit DAC frames). One write/DMA per sample; bottleneck becomes the DAC (~settling-bound, up to ~1 MS/s region).
- SPI lands on former status pins at the DB25: pin 13 (SELECT)→SCLK, 12 (PAPEREND)→MOSI, 11 (BUSY)→MISO, 10 (ACK)→SS0, 16 (INIT)→SS1. Those were inputs — card PCB may have pull-ups/RC on them, so run SCLK at a few MHz. SCLK needs external pull-down for SPI mode 0. ASIX ships a Linux ax99100 SPI driver; or drive BAR1/BAR5 MMIO directly.
- Datasheet copy saved during session; source: sc19.gongkong.com AX99100 PDF (also on asix.com.tw).

Trade-off vs [[daqc2-uart-dac-project]]: parallel-card route gives 16-bit, 4ch, bipolar HV ranges, no irreversible EFM8 erase, and (Plan B) far higher rates; UART route caps at ~11.5 kS/s 12-bit. See [[optiplex-sushi-serial-port-quirks]] for the machine's ioperm/lockdown status (mmap path already permitted).

**STATUS 2026-07-24: Plan B (restrap) deprioritized.** Use case turned out to be a closed loop fed by a 0.3 ms camera, so the whole DAC side is ~4% of the loop budget and the restrap buys ~2% end-to-end — not worth the fine-pitch rework, the forfeited parallel port, or the unresolved risk that the DB25 status pins sit behind unidirectional buffers. Superseded by [[pico-parallel-to-spi-dac-bridge]], which reaches the same latency for ~$5 with no rework. Plan A's MMIO bit-bang remains the fallback/baseline.

**STATUS 2026-07-29: Plan A works — the DAC outputs voltage.** Bit-banged SPI over the DB25 data pins now produces a real, correct analog output, not just clean bus timing. The missing piece had been `SPICONFIG.DEV-PWDWN`: the part comes out of reset device-wide powered down, so every earlier session that measured only frames/s could have been clocking into a dead device. Measure a *voltage*, never a frame rate, when judging this path. Driver and wiring notes live in `anyloop/doc/devices/parport_dac.md` and the header of `anyloop/devices/parport_dac.c`; smoke config is `contrib/conf_parport_dac_2v.json`. With Plan A delivering, the Pico bridge is no longer on the critical path.
