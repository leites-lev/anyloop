---
name: parallel-card-dac81404-plan
description: "Fast analog-out plan B: StarTech PEX1P2 (AX99100) parallel PCIe card bit-banging SPI to BP-DAC81404EVM, plus verified option to restrap the card into a hardware SPI master"
metadata: 
  node_type: memory
  type: project
  originSessionId: 074d0286-07a7-4d12-b074-dc4e8466407d
  modified: 2026-07-24T14:11:15.249Z
---

Alternative to the DAQC2 reflash (as of July 2026): StarTech PEX1P2 1-port parallel PCIe card driving a TI BP-DAC81404EVM (quad 16-bit DAC, ±5/±10/±20 V ranges, 50 MHz SPI) over direct wires. Card **not yet installed** in sushi as of 2026-07-24. User has all power supplies covered.

**Card facts (verified from AX99100 datasheet, July 2026):**
- PEX1P2 uses ASIX AX99100. Chip personality is set by strap resistors on pins 54/56/58 (8 modes). Parallel port exists only in modes 2S1P(001)/2MP1P(010); SPI master only in 2S1SPI(110)/2MP1SPI(100) — mutually exclusive, same package pins (Port 4). In parallel modes, PCI function 3 (SPI) doesn't enumerate at all — not software-enableable.
- Registers are available via **memory BARs**, so no outb needed: mmap `/sys/bus/pci/devices/.../resourceN` and use plain stores. Posted MMIO writes ~100–300 ns vs ~1–2 µs non-posted I/O writes.

**Plan A (unmodified card): MMIO bit-bang SPI**
- SCLK/SDIN/SYNC on data pins D0–D2 + GND; optional SDO readback on ACK (DB25 pin 10). ~50 writes per 24-bit frame → est. 60–200 kS/s.
- Needs: DB25 male breakout (card has no internal header), dupont wires, 33–100 Ω series R on SCLK, leads <20 cm. EVM IOVDD = 3.3 V (AX99100 is 3.3 V logic — verify swing with meter before connecting; 5 V would violate DAC abs-max at 3.3 V IOVDD). LDAC tied low, CLR inactive.
- Software: unbind parport_pc, find BARs via lspci -v, reuse ioperm-era RT recipe (SCHED_FIFO + /dev/cpu_dma_latency) from ~/uart-tests. DAC81404 powers up with outputs off — init must write range-select + power-up registers.

**Plan B (restrap card to hardware SPI, verified in datasheet):**
- Move strap pull-ups 2S1P(001)→2S1SPI(110): fine-pitch resistor rework, forfeits parallel port. SPI master then: 42 MHz SCLK, modes 0–3, SS[2:0], TX DMA up to 64 KB with "fragmentation" = automatic per-frame CS framing (ideal for streaming 24-bit DAC frames). One write/DMA per sample; bottleneck becomes the DAC (~settling-bound, up to ~1 MS/s region).
- SPI lands on former status pins at the DB25: pin 13 (SELECT)→SCLK, 12 (PAPEREND)→MOSI, 11 (BUSY)→MISO, 10 (ACK)→SS0, 16 (INIT)→SS1. Those were inputs — card PCB may have pull-ups/RC on them, so run SCLK at a few MHz. SCLK needs external pull-down for SPI mode 0. ASIX ships a Linux ax99100 SPI driver; or drive BAR1/BAR5 MMIO directly.
- Datasheet copy saved during session; source: sc19.gongkong.com AX99100 PDF (also on asix.com.tw).

Trade-off vs [[daqc2-uart-dac-project]]: parallel-card route gives 16-bit, 4ch, bipolar HV ranges, no irreversible EFM8 erase, and (Plan B) far higher rates; UART route caps at ~11.5 kS/s 12-bit. See [[optiplex-sushi-serial-port-quirks]] for the machine's ioperm/lockdown status (mmap path already permitted).

**STATUS 2026-07-24: Plan B (restrap) deprioritized.** Use case turned out to be a closed loop fed by a 0.3 ms camera, so the whole DAC side is ~4% of the loop budget and the restrap buys ~2% end-to-end — not worth the fine-pitch rework, the forfeited parallel port, or the unresolved risk that the DB25 status pins sit behind unidirectional buffers. Superseded by [[pico-parallel-to-spi-dac-bridge]], which reaches the same latency for ~$5 with no rework. Plan A's MMIO bit-bang remains the fallback/baseline.
