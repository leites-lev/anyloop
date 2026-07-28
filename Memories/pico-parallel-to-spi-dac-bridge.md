---
name: pico-parallel-to-spi-dac-bridge
description: "Chosen architecture (July 2026) for the closed-loop analog output: PC parallel port -> RP2040/RP2350 PIO -> SPI -> DAC81404, replacing the card-restrap plan"
metadata: 
  node_type: memory
  type: project
  originSessionId: 76754de2-ade7-489f-a895-526087cc51b7
  modified: 2026-07-24T14:11:06.907Z
---

Selected approach as of 2026-07-24 for driving the BP-DAC81404EVM from sushi: **PC → PCIe parallel card (DB25) → RP2040/RP2350 PIO → hardware SPI → DAC81404**. Supersedes the restrap-to-SPI idea in [[parallel-card-dac81404-plan]] (that plan's Plan B is deprioritized, not deleted — the DB25 continuity check is still its gate).

**The controlling constraint: this is a closed loop fed by a 0.3 ms latency camera.** PC computes every sample. Loop budget ≈ 300 µs camera + ~7.5 µs DAC write + ~5 µs settling ≈ 312 µs, so the entire DAC side is ~4% and any transport optimization is worth ≤2% end-to-end. This is why every purchase option below was rejected. **Jitter, not mean latency, is the real limit** — constant delay can be compensated with feedforward; the ~32 µs C-state spikes and 6.8 µs SCHED_FIFO jitter (see [[optiplex-sushi-serial-port-quirks]]) dominate everything on the DAC side.

**Wire protocol (4 MMIO writes per 16-bit sample, ~600 ns):**
```
write data port (8 data pins) = low byte
write ctrl port              = clk high   ← PIO samples rising edge
write data port              = high byte
write ctrl port              = clk low    ← PIO samples falling edge
```
DB25 gives 8 data + 4 control outputs; 3 control lines spare for channel select / frame-valid. Latency ≈ 0.6 µs (PC) + tens of ns (PIO capture) + ~0.5 µs (24-bit SPI frame @ 50 MHz) ≈ **1.5–2.5 µs**, vs 7.5 µs for direct MMIO bit-bang of SPI. RP2040 PIO tops out ~62.5 MHz (sysclk/2); DAC81404 max is 50 MHz.

**Two things that must be right:**
- **RP2040 GPIO is NOT 5 V tolerant.** AX99100 is a 3.3 V part but parallel cards often add 5 V buffers for IEEE-1284. Meter the DB25 data pins before connecting; if 5 V, add ~1–3.3 kΩ series resistors or a 74LVC245.
- **Do the conversion in PIO + DMA, never an ISR.** An interrupt handler adds variable µs turnaround; PIO/DMA is deterministic to the clock cycle. Jitter is the thing that actually matters here.

Side benefits: Pico owns the DAC81404 init sequence (range-select + power-up registers) so the PC only streams samples; precise LDAC timing for synchronized updates; PC critical section shrinks 7.5 µs → 600 ns, a 12× smaller window for an SMI/preemption to land mid-transaction. Prefer **RP2350 / Pico 2** (~$5, 150 MHz, improved PIO).

**Dead ends already researched — do not re-search:**
- No commercial AX99100 card ships in 2S1SPI mode (confirmed on ASIX's own support forum). ASIX publishes schematics/PCB/Gerbers for a 2S1SPI demo board, so self-fab is possible (~$100–200 + QFN assembly).
- AX99100A-DMB-2SSPI-1 demo board (SPI master to 41.6 MHz): 0 stock, 30-week lead, no public price, North America only, no Octopart distributors. Contact Sales@asix.com.tw if ever revisited.
- Xdimax DMX-10 PCIe I2C/SPI/GPIO adapter: back-order, price on request only, page frozen at "Copyright 2002-2012", and SPI caps at **10 Mbps** anyway.
- Parallel-interface DAC EVMs matching DAC81404 specs under $60 do not exist. DAC8728EVM obsolete; DAC8718EVM $178.80; DAC8734EVM is serial + no stock; DAC8820EVM single-channel multiplying. All want a **16-bit** bus, which DB25's 8 data pins cannot drive without latches. Only LTC1599 has a native byte-wide 8-bit bus, but it's single-channel current-output needing external op-amps.
- Parallel DACs are legacy because serial outran the analog settling time; modern low-latency answer is fast QSPI/DDR (e.g. ADI AD3552R), and GSPS parts use JESD204B/C.
