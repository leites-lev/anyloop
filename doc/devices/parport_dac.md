anyloop:parport_dac
===================

Types and units: `[T_VECTOR, U_MINMAX] -> [T_UNCHANGED, U_UNCHANGED]`.

Drives a TI DAC81404 (quad 16-bit, high-voltage output) through a PCIe
parallel-port card — the fast replacement for the `piplate_bridge` /
DAQC2 serial DAC path, whose ~0.5–0.7 ms per `setDAC` was a large share of the
loop's transport delay.

Like `piplate_bridge`, one stage can drive several channels: pass arrays for
`channel`/`index`/`scale`/`offset`/`range`/`start_delay` (any scalar is
broadcast to every channel), and each channel's output voltage is
`offset + cmd*scale`, where `cmd` is the pipeline vector element named by
`index`.

Hardware
--------

The transport is a StarTech PEX1P2 (ASIX AX99100) PCIe parallel card, seen on
this machine as PCI function `0000:05:00.2`. Its registers are available
through *memory* BARs, so a pin change is a plain store to mapped memory
(~100–300 ns, posted) rather than an `outb` (~1–2 µs, non-posted).

Two ways to get from the DB25 to the DAC, chosen with `link`:

  - `"spi"` (default): the DB25 data pins *are* the SPI bus — D0 = SCLK,
    D1 = SDIN, D2 = SYNC by default, GND on pins 18–25. The 24-bit frames are
    bit-banged here: two stores per bit plus the frame brackets. **Measured
    2026-07-27: 156 k frames/s = 6.4 µs per channel update** (~123 ns per
    store, effective SCLK ≈ 3.75 MHz, well under the DAC's 50 MHz ceiling), so
    a two-channel update costs ~12.8 µs against the BRIDGEplate's ~0.5–1.4 ms.
    Needs nothing but wires between the card and the BP-DAC81404EVM.
  - `"pico"`: the DB25 carries a byte-parallel handshake to an RP2040/RP2350
    whose PIO runs the SPI at up to 50 MHz. Four stores per sample (~0.6 µs):
    low byte, clock high, high byte, clock low, with the channel number on
    spare control lines. The Pico owns the DAC's init and LDAC timing. Needs
    the bridge firmware.

**Levels — measured 2026-07-27, and the two pin groups are NOT the same.**

  - **Data pins D0–D7 (DB25 2–9): 3.3 V.** Driven high they measure 3.3 V,
    unbuffered, idle pins 0 V. Safe to wire straight to the EVM (IOVDD 3.3 V,
    V_IH = 0.7 × IOVDD = 2.31 V, so ~1 V of margin) or to an RP2040/RP2350.
  - **Control pins (DB25 1, 14, 16, 17): 4.94 V.** These *are* buffered to
    IEEE-1284 5 V levels. **Do not connect them to the DAC or to a Pico
    without level shifting** — 5 V exceeds the DAC81404's absolute maximum at
    3.3 V IOVDD, and RP2040/RP2350 GPIOs are not 5 V tolerant. This is the
    constraint on the `pico` link, whose clock and channel-select lines live
    on control pins: it needs a divider, series resistors, or a 74LVC245 on
    those four lines even though the data byte is fine as-is. The same applies
    to putting a hardware LDAC line on a control pin.

Keep leads under ~20 cm and put 33–100 Ω in series with SCLK for edge quality.

Pinout (measure against ground on pins 18–25):

| signal | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
|---|---|---|---|---|---|---|---|---|
| DB25 pin | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |

DB25 pin 1 is `/STROBE`, a *control* line, not D0 — the control lines are
pins 1 (STROBE), 14 (AUTOFD), 16 (INIT) and 17 (SELECT), and all but INIT are
inverted between the register bit and the pin. This device takes control
values in pin polarity and folds the inversion in for you.

Wiring to the BP-DAC81404EVM
---------------------------

From SLAU825 Figure 16 (schematic, p. 19) — note the signals are split across
**both** BoosterPack receptacles, and J4's physical pin numbers run opposite to
the BoosterPack numbering, which makes the Figure 7 pinout diagram easy to
misread:

| DB25 | signal | connector | physical pin | BoosterPack pin | net |
|---|---|---|---|---|---|
| 2 (D0) | SCLK | **J1** | **13** | 7 | `DAC_SCLK` |
| 3 (D1) | SDIN | **J4** | **12** | 15 | `DAC_SDIN` |
| 4 (D2) | SYNC | **J4** | **18** | 12 | `DAC_SYNCZ` |
| 18–25 | GND | **J4** | **2** | 20 | `DGND` |
| 10 (ACK) | SDO, optional | **J4** | **14** | 14 | `DAC_SDO` |

Count physical pins: pin 1 is the square pad, then double-row order (1,2 across
the top, 3,4 next), odd down one column, even down the other. Check J4 pin 2
reads continuity to ground before powering anything. Because SCLK sits on the
opposite edge from SDIN/SYNC, take a ground at **each** connector — J4 pin 2
and J1 pin 4 (`DGND`, BoosterPack 22) — so SCLK's return doesn't have to cross
the board.

Jumpers: the defaults are already right (J2 1–2 = LDAC low, suiting the
asynchronous update this device uses; J3 none = CLR inactive; J16 none =
internal reference, which init powers up; J11 2–3 = unipolar). **J19 is the one
to think about**: its 2–3 default draws DAC IOVDD from the LaunchPad connector,
so with no LaunchPad either feed 3.3 V into J1 physical pin 1 (`AEVM_3V3`) or
move J19 to 1–2 and feed terminal block J20. It must be 3.3 V to match the
card's data-pin levels.

Supplies land on terminal block J17: pin 1 DAC_VDD (0–5.5 V), pins 2/3 GND,
pin 4 DAC_AVDD (≤41.5 V, needs headroom over the output range), pin 5 DAC_AVSS
(ground it in unipolar mode). AVDD − AVSS must not exceed 42 V. Outputs are on
the four 5-pin terminal blocks silkscreened `DAC_VOUT_A`…`DAC_VOUT_D`, each
with force and sense.

The EVM already carries 33 Ω in series on SCLK/SDIN/SYNC at its own end (R18,
R27, R28). That damps ringing into the DAC input but is not source termination
for a long cable, so it does not replace resistors at the DB25 end.

**Privileges and kernel lockdown.** The `mmio` backend needs `CAP_SYS_RAWIO`
(run anyloop as root) and needs `parport_pc` off the device, which `unbind`
does by default. Rebind afterwards with

```
echo 0000:05:00.2 | sudo tee /sys/bus/pci/drivers/parport_pc/bind
```

It also needs kernel lockdown off. With Secure Boot **on**, lockdown is
`integrity` and mapping the BAR fails with `EPERM` even as root (`Lockdown:
direct PCI access is restricted` in `dmesg`); lockdown blocks `ioperm`/`outb`
the same way, leaving no user-space fast path at all. Secure Boot was disabled
on this machine on 2026-07-27, so `cat /sys/kernel/security/lockdown` now
reads `[none] integrity confidentiality` — the *bracketed* word is the active
mode, which is easy to misread.

The `ppdev` backend works under lockdown and without root (given `lp` group
access to `/dev/parport0`), but every edge becomes an ioctl. Measured on this
machine: **~4 µs per edge, ~200 µs per 24-bit frame, ~4.9 k frames/s**, so a
two-channel update costs ~0.4 ms — about what the BRIDGEplate cost. Use it to
check wiring and frames, not to close a loop.

**DAC power-up.** The DAC81404 comes out of reset with its reference and all
four output amplifiers powered down (outputs clamped to ground through 10 kΩ).
In the `spi` link, init writes `GENCONFIG` (reference on), `DACRANGE` (the
configured ranges) and `DACPWDWN` (channels on), then parks every channel at
its `offset` before the loop starts.

Parameters
----------

### Transport

  - `backend` (string) (optional)
    - `"mmio"` (default) or `"ppdev"`.
  - `pci` (string) (optional)
    - PCI slot of the parallel function (default `"0000:05:00.2"`). Find it
      with `lspci -d 125b: -nn`.
  - `bar` (integer) (optional)
    - Memory BAR index holding the parallel port's registers (default 2).
  - `data_offset`, `status_offset`, `ctrl_offset` (integer) (optional)
    - Register offsets inside `bar`, defaults `0x280`, `0x284`, `0x288`.
      The AX99100 does **not** expose the byte-packed ISA layout through its
      memory BAR: the block is dword-spaced at +0x280, mirrored again at
      +0x2c0, with the whole 0x400 window repeating through the 4 kB page.
      Mapped and verified on this card on 2026-07-27 by writing through the
      memory window and reading the I/O window at 0x3010, and vice versa.
  - `ecr_bar`, `ecr_offset`, `ecr_value` (integer) (optional)
    - Where the ECP ECR lives — defaults 2 and `0x2a8`, i.e. the *same* BAR as
      the other registers (BAR5 holds the AX99100's own configuration
      registers and does not mirror the ECR) — and what to write to it
      (default `0x14`: SPP/compatibility mode, interrupts off) so the data
      pins are plain outputs. Set `ecr_bar` to -1 to leave the ECR alone.
  - `unbind` (boolean) (optional)
    - Unbind `parport_pc` from `pci` before mapping (default true).
  - `probe` (boolean) (optional)
    - Read the data register back at init and check it holds what was written
      (default true). Catches a wrong `bar`/`data_offset`, which otherwise
      looks like a DAC that silently ignores every frame. Only the idle
      pattern is written, so an attached DAC sees no edges.
  - `port` (string) (optional)
    - ppdev node (default `"/dev/parport0"`).

### Link

  - `link` (string) (optional)
    - `"spi"` (default) or `"pico"`.
  - `sclk_bit`, `sdin_bit`, `sync_bit` (integer) (optional)
    - Data pins (0–7) carrying the SPI signals in the `spi` link
      (defaults 0, 1, 2 = DB25 pins 2, 3, 4).
  - `clk_ctrl_bit` (integer) (optional)
    - Control line clocking bytes into the bridge in the `pico` link:
      0 = STROBE (pin 1), 1 = AUTOFD (14), 2 = INIT (16), 3 = SELECT (17).
      Default 0.
  - `chan_ctrl_shift` (integer) (optional)
    - First of the two control lines carrying the channel number in the
      `pico` link (default 1, i.e. AUTOFD + INIT).
  - `delay_ns` (integer) (optional)
    - Extra dwell after each edge, in nanoseconds (default 0). The store
      latency alone already exceeds the DAC's setup requirement; raise this
      only for long or unterminated wiring.

### Outputs

  - `channel` (integer or array) (optional)
    - DAC channel(s): 0 = DACA, 1 = DACB, 2 = DACC, 3 = DACD (default 0).
  - `index` (integer or array) (optional)
    - Pipeline vector element(s) to command (default 0).
  - `scale` (number or array) (optional)
    - Volts per pipeline unit (default 1.0).
  - `offset` (number or array) (optional)
    - Volts at zero command (default 0.0).
  - `range` (string or array) (optional)
    - Output range per channel (default `"0-10"`): `"0-5"`, `"0-6"`,
      `"0-10"`, `"0-12"`, `"0-20"`, `"0-24"`, `"0-40"`, `"+-5"`, `"+-6"`,
      `"+-10"`, `"+-12"`, `"+-20"`. Codes are MSB-aligned straight binary
      across the selected range.
  - `start_delay` (number or array) (optional)
    - Seconds to hold a channel at its `offset` before letting the pipeline
      command through (default 0). Same purpose as in `piplate_bridge`: give
      a coarse stage time to walk the beam near centre. Give the upstream
      `pid` a matching `start_delay`, or its integrator winds up during the
      hold.
  - `skip_unchanged` (boolean) (optional)
    - Only send a channel when its 16-bit code changes (default false).
  - `reset_at_init` (boolean) (optional)
    - Issue a DAC soft reset before configuring (default true; `spi` link
      only).

Example
-------

Two fine channels, ±2 V about a 2 V bias on a 0–10 V range, held at bias for
the first 30 s while a coarse stage settles:

```json
{
 "uri": "anyloop:parport_dac",
 "params": {
  "pci": "0000:05:00.2",
  "link": "spi",
  "channel": [0, 1],
  "index": [0, 1],
  "range": "0-10",
  "scale": [2, -2],
  "offset": [2, 2],
  "start_delay": [30, 30]
 }
}
```

`contrib/conf_parport_dac_smoke.json` is a standalone bench check: a 0.2 Hz
sine into DACA/DACB with no camera and no mirror.
