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
    D1 = SDIN, D2 = SYNC by default, GND on pins 18–25. SYNC (frame bracket)
    and LDAC (load strobe, hardwired low — see Jumpers below) are both
    active-low pins, per the DAC81404's own datasheet naming (no "Z"
    suffix — see the pinout table's footnote). The 24-bit frames are
    bit-banged here: two stores per bit plus the frame
    brackets. **Measured
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
| 4 (D2) | SYNC | **J4** | **18** | 12 | `DAC_SYNCZ`(1) |
| 18–25 | GND | **J4** | **2** | 20 | `DGND` |
| 10 (ACK) | SDO, optional | **J4** | **14** | 14 | `DAC_SDO` |

(1) The "signal" column here is the chip's own pin name, straight from the
DAC81404 datasheet (SLASEH2A) — confirmed 2026-07-28 against a local copy,
`dac81404.pdf`: the pin table literally reads "SDO SCLK SDIN SYNC LDAC GND
IOVDD CLR", no "Z" on SYNC or LDAC anywhere. The "net" column is the
BP-DAC81404EVM board's *own* schematic net name (SLAU825), which is the EVM
designer's active-low notation and doesn't have to match the chip's pin
name — it doesn't, here. Don't take `DAC_SYNCZ` as evidence the pin itself
is called SYNCZ; the datasheet is the authority for that.

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

**DAC power-up.** The DAC81404 comes out of reset *device-wide* powered down
(`SPICONFIG.DEV-PWDWN`), separately from its reference and all four output
amplifiers also being powered down (outputs clamped to ground through 10 kΩ).
Found 2026-07-28 reading a local copy of the datasheet: nothing here used to
clear `SPICONFIG.DEV-PWDWN` at all, which is a different, higher-level gate
than the reference/channel power bits below it — so every earlier bench
session that only measured SPI *bus* timing (frames/s) rather than an actual
DAC output voltage could plausibly have been talking to a device that never
left device-wide power-down. In the `spi` link, init now writes `SPICONFIG`
(device active, and `FSDO=1` for `verify`'s benefit — see below), `GENCONFIG`
(reference on), `DACRANGE` (the configured ranges) and `DACPWDWN` (channels
on), then parks every channel at its `offset` before the loop starts. When
`verify` is set, all four of those init writes are confirmed over SDO/ACK the
same way the per-channel writes are (`dac_write_checked()`), each logging its
own confirmation once (`SPICONFIG write confirmed by SDO/ACK readback: ...`,
etc.) — so a clean startup rules out the *whole* digital chain up front, not
just whatever channel write happens to run first. `TRIGGER`'s soft-reset is
deliberately excluded: its fields are momentary actions (SLASEH2A table 8-17
types them all "W", reset `0000h`), so reading `SOFT-RESET` back afterward
isn't expected to still show the `1010b` that was sent — a "mismatch" there
would be a false alarm, not a real one.

**Register safety.** SLASEH2A 8.6: "All register addresses not listed
should be considered as reserved locations and the register contents
should not be modified" — on parts like this, reserved space is commonly
factory trim/calibration, so a stray write there can do real, permanent
damage. Every write in this file funnels through one function
(`spi_frame_rw()`), which checks the target address against an allow-list
built from Table 8-7 (`dac_addr_writable()`) before anything is sent, and
refuses (logging an error, no frame transmitted) if it isn't one of NOP,
SPICONFIG, GENCONFIG, BRDCONFIG, SYNCCONFIG, DACPWDWN, DACRANGE, TRIGGER,
BRDCAST, or DACA–D. This is enforced in code, not just true by inspection of
the current callers — the only runtime-variable register address this file
ever constructs is `DACx = DAC0 + channel`, and `channel` is itself bounds
checked to 0–3 at init, so it can't drift outside 0x10–0x13 either.

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
  - `verify` (boolean) (optional, `spi` link only)
    - After every write — including the one-shot init sequence
      (`SPICONFIG`/`GENCONFIG`/`DACRANGE`/`DACPWDWN`), not just the
      per-channel writes — read the register back over SDO (DB25 pin 10 →
      EVM `DAC_SDO`, only meaningful if that wire is connected) and warn if
      it doesn't match what was sent (default false). Each init write logs
      its own confirmation once, since it only happens once; the first time
      a given channel's readback confirms what was sent, it likewise logs
      once at `info` level (`channel N write confirmed by SDO/ACK
      readback: code 0x.... (V V)`), then stays quiet on success and only
      logs again on a problem, so a clean run isn't spammed once per write.
      Costs two extra 24-bit frames per write: a read-command for the
      address just written,
      then a NOP frame to shift the answer out — TI's readback convention
      echoes the *previous* cycle's R/W+address alongside the data, so it
      always trails by one frame. **This is meaningfully slower than the
      unverified path, not just "two more frames" slower**: only the last
      of the three frames' answer is ever used, but reading it back means a
      non-posted PCIe status read per bit — unlike every write in this
      file, which is posted (fire-and-forget), a read has to wait for an
      actual completion round-trip. Measured 2026-07-28: verified writes
      run roughly 0.1–0.3 ms, against ~6 µs unverified — call it two orders
      of magnitude, not a small tax. Treat `verify` as a bring-up/diagnostic
      tool you turn on to confirm the hardware, not something to leave
      enabled across a real closed loop. This device cross-checks the echo
      before trusting the data, and logs (without failing) if the echo
      itself looks wrong. **Confidence note, updated 2026-07-28 against a
      local copy of
      SLASEH2A (`dac81404.pdf`):** the frame format, R/W convention, 24-bit
      no-CRC framing, and the read-command-then-second-cycle readback
      protocol above are now confirmed from the datasheet text directly
      (Table 8-2/8-3), not just corroborated secondhand. `init` also now
      forces `SPICONFIG.FSDO=1` so SDO updates on the same SCLK falling
      edges this device already samples on (8.6.4), removing one whole axis
      of uncertainty. Two things are still open: whether the DAC preloads
      its first output bit before any clock edge, which would shift every
      sampled bit by one position (the timing diagram for this is a vector
      graphic that didn't survive text extraction, unlike the daisy-chain
      figure); and the *exact* status-register bit SDO/ACK lands on for this
      specific AX99100 card, which was never independently measured the way
      the data/control pin levels were, and isn't in the DAC's datasheet at
      all — `ack_status_bit`'s default is the generic SPP convention, not a
      bench-confirmed one. Run with `verify: true` and watch the log for
      `readback echo mismatch` before trusting a quiet run as proof anything
      is actually being checked.
  - `ack_status_bit` (integer) (optional)
    - Status-register bit carrying SDO (default 6, the standard SPP `/ACK`
      bit position). See the confidence note under `verify`.
  - `crc` (boolean) (optional, `spi` link only)
    - Use 32-bit frames with an appended CRC-8 instead of plain 24-bit
      ones, and turn on `SPICONFIG.CRC-EN` so the device itself rejects
      (rather than silently accepting) any write whose CRC doesn't check
      out (default false). Independent of `verify` — protects *every*
      write, not just ones being read back, so a marginal connection can't
      quietly land a corrupted voltage. Requires `reset_at_init` (default
      true): the frame that turns CRC on has to be sent in the old, plain
      format the device is still expecting, which is only guaranteed if
      the soft-reset just put the device back to its `CRC-EN=0` default.
      **Confidence note:** SLASEH2A 8.5.3 gives the polynomial
      (`x8+x2+x+1`, i.e. `0x07`) and nothing else — no worked example, no
      stated initial remainder, bit order, or final XOR. The implementation
      (`crc8_atm()`) uses the standard textbook parameters for that
      polynomial (initial remainder `0x00`, MSB first, no reflection, no
      XOR-out); those are assumed to match the device, not confirmed
      against it. When `verify` is also on, the readback path additionally
      recomputes the CRC over the echoed content and compares it to what
      the chip sent — an independent cross-check on top of the existing
      address-echo check, since matching both by coincidence is far less
      likely than matching just one.

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
