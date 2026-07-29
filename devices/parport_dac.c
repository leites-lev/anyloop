// Parallel-port-driven TI DAC81404 (quad 16-bit, high-voltage output) --
// the fast replacement for the DAQC2/BRIDGEplate serial DAC path.
//
// The transport is a StarTech PEX1P2 (ASIX AX99100) PCIe parallel card. Its
// registers are reachable through a *memory* BAR, so a pin change is a plain
// store to mapped memory (~100-300 ns, posted) instead of an outb (~1-2 us,
// non-posted) or a serial round-trip (~500-700 us on the BRIDGEplate).
//
// Two ways to get from the DB25 to the DAC, selected by `link`:
//
//   link="spi" (default) -- the DB25 data pins ARE the SPI bus: D0=SCLK,
//     D1=SDIN, D2=SYNC by default (all configurable), GND on 18-25. SYNC and
//     LDAC are both active-low pins, per TI's own naming (SLASEH2A pin table
//     -- no "Z" suffix; see the polarity note below). We bit-bang the 24-bit
//     frames ourselves: 2 stores per bit plus the frame
//     channel update (~123 ns per store, effective SCLK ~3.75 MHz). That is
//     far under every SCLK ceiling the part specifies: 50 MHz for writes at
//     IOVDD 2.7-5.5 V (SLASEH2A 7.7), but only 35 MHz for reads with FSDO=1
//     and 20 MHz with FSDO=0 (7.10, 7.11) -- worth knowing because `verify`
//     reads. Nothing but wires between the card and the BP-DAC81404EVM, so
//     this works today.
//
//   link="pico" -- the DB25 carries a byte-parallel handshake to an
//     RP2040/RP2350 whose PIO does the SPI at 50 MHz. Four stores per sample
//     (~0.6 us): low byte, clock high, high byte, clock low; the channel
//     number rides on spare control lines. About 12x less time spent inside
//     the loop's critical section, and the Pico owns the DAC's init sequence
//     and LDAC timing. Requires the bridge firmware to exist.
//
// LEVELS: MEASURED 2026-07-27 on this card, and the two pin groups DIFFER.
// Data pins D0-D7 swing to 3.3 V unbuffered -- safe straight into the EVM
// (IOVDD 3.3 V) or a Pico. But the CONTROL pins (1, 14, 16, 17) swing to
// 4.94 V: they are buffered to IEEE-1284 levels, which exceeds the DAC's
// absolute maximum and would kill an RP2040/RP2350 GPIO. So the spi link
// (data pins only) needs no level shifting, while the pico link -- whose
// clock and channel-select lines sit on control pins -- DOES, as would a
// hardware LDAC line on a control pin. Series resistors of 33-100 ohm on SCLK
// and leads under ~20 cm keep the bit-bang edges clean.
//
// PINOUT: D0-D7 are DB25 pins 2-9, ground is 18-25. DB25 pin 1 is /STROBE, a
// CONTROL line, not D0. Control lines are pins 1 (STROBE), 14 (AUTOFD), 16
// (INIT), 17 (SELECT); all but INIT are inverted between register bit and pin,
// which pp_write_ctrl() folds in so callers work in pin polarity.
//
// POLARITY: the DAC's frame-bracket pin is SYNC and its load-strobe pin is
// LDAC, both active-low -- confirmed 2026-07-28 against a locally saved copy
// of SLASEH2A (dac81404.pdf, opened with pdftotext -- TI's own pin table
// literally reads "SDO SCLK SDIN SYNC LDAC GND IOVDD CLR", no "Z" anywhere).
// This file briefly called them SYNCZ/LDACZ, reasoning from the BP-DAC81404
// EVM's *schematic* net name `DAC_SYNCZ` (SLAU825) -- that's the EVM board
// designer's own active-low notation for the net, not the chip's pin name,
// and the datasheet is the authority for what the pin is actually called.
// The bit-bang logic was never affected either way (SYNC is driven low to
// open a frame and high to close it, see spi_frame() below); only the name
// wobbled.
//
// LDAC is never driven by this code at all, and it is not what makes the
// outputs move. What does is SYNCCONFIG: with every DACx-SYNC-EN bit clear
// the channel is in ASYNCHRONOUS mode, where "a DAC data register write
// results in an immediate update of the DAC active register and DAC output
// on a SYNC rising edge" (SLASEH2A 8.3.3.1.2) and LDAC is simply ignored.
// That is the reset default, but dac_configure() writes it explicitly rather
// than assume it -- in synchronous mode every DACx write would be accepted
// and no output would ever move, which is a miserable thing to debug.
//
// UNUSED PINS: LDAC (13), CLR (16) and RST (32) are driven by nothing here.
// Tie all three to IOVDD. SLASEH2A's pin table says so outright for LDAC and
// CLR ("Connect to IOVDD if unused"); it says nothing of the sort for RST,
// but all three are active-low inputs with no documented internal pull-up, so
// leaving one open is not the same as holding it inactive. Each fails
// differently:
//   LDAC -- grounding it (the EVM's J2 1-2 default) is harmless only as long
//     as every channel stays in asynchronous mode.
//   CLR  -- a low "forces all DAC channels to clear the contents of their
//     buffer and active registers to the clear code regardless of their
//     synchronization setting" (8.3.3.3): zero code unipolar, midscale
//     bipolar, all four channels. With skip_unchanged set this code would not
//     notice, since it only re-sends a channel whose computed code CHANGES.
//   RST  -- worst of the three. tRSTW is 20 ns, and a POR "causes all
//     registers to initialize to default values" (8.3.5): DEV-PWDWN set,
//     reference off, channels powered down and clamped to ground, ranges back
//     to 0-5 V, FSDO back to 0, CRC-EN off. dac_configure() runs once in init
//     and nothing re-checks it, so every frame after that is silently ignored
//     until anyloop restarts. Nothing here needs a hardware reset line
//     anyway: reset_at_init issues a SOFT-RESET through TRIGGER, which 8.3.5
//     defines as the same POR event. 11.1 backs the tie-off up -- "the RST
//     and FAULT signals are static lines".
// Meter all three at ~IOVDD before wiring the DB25.
//
// ROOT / DRIVER: the mmio backend needs CAP_SYS_RAWIO (run as root) and needs
// parport_pc off the device, which `unbind` does by default (rebind after with
// `echo 0000:05:00.2 > /sys/bus/pci/drivers/parport_pc/bind`). It ALSO needs
// kernel lockdown off: with Secure Boot ON the machine boots lockdown
// "integrity", which refuses to mmap a PCI BAR ("Lockdown: direct PCI access
// is restricted" in dmesg) and equally refuses ioperm/outb, leaving no
// user-space fast path at all. Check with
// `cat /sys/kernel/security/lockdown` -- the BRACKETED word is the active mode
// and must read "[none]" (Secure Boot was disabled 2026-07-27 to get there).
//
// backend="ppdev" works regardless: same wiring, same frames, but every edge
// becomes an ioctl. MEASURED here at ~4 us per edge, i.e. ~200 us per 24-bit
// frame (~4.9 k frames/s), so a two-channel update costs ~0.4 ms -- as slow as
// the BRIDGEplate it replaces. Use it to check wiring and frames, not to close
// a loop. Nothing here touches interrupts or DMA.
//
// Params (transport):
//   backend   -- "mmio" (default) or "ppdev"
//   pci       -- PCI slot of the parallel function (default "0000:05:00.2");
//                check with `lspci -d 125b: -nn`
//   bar       -- memory BAR index holding the SPP registers (default 2)
//   data_offset, status_offset, ctrl_offset -- register offsets inside `bar`
//                (defaults 0x280, 0x284, 0x288). The AX99100 does NOT use the
//                byte-packed ISA layout: the block is dword-spaced at +0x280,
//                measured on this card 2026-07-27
//   ecr_bar   -- memory BAR holding the ECP ECR (default 2 = same BAR as the
//                other registers, -1 to skip). The ECR is forced to SPP/
//                compatibility mode at init so the data pins are plain outputs
//   ecr_offset -- ECR offset inside ecr_bar (default 0x2a8)
//   ecr_value -- what to write there (default 0x14: SPP, interrupts off)
//   unbind    -- unbind parport_pc from `pci` before mapping (default true)
//   probe     -- read the data register back at init and check it holds what
//                we wrote, which catches a wrong `bar` (default true)
//   port      -- ppdev node (default "/dev/parport0")
//
// Params (link):
//   link      -- "spi" (default) or "pico"
//   sclk_bit, sdin_bit, sync_bit -- data pins carrying the SPI signals in
//                spi link (defaults 0, 1, 2 = DB25 pins 2, 3, 4)
//   clk_ctrl_bit -- control line clocking bytes into the bridge in pico link:
//                0=STROBE(pin 1) 1=AUTOFD(14) 2=INIT(16) 3=SELECT(17)
//                (default 0)
//   chan_ctrl_shift -- first control line carrying the channel number in pico
//                link; two lines are used (default 1, i.e. AUTOFD+INIT)
//   delay_ns  -- extra dwell after each edge, nanoseconds (default 0). The
//                store latency alone (~123 ns) is an order of magnitude past
//                the DAC's tSDIS/tSDIH of 10 ns at IOVDD 2.7-5.5 V (20 ns
//                below that) -- SLASEH2A 7.6, 7.7 -- so raise this only if
//                the wiring is long or unterminated
//   verify    -- spi link only; after each write -- including the one-shot
//                SPICONFIG/GENCONFIG/SYNCCONFIG/DACRANGE/DACPWDWN init
//                sequence in dac_configure(), not just per-channel writes --
//                check over SDO/ACK (DB25 pin 10 -> status register) that it
//                landed, and warn if it didn't (default false).
//                HOW MUCH gets checked depends on the register, because half
//                the DAC81404's map is write-only (SLASEH2A table 8-7 TYPE
//                column). SPICONFIG/GENCONFIG/SYNCCONFIG/DACPWDWN are R/W, so
//                the value itself is read back and compared. DACRANGE and
//                DACA-D are typed plain W: nothing can be read back from
//                them, and a read command aimed at one proves nothing, so
//                what gets checked is the R/W+address echo the NEXT cycle
//                carries (table 8-3). Turning `crc` on upgrades even those to
//                a full data check, because a CRC-mode echo carries the
//                previous cycle's data as well (table 8-5). The logs say
//                which of the two happened ("confirmed by ... readback" vs.
//                "acknowledged by ... (R/W+address echo)").
//                Cost: one or two extra frames per write, PLUS a non-posted
//                PCIe status read on every bit of the one frame whose answer
//                is used -- measured 2026-07-28: verified writes run roughly
//                two orders of magnitude slower than the ~6 us unverified
//                figure above (order 0.1-0.3 ms), because pp_read_status() is
//                a non-posted read (the CPU waits for a completion
//                round-trip) where every other operation in this file is a
//                posted, fire-and-forget write. This is inherent to
//                reading anything back over this transport, not a bug to
//                chase further -- verify is a bring-up/diagnostic tool,
//                not something to leave on across a real closed loop.
//                dac_configure() also forces SPICONFIG.FSDO=1, which is
//                REQUIRED, not cosmetic: SCLK idles high, so the first edge
//                inside a cycle is a falling one, and only "SDO updates on
//                SCLK falling edges" (SLASEH2A 8.6.4) lines the 24 samples up
//                with the 24 output bits. With the reset default FSDO=0 every
//                sample would sit one bit position early. One thing is still
//                NOT confirmed: which AX99100 status register bit ACK
//                actually lands on for THIS card (that's not in the DAC's
//                datasheet at all -- it would need the AX99100 datasheet or a
//                bench measurement). See dac_write_and_verify()'s echo check,
//                which exists specifically to catch that rather than silently
//                trusting misaligned data.
//   ack_status_bit -- status-register bit carrying SDO (default 6, the
//                standard SPP ACK bit position; unverified on this card,
//                see `verify` above)
//   crc       -- spi link only; use 32-bit frames with an appended CRC-8
//                (SLASEH2A 8.5.3, polynomial x8+x2+x+1) instead of plain
//                24-bit ones, and turn on SPICONFIG.CRC-EN so the device
//                rejects (instead of silently accepting) a write whose CRC
//                doesn't check out (default false). Independent of
//                `verify`: this protects every write, not just ones being
//                read back -- and when `verify` IS on it strictly improves
//                it, since a CRC-mode echo carries the previous cycle's data
//                (table 8-5), which is the only way to confirm a write to a
//                write-only register like DACx. crc8_atm() uses initial
//                remainder 0x00, MSB first, no reflection and no XOR-out;
//                8.5.3's "if no error exists, the CRC remainder is zero"
//                pins down everything there except bit order. Requires
//                reset_at_init (default true); see dac_configure()'s
//                comment on why.
//
// Params (outputs -- same meaning as piplate_bridge, so configs port over):
//   channel   -- DAC channel(s), 0=DACA 1=DACB 2=DACC 3=DACD, scalar or array
//   index     -- pipeline vector element(s) to command (default 0)
//   scale     -- volts per pipeline unit (default 1.0)
//   offset    -- volts at zero command (default 0.0)
//   range     -- output range per channel, scalar or array (default "0-10"):
//                "0-5" "0-6" "0-10" "0-12" "0-20" "0-24" "0-40"
//                "+-5" "+-6" "+-10" "+-12" "+-20"
//   start_delay -- seconds to hold a channel at `offset` before letting the
//                pipeline command through (default 0). Give the upstream pid
//                a matching start_delay, or its integrator winds up during
//                the hold
//   skip_unchanged -- only send a channel when its 16-bit code changes
//                (default false). The DAC holds its register, so this is safe
//                here -- unlike the BRIDGEplate there is no droop argument --
//                but it costs nothing to leave off
//   reset_at_init -- issue a DAC soft reset before configuring (default true;
//                spi link only)
//
// The DAC81404 wakes up device-wide powered down (SPICONFIG.DEV-PWDWN) with
// its reference and all four output amplifiers powered down too, so in spi
// link init writes SPICONFIG (device active), GENCONFIG (reference on),
// SYNCCONFIG (asynchronous update), DACRANGE (the configured ranges) and
// DACPWDWN (the configured channels on, the rest left clamped to ground
// through their internal 10 kohm) before parking every channel at its
// `offset`. Codes are MSB-aligned straight binary across the selected range,
// divided into 2^16 steps rather than 2^16 - 1: SLASEH2A equations 1 and 2
// make full-scale code 0xFFFF one LSB SHORT of the nominal top of the range
// (9.99985 V on the 0-10 V range), and volts_to_code() scales accordingly.
//
// Output updates are spaced at least SLASEH2A's tDACWAIT (2.4 us) apart --
// see dac_update_wait(). A bit-banged frame is longer than that anyway, but
// the pico link's ~0.6 us per sample is not.

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <json-c/json.h>
#include <linux/parport.h>
#include <linux/ppdev.h>

#include "anyloop.h"
#include "logging.h"
#include "parport_dac.h"
#include "xalloc.h"

// Where the AX99100 puts the parallel port's registers inside its memory BAR.
// They are NOT the byte-packed ISA layout: the block sits at BAR2 + 0x280 and
// is dword-spaced, with a second copy at +0x2c0 and the whole 0x400 window
// repeating through the 4 kB page. Verified 2026-07-27 against the I/O window
// at 0x3010 (a byte written through either window reads back in the other).
#define SPP_DATA_DEFAULT    0x280
#define SPP_STATUS_DEFAULT  0x284
#define SPP_CONTROL_DEFAULT 0x288
// Standard SPP status-register bit for /ACK, direct (not inverted, unlike
// BUSY). This is a generic ISA-parallel-port convention, NOT something
// measured on this card's status register the way the data/control pin
// levels were -- see `verify`'s doc comment.
#define SPP_STATUS_ACK_DEFAULT 6
// the ECP ECR lives in the same block, not in the second memory BAR (BAR5
// holds the AX99100's own configuration registers and does not mirror it)
#define SPP_ECR_DEFAULT     0x2A8

// Control-register bits that are inverted between the register and the DB25
// pin: STROBE (pin 1), AUTOFD (14) and SELECT (17). INIT (16) is not.
#define CTRL_INVERT_MASK 0x0B
// Control bit 4 enables the port IRQ and bit 5 turns the data pins into
// inputs; both must stay clear.
#define CTRL_WRITE_MASK 0x0F

// DAC81404 register addresses (SLASEH2A table 8-7)
#define DAC_REG_NOP       0x00
#define DAC_REG_DEVICEID  0x01
#define DAC_REG_STATUS    0x02
#define DAC_REG_SPICONFIG 0x03
#define DAC_REG_GENCONFIG 0x04
#define DAC_REG_BRDCONFIG 0x05
#define DAC_REG_SYNCCONFIG 0x06
#define DAC_REG_DACPWDWN  0x09
#define DAC_REG_DACRANGE  0x0A
#define DAC_REG_TRIGGER   0x0E
#define DAC_REG_BRDCAST   0x0F
#define DAC_REG_DAC0      0x10	// DACA; DACB/C/D follow at 0x11-0x13

// GENCONFIG with REF-PWDWN (bit 14) cleared: internal reference on
#define DAC_GENCONFIG_REF_ON 0x0000
// SYNCCONFIG with every DACx-SYNC-EN bit clear: asynchronous update, i.e.
// "a DAC data register write results in an immediate update of the DAC active
// register and DAC output on a SYNC rising edge" (SLASEH2A 8.3.3.1.2). This is
// the reset default, but this file depends on it (it never issues a SOFT-LDAC
// and never pulses the LDAC pin), so write it rather than assume it: with
// reset_at_init=false, a device left in synchronous mode by a previous process
// would accept every DACx write and never move its outputs.
#define DAC_SYNCCONFIG_ASYNC 0x0000
// DACPWDWN bits 3-0 are per-channel power-DOWN flags, set at reset; bits 15-4
// are read-only reserved and read back as FFFh (SLASEH2A table 8-15), so start
// from FFFF and clear only the channels this stage actually drives
#define DAC_PWDWN_NONE 0xFFFF
// TRIGGER SOFT-RESET[3:0] = 1010b (SLASEH2A table 8-17, "reserved code 1010
// to reset the device to the default state")
#define DAC_TRIGGER_SOFT_RESET 0x000A
// SPICONFIG reset value is 0x0AA4 (SLASEH2A 8.6.4), which has bit 5
// (DEV-PWDWN) set -- the WHOLE DEVICE, not just the reference or individual
// channels, wakes up powered down, separately from GENCONFIG.REF-PWDWN and
// DACPWDWN's per-channel bits. Found 2026-07-28 reading a local copy of the
// datasheet: nothing in this file was ever clearing it, so every prior
// bench session that only measured SPI bus timing rather than an actual DAC
// output voltage could have been driving a device that was still in
// device-wide power-down the whole time. This value clears DEV-PWDWN (bit 5)
// and sets FSDO=1 (bit 1, SDO updates on SCLK falling edges -- see
// spi_frame_rw()'s doc comment for why that matters to `verify`), keeping
// SDO-EN=1 and CRC-EN=0 at their reset defaults and leaving the alarm-enable
// bits (11, 9) as-is.
#define DAC_SPICONFIG_ACTIVE 0x0A86
// SPICONFIG bit 4, CRC-EN: "When set to 1, frame error checking is
// enabled" (SLASEH2A 8.6.4). OR this into the SPICONFIG write when `crc`
// is requested.
#define SPICONFIG_CRC_EN_BIT 0x0010

// The transfer function is CODE / 2^N, not CODE / (2^N - 1): SLASEH2A
// equations 1 and 2 read VOUT = VREFIO*GAIN*CODE/2^N (unipolar) and
// VREFIO*GAIN*CODE/2^N - GAIN*VREFIO/2 (bipolar), so full scale is one LSB
// *below* the nominal top of the range -- code 0xFFFF on the 0-10 V range is
// 9.99985 V, not 10 V. Scaling by 65535 instead put every code up to a full
// LSB off.
#define DAC_CODE_COUNT 65536.0
#define DAC_CODE_MAX 65535
// SLASEH2A tDACWAIT (tables 7-6, 7-7) and 8.3.3.1: "In both update modes, a
// minimum wait time of 2.4 us is required between DAC output updates."
#define DAC_UPDATE_WAIT_NS 2400
// seconds between throughput reports
#define PARPORT_DAC_DIAG_PERIOD 5.0

struct dac_range {
	const char *name;
	int code;		// DACRANGE nibble
	double vmin, vmax;
};

// SLASEH2A table 8-16
static const struct dac_range dac_ranges[] = {
	{ "0-5",   0x0,   0.0,  5.0 },
	{ "0-10",  0x1,   0.0, 10.0 },
	{ "0-20",  0x2,   0.0, 20.0 },
	{ "0-40",  0x3,   0.0, 40.0 },
	{ "+-5",   0x5,  -5.0,  5.0 },
	{ "+-10",  0x6, -10.0, 10.0 },
	{ "+-20",  0x7, -20.0, 20.0 },
	{ "0-6",   0x8,   0.0,  6.0 },
	{ "0-12",  0x9,   0.0, 12.0 },
	{ "0-24",  0xA,   0.0, 24.0 },
	{ "+-6",   0xD,  -6.0,  6.0 },
	{ "+-12",  0xE, -12.0, 12.0 },
};

static const struct dac_range *find_range(const char *name)
{
	// accept the obvious spellings of a bipolar range
	if (!strncmp(name, "±", strlen("±"))) name += strlen("±");
	else if (name[0] == '+' && name[1] == '-') name += 2;
	else if (name[0] == '-') name += 1;
	else goto unipolar;
	for (size_t i = 0; i < sizeof dac_ranges / sizeof dac_ranges[0]; i++)
		if (dac_ranges[i].vmin < 0.0
		&& !strcmp(dac_ranges[i].name + 2, name))
			return &dac_ranges[i];
	return NULL;
unipolar:
	for (size_t i = 0; i < sizeof dac_ranges / sizeof dac_ranges[0]; i++)
		if (!strcmp(dac_ranges[i].name, name))
			return &dac_ranges[i];
	return NULL;
}

static double monotonic_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + 1e-9 * t.tv_nsec;
}

static int64_t monotonic_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

/** Round a target voltage to the code the DAC will actually produce. Data are
* MSB-aligned straight binary across the configured range (SLASEH2A 8.6.12),
* and the range is divided into 2^N steps, not 2^N - 1 (SLASEH2A equations 1
* and 2) -- so `vmax` is the nominal top of the range and is one LSB above
* anything the part can actually output. */
static int volts_to_code(double volts, double vmin, double vmax)
{
	double frac = (volts - vmin) / (vmax - vmin);
	long c = lround(frac * DAC_CODE_COUNT);
	if (c < 0) c = 0;
	if (c > DAC_CODE_MAX) c = DAC_CODE_MAX;
	return (int)c;
}

static double code_to_volts(int code, double vmin, double vmax)
{
	return vmin + (vmax - vmin) * code / DAC_CODE_COUNT;
}

/** CRC-8 over the 24-bit content of a frame, per SLASEH2A 8.5.3: "based on
* the CRC-8-ATM (HEC) polynomial: x8 + x2 + x + 1 (that is, 100000111)"
* (0x07). The datasheet gives no worked example, but it does pin down the
* parameters indirectly: "The device decodes the 32-bit access cycle to
* compute the CRC remainder on SYNC rising edges. If no error exists, the CRC
* remainder is zero." A remainder of zero over content-plus-appended-CRC only
* comes out for initial remainder 0x00 with no final XOR -- textbook
* CRC-8-ATM (HEC) as used for ATM headers adds a 0x55 XOR-out, which would
* leave a constant non-zero remainder instead. So: initial remainder 0x00,
* MSB first, no reflection, no XOR-out. Bit order is the one thing still
* taken on faith, and it is the natural one for an MSB-first bus. */
static uint8_t crc8_atm(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0x00;
	for (size_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
				: (uint8_t)(crc << 1);
		}
	}
	return crc;
}

// ---------------------------------------------------------------------------
// parallel port register access

/** Spin for delay_ns after an edge. Zero (the default) skips the clock read
* entirely: an MMIO store already costs ~100-300 ns, an order of magnitude
* more than the DAC's 10 ns tSDIS/tSDIH at IOVDD 2.7-5.5 V (SLASEH2A 7.7). */
static inline void pp_dwell(const struct aylp_parport_dac_data *data)
{
	if (LIKELY(!data->delay_ns)) return;
	struct timespec t0, t;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	do {
		clock_gettime(CLOCK_MONOTONIC, &t);
	} while ((t.tv_sec - t0.tv_sec) * 1000000000L
		+ (t.tv_nsec - t0.tv_nsec) < data->delay_ns);
}

static inline void pp_write_data(struct aylp_parport_dac_data *data, uint8_t v)
{
	data->data_shadow = v;
	if (LIKELY(data->backend == AYLP_PARPORT_MMIO)) {
		*data->reg_data = v;
	} else {
		unsigned char c = v;
		ioctl(data->pp_fd, PPWDATA, &c);
	}
	pp_dwell(data);
}

/** Write the four control lines. `v` is in *pin* polarity (bit set = pin
* high), so callers don't have to remember which lines the hardware inverts;
* the mmio path folds in the inversion, and ppdev's driver does it for us. */
static inline void pp_write_ctrl(struct aylp_parport_dac_data *data, uint8_t v)
{
	data->ctrl_shadow = v & CTRL_WRITE_MASK;
	if (LIKELY(data->backend == AYLP_PARPORT_MMIO)) {
		*data->reg_ctrl = data->ctrl_shadow ^ CTRL_INVERT_MASK;
	} else {
		unsigned char c = data->ctrl_shadow;
		ioctl(data->pp_fd, PPWCONTROL, &c);
	}
	pp_dwell(data);
}

/** Read the status register (holds SDO/ACK among other input pins). */
static inline uint8_t pp_read_status(struct aylp_parport_dac_data *data)
{
	if (LIKELY(data->backend == AYLP_PARPORT_MMIO)) {
		return *data->reg_status;
	} else {
		unsigned char c = 0;
		ioctl(data->pp_fd, PPRSTATUS, &c);
		return c;
	}
}

/** mmap a PCI BAR through sysfs, at least far enough to reach `span`.
* Returns the base of the mapping, or NULL. */
static volatile uint8_t *map_bar(const char *pci, int bar, size_t span,
	void **map_out, size_t *len_out)
{
	char path[128];
	snprintf(path, sizeof path,
		"/sys/bus/pci/devices/%s/resource%d", pci, bar);
	int fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		log_error("parport_dac: open %s: %s", path, strerror(errno));
		if (errno == EACCES || errno == EPERM)
			log_error("parport_dac: mapping a BAR needs root "
				"(CAP_SYS_RAWIO); use backend=\"ppdev\" to run "
				"unprivileged");
		return NULL;
	}
	long page = sysconf(_SC_PAGESIZE);
	size_t len = (size_t)page;
	while (len < span + 1) len += (size_t)page;
	void *m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (m == MAP_FAILED) {
		log_error("parport_dac: mmap %s: %s", path, strerror(errno));
		return NULL;
	}
	*map_out = m;
	*len_out = len;
	return (volatile uint8_t *)m;
}

/** Detach parport_pc from the card so the kernel driver and we aren't both
* driving the same pins. Missing driver or already-unbound is not an error. */
static void unbind_parport_pc(const char *pci)
{
	int fd = open("/sys/bus/pci/drivers/parport_pc/unbind", O_WRONLY);
	if (fd < 0) {
		if (errno != ENOENT)
			log_warn("parport_dac: could not open parport_pc "
				"unbind: %s", strerror(errno));
		return;
	}
	ssize_t w = write(fd, pci, strlen(pci));
	close(fd);
	if (w < 0) {
		// ENODEV just means it wasn't bound in the first place
		if (errno != ENODEV)
			log_warn("parport_dac: unbind %s from parport_pc: %s",
				pci, strerror(errno));
	} else {
		log_info("parport_dac: unbound %s from parport_pc", pci);
	}
}

// ---------------------------------------------------------------------------
// links

/** Every register this code is allowed to write (SLASEH2A table 8-7),
* excluding the two read-only entries DEVICEID and STATUS. SLASEH2A 8.6:
* "All register addresses not listed should be considered as reserved
* locations and the register contents should not be modified" -- on parts
* like this, reserved space is commonly factory trim/calibration, so a
* stray write there can do real, permanent damage, not just nothing.
* Checked in spi_frame_rw() itself, the single choke point every write goes
* through, rather than trusted to be true by inspection of the callers. */
static bool dac_addr_writable(uint8_t addr)
{
	switch (addr & 0x3F) {
	case DAC_REG_NOP:
	case DAC_REG_SPICONFIG:
	case DAC_REG_GENCONFIG:
	case DAC_REG_BRDCONFIG:
	case DAC_REG_SYNCCONFIG:
	case DAC_REG_DACPWDWN:
	case DAC_REG_DACRANGE:
	case DAC_REG_TRIGGER:
	case DAC_REG_BRDCAST:
	case DAC_REG_DAC0 + 0:
	case DAC_REG_DAC0 + 1:
	case DAC_REG_DAC0 + 2:
	case DAC_REG_DAC0 + 3:
		return true;
	default:
		return false;
	}
}

/** Registers whose contents can be read back, i.e. the ones SLASEH2A table
* 8-7 types "R" or "R/W". NOP, DACRANGE, BRDCAST and DACA-D are typed plain
* "W": they are write-only, and a read command aimed at one has no defined
* data to return, so comparing a readback against what was written is
* meaningless for them (it was reliably reporting a "mismatch" on DACRANGE
* and on every channel write). TRIGGER is listed R/W in table 8-7 but every
* one of its fields is typed W with reset 0000h in table 8-17, so it counts
* as write-only here too. spi_write_verified() falls back to checking the
* R/W+address echo (SLASEH2A table 8-3) for these. */
static bool dac_addr_readable(uint8_t addr)
{
	switch (addr & 0x3F) {
	case DAC_REG_DEVICEID:
	case DAC_REG_STATUS:
	case DAC_REG_SPICONFIG:
	case DAC_REG_GENCONFIG:
	case DAC_REG_BRDCONFIG:
	case DAC_REG_SYNCCONFIG:
	case DAC_REG_DACPWDWN:
		return true;
	default:
		return false;
	}
}

/** Emit one 24-bit DAC81404 access cycle by bit-banging the DB25 data pins,
* optionally reading SDO back over the status register at the same time.
* Refuses (logs and returns without sending) any write outside
* dac_addr_writable()'s allow-list.
*
* SDIN is sampled on SCLK falling edges and the cycle is bracketed by SYNC
* going low and back high (SLASEH2A 8.5.1), so each bit costs two stores: one
* that presents the bit with SCLK high, one that drops SCLK. Idle state is
* SYNC high, SCLK high.
*
* When `sample_sdo` is set, we also sample the ACK/SDO status bit on each
* of the 24 falling edges and return the 24 bits shifted in, MSB first.
* Confirmed 2026-07-28 against a local copy of SLASEH2A (Table 8-2, 8-3):
* a read needs a read-command cycle followed by a second cycle to shift the
* answer out on SDO, which carries an echo of the *previous* cycle's
* R/W+address in its top 8 bits and that cycle's requested data in the
* bottom 16 -- hence a verified write is 3 calls (write, read-command, NOP
* to shift the answer out), and only the LAST of those three has anything
* worth sampling; `sample_sdo` lets callers skip the status read on the
* other two, which matters because it is not free: pp_read_status() is a
* non-posted PCIe read (the CPU waits for a completion round-trip), unlike
* the posted writes this file is otherwise built on, and measures roughly
* an order of magnitude slower per access -- sampling on all three frames
* instead of just the one that's used was needlessly tripling that cost.
* SPICONFIG.FSDO is forced to 1 in dac_configure() specifically so "SDO
* updates on SCLK falling edges" (8.6.4) lines up with where this loop
* samples, and that is a requirement here, not a nicety: because the idle
* level of SCLK is high, the first SCLK edge inside a cycle is a FALLING one,
* so the loop's 24 samples line up one-for-one with the 24 falling edges
* (D23 first). With the reset default FSDO=0 -- "SDO updates on SCLK rising
* edges" -- the first rising edge does not arrive until after the first
* falling edge, so every sample would be shifted one position and D0 would
* never be seen at all. That leaves one thing unconfirmed, on purpose not
* papered over: which AX99100 status-register bit ACK physically lands on for
* this card (not in the DAC datasheet at all). dac_write_and_verify()
* cross-checks the echoed R/W+address before trusting the data for exactly
* that reason. */
static uint32_t spi_frame_rw(struct aylp_parport_dac_data *data, bool read,
	uint8_t addr, uint16_t value, bool sample_sdo)
{
	if (UNLIKELY(!read && !dac_addr_writable(addr))) {
		log_error("parport_dac: refusing to write register 0x%02x -- "
			"not in the SLASEH2A 8.6 register map, so it's a "
			"reserved/undocumented location; this frame was NOT "
			"sent", addr & 0x3F);
		return 0;
	}
	uint8_t sclk = (uint8_t)(1u << data->sclk_bit);
	uint8_t sdin = (uint8_t)(1u << data->sdin_bit);
	uint8_t sync = (uint8_t)(1u << data->sync_bit);
	// bit 23 = R/W (0 = write, 1 = read), bit 22 don't care, bits 21-16
	// = address, bits 15-0 = data (don't care on a read command)
	uint32_t content = ((uint32_t)(read ? 1 : 0) << 23)
		| ((uint32_t)(addr & 0x3F) << 16) | value;
	// with `crc` on, the same 24 bits move up to fill bits 31-8 of a
	// 32-bit cycle, with an 8-bit CRC over those 24 bits (MSB first)
	// appended in bits 7-0 (SLASEH2A 8.5.3, Table 8-4) -- see
	// crc8_atm()'s doc comment for the caveat on its exact parameters
	uint32_t frame = content;
	int nbits = 24;
	if (UNLIKELY(data->crc_active)) {
		uint8_t buf[3] = {
			(uint8_t)(content >> 16),
			(uint8_t)(content >> 8),
			(uint8_t)content,
		};
		frame = (content << 8) | crc8_atm(buf, sizeof buf);
		nbits = 32;
	}
	// other data pins keep whatever they were left at
	uint8_t base = data->data_shadow & (uint8_t)~(sclk | sdin | sync);
	uint32_t rx = 0;

	pp_write_data(data, base | sync | sclk);	// idle
	pp_write_data(data, base | sclk);		// SYNC low: cycle start
	for (int i = nbits - 1; i >= 0; i--) {
		uint8_t bit = (frame >> i) & 1 ? sdin : 0;
		pp_write_data(data, base | sclk | bit);
		pp_write_data(data, base | bit);	// falling edge samples
		if (UNLIKELY(sample_sdo))
			rx = (rx << 1) | ((uint32_t)(pp_read_status(data)
				>> data->ack_status_bit) & 1u);
	}
	pp_write_data(data, base | sclk);
	pp_write_data(data, base | sync | sclk);	// SYNC high: cycle end
	return rx;
}

static inline void spi_frame(struct aylp_parport_dac_data *data, uint8_t addr,
	uint16_t value)
{
	spi_frame_rw(data, false, addr, value, false);
}

/** Hand one sample to the RP2040/RP2350 bridge: low byte, clock high, high
* byte, clock low. The channel number rides on spare control lines and stays
* valid across the whole handshake. Four stores, ~0.6 us. */
static void pico_frame(struct aylp_parport_dac_data *data, int channel,
	uint16_t value)
{
	uint8_t clk = (uint8_t)(1u << data->clk_ctrl_bit);
	uint8_t chan = (uint8_t)((channel & 3) << data->chan_ctrl_shift);
	pp_write_data(data, (uint8_t)(value & 0xFF));
	pp_write_ctrl(data, chan | clk);
	pp_write_data(data, (uint8_t)(value >> 8));
	pp_write_ctrl(data, chan);
}

enum verify_result {
	VERIFY_MISMATCH = 0,	// echo was trustworthy, data didn't match
	VERIFY_OK = 1,		// echo was trustworthy, data matched
	VERIFY_ECHO_OK = 2,	// write-only register: only the address echo
				// could be checked, and it matched
	VERIFY_UNCONFIRMED = -1,	// echo itself didn't check out
};

/** Write `value` to `addr` and check over SDO/ACK that it landed. `label` is
* only for log messages.
*
* How much can be checked depends on the register, because half of the
* DAC81404's map is write-only (SLASEH2A table 8-7 TYPE column, see
* dac_addr_readable()):
*
*   - A readable (R or R/W) register costs 3 frames: the write, a read command
*     for the same address, and a NOP frame to shift the answer out (see
*     spi_frame_rw()'s doc comment for why it takes a third frame). The value
*     itself comes back and is compared -- VERIFY_OK.
*   - A write-only register (DACRANGE and DACA-D, the ones this file writes
*     most) has no readback data at all, so a read command aimed at one proves
*     nothing; asking for it anyway was guaranteeing a bogus "not confirmed"
*     warning on every single channel write. What the device does still give
*     back is the next cycle's echo of this cycle's R/W bit and address (table
*     8-3), so those get checked with one extra frame instead of two --
*     VERIFY_ECHO_OK. With `crc` on, table 8-5 promises the echo carries the
*     *data* of the previous cycle too, so in that case the value is fully
*     confirmed after all and this returns VERIFY_OK.
*
* Logs and returns VERIFY_MISMATCH on a genuine data mismatch or a
* device-reported CRC error; logs (but doesn't call it a mismatch) and returns
* VERIFY_UNCONFIRMED on an echo mismatch, which means the readback itself
* couldn't be trusted rather than that the write was necessarily wrong -- see
* the big caveat on the `verify` param. Never logs on success; callers decide
* whether that's worth announcing (a one-shot init write vs. a per-loop
* channel write have very different answers to that). */
static enum verify_result spi_write_verified(struct aylp_parport_dac_data *data,
	uint8_t addr, uint16_t value, const char *label)
{
	bool readable = dac_addr_readable(addr);
	// only the last frame's answer is ever read, so only it needs to pay
	// for sampling the (non-posted, comparatively expensive) status reads
	spi_frame_rw(data, false, addr, value, false);		// the real write
	if (readable) {
		spi_frame_rw(data, true, addr, 0x0000, false);	// read command
		data->diag_frames++;
	}
	uint32_t rx = spi_frame_rw(data, false, DAC_REG_NOP, 0x0000, true);
	data->diag_frames += 2;
	// the echoed cycle is the read command for a readable register, and
	// the write itself for a write-only one
	uint8_t want_rw = readable ? 1 : 0;

	uint8_t echo_rw, echo_addr;
	uint16_t got;
	bool got_is_data = readable;
	if (UNLIKELY(data->crc_active)) {
		// the echoed R/W+addr+data occupy bits 31-8 here instead of
		// 23-0, with the device's own CRC of that content in 7-0
		uint32_t content = (rx >> 8) & 0xFFFFFFu;
		echo_rw = (uint8_t)((content >> 23) & 1u);
		echo_addr = (uint8_t)((content >> 16) & 0x3Fu);
		got = (uint16_t)(content & 0xFFFFu);
		uint8_t buf[3] = {
			(uint8_t)(content >> 16),
			(uint8_t)(content >> 8),
			(uint8_t)content,
		};
		uint8_t want_crc = crc8_atm(buf, sizeof buf);
		uint8_t rx_crc = (uint8_t)(rx & 0xFFu);
		if (rx_crc != want_crc) {
			log_warn("parport_dac: CRC mismatch on readback for %s "
				"(chip sent 0x%02x, computed 0x%02x from the "
				"echoed content) -- either crc8_atm()'s assumed "
				"parameters don't match the device, or the "
				"echoed content itself is corrupted", label,
				rx_crc, want_crc);
			data->diag_verify_unconfirmed++;
			return VERIFY_UNCONFIRMED;
		}
		// bit 30 of the cycle (bit 22 of the content) is CRC-ERROR:
		// "Returns a 1 when a CRC error is detected" (tables 8-5, 8-6).
		// The device ignores a write whose CRC doesn't check out, so
		// this is the difference between "rejected" and "accepted" --
		// it was being masked away and never looked at.
		if (UNLIKELY((content >> 22) & 1u)) {
			log_warn("parport_dac: device reported CRC-ERROR on the "
				"frame preceding the %s readback -- that frame "
				"was rejected, not applied; check wiring and "
				"crc8_atm()'s parameters", label);
			data->diag_verify_fail++;
			return VERIFY_MISMATCH;
		}
		// table 8-5: a write cycle's echo carries its data as well
		got_is_data = true;
	} else {
		echo_rw = (uint8_t)((rx >> 23) & 1u);
		echo_addr = (uint8_t)((rx >> 16) & 0x3Fu);
		got = (uint16_t)(rx & 0xFFFFu);
	}
	if (echo_rw != want_rw || echo_addr != (addr & 0x3F)) {
		log_warn("parport_dac: readback echo mismatch writing %s "
			"(got rw=%d addr=0x%02x, expected rw=%d addr=0x%02x) -- "
			"ack_status_bit or frame timing is probably wrong; "
			"verify is unconfirmed until this is fixed",
			label, echo_rw, echo_addr, want_rw, addr & 0x3F);
		data->diag_verify_unconfirmed++;
		return VERIFY_UNCONFIRMED;
	}
	if (!got_is_data) return VERIFY_ECHO_OK;
	if (got != value) {
		log_warn("parport_dac: %s write not confirmed: wrote 0x%04x, "
			"read back 0x%04x", label, value, got);
		data->diag_verify_fail++;
		return VERIFY_MISMATCH;
	}
	return VERIFY_OK;
}

/** dac_configure()'s helper: write `value` to `addr`, verified over SDO/ACK
* if `data->verify` is set (logging a confirmation every time -- these each
* run once at init, unlike the per-channel writes below, so that's useful
* rather than spammy), otherwise a plain unverified write. */
static void dac_write_checked(struct aylp_parport_dac_data *data,
	uint8_t addr, uint16_t value, const char *label)
{
	if (UNLIKELY(data->verify)) {
		switch (spi_write_verified(data, addr, value, label)) {
		case VERIFY_OK:
			log_info("parport_dac: %s write confirmed by SDO/ACK "
				"readback: 0x%04x", label, value);
			break;
		case VERIFY_ECHO_OK:
			log_info("parport_dac: %s write acknowledged by SDO/ACK "
				"(R/W+address echo for 0x%04x); SLASEH2A table "
				"8-7 types this register W, so its contents "
				"cannot be read back to compare", label, value);
			break;
		default:
			break;
		}
	} else {
		spi_frame(data, addr, value);
		data->diag_frames++;
	}
}

/** Write a DACx register, verified over SDO/ACK if `data->verify` is set.
* Unlike dac_write_checked(), only logs a confirmation the first time a
* given channel succeeds -- this runs every loop iteration, so logging
* every time would spam the terminal once per write instead of once ever. */
static void dac_write_and_verify(struct aylp_parport_dac_data *data,
	int channel, uint16_t code)
{
	char label[24];
	snprintf(label, sizeof label, "channel %d", channel);
	uint8_t addr = (uint8_t)(DAC_REG_DAC0 + channel);
	enum verify_result r = spi_write_verified(data, addr, code, label);
	if ((r != VERIFY_OK && r != VERIFY_ECHO_OK)
	|| data->verify_confirmed[channel])
		return;
	data->verify_confirmed[channel] = true;
	double volts = NAN;
	for (size_t i = 0; i < data->n_outputs; i++) {
		if (data->channels[i] == channel) {
			volts = code_to_volts(code, data->vmin[i],
				data->vmax[i]);
			break;
		}
	}
	// the DACx registers are write-only (SLASEH2A table 8-7), so without
	// `crc` the strongest thing SDO can confirm is that the device took
	// the right address -- say which one actually happened
	if (r == VERIFY_OK)
		log_info("parport_dac: channel %d write confirmed by SDO/ACK "
			"readback: code 0x%04x (%.4f V)", channel, code, volts);
	else
		log_info("parport_dac: channel %d write acknowledged by SDO/ACK "
			"(R/W+address echo) for code 0x%04x (%.4f V); DACx is a "
			"write-only register, so enable `crc` if you want the "
			"code itself echoed back too", channel, code, volts);
}

/** Hold off until at least tDACWAIT has passed since the last DAC output
* update. SLASEH2A 8.3.3.1: "In both update modes, a minimum wait time of
* 2.4 us is required between DAC output updates" (tables 7-6/7-7, tDACWAIT).
* A bit-banged 24-bit frame already takes longer than that on its own, so
* this never actually spins on the spi link -- but the pico link hands a
* sample over in ~0.6 us, so two channels in the same iteration would have
* been issued about four times too fast. Cheap enough to just enforce for
* both rather than leave it as an assumption about store latency. */
static inline void dac_update_wait(struct aylp_parport_dac_data *data)
{
	if (UNLIKELY(!data->last_update_ns)) return;
	int64_t now;
	do {
		now = monotonic_ns();
	} while (now - data->last_update_ns < DAC_UPDATE_WAIT_NS);
}

/** Send one channel's 16-bit code, whichever link is configured. */
static void dac_write_channel(struct aylp_parport_dac_data *data, int channel,
	uint16_t code)
{
	dac_update_wait(data);
	if (data->link == AYLP_PARPORT_LINK_SPI) {
		if (UNLIKELY(data->verify)) {
			// spi_write_verified() already counted its own frames
			dac_write_and_verify(data, channel, code);
			data->last_update_ns = monotonic_ns();
			return;
		}
		spi_frame(data, (uint8_t)(DAC_REG_DAC0 + channel), code);
	} else {
		pico_frame(data, channel, code);
	}
	data->diag_frames++;
	// the output updates on the SYNC rising edge that just went out, so
	// time the next one from here rather than from the start of the frame
	data->last_update_ns = monotonic_ns();
}

// ---------------------------------------------------------------------------

// helpers for a param that may be a scalar (broadcast to every channel) or an
// array (one value per channel), as in piplate_bridge
static size_t val_count(struct json_object *v)
{
	if (v && json_object_is_type(v, json_type_array))
		return json_object_array_length(v);
	return 1;
}

static int val_int_at(struct json_object *v, size_t i, int def)
{
	if (!v) return def;
	if (json_object_is_type(v, json_type_array)) {
		struct json_object *e = json_object_array_get_idx(v, i);
		return e ? (int)json_object_get_int64(e) : def;
	}
	return (int)json_object_get_int64(v);
}

static double val_dbl_at(struct json_object *v, size_t i, double def)
{
	if (!v) return def;
	if (json_object_is_type(v, json_type_array)) {
		struct json_object *e = json_object_array_get_idx(v, i);
		return e ? json_object_get_double(e) : def;
	}
	return json_object_get_double(v);
}

static const char *val_str_at(struct json_object *v, size_t i, const char *def)
{
	if (!v) return def;
	if (json_object_is_type(v, json_type_array)) {
		struct json_object *e = json_object_array_get_idx(v, i);
		return e ? json_object_get_string(e) : def;
	}
	return json_object_get_string(v);
}

static int open_mmio(struct aylp_parport_dac_data *data)
{
	if (data->unbind) unbind_parport_pc(data->pci);

	// map far enough to reach the furthest register we were asked for
	size_t span = (size_t)data->data_offset;
	if ((size_t)data->status_offset > span) span = data->status_offset;
	if ((size_t)data->ctrl_offset > span) span = data->ctrl_offset;
	if (data->ecr_bar == data->bar && (size_t)data->ecr_offset > span)
		span = data->ecr_offset;
	volatile uint8_t *base = map_bar(data->pci, data->bar, span,
		&data->regs_map, &data->regs_len);
	if (!base) return -1;
	data->reg_data   = base + data->data_offset;
	data->reg_status = base + data->status_offset;
	data->reg_ctrl   = base + data->ctrl_offset;

	// Put the port in SPP/compatibility mode with its interrupts off, so
	// the data pins are plain outputs and nothing else drives them. The
	// ECR normally lives in the same BAR, in which case there is nothing
	// more to map.
	if (data->ecr_bar == data->bar) {
		data->ecr = base + data->ecr_offset;
		*data->ecr = (uint8_t)data->ecr_value;
	} else if (data->ecr_bar >= 0) {
		volatile uint8_t *ecr_base = map_bar(data->pci, data->ecr_bar,
			(size_t)data->ecr_offset, &data->ecr_map,
			&data->ecr_len);
		if (!ecr_base) return -1;
		data->ecr = ecr_base + data->ecr_offset;
		*data->ecr = (uint8_t)data->ecr_value;
	}
	// control bit 5 clear = data pins drive, bit 4 clear = no IRQ
	pp_write_ctrl(data, 0);

	if (data->probe) {
		// Park the SPI signals in their idle state and read the data
		// register back. A wrong `bar` reads as 0xFF (or garbage) and
		// would otherwise show up as a DAC that mysteriously ignores
		// us. This writes the idle pattern only -- no edges, so a DAC
		// already wired up cannot mistake it for a frame.
		uint8_t idle = data->link == AYLP_PARPORT_LINK_SPI
			? (uint8_t)((1u << data->sclk_bit)
				| (1u << data->sync_bit))
			: 0;
		pp_write_data(data, idle);
		uint8_t got = *data->reg_data;
		if (got != idle) {
			log_error("parport_dac: data register read back 0x%02x "
				"after writing 0x%02x -- wrong bar (%d) for %s?",
				got, idle, data->bar, data->pci);
			return -1;
		}
		log_info("parport_dac: %s BAR%d mapped, data register verified",
			data->pci, data->bar);
	}
	return 0;
}

static int open_ppdev(struct aylp_parport_dac_data *data)
{
	data->pp_fd = open(data->port, O_RDWR);
	if (data->pp_fd < 0) {
		log_error("parport_dac: open %s: %s",
			data->port, strerror(errno));
		return -1;
	}
	if (ioctl(data->pp_fd, PPCLAIM)) {
		log_error("parport_dac: PPCLAIM %s: %s",
			data->port, strerror(errno));
		return -1;
	}
	data->pp_claimed = true;
	int mode = IEEE1284_MODE_COMPAT;
	if (ioctl(data->pp_fd, PPSETMODE, &mode)) {
		log_error("parport_dac: PPSETMODE: %s", strerror(errno));
		return -1;
	}
	int dir = 0;	// forward: we drive the data pins
	if (ioctl(data->pp_fd, PPDATADIR, &dir)) {
		log_error("parport_dac: PPDATADIR: %s", strerror(errno));
		return -1;
	}
	pp_write_ctrl(data, 0);
	log_info("parport_dac: %s claimed via ppdev (an edge costs an ioctl; "
		"expect ~1 us each)", data->port);
	return 0;
}

/** Bring the DAC81404 up: device active, reference on, asynchronous update,
* ranges set, the configured channels powered. After a power-on or reset the
* device, its reference and all four output amplifiers are powered down
* (SLASEH2A 8.4.1), so skipping this leaves the outputs clamped to ground
* through 10 kohm and nothing appears to work. */
static void dac_configure(struct aylp_parport_dac_data *data)
{
	if (data->reset_at_init) {
		// deliberately NOT run through dac_write_checked(): TRIGGER's
		// fields are momentary actions (SLASEH2A table 8-17 types
		// them all "W", reset 0000h), so reading SOFT-RESET back
		// afterward is not expected to still show the 1010b we sent
		// -- a "mismatch" here would be a false alarm, not signal
		spi_frame(data, DAC_REG_TRIGGER, DAC_TRIGGER_SOFT_RESET);
		data->diag_frames++;
		// SLASEH2A 8.3.5: a soft reset is a POR event, and
		// "communication with the device is valid only after a 1 ms
		// POR delay". 5 ms is comfortably past that.
		struct timespec ts = {0, 5000000};	// 5 ms
		nanosleep(&ts, NULL);
	}
	// Each of these is checked over SDO/ACK when `verify` is set, same as
	// the per-channel writes below -- as far as each register allows, see
	// spi_write_verified() -- so a clean `verify` run exercises the whole
	// digital chain up front rather than only the last thing sent.
	//
	// SPICONFIG clears DEV-PWDWN (device-wide power-down, a separate and
	// higher-level gate than the GENCONFIG/DACPWDWN bits below, and set at
	// reset) -- see DAC_SPICONFIG_ACTIVE.
	uint16_t spiconfig = DAC_SPICONFIG_ACTIVE
		| (data->crc ? SPICONFIG_CRC_EN_BIT : 0);
	if (UNLIKELY(data->crc)) {
		// This one write can't go through dac_write_checked(): CRC-EN
		// takes effect on the write's own SYNC rising edge, so the
		// read-command and NOP frames a verified write would append
		// would still be 24 bits long while the device had already
		// started demanding 32. The device ignores an access cycle
		// with too few clock edges (SLASEH2A 8.5.1), so those two
		// frames would come back as a spurious "unconfirmed". Send it
		// plain, switch our own framing, then re-issue it -- writing
		// the same value twice is harmless, and the second one is in
		// the format the device now expects, so it can be verified.
		spi_frame(data, DAC_REG_SPICONFIG, spiconfig);
		data->diag_frames++;
		data->crc_active = true;
	}
	dac_write_checked(data, DAC_REG_SPICONFIG, spiconfig, "SPICONFIG");
	dac_write_checked(data, DAC_REG_GENCONFIG, DAC_GENCONFIG_REF_ON,
		"GENCONFIG");
	// Asynchronous update for every channel, so a DACx write moves the
	// output on the frame's own SYNC rising edge. This is the reset
	// default, but nothing else here would notice if it weren't.
	dac_write_checked(data, DAC_REG_SYNCCONFIG, DAC_SYNCCONFIG_ASYNC,
		"SYNCCONFIG");
	// DACRANGE holds all four channels' nibbles; channels this stage does
	// not drive keep the reset value (0 = 0 to 5 V)
	uint16_t ranges = 0;
	for (size_t i = 0; i < data->n_outputs; i++)
		ranges |= (uint16_t)((data->range_codes[i] & 0xF)
			<< (4 * data->channels[i]));
	dac_write_checked(data, DAC_REG_DACRANGE, ranges, "DACRANGE");
	// Power up only the channels this stage drives. The rest keep their
	// reset PWDWN bit and stay clamped to ground through the internal
	// 10 kohm (SLASEH2A 8.4.1) -- powering all four on unconditionally
	// released outputs whose range was never configured either.
	uint16_t pwdwn = DAC_PWDWN_NONE;
	for (size_t i = 0; i < data->n_outputs; i++)
		pwdwn &= (uint16_t)~(1u << data->channels[i]);
	dac_write_checked(data, DAC_REG_DACPWDWN, pwdwn, "DACPWDWN");
}

int parport_dac_init(struct aylp_device *self)
{
	self->proc = &parport_dac_proc;
	self->fini = &parport_dac_fini;
	struct aylp_parport_dac_data *data = xcalloc(1, sizeof *data);
	self->device_data = data;

	data->backend = AYLP_PARPORT_MMIO;
	data->link = AYLP_PARPORT_LINK_SPI;
	data->pci = xstrdup("0000:05:00.2");
	data->port = xstrdup("/dev/parport0");
	data->bar = 2;
	data->data_offset   = SPP_DATA_DEFAULT;
	data->status_offset = SPP_STATUS_DEFAULT;
	data->ctrl_offset   = SPP_CONTROL_DEFAULT;
	data->ecr_bar = 2;
	data->ecr_offset = SPP_ECR_DEFAULT;
	data->ecr_value = 0x14;		// SPP mode, interrupts disabled
	data->unbind = true;
	data->probe = true;
	data->pp_fd = -1;
	data->sclk_bit = 0;
	data->sdin_bit = 1;
	data->sync_bit = 2;
	data->clk_ctrl_bit = 0;
	data->chan_ctrl_shift = 1;
	data->reset_at_init = true;
	data->verify = false;
	data->ack_status_bit = SPP_STATUS_ACK_DEFAULT;
	data->crc = false;
	data->crc_active = false;

	struct json_object *ch_val = NULL, *idx_val = NULL;
	struct json_object *scale_val = NULL, *offset_val = NULL;
	struct json_object *delay_val = NULL, *range_val = NULL;

	if (self->params) {
		json_object_object_foreach(self->params, key, val) {
			if (key[0] == '_') {
			} else if (!strcmp(key, "backend")) {
				const char *s = json_object_get_string(val);
				if (!strcmp(s, "mmio")) {
					data->backend = AYLP_PARPORT_MMIO;
				} else if (!strcmp(s, "ppdev")) {
					data->backend = AYLP_PARPORT_PPDEV;
				} else {
					log_error("parport_dac: backend must "
						"be \"mmio\" or \"ppdev\"");
					return -1;
				}
			} else if (!strcmp(key, "link")) {
				const char *s = json_object_get_string(val);
				if (!strcmp(s, "spi")) {
					data->link = AYLP_PARPORT_LINK_SPI;
				} else if (!strcmp(s, "pico")) {
					data->link = AYLP_PARPORT_LINK_PICO;
				} else {
					log_error("parport_dac: link must be "
						"\"spi\" or \"pico\"");
					return -1;
				}
			} else if (!strcmp(key, "pci")) {
				xfree(data->pci);
				data->pci = xstrdup(json_object_get_string(val));
			} else if (!strcmp(key, "port")) {
				xfree(data->port);
				data->port = xstrdup(json_object_get_string(val));
			} else if (!strcmp(key, "bar")) {
				data->bar = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "data_offset")) {
				data->data_offset =
					(int)json_object_get_int64(val);
			} else if (!strcmp(key, "status_offset")) {
				data->status_offset =
					(int)json_object_get_int64(val);
			} else if (!strcmp(key, "ctrl_offset")) {
				data->ctrl_offset =
					(int)json_object_get_int64(val);
			} else if (!strcmp(key, "ecr_bar")) {
				data->ecr_bar = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "ecr_offset")) {
				data->ecr_offset = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "ecr_value")) {
				data->ecr_value = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "unbind")) {
				data->unbind = json_object_get_boolean(val);
			} else if (!strcmp(key, "probe")) {
				data->probe = json_object_get_boolean(val);
			} else if (!strcmp(key, "sclk_bit")) {
				data->sclk_bit = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "sdin_bit")) {
				data->sdin_bit = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "sync_bit")) {
				data->sync_bit = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "verify")) {
				data->verify = json_object_get_boolean(val);
			} else if (!strcmp(key, "ack_status_bit")) {
				data->ack_status_bit =
					(int)json_object_get_int64(val);
			} else if (!strcmp(key, "crc")) {
				data->crc = json_object_get_boolean(val);
			} else if (!strcmp(key, "clk_ctrl_bit")) {
				data->clk_ctrl_bit = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "chan_ctrl_shift")) {
				data->chan_ctrl_shift =
					(int)json_object_get_int64(val);
			} else if (!strcmp(key, "delay_ns")) {
				data->delay_ns = (long)json_object_get_int64(val);
			} else if (!strcmp(key, "channel")) {
				ch_val = val;
			} else if (!strcmp(key, "index")) {
				idx_val = val;
			} else if (!strcmp(key, "scale")) {
				scale_val = val;
			} else if (!strcmp(key, "offset")) {
				offset_val = val;
			} else if (!strcmp(key, "range")) {
				range_val = val;
			} else if (!strcmp(key, "start_delay")) {
				delay_val = val;
			} else if (!strcmp(key, "skip_unchanged")) {
				data->skip_unchanged =
					json_object_get_boolean(val);
			} else if (!strcmp(key, "reset_at_init")) {
				data->reset_at_init =
					json_object_get_boolean(val);
			} else {
				log_warn("parport_dac: unknown param \"%s\"",
					key);
			}
		}
	}

	// expand the per-channel params; the number of outputs is the longest
	// array given, and scalars are broadcast to every channel
	size_t n = val_count(ch_val);
	size_t counts[] = {
		val_count(idx_val), val_count(scale_val), val_count(offset_val),
		val_count(delay_val), val_count(range_val)
	};
	for (size_t k = 0; k < sizeof counts / sizeof counts[0]; k++)
		if (counts[k] > n) n = counts[k];

	data->n_outputs = n;
	data->channels      = xcalloc(n, sizeof *data->channels);
	data->indices       = xcalloc(n, sizeof *data->indices);
	data->scales        = xcalloc(n, sizeof *data->scales);
	data->offsets       = xcalloc(n, sizeof *data->offsets);
	data->start_delays  = xcalloc(n, sizeof *data->start_delays);
	data->released      = xcalloc(n, sizeof *data->released);
	data->vmin          = xcalloc(n, sizeof *data->vmin);
	data->vmax          = xcalloc(n, sizeof *data->vmax);
	data->range_codes   = xcalloc(n, sizeof *data->range_codes);
	data->last_codes    = xcalloc(n, sizeof *data->last_codes);
	for (size_t i = 0; i < n; i++) {
		data->channels[i] = val_int_at(ch_val, i, 0);
		if (data->channels[i] < 0 || data->channels[i] > 3) {
			log_error("parport_dac: channel must be 0-3 "
				"(0=DACA .. 3=DACD)");
			return -1;
		}
		data->indices[i] = val_int_at(idx_val, i, 0);
		if (data->indices[i] < 0) {
			log_error("parport_dac: index must be >= 0");
			return -1;
		}
		data->scales[i]  = val_dbl_at(scale_val, i, 1.0);
		data->offsets[i] = val_dbl_at(offset_val, i, 0.0);
		data->start_delays[i] = val_dbl_at(delay_val, i, 0.0);
		if (data->start_delays[i] < 0.0) {
			log_error("parport_dac: start_delay must be >= 0");
			return -1;
		}
		if (data->start_delays[i] > 0.0) data->has_start_delay = true;
		const char *rname = val_str_at(range_val, i, "0-10");
		const struct dac_range *r = find_range(rname);
		if (!r) {
			log_error("parport_dac: unknown range \"%s\" (try "
				"\"0-5\", \"0-10\", \"0-20\", \"0-40\", "
				"\"+-5\", \"+-10\", \"+-20\", ...)", rname);
			return -1;
		}
		data->range_codes[i] = r->code;
		data->vmin[i] = r->vmin;
		data->vmax[i] = r->vmax;
		data->last_codes[i] = -1;	// force a write on iteration 0
	}
	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			if (data->channels[i] != data->channels[j]) continue;
			log_error("parport_dac: channel %d is configured "
				"twice", data->channels[i]);
			return -1;
		}
	}
	if (data->link == AYLP_PARPORT_LINK_SPI) {
		int bits[] = {data->sclk_bit, data->sdin_bit, data->sync_bit};
		for (size_t i = 0; i < 3; i++) {
			if (bits[i] < 0 || bits[i] > 7) {
				log_error("parport_dac: sclk_bit, sdin_bit and "
					"sync_bit must be data pins 0-7");
				return -1;
			}
			for (size_t j = i + 1; j < 3; j++) {
				if (bits[i] != bits[j]) continue;
				log_error("parport_dac: sclk_bit, sdin_bit and "
					"sync_bit must be different data pins");
				return -1;
			}
		}
		if (data->verify
		&& (data->ack_status_bit < 0 || data->ack_status_bit > 7)) {
			log_error("parport_dac: ack_status_bit must be 0-7");
			return -1;
		}
		if (data->crc && !data->reset_at_init)
			log_warn("parport_dac: crc without reset_at_init -- the "
				"frame that sets CRC-EN has to be sent in the "
				"device's *current* format, and without a reset "
				"there is nothing guaranteeing that is the plain "
				"24-bit one (a device left in CRC mode by a "
				"previous run will desync here). Power-cycle or "
				"hardware-reset the DAC first");
	} else if (data->verify || data->crc) {
		log_error("parport_dac: verify/crc are only meaningful on the "
			"spi link -- the pico link never speaks the DAC's "
			"register protocol from the PC side");
		return -1;
	} else {
		if (data->clk_ctrl_bit < 0 || data->clk_ctrl_bit > 3
		|| data->chan_ctrl_shift < 0 || data->chan_ctrl_shift > 2) {
			log_error("parport_dac: clk_ctrl_bit must be 0-3 and "
				"chan_ctrl_shift 0-2 (four control lines)");
			return -1;
		}
		int chan_mask = 3 << data->chan_ctrl_shift;
		if (chan_mask & (1 << data->clk_ctrl_bit)) {
			log_error("parport_dac: clk_ctrl_bit %d collides with "
				"the channel-select lines at shift %d",
				data->clk_ctrl_bit, data->chan_ctrl_shift);
			return -1;
		}
	}

	if (data->backend == AYLP_PARPORT_MMIO) {
		if (open_mmio(data)) return -1;
	} else {
		if (open_ppdev(data)) return -1;
	}

	// in the spi link we own the DAC's configuration; in the pico link the
	// bridge firmware has already done it and only takes samples
	if (data->link == AYLP_PARPORT_LINK_SPI) dac_configure(data);
	// park every channel at its bias before the loop starts, so the mirror
	// doesn't see a step when the first command lands
	for (size_t i = 0; i < data->n_outputs; i++) {
		int code = volts_to_code(data->offsets[i],
			data->vmin[i], data->vmax[i]);
		dac_write_channel(data, data->channels[i], (uint16_t)code);
		data->last_codes[i] = code;
	}

	self->type_in   = AYLP_T_VECTOR;
	self->units_in  = AYLP_U_MINMAX;
	self->type_out  = AYLP_T_UNCHANGED;
	self->units_out = AYLP_U_UNCHANGED;

	log_info("parport_dac: %s link over %s, %zu output(s)%s",
		data->link == AYLP_PARPORT_LINK_SPI ? "bit-banged SPI"
			: "pico bridge",
		data->backend == AYLP_PARPORT_MMIO ? "mmio" : "ppdev",
		data->n_outputs,
		data->verify ? ", verifying every write over SDO/ACK "
			"(unconfirmed protocol assumption -- watch for "
			"echo-mismatch warnings)" : "");
	for (size_t i = 0; i < data->n_outputs; i++)
		log_info("parport_dac:   channel=%d (DAC%c) index=%d scale=%g "
			"offset=%g range=%g..%g V", data->channels[i],
			'A' + data->channels[i], data->indices[i],
			data->scales[i], data->offsets[i],
			data->vmin[i], data->vmax[i]);
	return 0;
}

int parport_dac_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_parport_dac_data *data = self->device_data;

	double now = monotonic_s();
	// measure the startup hold from the first proc, like piplate_bridge:
	// this is when the channels have all reached their bias
	if (UNLIKELY(!data->t0)) data->t0 = now;
	double elapsed = now - data->t0;

	for (size_t i = 0; i < data->n_outputs; i++) {
		int idx = data->indices[i];
		if (UNLIKELY((size_t)idx >= state->vector->size)) {
			log_error("parport_dac: index %d out of range "
				"(vector size %zu)", idx, state->vector->size);
			break;
		}
		double cmd = gsl_vector_get(state->vector, idx);
		// startup hold: park this channel at its bias until its
		// start_delay expires, so a fine channel doesn't chase the
		// large startup error into its rail while a coarse stage is
		// still walking the beam in
		if (UNLIKELY(data->has_start_delay && !data->released[i])) {
			if (elapsed < data->start_delays[i]) {
				cmd = 0.0;
			} else {
				data->released[i] = true;
				if (data->start_delays[i] > 0.0)
					log_info("parport_dac: channel %d "
						"released after %.2f s hold",
						data->channels[i], elapsed);
			}
		}
		int code = volts_to_code(
			data->offsets[i] + cmd * data->scales[i],
			data->vmin[i], data->vmax[i]);

		if (data->skip_unchanged && code == data->last_codes[i]) {
			// the DAC register already holds this code; a finer
			// command change than 1 LSB cannot move the output
			data->diag_skips++;
			continue;
		}
		dac_write_channel(data, data->channels[i], (uint16_t)code);
		data->last_codes[i] = code;
		log_trace("parport_dac: ch%d cmd=%g -> code %d (%.4f V)",
			data->channels[i], cmd, code,
			code_to_volts(code, data->vmin[i], data->vmax[i]));
	}

	if (UNLIKELY(!data->diag_t0)) data->diag_t0 = now;
	double dt = now - data->diag_t0;
	if (UNLIKELY(dt >= PARPORT_DAC_DIAG_PERIOD)) {
		char vbuf[160];
		size_t voff = 0;
		for (size_t i = 0; i < data->n_outputs && voff < sizeof vbuf;
		i++) {
			voff += (size_t)snprintf(vbuf + voff,
				sizeof vbuf - voff, " ch%d=%.4fV",
				data->channels[i], code_to_volts(
					data->last_codes[i], data->vmin[i],
					data->vmax[i]));
		}
		if (data->verify && (data->diag_verify_fail
		|| data->diag_verify_unconfirmed)) {
			log_info("parport_dac: %.0f frames/s | %.0f skips/s | "
				"%ld verify_fail | %ld verify_unconfirmed |%s",
				data->diag_frames / dt, data->diag_skips / dt,
				data->diag_verify_fail,
				data->diag_verify_unconfirmed, vbuf);
		} else {
			log_info("parport_dac: %.0f frames/s | %.0f skips/s |%s",
				data->diag_frames / dt, data->diag_skips / dt,
				vbuf);
		}
		data->diag_t0 = now;
		data->diag_frames = data->diag_skips = 0;
		data->diag_verify_fail = data->diag_verify_unconfirmed = 0;
	}

	return 0;
}

int parport_dac_fini(struct aylp_device *self)
{
	struct aylp_parport_dac_data *data = self->device_data;

	// leave the outputs where they are (the mirror is presumably still
	// holding a beam) but park the bus in its idle state
	if (data->reg_data || data->pp_fd >= 0) {
		if (data->link == AYLP_PARPORT_LINK_SPI)
			pp_write_data(data, (uint8_t)((1u << data->sclk_bit)
				| (1u << data->sync_bit)));
		pp_write_ctrl(data, 0);
	}
	if (data->ecr_map) munmap(data->ecr_map, data->ecr_len);
	if (data->regs_map) munmap(data->regs_map, data->regs_len);
	if (data->pp_fd >= 0) {
		if (data->pp_claimed) ioctl(data->pp_fd, PPRELEASE);
		close(data->pp_fd);
	}
	if (data->pci) xfree(data->pci);
	if (data->port) xfree(data->port);
	if (data->channels) xfree(data->channels);
	if (data->indices) xfree(data->indices);
	if (data->scales) xfree(data->scales);
	if (data->offsets) xfree(data->offsets);
	if (data->start_delays) xfree(data->start_delays);
	if (data->released) xfree(data->released);
	if (data->vmin) xfree(data->vmin);
	if (data->vmax) xfree(data->vmax);
	if (data->range_codes) xfree(data->range_codes);
	if (data->last_codes) xfree(data->last_codes);
	xfree(data);
	return 0;
}
