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
//     channel update (~123 ns per store, effective SCLK ~3.75 MHz, far under
//     the DAC's 50 MHz ceiling). Nothing but wires between the card and the
//     BP-DAC81404EVM, so this works today.
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
// wobbled. LDAC is never driven by this code at all -- it is hardwired low
// (jumper or a grounded wire), so the DAC updates its output the instant
// SYNC latches a DACx write; there is nothing for software to get right or
// wrong about its polarity.
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
//                store latency alone is usually enough setup/hold for the
//                DAC; raise it if the wiring is long or unterminated
//   verify    -- spi link only; after each write, read the register back
//                over SDO/ACK (DB25 pin 10 -> status register) and warn if
//                it does not match what we sent (default false). Costs two
//                extra 24-bit frames per write. dac_configure() forces
//                SPICONFIG.FSDO=1 so SDO updates on the same SCLK falling
//                edges this file already samples on, per SLASEH2A 8.6.4 --
//                confirmed against a local copy of the datasheet 2026-07-28.
//                Two things are still NOT confirmed, though: whether the
//                DAC's shift register preloads its first output bit before
//                any clock edge (the timing diagram for this is a vector
//                graphic pdftotext can't read, only the daisy-chain figure
//                had extractable bit labels), which would shift every
//                sampled bit by one position; and which AX99100 status
//                register bit ACK actually lands on for THIS card (that's
//                not in the DAC's datasheet at all -- it would need the
//                AX99100 datasheet or a bench measurement). See
//                dac_write_and_verify()'s echo check, which exists
//                specifically to catch both failure modes rather than
//                silently trusting misaligned data.
//   ack_status_bit -- status-register bit carrying SDO (default 6, the
//                standard SPP ACK bit position; unverified on this card,
//                see `verify` above)
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
// DACRANGE (the configured ranges) and DACPWDWN (channels on) before parking
// every channel at its `offset`. Codes are MSB-aligned straight binary
// across the selected range.

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
// DACPWDWN bits 3-0 are per-channel power-DOWN flags, set at reset
#define DAC_PWDWN_ALL_ON 0xFFF0
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

#define DAC_CODE_MAX 65535
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

/** Round a target voltage to the code the DAC will actually produce. Data are
* MSB-aligned straight binary across the configured range (SLASEH2A 8.6.12). */
static int volts_to_code(double volts, double vmin, double vmax)
{
	double frac = (volts - vmin) / (vmax - vmin);
	long c = lround(frac * DAC_CODE_MAX);
	if (c < 0) c = 0;
	if (c > DAC_CODE_MAX) c = DAC_CODE_MAX;
	return (int)c;
}

static double code_to_volts(int code, double vmin, double vmax)
{
	return vmin + (vmax - vmin) * code / DAC_CODE_MAX;
}

// ---------------------------------------------------------------------------
// parallel port register access

/** Spin for delay_ns after an edge. Zero (the default) skips the clock read
* entirely: an MMIO store already costs ~100-300 ns, which is orders of
* magnitude more than the DAC's 5 ns setup requirement. */
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
* When `data->verify` is set, we also sample the ACK/SDO status bit on each
* of the 24 falling edges and return the 24 bits shifted in, MSB first.
* Confirmed 2026-07-28 against a local copy of SLASEH2A (Table 8-2, 8-3):
* a read needs a read-command cycle followed by a second cycle to shift the
* answer out on SDO, which carries an echo of the *previous* cycle's
* R/W+address in its top 8 bits and that cycle's requested data in the
* bottom 16 -- hence a verified write is 3 calls (write, read-command, NOP
* to shift the answer out). SPICONFIG.FSDO is forced to 1 in dac_configure()
* specifically so "SDO updates on SCLK falling edges" (8.6.4) lines up with
* where this loop samples. Two things remain unconfirmed, on purpose not
* papered over: whether the DAC preloads its first output bit before any
* clock edge (would shift every sampled bit by one position -- the timing
* diagram for this is a vector graphic, not extractable text), and which
* AX99100 status-register bit ACK physically lands on for this card (not in
* the DAC datasheet at all). dac_write_and_verify() cross-checks the echoed
* R/W+address before trusting the data for exactly these reasons. */
static uint32_t spi_frame_rw(struct aylp_parport_dac_data *data, bool read,
	uint8_t addr, uint16_t value)
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
	uint32_t frame = ((uint32_t)(read ? 1 : 0) << 23)
		| ((uint32_t)(addr & 0x3F) << 16) | value;
	// other data pins keep whatever they were left at
	uint8_t base = data->data_shadow & (uint8_t)~(sclk | sdin | sync);
	uint32_t rx = 0;

	pp_write_data(data, base | sync | sclk);	// idle
	pp_write_data(data, base | sclk);		// SYNC low: cycle start
	for (int i = 23; i >= 0; i--) {
		uint8_t bit = (frame >> i) & 1 ? sdin : 0;
		pp_write_data(data, base | sclk | bit);
		pp_write_data(data, base | bit);	// falling edge samples
		if (UNLIKELY(data->verify))
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
	spi_frame_rw(data, false, addr, value);
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

/** Write a DACx register and read it straight back over SDO/ACK to confirm
* the frame landed. Costs 3 frames total: the write, a read-command for the
* same address, and a NOP frame to shift the answer out (see spi_frame_rw()'s
* doc comment for why it takes a third frame). Logs and returns false on a
* mismatch; also logs (but does not fail on) an echo mismatch, which means
* the readback itself could not be trusted rather than that the write was
* wrong -- see the big caveat on the `verify` param. */
static bool dac_write_and_verify(struct aylp_parport_dac_data *data,
	int channel, uint16_t code)
{
	uint8_t addr = (uint8_t)(DAC_REG_DAC0 + channel);
	spi_frame_rw(data, false, addr, code);		// the real write
	spi_frame_rw(data, true, addr, 0x0000);	// request readback
	uint32_t rx = spi_frame_rw(data, false, DAC_REG_NOP, 0x0000);
	data->diag_frames += 2;	// +1 more from dac_write_channel's own

	uint8_t echo_rw = (uint8_t)((rx >> 23) & 1u);
	uint8_t echo_addr = (uint8_t)((rx >> 16) & 0x3Fu);
	uint16_t got = (uint16_t)(rx & 0xFFFFu);
	if (echo_rw != 1 || echo_addr != (addr & 0x3F)) {
		log_warn("parport_dac: readback echo mismatch on channel %d "
			"(got rw=%d addr=0x%02x, expected rw=1 addr=0x%02x) -- "
			"ack_status_bit or frame timing is probably wrong; "
			"verify is unconfirmed until this is fixed",
			channel, echo_rw, echo_addr, addr & 0x3F);
		data->diag_verify_unconfirmed++;
		return true;
	}
	if (got != code) {
		log_warn("parport_dac: channel %d write not confirmed: wrote "
			"0x%04x, read back 0x%04x", channel, code, got);
		data->diag_verify_fail++;
		return false;
	}
	if (!data->verify_confirmed[channel]) {
		data->verify_confirmed[channel] = true;
		double volts = NAN;
		for (size_t i = 0; i < data->n_outputs; i++) {
			if (data->channels[i] == channel) {
				volts = code_to_volts(code, data->vmin[i],
					data->vmax[i]);
				break;
			}
		}
		log_info("parport_dac: channel %d write confirmed by SDO/ACK "
			"readback: code 0x%04x (%.4f V)", channel, code, volts);
	}
	return true;
}

/** Send one channel's 16-bit code, whichever link is configured. */
static void dac_write_channel(struct aylp_parport_dac_data *data, int channel,
	uint16_t code)
{
	if (data->link == AYLP_PARPORT_LINK_SPI) {
		if (UNLIKELY(data->verify))
			dac_write_and_verify(data, channel, code);
		else
			spi_frame(data, (uint8_t)(DAC_REG_DAC0 + channel),
				code);
	} else {
		pico_frame(data, channel, code);
	}
	data->diag_frames++;
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

/** Bring the DAC81404 up: reference on, ranges set, outputs powered.
* After a power-on or reset the reference and all four output amplifiers are
* powered down (SLASEH2A 8.4.x), so skipping this leaves the outputs clamped
* to ground through 10 kohm and nothing appears to work. */
static void dac_configure(struct aylp_parport_dac_data *data)
{
	if (data->reset_at_init) {
		spi_frame(data, DAC_REG_TRIGGER, DAC_TRIGGER_SOFT_RESET);
		struct timespec ts = {0, 5000000};	// 5 ms
		nanosleep(&ts, NULL);
	}
	// clears SPICONFIG.DEV-PWDWN (device-wide power-down, separate from
	// GENCONFIG/DACPWDWN below and set at reset) -- see DAC_SPICONFIG_ACTIVE
	spi_frame(data, DAC_REG_SPICONFIG, DAC_SPICONFIG_ACTIVE);
	spi_frame(data, DAC_REG_GENCONFIG, DAC_GENCONFIG_REF_ON);
	// DACRANGE holds all four channels' nibbles; channels this stage does
	// not drive keep the reset value (0 = 0 to 5 V)
	uint16_t ranges = 0;
	for (size_t i = 0; i < data->n_outputs; i++)
		ranges |= (uint16_t)((data->range_codes[i] & 0xF)
			<< (4 * data->channels[i]));
	spi_frame(data, DAC_REG_DACRANGE, ranges);
	spi_frame(data, DAC_REG_DACPWDWN, DAC_PWDWN_ALL_ON);
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
	} else if (data->verify) {
		log_error("parport_dac: verify is only meaningful on the spi "
			"link -- the pico link never speaks the DAC's register "
			"protocol from the PC side");
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
