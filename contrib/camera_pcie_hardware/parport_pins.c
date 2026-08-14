// Park the parallel port's SPI pins at a static logic level, for metering.
//
// This drives SCLK, SDIN and SYNC -- the three DB25 data pins that carry the
// DAC81404's SPI bus -- to a fixed level and leaves them there. It emits no
// edges and no DAC frames, so it is purely a wiring/level check: put a meter
// or a scope probe on the pin and read what the card is actually delivering
// through the breakout, the covers and the cable.
//
// Independent of anyloop, exactly like contrib/camera_pcie_hardware/dac_square.c: it maps the
// AX99100's BAR itself and stores one byte to the SPP data register.
//
// LEVELS. The AX99100's D0-D7 are 3.3 V logic (its control lines are not --
// they come out at ~4.94 V IEEE-1284 levels, which is why the DAC is wired to
// data pins only; see the pinout comment in devices/parport_dac.c). So "high"
// here means ~3.3 V at the pin, and that is what you should measure. A pin
// reading near 5 V means you are on a control pin, not a data pin; near 0 V
// with --level high means the data register write did not land -- check the
// readback line this tool prints before you go looking at the wiring.
//
// PINOUT: D0-D7 are DB25 pins 2-9, ground is 18-25. DB25 pin 1 is /STROBE, a
// control line, NOT D0. As built: D0=SCLK (pin 2), D1=SDIN (pin 3),
// D2=SYNC (pin 4) -- the same defaults parport_dac.c uses.
//
// IS THIS SAFE WITH THE DAC CONNECTED? Yes. A DAC81404 frame is bracketed by
// SYNC falling and rising (SLASEH2A 8.5.1) and SDIN is sampled on SCLK
// edges. Holding all three static produces no edges at all, so the part sees
// nothing to latch. --level high in particular is the SPI idle state (SYNC
// high, SCLK high) with SDIN also parked high. --level low DOES hold SYNC
// asserted, which leaves the DAC's frame receiver open mid-cycle; it is still
// harmless with no clock, but the next real frame from anyloop starts with an
// idle store anyway, so nothing is left half-shifted.
//
// BUILD:
//   ninja -C build                 -> build/parport_pins (a meson target)
//   gcc -O2 -o parport_pins contrib/camera_pcie_hardware/parport_pins.c   (standalone)
// Not installed -- run it from the build tree.
//
// RUN (needs root for the BAR mapping, same as backend="mmio"):
//   sudo ./build/parport_pins                  # all three pins to 3.3 V
//   sudo ./build/parport_pins --hold           # ...and stay up until Ctrl-C
//   sudo ./build/parport_pins --level low      # all three to 0 V
//   sudo ./build/parport_pins --pins sdin      # only SDIN, leave the others
//
// The pins LATCH: the SPP data register keeps whatever was last stored to it,
// so the levels persist after this tool exits and the BAR is unmapped. That
// is deliberate -- you can set the pins and then take your time with the
// meter. Use --hold if you would rather the tool sit there holding the port
// (it keeps parport_pc unbound for the duration and restores nothing on
// exit either). Run anyloop, or dac_square, to take the port back.
//
// It unbinds parport_pc from the card by default, same as the driver. Rebind
// when you are done if you want /dev/parport0 back:
//   echo 0000:05:00.2 | sudo tee /sys/bus/pci/drivers/parport_pc/bind

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

// AX99100 SPP register offsets inside the memory BAR, and the ECR that puts
// the card in plain SPP mode -- identical to parport_dac.c's defaults.
#define SPP_DATA_OFFSET    0x280
#define SPP_CONTROL_OFFSET 0x288
#define SPP_ECR_OFFSET     0x2A8
#define SPP_ECR_VALUE      0x14		// SPP mode, interrupts disabled
#define SPP_BAR            2

// Control lines in pin polarity; all but INIT are inverted between register
// bit and pin, and bit 5 is the direction bit (clear = data pins drive).
#define CTRL_INVERT_MASK 0x0B
#define CTRL_WRITE_MASK  0x0F

static volatile sig_atomic_t stop_requested;

static void on_signal(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static volatile uint8_t *map_bar(const char *pci, int bar, size_t span,
	void **map_out, size_t *len_out)
{
	char path[128];
	snprintf(path, sizeof path,
		"/sys/bus/pci/devices/%s/resource%d", pci, bar);
	int fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		if (errno == EACCES || errno == EPERM)
			fprintf(stderr,
				"Mapping a BAR needs root (CAP_SYS_RAWIO), and "
				"kernel lockdown must be off -- check that\n"
				"`cat /sys/kernel/security/lockdown` shows "
				"[none] in brackets.\n");
		return NULL;
	}
	long page = sysconf(_SC_PAGESIZE);
	size_t len = (size_t)page;
	while (len < span + 1) len += (size_t)page;
	void *m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (m == MAP_FAILED) {
		fprintf(stderr, "mmap %s: %s\n", path, strerror(errno));
		return NULL;
	}
	*map_out = m;
	*len_out = len;
	return (volatile uint8_t *)m;
}

static void unbind_parport_pc(const char *pci)
{
	int fd = open("/sys/bus/pci/drivers/parport_pc/unbind", O_WRONLY);
	if (fd < 0) {
		if (errno != ENOENT)
			fprintf(stderr, "warning: open parport_pc unbind: "
				"%s\n", strerror(errno));
		return;
	}
	ssize_t w = write(fd, pci, strlen(pci));
	close(fd);
	// ENODEV just means it wasn't bound in the first place
	if (w < 0 && errno != ENODEV)
		fprintf(stderr, "warning: unbind %s: %s\n", pci,
			strerror(errno));
}

static void usage(const char *argv0)
{
	printf("usage: sudo %s [options]\n"
		"  --level high|low   level to drive (default high = ~3.3 V)\n"
		"  --pins LIST        comma-separated subset of "
			"sclk,sdin,sync (default all three)\n"
		"  --others keep|low  what to do with the data pins not named "
			"(default keep)\n"
		"  --hold             stay resident holding the port until "
			"Ctrl-C\n"
		"  --pci SLOT         PCI slot (default 0000:05:00.2)\n"
		"  --sclk-bit N       data pin carrying SCLK (default 0)\n"
		"  --sdin-bit N       data pin carrying SDIN (default 1)\n"
		"  --sync-bit N       data pin carrying SYNC (default 2)\n"
		"  --no-unbind        don't unbind parport_pc first\n"
		"  -h, --help         this\n", argv0);
}

int main(int argc, char **argv)
{
	const char *pci = "0000:05:00.2";
	int sclk_bit = 0, sdin_bit = 1, sync_bit = 2;
	bool high = true, hold = false, unbind = true, zero_others = false;
	bool want_sclk = true, want_sdin = true, want_sync = true;

	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		#define NEXT(what) (++i < argc ? argv[i] : \
			(fprintf(stderr, "%s needs an argument\n", what), \
			exit(1), ""))
		if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
			usage(argv[0]);
			return 0;
		} else if (!strcmp(a, "--level")) {
			const char *v = NEXT("--level");
			if (!strcmp(v, "high")) high = true;
			else if (!strcmp(v, "low")) high = false;
			else {
				fprintf(stderr, "--level must be high or low, "
					"not \"%s\"\n", v);
				return 1;
			}
		} else if (!strcmp(a, "--pins")) {
			char *v = strdup(NEXT("--pins"));
			want_sclk = want_sdin = want_sync = false;
			for (char *tok = strtok(v, ","); tok;
				tok = strtok(NULL, ",")) {
				if (!strcmp(tok, "sclk")) want_sclk = true;
				else if (!strcmp(tok, "sdin")) want_sdin = true;
				else if (!strcmp(tok, "sync")) want_sync = true;
				else {
					fprintf(stderr, "unknown pin \"%s\" -- "
						"expected sclk, sdin or "
						"sync\n", tok);
					return 1;
				}
			}
			free(v);
			if (!want_sclk && !want_sdin && !want_sync) {
				fprintf(stderr, "--pins named nothing\n");
				return 1;
			}
		} else if (!strcmp(a, "--others")) {
			const char *v = NEXT("--others");
			if (!strcmp(v, "keep")) zero_others = false;
			else if (!strcmp(v, "low")) zero_others = true;
			else {
				fprintf(stderr, "--others must be keep or low, "
					"not \"%s\"\n", v);
				return 1;
			}
		} else if (!strcmp(a, "--hold")) {
			hold = true;
		} else if (!strcmp(a, "--pci")) {
			pci = NEXT("--pci");
		} else if (!strcmp(a, "--sclk-bit")) {
			sclk_bit = atoi(NEXT("--sclk-bit"));
		} else if (!strcmp(a, "--sdin-bit")) {
			sdin_bit = atoi(NEXT("--sdin-bit"));
		} else if (!strcmp(a, "--sync-bit")) {
			sync_bit = atoi(NEXT("--sync-bit"));
		} else if (!strcmp(a, "--no-unbind")) {
			unbind = false;
		} else {
			fprintf(stderr, "unknown option \"%s\"\n", a);
			usage(argv[0]);
			return 1;
		}
		#undef NEXT
	}

	int bits[3] = {sclk_bit, sdin_bit, sync_bit};
	for (int i = 0; i < 3; i++) {
		if (bits[i] < 0 || bits[i] > 7) {
			fprintf(stderr, "sclk/sdin/sync must be data pins "
				"0-7, got %d\n", bits[i]);
			return 1;
		}
		for (int j = i + 1; j < 3; j++)
			if (bits[i] == bits[j]) {
				fprintf(stderr, "sclk/sdin/sync must be "
					"different data pins\n");
				return 1;
			}
	}

	struct sigaction sa = { .sa_handler = on_signal };
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	if (unbind) unbind_parport_pc(pci);
	if (stop_requested) return 0;

	void *map = NULL;
	size_t map_len = 0;
	volatile uint8_t *base = map_bar(pci, SPP_BAR, SPP_ECR_OFFSET,
		&map, &map_len);
	if (!base) return 1;
	volatile uint8_t *reg_data = base + SPP_DATA_OFFSET;
	volatile uint8_t *reg_ctrl = base + SPP_CONTROL_OFFSET;
	volatile uint8_t *reg_ecr  = base + SPP_ECR_OFFSET;

	*reg_ecr = SPP_ECR_VALUE;
	// control bit 5 clear = data pins drive, bit 4 clear = no IRQ
	*reg_ctrl = (uint8_t)(0 & CTRL_WRITE_MASK) ^ CTRL_INVERT_MASK;

	// In SPP forward mode the data register reads back what was last
	// written to it, so this recovers the levels the other data pins are
	// already sitting at and keeps them undisturbed under --others keep.
	uint8_t before = *reg_data;
	uint8_t mask = 0;
	if (want_sclk) mask |= (uint8_t)(1u << sclk_bit);
	if (want_sdin) mask |= (uint8_t)(1u << sdin_bit);
	if (want_sync) mask |= (uint8_t)(1u << sync_bit);
	uint8_t out = zero_others ? 0 : before;
	out = high ? (uint8_t)(out | mask) : (uint8_t)(out & ~mask);

	*reg_data = out;
	uint8_t got = *reg_data;

	printf("parport %s BAR%d data register: 0x%02X -> 0x%02X "
		"(read back 0x%02X)\n", pci, SPP_BAR, before, out, got);
	if (got != out) {
		fprintf(stderr, "data register did NOT take the write -- "
			"wrong bar/offset for %s, or the card is in a "
			"non-SPP mode. The pins are not where this says.\n",
			pci);
		munmap(map, map_len);
		return 1;
	}
	const char *lvl = high ? "HIGH (~3.3 V)" : "LOW (0 V)";
	static const char *names[3] = {"SCLK", "SDIN", "SYNC"};
	bool wanted[3] = {want_sclk, want_sdin, want_sync};
	for (int i = 0; i < 3; i++) {
		if (!wanted[i]) continue;
		printf("  %-4s  D%d = DB25 pin %d  %s\n",
			names[i], bits[i], bits[i] + 2, lvl);
	}
	printf("  ground on DB25 pins 18-25\n");

	if (hold) {
		printf("holding the port; Ctrl-C to release "
			"(levels stay latched).\n");
		fflush(stdout);
		while (!stop_requested) {
			struct timespec ts = {0, 200000000};
			nanosleep(&ts, NULL);
		}
		printf("released.\n");
	} else {
		printf("levels are latched in the port and persist after "
			"exit.\n");
	}

	munmap(map, map_len);
	return 0;
}
