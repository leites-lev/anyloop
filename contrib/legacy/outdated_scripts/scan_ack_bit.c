// Standalone diagnostic: find which STATUS-register bit DB25 pin 10 (ACK)
// actually lands on for this AX99100 card. Deliberately independent of
// parport_dac.c and the DAC entirely -- it only tells you the pin<->bit
// mapping, so it can't be confused by anything upstream (power, DAC
// wiring, register state) being broken. That's the point: parport_dac's
// `ack_status_bit` default (6) is the generic SPP convention, unverified
// on this specific card, and this tool is how you verify it.
//
// Uses ppdev (/dev/parportN), NOT the mmio backend, on purpose:
//   - No root needed (just `lp` group membership on /dev/parportN).
//   - Does not unbind parport_pc, so it can't collide with anything else
//     that expects the kernel driver to own the port -- but that also
//     means parport_pc must currently OWN the device for /dev/parportN to
//     exist. If you've been running parport_dac with the mmio backend
//     (unbind: true, the default) and it's still unbound from a prior
//     run, rebind first:
//       echo 0000:05:00.2 | sudo tee /sys/bus/pci/drivers/parport_pc/bind
//
// BUILD:
//   gcc -O2 -o scan_ack_bit contrib/legacy/outdated_scripts/scan_ack_bit.c
//
// RUN:
//   ./scan_ack_bit [/dev/parportN]          # watch mode (hand-touched)
//   ./scan_ack_bit --auto [/dev/parportN]   # automatic (jumper left in)
//
// This tool drives the data register itself, so it always provides its
// own known-HIGH source: pins 2-9 are held HIGH in watch mode, and D0
// (pin 2) is toggled in --auto mode. Nothing else needs to be running.
//
// PROCEDURE (--auto, preferred -- no hand timing, and it correlates over
// many toggles so a one-off glitch can't fake an answer):
//   1. Disconnect pin 10 from the DAC/EVM -- you want to drive it
//      yourself with a known level, not trust the DAC.
//   2. Jumper DB25 pin 10 to DB25 pin 2 (D0) and leave it there.
//   3. Run with --auto. It toggles D0 low/high repeatedly and reports
//      which status bit(s) track it perfectly. That bit is the real
//      ack_status_bit.
//
// PROCEDURE (watch mode, if you'd rather do it by hand):
//   1. Disconnect pin 10 from the DAC/EVM, as above.
//   2. Start the tool. Pins 2-9 are now driven HIGH. It prints the
//      status byte only when it changes.
//   3. Touch a jumper from pin 10 to any of pins 2-9, then to a ground
//      pin (18-25).
//   4. Whichever bit flips between those two touches is the real
//      ack_status_bit.
//
// In either mode, if NOTHING changes, the pin isn't reaching the status
// register at all through this path (check `ecr_value`/SPP mode, or that
// this is really the AX99100 function and not a different parallel port
// in a multi-port system).
//
// The standard SPP status-byte layout is printed for reference, but the
// whole reason to run this is that this card is not guaranteed to match
// it -- trust the observed bit flip over the printed labels.

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/ppdev.h>
#include <linux/parport.h>

// Number of low/high toggles --auto correlates over. A real connection
// tracks all of them; noise or a floating pin won't.
#define AUTO_TOGGLES 12

int main(int argc, char **argv)
{
	int automode = 0, sweepmode = 0;
	const char *port = "/dev/parport0";
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--auto")) automode = 1;
		else if (!strcmp(argv[i], "--sweep")) sweepmode = 1;
		else port = argv[i];
	}

	int fd = open(port, O_RDWR);
	if (fd < 0) {
		perror("open");
		fprintf(stderr, "Is %s present, and are you in the 'lp' "
			"group (or root)? If parport_dac left the card "
			"unbound from parport_pc, rebind it first:\n"
			"  echo 0000:05:00.2 | sudo tee "
			"/sys/bus/pci/drivers/parport_pc/bind\n", port);
		return 1;
	}
	if (ioctl(fd, PPCLAIM)) {
		perror("PPCLAIM");
		return 1;
	}
	int mode = IEEE1284_MODE_COMPAT;
	if (ioctl(fd, PPSETMODE, &mode)) {
		perror("PPSETMODE");
		return 1;
	}

	// Forward direction, so the data pins are outputs we can drive.
	int dir = 0;
	if (ioctl(fd, PPDATADIR, &dir))
		perror("PPDATADIR (continuing anyway)");

	if (sweepmode) {
		// Which data pin the jumper actually landed on is a common
		// misread (DB25 numbering reverses between male and female),
		// so don't assume D0: walk all eight and report any status
		// bit that responds to any of them. Each bit is tested both
		// as a lone high against an all-low background and as a lone
		// low against an all-high one, so a pin held by something
		// else still shows up.
		printf("Sweep on %s: toggling each data pin D0-D7 (DB25 pins "
			"2-9) individually.\n", port);
		printf("Reports any status bit that responds to any of them.\n"
			"\n");
		int any = 0;
		for (int b = 0; b < 8; b++) {
			unsigned char lo = 0x00, hi = (unsigned char)(1u << b);
			unsigned char s_lo, s_hi, n_lo, n_hi;
			unsigned char nlo = 0xFF;
			unsigned char nhi = (unsigned char)~(1u << b);
			ioctl(fd, PPWDATA, &lo);   usleep(20000);
			ioctl(fd, PPRSTATUS, &s_lo);
			ioctl(fd, PPWDATA, &hi);   usleep(20000);
			ioctl(fd, PPRSTATUS, &s_hi);
			ioctl(fd, PPWDATA, &nlo);  usleep(20000);
			ioctl(fd, PPRSTATUS, &n_lo);
			ioctl(fd, PPWDATA, &nhi);  usleep(20000);
			ioctl(fd, PPRSTATUS, &n_hi);
			unsigned char d = (s_lo ^ s_hi) | (n_lo ^ n_hi);
			printf("  D%d (pin %d): status %02x/%02x  %02x/%02x",
				b, b + 2, s_lo, s_hi, n_lo, n_hi);
			if (d) {
				printf("   <-- RESPONDS:");
				for (int k = 7; k >= 0; k--)
					if ((d >> k) & 1)
						printf(" bit%d", k);
				any = 1;
			}
			putchar('\n');
		}
		unsigned char off = 0;
		ioctl(fd, PPWDATA, &off);
		if (!any)
			printf("\nNo data pin moved any status bit. Pin 10 is "
				"not reaching the status register at all.\n");
		ioctl(fd, PPRELEASE);
		close(fd);
		return any ? 0 : 2;
	}

	if (automode) {
		// Correlate each status bit against a D0 we control. A bit
		// that is 0 for every low and 1 for every high (or the exact
		// inverse -- some status lines are inverted in hardware) is
		// carrying pin 10.
		unsigned char ones[8] = {0}, zeros[8] = {0}, s;
		printf("Auto mode on %s: toggling D0 (pin 2) %d times.\n",
			port, AUTO_TOGGLES);
		printf("Jumper DB25 pin 10 -> pin 2 must be in place, and pin "
			"10 disconnected from the DAC.\n\n");
		for (int t = 0; t < AUTO_TOGGLES; t++) {
			unsigned char d = (t & 1) ? 0xFF : 0x00;
			if (ioctl(fd, PPWDATA, &d)) {
				perror("PPWDATA");
				return 1;
			}
			usleep(20000);	// settle; no speed pressure here
			if (ioctl(fd, PPRSTATUS, &s)) {
				perror("PPRSTATUS");
				return 1;
			}
			printf("  D0=%s  status = 0x%02x  ", d ? "HIGH" : "LOW ",
				s);
			for (int b = 7; b >= 0; b--)
				putchar((s >> b) & 1 ? '1' : '0');
			putchar('\n');
			for (int b = 0; b < 8; b++) {
				if ((s >> b) & 1) ones[b] |= d ? 2 : 1;
				else zeros[b] |= d ? 2 : 1;
			}
		}
		printf("\n");
		int found = 0;
		for (int b = 0; b < 8; b++) {
			// Tracks D0: high only when D0 high, low only when low.
			if (ones[b] == 2 && zeros[b] == 1) {
				printf("bit%d follows pin 10 (non-inverted) "
					"-- ack_status_bit = %d\n", b, b);
				found = 1;
			} else if (ones[b] == 1 && zeros[b] == 2) {
				printf("bit%d follows pin 10 INVERTED "
					"-- ack_status_bit = %d, and the "
					"driver must invert its SDO read\n",
					b, b);
				found = 1;
			}
		}
		if (!found)
			printf("No status bit tracked D0. Pin 10 is not "
				"reaching the status register on this path: "
				"check the jumper, that pin 10 really is "
				"disconnected from the DAC, and that this is "
				"the right parallel port.\n");
		unsigned char off = 0;
		ioctl(fd, PPWDATA, &off);
		ioctl(fd, PPRELEASE);
		close(fd);
		return found ? 0 : 2;
	}

	// Watch mode: hold pins 2-9 HIGH so there's something to touch.
	unsigned char high = 0xFF;
	if (ioctl(fd, PPWDATA, &high))
		perror("PPWDATA (no known-HIGH source available)");

	printf("Watching %s status register (Ctrl+C to stop).\n", port);
	printf("Data pins 2-9 are being driven HIGH for you to touch pin 10 "
		"against.\n");
	printf("Standard SPP bit meanings -- NOT assumed correct for this "
		"card, that's what we're checking:\n");
	printf("  bit7 BUSY(inverted)  bit6 ACK  bit5 PAPER-OUT  "
		"bit4 SELECT  bit3 ERROR  bits2-0 unused\n\n");
	printf("Only prints when the byte changes. Touch pin 10 to a known "
		"HIGH, then to GND, and watch which bit flips.\n\n");

	unsigned char last = 0xFF, cur;
	for (;;) {
		if (ioctl(fd, PPRSTATUS, &cur)) {
			perror("PPRSTATUS");
			break;
		}
		if (cur != last) {
			printf("status = 0x%02x  ", cur);
			for (int b = 7; b >= 0; b--)
				putchar((cur >> b) & 1 ? '1' : '0');
			printf("   changed:");
			for (int b = 7; b >= 0; b--)
				if (((cur >> b) & 1) != ((last >> b) & 1))
					printf(" bit%d", b);
			printf("\n");
			last = cur;
		}
		usleep(100000);	// 10 Hz is plenty for a hand-touched jumper
	}

	ioctl(fd, PPRELEASE);
	close(fd);
	return 0;
}
