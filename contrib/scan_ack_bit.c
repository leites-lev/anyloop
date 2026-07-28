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
//   gcc -O2 -o scan_ack_bit contrib/scan_ack_bit.c
//
// RUN:
//   ./scan_ack_bit [/dev/parportN]
//
// PROCEDURE:
//   1. Disconnect pin 10 from the DAC/EVM entirely for this test -- you
//      want to drive it yourself with a known level, not trust the DAC.
//   2. Start the tool. It prints the status byte only when it changes.
//   3. Touch a jumper wire from DB25 pin 10 to a data pin that's
//      currently driven HIGH (measure with a meter first -- with nothing
//      else running, all data pins idle LOW, so you'll need something
//      else driving one high, e.g. run parport_dac with backend "ppdev"
//      in the background, or use any other 3.3V logic-high source on the
//      card). Then touch it to a ground pin (18-25).
//   4. Whichever bit flips between those two touches is the real
//      ack_status_bit. If NOTHING changes, the pin isn't reaching the
//      status register at all through this path (check `ecr_value`/SPP
//      mode, or that this is really the AX99100 function and not a
//      different parallel port in a multi-port system).
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

int main(int argc, char **argv)
{
	const char *port = argc > 1 ? argv[1] : "/dev/parport0";

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

	printf("Watching %s status register (Ctrl+C to stop).\n", port);
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
