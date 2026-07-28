#ifndef AYLP_DEVICES_PARPORT_DAC_H_
#define AYLP_DEVICES_PARPORT_DAC_H_

#include <stdbool.h>
#include <stdint.h>

#include "anyloop.h"

/** How we reach the parallel port's SPP registers. */
enum aylp_parport_backend {
	/** mmap the AX99100's memory BAR and store to it directly (fast). */
	AYLP_PARPORT_MMIO,
	/** ppdev ioctls on /dev/parportN (slow, but no root, no unbind). */
	AYLP_PARPORT_PPDEV,
};

/** What sits on the other end of the DB25. */
enum aylp_parport_link {
	/** DB25 wires are the DAC's SPI bus; we bit-bang the frames. */
	AYLP_PARPORT_LINK_SPI,
	/** DB25 carries a byte-parallel handshake to an RP2040/RP2350 that
	* does the SPI itself. */
	AYLP_PARPORT_LINK_PICO,
};

struct aylp_parport_dac_data {
	// ------------------------------------------------------------------
	// transport
	enum aylp_parport_backend backend;
	// mmio backend
	char *pci;		// PCI slot, e.g. "0000:05:00.2"
	int bar;		// memory BAR holding the SPP registers
	int data_offset;	// SPP data register offset within bar
	int status_offset;	// SPP status register offset within bar
	int ctrl_offset;	// SPP control register offset within bar
	int ecr_bar;		// memory BAR holding the ECP ECR (-1 = skip)
	int ecr_offset;		// ECR offset within ecr_bar
	int ecr_value;		// value written to ECR to force SPP mode
	bool unbind;		// unbind parport_pc from the PCI device first
	volatile uint8_t *reg_data;	// mapped SPP data register
	volatile uint8_t *reg_status;	// mapped SPP status register
	volatile uint8_t *reg_ctrl;	// mapped SPP control register
	void *regs_map;		// mmap base for the register BAR (page-aligned)
	size_t regs_len;
	volatile uint8_t *ecr;	// mapped ECR byte
	void *ecr_map;		// null when the ECR shares the register BAR
	size_t ecr_len;
	// ppdev backend
	char *port;		// e.g. "/dev/parport0"
	int pp_fd;
	bool pp_claimed;
	// last values written, so a partial update doesn't disturb other pins
	uint8_t data_shadow;
	uint8_t ctrl_shadow;	// logical (pin-level) control bits

	// ------------------------------------------------------------------
	// link
	enum aylp_parport_link link;
	// spi link: which data pin (D0-D7) carries each SPI signal
	int sclk_bit;
	int sdin_bit;
	int sync_bit;		// SYNC: active-low frame bracket (SLASEH2A pin name, no "Z")
	// pico link: which control line clocks a byte in, and where the
	// channel number sits
	int clk_ctrl_bit;
	int chan_ctrl_shift;
	long delay_ns;		// extra dwell after each edge (0 = none)
	// readback (spi link only): confirm each write by reading the
	// register back over SDO/ACK
	bool verify;
	int ack_status_bit;	// status-register bit carrying SDO (default 6)
	bool verify_confirmed[4];	// per DAC channel: logged its first OK yet?
	long diag_verify_fail;		// confirmed-wrong writes
	long diag_verify_unconfirmed;	// echo sanity check failed

	// ------------------------------------------------------------------
	// outputs -- one stage can drive all four DAC channels
	size_t n_outputs;
	int *channels;		// DAC channel per output: 0=DACA .. 3=DACD
	int *indices;		// which pipeline vector element to command
	double *scales;		// volts per pipeline unit
	double *offsets;	// volts at zero command
	double *start_delays;	// seconds to hold at `offset` before releasing
	bool *released;		// has this channel's startup hold expired?
	bool has_start_delay;
	double *vmin;		// low end of the channel's configured range
	double *vmax;		// high end of the channel's configured range
	int *range_codes;	// DACRANGE nibble per channel
	int *last_codes;	// last 16-bit code written (-1 = never)
	bool skip_unchanged;	// don't re-send a channel whose code is the same
	bool reset_at_init;	// issue a DAC soft reset before configuring
	bool probe;		// read the data register back to check the BAR
	double t0;		// CLOCK_MONOTONIC of the first proc (s)

	// diagnostics
	long diag_frames;	// DAC frames actually emitted
	long diag_skips;	// channels skipped (code unchanged)
	double diag_t0;
};

int parport_dac_init(struct aylp_device *self);
int parport_dac_proc(struct aylp_device *self, struct aylp_state *state);
int parport_dac_fini(struct aylp_device *self);

#endif
