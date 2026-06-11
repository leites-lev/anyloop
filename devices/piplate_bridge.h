#ifndef AYLP_DEVICES_PIPLATE_BRIDGE_H_
#define AYLP_DEVICES_PIPLATE_BRIDGE_H_

#include "anyloop.h"

struct aylp_piplate_bridge_data {
	int spi_fd;	// /dev/spidev0.1 file descriptor
	int gpio_fd;	// sysfs value fd for GPIO 25 (PiPlate frame select)
	int index;	// which pipeline vector element to command
	int board;	// PiPlate board address (0–7)
	int channel;	// DAC output channel (0–3)
	double scale;	// volts per pipeline unit
	double offset;	// volts at zero command
};

int piplate_bridge_init(struct aylp_device *self);
int piplate_bridge_proc(struct aylp_device *self, struct aylp_state *state);
int piplate_bridge_fini(struct aylp_device *self);

#endif
