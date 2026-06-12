#ifndef AYLP_DEVICES_PIPLATE_BRIDGE_H_
#define AYLP_DEVICES_PIPLATE_BRIDGE_H_

#include "anyloop.h"

struct aylp_piplate_bridge_data {
	int serial_fd;	// /dev/ttyACM0 (or configured port) file descriptor
	int index;	// which pipeline vector element to command
	int board;	// DAQC board address (0–7)
	int channel;	// DAC output channel (0–3)
	double scale;	// volts per pipeline unit
	double offset;	// volts at zero command
};

int piplate_bridge_init(struct aylp_device *self);
int piplate_bridge_proc(struct aylp_device *self, struct aylp_state *state);
int piplate_bridge_fini(struct aylp_device *self);

#endif
