#ifndef AYLP_DEVICES_PIPLATE_BRIDGE_H_
#define AYLP_DEVICES_PIPLATE_BRIDGE_H_

#include "anyloop.h"

struct aylp_piplate_bridge_data {
	int serial_fd;		// /dev/ttyACM0 (or configured port) file descriptor
	int board;		// DAQC board address (0–7)
	int channel;		// DAC output channel; DAC: 0–3, FG: 1–2
	bool use_fg;		// use hardware function generator instead of setDAC
	// DAC mode
	int index;		// which pipeline vector element to command
	double scale;		// volts per pipeline unit
	double offset;		// volts at zero command
	bool pending_drain;	// response outstanding from previous write
	// function generator mode
	int fg_type;		// 1=sine 2=triangle 3=square 4=sawtooth 5=inv_saw 6=noise 7=sinc
	int fg_frequency;	// Hz, 10–10000
	int fg_level;		// 4=full 3=half 2=quarter 1=eighth
};

int piplate_bridge_init(struct aylp_device *self);
int piplate_bridge_proc(struct aylp_device *self, struct aylp_state *state);
int piplate_bridge_fini(struct aylp_device *self);

#endif
