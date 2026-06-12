// PiPlate BRIDGEplate device -- drives one DAQC DAC channel via USB serial.
// Maps a pipeline vector element from MINMAX [-1, 1] to a voltage [0, 4] V.
// Sends "DAQC.setDAC(board, channel, voltage)\n" over /dev/ttyACM0 at 115200
// baud and drains the newline-terminated response each iteration.
//
// Params:
//   port     -- serial device path (default "/dev/ttyACM0")
//   board    -- DAQC board address 0-7 (default 0)
//   channel  -- DAC output channel 0-3 (default 0)
//   index    -- pipeline vector element to command (default 0)
//   scale    -- volts per pipeline unit (default 2.0)
//   offset   -- volts at zero command (default 2.0)

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <json-c/json.h>

#include "anyloop.h"
#include "logging.h"
#include "piplate_bridge.h"
#include "xalloc.h"

static int serial_open(const char *port)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) return -1;

	struct termios tty;
	if (tcgetattr(fd, &tty)) { close(fd); return -1; }
	cfmakeraw(&tty);
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 20;  // 2-second read timeout in 0.1s units
	if (tcsetattr(fd, TCSANOW, &tty)) { close(fd); return -1; }
	return fd;
}

static int dac_write(struct aylp_piplate_bridge_data *data, double volts)
{
	if (volts < 0.0) volts = 0.0;
	if (volts > 4.095) volts = 4.095;

	char cmd[64];
	int n = snprintf(cmd, sizeof cmd,
		"DAQC2.setDAC(%d, %d, %.4f)\n", data->board, data->channel, volts);

	if (write(data->serial_fd, cmd, (size_t)n) != n)
		return -1;

	// drain the response line so the port stays in sync
	char c;
	while (read(data->serial_fd, &c, 1) == 1 && c != '\n')
		;
	return 0;
}

int piplate_bridge_init(struct aylp_device *self)
{
	self->proc = &piplate_bridge_proc;
	self->fini = &piplate_bridge_fini;
	struct aylp_piplate_bridge_data *data = xcalloc(1, sizeof *data);
	self->device_data = data;

	data->serial_fd = -1;
	data->board     = 0;
	data->channel   = 0;
	data->index     = 0;
	data->scale     = 1.0;
	data->offset    = 0;

	const char *port = "/dev/ttyACM0";

	if (self->params) {
		json_object_object_foreach(self->params, key, val) {
			if (key[0] == '_') {
			} else if (!strcmp(key, "port")) {
				port = json_object_get_string(val);
				log_trace("piplate_bridge: port = %s", port);
			} else if (!strcmp(key, "board")) {
				data->board = (int)json_object_get_int64(val);
				log_trace("piplate_bridge: board = %d", data->board);
			} else if (!strcmp(key, "channel")) {
				data->channel = (int)json_object_get_int64(val);
				log_trace("piplate_bridge: channel = %d", data->channel);
			} else if (!strcmp(key, "index")) {
				data->index = (int)json_object_get_int64(val);
				log_trace("piplate_bridge: index = %d", data->index);
			} else if (!strcmp(key, "scale")) {
				data->scale = json_object_get_double(val);
				log_trace("piplate_bridge: scale = %g", data->scale);
			} else if (!strcmp(key, "offset")) {
				data->offset = json_object_get_double(val);
				log_trace("piplate_bridge: offset = %g", data->offset);
			} else {
				log_warn("piplate_bridge: unknown param \"%s\"", key);
			}
		}
	}

	data->serial_fd = serial_open(port);
	if (data->serial_fd < 0) {
		log_error("piplate_bridge: open %s: %s", port, strerror(errno));
		return -1;
	}

	self->type_in   = AYLP_T_VECTOR;
	self->units_in  = AYLP_U_MINMAX;
	self->type_out  = AYLP_T_UNCHANGED;
	self->units_out = AYLP_U_UNCHANGED;

	log_info("piplate_bridge: %s board=%d channel=%d index=%d scale=%g offset=%g",
		port, data->board, data->channel, data->index, data->scale, data->offset);
	return 0;
}

int piplate_bridge_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_piplate_bridge_data *data = self->device_data;

	if (UNLIKELY((size_t)data->index >= state->vector->size)) {
		log_error("piplate_bridge: index %d out of range (vector size %zu)",
			data->index, state->vector->size);
		return -1;
	}
	double cmd = gsl_vector_get(state->vector, data->index);
	double volts = data->offset + cmd * data->scale;

	if (UNLIKELY(dac_write(data, volts)))
		log_warn("piplate_bridge: serial write failed: %s", strerror(errno));

	log_trace("piplate_bridge: cmd=%g → %.4f V", cmd, volts);
	return 0;
}

int piplate_bridge_fini(struct aylp_device *self)
{
	struct aylp_piplate_bridge_data *data = self->device_data;
	if (data->serial_fd >= 0) close(data->serial_fd);
	xfree(data);
	return 0;
}
