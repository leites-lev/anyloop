// PiPlate BRIDGEplate device -- drives one DAQC2 DAC channel via USB serial.
//
// DAC mode (default): maps a pipeline vector element from MINMAX [-1, 1] to a
// voltage using offset + cmd*scale, and calls DAQC2.setDAC each iteration.
//
// FG mode (mode="fg"): configures the DAQC2 hardware function generator at
// init (fgTYPE/fgFREQ/fgLEVEL/fgON) and passes through each iteration with no
// USB traffic. The hardware updates its DAC at 200 kHz autonomously.
//
// Params (DAC mode):
//   port      -- serial device path (default "/dev/ttyACM0")
//   board     -- DAQC board address 0-7 (default 0)
//   channel   -- DAC output channel 0-3 (default 0)
//   index     -- pipeline vector element to command (default 0)
//   scale     -- volts per pipeline unit (default 2.0)
//   offset    -- volts at zero command (default 2.0)
//
// Params (FG mode, mode="fg"):
//   port      -- serial device path (default "/dev/ttyACM0")
//   board     -- DAQC board address 0-7 (default 0)
//   channel   -- FG channel 1 or 2 (default 1)
//   frequency -- Hz, 10–10000 (default 1000)
//   type      -- waveform: 1=sine 2=triangle 3=square 4=sawtooth
//                          5=inv_sawtooth 6=noise 7=sinc (default 1)
//   level     -- amplitude: 4=full 3=half 2=quarter 1=eighth (default 4)

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
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
	int fd = open(port, O_RDWR | O_NOCTTY);
	if (fd < 0) return -1;

	struct termios tty;
	if (tcgetattr(fd, &tty)) { close(fd); return -1; }
	cfmakeraw(&tty);
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 5;  // 500ms read timeout in 0.1s units
	if (tcsetattr(fd, TCSANOW, &tty)) { close(fd); return -1; }
	return fd;
}

static void drain_response(int fd)
{
	char c;
	while (read(fd, &c, 1) == 1 && c != '\n')
		;
}

static int send_cmd(int fd, const char *cmd)
{
	size_t n = strlen(cmd);
	if (write(fd, cmd, n) != (ssize_t)n) return -1;
	drain_response(fd);
	return 0;
}

static int dac_write(struct aylp_piplate_bridge_data *data, double volts)
{
	if (volts < 0.0) volts = 0.0;
	if (volts > 4.095) volts = 4.095;

	// drain the previous iteration's response before sending the next command;
	// this pipelines both channels' writes so the BRIDGEplate can process
	// channel N+1 while we're still waiting for channel N's response
	if (data->pending_drain)
		drain_response(data->serial_fd);

	char cmd[64];
	int n = snprintf(cmd, sizeof cmd,
		"DAQC2.setDAC(%d, %d, %.4f)\n", data->board, data->channel, volts);

	if (write(data->serial_fd, cmd, (size_t)n) != n)
		return -1;

	data->pending_drain = true;
	return 0;
}

int piplate_bridge_init(struct aylp_device *self)
{
	self->proc = &piplate_bridge_proc;
	self->fini = &piplate_bridge_fini;
	struct aylp_piplate_bridge_data *data = xcalloc(1, sizeof *data);
	self->device_data = data;

	data->serial_fd   = -1;
	data->board       = 0;
	data->channel     = 0;
	data->index       = 0;
	data->scale       = 1.0;
	data->offset      = 0;
	data->fg_type      = 1;
	data->fg_frequency = 1000;
	data->fg_level     = 4;

	const char *port = "/dev/ttyACM0";
	const char *mode = "dac";

	if (self->params) {
		json_object_object_foreach(self->params, key, val) {
			if (key[0] == '_') {
			} else if (!strcmp(key, "port")) {
				port = json_object_get_string(val);
			} else if (!strcmp(key, "mode")) {
				mode = json_object_get_string(val);
			} else if (!strcmp(key, "board")) {
				data->board = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "channel")) {
				data->channel = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "index")) {
				data->index = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "scale")) {
				data->scale = json_object_get_double(val);
			} else if (!strcmp(key, "offset")) {
				data->offset = json_object_get_double(val);
			} else if (!strcmp(key, "frequency")) {
				data->fg_frequency = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "type")) {
				data->fg_type = (int)json_object_get_int64(val);
			} else if (!strcmp(key, "level")) {
				data->fg_level = (int)json_object_get_int64(val);
			} else {
				log_warn("piplate_bridge: unknown param \"%s\"", key);
			}
		}
	}

	data->use_fg = !strcmp(mode, "fg");

	data->serial_fd = serial_open(port);
	if (data->serial_fd < 0) {
		log_error("piplate_bridge: open %s: %s", port, strerror(errno));
		return -1;
	}

	if (data->use_fg) {
		char cmd[64];
		snprintf(cmd, sizeof cmd, "DAQC2.fgTYPE(%d, %d, %d)\n",
			data->board, data->channel, data->fg_type);
		if (send_cmd(data->serial_fd, cmd)) goto serial_err;
		snprintf(cmd, sizeof cmd, "DAQC2.fgFREQ(%d, %d, %d)\n",
			data->board, data->channel, data->fg_frequency);
		if (send_cmd(data->serial_fd, cmd)) goto serial_err;
		snprintf(cmd, sizeof cmd, "DAQC2.fgLEVEL(%d, %d, %d)\n",
			data->board, data->channel, data->fg_level);
		if (send_cmd(data->serial_fd, cmd)) goto serial_err;
		snprintf(cmd, sizeof cmd, "DAQC2.fgON(%d, %d)\n",
			data->board, data->channel);
		if (send_cmd(data->serial_fd, cmd)) goto serial_err;

		self->type_in   = AYLP_T_ANY;
		self->units_in  = AYLP_U_ANY;
		self->type_out  = AYLP_T_UNCHANGED;
		self->units_out = AYLP_U_UNCHANGED;
		log_info("piplate_bridge: %s board=%d fg_channel=%d type=%d freq=%dHz level=%d",
			port, data->board, data->channel,
			data->fg_type, data->fg_frequency, data->fg_level);
	} else {
		// stop any FG that may still be running on this channel from a previous run
		char cmd[64];
		snprintf(cmd, sizeof cmd, "DAQC2.fgOFF(%d, %d)\n", data->board, data->channel);
		send_cmd(data->serial_fd, cmd);

		self->type_in   = AYLP_T_VECTOR;
		self->units_in  = AYLP_U_MINMAX;
		self->type_out  = AYLP_T_UNCHANGED;
		self->units_out = AYLP_U_UNCHANGED;
		log_info("piplate_bridge: %s board=%d channel=%d index=%d scale=%g offset=%g",
			port, data->board, data->channel,
			data->index, data->scale, data->offset);
	}
	return 0;

serial_err:
	log_error("piplate_bridge: serial command failed: %s", strerror(errno));
	return -1;
}

int piplate_bridge_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_piplate_bridge_data *data = self->device_data;

	if (data->use_fg)
		return 0;

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
	if (data->serial_fd >= 0) {
		if (data->use_fg) {
			char cmd[64];
			snprintf(cmd, sizeof cmd, "DAQC2.fgOFF(%d, %d)\n",
				data->board, data->channel);
			send_cmd(data->serial_fd, cmd);
		} else if (data->pending_drain) {
			drain_response(data->serial_fd);
		}
		close(data->serial_fd);
	}
	xfree(data);
	return 0;
}
