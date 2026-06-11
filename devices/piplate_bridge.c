// PiPlate DAQC bridge plate device -- drives one DAC channel via SPI.
// Maps a pipeline vector element from MINMAX [-1, 1] to a voltage [0, 4] V.
// Uses the PiPlate frame protocol: GPIO 25 frame select, /dev/spidev0.1 at 500 kHz.
//
// Params:
//   board    -- PiPlate board address 0-7 (default 0)
//   channel  -- DAC output channel 0-3 (default 0)
//   index    -- pipeline vector element to command (default 0)
//   scale    -- volts per pipeline unit (default 2.0)
//   offset   -- volts at zero command (default 2.0)

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <json-c/json.h>

#include "anyloop.h"
#include "logging.h"
#include "piplate_bridge.h"
#include "xalloc.h"

#define PIPLATE_SPI_HZ   500000u
#define PIPLATE_GPIO_FS  25       // BCM GPIO 25, physical pin 22
#define PIPLATE_CMD_DAC  0x40u    // DAQC setDAC command

static int gpio_export(int pin)
{
	char buf[8];
	int n = snprintf(buf, sizeof buf, "%d", pin);

	// Export the pin; ignore EBUSY (already exported is fine).
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd < 0) return -1;
	write(fd, buf, (size_t)n);
	close(fd);

	char path[64];
	snprintf(path, sizeof path, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (fd < 0) return -1;
	write(fd, "out", 3);
	close(fd);

	snprintf(path, sizeof path, "/sys/class/gpio/gpio%d/value", pin);
	return open(path, O_RDWR);
}

static inline void gpio_set(int fd, int val)
{
	lseek(fd, 0, SEEK_SET);
	write(fd, val ? "1" : "0", 1);
}

static int dac_write(struct aylp_piplate_bridge_data *data, double volts)
{
	if (volts < 0.0) volts = 0.0;
	if (volts > 4.095) volts = 4.095;
	int raw = (int)(volts * 1000.0 + 0.5);  // millivolts, rounded to int

	uint8_t tx[4] = {
		(uint8_t)(data->board + 8),
		PIPLATE_CMD_DAC,
		(uint8_t)((data->channel << 4) | (raw >> 8)),
		(uint8_t)(raw & 0xFF),
	};
	uint8_t rx[4] = {0};
	struct spi_ioc_transfer tr = {
		.tx_buf        = (unsigned long)tx,
		.rx_buf        = (unsigned long)rx,
		.len           = 4,
		.speed_hz      = PIPLATE_SPI_HZ,
		.bits_per_word = 8,
	};

	gpio_set(data->gpio_fd, 0);
	usleep(100);
	int ret = ioctl(data->spi_fd, SPI_IOC_MESSAGE(1), &tr);
	usleep(100);
	gpio_set(data->gpio_fd, 1);

	return ret < 0 ? -1 : 0;
}

int piplate_bridge_init(struct aylp_device *self)
{
	self->proc = &piplate_bridge_proc;
	self->fini = &piplate_bridge_fini;
	struct aylp_piplate_bridge_data *data = xcalloc(1, sizeof *data);
	self->device_data = data;

	data->spi_fd  = -1;
	data->gpio_fd = -1;
	data->board   = 0;
	data->channel = 0;
	data->index   = 0;
	data->scale   = 2.0;
	data->offset  = 2.0;

	if (self->params) {
		json_object_object_foreach(self->params, key, val) {
			if (key[0] == '_') {
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

	data->spi_fd = open("/dev/spidev0.1", O_RDWR);
	if (data->spi_fd < 0) {
		log_error("piplate_bridge: open /dev/spidev0.1: %s", strerror(errno));
		return -1;
	}

	data->gpio_fd = gpio_export(PIPLATE_GPIO_FS);
	if (data->gpio_fd < 0) {
		log_error("piplate_bridge: GPIO %d setup failed: %s",
			PIPLATE_GPIO_FS, strerror(errno));
		return -1;
	}
	gpio_set(data->gpio_fd, 1);  // frame select idles high

	self->type_in   = AYLP_T_VECTOR;
	self->units_in  = AYLP_U_MINMAX;
	self->type_out  = AYLP_T_UNCHANGED;
	self->units_out = AYLP_U_UNCHANGED;

	log_info("piplate_bridge: board=%d channel=%d index=%d scale=%g offset=%g",
		data->board, data->channel, data->index, data->scale, data->offset);
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
		log_warn("piplate_bridge: SPI write failed: %s", strerror(errno));

	log_trace("piplate_bridge: cmd=%g → %.3f V", cmd, volts);
	return 0;
}

int piplate_bridge_fini(struct aylp_device *self)
{
	struct aylp_piplate_bridge_data *data = self->device_data;
	if (data->spi_fd  >= 0) close(data->spi_fd);
	if (data->gpio_fd >= 0) close(data->gpio_fd);
	xfree(data);
	return 0;
}
