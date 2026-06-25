#ifndef AYLP_DEVICES_DEVICE_H_
#define AYLP_DEVICES_DEVICE_H_

#include "devices/center_of_mass.h"
#include "devices/kdc101.h"
#include "devices/tic_t834.h"
#include "devices/clamp.h"
#include "devices/biquad_filter.h"
#include "devices/delay.h"
#include "devices/file_sink.h"
#include "devices/logger.h"
#include "devices/matmul.h"
#include "devices/pid.h"
#include "devices/poke.h"
#include "devices/remove_piston.h"
#include "devices/stop_after_count.h"
#include "devices/test_source.h"
#include "devices/udp_sink.h"
#include "devices/udp_source.h"
#include "devices/vonkarman_stream.h"
#include "devices/piplate_bridge.h"
#include "devices/bode_plot.h"

static const struct {
	const char *uri;
	int (*init_fun)(struct aylp_device *);
} init_map [] = {
	{ "anyloop:center_of_mass", center_of_mass_init },
	{ "anyloop:kdc101", kdc101_init },
	{ "anyloop:tic_t834", tic_t834_init },
	{ "anyloop:clamp", clamp_init },
	{ "anyloop:biquad_filter", biquad_filter_init },
	{ "anyloop:delay", delay_init },
	{ "anyloop:file_sink", file_sink_init },
	{ "anyloop:logger", logger_init },
	{ "anyloop:matmul", matmul_init },
	{ "anyloop:pid", pid_init },
	{ "anyloop:poke", poke_init },
	{ "anyloop:remove_piston", remove_piston_init },
	{ "anyloop:stop_after_count", stop_after_count_init },
	{ "anyloop:test_source", test_source_init },
	{ "anyloop:udp_sink", udp_sink_init },
	{ "anyloop:udp_source", udp_source_init },
	{ "anyloop:vonkarman_stream", vonkarman_stream_init },
	{ "anyloop:piplate_bridge", piplate_bridge_init },
	{ "anyloop:bode_plot", bode_plot_init },
};

// match an aylp_device with its initializer function and initialize it
int init_device(struct aylp_device *dev);

#endif

