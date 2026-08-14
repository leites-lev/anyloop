#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <alsa/asoundlib.h>

#include "anyloop.h"
#include "logging.h"
#include "xalloc.h"
#include "aylp_alsa.h"


/** Sets hardware parameters from the data struct.
 * Specifically, sets: access, format, channels, rate, buffer time/size, period
 * time/size.
 */
static int set_hwparams(struct aylp_alsa_data *data)
{
	int err;
	int dir;	// we don't really care for now but see alsa docs
	snd_pcm_t *handle = data->handle;
	snd_pcm_hw_params_t *params;
	snd_pcm_hw_params_alloca(&params);

	err = snd_pcm_hw_params_any(handle, params);
	if (err < 0) {
		log_error("Broken configuration for playback: "
			"no configurations available: %s", snd_strerror(err)
		);
		return err;
	}

	err = snd_pcm_hw_params_set_access(handle, params, data->access);
	if (err < 0) {
		log_error("Access type not available for playback: %s",
			snd_strerror(err)
		);
		return err;
	}

	err = snd_pcm_hw_params_set_format(handle, params, data->format);
	if (err < 0) {
		log_error("Sample format not available for playback: %s",
			snd_strerror(err)
		);
		return err;
	}

	err = snd_pcm_hw_params_set_channels(handle, params, data->channels);
	if (err < 0) {
		log_error("Channels count (%u) not available: %s",
			data->channels, snd_strerror(err)
		);
		return err;
	}

	unsigned rrate = data->rate;
	err = snd_pcm_hw_params_set_rate_near(handle, params, &data->rate, 0);
	if (err < 0) {
		log_error("Rate (%u Hz) not available for playback: %s",
			data->rate, snd_strerror(err)
		);
		return err;
	}
	if (rrate != data->rate) {
		log_warn("Rate doesn't match (requested %u Hz, got %i Hz)",
			rrate, data->rate
		);
	}

	err = snd_pcm_hw_params_set_buffer_time_near(handle,
		params, &data->buffer_time, &dir
	);
	if (err < 0) {
		log_error("Unable to set buffer time %u for playback: %s",
			data->buffer_time, snd_strerror(err)
		);
		return err;
	}
	err = snd_pcm_hw_params_get_buffer_size(params, &data->buffer_size);
	if (err < 0) {
		log_error("Unable to get buffer size for playback: %s",
			snd_strerror(err)
		);
		return err;
	}
	log_trace("Buffer size set to %lu", data->buffer_size);

	err = snd_pcm_hw_params_set_period_time_near(handle, params,
		&data->period_time, &dir
	);
	if (err < 0) {
		log_error("Unable to set period time %u for playback: %s",
			data->period_time, snd_strerror(err)
		);
		return err;
	}
	err = snd_pcm_hw_params_get_period_size(params, &data->period_size,
		&dir	// I don't understand how we can't get it exactly??
	);
	if (err < 0) {
		log_error("Unable to get period size for playback: %s",
			snd_strerror(err)
		);
		return err;
	}
	log_trace("Period size set to %lu", data->period_size);

	// write the parameters to device
	err = snd_pcm_hw_params(handle, params);
	if (err < 0) {
		log_error("Unable to set hw params for playback: %s",
			snd_strerror(err)
		);
		return err;
	}

	// https://github.com/alsa-project/alsa-lib/issues/341
	//snd_pcm_hw_params_free(params);
	return 0;
}

/** Sets software parameters based on hardware parameters. */
static int set_swparams(struct aylp_alsa_data *data)
{
	int err;
	snd_pcm_t *handle = data->handle;
	snd_pcm_sw_params_t *params;
	snd_pcm_sw_params_alloca(&params);

	// get the current swparams
	err = snd_pcm_sw_params_current(handle, params);
	if (err < 0) {
		log_error("Unable to determine current swparams for playback: "
			"%s", snd_strerror(err)
		);
		return err;
	}

	// start threshold: in low-latency mode auto-start as soon as we have
	// topped the queue up to latency_frames (so the queue stays shallow);
	// otherwise start when the buffer is almost full.
	snd_pcm_uframes_t start_th = data->latency_frames
		? data->latency_frames
		: (data->buffer_size / data->period_size) * data->period_size;
	err = snd_pcm_sw_params_set_start_threshold(handle, params, start_th);
	if (err < 0) {
		log_error("Unable to set start threshold mode for playback: "
			"%s", snd_strerror(err)
		);
		return err;
	}

	// allow the transfer when at least period_size samples can be processed
	err = snd_pcm_sw_params_set_avail_min(handle, params,
		data->period_size
	);
	if (err < 0) {
		log_error("Unable to set avail min for playback: %s",
			snd_strerror(err)
		);
		return err;
	}

	// since we expect to underrun arbitrarily often, disable xrun check
	snd_pcm_uframes_t boundary;
	snd_pcm_sw_params_get_boundary(params, &boundary);
	snd_pcm_sw_params_set_stop_threshold(handle, params, boundary);

	// write the parameters to the playback device
	err = snd_pcm_sw_params(handle, params);
	if (err < 0) {
		log_error("Unable to set sw params for playback: %s",
			snd_strerror(err)
		);
		return err;
	}

	// https://github.com/alsa-project/alsa-lib/issues/341
	//snd_pcm_sw_params_free(params);
	return 0;
}


/** Scale + clamp the pipeline vector into a per-channel command array. */
static inline void scale_cmd(struct aylp_alsa_data *data,
	struct aylp_state *state, double *out)
{
	for (unsigned c = 0; c < data->channels; c++) {
		double f = state->vector->data[c] * data->scale[c];
		if (UNLIKELY(f > 1.0)) f = 1.0;
		else if (UNLIKELY(f < -1.0)) f = -1.0;
		out[c] = f;
	}
}


/** MMAP-write exactly `want` frames of `cmd` (already scaled+clamped, one value
 * per channel) into the ring. Every frame gets the same held value (zero-order
 * hold). Returns 0 on success or a negative error; sets needs_start on a broken
 * commit. */
static int fill_command(struct aylp_alsa_data *data, const double *cmd,
	snd_pcm_uframes_t want)
{
	int err;
	snd_pcm_uframes_t offset, frames, size;
	const snd_pcm_channel_area_t *my_areas;
	size = want;
	while (size > 0) {
		frames = size;
		err = snd_pcm_mmap_begin(data->handle,
			&my_areas, &offset, &frames
		);
		if (err < 0) {
			log_error("mmap_begin error: %s", snd_strerror(err));
			data->needs_start = true;
			return err;
		}
		if (frames == 0) break;	// no room right now
		unsigned char *samples[data->channels];
		for (unsigned c = 0; c < data->channels; c++) {
			if (my_areas[c].first & 0x8) {
				log_error("areas[%u].first == %u, aborting",
					c, my_areas[c].first
				);
				return -1;
			}
			samples[c] = (unsigned char *)my_areas[c].addr
				+ (my_areas[c].first / 8);
			if (my_areas[c].step & 0xF) {
				log_error("areas[%u].step == %u, aborting",
					c, my_areas[c].step
				);
				return -1;
			}
			samples[c] += offset * my_areas[c].step/8;
		}
		for (int count = frames-1; count >= 0; count--) {
			for (unsigned c = 0; c < data->channels; c++) {
				double f = cmd[c];
				// map command [-1,1] to the FULL DAC range:
				// signed -> [-maxval, maxval]; unsigned -> [0, max].
				int res = data->maxval;
				if (data->to_unsigned) res *= f/2+0.5;
				else res *= f;
				if (UNLIKELY(data->big_endian)) {
					for (int i=0; i < data->format_bits/8;
					i++) {
						*(samples[c]
						+ data->phys_bps - 1 - i)
							= (res >> i*8) & 0xFF;
					}
				} else {
					for (int i=0; i < data->format_bits/8;
					i++) {
						*(samples[c] + i)
							= (res >> i*8) & 0xFF;
					}
				}
				samples[c] += my_areas[c].step/8;
			}
		}
		snd_pcm_sframes_t committed = snd_pcm_mmap_commit(data->handle,
			offset, frames
		);
		if (UNLIKELY(committed < 0
		|| (snd_pcm_uframes_t)committed != frames)) {
			log_warn("mmap_commit error: %s", snd_strerror(committed));
			data->needs_start = true;
			return committed < 0 ? committed : -1;
		}
		size -= frames;
	}
	return 0;
}


/** LOW-LATENCY target-fill: keep exactly data->latency_frames queued ahead of
 * the DAC (measured with snd_pcm_delay), topping up with the current command.
 * The PCM auto-starts at start_threshold == latency_frames. Reports the live
 * queue latency periodically. */
static int process_lowlat(struct aylp_alsa_data *data, struct aylp_state *state)
{
	int err;
	snd_pcm_state_t st = snd_pcm_state(data->handle);
	if (UNLIKELY(st == SND_PCM_STATE_XRUN || st == SND_PCM_STATE_SETUP)) {
		// underran (buffer starved) -- re-prepare; it auto-starts again
		// once we have topped it back up to the threshold.
		if ((err = snd_pcm_prepare(data->handle)) < 0) {
			log_warn("prepare after xrun failed: %s",
				snd_strerror(err));
			return 0;
		}
		data->needs_start = true;
	}

	snd_pcm_sframes_t delay = 0;
	err = snd_pcm_delay(data->handle, &delay);
	if (UNLIKELY(err < 0)) { snd_pcm_prepare(data->handle); delay = 0; }
	if (delay < 0) delay = 0;

	snd_pcm_sframes_t avail = snd_pcm_avail_update(data->handle);
	if (UNLIKELY(avail < 0)) { snd_pcm_prepare(data->handle); return 0; }

	// top up to the target queue depth (no more), so latency stays minimal
	if ((snd_pcm_uframes_t)delay < data->latency_frames) {
		snd_pcm_uframes_t want = data->latency_frames
			- (snd_pcm_uframes_t)delay;
		if (want > (snd_pcm_uframes_t)avail)
			want = (snd_pcm_uframes_t)avail;
		if (want) {
			scale_cmd(data, state, data->fill_scratch);
			err = fill_command(data, data->fill_scratch, want);
			if (err < 0) return err;
		}
	}

	// live latency report: snd_pcm_delay = frames from the sample we just
	// wrote to when it is heard = the command->output latency.
	if (UNLIKELY(++data->report_ctr % 20000 == 0)) {
		snd_pcm_sframes_t d = 0;
		if (snd_pcm_delay(data->handle, &d) == 0)
			log_info("aylp_alsa: queue latency %ld fr = %.3f ms "
				"(target %lu fr)", (long)d,
				1e3 * (double)d / data->rate,
				data->latency_frames);
	}
	return 0;
}


/** Process one period according to data and state. */
static int process_period(struct aylp_alsa_data *data, struct aylp_state *state)
{
	int err;
	// check for suspend event
	if (UNLIKELY(snd_pcm_state(data->handle) == SND_PCM_STATE_SUSPENDED)) {
		log_warn("Detected suspend event");
		// wait until suspend flag is released
		while ((err = snd_pcm_resume(data->handle)) == -EAGAIN)
			sleep(1);
		if (err < 0) {
			err = snd_pcm_prepare(data->handle);
			if (err < 0) {
				log_error("Can't recover from suspend; prepare "
					"failed: %s", snd_strerror(err)
				);
				return err;
			}
		}
	}

	// make sure we have a period available
	snd_pcm_uframes_t avail = snd_pcm_avail_update(data->handle);
	if (UNLIKELY((snd_pcm_sframes_t)avail < 0)) {
		log_warn("Failed to check availability: %s",
			snd_strerror(avail)
		);
		data->needs_start = true;
		return avail;
	} else if (UNLIKELY(avail < data->period_size)) {
		if (data->needs_start) {
			data->needs_start = false;
			log_trace("Starting pcm");
			err = snd_pcm_start(data->handle);
			if (err < 0) {
				log_error("Start error: %s", snd_strerror(err));
				return err;
			}
		} else if (data->blocking) {
			// legacy behaviour: wait for the card, which PACES the
			// whole anyloop loop at the sound-card period rate.
			err = snd_pcm_wait(data->handle, -1);
			if (err < 0) {
				log_warn("snd_pcm_wait error: %s",
					snd_strerror(err)
				);
				data->needs_start = true;
				return err;
			}
		}
		// Non-blocking (default): the buffer has no room for a full
		// period right now, so we drop this iteration's write and get on
		// with the loop rather than stalling it. The card keeps playing
		// the freshest samples already buffered. This keeps the loop
		// paced by the real source (the camera), not by ALSA.
		return 0;
	}

	// write one period of the held command
	scale_cmd(data, state, data->fill_scratch);
	return fill_command(data, data->fill_scratch, data->period_size);
}


/** Dedicated feeder: refills the card in a tight loop to hold the queue at
 * latency_frames, independent of the (camera-paced) anyloop loop. This is what
 * makes a sub-loop-period queue sustainable without underrunning. */
static void *feeder_thread_fn(void *arg)
{
	struct aylp_alsa_data *data = arg;
	if (data->rt_prio > 0) {
		struct sched_param sp = { .sched_priority = data->rt_prio };
		if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
			log_warn("aylp_alsa: SCHED_FIFO prio %d failed (%s); "
				"need privilege / RLIMIT_RTPRIO -- feeder runs "
				"at normal priority and may miss deadlines",
				data->rt_prio, strerror(errno));
		else
			log_info("aylp_alsa: feeder at SCHED_FIFO prio %d",
				data->rt_prio);
	}
	struct timespec ts = { 0, (long)data->poll_us * 1000L };
	double local[data->channels];
	while (atomic_load_explicit(&data->feeder_run, memory_order_acquire)) {
		snd_pcm_state_t st = snd_pcm_state(data->handle);
		if (UNLIKELY(st == SND_PCM_STATE_XRUN
		|| st == SND_PCM_STATE_SETUP)) {
			snd_pcm_prepare(data->handle);
			data->xrun_count++;
		}
		snd_pcm_sframes_t delay = 0;
		if (UNLIKELY(snd_pcm_delay(data->handle, &delay) < 0)) {
			snd_pcm_prepare(data->handle);
			delay = 0;
		}
		if (delay < 0) delay = 0;
		snd_pcm_sframes_t avail = snd_pcm_avail_update(data->handle);
		if (UNLIKELY(avail < 0)) {
			snd_pcm_prepare(data->handle);
			nanosleep(&ts, NULL);
			continue;
		}
		if ((snd_pcm_uframes_t)delay < data->latency_frames) {
			snd_pcm_uframes_t want = data->latency_frames
				- (snd_pcm_uframes_t)delay;
			if (want > (snd_pcm_uframes_t)avail)
				want = (snd_pcm_uframes_t)avail;
			if (want) {
				pthread_mutex_lock(&data->cmd_lock);
				for (unsigned c = 0; c < data->channels; c++)
					local[c] = data->shared_cmd[c];
				pthread_mutex_unlock(&data->cmd_lock);
				fill_command(data, local, want);
			}
		}
		nanosleep(&ts, NULL);
	}
	return NULL;
}


int aylp_alsa_init(struct aylp_device *self)
{
	int err;
	self->device_data = xcalloc(1, sizeof(struct aylp_alsa_data));
	struct aylp_alsa_data *data = self->device_data;
	// attach methods
	self->proc = &aylp_alsa_proc;
	self->fini = &aylp_alsa_fini;

	// default params
	data->device = xstrdup("default");
	data->access = SND_PCM_ACCESS_MMAP_INTERLEAVED;
	data->format = SND_PCM_FORMAT_S16;
	data->channels = 2;
	data->rate = 192000;
	// small buffer/period keeps the actuator command fresh (low latency);
	// enlarge if you get underruns. 0 lets ALSA pick its (large) maximum.
	data->buffer_time = 2000;	// microseconds
	data->period_time = 500;	// microseconds
	data->blocking = false;		// real-time: never stall the loop
	data->threaded = false;		// dedicated feeder thread off by default
	data->rt_prio = 0;		// SCHED_FIFO prio for the feeder (0=none)
	data->poll_us = 50;		// feeder poll interval [us]

	const char *format_name = NULL;
	struct json_object *scale_arr = NULL;

	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
			// keys starting with _ are comments
		} else if (!strcmp(key, "device")) {
			xfree(data->device);
			data->device = xstrdup(json_object_get_string(val));
			log_trace("device = %s", data->device);
		} else if (!strcmp(key, "format")) {
			format_name = json_object_get_string(val);
			log_trace("format = %s", format_name);
		} else if (!strcmp(key, "channels")) {
			data->channels =
				(unsigned)json_object_get_uint64(val);
			log_trace("channels = %u", data->channels);
		} else if (!strcmp(key, "rate")) {
			data->rate = (unsigned)json_object_get_uint64(val);
			log_trace("rate = %u", data->rate);
		} else if (!strcmp(key, "buffer_time")) {
			data->buffer_time =
				(unsigned)json_object_get_uint64(val);
			log_trace("buffer_time = %u", data->buffer_time);
		} else if (!strcmp(key, "period_time")) {
			data->period_time =
				(unsigned)json_object_get_uint64(val);
			log_trace("period_time = %u", data->period_time);
		} else if (!strcmp(key, "scale")) {
			scale_arr = val;
		} else if (!strcmp(key, "blocking")) {
			data->blocking = json_object_get_boolean(val);
			log_trace("blocking = %d", data->blocking);
		} else if (!strcmp(key, "latency_frames")) {
			data->latency_frames =
				(snd_pcm_uframes_t)json_object_get_uint64(val);
			log_trace("latency_frames = %lu", data->latency_frames);
		} else if (!strcmp(key, "threaded")) {
			data->threaded = json_object_get_boolean(val);
			log_trace("threaded = %d", data->threaded);
		} else if (!strcmp(key, "rt_prio")) {
			data->rt_prio = (int)json_object_get_int64(val);
			log_trace("rt_prio = %d", data->rt_prio);
		} else if (!strcmp(key, "poll_us")) {
			data->poll_us = (unsigned)json_object_get_uint64(val);
			log_trace("poll_us = %u", data->poll_us);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	if (format_name) {
		snd_pcm_format_t f = snd_pcm_format_value(format_name);
		if (f == SND_PCM_FORMAT_UNKNOWN) {
			log_error("Unknown sample format \"%s\"", format_name);
			return -1;
		}
		data->format = f;
	}

	if (!data->channels) {
		log_error("channels must be nonzero.");
		return -1;
	}

	// per-channel scale, default 1.0
	data->scale = xcalloc(data->channels, sizeof(double));
	for (unsigned c = 0; c < data->channels; c++) data->scale[c] = 1.0;
	// command scratch + shared (feeder) command buffers
	data->fill_scratch = xcalloc(data->channels, sizeof(double));
	data->shared_cmd = xcalloc(data->channels, sizeof(double));
	pthread_mutex_init(&data->cmd_lock, NULL);

	// the feeder holds the queue at latency_frames, so it must be > 0
	if (data->threaded && data->latency_frames == 0) {
		data->latency_frames = 8;
		log_info("aylp_alsa: threaded mode needs latency_frames>0; "
			"defaulting to 8 (%.3f ms)",
			1e3 * 8.0 / data->rate);
	}
	if (scale_arr) {
		size_t n = json_object_array_length(scale_arr);
		if (n != data->channels) {
			log_error("scale array has %zu entries but there are "
				"%u channels", n, data->channels);
			return -1;
		}
		for (unsigned c = 0; c < data->channels; c++) {
			data->scale[c] = json_object_get_double(
				json_object_array_get_idx(scale_arr, c)
			);
		}
	}

	err = snd_output_stdio_attach(&data->output, stdout, 0);
	if (err < 0) {
		log_error("Output failed: %s", snd_strerror(err));
		return -1;
	}

	log_trace("Stream parameters are %u Hz, %s, %u channels",
		data->rate, snd_pcm_format_name(data->format), data->channels
	);

	err = snd_pcm_open(&data->handle, data->device,
		SND_PCM_STREAM_PLAYBACK, 0
	);
	if (err < 0) {
		log_error("Playback open error (%s): %s",
			data->device, snd_strerror(err));
		return -1;
	}

	err = set_hwparams(data);
	if (err) {
		log_error("Setting of hwparams failed: %s", snd_strerror(err));
		return -1;
	}
	err = set_swparams(data);
	if (err) {
		log_error("Setting of swparams failed: %s", snd_strerror(err));
		return -1;
	}

	if (log_get_level() <= LOG_TRACE)
		snd_pcm_dump(data->handle, data->output);

	data->samples = xmalloc((data->period_size * data->channels
		* snd_pcm_format_physical_width(data->format)) / 8
	);
	data->areas = xcalloc(data->channels, sizeof(snd_pcm_channel_area_t));

	for (unsigned c = 0; c < data->channels; c++) {
		data->areas[c].addr = data->samples;
		data->areas[c].first = c
			* snd_pcm_format_physical_width(data->format);
		data->areas[c].step = data->channels
			* snd_pcm_format_physical_width(data->format);
	}

	data->needs_start = true;
	data->format_bits = snd_pcm_format_width(data->format);
	data->maxval = (1 << (data->format_bits - 1)) - 1;
	data->phys_bps = snd_pcm_format_physical_width(data->format) / 8;
	data->big_endian = snd_pcm_format_big_endian(data->format);
	data->to_unsigned = snd_pcm_format_unsigned(data->format);

	// set types and units
	self->type_in = AYLP_T_VECTOR;
	self->units_in = AYLP_U_MINMAX;
	self->type_out = 0;
	self->units_out = 0;

	// Try to raise the memlock limit so mlockall can succeed, then lock our
	// pages into RAM so a page fault can't stall the feeder and starve the
	// (tiny) audio buffer -- standard low-latency-audio practice.
	struct rlimit rl = { RLIM_INFINITY, RLIM_INFINITY };
	setrlimit(RLIMIT_MEMLOCK, &rl);	// best effort (needs privilege)
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
		log_warn("aylp_alsa: mlockall failed (%s); raise 'ulimit -l' "
			"(or limits.conf memlock) for glitch-free tiny buffers",
			strerror(errno));

	// Start the dedicated feeder thread if requested.
	if (data->threaded) {
		atomic_store_explicit(&data->feeder_run, true,
			memory_order_release);
		err = pthread_create(&data->feeder, NULL,
			feeder_thread_fn, data);
		if (err) {
			log_error("aylp_alsa: pthread_create failed: %s",
				strerror(err));
			return -1;
		}
		data->feeder_started = true;
		log_info("aylp_alsa: feeder thread started (poll %u us, "
			"target %lu fr = %.3f ms)", data->poll_us,
			data->latency_frames,
			1e3 * (double)data->latency_frames / data->rate);
	}

	log_info("aylp_alsa: %s, %u Hz, %s, %u ch, buffer %lu / period %lu fr%s",
		data->device, data->rate,
		snd_pcm_format_name(data->format), data->channels,
		data->buffer_size, data->period_size,
		data->latency_frames ? " [LOW-LATENCY]" : ""
	);
	if (data->latency_frames)
		log_info("aylp_alsa: low-latency target %lu fr = %.3f ms queue "
			"(period %lu fr floors it; underruns if < ~1 loop period"
			" of frames)", data->latency_frames,
			1e3 * (double)data->latency_frames / data->rate,
			data->period_size
		);
	return 0;
}


int aylp_alsa_proc(struct aylp_device *self, struct aylp_state *state)
{
	int err;
	struct aylp_alsa_data *data = self->device_data;
	if (UNLIKELY(state->vector->size != data->channels)) {
		log_error("Pipeline vector is size %zu but we have %u channels",
			state->vector->size, data->channels
		);
		return -1;
	}
	// THREADED mode: just publish the latest command; the feeder thread
	// holds the queue. proc never touches ALSA, so it can't stall the loop
	// and the queue can be smaller than one loop period of frames.
	if (data->threaded) {
		pthread_mutex_lock(&data->cmd_lock);
		scale_cmd(data, state, data->shared_cmd);
		pthread_mutex_unlock(&data->cmd_lock);
		if (UNLIKELY(++data->report_ctr % 20000 == 0)) {
			snd_pcm_sframes_t d = 0;
			if (snd_pcm_delay(data->handle, &d) == 0)
				log_info("aylp_alsa: [threaded] queue %ld fr = "
					"%.3f ms, xruns %lu", (long)d,
					1e3 * (double)d / data->rate,
					data->xrun_count);
		}
		return 0;
	}
	// LOW-LATENCY mode: keep the queue at latency_frames, minimal delay.
	if (data->latency_frames)
		return process_lowlat(data, state);
	// legacy mode: top the buffer up period-by-period.
	for (unsigned p = 0; p < data->buffer_size / data->period_size; p++) {
		log_trace("Processing period %u", p);
		err = process_period(data, state);
		if (err) return err;
	}
	return 0;
}


int aylp_alsa_fini(struct aylp_device *self)
{
	struct aylp_alsa_data *data = self->device_data;
	// stop and join the feeder thread before tearing down the pcm
	if (data->feeder_started) {
		atomic_store_explicit(&data->feeder_run, false,
			memory_order_release);
		pthread_join(data->feeder, NULL);
		log_info("aylp_alsa: feeder stopped (total xruns %lu)",
			data->xrun_count);
	}
	pthread_mutex_destroy(&data->cmd_lock);
	if (data->handle) snd_pcm_close(data->handle);
	xfree(data->shared_cmd);
	xfree(data->fill_scratch);
	xfree(data->scale);
	xfree(data->device);
	xfree(data->areas);
	xfree(data->samples);
	xfree(self->device_data);
	return 0;
}
