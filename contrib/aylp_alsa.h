// this plugin writes a command vector to a sound card using alsa (playback).
// Intended as the AC/high-frequency actuator path in a dual-actuator (woofer-
// tweeter) FSM drive: the soundcard's hardware AC-coupling passes only the high
// band, while a DC-coupled DAC (e.g. anyloop:piplate_bridge) carries the low
// band. The two are summed in the analog electronics.
#ifndef AYLP_ALSA_H_
#define AYLP_ALSA_H_

#include <alsa/asoundlib.h>
#include <pthread.h>
#include <stdatomic.h>

#include "anyloop.h"

struct aylp_alsa_data {
	snd_pcm_t *handle;
	snd_output_t *output;
	snd_pcm_hw_params_t *hwparams;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_channel_area_t *areas;
	// playback device from `aplay -L` (e.g. "front", "default", "hw:0,0")
	char *device;
	// write access method (e.g. SND_PCM_ACCESS_MMAP_INTERLEAVED)
	snd_pcm_access_t access;
	// sample format (e.g. SND_PCM_FORMAT_S16)
	snd_pcm_format_t format;
	// number of channels
	unsigned channels;
	// per-channel gain applied to the [-1,1] command before it is written
	// (default 1.0). Lets the AC path match the DC path's per-axis sign and
	// authority; e.g. if piplate_bridge inverts x, set scale [1,-1].
	double *scale;
	// sample rate [Hz]
	unsigned rate;
	// requested time and returned size of buffer
	unsigned buffer_time;
	snd_pcm_uframes_t buffer_size;
	// requested time and returned size of period
	unsigned period_time;
	snd_pcm_uframes_t period_size;
	// if the pcm needs to be started
	bool needs_start;
	// if true, block (snd_pcm_wait) when the buffer is full until a period
	// drains -- this PACES the whole anyloop loop at the sound-card period
	// rate. For a real-time actuator leave false (default): a full buffer
	// just skips this iteration's write and returns immediately, so the
	// camera keeps pacing the loop and the card plays the freshest samples
	// it already has.
	bool blocking;
	// LOW-LATENCY target-fill mode: if >0, each proc keeps only this many
	// frames queued ahead of the DAC (measured via snd_pcm_delay) instead of
	// filling the buffer, minimizing command->output latency. Floor is set by
	// the loop refill interval: it must exceed one loop period of frames or it
	// underruns (at a 3788 Hz loop / 48 kHz that is ~13 frames). 0 = legacy
	// fill-the-buffer behaviour.
	snd_pcm_uframes_t latency_frames;
	// throttle counter for the periodic snd_pcm_delay latency report
	unsigned long report_ctr;

	// DEDICATED FEEDER THREAD: decouples the card refill from the anyloop
	// (camera-paced) loop, so a queue smaller than one loop period of frames
	// can be sustained without underrunning -- this is what lets the reported
	// latency go below ~13 frames. proc() just publishes the latest command;
	// the feeder holds the queue at latency_frames in a tight loop.
	bool threaded;			// param: enable the feeder thread
	int rt_prio;			// param: SCHED_FIFO prio (0 = none)
	unsigned poll_us;		// param: feeder poll interval [us]
	pthread_t feeder;
	bool feeder_started;
	atomic_bool feeder_run;
	pthread_mutex_t cmd_lock;
	double *shared_cmd;		// channels, scaled+clamped latest command
	double *fill_scratch;		// channels, feeder/legacy scratch
	unsigned long xrun_count;
	// scratch sample buffer (unused by the mmap path but kept for parity)
	signed short *samples;
	// how many bits in our format
	int format_bits;
	// maximum unsigned value in our format
	unsigned maxval;
	// physical bits per sample (usually same as format_bits)
	int phys_bps;
	// is the requested format big endian?
	bool big_endian;
	// is the requested format unsigned?
	bool to_unsigned;
};

// initialize alsa device
int aylp_alsa_init(struct aylp_device *self);

// write vector to alsa
int aylp_alsa_proc(struct aylp_device *self, struct aylp_state *state);

// close alsa device when loop exits
int aylp_alsa_fini(struct aylp_device *self);

#endif
