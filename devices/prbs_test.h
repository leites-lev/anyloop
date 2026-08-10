#ifndef AYLP_DEVICES_PRBS_TEST_H_
#define AYLP_DEVICES_PRBS_TEST_H_

#include <stdbool.h>
#include <stdio.h>

#include "anyloop.h"

// test stages; see prbs_test.c
#define PT_WARMUP	0	// park at bias; sensor side acquires
#define PT_BURST	1	// driving the PRBS
#define PT_QUIET	2	// parked at bias between bursts
#define PT_DONE		3

struct aylp_prbs_test_data {
	// params
	int index_cmd;		// output element to drive
	int index_err;		// input element to watch
	size_t out_size;	// output vector length
	double amplitude;	// PRBS half-swing, command units
	double bias;		// command between bursts and after the test
	unsigned order;		// LFSR order; the run is 2^order - 1 chips
	size_t chip_frames;	// loop iterations per chip
	size_t n_bursts;	// bursts to average
	size_t quiet_frames;	// frames parked at bias after each burst
	double warmup;		// seconds parked at bias before the first burst
	size_t max_lag;		// positive lags evaluated
	size_t neg_lags;	// acausal lags, which measure the noise floor
	double onset_frac;	// fraction of the peak that counts as onset
	double phase_f_lo;	// band for the cross-spectrum phase-slope fit
	double phase_f_hi;
	double volts_per_unit;	// DAC scale of the swept channel, for reporting
	double pixel_scale;	// px per response unit, for reporting
	char *output_file;	// PDF path; the .dat is written alongside it
	char *config;		// free-text note copied into the .dat header

	// output vector; element index_cmd carries the PRBS, rest are 0
	gsl_vector *out;

	// stimulus
	size_t n_chips;		// 2^order - 1
	signed char *chips;	// +-1, one per chip
	size_t burst_frames;	// n_chips * chip_frames
	size_t win;		// burst_frames + quiet_frames
	double *cmd_win;	// the command of one whole window, mean removed

	// state machine
	int stage;
	double t0;		// time of the first proc (s, CLOCK_MONOTONIC)
	size_t burst;		// bursts started so far
	size_t frame;		// frames into the current window
	double cmd;		// command currently published

	// per-burst response, [burst*win + frame]
	double *resp;

	// loop-rate bookkeeping
	double t_prev, dt_sum, dt_sum2;
	size_t dt_cnt;

	// results
	double *rho;		// correlogram over [-neg_lags, max_lag]
	double *rbar;		// ensemble-mean response over the window
	double *rsem;		// standard error of that mean
	double fs;		// measured loop rate, Hz
	double rho_peak;	// signed peak of the correlogram
	double lag_peak;	// parabola-refined peak lag, frames
	double lag_onset;	// interpolated onset lag, frames
	double lag_centroid;	// first moment of the correlation lobe
	double rho_noise;	// rms correlation over the acausal lags
	double tau_phase_ms;	// delay from the cross-spectrum phase slope
	double phase_resid_deg;	// rms residual of that fit
	size_t phase_n;		// points it used
	double lag_peak_med;	// median over the per-burst estimates
	double lag_peak_mad;	// and their median absolute deviation
	double lag_onset_med;
	double lag_onset_mad;
	size_t n_burst_ok;
	bool have_result;
};

int prbs_test_init(struct aylp_device *self);
int prbs_test_proc(struct aylp_device *self, struct aylp_state *state);
int prbs_test_fini(struct aylp_device *self);

#endif
