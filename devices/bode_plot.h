#ifndef AYLP_DEVICES_BODE_PLOT_H_
#define AYLP_DEVICES_BODE_PLOT_H_

#include <stdbool.h>
#include "anyloop.h"

// output modes for bode_plot
enum {
	BODE_OUTPUT_RAW  = 0,  // use element value directly as y
	BODE_OUTPUT_DIFF = 1,  // use element - element_previous as y (e.g. com-com_prev)
};

// inject modes for bode_plot
enum {
	BODE_INJECT_ADD     = 0,  // add u to existing element (closed-loop perturbation)
	BODE_INJECT_REPLACE = 1,  // replace element with u (open-loop FSM command)
};

struct aylp_bode_plot_data {
	// required
	double sample_rate;     // Hz
	// frequency sweep range
	double freq_start;      // Hz (default: 1.0)
	double freq_end;        // Hz (default: Nyquist)
	size_t n_freqs;         // number of log-spaced frequencies (default: 30)
	// measurement quality
	size_t n_settle;        // samples to skip at each frequency for settling (default: 50)
	double n_cycles;        // sinusoidal cycles to average per frequency (default: 10)
	// injection
	double amplitude;       // injection amplitude in pipeline units (default: 0.1)
	unsigned inject_mode;   // BODE_INJECT_ADD or BODE_INJECT_REPLACE
	// output channel
	unsigned output_mode;   // BODE_OUTPUT_RAW or BODE_OUTPUT_DIFF
	size_t element;         // linear index into vector/matrix (default: 0)
	// frequency spacing
	bool freq_linear;       // if true, linear spacing; default false (log)
	// output
	char *output_file;      // PDF filename (default: "bode_plot.pdf")

	// sweep state
	size_t freq_idx;        // current frequency index
	size_t step_count;      // samples elapsed at current frequency
	size_t n_avg_cur;       // averaging samples for current frequency
	double start_time;      // real-world start timestamp of current frequency
	double prev_time;       // real-world timestamp of previous iteration
	double total_time;      // accumulated duration of frequency sampling
	double phase;           // current excitation phase accumulator
	double prev_value;      // previous raw value for diff mode
	double Y_cos;           // correlator: integral of y * cos(phi) dt
	double Y_sin;           // correlator: integral of y * sin(phi) dt
	double prev_cos_term;   // trapezoidal integration baseline (cos)
	double prev_sin_term;   // trapezoidal integration baseline (sin)
	double prev_phi;        // injection phase from previous step (y[n] responds to phi[n-1])

	// hardware FG mode (optional — bypasses software sine injection)
	// Set fg_port to use the DAQC2 hardware function generator instead of computing
	// sine in software.  The FG runs at 200 kHz so the output waveform is clean
	// regardless of loop-rate jitter.  The piplate_bridge for the swept axis must
	// NOT be in the pipeline when fg_port is set (they would fight over the port).
	char  *fg_port;    // serial device, e.g. "/dev/ttyACM0"; NULL = software mode
	int    fg_fd;      // open serial fd, -1 when unused
	int    fg_board;   // DAQC2 board 0-7 (default 0)
	int    fg_channel; // FG output channel 1 or 2 (default 1)
	int    fg_level;   // amplitude: 1=eighth 2=quarter 3=half 4=full (default 1)
	double fg_offset;  // DC center voltage in V, 0.0-4.095 (default 0 = hardware default 2.048V)

	// results (one entry per frequency)
	double *freqs;
	double *mags;
	double *phases;
};

int bode_plot_init(struct aylp_device *self);
int bode_plot_proc(struct aylp_device *self, struct aylp_state *state);
int bode_plot_fini(struct aylp_device *self);

#endif
