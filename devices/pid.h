#ifndef AYLP_DEVICES_PID_H_
#define AYLP_DEVICES_PID_H_

#include <time.h>
#include "anyloop.h"

struct aylp_pid_data {
	// param type in ["vector", "matrix"]
	aylp_type type;
	// units to output
	aylp_units units;
	// accumulated error
	union {
		gsl_vector *acc_v;
		gsl_matrix *acc_m;
	};
	// previous error
	union {
		gsl_vector *pre_v;
		gsl_matrix *pre_m;
	};
	// correction result
	union {
		gsl_vector *res_v;
		gsl_matrix *res_m;
	};
	// previous (filtered) derivative, for the derivative-filter recursion
	union {
		gsl_vector *dfd_v;
		gsl_matrix *dfd_m;
	};
	// previous timestamp
	struct timespec tp;
	// pid params, clamp for maximum correction (by i term and in total)
	double p, i, d, clamp, g;
	double p_y, i_y, d_y, g_y;
	// derivative filter: low-pass the D term at this cutoff (Hz) to limit
	// noise amplification. <=0 disables it (pure derivative). Per-axis, with
	// the same convention as d: dfilt_y < 0 inherits the base dfilt.
	double dfilt, dfilt_y;
	// deadband: errors with |error| < deadband are treated as zero, to
	// suppress quantization/relay limit cycles around the null. Per-axis.
	double deadband, deadband_y;
	// optional series lead compensator on the PID output, per axis:
	//   C(s) = lead_g * (s + 2pi*lead_fz) / (s + 2pi*lead_fp)
	// DC gain = lead_g*fz/fp, HF gain = lead_g. The pole (lead_fp > 0) enables
	// the section; the zero may sit at 0 Hz (lead_fz = 0) for a band-limited
	// differentiator s/(s+wp). fp > fz gives phase lead. Base fields are the x
	// axis; _y fields are the y axis (independent, no inheritance, so x-only /
	// y-only / both-different are all expressible).
	double lead_fz, lead_fp, lead_g;
	double lead_fz_y, lead_fp_y, lead_g_y;
	// lead compensator state: previous raw input and filtered output (vector
	// type only). Bilinear coeffs are recomputed each step from the live dt.
	gsl_vector *lead_in_v;
	gsl_vector *lead_out_v;
};

// initialize pid device
int pid_init(struct aylp_device *self);

// process pid device once per loop
int pid_proc(struct aylp_device *self, struct aylp_state *state);

// close pid device when loop exits
int pid_fini(struct aylp_device *self);

#endif


