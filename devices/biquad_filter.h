#ifndef AYLP_DEVICES_BIQUAD_FILTER_H_
#define AYLP_DEVICES_BIQUAD_FILTER_H_

#include "anyloop.h"

// which second-order section to realize
enum aylp_biquad_filter_mode {
	AYLP_BIQUAD_FILTER_LOWPASS,
	AYLP_BIQUAD_FILTER_NOTCH,
};

struct aylp_biquad_filter_data {
	// param type in ["vector", "matrix"]
	aylp_type type;
	// units to output
	aylp_units units;
	// filter mode (lowpass or notch)
	enum aylp_biquad_filter_mode mode;
	// design parameters
	double f0;	// center (notch) / corner (lowpass) frequency, Hz
	double q;	// quality factor (sets width/damping)
	double fs;	// assumed sample rate, Hz
	// normalized Direct Form I coefficients (a0 has been divided out)
	double b0, b1, b2, a1, a2;
	// per-element filter state: two past inputs and two past outputs
	size_t n;
	double *x1, *x2, *y1, *y2;
	// place to put the result
	union {
		gsl_vector *res_v;
		gsl_matrix *res_m;
	};
};

// initialize biquad_filter device
int biquad_filter_init(struct aylp_device *self);

// process biquad_filter device once per loop
int biquad_filter_proc(struct aylp_device *self, struct aylp_state *state);

// close biquad_filter device when loop exits
int biquad_filter_fini(struct aylp_device *self);

#endif

