#ifndef AYLP_DEVICES_FIT_COM_H_
#define AYLP_DEVICES_FIT_COM_H_

#include <stdbool.h>
#include "anyloop.h"

// Number of model parameters: y0, x0, slope_y, slope_x, sigma, amplitude,
// background. Fixed, so the normal equations live on the stack.
#define AYLP_FIT_COM_NP 7
enum {
	AYLP_FIT_P_Y0 = 0,
	AYLP_FIT_P_X0,
	AYLP_FIT_P_SY,		// px of y motion per image row of readout
	AYLP_FIT_P_SX,		// px of x motion per image row of readout
	AYLP_FIT_P_SIGMA,
	AYLP_FIT_P_AMP,
	AYLP_FIT_P_BG,
};

// Joint spatio-temporal beam fit -- see doc/devices/fit_com.md.
//
// A rolling-shutter frame does not have "a" beam position: every row is
// captured at a different instant, so a frame in which the beam moves is
// sheared, and any single position extracted from it is ill-defined. Rather
// than measure one position and then try to undo the shear afterwards (which
// is what anyloop:wfs_com's row regression attempts, from 2-3 row-group means
// -- far too few points to condition a slope), this device puts the motion
// INSIDE the model and fits it directly:
//
//   I(row,col) = bg + amp * exp(-[(row-y(row))^2 + (col-x(row))^2] / 2 sigma^2)
//     y(row) = y0 + slope_y * (row - ref_row)
//     x(row) = x0 + slope_x * (row - ref_row)
//
// Seven parameters against every pixel in the window -- overdetermined by
// orders of magnitude, where the row-group regression had two parameters from
// three points. Shear stops being a defect to be detected and rejected, and
// becomes the signal that identifies the slope. Nothing is ever discarded for
// moving, so there is no shear gate, and therefore none of the
// velocity-correlated frame dropout that a shear gate necessarily produces (a
// shear threshold is a velocity threshold, so it preferentially drops frames
// at the fast part of every oscillation and keeps the turning points).
struct aylp_fit_com_data {
	// --- model state ---
	double p[AYLP_FIT_COM_NP];	// current parameter vector
	bool active[AYLP_FIT_COM_NP];	// slopes are inactive if !fit_slope
	bool acquired;

	// --- window (0 => whole image) ---
	size_t window_h, window_w;	// as configured
	size_t win_h, win_w;		// actual, clamped to the image
	size_t win_y, win_x;		// window centre, image coords
	long init_y, init_x;		// <0 => acquire from the brightest pixel

	// --- solver tuning ---
	size_t max_iter;
	double tol;			// relative cost improvement to stop at
	double lambda;			// carried between frames as a warm start
	double robust_k;		// Tukey biweight cutoff in residual
					// sigmas; 0 = no reweighting
	size_t robust_iter;		// plain iterations before reweighting starts

	// --- model bounds / validity gates ---
	double sigma_init, sigma_min, sigma_max;
	double min_amplitude;		// fitted amp below this => no beam
	double max_residual;		// rms residual (counts) above this => reject
	size_t reacquire_after;
	size_t lost;

	// --- timing ---
	// Seconds per image row of readout. The POSITION fit does not need it --
	// slope is fitted in px/row and the reported position is the model
	// evaluated at ref_row, both of which are independent of it. It converts
	// the slope to a physical velocity and fixes the epoch the reported
	// position belongs to, which is what a delay-modelling controller
	// downstream needs. 0 => velocity is reported as 0 and nothing else
	// changes.
	double row_time;
	double ref_row;			// fixed epoch, image coords
	double vel_y, vel_x;		// px/s, last good fit (diagnostic)

	// --- last good output, normalized [-1,1] ---
	double last_y, last_x;
	double last_rms;		// residual rms of the last accepted fit
	size_t n_iter_last;

	// --- scratch, allocated at init ---
	double *weights;		// per-pixel robust weights, win_h*win_w
	double *resid;			// per-pixel residual, win_h*win_w
	// spare residual buffer: a trial step writes here and, if accepted,
	// swaps rather than recomputing what it already walked the window for
	double *resid_alt;

	gsl_vector *com;		// output [y, x]
};

int fit_com_init(struct aylp_device *self);
int fit_com_proc(struct aylp_device *self, struct aylp_state *state);
int fit_com_fini(struct aylp_device *self);

#endif
