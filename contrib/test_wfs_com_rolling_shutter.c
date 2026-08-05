// Standalone diagnostic/calibration tool for wfs_com's row_time
// rolling-shutter correction (see the "Rolling-shutter correction" section
// of doc/devices/wfs_com.md). NOT installed, NOT a pass/fail test -- built
// alongside the rest of the tree so `ninja -C build` covers it, run manually.
//
// row_time cannot be self-calibrated from a fit residual: rescaling
// row_time merely rescales the fitted line's slope without producing a
// detectable fit error (a linear fit against a linearly-rescaled independent
// variable is still a perfect linear fit). It has to be calibrated against a
// KNOWN ground truth instead -- either an external stage moving a target at
// a measured rate, or, as here, a simulated ground truth. This tool sweeps a
// grid of row_time guesses against synthetic rolling-shutter frames with a
// known true trajectory and reports tracking RMS error per guess, so the
// minimum can be read off directly -- exactly the procedure to run against
// real bench data (with a known commanded trajectory in place of the
// simulated one) to calibrate a real camera's row_time.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <gsl/gsl_matrix.h>

#include "anyloop.h"
#include "wfs_com.h"
#include "xalloc.h"

static void wfs_setup(struct aylp_wfs_com_data *data, size_t init_y,
	size_t init_x, double row_time
) {
	memset(data, 0, sizeof(*data));
	data->subap_h = 8; data->subap_w = 8;
	data->subap_rows = 4; data->subap_cols = 4;
	data->threshold = 5;
	data->search_radius = 3;
	data->ref_beta = 0.05;
	data->min_confidence = 0.4;
	data->flux_floor = 15.0;
	data->min_valid_subaps = 4;
	data->row_time = row_time;
	data->rolling_shutter = row_time > 0.0;
	data->init_y = (long)init_y;
	data->init_x = (long)init_x;
	data->reacquire_after = 10;
	data->n_subaps = data->subap_rows * data->subap_cols;
	data->ext_h = data->subap_h + 2*data->search_radius;
	data->ext_w = data->subap_w + 2*data->search_radius;
	data->win_h = data->subap_rows*data->subap_h + 2*data->search_radius;
	data->win_w = data->subap_cols*data->subap_w + 2*data->search_radius;
	data->ref = xcalloc(data->n_subaps * data->subap_h * data->subap_w, sizeof(double));
	data->ref_backup = xcalloc(data->n_subaps * data->subap_h * data->subap_w, sizeof(double));
	data->ref_set = xcalloc(data->n_subaps, sizeof(bool));
	data->ref_set_backup = xcalloc(data->n_subaps, sizeof(bool));
	data->matched = xcalloc(data->n_subaps, sizeof(bool));
	data->sub_dy = xcalloc(data->n_subaps, sizeof(double));
	data->sub_dx = xcalloc(data->n_subaps, sizeof(double));
	data->ext = xcalloc(data->ext_h * data->ext_w, sizeof(double));
	data->row_sum_w = xcalloc(data->subap_rows, sizeof(double));
	data->row_sum_wt = xcalloc(data->subap_rows, sizeof(double));
	data->row_sum_wdy = xcalloc(data->subap_rows, sizeof(double));
	data->row_sum_wdx = xcalloc(data->subap_rows, sizeof(double));
	data->com = xmalloc_type(gsl_vector, 2);
}

static void wfs_teardown(struct aylp_wfs_com_data *data)
{
	xfree(data->ref); xfree(data->ref_backup); xfree(data->ref_set); xfree(data->ref_set_backup);
	xfree(data->matched); xfree(data->sub_dy); xfree(data->sub_dx); xfree(data->ext);
	xfree(data->row_sum_w); xfree(data->row_sum_wt); xfree(data->row_sum_wdy); xfree(data->row_sum_wdx);
	xfree_type(gsl_vector, data->com);
}

// Run n_frames of synthetic rolling-shutter data (true motion known exactly)
// through wfs_com configured with the given row_time_guess, and return the
// RMS tracking error against ground truth.
static double eval_row_time(size_t img_sz, double frame_period,
	double row_time_true, double row_time_guess,
	double ay, double fy, double ax, double fx, double phx, int n_frames
) {
	gsl_matrix_uchar *img = gsl_matrix_uchar_alloc(img_sz, img_sz);
	struct aylp_wfs_com_data data;
	wfs_setup(&data, img_sz/2, img_sz/2, row_time_guess);
	struct aylp_device self = {0};
	self.device_data = &data;
	self.type_out = AYLP_T_VECTOR;
	self.units_out = AYLP_U_MINMAX;
	struct aylp_state st = {0};

	double y0 = img_sz/2.0, x0 = img_sz/2.0, t0 = 0.0;
	double se = 0.0;
	int n_scored = 0;
	for (int f = 0; f < n_frames; f++) {
		gsl_matrix_uchar_set_zero(img);
		for (size_t row = 0; row < img_sz; row++) {
			double t = t0 + row_time_true*(double)row;
			double cy = y0 + ay*sin(2*M_PI*fy*t);
			double cx = x0 + ax*sin(2*M_PI*fx*t + phx);
			for (size_t col = 0; col < img_sz; col++) {
				double dy = (double)row-cy, dx = (double)col-cx;
				double v = 200.0*exp(-(dy*dy+dx*dx)/(2*2.5*2.5));
				img->data[row*img->tda+col] = (unsigned char)(v > 255 ? 255 : v);
			}
		}
		bool score = f >= 8;
		size_t org_y = score ? data.win_y - data.win_h/2 : 0;
		double t0_frame = t0;

		st.matrix_uchar = img;
		wfs_com_proc(&self, &st);
		t0 += frame_period;

		if (score) {
			// Ground truth uses row_time_TRUE (the real physical
			// skew) even though the device was configured with
			// row_time_guess -- we're scoring how good a GUESS was,
			// not redefining the truth to match it.
			double tr = t0_frame + row_time_true
				*((double)org_y + data.subap_rows*data.subap_h/2.0);
			double ty = y0+ay*sin(2*M_PI*fy*tr);
			double tx = x0+ax*sin(2*M_PI*fx*tr+phx);
			double ry = (data.last_y+1.0)/2.0*(img_sz-1);
			double rx = (data.last_x+1.0)/2.0*(img_sz-1);
			double e = hypot(ry-ty, rx-tx);
			se += e*e;
			n_scored++;
		}
	}
	double rms = sqrt(se/n_scored);
	wfs_teardown(&data);
	gsl_matrix_uchar_free(img);
	return rms;
}

int main(void)
{
	const size_t IMG = 96;
	const double FRAME_PERIOD = 1.0/3788.0;
	const double ROW_TIME_TRUE = FRAME_PERIOD / (double)IMG;
	const double AY = 10.0, FY = 250.0, AX = 4.0, FX = 180.0, PHX = 1.0;
	const int N_FRAMES = 80;

	printf("True row_time: %.4e s/row\n\n", ROW_TIME_TRUE);
	printf("This sweeps a grid of row_time GUESSES against synthetic\n"
		"rolling-shutter data with a KNOWN true row_time, and reports\n"
		"tracking RMS error per guess -- the same procedure to run\n"
		"against real bench data (known commanded trajectory in place\n"
		"of the simulated one) to calibrate a real camera's row_time.\n\n");
	printf("%12s  %10s\n", "row_time", "RMS (px)");

	double best_rms = 1e300, best_guess = 0.0;
	// 0 (uncorrected) plus a grid from 0.25x to 2x the true value
	double factors[] = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0};
	for (size_t i = 0; i < sizeof(factors)/sizeof(factors[0]); i++) {
		double guess = factors[i] * ROW_TIME_TRUE;
		double rms = eval_row_time(IMG, FRAME_PERIOD, ROW_TIME_TRUE, guess,
			AY, FY, AX, FX, PHX, N_FRAMES);
		printf("%12.4e  %10.4f%s\n", guess, rms,
			factors[i] == 0.0 ? "   <- uncorrected" : "");
		if (rms < best_rms) { best_rms = rms; best_guess = guess; }
	}

	printf("\nbest guess in this sweep: %.4e s/row (%.2fx true), RMS %.4f px\n",
		best_guess, best_guess/ROW_TIME_TRUE, best_rms);

	// If every positive guess landed on (near enough) the same RMS, the
	// magnitude of row_time doesn't matter to the current implementation
	// -- confirm that here instead of assuming a finer grid would find a
	// sharper optimum. See the "row_time" comment in wfs_com.h for why:
	// the regression's fitted variable and its evaluation point both
	// scale by row_time, so it cancels out of the extrapolation exactly.
	double spread = 0.0;
	for (size_t i = 1; i < sizeof(factors)/sizeof(factors[0]); i++) {
		if (factors[i] <= 0.0) continue;
		double rms = eval_row_time(IMG, FRAME_PERIOD, ROW_TIME_TRUE,
			factors[i]*ROW_TIME_TRUE, AY, FY, AX, FX, PHX, N_FRAMES);
		double d = fabs(rms - best_rms);
		if (d > spread) spread = d;
	}
	if (spread < 1e-3) {
		printf("\nEvery positive guess gave the same RMS (spread %.2e px):\n"
			"row_time's MAGNITUDE doesn't matter here, only its sign --\n"
			"any positive value enables the correction identically. There\n"
			"is nothing to calibrate beyond turning it on. See\n"
			"doc/devices/wfs_com.md's \"Rolling-shutter correction\" section.\n",
			spread);
	} else {
		printf("\nGuesses differ meaningfully (spread %.4f px) -- refine the\n"
			"grid around %.4e and confirm against a second true velocity\n"
			"before trusting this as the calibrated value.\n", spread, best_guess);
	}
	return 0;
}
