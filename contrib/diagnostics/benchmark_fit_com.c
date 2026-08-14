// Latency benchmark for anyloop:fit_com.
//
// The device sits in a hard-real-time AO loop, so what matters is not the mean
// cost but the tail: a frame that overruns is a frame the controller runs open
// on. This measures the per-frame wall time over a moving, sheared, noisy scene
// at the largest sensor size the loop is specified for, and reports the
// distribution plus the worst frame seen.
//
// Frames are rendered up front into a ring big enough to fall out of last-level
// cache, and the loop then walks it. That is what the real device sees -- a
// frame just DMAed in by the camera, cold, while its own scratch buffers stay
// warm from the previous iteration -- and it is what rendering each frame
// immediately before timing it does NOT: that leaves the frame hot and evicts
// everything else, which is exactly backwards.
//
// Usage: fit_com_bench [size] [window] [max_iter] [frames] [fit_radius]
//                      [fit_gaussian] [max_us] [sigma] [moment_col_stride]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>

#include "anyloop.h"
#include "fit_com.h"

// enough 248x248 frames to exceed a 16 MB last-level cache
#define RING 320

static double now_us(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec*1e6 + t.tv_nsec*1e-3;
}

static int cmpd(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

// xorshift, so the noise is identical run to run and two builds can be
// compared on the same frames.
static unsigned long rng_state = 88172645463325252ul;
static double urand(void)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 7;
	rng_state ^= rng_state << 17;
	return (double)(rng_state >> 11) / 9007199254740992.0;
}

static void render(unsigned char *data, size_t n, double y0, double x0,
	double sy, double sx, double sigma, double amp, double bg, double noise
) {
	double ref = 0.5*((double)n - 1.0);
	for (size_t r = 0; r < n; r++) {
		double t = (double)r - ref;
		double yc = y0 + sy*t, xc = x0 + sx*t;
		for (size_t c = 0; c < n; c++) {
			double dy = (double)r - yc, dx = (double)c - xc;
			double v = bg + amp*exp(-(dy*dy+dx*dx)/(2*sigma*sigma))
				+ noise*(urand() - 0.5);
			if (v < 0) v = 0;
			data[r*n+c] = (unsigned char)(v > 255 ? 255 : v);
		}
	}
}

int main(int argc, char **argv)
{
	size_t n = argc > 1 ? (size_t)atol(argv[1]) : 248;
	size_t win = argc > 2 ? (size_t)atol(argv[2]) : 0;
	size_t max_iter = argc > 3 ? (size_t)atol(argv[3]) : 10;
	size_t frames = argc > 4 ? (size_t)atol(argv[4]) : 4000;
	double frad = argc > 5 ? atof(argv[5]) : 3.5;
	bool fit_gaussian = argc > 6 ? !!atoi(argv[6]) : true;
	double max_us = argc > 7 ? atof(argv[7]) : 10.0;
	double sigma = argc > 8 ? atof(argv[8]) : 2.5;
	size_t moment_col_stride = argc > 9 ? (size_t)atol(argv[9]) : 1;

	char buf[512];
	snprintf(buf, sizeof buf, "{\"sigma_init\":2.5,\"sigma_min\":0.5,"
		"\"sigma_max\":50.0,\"max_iter\":%zu,\"min_amplitude\":5.0,"
		"\"max_us\":%g,\"reacquire_after\":10,\"row_time\":8.25e-6,"
		"\"window_height\":%zu,\"window_width\":%zu,\"fit_radius\":%g,"
		"\"moment_output\":true,\"moment_col_stride\":%zu,"
		"\"fit_gaussian\":%s}",
		max_iter, max_us, win, win, frad, moment_col_stride,
		fit_gaussian ? "true" : "false");
	json_object *p = json_tokener_parse(buf);
	struct aylp_device self = {0};
	self.params = p;
	if (fit_com_init(&self)) { fprintf(stderr, "init failed\n"); return 1; }
	struct aylp_fit_com_data *d = self.device_data;
	struct aylp_state st = {0};

	// 3 px p-p tip/tilt at fs 3788, four periods to the ring so it wraps
	// without a jump in the disturbance the solver is warm-started against.
	double fs = 3788.0, ampl = 3.0;
	double fdist = 4.0*fs/(double)RING;
	double cy = 0.5*(double)n, cx = 0.5*(double)n;

	unsigned char *ring = malloc(RING*n*n);
	double *ty = malloc(RING*sizeof(double));
	double *tx = malloc(RING*sizeof(double));
	for (size_t k = 0; k < RING; k++) {
		double t = (double)k/fs;
		ty[k] = cy + ampl*sin(2*M_PI*fdist*t);
		tx[k] = cx + ampl*cos(2*M_PI*fdist*t);
		// shear = velocity * row_time, in px/row
		double sy = ampl*2*M_PI*fdist*cos(2*M_PI*fdist*t)*8.25e-6;
		double sx = -ampl*2*M_PI*fdist*sin(2*M_PI*fdist*t)*8.25e-6;
		render(ring + k*n*n, n, ty[k], tx[k], sy, sx, sigma, 200.0, 8.0, 6.0);
	}

	gsl_matrix_uchar img = {.size1 = n, .size2 = n, .tda = n,
		.data = ring, .block = 0, .owner = 0};
	double *us = malloc(frames*sizeof(double));
	double worst_err = 0.0, sum_err = 0.0;
	size_t nerr = 0, rejected = 0, iters = 0, nbudget = 0;
	size_t ihist[8] = {0};

	for (size_t f = 0; f < frames; f++) {
		size_t k = f % RING;
		img.data = ring + k*n*n;
		st.matrix_uchar = &img;
		st.header.status = 0;
		double t0 = now_us();
		fit_com_proc(&self, &st);
		us[f] = now_us() - t0;

		if (d->budget_hit) nbudget++;
		ihist[d->n_iter_last < 7 ? d->n_iter_last : 7]++;
		if (st.header.status & AYLP_FRAME_REJECTED) { rejected++; continue; }
		if (f < 8) continue;	// let acquisition settle
		double ry = (d->last_y+1.0)/2.0*((double)n-1);
		double rx = (d->last_x+1.0)/2.0*((double)n-1);
		double e = hypot(ry-ty[k], rx-tx[k]);
		if (e > worst_err) worst_err = e;
		sum_err += e*e; nerr++;
		iters += d->n_iter_last;
	}

	double first = us[0];
	qsort(us, frames, sizeof(double), cmpd);
	printf("%zux%zu  window %zu  max_iter %zu  %zu frames\n",
		n, n, win ? win : n, max_iter, frames);
	printf("  latency   p50 %7.2f us   p90 %7.2f   p99 %7.2f   "
		"max %7.2f   (acq frame %7.2f)\n",
		us[frames/2], us[(size_t)(frames*0.90)],
		us[(size_t)(frames*0.99)], us[frames-1], first);
	printf("  accuracy  rms %.4f px   worst %.4f px   rejected %zu   "
		"mean iter %.2f\n",
		nerr ? sqrt(sum_err/(double)nerr) : 0.0, worst_err, rejected,
		nerr ? (double)iters/(double)nerr : 0.0);
	printf("  spread    p10 %7.2f  p25 %7.2f  p75 %7.2f  p95 %7.2f\n",
		us[(size_t)(frames*0.10)], us[(size_t)(frames*0.25)],
		us[(size_t)(frames*0.75)], us[(size_t)(frames*0.95)]);
	printf("  iters     ");
	for (size_t k = 1; k <= 6; k++) printf("%zu:%zu  ", k, ihist[k]);
	printf(">6:%zu   budget_hit %zu\n", ihist[7], nbudget);
	printf("  %s 10 us budget at p99 (%.2f us)\n",
		us[(size_t)(frames*0.99)] <= 10.0 ? "MEETS" : "MISSES",
		us[(size_t)(frames*0.99)]);

	free(us); free(ring); free(ty); free(tx);
	fit_com_fini(&self);
	json_object_put(p);
	return 0;
}
