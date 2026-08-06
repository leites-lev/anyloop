// com_survey -- decide between anyloop:fit_com and anyloop:wfs_com from a
// real open-loop capture, and report the beam/camera numbers the rest of the
// tuning depends on.
//
// Capture with contrib/com_survey.json (one minute, no tracker, no loop, no
// DAC), then run this over the resulting .aylp file. Both trackers are driven
// over the SAME frames, so every comparison here is like-for-like.
//
// Why this needs measuring rather than reasoning: the two devices fail in
// opposite directions and which one wins is a property of your beam.
//   fit_com  fits a gaussian plus its intra-frame motion. It is far more
//            accurate when that model describes the beam, reports absolute
//            position, and treats rolling shutter as a fitted parameter --
//            but a beam that is not gaussian biases it, and if the departure
//            from gaussian MOVES (boiling speckle) that bias moves too and
//            appears at the controller as real motion.
//   wfs_com  correlates against a learned template. It does not care what
//            shape the beam is, only that the shape is STABLE, so boiling
//            speckle only adds noise instead of fabricating motion. It is
//            less accurate on a clean beam and carries an acquisition-time
//            offset.
// So the decision turns on two measurables: how far the beam is from
// gaussian, and whether that departure is frozen or boiling.
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>

#include "anyloop.h"
#include "fit_com.h"
#include "wfs_com.h"

#define HDR 40
#define MAXLAG 1024

static double now_s(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + 1e-9*t.tv_nsec;
}
static int cmpd(const void *a, const void *b)
{ double x = *(const double*)a, y = *(const double*)b; return x<y?-1:(x>y); }

struct series { double *v; size_t n, cap; };
static void ser_add(struct series *s, double x)
{
	if (s->n == s->cap) {
		s->cap = s->cap ? s->cap*2 : 4096;
		s->v = realloc(s->v, s->cap*sizeof(double));
	}
	s->v[s->n++] = x;
}
static double ser_mean(const struct series *s)
{
	double m = 0; for (size_t i = 0; i < s->n; i++) m += s->v[i];
	return s->n ? m/s->n : 0.0;
}
static double ser_rms_about_mean(const struct series *s)
{
	double m = ser_mean(s), q = 0;
	for (size_t i = 0; i < s->n; i++) q += (s->v[i]-m)*(s->v[i]-m);
	return s->n ? sqrt(q/s->n) : 0.0;
}
// rms of the frame-to-frame difference / sqrt(2): the high-frequency content,
// which is what a steering loop is actually being asked to reject, and where
// fabricated motion shows up most clearly.
static double ser_hf(const struct series *s)
{
	if (s->n < 2) return 0.0;
	double q = 0;
	for (size_t i = 1; i < s->n; i++) {
		double d = s->v[i]-s->v[i-1];
		q += d*d;
	}
	return sqrt(q/(2.0*(s->n-1)));
}

int main(int argc, char **argv)
{
	const char *path = 0;
	double fs = 3788.0, row_time = 0.0;
	size_t max_frames = 0;
	// wfs_com geometry/gate overrides. The defaults below are derived from
	// the measured spot further down; these exist because those heuristics
	// are only a starting point and the rejection rate is the thing that
	// tells you they were wrong. A grid whose outer cells hold almost no
	// light cannot meet min_valid_subaps no matter how good the tracker is,
	// and the resulting 90%+ rejection reads as "wfs_com loses" when it is
	// really "wfs_com was asked for consensus among cells that are dark".
	// Any value left at 0 keeps the derived default.
	size_t o_cell = 0, o_grid = 0, o_minvalid = 0, o_fitwin = 0;
	long o_sr = -1, o_edge = -1;
	double o_minconf = -1.0, o_fluxfloor = -1.0;
	static const char *usage =
		"usage: %s CAPTURE.aylp [--fs HZ] [--row-time S] "
		"[--max-frames N]\n"
		"  wfs_com overrides (0/unset = derive from the beam):\n"
		"    --wfs-cell PX        subaperture size\n"
		"    --wfs-grid N         N by N subapertures\n"
		"    --wfs-min-valid N    subapertures required per frame\n"
		"    --wfs-min-conf F     NCC confidence threshold\n"
		"    --wfs-search-radius PX  correlation search half-width\n"
		"    --wfs-edge 0|1       reject boundary matches\n"
		"    --wfs-flux-floor F   per-cell flux gate\n"
		"    --fit-window PX      fit_com window size\n";
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--fs") && i+1 < argc) fs = atof(argv[++i]);
		else if (!strcmp(argv[i], "--row-time") && i+1 < argc)
			row_time = atof(argv[++i]);
		else if (!strcmp(argv[i], "--max-frames") && i+1 < argc)
			max_frames = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-cell") && i+1 < argc)
			o_cell = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-grid") && i+1 < argc)
			o_grid = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-min-valid") && i+1 < argc)
			o_minvalid = strtoull(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-min-conf") && i+1 < argc)
			o_minconf = atof(argv[++i]);
		else if (!strcmp(argv[i], "--wfs-search-radius") && i+1 < argc)
			o_sr = strtol(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-edge") && i+1 < argc)
			o_edge = strtol(argv[++i], 0, 0);
		else if (!strcmp(argv[i], "--wfs-flux-floor") && i+1 < argc)
			o_fluxfloor = atof(argv[++i]);
		else if (!strcmp(argv[i], "--fit-window") && i+1 < argc)
			o_fitwin = strtoull(argv[++i], 0, 0);
		else if (argv[i][0] != '-') path = argv[i];
		else { fprintf(stderr, usage, argv[0]); return 2; }
	}
	if (!path) { fprintf(stderr, usage, argv[0]); return 2; }
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); return 1; }

	unsigned char hdr[HDR];
	if (fread(hdr, 1, HDR, f) != HDR) {
		fprintf(stderr, "%s: too short to hold one frame header\n", path);
		return 1;
	}
	uint64_t H, W;
	memcpy(&H, hdr+8, 8);
	memcpy(&W, hdr+16, 8);
	uint8_t type = hdr[6];
	if (type != AYLP_T_MATRIX_UCHAR) {
		fprintf(stderr, "%s: first frame is type %u, expected "
			"MATRIX_UCHAR (%u). Capture with com_survey.json.\n",
			path, type, (unsigned)AYLP_T_MATRIX_UCHAR);
		return 1;
	}
	if (H < 8 || W < 8 || H > 4096 || W > 4096) {
		fprintf(stderr, "%s: implausible frame size %lux%lu\n", path,
			(unsigned long)H, (unsigned long)W);
		return 1;
	}
	fseek(f, 0, SEEK_END);
	long long bytes = ftell(f);
	fseek(f, 0, SEEK_SET);
	size_t fsz = HDR + H*W;
	size_t nframes = (size_t)(bytes / (long long)fsz);
	if (max_frames && nframes > max_frames) nframes = max_frames;
	if (nframes < 100) {
		fprintf(stderr, "%s: only %zu frames; need at least 100\n",
			path, nframes);
		return 1;
	}

	printf("=================================================================\n");
	printf(" com_survey: %s\n", path);
	printf(" %zu frames of %lux%lu at fs %.0f Hz  (%.1f s of data)\n",
		nframes, (unsigned long)H, (unsigned long)W, fs, nframes/fs);
	printf("=================================================================\n");

	gsl_matrix_uchar *img = gsl_matrix_uchar_alloc(H, W);
	unsigned char *raw = malloc(fsz);

	// ---------------- pass 1: camera and beam characterisation -------
	double bg_sum = 0, bg_sq = 0; size_t bg_n = 0;
	double peak_sum = 0; size_t nsat = 0, npix_tot = 0;
	double sig_sum = 0; size_t sig_n = 0;
	double flux_sum = 0;
	double cy_sum = 0, cx_sum = 0;
	size_t sample_every = nframes/2000 ? nframes/2000 : 1;
	// running background sigma, so the moment cut below has something to
	// use on the very first sampled frame
	double bg_sd = 0.5;
	for (size_t k = 0; k < nframes; k++) {
		if (fread(raw, 1, fsz, f) != fsz) { nframes = k; break; }
		if (k % sample_every) continue;
		const unsigned char *px = raw + HDR;
		// background from the frame border, peak from anywhere
		double bs = 0; size_t bn = 0; unsigned peak = 0;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				unsigned char v = px[i*W+j];
				if (i==0 || j==0 || i==H-1 || j==W-1) {
					bs += v; bn++;
					bg_sum += v; bg_sq += (double)v*v; bg_n++;
				}
				if (v > peak) peak = v;
				if (v >= 255) nsat++;
				npix_tot++;
			}
		double bg = bn ? bs/bn : 0.0;
		if (bg_n > 64) {
			double m = bg_sum/bg_n;
			double v = bg_sq/bg_n - m*m;
			bg_sd = v > 0.25 ? sqrt(v) : 0.5;
		}
		peak_sum += peak - bg;
		// flux-weighted centroid and radial second moment above bg
		double sw = 0, sy = 0, sx = 0;
		double cut = 5.0*(bg_sd > 0.5 ? bg_sd : 0.5);
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				double v = (double)px[i*W+j] - bg;
				if (v <= cut) continue;
				sw += v; sy += v*i; sx += v*j;
			}
		if (sw <= 0) continue;
		double cy = sy/sw, cx = sx/sw, sr2 = 0;
		for (uint64_t i = 0; i < H; i++)
			for (uint64_t j = 0; j < W; j++) {
				double v = (double)px[i*W+j] - bg;
				if (v <= cut) continue;
				double dy = i-cy, dx = j-cx;
				sr2 += v*(dy*dy+dx*dx);
			}
		sig_sum += sqrt(sr2/sw/2.0); sig_n++;
		flux_sum += sw; cy_sum += cy; cx_sum += cx;
	}
	double bg_mean = bg_n ? bg_sum/bg_n : 0;
	bg_sd = bg_n ? sqrt(bg_sq/bg_n - bg_mean*bg_mean) : 0;
	double amp_mean = sig_n ? peak_sum/sig_n : 0;
	double sigma = sig_n ? sig_sum/sig_n : 2.0;
	double flux_mean = sig_n ? flux_sum/sig_n : 0;
	double cy_mean = sig_n ? cy_sum/sig_n : H/2.0;
	double cx_mean = sig_n ? cx_sum/sig_n : W/2.0;
	double sat_frac = npix_tot ? (double)nsat/npix_tot : 0;

	printf("\n--- camera and beam -------------------------------------------\n");
	printf("  background        %6.2f counts, sigma %.2f\n", bg_mean, bg_sd);
	printf("  beam amplitude    %6.1f counts above background\n", amp_mean);
	printf("  beam sigma        %6.2f px   (FWHM %.2f px)\n",
		sigma, 2.3548*sigma);
	printf("  mean position     (%.2f, %.2f) px\n", cy_mean, cx_mean);
	printf("  total flux        %.0f counts\n", flux_mean);
	printf("  saturated pixels  %.4f%% %s\n", 100*sat_frac,
		sat_frac > 1e-3 ? "  <-- TOO MANY, reduce exposure or gain"
		: "(ok)");

	int sugg_thresh = (int)ceil(bg_mean + 5*bg_sd);
	size_t sugg_win = (size_t)(2*ceil(4*sigma)+1);
	if (sugg_win > H) sugg_win = H;
	if (sugg_win > W) sugg_win = W;
	printf("\n  suggested threshold   %d      (background + 5 sigma)\n",
		sugg_thresh);
	printf("  suggested sigma_init  %.2f\n", sigma);
	printf("  suggested window      %zu x %zu   (+/-4 sigma)\n",
		sugg_win, sugg_win);

	// ---------------- pass 2: run both trackers over the same frames --
	char fitcfg[700], wfscfg[700];
	if (o_fitwin) sugg_win = o_fitwin;
	snprintf(fitcfg, sizeof fitcfg,
		"{\"window_height\":%zu,\"window_width\":%zu,\"sigma_init\":%.3f,"
		"\"sigma_min\":%.3f,\"sigma_max\":%.3f,\"min_amplitude\":%.2f,"
		"\"max_iter\":4,\"robust_k\":2.5,\"robust_iter\":1,"
		"\"fit_slope\":true,\"row_time\":%g,\"reacquire_after\":10}",
		sugg_win, sugg_win, sigma, sigma*0.3 > 0.3 ? sigma*0.3 : 0.3,
		sigma*4, amp_mean*0.1 > 5 ? amp_mean*0.1 : 5.0, row_time);
	// wfs geometry sized from the measured spot: cells about one FWHM so
	// each lit cell contains curvature rather than a bare gradient, which
	// is the regime its correlation peak goes degenerate in.
	size_t cell = o_cell ? o_cell : (size_t)ceil(2.0*sigma);
	if (cell < 3) cell = 3;
	size_t rows = o_grid ? o_grid : 3, cols = o_grid ? o_grid : 3;
	size_t sr = o_sr >= 0 ? (size_t)o_sr : 2;
	while (rows*cell + 2*sr > H && rows > 1) rows--;
	while (cols*cell + 2*sr > W && cols > 1) cols--;
	size_t minvalid = o_minvalid ? o_minvalid : (rows*cols+1)/2;
	if (minvalid > rows*cols) minvalid = rows*cols;
	double minconf = o_minconf >= 0.0 ? o_minconf : 0.55;
	double fluxfloor = o_fluxfloor >= 0.0 ? o_fluxfloor
		: (amp_mean*0.5 > 5 ? amp_mean*0.5 : 5.0);
	snprintf(wfscfg, sizeof wfscfg,
		"{\"subap_height\":%zu,\"subap_width\":%zu,\"subap_rows\":%zu,"
		"\"subap_cols\":%zu,\"search_radius\":%zu,"
		"\"reject_edge_matches\":%s,\"threshold\":%d,\"ref_beta\":0.01,"
		"\"ref_beta_init\":0.25,\"ref_beta_tau\":2.0,"
		"\"min_confidence\":%.3f,\"flux_floor\":%.1f,"
		"\"min_valid_subaps\":%zu,\"rolling_shutter\":false,"
		"\"max_row_shear\":2.5,\"reacquire_after\":10}",
		cell, cell, rows, cols, sr,
		o_edge == 0 ? "false" : "true", sugg_thresh, minconf,
		fluxfloor, minvalid);
	printf("\n--- tracker configuration used -------------------------------\n");
	printf("  fit_com  window %zux%zu  sigma_init %.2f\n",
		sugg_win, sugg_win, sigma);
	printf("  wfs_com  %zux%zu cells of %zux%zu px  search_radius %zu"
		"  reject_edge %s\n"
		"           min_valid_subaps %zu  min_confidence %.2f  "
		"flux_floor %.1f\n",
		rows, cols, cell, cell, sr, o_edge == 0 ? "false" : "true",
		minvalid, minconf, fluxfloor);

	struct aylp_device fit_dev = {0}, wfs_dev = {0};
	json_object *fp = json_tokener_parse(fitcfg);
	json_object *wp = json_tokener_parse(wfscfg);
	fit_dev.params = fp; wfs_dev.params = wp;
	if (fit_com_init(&fit_dev)) { fprintf(stderr, "fit_com init failed\n"); return 1; }
	if (wfs_com_init(&wfs_dev)) { fprintf(stderr, "wfs_com init failed\n"); return 1; }
	struct aylp_fit_com_data *fd = fit_dev.device_data;
	struct aylp_wfs_com_data *wd = wfs_dev.device_data;

	struct series fy = {0}, fx = {0}, wy = {0}, wx = {0}, rres = {0};
	double *ftime = malloc(nframes*sizeof(double));
	double *wtime = malloc(nframes*sizeof(double));
	size_t frej = 0, wrej = 0, flost = 0, wlost = 0;
	// fit_com sizes its window on the first frame it sees, so the residual
	// map's extent is not known until then -- allocate the ring lazily.
	size_t nlag = 0;
	double *ring = 0;
	int lags[] = {1, 10, 100, 1000};
	double lag_num[4] = {0}, lag_da[4] = {0}, lag_db[4] = {0};
	size_t lag_n[4] = {0};

	fseek(f, 0, SEEK_SET);
	struct aylp_state st = {0};
	size_t used = 0;
	for (size_t k = 0; k < nframes; k++) {
		if (fread(raw, 1, fsz, f) != fsz) break;
		memcpy(img->data, raw+HDR, H*W);

		st.matrix_uchar = img; st.header.status = 0;
		double t0 = now_s();
		fit_com_proc(&fit_dev, &st);
		ftime[used] = 1e6*(now_s()-t0);
		int fr = (st.header.status & AYLP_FRAME_REJECTED) != 0;
		if (fr) frej++;
		if (st.header.status & AYLP_BEAM_LOST) flost++;

		st.matrix_uchar = img; st.header.status = 0;
		t0 = now_s();
		wfs_com_proc(&wfs_dev, &st);
		wtime[used] = 1e6*(now_s()-t0);
		if (st.header.status & AYLP_FRAME_REJECTED) wrej++;
		if (st.header.status & AYLP_BEAM_LOST) wlost++;
		used++;

		if (k < 200) continue;	// let both settle
		ser_add(&fy, (fd->last_y+1)/2*(H-1));
		ser_add(&fx, (fd->last_x+1)/2*(W-1));
		ser_add(&wy, (wd->last_y+1)/2*(H-1));
		ser_add(&wx, (wd->last_x+1)/2*(W-1));
		if (!fr) ser_add(&rres, fd->last_rms);

		// residual-map autocorrelation at several lags: does the part
		// of the beam the gaussian cannot explain stay put, or boil?
		if (!ring) {
			nlag = fd->win_h*fd->win_w;
			if (!nlag) continue;
			ring = calloc(MAXLAG*nlag, sizeof(double));
		}
		double *slot = ring + (k % MAXLAG)*nlag;
		memcpy(slot, fd->resid, nlag*sizeof(double));
		for (int L = 0; L < 4; L++) {
			size_t lag = (size_t)lags[L];
			if (k < 200 + lag || lag >= MAXLAG) continue;
			double *old = ring + ((k-lag) % MAXLAG)*nlag;
			double sa=0, sb=0, saa=0, sbb=0, sab=0;
			for (size_t i = 0; i < nlag; i++) {
				sa += slot[i]; sb += old[i];
				saa += slot[i]*slot[i]; sbb += old[i]*old[i];
				sab += slot[i]*old[i];
			}
			double na = nlag;
			double va = saa/na - (sa/na)*(sa/na);
			double vb = sbb/na - (sb/na)*(sb/na);
			double cv = sab/na - (sa/na)*(sb/na);
			if (va > 1e-12 && vb > 1e-12) {
				lag_num[L] += cv/sqrt(va*vb);
				lag_n[L]++;
			}
			lag_da[L] += va; lag_db[L] += vb;
		}
	}
	fclose(f);

	// ---------------- report -----------------------------------------
	double resid = ser_mean(&rres);
	double rel_resid = amp_mean > 0 ? resid/amp_mean : 0;
	double corr[4];
	for (int L = 0; L < 4; L++)
		corr[L] = lag_n[L] ? lag_num[L]/lag_n[L] : 0.0;
	// normalise by lag 1 so per-frame photon noise, which decorrelates
	// instantly and is not speckle, does not masquerade as boiling
	double persist = corr[0] > 0.05 ? corr[2]/corr[0] : 0.0;
	if (persist > 1.0) persist = 1.0;
	if (persist < 0.0) persist = 0.0;

	printf("\n--- gaussian model fit (fit_com) ------------------------------\n");
	printf("  residual rms      %6.3f counts\n", resid);
	printf("  relative residual %6.4f   (residual / amplitude)\n", rel_resid);
	printf("      < 0.010  beam is gaussian to within noise\n");
	printf("      > 0.035  substantial departure from gaussian\n");

	printf("\n--- is the departure frozen or boiling? -----------------------\n");
	printf("  residual-map correlation:  lag1 %.3f  lag10 %.3f  "
		"lag100 %.3f  lag1000 %.3f\n", corr[0], corr[1], corr[2], corr[3]);
	printf("  persistence (lag100/lag1)  %.3f   "
		"(1.0 = frozen, 0.0 = boiling)\n", persist);
	if (corr[2] > 0.01 && corr[3] > 0.01 && corr[2] > corr[3]) {
		double tau = 100.0*log(corr[0]/corr[3])/log(corr[0]/corr[2]);
		(void)tau;
	}

	qsort(ftime, used, sizeof(double), cmpd);
	qsort(wtime, used, sizeof(double), cmpd);
	double period_us = 1e6/fs;
	printf("\n--- cost per frame (this machine) -----------------------------\n");
	printf("  fit_com  p50 %6.1f us  p99 %6.1f us  max %6.1f us   "
		"(%.0f%% of the %.0f us period at p99)\n",
		ftime[used/2], ftime[(size_t)(used*0.99)], ftime[used-1],
		100*ftime[(size_t)(used*0.99)]/period_us, period_us);
	printf("  wfs_com  p50 %6.1f us  p99 %6.1f us  max %6.1f us   "
		"(%.0f%%)\n",
		wtime[used/2], wtime[(size_t)(used*0.99)], wtime[used-1],
		100*wtime[(size_t)(used*0.99)]/period_us);

	printf("\n--- what each tracker reported --------------------------------\n");
	printf("  %-9s %-12s %-12s %-11s %-9s %s\n", "", "rms y (px)",
		"rms x (px)", "hf y (px)", "rejected", "beam_lost");
	printf("  %-9s %-12.4f %-12.4f %-11.4f %-9.2f%% %zu\n", "fit_com",
		ser_rms_about_mean(&fy), ser_rms_about_mean(&fx), ser_hf(&fy),
		100.0*frej/used, flost);
	printf("  %-9s %-12.4f %-12.4f %-11.4f %-9.2f%% %zu\n", "wfs_com",
		ser_rms_about_mean(&wy), ser_rms_about_mean(&wx), ser_hf(&wy),
		100.0*wrej/used, wlost);
	printf("  hf = rms of the frame-to-frame difference / sqrt(2): the\n"
		"       high-frequency content, which is what the loop must\n"
		"       reject and where fabricated motion shows up first.\n");
	// disagreement between the two, which bounds how wrong at least one is
	double dy = 0, dx = 0;
	size_t nd = fy.n < wy.n ? fy.n : wy.n;
	for (size_t i = 0; i < nd; i++) {
		double a = (fy.v[i]-ser_mean(&fy)) - (wy.v[i]-ser_mean(&wy));
		double b = (fx.v[i]-ser_mean(&fx)) - (wx.v[i]-ser_mean(&wx));
		dy += a*a; dx += b*b;
	}
	printf("  they disagree by  %.4f px rms (y), %.4f px rms (x)\n",
		nd ? sqrt(dy/nd) : 0.0, nd ? sqrt(dx/nd) : 0.0);

	// ---------------- recommendation ---------------------------------
	printf("\n=================================================================\n");
	const char *pick; const char *why;
	if (sat_frac > 1e-3) {
		pick = "NEITHER YET";
		why = "the beam is saturating, which destroys sub-pixel accuracy\n"
		"  for both trackers. Reduce exposure or gain and re-run this\n"
		"  survey before choosing.";
	} else if (rel_resid < 0.010) {
		pick = "fit_com";
		why = "the beam is gaussian to within noise, which is fit_com's\n"
		"  best case: far better accuracy, absolute position, and rolling\n"
		"  shutter handled as a fitted parameter if it ever appears.";
	} else if (persist > 0.70) {
		pick = "fit_com";
		why = "the beam departs from gaussian, but that departure is\n"
		"  FROZEN. A static model error is a static bias, which the loop\n"
		"  absorbs; it does not become fabricated motion. fit_com still\n"
		"  wins on the part that matters.";
	} else if (rel_resid > 0.035) {
		pick = "wfs_com";
		why = "the beam departs substantially from gaussian AND that\n"
		"  departure is BOILING. A moving model error becomes fabricated\n"
		"  motion that fit_com feeds to the controller as real. wfs_com\n"
		"  correlates against a learned template, so it does not care what\n"
		"  the shape is -- only that it is stable -- and boiling speckle\n"
		"  costs it noise rather than a wandering centre.";
	} else {
		pick = "fit_com (marginal)";
		why = "this sits between the clear cases: a moderate,\n"
		"  partly-boiling departure from gaussian. fit_com is the better\n"
		"  starting point, but compare the hf columns above -- if fit_com's\n"
		"  high-frequency content is much larger than wfs_com's on a beam\n"
		"  you believe is quiet, that excess is fabricated and you should\n"
		"  switch.";
	}
	printf(" RECOMMENDATION:  %s\n", pick);
	printf("=================================================================\n");
	printf("  %s\n", why);

	printf("\n  Caveats you should not skip:\n");
	printf("  - This capture has NO rolling-shutter shear in it (open loop,\n"
	       "    beam near rest). Shear only ever helps fit_com, whose model\n"
	       "    fits it, and only ever hurts wfs_com, which cannot correct\n"
	       "    it. So this survey is the WORST case for fit_com -- re-run\n"
	       "    it under real vibration before reversing a fit_com verdict.\n");
	printf("  - rms/hf above are not accuracy. Real beam motion is in them\n"
	       "    too, and there is no ground truth here. They are only\n"
	       "    comparable BETWEEN the two trackers on the same frames.\n");
	printf("  - The thresholds (0.010 / 0.035 / 0.70) come from a synthetic\n"
	       "    multiplicative-speckle model, not from your optics. Near a\n"
	       "    boundary, trust the hf comparison over the verdict.\n");

	printf("\n  Suggested wfs_com geometry for this beam: %zux%zu cells of\n"
	       "  %zux%zu px, search_radius %zu, min_valid_subaps %zu.\n",
		rows, cols, cell, cell, sr, (rows*cols+1)/2);
	printf("  Check that against the per-frame rejection rate above: if\n"
	       "  wfs_com rejected a large fraction, min_valid_subaps is more\n"
	       "  than the number of cells your beam actually lights.\n");

	fit_com_fini(&fit_dev); wfs_com_fini(&wfs_dev);
	json_object_put(fp); json_object_put(wp);
	free(fy.v); free(fx.v); free(wy.v); free(wx.v); free(rres.v);
	free(ftime); free(wtime); free(ring); free(raw);
	gsl_matrix_uchar_free(img);
	return 0;
}
