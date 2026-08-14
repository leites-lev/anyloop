/* fsp broadband-path replay: NLMS closed loop, disturbance from stdin.
 *
 * in : float64 disturbance stream (px units already applied upstream)
 * out: float64 closed-loop error stream
 * argv: K_true K_mod d_int d_frac mu mu_init mu_tau leak lp order
 *       [noise_sigma [dark_run dark_period dark_predict]]
 *
 * Mirrors fsp.c's Smith reconstruction, boxcar prefilter, provenance-aware
 * dual NLMS predictors, and direct multi-horizon dark-frame command bank.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int main(int argc, char **argv)
{
	double Kt = atof(argv[1]), Km = atof(argv[2]);
	int di = atoi(argv[3]);
	double df = atof(argv[4]);
	double mu = atof(argv[5]), mu0 = atof(argv[6]), mutau = atof(argv[7]);
	double leak = atof(argv[8]);
	int lp = atoi(argv[9]), P = atoi(argv[10]);
	double nsig = argc > 11 ? atof(argv[11]) : 0.0;
	int dark_run = argc > 12 ? atoi(argv[12]) : 0;
	int dark_period = argc > 13 ? atoi(argv[13]) : 0;
	int dark_predict = argc > 14 ? atoi(argv[14]) : 0;
	unsigned long rs = 12345;
	double fs = 3788.0, hold = 20.0, ramp = 10.0, clamp = 1.0;
	int gd = (lp - 1) / 2, bd = di + gd;

	size_t cap = 1 << 22, n = 0;
	double *phi = malloc(cap * sizeof *phi);
	while (fread(phi + n, sizeof *phi, 1, stdin) == 1)
		if (++n == cap) phi = realloc(phi, (cap *= 2) * sizeof *phi);

	int dmax = 2 * bd;
	if (dmax < 4) dmax = 4;
	int bankmax = dmax + bd + 1;
	size_t H = P + bankmax + 2;
	double *w = calloc(P, sizeof *w), *wn = calloc(P, sizeof *wn);
	double *dw = calloc((size_t)bankmax * P, sizeof *dw);
	double *bl = calloc(n + 8, sizeof *bl);
	double *blmask = calloc(n + 8, sizeof *blmask);
	unsigned char *syn = calloc(n + 8, 1);
	double *e = calloc(n + 8, sizeof *e);
	double *ud = calloc(n + di + 8, sizeof *ud);
	double *lpb = calloc(lp, sizeof *lpb);
	double a_th = (1.0 - df) / (1.0 + df), tzx = 0, tzy = 0, lpsum = 0;
	size_t n0 = H + 6, dark_streak = 0, live_streak = 0;
	size_t dh = 0, predicted = 0;
	double u_last = 0.0;

	for (size_t i = 0; i < n; i++) {
		double pu = ud[i];
		/* fresh measurement noise: corrupts what the loop SEES and what we score */
		double nz = 0.0;
		if (nsig > 0.0) {
			rs = rs * 6364136223846793005UL + 1442695040888963407UL;
			double u1 = ((rs >> 11) + 0.5) / 9007199254740992.0;
			rs = rs * 6364136223846793005UL + 1442695040888963407UL;
			double u2 = ((rs >> 11) + 0.5) / 9007199254740992.0;
			nz = nsig * sqrt(-2*log(u1)) * cos(6.283185307179586*u2);
		}
		e[i] = phi[i] + Kt * pu + nz;
		int dark = dark_run > 0 && dark_period > dark_run
			&& (int)(i % (size_t)dark_period) >= dark_period - dark_run;
		double pm = dark ? 0.0 : e[i] - Km * pu;
		lpsum += pm - lpb[i % lp];
		lpb[i % lp] = pm;
		blmask[i] = lpsum / lp;
		bl[i] = blmask[i];
		if (dark) {
			dark_streak++;
			live_streak = 0;
			syn[i] = 1;
		} else {
			dark_streak = 0;
			live_streak++;
		}
		double t = i / fs, bh = 0.0;
		int dark_commandable = 0;
		if (i >= n0) {
			double muc = mu + (mu0 > mu ? (mu0 - mu) *
				exp(-(t > hold ? t - hold : 0.0) / mutau) : 0.0);
			size_t i0 = i - bd;
			double p1 = 0, p2 = 0, en1 = 1e-12, en2 = 1e-12;
			for (int k = 0; k < P; k++) {
				double v1 = bl[i0 - k], v2 = bl[i0 - 1 - k];
				p1 += w[k] * v1;  en1 += v1 * v1;
				p2 += wn[k] * v2; en2 += v2 * v2;
			}
			if (!dark && live_streak >= (size_t)lp) {
				double eu1 = 1e-12, eu2 = 1e-12;
				int real1 = 0, real2 = 0;
				for (int k = 0; k < P; k++) {
					if (!syn[i0-k]) { eu1 += bl[i0-k]*bl[i0-k]; real1++; }
					if (!syn[i0-1-k]) { eu2 += bl[i0-1-k]*bl[i0-1-k]; real2++; }
				}
				double rp1 = 0.0, rp2 = 0.0;
				for (int k = 0; k < P; k++) {
					if (!syn[i0-k]) rp1 += w[k]*bl[i0-k];
					if (!syn[i0-1-k]) rp2 += wn[k]*bl[i0-1-k];
				}
				double s1 = muc * (bl[i] - rp1) / eu1;
				double s2 = muc * (bl[i] - rp2) / eu2;
				double lk = 1.0 - leak;
				for (int k = 0; k < P; k++) {
					w[k] *= lk; wn[k] *= lk;
					if (real1 >= P/2 && !syn[i0-k]) w[k] += s1*bl[i0-k];
					if (real2 >= P/2 && !syn[i0-1-k]) wn[k] += s2*bl[i0-1-k];
				}
				/* Production trains four bank horizons per live frame. */
				for (int q = 0; q < 4; q++) {
					size_t h = ++dh;
					if (dh == (size_t)bankmax) dh = 0;
					double *wk = dw + (h - 1) * P;
					double dp = 0.0, de = 1e-12;
					for (int k = 0; k < P; k++) {
						double v = blmask[i-h-k];
						dp += wk[k]*v; de += v*v;
					}
					double ds = muc * (bl[i] - dp) / de;
					if (isfinite(ds)) for (int k = 0; k < P; k++)
						wk[k] += ds * blmask[i-h-k];
				}
			}
			double c1 = 0, c2 = 0;
			for (int k = 0; k < P; k++) {
				c1 += w[k] * bl[i - k];
				c2 += wn[k] * bl[i - k];
			}
			bh = (1.0 - df) * c1 + df * c2;
			if (!isfinite(bh)) bh = 0.0;
			if (dark && dark_predict) {
				size_t hc = dark_streak + bd;
				if (dark_streak <= (size_t)dmax
						&& hc + 1 <= (size_t)bankmax) {
					double *wc = dw + (hc - 1) * P;
					double dc = 0.0, dcn = 0.0;
					size_t end = i - dark_streak;
					for (int k = 0; k < P; k++) {
						dc += wc[k] * blmask[end-k];
						dcn += wc[P+k] * blmask[end-k];
					}
					bh = (1.0 - df)*dc + df*dcn;
					dark_commandable = isfinite(bh);
					if (dark_commandable) predicted++;
				}
			}
		}
		double frac = t < hold ? 0.0 : ((t - hold) / ramp < 1.0 ? (t - hold) / ramp : 1.0);
		double u = dark && !dark_commandable
			? u_last : -frac * bh / Km;
		if (u > clamp) u = clamp;
		if (u < -clamp) u = -clamp;
		double yt = a_th * u + tzx - a_th * tzy;
		u_last = u;
		tzx = u; tzy = yt;
		ud[i + di] = yt;
	}
	fwrite(e, sizeof *e, n, stdout);
	if (dark_period) fprintf(stderr,
		"dark predictor: %zu frame(s) directly commanded\n", predicted);
	return 0;
}

/* BUILD:  gcc -O3 -march=native -ffast-math -o fsp_replay contrib/diagnostics/fsp_replay.c -lm
 *
 * USAGE:  fsp_replay K_true K_mod d_int d_frac mu mu_init mu_tau leak lp order
 *           [noise_sigma [dark_run dark_period dark_predict]]
 *           < disturbance.f64 > closed_error.f64
 *
 * Feed it a float64 stream of open-loop disturbance (e.g. the open phase of an
 * atten_par_err*.aylp, stride 56, 40-byte header, [y,x] doubles) and it returns
 * the closed-loop error the fsp broadband path would have produced.
 *
 * 465 s of 3788 Hz data with 512 taps runs in ~0.8 s (560x real time); the
 * equivalent numpy loop took 73 s. Verified to reproduce that implementation to
 * 3 significant figures.
 *
 * VALIDITY, MEASURED 2026-07-31: matches hardware below 100 Hz (1-30 Hz sim
 * 15.5x vs hardware 17.1x; 60-100 Hz 2.16x vs 2.04x) but is 3.7x OPTIMISTIC at
 * 100-150 Hz and 2.8x at 250-400 Hz. Trust it for low-frequency questions.
 * Do NOT use it to predict a total RMS. See the memory note
 * plant-biquad-unused-2026-07-31 for what has been ruled out as the cause.
 */
