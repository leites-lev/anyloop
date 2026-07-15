/* Measurement-noise-inclusive d-step linear prediction bound from an
 * attenuation_test one-sided open-loop PSD. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MAXBIN 8192
#define MAXP 512
#define PI 3.14159265358979323846

static int solve(double a[MAXP][MAXP], double *b, double *x, int n)
{
	for (int k = 0; k < n; k++) {
		int p = k;
		for (int i = k + 1; i < n; i++)
			if (fabs(a[i][k]) > fabs(a[p][k])) p = i;
		if (fabs(a[p][k]) < 1e-24) return -1;
		for (int j = k; j < n; j++) {
			double t = a[k][j]; a[k][j] = a[p][j]; a[p][j] = t;
		}
		double t = b[k]; b[k] = b[p]; b[p] = t;
		for (int i = k + 1; i < n; i++) {
			double q = a[i][k] / a[k][k];
			for (int j = k; j < n; j++) a[i][j] -= q * a[k][j];
			b[i] -= q * b[k];
		}
	}
	for (int i = n - 1; i >= 0; i--) {
		double s = b[i];
		for (int j = i + 1; j < n; j++) s -= a[i][j] * x[j];
		x[i] = s / a[i][i];
	}
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 2 || argc > 3) return 2;
	double delay = argc == 3 ? atof(argv[2]) : 5.0;
	if (delay < 0.0 || delay > 5.0) return 2;
	FILE *fp = fopen(argv[1], "r");
	if (!fp) return 3;
	double f[MAXBIN], py[MAXBIN], px[MAXBIN];
	char line[1024]; int n = 0;
	while (n < MAXBIN && fgets(line, sizeof line, fp)) {
		double yc, ya, xc, xa;
		if (sscanf(line, "%lf%lf%lf%lf%lf%lf%lf",
			&f[n], &py[n], &yc, &ya, &px[n], &xc, &xa) == 7) n++;
	}
	fclose(fp);
	if (n < 2) return 4;
	double df = f[1] - f[0], fs = 2.0 * f[n - 1], r[2][MAXP + 6];
	for (int axis = 0; axis < 2; axis++)
		for (int lag = 0; lag < MAXP + 6; lag++) {
			double s = 0.0;
			for (int k = 0; k < n; k++) {
				double p = axis ? px[k] : py[k];
				s += p * cos(2.0 * PI * f[k] * lag / fs) * df;
			}
			r[axis][lag] = s;
		}
	for (int p = 8; p <= MAXP; p *= 2) {
		printf("order %d", p);
		for (int axis = 0; axis < 2; axis++) {
			static double A[MAXP][MAXP]; double b[MAXP], target[MAXP], x[MAXP] = {0};
			for (int i = 0; i < p; i++) {
				b[i] = 0.0;
				for (int k = 0; k < n; k++) {
					double ps = axis ? px[k] : py[k];
					b[i] += ps * cos(2.0 * PI * f[k] * (delay+i) / fs) * df;
				}
				target[i] = b[i];
				for (int j = 0; j < p; j++) A[i][j] = r[axis][abs(i-j)];
			}
			if (solve(A, b, x, p)) return 5;
			double mse = r[axis][0];
			for (int i = 0; i < p; i++) mse -= x[i] * target[i];
			printf("  %c %.4f px", axis ? 'x' : 'y', 31.5 * sqrt(fmax(0,mse)));
		}
		putchar('\n');
	}
}
