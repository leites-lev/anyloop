#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/wait.h>
#include <time.h>

#include "anyloop.h"
#include "logging.h"
#include "bode_plot.h"
#include "xalloc.h"

static int fg_serial_open(const char *port)
{
	int fd = open(port, O_RDWR | O_NOCTTY);
	if (fd < 0) return -1;
	struct termios tty;
	if (tcgetattr(fd, &tty)) { close(fd); return -1; }
	cfmakeraw(&tty);
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 5;
	if (tcsetattr(fd, TCSANOW, &tty)) { close(fd); return -1; }
	return fd;
}

static void fg_drain(int fd)
{
	char c;
	while (read(fd, &c, 1) == 1 && c != '\n')
		;
}

static int fg_send(int fd, const char *cmd)
{
	size_t n = strlen(cmd);
	if (write(fd, cmd, n) != (ssize_t)n) return -1;
	fg_drain(fd);
	return 0;
}


static const char *PLOT_SCRIPT =
	"import sys\n"
	"import matplotlib\n"
	"matplotlib.use('PDF')\n"
	"import matplotlib.pyplot as plt\n"
	"import numpy as np\n"
	"data=np.loadtxt(sys.argv[1])\n"
	"freqs,mags,phases=data[:,0],data[:,1],data[:,2]\n"
	"ok=mags>1e-6\n"
	"freqs,mags,phases=freqs[ok],mags[ok],phases[ok]\n"
	"fig,(ax1,ax2)=plt.subplots(2,1,figsize=(10,8),sharex=True)\n"
	"fig.suptitle('Bode Plot')\n"
	"ax1.semilogx(freqs,20*np.log10(mags+1e-300))\n"
	"ax1.set_ylabel('Magnitude (dB)')\n"
	"ax1.grid(True,which='both',ls='--',alpha=0.5)\n"
	"ax2.semilogx(freqs,np.degrees(np.unwrap(phases)))\n"
	"ax2.set_ylabel('Phase (deg)')\n"
	"ax2.set_xlabel('Frequency (Hz)')\n"
	"ax2.grid(True,which='both',ls='--',alpha=0.5)\n"
	"plt.tight_layout()\n"
	"plt.savefig(sys.argv[2],format='pdf',bbox_inches='tight')\n"
	"print('Saved Bode plot to '+sys.argv[2])\n";


static int generate_plot(struct aylp_bode_plot_data *data)
{
	char data_path[] = "/tmp/bode_data_XXXXXX";
	int fd = mkstemp(data_path);
	if (fd < 0) {
		log_error("mkstemp failed: %m");
		return -1;
	}
	FILE *f = fdopen(fd, "w");
	if (!f) {
		log_error("fdopen for bode data failed: %m");
		close(fd);
		unlink(data_path);
		return -1;
	}
	for (size_t k = 0; k < data->n_freqs; k++)
		fprintf(f, "%g %g %g\n", data->freqs[k], data->mags[k], data->phases[k]);
	fclose(f);

	char script_path[64];
	snprintf(script_path, sizeof(script_path), "/tmp/bode_plot_%d.py", getpid());
	FILE *script = fopen(script_path, "w");
	if (!script) {
		log_error("fopen for plot script failed: %m");
		unlink(data_path);
		return -1;
	}
	fputs(PLOT_SCRIPT, script);
	fclose(script);

	char cmd[512];
	snprintf(cmd, sizeof(cmd), "python3 %s %s %s", script_path, data_path, data->output_file);
	int ret = system(cmd);
	int ok = (ret != -1) && WIFEXITED(ret) && (WEXITSTATUS(ret) == 0);
	if (!ok)
		log_error("Bode plot script failed (exit status %d); "
			"is python3 with numpy and matplotlib installed?",
			(ret != -1 && WIFEXITED(ret)) ? WEXITSTATUS(ret) : ret);
	else
		log_info("Bode plot saved to %s", data->output_file);

	// save data alongside the PDF for offline fitting
	{
		const char *out = data->output_file;
		const char *dot = strrchr(out, '.');
		size_t base = dot ? (size_t)(dot - out) : strlen(out);
		char *dat_path = xmalloc(base + 5);
		memcpy(dat_path, out, base);
		memcpy(dat_path + base, ".dat", 5);
		FILE *fdat = fopen(dat_path, "w");
		if (fdat) {
			for (size_t k = 0; k < data->n_freqs; k++)
				fprintf(fdat, "%g %g %g\n",
					data->freqs[k], data->mags[k], data->phases[k]);
			fclose(fdat);
			log_info("Bode data saved to %s", dat_path);
		}
		xfree(dat_path);
	}

	unlink(data_path);
	unlink(script_path);
	return ok ? 0 : -1;
}

static inline double get_time_seconds(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// get the scalar of interest from the current pipeline state
static double get_element(struct aylp_state *state, size_t elem)
{
	switch (state->header.type) {
	case AYLP_T_VECTOR:
		return gsl_vector_get(state->vector, elem);
	case AYLP_T_MATRIX:
		return gsl_matrix_get(state->matrix,
			elem / state->matrix->size2,
			elem % state->matrix->size2);
	default:
		return 0.0;
	}
}

// write to element in the current pipeline state (add or replace)
static void write_element(struct aylp_state *state, size_t elem,
	double val, unsigned inject_mode)
{
	double out = val;
	switch (state->header.type) {
	case AYLP_T_VECTOR:
		if (inject_mode == BODE_INJECT_ADD)
			out += gsl_vector_get(state->vector, elem);
		gsl_vector_set(state->vector, elem, out);
		break;
	case AYLP_T_MATRIX: {
		size_t row = elem / state->matrix->size2;
		size_t col = elem % state->matrix->size2;
		if (inject_mode == BODE_INJECT_ADD)
			out += gsl_matrix_get(state->matrix, row, col);
		gsl_matrix_set(state->matrix, row, col, out);
		break;
	}
	default:
		break;
	}
}

// update n_avg_cur and physical time trackers for freq_idx
static void advance_frequency(struct aylp_bode_plot_data *data)
{
	double f = data->freqs[data->freq_idx];
	double now = get_time_seconds();

	data->start_time = now;
	data->prev_time  = now;
	data->total_time = 0.0;
	
	size_t n = (size_t)(data->n_cycles * data->sample_rate / f);
	data->n_avg_cur = (n < 50) ? 50 : n;
}


int bode_plot_init(struct aylp_device *self)
{
	self->proc = &bode_plot_proc;
	self->fini = &bode_plot_fini;
	self->device_data = xcalloc(1, sizeof(struct aylp_bode_plot_data));
	struct aylp_bode_plot_data *data = self->device_data;

	// defaults
	data->inject_mode  = BODE_INJECT_ADD;
	data->freq_start   = 1.0;
	data->freq_end     = 0.0;   // resolved to Nyquist below if unset
	data->n_freqs      = 30;
	data->n_settle     = 50;
	data->n_cycles     = 10.0;
	data->amplitude    = 0.1;
	data->output_mode  = BODE_OUTPUT_RAW;
	data->element      = 0;
	data->output_file  = xstrdup("bode_plot.pdf");
	data->fg_fd        = -1;
	data->fg_board     = 0;
	data->fg_channel   = 1;
	data->fg_level     = 1;
	data->fg_offset    = 0.0;

	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
			// comment
		} else if (!strcmp(key, "sample_rate")) {
			data->sample_rate = json_object_get_double(val);
			log_trace("sample_rate = %G Hz", data->sample_rate);
		} else if (!strcmp(key, "freq_start")) {
			data->freq_start = json_object_get_double(val);
			log_trace("freq_start = %G Hz", data->freq_start);
		} else if (!strcmp(key, "freq_end")) {
			data->freq_end = json_object_get_double(val);
			log_trace("freq_end = %G Hz", data->freq_end);
		} else if (!strcmp(key, "n_freqs")) {
			data->n_freqs = json_object_get_uint64(val);
			log_trace("n_freqs = %zu", data->n_freqs);
		} else if (!strcmp(key, "n_settle")) {
			data->n_settle = json_object_get_uint64(val);
			log_trace("n_settle = %zu", data->n_settle);
		} else if (!strcmp(key, "n_cycles")) {
			data->n_cycles = json_object_get_double(val);
			log_trace("n_cycles = %G", data->n_cycles);
		} else if (!strcmp(key, "amplitude")) {
			data->amplitude = json_object_get_double(val);
			log_trace("amplitude = %G", data->amplitude);
		} else if (!strcmp(key, "inject")) {
			const char *s = json_object_get_string(val);
			if (!strcmp(s, "add"))
				data->inject_mode = BODE_INJECT_ADD;
			else if (!strcmp(s, "replace"))
				data->inject_mode = BODE_INJECT_REPLACE;
			else
				log_error("Unknown inject mode \"%s\"; use \"add\" or \"replace\"", s);
			log_trace("inject = %s", s);
		} else if (!strcmp(key, "output")) {
			const char *s = json_object_get_string(val);
			if (!strcmp(s, "raw"))
				data->output_mode = BODE_OUTPUT_RAW;
			else if (!strcmp(s, "diff"))
				data->output_mode = BODE_OUTPUT_DIFF;
			else
				log_error("Unknown output mode \"%s\"; use \"raw\" or \"diff\"", s);
			log_trace("output = %s", s);
		} else if (!strcmp(key, "element")) {
			data->element = json_object_get_uint64(val);
			log_trace("element = %zu", data->element);
		} else if (!strcmp(key, "freq_linear")) {
			data->freq_linear = json_object_get_boolean(val);
			log_trace("freq_linear = %s", data->freq_linear ? "true" : "false");
		} else if (!strcmp(key, "output_file")) {
			xfree(data->output_file);
			data->output_file = xstrdup(json_object_get_string(val));
			log_trace("output_file = %s", data->output_file);
		} else if (!strcmp(key, "fg_port")) {
			xfree(data->fg_port);
			data->fg_port = xstrdup(json_object_get_string(val));
			log_trace("fg_port = %s", data->fg_port);
		} else if (!strcmp(key, "fg_board")) {
			data->fg_board = (int)json_object_get_int64(val);
			log_trace("fg_board = %d", data->fg_board);
		} else if (!strcmp(key, "fg_channel")) {
			data->fg_channel = (int)json_object_get_int64(val);
			log_trace("fg_channel = %d", data->fg_channel);
		} else if (!strcmp(key, "fg_level")) {
			data->fg_level = (int)json_object_get_int64(val);
			log_trace("fg_level = %d", data->fg_level);
		} else if (!strcmp(key, "fg_offset")) {
			data->fg_offset = json_object_get_double(val);
			log_trace("fg_offset = %g V", data->fg_offset);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	if (!data->sample_rate) {
		log_error("sample_rate is required.");
		return -1;
	}
	if (!data->n_freqs) {
		log_error("n_freqs must be > 0.");
		return -1;
	}
	if (!data->freq_end)
		data->freq_end = data->sample_rate / 2.0;
	if (data->freq_start <= 0.0) {
		log_error("freq_start must be > 0.");
		return -1;
	}

	data->freqs  = xcalloc(data->n_freqs, sizeof(double));
	data->mags   = xcalloc(data->n_freqs, sizeof(double));
	data->phases = xcalloc(data->n_freqs, sizeof(double));

	// frequency points: linear or log spaced
	for (size_t k = 0; k < data->n_freqs; k++) {
		double t = (data->n_freqs > 1) ? (double)k / (data->n_freqs - 1) : 0.0;
		if (data->freq_linear) {
			data->freqs[k] = data->freq_start + t * (data->freq_end - data->freq_start);
		} else {
			double log_start = log10(data->freq_start);
			double log_end   = log10(data->freq_end);
			data->freqs[k] = pow(10.0, log_start + t * (log_end - log_start));
		}
	}

	data->freq_idx   = 0;
	data->step_count = 0;
	data->phase      = 0.0;
	data->prev_value = 0.0;
	data->Y_cos      = 0.0;
	data->Y_sin      = 0.0;
	advance_frequency(data);

	log_info("Sweeping %zu frequencies from %G to %G Hz (~%g cycles each)",
		data->n_freqs, data->freq_start, data->freq_end, data->n_cycles);

	if (data->fg_port) {
		data->fg_fd = fg_serial_open(data->fg_port);
		if (data->fg_fd < 0) {
			log_error("bode_plot: open FG port %s: %s",
				data->fg_port, strerror(errno));
			return -1;
		}
		char cmd[64];
		snprintf(cmd, sizeof cmd, "DAQC2.fgTYPE(%d, %d, 1)\n",
			data->fg_board, data->fg_channel);
		if (fg_send(data->fg_fd, cmd)) {
			log_error("bode_plot: FG init failed: %s", strerror(errno));
			return -1;
		}
		snprintf(cmd, sizeof cmd, "DAQC2.fgLEVEL(%d, %d, %d)\n",
			data->fg_board, data->fg_channel, data->fg_level);
		fg_send(data->fg_fd, cmd);
		if (data->fg_offset > 0.0) {
			// setDAC on the corresponding DAC channel sets the DC center;
			// the FG oscillates around that value when fgON is called.
			// FG channel N maps to DAC channel N-1.
			snprintf(cmd, sizeof cmd, "DAQC2.setDAC(%d, %d, %.4f)\n",
				data->fg_board, data->fg_channel - 1, data->fg_offset);
			fg_send(data->fg_fd, cmd);
		}
		log_info("bode_plot: hardware FG on %s board=%d channel=%d level=%d offset=%.3fV",
			data->fg_port, data->fg_board, data->fg_channel, data->fg_level,
			data->fg_offset > 0.0 ? data->fg_offset : 2.048);
	}

	self->type_in  = AYLP_T_VECTOR | AYLP_T_MATRIX;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_UNCHANGED;
	self->units_out = AYLP_U_UNCHANGED;

	return 0;
}


int bode_plot_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct aylp_bode_plot_data *data = self->device_data;

	if (data->freq_idx >= data->n_freqs) {
		state->header.status |= AYLP_DONE;
		return 0;
	}

	// 1. Initialize clock baseline cleanly on the very first sample of each frequency.
	// In FG mode, program the hardware FG first so start_time is captured right
	// after fgON completes — the FG is running from that moment and elapsed is the
	// correct phase reference.
	if (data->step_count == 0) {
		if (data->fg_fd >= 0) {
			double f = data->freqs[data->freq_idx];
			int f_int = (int)round(f);
			if (f_int < 10)    f_int = 10;
			if (f_int > 10000) f_int = 10000;
			char cmd[64];
			snprintf(cmd, sizeof cmd, "DAQC2.fgFREQ(%d, %d, %d)\n",
				data->fg_board, data->fg_channel, f_int);
			fg_send(data->fg_fd, cmd);
			snprintf(cmd, sizeof cmd, "DAQC2.fgON(%d, %d)\n",
				data->fg_board, data->fg_channel);
			fg_send(data->fg_fd, cmd);
			data->freqs[data->freq_idx] = (double)f_int;
		}
		double now = get_time_seconds();
		data->start_time = now;
		data->prev_time  = now;
		data->total_time = 0.0;
	}

	// 2. read output signal y[n] before modifying state
	double raw = get_element(state, data->element);
	double y = (data->output_mode == BODE_OUTPUT_DIFF)
		? raw - data->prev_value
		: raw;
	data->prev_value = raw;

	// 3. Measure time delta
	double now = get_time_seconds();
	double dt = now - data->prev_time;
	data->prev_time = now;
	double elapsed = now - data->start_time;

	// 4. Compute phase and injection target
	double f = data->freqs[data->freq_idx];
	double phi = 2.0 * M_PI * f * elapsed;
	double u   = data->amplitude * sin(phi);

	// 5. Calculate current integrand terms using prev_phi: y[n] is the response
	// to u[n-1] which was injected at prev_phi, not the current phi.
	double corr_phi = (data->step_count > 0) ? data->prev_phi : phi;
	double curr_cos_term = y * cos(corr_phi);
	double curr_sin_term = y * sin(corr_phi);

	// 6. Accumulate using trapezoidal integration to eliminate high-frequency phase shift
	if (data->step_count >= data->n_settle) {
		if (data->step_count == data->n_settle) {
			// First integration sample: establish the historical baseline term
			data->prev_cos_term = curr_cos_term;
			data->prev_sin_term = curr_sin_term;
		} else {
			data->Y_cos      += 0.5 * (data->prev_cos_term + curr_cos_term) * dt;
			data->Y_sin      += 0.5 * (data->prev_sin_term + curr_sin_term) * dt;
			data->total_time += dt;
			
			data->prev_cos_term = curr_cos_term;
			data->prev_sin_term = curr_sin_term;
		}
	}

	// 7. write injection to pipeline state (software mode only; FG drives hardware directly)
	if (data->fg_fd < 0)
		write_element(state, data->element, u, data->inject_mode);
	data->phase    = phi;
	data->prev_phi = phi;
	data->step_count++;

	// 8. Evaluate termination condition using strict time thresholds
	double target_duration = data->n_cycles / f;
	if (data->step_count < data->n_settle || data->total_time < target_duration) {
		return 0;
	}

	// 9. frequency complete: compute H(f) using total actual elapsed time
	double T = data->total_time;
	double A = data->amplitude;
	
	double mag = (T > 0.0) ? 2.0 * sqrt(data->Y_cos*data->Y_cos + data->Y_sin*data->Y_sin) / (A * T) : 0.0;
	double phase = atan2(data->Y_cos, data->Y_sin);

	data->mags[data->freq_idx]   = mag;
	data->phases[data->freq_idx] = phase;
	log_info("f=%7.2f Hz  |H|=%8.4f (%6.1f dB)  ph=%6.1f deg",
		data->freqs[data->freq_idx],
		mag, 20.0*log10(mag + 1e-300),
		phase * 180.0 / M_PI);

	// 10. advance to next frequency step
	data->freq_idx++;
	data->step_count = 0;
	data->phase    = 0.0;
	data->prev_phi = 0.0;
	data->Y_cos    = 0.0;
	data->Y_sin    = 0.0;

	if (data->freq_idx < data->n_freqs) {
		advance_frequency(data);
		return 0;
	}

	// 11. End of sweep: plot results
	int ret = generate_plot(data);
	state->header.status |= AYLP_DONE;
	return ret;
}


int bode_plot_fini(struct aylp_device *self)
{
	struct aylp_bode_plot_data *data = self->device_data;
	if (data->fg_fd >= 0) {
		char cmd[64];
		snprintf(cmd, sizeof cmd, "DAQC2.fgOFF(%d, %d)\n",
			data->fg_board, data->fg_channel);
		fg_send(data->fg_fd, cmd);
		close(data->fg_fd);
	}
	xfree(data->fg_port);
	xfree(data->freqs);
	xfree(data->mags);
	xfree(data->phases);
	xfree(data->output_file);
	xfree(data);
	return 0;
}
