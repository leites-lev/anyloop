#include <time.h>
#include <math.h>

#include "anyloop.h"
#include "logging.h"
#include "xalloc.h"
#include "pid.h"


int pid_init(struct aylp_device *self)
{
	int err;
	self->proc = &pid_proc;
	self->fini = &pid_fini;
	self->device_data = xcalloc(1, sizeof(struct aylp_pid_data));
	struct aylp_pid_data *data = self->device_data;

	// set defaults
	data->p = 1.0;
	data->i = 0.0;
	data->d = 0.0;
	data->clamp = 1.0;
	data->g = 1.0;
	data->p_y = data->i_y = data->d_y = data->g_y = -1.0;
	data->dfilt = 0.0;	// derivative filter off by default
	data->dfilt_y = -1.0;	// <0 means "use base dfilt"
	data->deadband = 0.0;	// off by default
	data->deadband_y = -1.0;	// <0 means "use base deadband"
	// lead compensator off by default (0 freqs disable it), unity gain
	data->lead_fz = data->lead_fp = 0.0;
	data->lead_g = 1.0;
	data->lead_fz_y = data->lead_fp_y = 0.0;
	data->lead_g_y = 1.0;

	// parse parameters
	if (!self->params) {
		log_error("No params object found.");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
			// keys starting with _ are comments
		} else if (!strcmp(key, "type")) {
			const char *s = json_object_get_string(val);
			if (!strcmp(s, "vector"))
				data->type = AYLP_T_VECTOR;
			else if (!strcmp(s, "matrix"))
				data->type = AYLP_T_MATRIX;
			else log_error("Unrecognized type: %s", s);
			log_trace("type = %s (0x%hhX)", s, data->type);
		} else if (!strcmp(key, "units")) {
			const char *s = json_object_get_string(val);
			data->units = aylp_units_from_string(s);
			log_trace("units = %s (0x%hhX)", s, data->units);
		} else if (!strcmp(key, "p")) {
			data->p = json_object_get_double(val);
			log_trace("p = %G", data->p);
		} else if (!strcmp(key, "i")) {
			data->i = json_object_get_double(val);
			log_trace("i = %G", data->i);
		} else if (!strcmp(key, "d")) {
			data->d = json_object_get_double(val);
			log_trace("d = %G", data->d);
		} else if (!strcmp(key, "g")) {
			data->g = json_object_get_double(val);
			log_trace("g = %G", data->g);
		} else if (!strcmp(key, "py")) { data->p_y = json_object_get_double(val);
		} else if (!strcmp(key, "iy")) { data->i_y = json_object_get_double(val);
		} else if (!strcmp(key, "dy")) { data->d_y = json_object_get_double(val);
		} else if (!strcmp(key, "gy")) { data->g_y = json_object_get_double(val);
		} else if (!strcmp(key, "dfilt")) {
			data->dfilt = json_object_get_double(val);
			log_trace("dfilt = %G Hz", data->dfilt);
		} else if (!strcmp(key, "dfilty")) {
			data->dfilt_y = json_object_get_double(val);
			log_trace("dfilty = %G Hz", data->dfilt_y);
		} else if (!strcmp(key, "deadband")) {
			data->deadband = json_object_get_double(val);
			if (data->deadband < 0) data->deadband = -data->deadband;
			log_trace("deadband = %G", data->deadband);
		} else if (!strcmp(key, "deadbandy")) {
			data->deadband_y = json_object_get_double(val);
			log_trace("deadbandy = %G", data->deadband_y);
		} else if (!strcmp(key, "clamp")) {
			data->clamp = json_object_get_double(val);
			if (data->clamp < 0)
				data->clamp = -data->clamp;
			log_trace("clamp = ±%G", data->clamp);
		} else if (!strcmp(key, "lead_fz")) {
			data->lead_fz = json_object_get_double(val);
			log_trace("lead_fz = %G", data->lead_fz);
		} else if (!strcmp(key, "lead_fp")) {
			data->lead_fp = json_object_get_double(val);
			log_trace("lead_fp = %G", data->lead_fp);
		} else if (!strcmp(key, "lead_g")) {
			data->lead_g = json_object_get_double(val);
			log_trace("lead_g = %G", data->lead_g);
		} else if (!strcmp(key, "lead_fzy")) {
			data->lead_fz_y = json_object_get_double(val);
			log_trace("lead_fzy = %G", data->lead_fz_y);
		} else if (!strcmp(key, "lead_fpy")) {
			data->lead_fp_y = json_object_get_double(val);
			log_trace("lead_fpy = %G", data->lead_fp_y);
		} else if (!strcmp(key, "lead_gy")) {
			data->lead_g_y = json_object_get_double(val);
			log_trace("lead_gy = %G", data->lead_g_y);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}
	if (data->p_y < 0) data->p_y = data->p;
	if (data->i_y < 0) data->i_y = data->i;
	if (data->d_y < 0) data->d_y = data->d;
	if (data->g_y < 0) data->g_y = data->g;
	if (data->dfilt_y < 0) data->dfilt_y = data->dfilt;
	if (data->deadband_y < 0) data->deadband_y = data->deadband;
	// make sure we didn't miss any params
	if (!data->type) {
		log_error("You must provide valid type param.");
		return -1;
	}
	// the pole enables the lead; a zero without a pole can't be realized
	if (data->lead_fz > 0 && data->lead_fp <= 0)
		log_warn("x lead has a zero but no pole (lead_fp); lead disabled.");
	if (data->lead_fz_y > 0 && data->lead_fp_y <= 0)
		log_warn("y lead has a zero but no pole (lead_fpy); lead disabled.");
	int lead_on = data->lead_fp > 0 || data->lead_fp_y > 0;
	if (lead_on && data->type != AYLP_T_VECTOR)
		log_warn("Lead compensator only applies for type=vector; ignoring.");

	// allocate dummy vectors or matrices so we can skip checking if they
	// exist in proc()
	switch (data->type) {
	case AYLP_T_VECTOR:
		data->acc_v = xmalloc_type(gsl_vector, 0);
		data->pre_v = xmalloc_type(gsl_vector, 0);
		data->res_v = xmalloc_type(gsl_vector, 0);
		data->dfd_v = xmalloc_type(gsl_vector, 0);
		data->lead_in_v = xmalloc_type(gsl_vector, 0);
		data->lead_out_v = xmalloc_type(gsl_vector, 0);
		break;
	case AYLP_T_MATRIX:
		data->acc_m = xmalloc_type(gsl_matrix, 0, 0);
		data->pre_m = xmalloc_type(gsl_matrix, 0, 0);
		data->res_m = xmalloc_type(gsl_matrix, 0, 0);
		data->dfd_m = xmalloc_type(gsl_matrix, 0, 0);
		break;
	}

	err = clock_gettime(CLOCK_MONOTONIC, &data->tp);
	if (err) {
		log_error("Couldn't get time: %s", strerror(err));
		return -1;
	}

	// set types and units
	self->type_in = data->type;
	self->units_in = AYLP_U_ANY;
	self->type_out = AYLP_T_UNCHANGED;
	self->units_out = data->units;
	return 0;
}


int pid_proc(struct aylp_device *self, struct aylp_state *state)
{
	int err;
	struct aylp_pid_data *data = self->device_data;
	struct timespec tp1;
	err = clock_gettime(CLOCK_MONOTONIC, &tp1);
	if (err) {
		log_error("Couldn't get time: %s", strerror(err));
		return -1;
	}
	double dt = tp1.tv_sec - data->tp.tv_sec;
	dt += 1E-9 * (tp1.tv_nsec - data->tp.tv_nsec);
	data->tp = tp1;
	log_trace("dt = %G s", dt);

	switch (data->type) {
	case AYLP_T_VECTOR: {
		gsl_vector *a = data->acc_v;
		gsl_vector *p = data->pre_v;
		gsl_vector *r = data->res_v;
		gsl_vector *s = state->vector;
		// check if we need to (re)initialize
		if (a->size != s->size) {
			xfree_type(gsl_vector, data->acc_v);
			data->acc_v = xcalloc_type(gsl_vector, s->size);
		}
		if (p->size != s->size) {
			xfree_type(gsl_vector, data->pre_v);
			data->pre_v = xcalloc_type(gsl_vector, s->size);
		}
		if (r->size != s->size) {
			xfree_type(gsl_vector, data->res_v);
			data->res_v = xmalloc_type(gsl_vector, s->size);
		}
		// derivative-filter state, zeroed so it starts at rest
		if (data->dfd_v->size != s->size) {
			xfree_type(gsl_vector, data->dfd_v);
			data->dfd_v = xcalloc_type(gsl_vector, s->size);
		}
		// lead compensator state, zeroed so the filter starts at rest
		if (data->lead_in_v->size != s->size) {
			xfree_type(gsl_vector, data->lead_in_v);
			data->lead_in_v = xcalloc_type(gsl_vector, s->size);
		}
		if (data->lead_out_v->size != s->size) {
			xfree_type(gsl_vector, data->lead_out_v);
			data->lead_out_v = xcalloc_type(gsl_vector, s->size);
		}
		// loop over elements and apply PID control
		for (size_t j = 0; j < s->size; j++) {
			double pj = j ? data->p : data->p_y;
			double ij = j ? data->i : data->i_y;
			double dj = j ? data->d : data->d_y;
			double gj = j ? data->g : data->g_y;
			double dfj = j ? data->dfilt : data->dfilt_y;
			double dbj = j ? data->deadband : data->deadband_y;
			// deadband: ignore sub-threshold error so the loop stops
			// chattering on quantization/relay nonlinearities at the null
			double e = s->data[j*s->stride];
			if (e < dbj && e > -dbj) e = 0.0;
			// update accumulator and clamp if needed
			a->data[j*a->stride] = gj * a->data[j*a->stride] + dt * e;
			if (a->data[j*a->stride] > data->clamp)
				a->data[j*a->stride] = data->clamp;
			else if (a->data[j*a->stride] < -data->clamp)
				a->data[j*a->stride] = -data->clamp;
			// derivative term, optionally low-passed at dfj Hz to limit
			// noise gain: D = s/(1+s/wf) via backward Euler. Reduces to the
			// raw difference (e-e_prev)/dt as the cutoff -> infinity.
			double de = (e - p->data[j*p->stride]) / dt;
			if (dfj > 0.0) {
				double wf = 2.0*M_PI*dfj;
				double dprev = data->dfd_v->data[j*data->dfd_v->stride];
				de = (dprev + wf*(e - p->data[j*p->stride]))
					/ (1.0 + wf*dt);
			}
			data->dfd_v->data[j*data->dfd_v->stride] = de;
			// raw PID output
			double raw =
				- pj * e
				- ij * a->data[j*a->stride]
				- dj * de
			;
			// per-axis series lead compensator on the PID output:
			//   C(s) = g*(s + 2pi*fz) / (s + 2pi*fp)
			// bilinear-transformed with the live dt so it tracks the
			// variable loop rate. The pole (fp>0) enables the section;
			// the zero may sit at 0 Hz (fz==0) for a band-limited
			// differentiator s/(s+wp). DC gain = g*fz/fp, HF gain = g.
			double fzj = j ? data->lead_fz : data->lead_fz_y;
			double fpj = j ? data->lead_fp : data->lead_fp_y;
			double glj = j ? data->lead_g  : data->lead_g_y;
			double out = raw;
			if (fpj > 0.0) {
				double k = 2.0 / dt;            // bilinear 2/T
				double wz = 2.0*M_PI*fzj;
				double wp = 2.0*M_PI*fpj;
				double a0 = k + wp;             // normalizing term
				double b0 = glj * (k + wz) / a0;
				double b1 = glj * (wz - k) / a0;
				double a1 = (wp - k) / a0;
				double li = data->lead_in_v->data[j*data->lead_in_v->stride];
				double lo = data->lead_out_v->data[j*data->lead_out_v->stride];
				out = b0*raw + b1*li - a1*lo;
				data->lead_in_v->data[j*data->lead_in_v->stride] = raw;
				data->lead_out_v->data[j*data->lead_out_v->stride] = out;
			}
			r->data[j*r->stride] = out;
			// clamp result if needed
			if (r->data[j*r->stride] > data->clamp)
				r->data[j*r->stride] = data->clamp;
			else if (r->data[j*r->stride] < -data->clamp)
				r->data[j*r->stride] = -data->clamp;
			// update previous
			p->data[j*p->stride] = e;
		}
		state->vector = data->res_v;
		break;
	}

	case AYLP_T_MATRIX: {
		gsl_matrix *a = data->acc_m;
		gsl_matrix *p = data->pre_m;
		gsl_matrix *r = data->res_m;
		gsl_matrix *s = state->matrix;
		// check if we need to (re)initialize
		if (UNLIKELY(a->size1 != s->size1 || a->size2 != s->size2)) {
			xfree_type(gsl_matrix, data->acc_m);
			data->acc_m = xcalloc_type(gsl_matrix,
				s->size1, s->size2
			);
		}
		if (UNLIKELY(p->size1 != s->size1 || p->size2 != s->size2)) {
			xfree_type(gsl_matrix, data->pre_m);
			data->pre_m = xcalloc_type(gsl_matrix,
				s->size1, s->size2
			);
		}
		if (UNLIKELY(r->size1 != s->size1 || r->size2 != s->size2)) {
			xfree_type(gsl_matrix, data->res_m);
			data->res_m = xmalloc_type(gsl_matrix,
				s->size1, s->size2
			);
		}
		if (UNLIKELY(data->dfd_m->size1 != s->size1
				|| data->dfd_m->size2 != s->size2)) {
			xfree_type(gsl_matrix, data->dfd_m);
			data->dfd_m = xcalloc_type(gsl_matrix,
				s->size1, s->size2
			);
		}
		// loop over elements and apply PID control
		for (size_t y = 0; y < s->size1; y++) {
			for (size_t x = 0; x < s->size2; x++) {
				// deadband: ignore sub-threshold error (see vector case)
				double e = s->data[y*s->tda+x];
				if (e < data->deadband && e > -data->deadband) e = 0.0;
				// update accumulator and clamp if needed
				a->data[y*a->tda+x] += dt * e;
				if (a->data[y*a->tda+x] > data->clamp)
					a->data[y*a->tda+x] = data->clamp;
				else if (a->data[y*a->tda+x] < -data->clamp)
					a->data[y*a->tda+x] = -data->clamp;
				// derivative term, optionally low-passed (see vector case)
				double de = (e - p->data[y*p->tda+x]) / dt;
				if (data->dfilt > 0.0) {
					double wf = 2.0*M_PI*data->dfilt;
					double dprev = data->dfd_m->data[y*data->dfd_m->tda+x];
					de = (dprev + wf*(e - p->data[y*p->tda+x]))
						/ (1.0 + wf*dt);
				}
				data->dfd_m->data[y*data->dfd_m->tda+x] = de;
				// apply pid params to result
				r->data[y*r->tda+x] =
					- data->p * e
					- data->i * a->data[y*a->tda+x]
					- data->d * de
				;
				// clamp result if needed
				if (r->data[y*r->tda+x] > data->clamp)
					r->data[y*r->tda+x] = data->clamp;
				if (r->data[y*r->tda+x] < -data->clamp)
					r->data[y*r->tda+x] = -data->clamp;
				// update previous
				p->data[y*p->tda+x] = e;
			}
		}
		state->matrix = data->res_m;
		break;
	}
	}

	return 0;
}


int pid_fini(struct aylp_device *self)
{
	struct aylp_pid_data *data = self->device_data;
	switch (data->type) {
	case AYLP_T_VECTOR:
		xfree_type(gsl_vector, data->acc_v);
		xfree_type(gsl_vector, data->pre_v);
		xfree_type(gsl_vector, data->res_v);
		xfree_type(gsl_vector, data->dfd_v);
		xfree_type(gsl_vector, data->lead_in_v);
		xfree_type(gsl_vector, data->lead_out_v);
		break;
	case AYLP_T_MATRIX:
		xfree_type(gsl_matrix, data->acc_m);
		xfree_type(gsl_matrix, data->pre_m);
		xfree_type(gsl_matrix, data->res_m);
		xfree_type(gsl_matrix, data->dfd_m);
		break;
	}
	xfree(data);
	return 0;
}


