#ifndef AYLP_DEVICES_CENTER_OF_MASS_H_
#define AYLP_DEVICES_CENTER_OF_MASS_H_

#include "anyloop.h"
#include "thread_pool.h"

// src bundle passed to com_mat_uchar via the task runner
struct aylp_com_src {
	gsl_matrix_uchar mat;
	unsigned char threshold;
};

struct aylp_center_of_mass_data {
	// param: height of regions/subapertures
	size_t region_height;
	// param: width of regions/subapertures
	size_t region_width;
	// param; set to 1 for no multithreading
	size_t thread_count;
	// param: subtract this value from each pixel before computing CoM
	unsigned char threshold;
	// array of threads
	pthread_t *threads;
	// array of tasks
	struct aylp_task *tasks;
	// number of tasks
	size_t n_tasks;
	// array of com_src inputs for the tasks (matrix view + threshold)
	struct aylp_com_src *com_srcs;
	// queue of tasks
	struct aylp_queue queue;

	// center of mass result (contiguous vector)
	gsl_vector *com;
};

// initialize center_of_mass device
int center_of_mass_init(struct aylp_device *self);

// process center_of_mass device once per loop
int center_of_mass_proc(struct aylp_device *self, struct aylp_state *state);
// multithreaded version of proc function
int center_of_mass_proc_threaded(
	struct aylp_device *self, struct aylp_state *state
);

// close center_of_mass device when loop exits
int center_of_mass_fini(struct aylp_device *self);
int center_of_mass_fini_threaded(struct aylp_device *self);

#endif

