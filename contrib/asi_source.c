// Anyloop source device for ZWO ASI cameras.
// Captures 8-bit mono frames via the ASI SDK and puts them in the pipeline as
// T_MATRIX_UCHAR so center_of_mass and udp_sink can consume them directly.
//
// Plugin URI example:
//   {"uri": "file:/home/fsl/anyloop_levfork/anyloop/build/asi_source.so", "params": {...}}
//
// Parameters:
//   "camera_name" (string): name substring to match (default: "ASI290MM")
//   "width"       (int):    ROI width in pixels, must be multiple of 8 (default: 256)
//   "height"      (int):    ROI height in pixels (default: 256)
//   "start_x"     (int):    ROI left edge; omit to auto-center on sensor
//   "start_y"     (int):    ROI top edge;  omit to auto-center on sensor
//   "exposure"    (int):    exposure time in microseconds (default: 1000)
//   "gain"        (int):    sensor gain, 0–570 for IMX290 (default: 100)
//   "bandwidth"   (int):    USB bandwidth %, 40–100 (default: 80)
//   "high_speed"  (int):    1 = 10-bit ADC fast readout, 0 = 12-bit (default: 0)

#include <string.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>
#include "anyloop.h"
#include "logging.h"
#include "xalloc.h"
#include "ASICamera2.h"

// How many proc() calls per latency-diagnostic report. At ~815 Hz this is a
// report roughly every ~2.5 s.
#define ASI_DIAG_WINDOW 2000


struct asi_source_data {
	int camera_id;
	int width;
	int height;
	long buf_size;
	gsl_matrix_uchar *matrix;

	// Latency diagnostics. The -p profiler only times how long proc() takes
	// to pull an already-buffered frame; it can't see how *old* that frame
	// is. The SDK free-runs the camera and queues frames, so comparing the
	// camera's production rate against the loop rate — and watching how many
	// frames we drain per call — reveals the queue backlog / stale-frame
	// latency that the profiler misses.
	struct timespec diag_t0;   // start of the current reporting window
	long diag_calls;           // proc() calls in this window
	long diag_frames;          // frames the camera produced this window
	int  diag_max_drained;     // worst single-call drain this window
	long diag_blocked;         // calls that found an empty queue and blocked
};


int asi_source_proc(struct aylp_device *self, struct aylp_state *state)
{
	struct asi_source_data *data = self->device_data;
	// The ASI SDK buffers frames continuously after ASIStartVideoCapture, and
	// ASIGetVideoData returns the OLDEST queued frame. If the control loop runs
	// slower than the camera, frames pile up and we end up acting on stale
	// data — a fixed dead-time in the loop that drives oscillation regardless
	// of gain. Drain the queue non-blocking (0 ms timeout) so we keep only the
	// freshest frame, then fall back to a blocking read if nothing was queued.
	int got = 0;
	int drained = 0;   // frames pulled non-blocking this call (queue depth)
	while (ASIGetVideoData(
			data->camera_id, data->matrix->data, data->buf_size, 0
		) == ASI_SUCCESS) {
		got = 1;
		drained++;
	}
	if (!got) {
		// 2000 ms timeout — well above any realistic exposure time
		ASI_ERROR_CODE err = ASIGetVideoData(
			data->camera_id, data->matrix->data, data->buf_size, 2000
		);
		if (UNLIKELY(err != ASI_SUCCESS)) {
			log_error("ASIGetVideoData error %d", err);
			return -1;
		}
	}

	// --- latency diagnostic ------------------------------------------------
	// frames the camera actually produced since the last call: the non-blocking
	// drain count when the queue was non-empty, else the single frame we blocked
	// for. Accumulate per window, then report loop vs camera rate and backlog.
	data->diag_calls++;
	data->diag_frames += got ? drained : 1;
	if (drained > data->diag_max_drained) data->diag_max_drained = drained;
	if (!got) data->diag_blocked++;
	if (data->diag_calls >= ASI_DIAG_WINDOW) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		double dt = (now.tv_sec - data->diag_t0.tv_sec)
			+ (now.tv_nsec - data->diag_t0.tv_nsec) / 1e9;
		if (dt > 0.0) {
			double loop_hz = data->diag_calls / dt;
			double cam_hz  = data->diag_frames / dt;
			double mean_drained = (double)data->diag_frames
				/ data->diag_calls;
			// mean_drained > 1 with cam_hz > loop_hz => the camera
			// outruns the loop and frames sit queued (stale-frame
			// latency); we keep the newest, but the freshest frame
			// still carries ~1/cam_hz + exposure of pipeline latency.
			// blocked% high => loop outruns the camera: no backlog,
			// latency is just one exposure+readout+USB transfer.
			log_info("asi latency: loop %.0f Hz | camera %.0f Hz | "
				"drained/call mean %.2f max %d | blocked %.0f%% | "
				"frame period %.2f ms",
				loop_hz, cam_hz, mean_drained,
				data->diag_max_drained,
				100.0 * data->diag_blocked / data->diag_calls,
				cam_hz > 0 ? 1000.0 / cam_hz : 0.0);
		}
		data->diag_t0 = now;
		data->diag_calls = 0;
		data->diag_frames = 0;
		data->diag_max_drained = 0;
		data->diag_blocked = 0;
	}
	state->matrix_uchar = data->matrix;
	state->header.type = self->type_out;
	state->header.units = self->units_out;
	state->header.log_dim.y = (uint64_t)data->height;
	state->header.log_dim.x = (uint64_t)data->width;
	return 0;
}


int asi_source_fini(struct aylp_device *self)
{
	struct asi_source_data *data = self->device_data;
	ASIStopVideoCapture(data->camera_id);
	ASICloseCamera(data->camera_id);
	xfree_type(gsl_matrix_uchar, data->matrix);
	xfree(data);
	return 0;
}


int asi_source_init(struct aylp_device *self)
{
	self->proc = &asi_source_proc;
	self->fini = &asi_source_fini;
	self->device_data = xcalloc(1, sizeof(struct asi_source_data));
	struct asi_source_data *data = self->device_data;

	// parameter defaults
	const char *camera_name = "ASI290MM";
	int width     = 256;
	int height    = 256;
	int start_x   = -1;   // <0 → auto-center
	int start_y   = -1;
	long exposure = 1000;  // µs
	long gain     = 100;
	long bandwidth = 80;
	long high_speed = 0;  // 1 = 10-bit ADC fast readout

	if (!self->params) {
		log_error("No params object found");
		return -1;
	}
	json_object_object_foreach(self->params, key, val) {
		if (key[0] == '_') {
			// comment key
		} else if (!strcmp(key, "camera_name")) {
			camera_name = json_object_get_string(val);
		} else if (!strcmp(key, "width")) {
			width = json_object_get_int(val);
		} else if (!strcmp(key, "height")) {
			height = json_object_get_int(val);
		} else if (!strcmp(key, "start_x")) {
			start_x = json_object_get_int(val);
		} else if (!strcmp(key, "start_y")) {
			start_y = json_object_get_int(val);
		} else if (!strcmp(key, "exposure")) {
			exposure = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "gain")) {
			gain = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "bandwidth")) {
			bandwidth = (long)json_object_get_int64(val);
		} else if (!strcmp(key, "high_speed")) {
			high_speed = (long)json_object_get_int64(val);
		} else {
			log_warn("Unknown parameter \"%s\"", key);
		}
	}

	if (width % 8 != 0) {
		log_error("width must be a multiple of 8 (got %d)", width);
		return -1;
	}

	// find camera by name substring
	int n_cams = ASIGetNumOfConnectedCameras();
	if (n_cams <= 0) {
		log_error("No ASI cameras found");
		return -1;
	}
	log_info("Found %d ASI camera(s)", n_cams);

	data->camera_id = -1;
	long sensor_w = 0, sensor_h = 0;
	for (int i = 0; i < n_cams; i++) {
		ASI_CAMERA_INFO info;
		ASIGetCameraProperty(&info, i);
		log_info("  [%d] %s (ID %d, %ldx%ld)",
			i, info.Name, info.CameraID, info.MaxWidth, info.MaxHeight
		);
		if (data->camera_id < 0 && strstr(info.Name, camera_name)) {
			data->camera_id = info.CameraID;
			sensor_w = info.MaxWidth;
			sensor_h = info.MaxHeight;
		}
	}
	if (data->camera_id < 0) {
		log_error("No camera matching \"%s\" found", camera_name);
		return -1;
	}

	// auto-center ROI if start position not given
	if (start_x < 0)
		start_x = (int)((sensor_w - width)  / 2) & ~7;  // align to 8px
	if (start_y < 0)
		start_y = (int)((sensor_h - height) / 2) & ~1;  // align to 2px

	ASI_ERROR_CODE err;
	err = ASIOpenCamera(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIOpenCamera failed (%d)", err);
		return -1;
	}
	err = ASIInitCamera(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIInitCamera failed (%d)", err);
		return -1;
	}

	ASISetControlValue(data->camera_id, ASI_BANDWIDTHOVERLOAD, bandwidth, ASI_FALSE);
	ASISetControlValue(data->camera_id, ASI_GAIN,              gain,      ASI_FALSE);
	ASISetControlValue(data->camera_id, ASI_EXPOSURE,          exposure,  ASI_FALSE);
	ASISetControlValue(data->camera_id, ASI_HIGH_SPEED_MODE,   high_speed, ASI_FALSE);

	err = ASISetROIFormat(data->camera_id, width, height, 1, ASI_IMG_RAW8);
	if (err != ASI_SUCCESS) {
		log_error("ASISetROIFormat failed (%d) for %dx%d RAW8", err, width, height);
		return -1;
	}
	err = ASISetStartPos(data->camera_id, start_x, start_y);
	if (err != ASI_SUCCESS) {
		log_error("ASISetStartPos failed (%d) for start=(%d,%d)",
			err, start_x, start_y
		);
		return -1;
	}

	log_info("Camera %d ready: %dx%d RAW8 @ (%d,%d), exp=%ldµs gain=%ld bw=%ld%%",
		data->camera_id, width, height, start_x, start_y, exposure, gain, bandwidth
	);

	err = ASIStartVideoCapture(data->camera_id);
	if (err != ASI_SUCCESS) {
		log_error("ASIStartVideoCapture failed (%d)", err);
		return -1;
	}

	// allocate the frame buffer (gsl_matrix_uchar is contiguous when tda==size2)
	data->width    = width;
	data->height   = height;
	data->buf_size = (long)height * width;
	data->matrix   = xmalloc_type(gsl_matrix_uchar, (size_t)height, (size_t)width);

	// start the first latency-diagnostic window
	clock_gettime(CLOCK_MONOTONIC, &data->diag_t0);

	self->type_in   = AYLP_T_ANY;
	self->units_in  = AYLP_U_ANY;
	self->type_out  = AYLP_T_MATRIX_UCHAR;
	self->units_out = AYLP_U_COUNTS;
	return 0;
}
