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

#include <string.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>
#include "anyloop.h"
#include "logging.h"
#include "xalloc.h"
#include "ASICamera2.h"


struct asi_source_data {
	int camera_id;
	int width;
	int height;
	long buf_size;
	gsl_matrix_uchar *matrix;
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
	while (ASIGetVideoData(
			data->camera_id, data->matrix->data, data->buf_size, 0
		) == ASI_SUCCESS) {
		got = 1;
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
	ASISetControlValue(data->camera_id, ASI_HIGH_SPEED_MODE,   0,         ASI_FALSE);

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

	self->type_in   = AYLP_T_ANY;
	self->units_in  = AYLP_U_ANY;
	self->type_out  = AYLP_T_MATRIX_UCHAR;
	self->units_out = AYLP_U_COUNTS;
	return 0;
}
