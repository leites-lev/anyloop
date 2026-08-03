/** Unit tests for center_of_mass's partial-beam gate.
*
* The case these exist for is a rolling shutter reading out across a chopped
* source: some rows catch the source's on-window and some do not, so a frame can
* hold a full-brightness fragment of the beam. `min_peak` cannot see that -- the
* lit rows are at normal brightness -- and the centroid of the fragment is
* pulled toward those rows, which is a systematic error along the shutter axis
* rather than noise that averages away.
*
* `ref_cut` learns what a whole beam's row profile looks like and divides each
* frame by it. A whole beam gives the same ratio on every row, so the frame's
* brightness cancels; a cut beam gives two groups of rows, lit and not. The
* tests below pin both halves of that: cuts are caught (including on a beam too
* tight for any shape-based test), and frames that are merely dim are not.
*
* Run: ninja -C build test  (or build/test_center_of_mass) */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "anyloop.h"
#include "logging.h"
#include "center_of_mass.h"

static unsigned n_fail = 0;
static const char *current_test = "";

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		n_fail++; \
		fprintf(stderr, "FAIL %s:%d [%s]: ", \
			__FILE__, __LINE__, current_test); \
		fprintf(stderr, __VA_ARGS__); \
		fprintf(stderr, "\n"); \
	} \
} while (0)

#define CHECK_NEAR(got, want, tol) \
	CHECK(fabs((got) - (want)) <= (tol), \
		#got " = %.6f, want %.6f (tol %g)", (double)(got), \
		(double)(want), (double)(tol))


// ---------------------------------------------------------------- fixtures

#define IMG 32

/** A Gaussian beam centred at (cy, cx), peaking at `peak` counts over a
* background of 2, with every row outside [cut_lo, cut_hi] forced to background
* -- which is what a rolling shutter does to a chopped source. */
static gsl_matrix_uchar *make_beam(
	double cy, double cx, double sigma, unsigned char peak,
	double cut_lo, double cut_hi
) {
	const unsigned char bg = 2;
	gsl_matrix_uchar *m = gsl_matrix_uchar_alloc(IMG, IMG);
	gsl_matrix_uchar_set_all(m, bg);
	for (size_t i = 0; i < IMG; i++) {
		if ((double)i < cut_lo || (double)i > cut_hi)
			continue;
		for (size_t j = 0; j < IMG; j++) {
			double dy = (double)i - cy, dx = (double)j - cx;
			double v = peak * exp(-(dy*dy + dx*dx)
				/ (2.0*sigma*sigma));
			double t = v + bg;
			m->data[i*m->tda + j] = t > 255.0 ? 255
				: (unsigned char)(t + 0.5);
		}
	}
	return m;
}

/** The whole beam, uncut, at the reference brightness. */
static gsl_matrix_uchar *full_beam(double cy, double cx)
{
	return make_beam(cy, cx, 2.0, 200, -1e9, 1e9);
}

static int make_com(struct aylp_device *dev, const char *params_json)
{
	memset(dev, 0, sizeof(*dev));
	dev->uri = "anyloop:center_of_mass";
	dev->params = json_tokener_parse(params_json);
	if (!dev->params) {
		fprintf(stderr, "test bug: bad json: %s\n", params_json);
		exit(2);
	}
	return center_of_mass_init(dev);
}

static void free_com(struct aylp_device *dev)
{
	if (dev->fini)
		dev->fini(dev);
	json_object_put(dev->params);
}

/** Push one frame through the device and return the emitted [y, x]. */
static void run_frame(
	struct aylp_device *dev, gsl_matrix_uchar *img, double *out_y,
	double *out_x
) {
	struct aylp_state state = {0};
	state.header.type = AYLP_T_MATRIX_UCHAR;
	state.matrix_uchar = img;
	int err = dev->proc(dev, &state);
	CHECK(!err, "proc returned %d", err);
	*out_y = state.vector->data[0];
	*out_x = state.vector->data[1];
}

/** Run the warmup so the reference is learned and the gate goes live. */
static void warm_up(struct aylp_device *dev, gsl_matrix_uchar *img, size_t n)
{
	double oy, ox;
	for (size_t k = 0; k < n; k++)
		run_frame(dev, img, &oy, &ox);
	struct aylp_center_of_mass_data *data = dev->device_data;
	CHECK(data->ref_ready, "reference not ready after %zu warmup frames", n);
}

/** Image-normalized coordinate of image row/column `p`. */
static double norm(double p)
{
	return -1.0 + 2.0*p/(IMG - 1);
}

#define WARMUP 5
// ref_cut 0.5 with the defaults. Measured on this fixture: a whole beam reads
// 1.00, a beam dimmed 4x still reads 0.61, and any cut through the significant
// rows reads 0.00 -- see test_dim_beam_is_not_a_cut
#define BASE "\"region_height\":12,\"region_width\":12,\"threshold\":10," \
	"\"track\":true,\"init_y\":16,\"init_x\":16," \
	"\"ref_warmup\":5,\"ref_cut\":0.5,"


// ------------------------------------------------------------------- tests

/** An uncut beam passes, and the warmup itself does not reject anything. */
static void test_full_beam_passes(void)
{
	current_test = "full_beam_passes";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "}"), "init failed");
	gsl_matrix_uchar *img = full_beam(16.0, 16.0);
	warm_up(&dev, img, WARMUP);
	double oy, ox;
	run_frame(&dev, img, &oy, &ox);
	CHECK_NEAR(oy, norm(16.0), 5e-3);
	CHECK_NEAR(ox, norm(16.0), 5e-3);
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 0, "uncut beam was rejected");
	gsl_matrix_uchar_free(img);
	free_com(&dev);
}

/** A beam that has moved still passes: the gate must track a real excursion,
* not freeze the loop the moment the beam is not where it was. */
static void test_moving_beam_passes(void)
{
	current_test = "moving_beam_passes";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "}"), "init failed");
	gsl_matrix_uchar *img = full_beam(16.0, 16.0);
	warm_up(&dev, img, WARMUP);
	gsl_matrix_uchar_free(img);
	double oy, ox;
	for (double cy = 16.0; cy <= 19.0; cy += 0.5) {
		gsl_matrix_uchar *m = full_beam(cy, 16.0);
		run_frame(&dev, m, &oy, &ox);
		CHECK_NEAR(oy, norm(cy), 5e-3);
		gsl_matrix_uchar_free(m);
	}
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 0, "a moving beam was rejected %zu times",
		data->n_held);
	free_com(&dev);
}

/** The defect: the cut frame passes every brightness test there is. Without the
* profile gate it is accepted and drags the centroid a pixel off a beam that
* never moved. */
static void test_brightness_tests_cannot_see_a_cut(void)
{
	current_test = "brightness_tests_cannot_see_a_cut";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{\"region_height\":12,\"region_width\":12,"
		"\"threshold\":10,\"track\":true,\"init_y\":16,\"init_x\":16,"
		"\"min_peak\":100}"), "init failed");
	double oy, ox;
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	run_frame(&dev, full, &oy, &ox);
	CHECK_NEAR(oy, norm(16.0), 5e-3);

	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200, -1e9, 16.0);
	run_frame(&dev, cut, &oy, &ox);
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 0,
		"min_peak was expected to let the cut frame through");
	CHECK(oy < norm(16.0) - 2.0/(IMG - 1)*0.7,
		"cut frame biased the centroid only to %.6f, from %.6f", oy,
		norm(16.0));

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** ref_cut rejects that frame and holds the last good centroid. */
static void test_cut_is_rejected_and_held(void)
{
	current_test = "cut_is_rejected_and_held";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "\"min_peak\":100}"), "init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	warm_up(&dev, full, WARMUP);
	double oy, ox;
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy, good_x = ox;

	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200, -1e9, 16.0);
	run_frame(&dev, cut, &oy, &ox);
	// held exactly, not merely close: nothing downstream should be able to
	// tell a held frame from a repeated measurement
	CHECK(oy == good_y && ox == good_x,
		"cut frame moved the output to (%.6f,%.6f), want held "
		"(%.6f,%.6f)", oy, ox, good_y, good_x);

	// the window did not chase the fragment either
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->win_y == 16, "window centre drifted to row %zu on a "
		"rejected frame", data->win_y);

	run_frame(&dev, full, &oy, &ox);
	CHECK_NEAR(oy, good_y, 1e-12);
	CHECK(data->n_held == 1, "n_held = %zu, want 1", data->n_held);
	CHECK(data->n_episodes == 1, "n_episodes = %zu, want 1",
		data->n_episodes);

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** Cuts from either side, and a cut leaving a thin slice, are all rejected. The
* shutter phase decides which, and the bias flips sign with it, so a gate that
* caught only one would leave a residual that still beat against the chop. */
static void test_cut_from_either_side(void)
{
	current_test = "cut_from_either_side";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "\"reacquire_after\":1000}"),
		"init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	warm_up(&dev, full, WARMUP);
	double oy, ox;
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy, good_x = ox;

	struct { const char *name; double lo, hi; } cuts[] = {
		{"bottom blanked", -1e9, 16.0},
		{"top blanked",    16.0,  1e9},
		{"thin slice",     15.0, 17.0},
		{"off-centre cut", -1e9, 17.0},
		{"one row short",   14.0, 1e9},
	};
	for (size_t k = 0; k < sizeof cuts / sizeof *cuts; k++) {
		gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200,
			cuts[k].lo, cuts[k].hi);
		run_frame(&dev, cut, &oy, &ox);
		CHECK(oy == good_y && ox == good_x,
			"%s leaked into the output: (%.6f,%.6f)",
			cuts[k].name, oy, ox);
		gsl_matrix_uchar_free(cut);
		run_frame(&dev, full, &oy, &ox);
	}
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 5, "n_held = %zu, want 5", data->n_held);

	gsl_matrix_uchar_free(full);
	free_com(&dev);
}

/** The property the whole design is for: a beam that is merely DIM is not a cut
* beam. Dividing by the reference makes the frame's own brightness cancel, so
* the source can drift a long way without the gate firing.
*
* Measured on this fixture, dimmest significant row as a fraction of the typical
* one: 1.00 at full brightness, 0.98 at 0.9x, 0.93 at 0.6x, 0.81 at 0.4x, 0.61
* at 0.25x. Every cut reads 0.00. The residual slope with brightness is the
* `threshold` subtraction, which is not linear in pixel value and so does not
* cancel perfectly -- it is the one part of this that is not exactly
* scale-free. */
static void test_dim_beam_is_not_a_cut(void)
{
	current_test = "dim_beam_is_not_a_cut";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "}"), "init failed");
	gsl_matrix_uchar *bright = full_beam(16.0, 16.0);
	warm_up(&dev, bright, WARMUP);
	double oy, ox;

	// a quarter of the brightness the reference was learned at
	unsigned char levels[] = {180, 160, 120, 80, 50};
	for (size_t k = 0; k < sizeof levels / sizeof *levels; k++) {
		gsl_matrix_uchar *dim = make_beam(16.0, 16.0, 2.0, levels[k],
			-1e9, 1e9);
		run_frame(&dev, dim, &oy, &ox);
		CHECK_NEAR(oy, norm(16.0), 1.5e-2);
		gsl_matrix_uchar_free(dim);
	}
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 0,
		"a whole but dimmed beam was rejected %zu times", data->n_held);

	gsl_matrix_uchar_free(bright);
	free_com(&dev);
}

/** A beam too tight for any shape-based test still works, because the reference
* is the beam's own profile and carries no assumption about how wide it is. A
* sigma-1 beam's own row-to-row change is 0.61 of its peak row, which no
* steepness threshold could tell from a cut. */
static void test_tight_beam(void)
{
	current_test = "tight_beam";
	struct aylp_device dev;
	// a tight beam has fewer rows above the floor, so lower it
	CHECK(!make_com(&dev, "{\"region_height\":12,\"region_width\":12,"
		"\"threshold\":10,\"track\":true,\"init_y\":16,\"init_x\":16,"
		"\"ref_warmup\":5,\"ref_cut\":0.5,\"ref_floor\":0.1}"),
		"init failed");
	gsl_matrix_uchar *full = make_beam(16.0, 16.0, 1.0, 200, -1e9, 1e9);
	warm_up(&dev, full, WARMUP);
	double oy, ox;
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy, good_x = ox;
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->n_held == 0, "a whole tight beam was rejected");

	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 1.0, 200, -1e9, 16.0);
	run_frame(&dev, cut, &oy, &ox);
	CHECK(oy == good_y && ox == good_x,
		"a cut tight beam leaked into the output: (%.6f,%.6f)", oy, ox);
	CHECK(data->n_held == 1, "n_held = %zu, want 1", data->n_held);

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** The bootstrap takes the row-wise maximum, so it learns the uncut profile
* even when the source is already chopping while it runs. A mean would learn a
* blend, set the reference low, and then wave the cut frames through. */
static void test_bootstrap_survives_chopping(void)
{
	current_test = "bootstrap_survives_chopping";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{\"region_height\":12,\"region_width\":12,"
		"\"threshold\":10,\"track\":true,\"init_y\":16,\"init_x\":16,"
		"\"ref_warmup\":8,\"ref_cut\":0.5,\"reacquire_after\":1000}"),
		"init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200, -1e9, 16.0);
	double oy, ox;
	// warm up on an alternating chopped stream, which is what a cold start
	// into a live chop actually looks like
	for (int k = 0; k < 8; k++)
		run_frame(&dev, k % 2 ? cut : full, &oy, &ox);
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->ref_ready, "reference not ready after the warmup");

	// the reference must describe the WHOLE beam, so cuts are now caught
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy;
	size_t held = data->n_held;
	run_frame(&dev, cut, &oy, &ox);
	CHECK(data->n_held == held + 1,
		"a cut frame was accepted after a chopped warmup");
	CHECK(oy == good_y, "cut frame leaked into the output: %.6f", oy);

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** Until the reference exists there is nothing to compare against, so the gate
* stays off and frames go through on the brightness tests alone. Documented
* rather than incidental: at loop rate the warmup is milliseconds. */
static void test_gate_off_during_warmup(void)
{
	current_test = "gate_off_during_warmup";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "}"), "init failed");
	double oy, ox;
	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200, -1e9, 16.0);
	run_frame(&dev, cut, &oy, &ox);
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(!data->ref_ready, "reference ready after one frame");
	CHECK(data->n_held == 0, "gate fired before the reference existed");
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** ref_cut 0 (and absent) is exactly the old behaviour, and allocates nothing.
*/
static void test_gate_off_is_old_behaviour(void)
{
	current_test = "gate_off_is_old_behaviour";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{\"region_height\":12,\"region_width\":12,"
		"\"threshold\":10,\"track\":true,\"init_y\":16,\"init_x\":16}"),
		"init failed");
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(!data->ref, "reference allocated with the gate off");
	double oy, ox;
	gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, 200, -1e9, 16.0);
	for (int k = 0; k < 10; k++)
		run_frame(&dev, cut, &oy, &ox);
	CHECK(data->n_held == 0, "cut frames rejected with the gate off");
	gsl_matrix_uchar_free(cut);
	free_com(&dev);
}

/** Losing the beam long enough to re-acquire throws the reference away: it
* describes a beam at the old window position, which is what we just stopped
* believing in. */
static void test_reference_reset_on_reacquire(void)
{
	current_test = "reference_reset_on_reacquire";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "\"reacquire_after\":3}"),
		"init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	warm_up(&dev, full, WARMUP);
	struct aylp_center_of_mass_data *data = dev.device_data;

	double oy, ox;
	// everything under threshold: no beam at all, for long enough to
	// trigger the re-acquire
	gsl_matrix_uchar *dark = make_beam(16.0, 16.0, 2.0, 0, -1e9, 1e9);
	for (int k = 0; k < 4; k++)
		run_frame(&dev, dark, &oy, &ox);
	CHECK(!data->ref_ready, "reference survived a re-acquire");
	CHECK(data->ref_seen == 0, "ref_seen = %zu after reset", data->ref_seen);

	// and it relearns. Not in WARMUP frames flat, though: re-acquiring off a
	// dark frame parks the window on the brightest speck of nothing, so the
	// window has to find the beam again (another reacquire_after frames)
	// before there is a profile to learn at all
	for (int k = 0; k < 20; k++)
		run_frame(&dev, full, &oy, &ox);
	CHECK(data->ref_ready, "reference did not relearn after re-acquiring");
	CHECK_NEAR(oy, norm(16.0), 5e-3);

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(dark);
	free_com(&dev);
}

/** Nothing above threshold is still "no beam", and still holds. */
static void test_dark_frame_holds(void)
{
	current_test = "dark_frame_holds";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "\"reacquire_after\":1000}"),
		"init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	warm_up(&dev, full, WARMUP);
	double oy, ox;
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy, good_x = ox;

	gsl_matrix_uchar *dark = make_beam(16.0, 16.0, 2.0, 0, -1e9, 1e9);
	run_frame(&dev, dark, &oy, &ox);
	CHECK(oy == good_y && ox == good_x, "dark frame did not hold");

	gsl_matrix_uchar_free(full);
	gsl_matrix_uchar_free(dark);
	free_com(&dev);
}

/** Config errors are caught at init rather than at 3 kHz. */
static void test_init_validation(void)
{
	struct aylp_device dev;

	current_test = "ref_cut_out_of_range";
	CHECK(make_com(&dev, "{" BASE "\"ref_cut\":1.5}"),
		"init accepted ref_cut > 1");
	free_com(&dev);
	CHECK(make_com(&dev, "{" BASE "\"ref_cut\":-0.1}"),
		"init accepted a negative ref_cut");
	free_com(&dev);

	current_test = "ref_knobs_out_of_range";
	CHECK(make_com(&dev, "{" BASE "\"ref_rate\":0}"),
		"init accepted ref_rate 0");
	free_com(&dev);
	CHECK(make_com(&dev, "{" BASE "\"ref_rate\":2}"),
		"init accepted ref_rate > 1");
	free_com(&dev);
	CHECK(make_com(&dev, "{" BASE "\"ref_floor\":1}"),
		"init accepted ref_floor 1");
	free_com(&dev);
	CHECK(make_com(&dev, "{" BASE "\"ref_warmup\":0}"),
		"init accepted ref_warmup 0");
	free_com(&dev);

	current_test = "ref_cut_needs_rows";
	CHECK(make_com(&dev, "{\"region_height\":3,\"region_width\":12,"
		"\"track\":true,\"ref_cut\":0.5}"),
		"init accepted a window too short to have a row profile");
	free_com(&dev);

	current_test = "ref_cut_ignored_without_track";
	CHECK(!make_com(&dev, "{\"region_height\":12,\"region_width\":12,"
		"\"ref_cut\":0.5}"), "init failed");
	struct aylp_center_of_mass_data *data = dev.device_data;
	CHECK(data->ref_cut == 0.0, "ref_cut = %G outside track mode, want 0",
		data->ref_cut);
	free_com(&dev);
}

/** The chop case end to end: the cut edge walks through the beam the way a chop
* beating against the frame rate does, while the source brightness also drifts.
* The emitted centroid must stay put throughout. */
static void test_chopped_sequence(void)
{
	current_test = "chopped_sequence";
	struct aylp_device dev;
	CHECK(!make_com(&dev, "{" BASE "\"min_peak\":50,"
		"\"reacquire_after\":1000}"), "init failed");
	gsl_matrix_uchar *full = full_beam(16.0, 16.0);
	warm_up(&dev, full, WARMUP);
	double oy, ox;
	run_frame(&dev, full, &oy, &ox);
	double good_y = oy, good_x = ox;
	const double tol = 0.1 * 2.0/(IMG - 1);	// a tenth of a pixel

	size_t n_cut = 0, n_caught = 0;
	unsigned char level = 200;
	for (double edge = 14.0; edge <= 18.0; edge += 0.5) {
		// the source dims as the sequence runs, to keep the gate
		// honest about telling "dim" from "cut"
		level = (unsigned char)(level * 0.95);
		gsl_matrix_uchar *cut = make_beam(16.0, 16.0, 2.0, level,
			-1e9, edge);
		struct aylp_center_of_mass_data *d = dev.device_data;
		size_t before = d->n_held;
		run_frame(&dev, cut, &oy, &ox);
		n_caught += d->n_held > before;
		n_cut++;
		CHECK(fabs(oy - good_y) <= tol && fabs(ox - good_x) <= tol,
			"cut at row %.1f (peak %u) moved the output to "
			"(%.6f,%.6f), more than 0.1 px off", edge, level,
			oy, ox);
		gsl_matrix_uchar_free(cut);

		gsl_matrix_uchar *whole = make_beam(16.0, 16.0, 2.0, level,
			-1e9, 1e9);
		run_frame(&dev, whole, &oy, &ox);
		CHECK(fabs(oy - good_y) <= tol,
			"whole beam at peak %u was mishandled: %.6f", level, oy);
		gsl_matrix_uchar_free(whole);
	}
	CHECK(n_caught == n_cut, "only %zu of %zu cuts were rejected",
		n_caught, n_cut);

	gsl_matrix_uchar_free(full);
	free_com(&dev);
}


int main(void)
{
	// the gate tests deliberately trip the beam-lost path, and its log
	// lines are expected output rather than failures
	log_init(LOG_ERROR);

	test_full_beam_passes();
	test_moving_beam_passes();
	test_brightness_tests_cannot_see_a_cut();
	test_cut_is_rejected_and_held();
	test_cut_from_either_side();
	test_dim_beam_is_not_a_cut();
	test_tight_beam();
	test_bootstrap_survives_chopping();
	test_gate_off_during_warmup();
	test_gate_off_is_old_behaviour();
	test_reference_reset_on_reacquire();
	test_dark_frame_holds();
	test_init_validation();
	test_chopped_sequence();

	if (n_fail)
		fprintf(stderr, "%u check(s) failed\n", n_fail);
	else
		fprintf(stderr, "all center_of_mass gate checks passed\n");
	return n_fail ? 1 : 0;
}
