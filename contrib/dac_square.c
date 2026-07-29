// Standalone square-wave generator for the BP-DAC81404EVM, for measuring the
// analog rise/fall time of the DAC output on a scope.
//
// Independent of anyloop: it maps the AX99100's BAR itself and bit-bangs the
// same 24-bit frames devices/parport_dac.c does, so you can put a clean,
// repeating edge on the output without standing up a pipeline. Nothing here
// touches the loop, and it does not need anyloop to be built.
//
// WHY A STANDALONE TOOL: anyloop's test_source has kinds constant/sine/noise
// but no square, and a square wave built out of the pipeline would carry the
// loop's own scheduling jitter into the edge timing -- which is exactly what
// you don't want when the thing under measurement IS the edge.
//
// WHAT ACTUALLY SETS THE RISE TIME. Every channel is in ASYNCHRONOUS mode
// (SYNCCONFIG = 0), where "a DAC data register write results in an immediate
// update of the DAC active register and DAC output on a SYNC rising edge"
// (SLASEH2A 8.3.3.1.2). So the output steps on ONE edge -- the SYNC rising
// edge that closes the frame -- and what you see afterwards is the DAC's own
// slew and settling. It does NOT depend on how fast we clocked the frame in.
// That is worth knowing before you distrust a reading: a slow frame does not
// smear the analog edge, it only delays it.
//
// TRIGGER OUTPUT. D3 (DB25 pin 5) is driven as a digital copy of the square
// wave, and its edge is emitted on the SAME MMIO store that raises SYNC --
// see square_frame(). So the trigger edge and the DAC's update instant
// coincide to within one store (~123 ns), which makes D3 a usable time zero:
//   ch1 -> DAC_VOUT_x (the analog edge)
//   ch2 -> DB25 pin 5 (trigger), scope triggered on this, rising
// Then t=0 on screen is the update instant, and you read the analog rise
// directly off it -- including the DAC's own dead time before it starts to
// move, which a scope triggered on the analog edge itself cannot show you.
//
// LEVELS AND YOUR SUPPLY RAILS -- READ BEFORE PICKING --low/--high. If the
// EVM is strapped unipolar (J11 2-3, DAC_AVSS grounded), the part cannot
// drive below ground no matter what range is selected, so a negative --low
// will simply clamp near 0 V and the "fall time" you measure will be the
// amplifier hitting its rail, not a real settling edge. The defaults here
// stay at 0 V and above for that reason. AVDD also needs headroom over the
// top of the swing. Widen the step only once you know how J11/J17 are set.
//
// BUILD, either way:
//   ninja -C build                 -> build/dac_square (it is a meson target)
//   gcc -O2 -o dac_square contrib/dac_square.c -lm    (standalone; -lm is for
//                    lround/llround and must come AFTER the source file)
// It is not installed -- run it from the build tree.
//
// RUN (needs root for the BAR mapping, same as backend="mmio"):
//   sudo ./dac_square                       # 1 kHz, 0 -> 5 V on channel A
//   sudo ./dac_square --freq 200 --high 9   # slower, bigger step
//   sudo ./dac_square --low 0 --high 10 --range 0-10 --rt
//   sudo ./dac_square --list-ranges
//
// It unbinds parport_pc from the card (same as the driver's `unbind`
// default). Rebind when you're done if you want /dev/parport0 back:
//   echo 0000:05:00.2 | sudo tee /sys/bus/pci/drivers/parport_pc/bind
//
// Ctrl-C (or SIGTERM) parks the output at --low, unmaps the BAR, and exits
// cleanly rather than leaving the DAC sitting at whatever level the last
// frame happened to write.  parport_pc remains unbound, as requested at
// startup, so it cannot reclaim the port while this tool owns the hardware.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

// AX99100 SPP register offsets inside the memory BAR, and the ECR that puts
// the card in plain SPP mode -- all identical to parport_dac.c's defaults.
#define SPP_DATA_OFFSET    0x280
#define SPP_STATUS_OFFSET  0x284
#define SPP_CONTROL_OFFSET 0x288
#define SPP_ECR_OFFSET     0x2A8
#define SPP_ECR_VALUE      0x14		// SPP mode, interrupts disabled
#define SPP_BAR            2

// DAC81404 registers (SLASEH2A table 8-7)
#define DAC_REG_SPICONFIG  0x03
#define DAC_REG_GENCONFIG  0x04
#define DAC_REG_SYNCCONFIG 0x06
#define DAC_REG_DACPWDWN   0x09
#define DAC_REG_DACRANGE   0x0A
#define DAC_REG_TRIGGER    0x0E
#define DAC_REG_DAC0       0x10		// DACA; B/C/D follow at 0x11-0x13

// Same constants parport_dac.c derives these from; see its comments for the
// reasoning behind each (particularly SPICONFIG clearing DEV-PWDWN, which is
// what makes the part actually drive its outputs at all).
#define DAC_SPICONFIG_ACTIVE   0x0A86
#define DAC_GENCONFIG_REF_ON   0x0000
#define DAC_SYNCCONFIG_ASYNC   0x0000
#define DAC_PWDWN_NONE         0xFFFF
#define DAC_TRIGGER_SOFT_RESET 0x000A

// SLASEH2A equations 1/2: the range is divided into 2^16 steps, so code
// 0xFFFF sits one LSB BELOW the nominal top of the range.
#define DAC_CODE_COUNT 65536.0
#define DAC_CODE_MAX   65535

// SLASEH2A tDACWAIT (tables 7-6, 7-7) and 8.3.3.1: "In both update modes, a
// minimum wait time of 2.4 us is required between DAC output updates."
#define DAC_UPDATE_WAIT_NS 2400

struct dac_range {
	const char *name;
	int code;
	double vmin, vmax;
};

// SLASEH2A table 8-16
static const struct dac_range dac_ranges[] = {
	{ "0-5",   0x0,   0.0,  5.0 },
	{ "0-10",  0x1,   0.0, 10.0 },
	{ "0-20",  0x2,   0.0, 20.0 },
	{ "0-40",  0x3,   0.0, 40.0 },
	{ "+-5",   0x5,  -5.0,  5.0 },
	{ "+-10",  0x6, -10.0, 10.0 },
	{ "+-20",  0x7, -20.0, 20.0 },
	{ "0-6",   0x8,   0.0,  6.0 },
	{ "0-12",  0x9,   0.0, 12.0 },
	{ "0-24",  0xA,   0.0, 24.0 },
	{ "+-6",   0xD,  -6.0,  6.0 },
	{ "+-12",  0xE, -12.0, 12.0 },
};

static const struct dac_range *find_range(const char *name)
{
	if (!strncmp(name, "±", strlen("±")))
		name += strlen("±");
	else if (name[0] == '+' && name[1] == '-') name += 2;
	else goto unipolar;
	for (size_t i = 0; i < sizeof dac_ranges / sizeof dac_ranges[0]; i++)
		if (dac_ranges[i].vmin < 0.0
		&& !strcmp(dac_ranges[i].name + 2, name))
			return &dac_ranges[i];
	return NULL;
unipolar:
	for (size_t i = 0; i < sizeof dac_ranges / sizeof dac_ranges[0]; i++)
		if (!strcmp(dac_ranges[i].name, name))
			return &dac_ranges[i];
	return NULL;
}

static int volts_to_code(double volts, double vmin, double vmax)
{
	double frac = (volts - vmin) / (vmax - vmin);
	long c = lround(frac * DAC_CODE_COUNT);
	if (c < 0) c = 0;
	if (c > DAC_CODE_MAX) c = DAC_CODE_MAX;
	return (int)c;
}

static double code_to_volts(int code, double vmin, double vmax)
{
	return vmin + (vmax - vmin) * code / DAC_CODE_COUNT;
}

// ---------------------------------------------------------------------------
// parallel port

static volatile uint8_t *reg_data, *reg_ecr;
static uint8_t data_shadow;

static volatile sig_atomic_t stop_requested;

static void on_sigint(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static inline void pp_write_data(uint8_t v)
{
	data_shadow = v;
	*reg_data = v;
}

static volatile uint8_t *map_bar(const char *pci, int bar, size_t span,
	void **map_out, size_t *len_out)
{
	char path[128];
	snprintf(path, sizeof path,
		"/sys/bus/pci/devices/%s/resource%d", pci, bar);
	int fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		if (errno == EACCES || errno == EPERM)
			fprintf(stderr,
				"Mapping a BAR needs root (CAP_SYS_RAWIO), and "
				"kernel lockdown must be off -- check that\n"
				"`cat /sys/kernel/security/lockdown` shows "
				"[none] in brackets.\n");
		return NULL;
	}
	long page = sysconf(_SC_PAGESIZE);
	size_t len = (size_t)page;
	while (len < span + 1) len += (size_t)page;
	void *m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (m == MAP_FAILED) {
		fprintf(stderr, "mmap %s: %s\n", path, strerror(errno));
		return NULL;
	}
	*map_out = m;
	*len_out = len;
	return (volatile uint8_t *)m;
}

static void unbind_parport_pc(const char *pci)
{
	int fd = open("/sys/bus/pci/drivers/parport_pc/unbind", O_WRONLY);
	if (fd < 0) {
		if (errno != ENOENT)
			fprintf(stderr, "warning: open parport_pc unbind: "
				"%s\n", strerror(errno));
		return;
	}
	ssize_t w = write(fd, pci, strlen(pci));
	close(fd);
	// ENODEV just means it wasn't bound in the first place
	if (w < 0 && errno != ENODEV)
		fprintf(stderr, "warning: unbind %s: %s\n", pci,
			strerror(errno));
}

// ---------------------------------------------------------------------------
// DAC frames

static int sclk_bit = 0, sdin_bit = 1, sync_bit = 2, trig_bit = 3;

/** Emit one 24-bit write cycle. `base_after` supplies the non-SPI data pins
* for the FINAL store only -- the one that raises SYNC and therefore updates
* the output -- while `base_before` supplies them for every store before it.
* Passing two different values is how the D3 trigger edge is made to land on
* the same store as the output update; passing the same value twice leaves
* the other pins alone, which is what the config writes want.
*
* SDIN is sampled on SCLK falling edges and the cycle is bracketed by SYNC
* going low and back high (SLASEH2A 8.5.1), so each bit costs two stores.
* Idle state is SYNC high, SCLK high. */
static void square_frame(uint8_t addr, uint16_t value,
	uint8_t base_before, uint8_t base_after)
{
	uint8_t sclk = (uint8_t)(1u << sclk_bit);
	uint8_t sdin = (uint8_t)(1u << sdin_bit);
	uint8_t sync = (uint8_t)(1u << sync_bit);
	uint8_t mask = (uint8_t)~(sclk | sdin | sync);
	uint8_t b = base_before & mask;
	uint8_t a = base_after & mask;
	// bit 23 = R/W (0 = write), bits 21-16 = address, bits 15-0 = data
	uint32_t frame = ((uint32_t)(addr & 0x3F) << 16) | value;

	pp_write_data(b | sync | sclk);		// idle
	pp_write_data(b | sclk);		// SYNC low: cycle start
	for (int i = 23; i >= 0; i--) {
		uint8_t bit = (frame >> i) & 1 ? sdin : 0;
		pp_write_data(b | sclk | bit);
		pp_write_data(b | bit);		// falling edge samples
	}
	pp_write_data(b | sclk);
	// SYNC rising: the DAC updates here, and the trigger pin moves with it
	pp_write_data(a | sync | sclk);
}

static void dac_write(uint8_t addr, uint16_t value)
{
	uint8_t base = data_shadow;
	square_frame(addr, value, base, base);
}

/** Same order parport_dac.c's dac_configure() uses. SPICONFIG first because
* it clears DEV-PWDWN, the device-wide power-down the part wakes up in --
* without it every later write is accepted and no output ever moves. */
static void dac_configure(int channel, int range_code, bool reset)
{
	if (reset) {
		dac_write(DAC_REG_TRIGGER, DAC_TRIGGER_SOFT_RESET);
		// SLASEH2A 8.3.5: a soft reset is a POR event, and
		// "communication with the device is valid only after a 1 ms
		// POR delay". 5 ms is comfortably past that.
		struct timespec ts = {0, 5000000};
		nanosleep(&ts, NULL);
	}
	dac_write(DAC_REG_SPICONFIG, DAC_SPICONFIG_ACTIVE);
	dac_write(DAC_REG_GENCONFIG, DAC_GENCONFIG_REF_ON);
	dac_write(DAC_REG_SYNCCONFIG, DAC_SYNCCONFIG_ASYNC);
	dac_write(DAC_REG_DACRANGE,
		(uint16_t)((range_code & 0xF) << (4 * channel)));
	// power up only the channel we drive; the rest stay clamped to ground
	dac_write(DAC_REG_DACPWDWN,
		(uint16_t)(DAC_PWDWN_NONE & ~(1u << channel)));
}

// ---------------------------------------------------------------------------
// timing

static inline int64_t ts_to_ns(const struct timespec *t)
{
	return (int64_t)t->tv_sec * 1000000000LL + t->tv_nsec;
}

static inline void ns_to_ts(int64_t ns, struct timespec *t)
{
	t->tv_sec = ns / 1000000000LL;
	t->tv_nsec = ns % 1000000000LL;
}

/** Sleep until an absolute CLOCK_MONOTONIC deadline, then spin out the last
* `spin_ns`. Deadlines are absolute and accumulated from a fixed origin, so
* the wave does not drift the way a loop of relative sleeps would. */
static void wait_until(int64_t deadline_ns, int64_t spin_ns)
{
	struct timespec t;
	if (deadline_ns - spin_ns > 0) {
		ns_to_ts(deadline_ns - spin_ns, &t);
		while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t,
			NULL) == EINTR && !stop_requested)
			;
	}
	do {
		clock_gettime(CLOCK_MONOTONIC, &t);
	} while (ts_to_ns(&t) < deadline_ns);
}

static void try_realtime(void)
{
	struct sched_param sp = { .sched_priority = 80 };
	if (sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "warning: SCHED_FIFO: %s (running normal "
			"priority)\n", strerror(errno));
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
		fprintf(stderr, "warning: mlockall: %s\n", strerror(errno));
}

// ---------------------------------------------------------------------------

static void usage(const char *me)
{
	printf(
"Usage: sudo %s [options]\n"
"\n"
"Drives a square wave out of one DAC81404 channel so you can measure the\n"
"analog rise and fall times on a scope.\n"
"\n"
"  --pci ADDR      PCI address of the parallel function (default"
					" 0000:05:00.2)\n"
"  --channel N     DAC channel 0-3 = A-D (default 0)\n"
"  --range NAME    output range (default 0-5; --list-ranges to see all)\n"
"  --low V         low level in volts (default 0.0)\n"
"  --high V        high level in volts (default 5.0)\n"
"  --freq HZ       square wave frequency (default 1000)\n"
"  --duty FRAC     high fraction of the period, 0-1 (default 0.5)\n"
"  --cycles N      stop after N cycles (default 0 = run until Ctrl-C)\n"
"  --trig-bit N    data bit for the scope trigger, 3-7 = DB25 pins 5-9\n"
"                  (default 3); --trig-bit -1 disables it\n"
"  --rt            SCHED_FIFO + mlockall, for less edge jitter\n"
"  --no-reset      skip the soft reset at startup\n"
"  --no-unbind     don't unbind parport_pc first\n"
"  --list-ranges   print the range table and exit\n"
"\n"
"Scope setup: ch1 on DAC_VOUT_<channel>, ch2 on the trigger pin, trigger on\n"
"ch2 rising for the rise time and ch2 falling for the fall time. The trigger\n"
"edge lands on the same store that updates the DAC, so t=0 is the update\n"
"instant and the DAC's dead time before it starts moving is visible.\n"
"\n"
"If the EVM is strapped unipolar (J11 2-3, AVSS grounded) the part cannot go\n"
"below ground -- keep --low at 0 or above, or you will measure the output\n"
"amplifier against its rail instead of a real edge.\n",
		me);
}

int main(int argc, char **argv)
{
	const char *pci = "0000:05:00.2";
	// 0-5 is the house range for this rig (see the steering/bode par
	// configs); a full 0->5 V step is also the right stimulus for a
	// slew/settling measurement.
	const char *range_name = "0-5";
	int channel = 0;
	double low_v = 0.0, high_v = 5.0, freq = 1000.0, duty = 0.5;
	long cycles = 0;
	bool reset = true, unbind = true, realtime = false;

	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		bool has_next = i + 1 < argc;
		if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
			usage(argv[0]);
			return 0;
		} else if (!strcmp(a, "--list-ranges")) {
			printf("%-8s %-6s %s\n", "name", "code", "span");
			for (size_t j = 0;
				j < sizeof dac_ranges / sizeof dac_ranges[0];
				j++)
				printf("%-8s 0x%X    %g to %g V\n",
					dac_ranges[j].name, dac_ranges[j].code,
					dac_ranges[j].vmin, dac_ranges[j].vmax);
			return 0;
		} else if (!strcmp(a, "--rt")) {
			realtime = true;
		} else if (!strcmp(a, "--no-reset")) {
			reset = false;
		} else if (!strcmp(a, "--no-unbind")) {
			unbind = false;
		} else if (!has_next) {
			fprintf(stderr, "%s needs an argument\n", a);
			return 1;
		} else if (!strcmp(a, "--pci")) {
			pci = argv[++i];
		} else if (!strcmp(a, "--range")) {
			range_name = argv[++i];
		} else if (!strcmp(a, "--channel")) {
			channel = atoi(argv[++i]);
		} else if (!strcmp(a, "--low")) {
			low_v = atof(argv[++i]);
		} else if (!strcmp(a, "--high")) {
			high_v = atof(argv[++i]);
		} else if (!strcmp(a, "--freq")) {
			freq = atof(argv[++i]);
		} else if (!strcmp(a, "--duty")) {
			duty = atof(argv[++i]);
		} else if (!strcmp(a, "--cycles")) {
			cycles = atol(argv[++i]);
		} else if (!strcmp(a, "--trig-bit")) {
			trig_bit = atoi(argv[++i]);
		} else {
			fprintf(stderr, "unrecognized option: %s\n", a);
			return 1;
		}
	}

	if (channel < 0 || channel > 3) {
		fprintf(stderr, "--channel must be 0-3 (A-D)\n");
		return 1;
	}
	const struct dac_range *rng = find_range(range_name);
	if (!rng) {
		fprintf(stderr, "unknown range \"%s\"; try --list-ranges\n",
			range_name);
		return 1;
	}
	if (freq <= 0.0) {
		fprintf(stderr, "--freq must be positive\n");
		return 1;
	}
	if (duty <= 0.0 || duty >= 1.0) {
		fprintf(stderr, "--duty must be strictly between 0 and 1\n");
		return 1;
	}
	if (trig_bit != -1 && (trig_bit < 3 || trig_bit > 7)) {
		fprintf(stderr, "--trig-bit must be 3-7 (D3-D7, DB25 pins "
			"5-9), or -1 to disable; D0-D2 are SCLK/SDIN/SYNC\n");
		return 1;
	}
	if (low_v < rng->vmin || high_v > rng->vmax) {
		fprintf(stderr, "--low/--high (%g, %g) fall outside the %s "
			"range (%g to %g V)\n", low_v, high_v, rng->name,
			rng->vmin, rng->vmax);
		return 1;
	}
	if (high_v <= low_v) {
		fprintf(stderr, "--high must be above --low\n");
		return 1;
	}

	int64_t period_ns = (int64_t)llround(1e9 / freq);
	int64_t high_ns = (int64_t)llround(period_ns * duty);
	int64_t low_ns = period_ns - high_ns;
	// SLASEH2A 8.3.3.1: at least 2.4 us between DAC output updates, in
	// both update modes. The shorter of the two phases is what binds.
	int64_t shortest = high_ns < low_ns ? high_ns : low_ns;
	if (shortest < DAC_UPDATE_WAIT_NS) {
		fprintf(stderr, "at %g Hz with duty %g the shorter phase is "
			"%lld ns, below the DAC's %d ns minimum between "
			"output updates (SLASEH2A 8.3.3.1) -- lower --freq or "
			"move --duty toward 0.5\n", freq, duty,
			(long long)shortest, DAC_UPDATE_WAIT_NS);
		return 1;
	}

	int code_lo = volts_to_code(low_v, rng->vmin, rng->vmax);
	int code_hi = volts_to_code(high_v, rng->vmin, rng->vmax);
	double act_lo = code_to_volts(code_lo, rng->vmin, rng->vmax);
	double act_hi = code_to_volts(code_hi, rng->vmin, rng->vmax);

	// Install this before unbinding or mapping the card.  The handler only
	// records the request; all DAC and VM work stays in normal process context.
	struct sigaction sa = { .sa_handler = on_sigint };
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	if (unbind) unbind_parport_pc(pci);
	if (stop_requested) return 0;

	size_t span = SPP_ECR_OFFSET;
	void *map = NULL;
	size_t map_len = 0;
	volatile uint8_t *base = map_bar(pci, SPP_BAR, span, &map, &map_len);
	if (!base) return 1;
	if (stop_requested) {
		munmap(map, map_len);
		return 0;
	}
	reg_data = base + SPP_DATA_OFFSET;
	reg_ecr = base + SPP_ECR_OFFSET;
	*reg_ecr = SPP_ECR_VALUE;

	// idle: SYNC high, SCLK high, trigger low
	uint8_t trig = trig_bit >= 0 ? (uint8_t)(1u << trig_bit) : 0;
	pp_write_data((uint8_t)((1u << sync_bit) | (1u << sclk_bit)));

	dac_configure(channel, rng->code, reset);

	if (realtime) try_realtime();

	printf("DAC81404 square wave on channel %c (DAC_VOUT_%c)\n",
		'A' + channel, 'A' + channel);
	printf("  range      %s (%g to %g V)\n", rng->name, rng->vmin,
		rng->vmax);
	printf("  low        %.4f V requested -> code 0x%04X = %.4f V\n",
		low_v, code_lo, act_lo);
	printf("  high       %.4f V requested -> code 0x%04X = %.4f V\n",
		high_v, code_hi, act_hi);
	printf("  step       %.4f V\n", act_hi - act_lo);
	printf("  10%%-90%%    %.4f V to %.4f V\n",
		act_lo + 0.1 * (act_hi - act_lo),
		act_lo + 0.9 * (act_hi - act_lo));
	printf("  frequency  %g Hz (period %lld ns, high %lld ns, low %lld "
		"ns)\n", freq, (long long)period_ns, (long long)high_ns,
		(long long)low_ns);
	if (trig_bit >= 0)
		printf("  trigger    D%d = DB25 pin %d, edge coincident with "
			"the DAC update\n", trig_bit, trig_bit + 2);
	else
		printf("  trigger    disabled\n");
	printf("Ctrl-C to stop (parks at %.4f V).\n\n", act_lo);
	fflush(stdout);

	uint8_t addr = (uint8_t)(DAC_REG_DAC0 + channel);
	struct timespec t0;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	int64_t origin = ts_to_ns(&t0);
	// spin out the last 60 us of each wait; below that clock_nanosleep's
	// own wakeup latency is the dominant error on a normal-priority task
	int64_t spin_ns = 60000;
	if (spin_ns > shortest / 2) spin_ns = shortest / 2;

	long done = 0, late = 0;
	int64_t worst_late = 0;
	int64_t next = origin;
	while (!stop_requested && (cycles == 0 || done < cycles)) {
		// If a cycle's own start deadline has already passed, the
		// requested frequency is beyond what this path can sustain
		// and the wave is no longer at --freq. Count it rather than
		// free-running silently at whatever rate the loop manages,
		// which would quietly invalidate the timebase you're
		// measuring against.
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		int64_t lateness = ts_to_ns(&now) - next;
		if (lateness > 0) {
			late++;
			if (lateness > worst_late) worst_late = lateness;
		}
		// rising edge: trigger pin goes high on the same store that
		// raises SYNC and updates the output
		wait_until(next, spin_ns);
		if (stop_requested) break;
		square_frame(addr, (uint16_t)code_hi, data_shadow & ~trig,
			(uint8_t)(data_shadow | trig));

		// falling edge
		wait_until(next + high_ns, spin_ns);
		if (stop_requested) break;
		square_frame(addr, (uint16_t)code_lo, data_shadow | trig,
			(uint8_t)(data_shadow & ~trig));

		next += period_ns;
		done++;
	}

	// park low, trigger low, so the output isn't left wherever the last
	// frame happened to put it
	square_frame(addr, (uint16_t)code_lo, data_shadow & ~trig,
		(uint8_t)(data_shadow & ~trig));

	printf("\n%ld cycles emitted; parked at %.4f V.\n", done, act_lo);
	if (late)
		printf("WARNING: %ld of %ld cycles started late (worst %.1f "
			"us). The output was NOT a clean %g Hz square -- lower "
			"--freq, or try --rt.\n", late, done,
			worst_late / 1000.0, freq);
	if (unbind)
		printf("parport_pc is still unbound. Rebind with:\n"
			"  echo %s | sudo tee "
			"/sys/bus/pci/drivers/parport_pc/bind\n", pci);

	if (map) munmap(map, map_len);
	return 0;
}
