project(
	'anyloop',
	'c',
	default_options: ['c_std=gnu17', 'warning_level=3', 'optimization=3']
	#default_options: ['warning_level=everything', 'optimization=3']
	# this is worth occasionally turning on to check
)

c = meson.get_compiler('c')
deps = [
	dependency('json-c'),
	dependency('gsl'),
	dependency('threads'),
	dependency('libusb-1.0'),
	c.find_library('dl')
]

project_source_files = [
	'libaylp/anyloop.c',
	'libaylp/logging.c',
	'libaylp/block.c',
	'libaylp/config.c',
	'libaylp/pretty.c',
	'libaylp/types.c',
	'libaylp/thread_pool.c',
	'libaylp/xalloc.c',
	'libaylp/profile.c',
	'devices/center_of_mass.c',
	'devices/kdc101.c',
	'devices/tic_t834.c',
	'devices/clamp.c',
	'devices/biquad_filter.c',
	'devices/kalman_filter.c',
	'devices/device.c',
	'devices/delay.c',
	'devices/file_sink.c',
	'devices/logger.c',
	'devices/matmul.c',
	'devices/pid.c',
	'devices/fsp.c',
	'devices/poke.c',
	'devices/remove_piston.c',
	'devices/stop_after_count.c',
	'devices/test_source.c',
	'devices/udp_sink.c',
	'devices/udp_source.c',
	'devices/vonkarman_stream.c',
	'devices/piplate_bridge.c',
	'devices/parport_dac.c',
	'devices/bode_plot.c',
	'devices/latency_test.c',
	'devices/attenuation_test.c'
]

incdir = include_directories('libaylp')

executable('anyloop', project_source_files,
	install: true,
	dependencies: deps,
	include_directories: incdir,
	export_dynamic: true
)

# Unit tests. These link the device sources they exercise directly rather than
# going through the anyloop binary, so they need no camera and no config file --
# `ninja -C build test` is safe to run on any machine.
test_center_of_mass = executable('test_center_of_mass',
	[
		'tests/test_center_of_mass.c',
		'devices/center_of_mass.c',
		'libaylp/logging.c',
		'libaylp/thread_pool.c',
		'libaylp/types.c',
		'libaylp/xalloc.c'
	],
	install: false,
	dependencies: deps,
	include_directories: [incdir, include_directories('devices')]
)
test('center_of_mass', test_center_of_mass)

test_pid_park = executable('test_pid_park',
	[
		'tests/test_pid_park.c',
		'devices/pid.c',
		'libaylp/logging.c',
		'libaylp/types.c',
		'libaylp/xalloc.c'
	],
	install: false,
	dependencies: deps,
	include_directories: [incdir, include_directories('devices')]
)
test('pid_park', test_pid_park)

shared_library('plugin_example', 'contrib/plugin_example.c',
	name_prefix: '',
	install: true,
	install_dir: get_option('libdir') / 'anyloop',
	dependencies: deps,
	include_directories: incdir,
	override_options: 'b_lundef=false'
)

# dac_square: standalone bench tool that puts a square wave on one DAC81404
# channel so the analog rise/fall time can be measured on a scope. Deliberately
# independent of libaylp -- it maps the AX99100 BAR and bit-bangs the frames
# itself, so it needs no anyloop deps, only libm for lround/llround. Built here
# purely so `ninja -C build` covers it; it is not installed, since it is a
# diagnostic you run from the build tree with sudo.
executable('dac_square', 'contrib/dac_square.c',
	install: false,
	dependencies: [c.find_library('m', required: false)]
)

# parport_pins: standalone bench tool that parks SCLK/SDIN/SYNC at a static
# level so the DB25 pins can be metered. Same standalone posture as dac_square
# (maps the BAR itself, no libaylp, not installed), and needs no libm.
executable('parport_pins', 'contrib/parport_pins.c',
	install: false
)

asi_sdk_dir = '/home/fsl/ASIStudio/lib'
asi_lib = c.find_library('ASICamera2', dirs: [asi_sdk_dir], required: false)
fs = import('fs')
if asi_lib.found() and fs.exists('contrib/asi_source.c')
	shared_library('asi_source', 'contrib/asi_source.c',
		name_prefix: '',
		install: true,
		install_dir: get_option('libdir') / 'anyloop',
		dependencies: deps + [asi_lib],
		include_directories: [incdir, include_directories('contrib')],
		build_rpath: asi_sdk_dir,
		install_rpath: asi_sdk_dir,
		override_options: 'b_lundef=false'
	)
else
	message('ZWO ASI SDK not found at ' + asi_sdk_dir + '; asi_source will not be built')
endif

# aylp_alsa: soundcard (ALSA playback) actuator plugin, for the AC path of a
# dual-actuator FSM drive. Built only if alsa-lib and the source are present.
alsa_dep = dependency('alsa', required: false)
if alsa_dep.found() and fs.exists('contrib/aylp_alsa.c')
	shared_library('aylp_alsa', 'contrib/aylp_alsa.c',
		name_prefix: '',
		install: true,
		install_dir: get_option('libdir') / 'anyloop',
		dependencies: deps + [alsa_dep],
		include_directories: [incdir, include_directories('contrib')],
		override_options: 'b_lundef=false'
	)
else
	message('alsa-lib not found; aylp_alsa will not be built')
endif

