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
	'libaylp/thread_pool.c',
	'libaylp/xalloc.c',
	'libaylp/profile.c',
	'devices/center_of_mass.c',
	'devices/kdc101.c',
	'devices/tic_t834.c',
	'devices/clamp.c',
	'devices/device.c',
	'devices/delay.c',
	'devices/file_sink.c',
	'devices/logger.c',
	'devices/matmul.c',
	'devices/pid.c',
	'devices/poke.c',
	'devices/remove_piston.c',
	'devices/stop_after_count.c',
	'devices/test_source.c',
	'devices/udp_sink.c',
	'devices/udp_source.c',
	'devices/vonkarman_stream.c',
	'devices/piplate_bridge.c',
]

incdir = include_directories('libaylp')

executable('anyloop', project_source_files,
	install: true,
	dependencies: deps,
	include_directories: incdir,
	export_dynamic: true
)

shared_library('plugin_example', 'contrib/plugin_example.c',
	name_prefix: '',
	install: true,
	install_dir: get_option('libdir') / 'anyloop',
	dependencies: deps,
	include_directories: incdir,
	override_options: 'b_lundef=false'
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

