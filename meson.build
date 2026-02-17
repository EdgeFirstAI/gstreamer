project('edgefirst-gstreamer', 'c',
  version : '0.1.1',
  meson_version : '>= 0.62',
  default_options : [
    'warning_level=2',
    'c_std=c11',
    'buildtype=debugoptimized',
  ]
)

gst_version = '>=1.20.0'
gst_dep = dependency('gstreamer-1.0', version : gst_version)
gst_base_dep = dependency('gstreamer-base-1.0', version : gst_version)
gst_video_dep = dependency('gstreamer-video-1.0', version : gst_version)
zenoh_c_dep = dependency('zenohc', required : get_option('zenoh'))
edgefirst_schemas_dep = dependency('edgefirst-schemas', version : '>=1.5',
                                   required : get_option('zenoh'))

gst_allocators_dep = dependency('gstreamer-allocators-1.0', version : gst_version,
                                required : get_option('hal'))
edgefirst_hal_dep = dependency('edgefirst-hal', required : get_option('hal'))

glib_dep = dependency('glib-2.0', version : '>=2.56')
gobject_dep = dependency('gobject-2.0')

# NNStreamer is optional but recommended
nnstreamer_dep = dependency('nnstreamer', version : '>=2.0', required : false)

plugins_install_dir = join_paths(get_option('libdir'), 'gstreamer-1.0')

# Project-wide include directory
config_inc = include_directories('.')

# Generate configuration header
conf_data = configuration_data()
conf_data.set_quoted('PACKAGE', meson.project_name())
conf_data.set_quoted('PACKAGE_VERSION', meson.project_version())
conf_data.set_quoted('PACKAGE_NAME', meson.project_name())
conf_data.set10('HAVE_NNSTREAMER', nnstreamer_dep.found())

configure_file(
  output : 'config.h',
  configuration : conf_data
)

# Subdirectories
subdir('gst-libs/gst/edgefirst')
subdir('gst/zenoh')
subdir('gst/fusion')
subdir('gst/hal')

if get_option('tests').enabled() or (get_option('tests').auto() and not meson.is_subproject())
  subdir('tests')
endif
