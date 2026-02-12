gstedgefirst_sources = files(
  'edgefirst.c',
  'edgefirstpointcloud2meta.c',
  'edgefirstradarcubemeta.c',
  'edgefirsttransformmeta.c',
  'edgefirstcamerainfometa.c',
)

gstedgefirst_headers = files(
  'edgefirst.h',
  'edgefirstpointcloud2meta.h',
  'edgefirstradarcubemeta.h',
  'edgefirsttransformmeta.h',
  'edgefirstcamerainfometa.h',
  'edgefirst-perception-types.h',
)

install_headers(gstedgefirst_headers,
  subdir : 'gstreamer-1.0/gst/edgefirst'
)

edgefirst_inc = include_directories('../..')

gstedgefirst_lib = shared_library('edgefirst-gstreamer-1.0',
  gstedgefirst_sources,
  c_args : ['-DBUILDING_GST_EDGEFIRST', '-DHAVE_CONFIG_H'],
  include_directories : [config_inc, edgefirst_inc],
  dependencies : [gst_dep, gst_base_dep, glib_dep, gobject_dep],
  install : true,
)

gstedgefirst_dep = declare_dependency(
  link_with : gstedgefirst_lib,
  include_directories : edgefirst_inc,
  dependencies : [gst_dep, gst_base_dep],
)

pkg = import('pkgconfig')
pkg.generate(gstedgefirst_lib,
  name : 'edgefirst-gstreamer-1.0',
  description : 'EdgeFirst Perception GStreamer library',
  subdirs : 'gstreamer-1.0',
)
