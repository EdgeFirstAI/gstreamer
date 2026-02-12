if not edgefirst_hal_dep.found()
  subdir_done()
endif

gst_hal_sources = files(
  'plugin.c',
  'edgefirstcameraadaptor.c',
  'edgefirstcameraadaptor-neon.c',
)

gstedgefirsthal = shared_library('gstedgefirsthal',
  gst_hal_sources,
  c_args : ['-DHAVE_CONFIG_H'],
  include_directories : [config_inc],
  dependencies : [
    gst_dep,
    gst_base_dep,
    gst_video_dep,
    gst_allocators_dep,
    edgefirst_hal_dep,
    gstedgefirst_dep,
  ],
  install : true,
  install_dir : plugins_install_dir,
)
