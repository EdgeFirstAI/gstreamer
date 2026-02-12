if get_option('fusion').enabled() or get_option('fusion').auto()
  json_glib_dep = dependency('json-glib-1.0')

  gst_fusion_sources = files(
    'plugin.c',
    'edgefirstpcdclassify.c',
    'edgefirsttransforminject.c',
  )

  gstedgefirst_fusion = shared_library('gstedgefirstfusion',
    gst_fusion_sources,
    c_args : ['-DHAVE_CONFIG_H'],
    include_directories : [config_inc],
    dependencies : [
      gst_dep,
      gst_base_dep,
      gst_video_dep,
      gstedgefirst_dep,
      json_glib_dep,
    ],
    install : true,
    install_dir : plugins_install_dir,
  )
endif
