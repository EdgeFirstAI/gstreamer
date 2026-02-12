if not get_option('zenoh').disabled()
  zenoh_c_dep = dependency('zenohc', required : get_option('zenoh'))
  edgefirst_schemas_dep = dependency('edgefirst-schemas', version : '>=1.5',
                                     required : get_option('zenoh'))

  if zenoh_c_dep.found() and edgefirst_schemas_dep.found()
    gst_zenoh_sources = files(
      'plugin.c',
      'edgefirstzenohsub.c',
      'edgefirstzenohpub.c',
      'transform-cache.c',
      'edgefirstzenoh-enums.c',
    )

    gstedgefirst_zenoh = shared_library('gstedgefirstzenoh',
      gst_zenoh_sources,
      c_args : ['-DHAVE_CONFIG_H'],
      include_directories : [config_inc],
      dependencies : [
        gst_dep,
        gst_base_dep,
        gst_video_dep,
        gstedgefirst_dep,
        zenoh_c_dep,
        edgefirst_schemas_dep,
      ],
      install : true,
      install_dir : plugins_install_dir,
    )
  endif
endif
