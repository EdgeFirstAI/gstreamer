# Test infrastructure for edgefirst-gstreamer

gst_check_dep = dependency('gstreamer-check-1.0', version : gst_version,
                           required : false)

test_install_dir = get_option('libexecdir') / meson.project_name()

if gst_check_dep.found()
  test_env = environment()
  test_env.set('GST_PLUGIN_PATH_1_0', meson.project_build_root())
  test_env.set('GST_PLUGIN_SYSTEM_PATH_1_0', '')
  test_env.set('GST_REGISTRY', '@0@/registry.dat'.format(meson.current_build_dir()))

  # Core library tests
  test_meta = executable('test_meta',
    'check/test_meta.c',
    dependencies : [gst_dep, gst_check_dep, gstedgefirst_dep],
    include_directories : [config_inc],
    install : true,
    install_dir : test_install_dir,
  )
  test('meta', test_meta, env : test_env)

  test_meta_copy = executable('test_meta_copy',
    'check/test_meta_copy.c',
    dependencies : [gst_dep, gst_check_dep, gstedgefirst_dep],
    include_directories : [config_inc],
    install : true,
    install_dir : test_install_dir,
  )
  test('meta_copy', test_meta_copy, env : test_env)

  test_math = executable('test_math',
    'check/test_math.c',
    dependencies : [gst_dep, gst_check_dep, gstedgefirst_dep],
    include_directories : [config_inc],
    install : true,
    install_dir : test_install_dir,
  )
  test('math', test_math, env : test_env)

  fixture_dir = meson.current_source_dir() / 'fixtures'

  test_fusion = executable('test_fusion_elements',
    'check/test_fusion_elements.c',
    c_args : ['-DFIXTURE_DIR="@0@"'.format(fixture_dir)],
    dependencies : [gst_dep, gst_base_dep, gst_check_dep, gstedgefirst_dep],
    include_directories : [config_inc],
    install : true,
    install_dir : test_install_dir,
  )
  test('fusion_elements', test_fusion, env : test_env)

  # HAL plugin tests (only when HAL is available)
  if edgefirst_hal_dep.found()
    gst_app_dep = dependency('gstreamer-app-1.0', version : gst_version)

    # HAL pipeline tests need system elements (videotestsrc, appsink),
    # so use an env that does not blank the system plugin path.
    hal_test_env = environment()
    hal_test_env.set('GST_PLUGIN_PATH_1_0', meson.project_build_root())
    hal_test_env.set('GST_REGISTRY',
      '@0@/hal_registry.dat'.format(meson.current_build_dir()))

    hal_src_inc = include_directories('../gst/hal')
    test_hal = executable('test_hal_elements',
      'check/test_hal_elements.c',
      '../gst/hal/edgefirstcameraadaptor-neon.c',
      dependencies : [gst_dep, gst_base_dep, gst_check_dep,
                      gst_allocators_dep, gst_app_dep, gstedgefirst_dep],
      include_directories : [config_inc, hal_src_inc],
      install : true,
      install_dir : test_install_dir,
    )
    test('hal_elements', test_hal, env : hal_test_env)
  endif

endif
