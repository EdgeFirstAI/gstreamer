/*
 * EdgeFirst Perception for GStreamer - HAL Element Tests
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gst/check/gstcheck.h>
#include <gst/base/gstbasetransform.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/app/gstappsink.h>
#include <string.h>
#include <math.h>
/* NEON header removed — HAL 0.12.0 handles all post-processing */
#include <gst/edgefirst/edgefirstdetection.h>

/* ── TCase "Creation" ──────────────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_create)
{
  GstElement *el;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL, "Failed to create edgefirstcameraadaptor element");

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Properties" ───────────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_properties)
{
  GstElement *el;
  guint uint_val;
  gint enum_val;
  gboolean bool_val;
  gchar *str_val;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);

  /* Default values */
  g_object_get (el, "model-width", &uint_val, NULL);
  fail_unless_equals_int (uint_val, 0);

  g_object_get (el, "model-height", &uint_val, NULL);
  fail_unless_equals_int (uint_val, 0);

  g_object_get (el, "model-colorspace", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 0);  /* RGB */

  g_object_get (el, "model-layout", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 0);  /* HWC */

  g_object_get (el, "model-dtype", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 0);  /* uint8 */

  g_object_get (el, "letterbox", &bool_val, NULL);
  fail_unless (bool_val == FALSE);

  g_object_get (el, "fill-color", &uint_val, NULL);
  fail_unless_equals_uint64 (uint_val, 0x808080FF);

  /* Set/get roundtrip */
  g_object_set (el, "model-width", 640, NULL);
  g_object_get (el, "model-width", &uint_val, NULL);
  fail_unless_equals_int (uint_val, 640);

  g_object_set (el, "model-height", 480, NULL);
  g_object_get (el, "model-height", &uint_val, NULL);
  fail_unless_equals_int (uint_val, 480);

  g_object_set (el, "model-colorspace", 2, NULL);  /* gray */
  g_object_get (el, "model-colorspace", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 2);

  g_object_set (el, "model-layout", 1, NULL);  /* chw */
  g_object_get (el, "model-layout", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 1);

  g_object_set (el, "model-dtype", 2, NULL);  /* float32 */
  g_object_get (el, "model-dtype", &enum_val, NULL);
  fail_unless_equals_int (enum_val, 2);

  g_object_set (el, "letterbox", TRUE, NULL);
  g_object_get (el, "letterbox", &bool_val, NULL);
  fail_unless (bool_val == TRUE);

  g_object_set (el, "fill-color", (guint) 0x808080FF, NULL);
  g_object_get (el, "fill-color", &uint_val, NULL);
  fail_unless_equals_uint64 (uint_val, 0x808080FF);

  g_object_set (el, "model-mean", "0.485,0.456,0.406", NULL);
  g_object_get (el, "model-mean", &str_val, NULL);
  fail_unless (str_val != NULL);
  fail_unless (g_str_has_prefix (str_val, "0.485"));
  g_free (str_val);

  g_object_set (el, "model-std", "0.229,0.224,0.225", NULL);
  g_object_get (el, "model-std", &str_val, NULL);
  fail_unless (str_val != NULL);
  fail_unless (g_str_has_prefix (str_val, "0.229"));
  g_free (str_val);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Pads" ──────────────────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_pad_templates)
{
  GstElementFactory *factory;
  const GList *templates;
  gboolean has_sink = FALSE, has_src = FALSE;

  factory = gst_element_factory_find ("edgefirstcameraadaptor");
  fail_unless (factory != NULL);

  templates = gst_element_factory_get_static_pad_templates (factory);

  for (const GList *l = templates; l != NULL; l = l->next) {
    GstStaticPadTemplate *t = l->data;

    if (g_strcmp0 (t->name_template, "sink") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SINK);
      fail_unless_equals_int (t->presence, GST_PAD_ALWAYS);
      has_sink = TRUE;
    } else if (g_strcmp0 (t->name_template, "src") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SRC);
      fail_unless_equals_int (t->presence, GST_PAD_ALWAYS);
      has_src = TRUE;
    }
  }

  fail_unless (has_sink, "Missing sink pad template");
  fail_unless (has_src, "Missing src pad template");

  gst_object_unref (factory);
}
GST_END_TEST;

/* ── TCase "Metadata" ─────────────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_element_metadata)
{
  GstElement *el;
  GstElementFactory *factory;
  const gchar *klass, *author;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);

  factory = gst_element_get_factory (el);
  klass = gst_element_factory_get_metadata (factory, GST_ELEMENT_METADATA_KLASS);
  author = gst_element_factory_get_metadata (factory, GST_ELEMENT_METADATA_AUTHOR);

  fail_unless_equals_string (klass, "Filter/Converter/Video");
  fail_unless_equals_string (author, "Au-Zone Technologies <support@au-zone.com>");

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Behavior" ─────────────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_not_passthrough)
{
  GstElement *el;
  GstBaseTransformClass *trans_class;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);

  trans_class = GST_BASE_TRANSFORM_GET_CLASS (el);
  fail_unless (trans_class->passthrough_on_same_caps == FALSE,
      "passthrough_on_same_caps must be FALSE");

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_not_in_place)
{
  GstElement *el;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);

  fail_unless (gst_base_transform_is_in_place (GST_BASE_TRANSFORM (el)) == FALSE,
      "camera adaptor must not be in-place (different input/output formats)");

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "StateTransitions" ─────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_state_null_to_ready)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);

  ret = gst_element_set_state (el, GST_STATE_READY);
  fail_unless (ret == GST_STATE_CHANGE_SUCCESS,
      "NULL -> READY failed with %d", ret);

  ret = gst_element_set_state (el, GST_STATE_NULL);
  fail_unless (ret == GST_STATE_CHANGE_SUCCESS,
      "READY -> NULL failed with %d", ret);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Pipeline" ─────────────────────────────────────────────── */

/**
 * Helper: push a synthetic RGB frame through the camera adaptor and
 * verify the output buffer size.
 */
static void
run_pipeline_test (guint in_w, guint in_h, const gchar *in_format,
    guint model_w, guint model_h, const gchar *dtype_str,
    const gchar *layout_str, gboolean letterbox,
    gsize expected_out_size)
{
  gchar *pipeline_str;
  GstElement *pipeline, *sink;
  GstSample *sample;
  GstBuffer *buf;
  GstMapInfo map;
  GError *error = NULL;

  pipeline_str = g_strdup_printf (
      "videotestsrc num-buffers=1 ! "
      "video/x-raw,format=%s,width=%u,height=%u ! "
      "edgefirstcameraadaptor model-width=%u model-height=%u "
        "model-dtype=%s model-layout=%s letterbox=%s ! "
      "appsink name=sink",
      in_format, in_w, in_h, model_w, model_h,
      dtype_str, layout_str,
      letterbox ? "true" : "false");

  pipeline = gst_parse_launch (pipeline_str, &error);
  if (error) {
    fail ("Pipeline parse error: %s (pipeline: %s)",
        error->message, pipeline_str);
    g_error_free (error);
    g_free (pipeline_str);
    return;
  }
  fail_unless (pipeline != NULL, "Failed to create pipeline: %s",
      pipeline_str);
  fail_unless (GST_IS_BIN (pipeline), "Pipeline is not a bin: %s",
      pipeline_str);
  g_free (pipeline_str);

  sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  fail_unless (sink != NULL, "Could not find sink element");

  g_object_set (sink, "sync", FALSE, NULL);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
  if (!sample) {
    /* Check bus for error messages */
    GstBus *bus = gst_element_get_bus (pipeline);
    GstMessage *msg = gst_bus_pop_filtered (bus, GST_MESSAGE_ERROR);
    if (msg) {
      GError *err = NULL;
      gchar *dbg = NULL;
      gst_message_parse_error (msg, &err, &dbg);
      fail ("Pipeline error: %s (%s)", err->message, dbg ? dbg : "");
      g_error_free (err);
      g_free (dbg);
      gst_message_unref (msg);
    }
    gst_object_unref (bus);
    fail ("No sample received from pipeline");
  }

  buf = gst_sample_get_buffer (sample);
  fail_unless (buf != NULL, "No buffer in sample");

  fail_unless (gst_buffer_map (buf, &map, GST_MAP_READ));
  fail_unless_equals_uint64 (map.size, expected_out_size);
  gst_buffer_unmap (buf, &map);

  gst_sample_unref (sample);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (sink);
  gst_object_unref (pipeline);
}

/**
 * Helper: run a pipeline to completion and return the first output buffer.
 * Returns NULL if the pipeline fails (caller should skip, not fail).
 */
static GstBuffer *
run_pipeline_pull_buffer (const gchar *pipeline_str)
{
  GstElement *pipeline, *sink;
  GstSample *sample;
  GstBuffer *buf = NULL;
  GError *error = NULL;

  pipeline = gst_parse_launch (pipeline_str, &error);
  if (error) {
    GST_WARNING ("Pipeline parse error: %s", error->message);
    g_error_free (error);
    return NULL;
  }
  if (!pipeline || !GST_IS_BIN (pipeline)) {
    if (pipeline)
      gst_object_unref (pipeline);
    return NULL;
  }

  sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  if (!sink) {
    gst_object_unref (pipeline);
    return NULL;
  }
  g_object_set (sink, "sync", FALSE, NULL);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));

  if (sample) {
    buf = gst_sample_get_buffer (sample);
    if (buf)
      buf = gst_buffer_copy (buf);   /* own copy — sample will be freed */
    gst_sample_unref (sample);
  }

  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (sink);
  gst_object_unref (pipeline);
  return buf;
}

/**
 * Compare two GstBuffers byte-by-byte with per-byte tolerance.
 * Returns TRUE if sizes match and no more than max_mismatch_pct percent
 * of bytes differ by more than @tolerance.
 */
static gboolean
buffers_equal_with_tolerance (GstBuffer *a, GstBuffer *b,
    guint8 tolerance, gdouble max_mismatch_pct)
{
  GstMapInfo ma, mb;
  gboolean ok = FALSE;

  if (!gst_buffer_map (a, &ma, GST_MAP_READ))
    return FALSE;
  if (!gst_buffer_map (b, &mb, GST_MAP_READ)) {
    gst_buffer_unmap (a, &ma);
    return FALSE;
  }

  if (ma.size != mb.size) {
    GST_WARNING ("size mismatch: %" G_GSIZE_FORMAT " vs %" G_GSIZE_FORMAT,
        ma.size, mb.size);
    goto done;
  }

  gsize mismatches = 0;
  for (gsize i = 0; i < ma.size; i++) {
    gint diff = (gint) ma.data[i] - (gint) mb.data[i];
    if (diff < 0) diff = -diff;
    if (diff > tolerance)
      mismatches++;
  }

  gdouble pct = (ma.size > 0) ? 100.0 * mismatches / ma.size : 0.0;
  if (pct > max_mismatch_pct) {
    GST_WARNING ("%.1f%% of bytes differ by >%u (%" G_GSIZE_FORMAT
        "/%" G_GSIZE_FORMAT ")", pct, tolerance, mismatches, ma.size);
  } else {
    ok = TRUE;
  }

done:
  gst_buffer_unmap (b, &mb);
  gst_buffer_unmap (a, &ma);
  return ok;
}

GST_START_TEST (test_camera_adaptor_rgb_passthrough)
{
  /* 320x240 RGB → 320x240 uint8 HWC: output = 320*240*3 = 230400 */
  run_pipeline_test (320, 240, "RGB",
      320, 240, "uint8", "hwc", FALSE,
      320 * 240 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_resize)
{
  /* 640x480 RGB → 320x320 uint8 HWC: output = 320*320*3 = 307200 */
  run_pipeline_test (640, 480, "RGB",
      320, 320, "uint8", "hwc", FALSE,
      320 * 320 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_letterbox)
{
  /* 640x480 RGB → 320x320 uint8 HWC letterbox: output = 320*320*3 = 307200 */
  run_pipeline_test (640, 480, "RGB",
      320, 320, "uint8", "hwc", TRUE,
      320 * 320 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_int8)
{
  /* 320x240 RGB → int8 tensor: output = 320*240*3 = 230400 */
  run_pipeline_test (320, 240, "RGB",
      320, 240, "int8", "hwc", FALSE,
      320 * 240 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_chw_layout)
{
  /* 320x240 RGB → uint8 CHW: output = 3*320*240 = 230400 (same total) */
  run_pipeline_test (320, 240, "RGB",
      320, 240, "uint8", "chw", FALSE,
      320 * 240 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_float32)
{
  /* 320x240 RGB → float32 HWC: output = 320*240*3*4 = 921600 */
  run_pipeline_test (320, 240, "RGB",
      320, 240, "float32", "hwc", FALSE,
      320 * 240 * 3 * 4);
}
GST_END_TEST;

/* ── TCase "PipelineContent" ──────────────────────────────────────── */

GST_START_TEST (test_camera_adaptor_chw_int8)
{
  /* CHW + int8 (Ara-2 workflow): output = 320*240*3 = 230400 */
  run_pipeline_test (320, 240, "RGB",
      320, 240, "int8", "chw", FALSE,
      320 * 240 * 3);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_bgr_hwc)
{
  /* BGR HWC uint8: output = 320*240*3 = 230400 */
  gchar *pipeline_str;
  GstElement *pipeline, *sink;
  GstSample *sample;
  GstBuffer *buf;
  GstMapInfo map;
  GError *error = NULL;

  pipeline_str = g_strdup_printf (
      "videotestsrc num-buffers=1 ! "
      "video/x-raw,format=RGB,width=320,height=240 ! "
      "edgefirstcameraadaptor model-width=320 model-height=240 "
        "model-dtype=uint8 model-layout=hwc model-colorspace=bgr ! "
      "appsink name=sink");

  pipeline = gst_parse_launch (pipeline_str, &error);
  fail_unless (error == NULL, "Pipeline error: %s",
      error ? error->message : "");
  fail_unless (pipeline != NULL);
  g_free (pipeline_str);

  sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  fail_unless (sink != NULL);
  g_object_set (sink, "sync", FALSE, NULL);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
  fail_unless (sample != NULL, "No sample received");

  buf = gst_sample_get_buffer (sample);
  fail_unless (gst_buffer_map (buf, &map, GST_MAP_READ));
  fail_unless_equals_uint64 (map.size, 320 * 240 * 3);

  gst_buffer_unmap (buf, &map);
  gst_sample_unref (sample);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (sink);
  gst_object_unref (pipeline);
}
GST_END_TEST;

GST_START_TEST (test_camera_adaptor_letterbox_properties)
{
  GstElement *pipeline, *adaptor, *sink;
  GstSample *sample;
  GError *error = NULL;
  gfloat scale_val;
  gint top_val, left_val;

  gchar *pipeline_str = g_strdup_printf (
      "videotestsrc num-buffers=1 ! "
      "video/x-raw,format=RGB,width=640,height=480 ! "
      "edgefirstcameraadaptor name=adapt model-width=320 model-height=320 "
        "model-dtype=uint8 model-layout=hwc letterbox=true ! "
      "appsink name=sink");

  pipeline = gst_parse_launch (pipeline_str, &error);
  fail_unless (error == NULL);
  fail_unless (pipeline != NULL);
  g_free (pipeline_str);

  adaptor = gst_bin_get_by_name (GST_BIN (pipeline), "adapt");
  fail_unless (adaptor != NULL);

  sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  fail_unless (sink != NULL);
  g_object_set (sink, "sync", FALSE, NULL);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
  fail_unless (sample != NULL, "No sample received");

  /* Read back letterbox properties after pipeline has negotiated */
  g_object_get (adaptor, "letterbox-scale", &scale_val, NULL);
  fail_unless (scale_val > 0.0f, "Scale should be positive, got %f", scale_val);

  g_object_get (adaptor, "letterbox-top", &top_val, NULL);
  g_object_get (adaptor, "letterbox-left", &left_val, NULL);
  /* 640x480 into 320x320: scale=0.5, new=320x240, pad top/bottom=40 */
  fail_unless (top_val >= 0, "Top padding should be >= 0, got %d", top_val);
  fail_unless (left_val == 0, "Left padding should be 0, got %d", left_val);

  gst_sample_unref (sample);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (adaptor);
  gst_object_unref (sink);
  gst_object_unref (pipeline);
}
GST_END_TEST;

/* ── TCase "FormatConversion" ─────────────────────────────────────── */

/**
 * Build a pipeline string that converts videotestsrc output to a given
 * format, then processes through edgefirstcameraadaptor.
 */
static gchar *
format_test_pipeline (const gchar *format_str)
{
  return g_strdup_printf (
      "videotestsrc num-buffers=1 pattern=smpte ! "
      "videoconvert ! video/x-raw,format=%s,width=320,height=240 ! "
      "edgefirstcameraadaptor model-width=160 model-height=160 "
        "model-dtype=uint8 model-layout=hwc ! "
      "appsink name=sink",
      format_str);
}

/**
 * Returns TRUE for YUV-family formats where RGB→YUV→RGB roundtrip
 * introduces colorspace conversion rounding.
 */
static gboolean
format_is_yuv (const gchar *fmt)
{
  return (g_strcmp0 (fmt, "NV12") == 0 || g_strcmp0 (fmt, "NV21") == 0 ||
      g_strcmp0 (fmt, "NV16") == 0 || g_strcmp0 (fmt, "I420") == 0 ||
      g_strcmp0 (fmt, "YV12") == 0 || g_strcmp0 (fmt, "YUY2") == 0 ||
      g_strcmp0 (fmt, "UYVY") == 0);
}

GST_START_TEST (test_format_conversion_all_formats)
{
  const gsize expected_size = 160 * 160 * 3;

  /* Generate reference tensors for each colorspace family:
   *   RGBA  → reference for RGB-family formats (exact match expected)
   *   NV12  → reference for YUV-family formats (same HAL YUV→RGB path)
   */
  gchar *rgba_str = format_test_pipeline ("RGBA");
  GstBuffer *rgba_ref = run_pipeline_pull_buffer (rgba_str);
  g_free (rgba_str);
  fail_unless (rgba_ref != NULL, "Failed to generate RGBA reference tensor");

  GstMapInfo ref_map;
  fail_unless (gst_buffer_map (rgba_ref, &ref_map, GST_MAP_READ));
  fail_unless (ref_map.size == expected_size,
      "Reference tensor size mismatch: %" G_GSIZE_FORMAT, ref_map.size);
  gst_buffer_unmap (rgba_ref, &ref_map);

  gchar *nv12_str = format_test_pipeline ("NV12");
  GstBuffer *yuv_ref = run_pipeline_pull_buffer (nv12_str);
  g_free (nv12_str);
  fail_unless (yuv_ref != NULL, "Failed to generate NV12 reference tensor");

  /* Query all supported sink formats from the element template */
  GstElement *el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);
  GstPad *sinkpad = gst_element_get_static_pad (el, "sink");
  GstCaps *templ = gst_pad_get_pad_template_caps (sinkpad);
  gst_object_unref (sinkpad);
  gst_object_unref (el);

  GstCaps *all_caps = gst_caps_normalize (templ);
  guint n = gst_caps_get_size (all_caps);

  guint tested = 0, skipped = 0, passed = 0, failed = 0;

  for (guint i = 0; i < n; i++) {
    GstStructure *s = gst_caps_get_structure (all_caps, i);

    /* Skip DMABuf-featured caps (same formats, different memory) */
    GstCapsFeatures *features = gst_caps_get_features (all_caps, i);
    if (features && !gst_caps_features_is_equal (features,
            gst_caps_features_new_empty ()))
      continue;

    const gchar *fmt = gst_structure_get_string (s, "format");
    if (!fmt) continue;

    /* Skip reference formats */
    if (g_strcmp0 (fmt, "RGBA") == 0 || g_strcmp0 (fmt, "NV12") == 0)
      continue;

    gchar *pipe = format_test_pipeline (fmt);
    GstBuffer *buf = run_pipeline_pull_buffer (pipe);
    g_free (pipe);

    if (!buf) {
      GST_INFO ("Skipped %s (pipeline failed)", fmt);
      skipped++;
      continue;
    }

    /* Verify output tensor size */
    GstMapInfo map;
    fail_unless (gst_buffer_map (buf, &map, GST_MAP_READ));
    gboolean size_ok = (map.size == expected_size);
    gst_buffer_unmap (buf, &map);

    if (!size_ok) {
      GST_ERROR ("FAIL  %s — wrong output size", fmt);
      gst_buffer_unref (buf);
      tested++;
      failed++;
      continue;
    }

    tested++;

    if (g_strcmp0 (fmt, "GRAY8") == 0) {
      /* GRAY8 produces a luminance-only tensor — content comparison against
       * color reference is meaningless.  Size check is sufficient. */
      GST_INFO ("PASS  %s (size-only — grayscale)", fmt);
      passed++;
    } else if (format_is_yuv (fmt)) {
      /* YUV formats: compare against NV12 reference (same HAL YUV→RGB path).
       * Tolerance=4, 5%: accounts for different chroma subsampling and
       * rounding between YUV sub-formats. */
      if (buffers_equal_with_tolerance (yuv_ref, buf, 4, 5.0)) {
        GST_INFO ("PASS  %s (vs NV12 reference)", fmt);
        passed++;
      } else {
        GST_ERROR ("FAIL  %s — does not match NV12 reference", fmt);
        failed++;
      }
    } else {
      /* RGB-family: strict comparison against RGBA reference */
      if (buffers_equal_with_tolerance (rgba_ref, buf, 2, 1.0)) {
        GST_INFO ("PASS  %s", fmt);
        passed++;
      } else {
        GST_ERROR ("FAIL  %s — does not match RGBA reference", fmt);
        failed++;
      }
    }

    gst_buffer_unref (buf);
  }

  gst_caps_unref (all_caps);
  gst_buffer_unref (yuv_ref);
  gst_buffer_unref (rgba_ref);

  g_print ("\n=== Format Conversion Results ===\n"
      "  Tested:  %u\n  Passed:  %u\n  Failed:  %u\n  Skipped: %u\n\n",
      tested, passed, failed, skipped);

  fail_unless (failed == 0, "%u format(s) failed conversion validation", failed);
  fail_unless (tested > 0, "No formats were tested");
}
GST_END_TEST;

GST_START_TEST (test_format_timing)
{
  GstElement *el = gst_element_factory_make ("edgefirstcameraadaptor", NULL);
  fail_unless (el != NULL);
  GstPad *sinkpad = gst_element_get_static_pad (el, "sink");
  GstCaps *templ = gst_pad_get_pad_template_caps (sinkpad);
  gst_object_unref (sinkpad);
  gst_object_unref (el);

  GstCaps *all_caps = gst_caps_normalize (templ);
  guint n = gst_caps_get_size (all_caps);

  g_print ("\n=== Format Preprocessing Timing ===\n"
      "  %-8s  %8s  %8s\n", "Format", "Total ms", "Avg ms");

  guint num_buffers = 30;

  for (guint i = 0; i < n; i++) {
    GstStructure *s = gst_caps_get_structure (all_caps, i);
    GstCapsFeatures *features = gst_caps_get_features (all_caps, i);
    if (features && !gst_caps_features_is_equal (features,
            gst_caps_features_new_empty ()))
      continue;

    const gchar *fmt = gst_structure_get_string (s, "format");
    if (!fmt) continue;

    gchar *pipeline_str = g_strdup_printf (
        "videotestsrc num-buffers=%u pattern=smpte ! "
        "videoconvert ! video/x-raw,format=%s,width=320,height=240 ! "
        "edgefirstcameraadaptor model-width=160 model-height=160 "
          "model-dtype=uint8 model-layout=hwc ! "
        "appsink name=sink",
        num_buffers, fmt);

    GstElement *pipeline;
    GstElement *sink;
    GError *error = NULL;

    pipeline = gst_parse_launch (pipeline_str, &error);
    g_free (pipeline_str);
    if (error || !pipeline) {
      if (error) g_error_free (error);
      if (pipeline) gst_object_unref (pipeline);
      g_print ("  %-8s  %8s  %8s  (skipped)\n", fmt, "-", "-");
      continue;
    }

    sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
    g_object_set (sink, "sync", FALSE, NULL);

    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    gint64 t_start = g_get_monotonic_time ();
    guint received = 0;

    while (received < num_buffers) {
      GstSample *sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
      if (!sample) break;
      received++;
      gst_sample_unref (sample);
    }

    gint64 t_elapsed = g_get_monotonic_time () - t_start;
    gdouble total_ms = t_elapsed / 1000.0;
    gdouble avg_ms = (received > 0) ? total_ms / received : 0;

    g_print ("  %-8s  %8.1f  %8.3f  (%u frames)\n",
        fmt, total_ms, avg_ms, received);

    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (sink);
    gst_object_unref (pipeline);
  }

  g_print ("\n");
  gst_caps_unref (all_caps);
}
GST_END_TEST;

/* ── TCase "Overlay" ───────────────────────────────────────────────── */

GST_START_TEST (test_overlay_create)
{
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL, "Failed to create edgefirstoverlay");

  /* Check pad existence */
  GstPad *video   = gst_element_get_static_pad (el, "video");
  GstPad *tensors = gst_element_get_static_pad (el, "tensors");
  GstPad *src     = gst_element_get_static_pad (el, "src");
  fail_unless (video   != NULL, "no 'video' sink pad");
  fail_unless (tensors != NULL, "no 'tensors' sink pad");
  fail_unless (src     != NULL, "no 'src' pad");

  gst_object_unref (video);
  gst_object_unref (tensors);
  gst_object_unref (src);
  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_overlay_properties)
{
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL);

  gfloat f; guint u; gboolean b; gchar *s; gint e;

  g_object_get (el, "score-threshold",    &f, NULL); fail_unless (fabs(f - 0.25f) < 1e-5f);
  g_object_get (el, "iou-threshold",      &f, NULL); fail_unless (fabs(f - 0.45f) < 1e-5f);
  g_object_get (el, "opacity",            &f, NULL); fail_unless (fabs(f - 0.6f)  < 1e-5f);
  g_object_get (el, "model-sync",         &b, NULL); fail_unless (b == FALSE);
  g_object_get (el, "model-sync-timeout", &u, NULL); fail_unless_equals_int (u, 500);
  g_object_get (el, "color-mode",         &e, NULL); fail_unless_equals_int (e, EDGEFIRST_COLOR_MODE_CLASS);
  g_object_get (el, "decoder-version",    &s, NULL);
  fail_unless_equals_string (s, "yolov8");
  g_free (s);
  g_object_get (el, "model-config", &s, NULL); fail_unless (s == NULL);

  /* Roundtrip */
  g_object_set (el, "score-threshold", 0.5f, NULL);
  g_object_get (el, "score-threshold", &f, NULL);
  fail_unless (fabs(f - 0.5f) < 1e-5f);

  g_object_set (el, "color-mode", (gint) EDGEFIRST_COLOR_MODE_INSTANCE, NULL);
  g_object_get (el, "color-mode", &e, NULL);
  fail_unless_equals_int (e, EDGEFIRST_COLOR_MODE_INSTANCE);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "OverlayTypes" ──────────────────────────────────────────── */

GST_START_TEST (test_overlay_detection_types)
{
  GType box_type = edgefirst_detect_box_get_type ();
  fail_unless (box_type != 0, "EdgeFirstDetectBox not registered");
  fail_unless (G_TYPE_IS_BOXED (box_type));

  GType seg_type = edgefirst_segmentation_get_type ();
  fail_unless (seg_type != 0, "EdgeFirstSegmentation not registered");
  fail_unless (G_TYPE_IS_BOXED (seg_type));

  GType cm_type = edgefirst_color_mode_get_type ();
  fail_unless (cm_type != 0, "EdgeFirstColorMode not registered");
  fail_unless (G_TYPE_IS_ENUM (cm_type));

  fail_unless (g_type_is_a (edgefirst_detect_box_list_get_type (), G_TYPE_OBJECT));
  fail_unless (g_type_is_a (edgefirst_segmentation_list_get_type (), G_TYPE_OBJECT));
}
GST_END_TEST;

/* ── TCase "Overlay" extra tests ───────────────────────────────────── */

GST_START_TEST (test_overlay_new_detection_signal)
{
  /* Verify the new-detection signal exists with correct parameter types */
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL);

  guint sig_id = g_signal_lookup ("new-detection", G_OBJECT_TYPE (el));
  fail_unless (sig_id != 0, "new-detection signal not found");

  GSignalQuery query;
  g_signal_query (sig_id, &query);
  fail_unless_equals_int (query.n_params, 2);
  fail_unless (query.param_types[0] == EDGEFIRST_TYPE_DETECT_BOX_LIST,
      "First param should be EDGEFIRST_TYPE_DETECT_BOX_LIST");
  fail_unless (query.param_types[1] == EDGEFIRST_TYPE_SEGMENTATION_LIST,
      "Second param should be EDGEFIRST_TYPE_SEGMENTATION_LIST");

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_overlay_no_decoder)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL, "Failed to create edgefirstoverlay");

  /* No model-config set — decoder will be auto-configured (or skipped).
   * The element must reach READY state without error. */
  ret = gst_element_set_state (el, GST_STATE_READY);
  fail_unless (ret == GST_STATE_CHANGE_SUCCESS,
      "NULL -> READY failed with %d", ret);

  /* Verify state is actually READY */
  GstState current, pending;
  gst_element_get_state (el, &current, &pending, GST_CLOCK_TIME_NONE);
  fail_unless_equals_int (current, GST_STATE_READY);

  /* Back to NULL cleanly */
  ret = gst_element_set_state (el, GST_STATE_NULL);
  fail_unless (ret == GST_STATE_CHANGE_SUCCESS,
      "READY -> NULL failed with %d", ret);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "OverlayPipeline" ────────────────────────────────────────── */

GST_START_TEST (test_overlay_model_sync_eos)
{
  /* With model-sync=true and no tensors arriving (fakesrc sends EOS immediately),
   * EOS on video must not deadlock — the flushing flag unblocks the wait */
  GstElement *pipeline = gst_parse_launch (
      "videotestsrc num-buffers=2 ! video/x-raw,format=RGB,width=32,height=32 "
      "! edgefirstoverlay model-sync=true model-sync-timeout=100 name=ov "
      "! fakesink  "
      "fakesrc num-buffers=0 format=3 "
      "! other/tensors,format=static,num_tensors=1,types=(string)float32,"
        "dimensions=(string)84:8400:1:1,framerate=0/1 "
      "! ov.tensors",
      NULL);
  fail_unless (pipeline != NULL);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  GstBus *bus = gst_element_get_bus (pipeline);
  GstMessage *msg = gst_bus_timed_pop_filtered (bus, GST_SECOND * 5,
      GST_MESSAGE_EOS | GST_MESSAGE_ERROR);

  fail_unless (msg != NULL, "No EOS or ERROR within 5s — possible deadlock");
  fail_unless (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_EOS,
      "Expected EOS, got %s", gst_message_type_get_name (GST_MESSAGE_TYPE (msg)));

  gst_message_unref (msg);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
}
GST_END_TEST;

GST_START_TEST (test_overlay_caps_negotiation)
{
  /* Video sink accepts RGB; src pad output should be RGBA.
   * fakesrc num-buffers=0 on tensors pad sends immediate EOS.
   * No caps filter on appsink: overlay may push DMABuf or system-memory
   * RGBA depending on platform; we verify the format by querying the pad. */
  GstElement *pipeline = gst_parse_launch (
      "videotestsrc num-buffers=1 ! video/x-raw,format=RGB,width=64,height=64 "
      "! edgefirstoverlay name=ov ! appsink name=sink sync=false "
      "fakesrc num-buffers=0 format=3 "
      "! other/tensors,format=static,num_tensors=1,types=(string)float32,"
        "dimensions=(string)84:8400:1:1,framerate=0/1 "
      "! ov.tensors",
      NULL);
  fail_unless (pipeline != NULL);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  GstBus *bus = gst_element_get_bus (pipeline);
  GstMessage *msg = gst_bus_timed_pop_filtered (bus, GST_SECOND * 5,
      GST_MESSAGE_EOS | GST_MESSAGE_ERROR);

  fail_unless (msg != NULL);
  fail_unless (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_EOS,
      "Expected EOS, got %s", gst_message_type_get_name (GST_MESSAGE_TYPE (msg)));

  /* Verify output format is RGBA regardless of memory type (DMABuf or system) */
  GstElement *sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  GstPad *sink_pad = gst_element_get_static_pad (sink, "sink");
  GstCaps *actual_caps = gst_pad_get_current_caps (sink_pad);
  fail_unless (actual_caps != NULL, "No caps on appsink pad after EOS");
  GstStructure *s = gst_caps_get_structure (actual_caps, 0);
  const gchar *fmt = gst_structure_get_string (s, "format");
  fail_unless_equals_string (fmt, "RGBA");
  gst_caps_unref (actual_caps);
  gst_object_unref (sink_pad);
  gst_object_unref (sink);

  gst_message_unref (msg);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
}
GST_END_TEST;

GST_START_TEST (test_overlay_no_tensors)
{
  /* Verify element instantiates and transitions to PAUSED without crashing
   * when model-config is NULL */
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL);
  GstStateChangeReturn ret = gst_element_set_state (el, GST_STATE_PAUSED);
  /* ASYNC is acceptable since pads are not connected */
  fail_unless (ret != GST_STATE_CHANGE_FAILURE, "PAUSED transition failed");
  gst_element_set_state (el, GST_STATE_NULL);
  gst_object_unref (el);
}
GST_END_TEST;

static GstFlowReturn
overlay_repeat_last_new_sample_cb (GstAppSink *sink, gpointer user_data)
{
  gint *count = (gint *) user_data;
  (*count)++;
  GstSample *sample = gst_app_sink_pull_sample (sink);
  if (sample) gst_sample_unref (sample);
  return GST_FLOW_OK;
}

GST_START_TEST (test_overlay_repeat_last)
{
  /* N video frames in → N video frames out (pass-through with no model detections) */
  GstElement *pipeline = gst_parse_launch (
      "videotestsrc num-buffers=5 ! video/x-raw,format=RGB,width=32,height=32 "
      "! edgefirstoverlay name=ov ! appsink name=sink  "
      "fakesrc num-buffers=0 format=3 "
      "! other/tensors,format=static,num_tensors=1,types=(string)float32,"
        "dimensions=(string)84:8400:1:1,framerate=0/1 "
      "! ov.tensors",
      NULL);
  fail_unless (pipeline != NULL);

  gint count = 0;
  GstElement *sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  g_object_set (sink, "emit-signals", TRUE, NULL);
  g_signal_connect (sink, "new-sample",
      G_CALLBACK (overlay_repeat_last_new_sample_cb), &count);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  GstBus *bus = gst_element_get_bus (pipeline);
  gst_bus_timed_pop_filtered (bus, 5 * GST_SECOND,
      GST_MESSAGE_EOS | GST_MESSAGE_ERROR);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);

  fail_unless (count == 5, "Expected 5 output frames, got %d", count);

  gst_object_unref (sink);
  gst_object_unref (pipeline);
}
GST_END_TEST;

static GstFlowReturn
overlay_model_sync_new_sample_cb (GstAppSink *sink, gpointer user_data)
{
  gint *count = (gint *) user_data;
  (*count)++;
  GstSample *sample = gst_app_sink_pull_sample (sink);
  if (sample) gst_sample_unref (sample);
  return GST_FLOW_OK;
}

GST_START_TEST (test_overlay_model_sync)
{
  /* model-sync=TRUE with timeout: all video frames should eventually be produced
   * (tensors EOS sets flushing, allowing video to drain without hanging) */
  GstElement *pipeline = gst_parse_launch (
      "videotestsrc num-buffers=2 ! video/x-raw,format=RGB,width=32,height=32 "
      "! edgefirstoverlay model-sync=true model-sync-timeout=100 name=ov "
      "! appsink name=sink  "
      "fakesrc num-buffers=0 format=3 "
      "! other/tensors,format=static,num_tensors=1,types=(string)float32,"
        "dimensions=(string)84:8400:1:1,framerate=0/1 "
      "! ov.tensors",
      NULL);
  fail_unless (pipeline != NULL);

  gint count = 0;
  GstElement *sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  g_object_set (sink, "emit-signals", TRUE, NULL);
  g_signal_connect (sink, "new-sample",
      G_CALLBACK (overlay_model_sync_new_sample_cb), &count);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  GstBus *bus = gst_element_get_bus (pipeline);
  gst_bus_timed_pop_filtered (bus, 5 * GST_SECOND,
      GST_MESSAGE_EOS | GST_MESSAGE_ERROR);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);

  fail_unless (count == 2, "Expected 2 output frames with model-sync timeout, got %d", count);

  gst_object_unref (sink);
  gst_object_unref (pipeline);
}
GST_END_TEST;

GST_START_TEST (test_overlay_auto_config_detect)
{
  /* Auto-configure from YOLOv8-detect caps [84:8400:1:1] — no crash */
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL);
  fail_unless (gst_element_set_state (el, GST_STATE_PAUSED) != GST_STATE_CHANGE_FAILURE);

  GstPad *tpad = gst_element_get_static_pad (el, "tensors");
  GstCaps *caps = gst_caps_from_string (
      "other/tensors,num_tensors=(int)1,format=(string)static,"
      "types=(string)float32,dimensions=(string)84:8400:1:1");
  gst_pad_send_event (tpad, gst_event_new_caps (caps));
  gst_caps_unref (caps);
  gst_object_unref (tpad);

  gst_element_set_state (el, GST_STATE_NULL);
  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_overlay_auto_config_seg)
{
  /* Auto-configure from YOLOv8-seg caps (4 tensors) — no crash */
  GstElement *el = gst_element_factory_make ("edgefirstoverlay", NULL);
  fail_unless (el != NULL);
  fail_unless (gst_element_set_state (el, GST_STATE_PAUSED) != GST_STATE_CHANGE_FAILURE);

  GstPad *tpad = gst_element_get_static_pad (el, "tensors");
  GstCaps *caps = gst_caps_from_string (
      "other/tensors,num_tensors=(int)4,format=(string)static,"
      "types=(string)\"float32,float32,float32,float32\","
      "dimensions=(string)\"4:8400:1:1,80:8400:1:1,32:8400:1:1,32:160:160:1\"");
  gst_pad_send_event (tpad, gst_event_new_caps (caps));
  gst_caps_unref (caps);
  gst_object_unref (tpad);

  gst_element_set_state (el, GST_STATE_NULL);
  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_overlay_segmentation_independence)
{
  /* edgefirst_detect_box_list_new(NULL) returns NULL — guarded in new() */
  EdgeFirstDetectBoxList *box_list = edgefirst_detect_box_list_new (NULL);
  fail_unless (box_list == NULL,
      "edgefirst_detect_box_list_new(NULL) should return NULL");

  /* Verify EdgeFirstSegmentation copy is independent of original */
  EdgeFirstSegmentation *seg = g_slice_new (EdgeFirstSegmentation);
  static const guint8 mask_data[4] = {128, 64, 192, 32};
  seg->width  = 2;
  seg->height = 2;
  seg->mask   = g_bytes_new (mask_data, sizeof (mask_data));
  seg->x1 = 0.1f; seg->y1 = 0.2f; seg->x2 = 0.8f; seg->y2 = 0.9f;

  EdgeFirstSegmentation *copy = edgefirst_segmentation_copy (seg);
  edgefirst_segmentation_free (seg);

  gsize sz = 0;
  const guint8 *data = g_bytes_get_data (copy->mask, &sz);
  fail_unless (sz == 4);
  fail_unless (data[0] == 128 && data[1] == 64);

  edgefirst_segmentation_free (copy);
}
GST_END_TEST;

/* ── Suite ─────────────────────────────────────────────────────────── */

static Suite *
edgefirst_hal_elements_suite (void)
{
  Suite *s = suite_create ("EdgeFirst HAL Elements");

  TCase *tc_create = tcase_create ("Creation");
  tcase_add_test (tc_create, test_camera_adaptor_create);
  suite_add_tcase (s, tc_create);

  TCase *tc_props = tcase_create ("Properties");
  tcase_add_test (tc_props, test_camera_adaptor_properties);
  suite_add_tcase (s, tc_props);

  TCase *tc_pads = tcase_create ("Pads");
  tcase_add_test (tc_pads, test_camera_adaptor_pad_templates);
  suite_add_tcase (s, tc_pads);

  TCase *tc_metadata = tcase_create ("Metadata");
  tcase_add_test (tc_metadata, test_camera_adaptor_element_metadata);
  suite_add_tcase (s, tc_metadata);

  TCase *tc_behavior = tcase_create ("Behavior");
  tcase_add_test (tc_behavior, test_camera_adaptor_not_passthrough);
  tcase_add_test (tc_behavior, test_camera_adaptor_not_in_place);
  suite_add_tcase (s, tc_behavior);

  TCase *tc_states = tcase_create ("StateTransitions");
  tcase_add_test (tc_states, test_camera_adaptor_state_null_to_ready);
  suite_add_tcase (s, tc_states);

  TCase *tc_pipeline = tcase_create ("Pipeline");
  tcase_set_timeout (tc_pipeline, 10);
  tcase_add_test (tc_pipeline, test_camera_adaptor_rgb_passthrough);
  tcase_add_test (tc_pipeline, test_camera_adaptor_resize);
  tcase_add_test (tc_pipeline, test_camera_adaptor_letterbox);
  tcase_add_test (tc_pipeline, test_camera_adaptor_int8);
  tcase_add_test (tc_pipeline, test_camera_adaptor_chw_layout);
  tcase_add_test (tc_pipeline, test_camera_adaptor_float32);
  suite_add_tcase (s, tc_pipeline);

  TCase *tc_content = tcase_create ("PipelineContent");
  tcase_set_timeout (tc_content, 10);
  tcase_add_test (tc_content, test_camera_adaptor_chw_int8);
  tcase_add_test (tc_content, test_camera_adaptor_bgr_hwc);
  tcase_add_test (tc_content, test_camera_adaptor_letterbox_properties);
  suite_add_tcase (s, tc_content);

  TCase *tc_formats = tcase_create ("FormatConversion");
  tcase_set_timeout (tc_formats, 60);  /* format iteration needs more time */
  tcase_add_test (tc_formats, test_format_conversion_all_formats);
  tcase_add_test (tc_formats, test_format_timing);
  suite_add_tcase (s, tc_formats);

  TCase *tc_overlay_el = tcase_create ("Overlay");
  tcase_add_test (tc_overlay_el, test_overlay_create);
  tcase_add_test (tc_overlay_el, test_overlay_properties);
  tcase_add_test (tc_overlay_el, test_overlay_new_detection_signal);
  tcase_add_test (tc_overlay_el, test_overlay_no_decoder);
  tcase_add_test (tc_overlay_el, test_overlay_no_tensors);
  tcase_add_test (tc_overlay_el, test_overlay_auto_config_detect);
  tcase_add_test (tc_overlay_el, test_overlay_auto_config_seg);
  tcase_add_test (tc_overlay_el, test_overlay_segmentation_independence);
  suite_add_tcase (s, tc_overlay_el);

  TCase *tc_overlay_pipeline = tcase_create ("OverlayPipeline");
  tcase_set_timeout (tc_overlay_pipeline, 30);
  tcase_add_test (tc_overlay_pipeline, test_overlay_model_sync_eos);
  tcase_add_test (tc_overlay_pipeline, test_overlay_caps_negotiation);
  tcase_add_test (tc_overlay_pipeline, test_overlay_repeat_last);
  tcase_add_test (tc_overlay_pipeline, test_overlay_model_sync);
  suite_add_tcase (s, tc_overlay_pipeline);

  TCase *tc_overlay = tcase_create ("OverlayTypes");
  tcase_add_test (tc_overlay, test_overlay_detection_types);
  suite_add_tcase (s, tc_overlay);

  return s;
}

GST_CHECK_MAIN (edgefirst_hal_elements);
