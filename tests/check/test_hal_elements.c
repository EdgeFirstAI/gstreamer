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
#include "edgefirstcameraadaptor-neon.h"

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

/* ── TCase "NeonKernels" ───────────────────────────────────────────── */

GST_START_TEST (test_neon_rgba_to_rgb_u8)
{
  /* 4 pixels: known RGBA → verify RGB, alpha dropped */
  const uint8_t rgba[] = {
    10, 20, 30, 255,
    40, 50, 60, 128,
    70, 80, 90, 0,
    100, 110, 120, 200,
  };
  uint8_t rgb[12] = { 0 };

  edgefirst_neon_rgba_to_rgb_u8 (rgba, rgb, 4, FALSE);
  fail_unless_equals_int (rgb[0], 10);
  fail_unless_equals_int (rgb[1], 20);
  fail_unless_equals_int (rgb[2], 30);
  fail_unless_equals_int (rgb[3], 40);
  fail_unless_equals_int (rgb[4], 50);
  fail_unless_equals_int (rgb[5], 60);
  fail_unless_equals_int (rgb[9], 100);
  fail_unless_equals_int (rgb[10], 110);
  fail_unless_equals_int (rgb[11], 120);
}
GST_END_TEST;

GST_START_TEST (test_neon_rgba_to_rgb_u8_bgr)
{
  const uint8_t rgba[] = { 10, 20, 30, 255, 40, 50, 60, 128 };
  uint8_t rgb[6] = { 0 };

  edgefirst_neon_rgba_to_rgb_u8 (rgba, rgb, 2, TRUE);
  /* BGR: R/B swapped */
  fail_unless_equals_int (rgb[0], 30);  /* B */
  fail_unless_equals_int (rgb[1], 20);  /* G */
  fail_unless_equals_int (rgb[2], 10);  /* R */
  fail_unless_equals_int (rgb[3], 60);
  fail_unless_equals_int (rgb[4], 50);
  fail_unless_equals_int (rgb[5], 40);
}
GST_END_TEST;

GST_START_TEST (test_neon_rgba_to_rgb_i8)
{
  const uint8_t rgba[] = { 0, 127, 128, 255, 255, 0, 1, 200 };
  uint8_t rgb[6] = { 0 };

  edgefirst_neon_rgba_to_rgb_i8 (rgba, rgb, 2, FALSE);
  /* XOR 0x80: 0→0x80, 127→0xFF, 128→0, 255→0x7F */
  fail_unless_equals_int (rgb[0], 0x80);
  fail_unless_equals_int (rgb[1], 0xFF);
  fail_unless_equals_int (rgb[2], 0x00);
  fail_unless_equals_int (rgb[3], 0x7F);
  fail_unless_equals_int (rgb[4], 0x80);
  fail_unless_equals_int (rgb[5], 0x81);
}
GST_END_TEST;

GST_START_TEST (test_neon_planar_u8_to_i8)
{
  const uint8_t src[] = { 0x00, 0x7F, 0x80, 0xFF };
  uint8_t dst[4] = { 0 };

  edgefirst_neon_planar_u8_to_i8 (src, dst, 4);
  fail_unless_equals_int (dst[0], 0x80);
  fail_unless_equals_int (dst[1], 0xFF);
  fail_unless_equals_int (dst[2], 0x00);
  fail_unless_equals_int (dst[3], 0x7F);
}
GST_END_TEST;

GST_START_TEST (test_neon_rgba_to_rgb_f32)
{
  const uint8_t rgba[] = { 255, 0, 128, 255 };
  float rgb[3] = { 0 };
  const float mean[3] = { 0.0f, 0.0f, 0.0f };
  const float std[3] = { 1.0f, 1.0f, 1.0f };

  edgefirst_neon_rgba_to_rgb_f32 (rgba, rgb, 1, mean, std, FALSE);
  /* 255/255=1.0, 0/255=0.0, 128/255≈0.502 */
  fail_unless (fabsf (rgb[0] - 1.0f) < 0.001f,
      "Expected ~1.0, got %f", rgb[0]);
  fail_unless (fabsf (rgb[1] - 0.0f) < 0.001f,
      "Expected ~0.0, got %f", rgb[1]);
  fail_unless (fabsf (rgb[2] - 128.0f / 255.0f) < 0.001f,
      "Expected ~0.502, got %f", rgb[2]);
}
GST_END_TEST;

GST_START_TEST (test_neon_large_buffer)
{
  /* 640x640 = 409600 pixels — tests NEON/scalar boundary handling */
  size_t npixels = 640 * 640;
  uint8_t *rgba = g_malloc (npixels * 4);
  uint8_t *rgb = g_malloc (npixels * 3);

  /* Fill with known pattern */
  for (size_t i = 0; i < npixels; i++) {
    rgba[i * 4 + 0] = (uint8_t) (i & 0xFF);
    rgba[i * 4 + 1] = (uint8_t) ((i >> 8) & 0xFF);
    rgba[i * 4 + 2] = (uint8_t) ((i >> 16) & 0xFF);
    rgba[i * 4 + 3] = 0xFF;
  }

  edgefirst_neon_rgba_to_rgb_u8 (rgba, rgb, npixels, FALSE);

  /* Spot-check a few pixels */
  for (size_t i = 0; i < npixels; i += 10000) {
    fail_unless_equals_int (rgb[i * 3 + 0], (uint8_t) (i & 0xFF));
    fail_unless_equals_int (rgb[i * 3 + 1], (uint8_t) ((i >> 8) & 0xFF));
    fail_unless_equals_int (rgb[i * 3 + 2], (uint8_t) ((i >> 16) & 0xFF));
  }

  g_free (rgba);
  g_free (rgb);
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

  TCase *tc_neon = tcase_create ("NeonKernels");
  tcase_add_test (tc_neon, test_neon_rgba_to_rgb_u8);
  tcase_add_test (tc_neon, test_neon_rgba_to_rgb_u8_bgr);
  tcase_add_test (tc_neon, test_neon_rgba_to_rgb_i8);
  tcase_add_test (tc_neon, test_neon_planar_u8_to_i8);
  tcase_add_test (tc_neon, test_neon_rgba_to_rgb_f32);
  tcase_add_test (tc_neon, test_neon_large_buffer);
  suite_add_tcase (s, tc_neon);

  TCase *tc_content = tcase_create ("PipelineContent");
  tcase_set_timeout (tc_content, 10);
  tcase_add_test (tc_content, test_camera_adaptor_chw_int8);
  tcase_add_test (tc_content, test_camera_adaptor_bgr_hwc);
  tcase_add_test (tc_content, test_camera_adaptor_letterbox_properties);
  suite_add_tcase (s, tc_content);

  return s;
}

GST_CHECK_MAIN (edgefirst_hal_elements);
