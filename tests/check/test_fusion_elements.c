/*
 * EdgeFirst Perception for GStreamer - Fusion Element Tests
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gst/check/gstcheck.h>
#include <gst/base/gstbasetransform.h>

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "."
#endif

/* ── TCase "Creation" ──────────────────────────────────────────────── */

GST_START_TEST (test_pcd_classify_create)
{
  GstElement *el;

  el = gst_element_factory_make ("edgefirstpcdclassify", NULL);
  fail_unless (el != NULL, "Failed to create edgefirstpcdclassify element");

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_create)
{
  GstElement *el;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL, "Failed to create edgefirsttransforminject element");

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Properties" ───────────────────────────────────────────── */

GST_START_TEST (test_pcd_classify_output_mode_property)
{
  GstElement *el;
  gint mode;

  el = gst_element_factory_make ("edgefirstpcdclassify", NULL);
  fail_unless (el != NULL);

  /* Default should be 0 (labels) */
  g_object_get (el, "output-mode", &mode, NULL);
  fail_unless_equals_int (mode, 0);

  /* Set to 1 (colors) */
  g_object_set (el, "output-mode", 1, NULL);
  g_object_get (el, "output-mode", &mode, NULL);
  fail_unless_equals_int (mode, 1);

  /* Set to 2 (both) */
  g_object_set (el, "output-mode", 2, NULL);
  g_object_get (el, "output-mode", &mode, NULL);
  fail_unless_equals_int (mode, 2);

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_properties)
{
  GstElement *el;
  gchar *str_val;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  /* Defaults should be NULL */
  g_object_get (el, "calibration-file", &str_val, NULL);
  fail_unless (str_val == NULL);

  g_object_get (el, "frame-id", &str_val, NULL);
  fail_unless (str_val == NULL);

  g_object_get (el, "parent-frame-id", &str_val, NULL);
  fail_unless (str_val == NULL);

  /* Set/get roundtrip */
  g_object_set (el, "calibration-file", "/tmp/calib.yaml", NULL);
  g_object_get (el, "calibration-file", &str_val, NULL);
  fail_unless_equals_string (str_val, "/tmp/calib.yaml");
  g_free (str_val);

  g_object_set (el, "frame-id", "camera_front", NULL);
  g_object_get (el, "frame-id", &str_val, NULL);
  fail_unless_equals_string (str_val, "camera_front");
  g_free (str_val);

  g_object_set (el, "parent-frame-id", "base_link", NULL);
  g_object_get (el, "parent-frame-id", &str_val, NULL);
  fail_unless_equals_string (str_val, "base_link");
  g_free (str_val);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Pads" ─────────────────────────────────────────────────── */

GST_START_TEST (test_pcd_classify_pad_templates)
{
  GstElementFactory *factory;
  const GList *templates;
  gboolean has_sink_cloud = FALSE, has_sink_mask = FALSE, has_src = FALSE;

  factory = gst_element_factory_find ("edgefirstpcdclassify");
  fail_unless (factory != NULL);

  templates = gst_element_factory_get_static_pad_templates (factory);

  for (const GList *l = templates; l != NULL; l = l->next) {
    GstStaticPadTemplate *t = l->data;

    if (g_strcmp0 (t->name_template, "sink_cloud") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SINK);
      fail_unless_equals_int (t->presence, GST_PAD_ALWAYS);
      has_sink_cloud = TRUE;
    } else if (g_strcmp0 (t->name_template, "sink_mask") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SINK);
      fail_unless_equals_int (t->presence, GST_PAD_ALWAYS);
      has_sink_mask = TRUE;
    } else if (g_strcmp0 (t->name_template, "src") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SRC);
      fail_unless_equals_int (t->presence, GST_PAD_ALWAYS);
      has_src = TRUE;
    }
  }

  fail_unless (has_sink_cloud, "Missing sink_cloud pad template");
  fail_unless (has_sink_mask, "Missing sink_mask pad template");
  fail_unless (has_src, "Missing src pad template");

  gst_object_unref (factory);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_pad_templates)
{
  GstElementFactory *factory;
  const GList *templates;
  gboolean has_sink = FALSE, has_src = FALSE;

  factory = gst_element_factory_find ("edgefirsttransforminject");
  fail_unless (factory != NULL);

  templates = gst_element_factory_get_static_pad_templates (factory);

  for (const GList *l = templates; l != NULL; l = l->next) {
    GstStaticPadTemplate *t = l->data;

    if (g_strcmp0 (t->name_template, "sink") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SINK);
      has_sink = TRUE;
    } else if (g_strcmp0 (t->name_template, "src") == 0) {
      fail_unless_equals_int (t->direction, GST_PAD_SRC);
      has_src = TRUE;
    }
  }

  fail_unless (has_sink, "Missing sink pad template");
  fail_unless (has_src, "Missing src pad template");

  gst_object_unref (factory);
}
GST_END_TEST;

/* ── TCase "Behavior" ─────────────────────────────────────────────── */

GST_START_TEST (test_transform_inject_not_passthrough)
{
  GstElement *el;
  GstBaseTransformClass *trans_class;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  trans_class = GST_BASE_TRANSFORM_GET_CLASS (el);
  fail_unless (trans_class->passthrough_on_same_caps == FALSE,
      "passthrough_on_same_caps must be FALSE to ensure transform_ip runs");

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_is_in_place)
{
  GstElement *el;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  fail_unless (gst_base_transform_is_in_place (GST_BASE_TRANSFORM (el)) == TRUE);

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Metadata" ─────────────────────────────────────────────── */

GST_START_TEST (test_pcd_classify_element_metadata)
{
  GstElement *el;
  GstElementFactory *factory;
  const gchar *klass, *author;

  el = gst_element_factory_make ("edgefirstpcdclassify", NULL);
  fail_unless (el != NULL);

  factory = gst_element_get_factory (el);
  klass = gst_element_factory_get_metadata (factory, GST_ELEMENT_METADATA_KLASS);
  author = gst_element_factory_get_metadata (factory, GST_ELEMENT_METADATA_AUTHOR);

  fail_unless_equals_string (klass, "Filter/Video");
  fail_unless_equals_string (author, "Au-Zone Technologies <support@au-zone.com>");

  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_element_metadata)
{
  GstElement *el;
  GstElementFactory *factory;
  const gchar *klass;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  factory = gst_element_get_factory (el);
  klass = gst_element_factory_get_metadata (factory, GST_ELEMENT_METADATA_KLASS);

  fail_unless_equals_string (klass, "Filter/Metadata");

  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "Calibration" ──────────────────────────────────────────── */

GST_START_TEST (test_transform_inject_load_calibration)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  g_object_set (el, "calibration-file", FIXTURE_DIR "/test_calibration.json", NULL);

  /* start() is called during READY->PAUSED */
  ret = gst_element_set_state (el, GST_STATE_PAUSED);
  fail_unless (ret != GST_STATE_CHANGE_FAILURE,
      "start() should succeed with valid calibration file, got %d", ret);

  gst_element_set_state (el, GST_STATE_NULL);
  gst_object_unref (el);
}
GST_END_TEST;

GST_START_TEST (test_transform_inject_load_invalid)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
  fail_unless (el != NULL);

  g_object_set (el, "calibration-file", FIXTURE_DIR "/test_calibration_invalid.json", NULL);

  /* start() is called during READY->PAUSED, should fail */
  ret = gst_element_set_state (el, GST_STATE_PAUSED);
  fail_unless (ret == GST_STATE_CHANGE_FAILURE,
      "start() should fail with invalid calibration file, got %d", ret);

  gst_element_set_state (el, GST_STATE_NULL);
  gst_object_unref (el);
}
GST_END_TEST;

/* ── TCase "StateTransitions" ─────────────────────────────────────── */

GST_START_TEST (test_transform_inject_state_null_to_ready)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirsttransforminject", NULL);
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

GST_START_TEST (test_pcd_classify_state_null_to_ready)
{
  GstElement *el;
  GstStateChangeReturn ret;

  el = gst_element_factory_make ("edgefirstpcdclassify", NULL);
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

/* ── Suite ─────────────────────────────────────────────────────────── */

static Suite *
edgefirst_fusion_elements_suite (void)
{
  Suite *s = suite_create ("EdgeFirst Fusion Elements");

  TCase *tc_create = tcase_create ("Creation");
  tcase_add_test (tc_create, test_pcd_classify_create);
  tcase_add_test (tc_create, test_transform_inject_create);
  suite_add_tcase (s, tc_create);

  TCase *tc_props = tcase_create ("Properties");
  tcase_add_test (tc_props, test_pcd_classify_output_mode_property);
  tcase_add_test (tc_props, test_transform_inject_properties);
  suite_add_tcase (s, tc_props);

  TCase *tc_pads = tcase_create ("Pads");
  tcase_add_test (tc_pads, test_pcd_classify_pad_templates);
  tcase_add_test (tc_pads, test_transform_inject_pad_templates);
  suite_add_tcase (s, tc_pads);

  TCase *tc_behavior = tcase_create ("Behavior");
  tcase_add_test (tc_behavior, test_transform_inject_not_passthrough);
  tcase_add_test (tc_behavior, test_transform_inject_is_in_place);
  suite_add_tcase (s, tc_behavior);

  TCase *tc_metadata = tcase_create ("Metadata");
  tcase_add_test (tc_metadata, test_pcd_classify_element_metadata);
  tcase_add_test (tc_metadata, test_transform_inject_element_metadata);
  suite_add_tcase (s, tc_metadata);

  TCase *tc_calib = tcase_create ("Calibration");
  tcase_add_test (tc_calib, test_transform_inject_load_calibration);
  tcase_add_test (tc_calib, test_transform_inject_load_invalid);
  suite_add_tcase (s, tc_calib);

  TCase *tc_states = tcase_create ("StateTransitions");
  tcase_add_test (tc_states, test_transform_inject_state_null_to_ready);
  tcase_add_test (tc_states, test_pcd_classify_state_null_to_ready);
  suite_add_tcase (s, tc_states);

  return s;
}

GST_CHECK_MAIN (edgefirst_fusion_elements);
