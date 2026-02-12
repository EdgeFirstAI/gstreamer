/*
 * EdgeFirst Perception for GStreamer - Math Tests
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gst/check/gstcheck.h>
#include <gst/edgefirst/edgefirst.h>
#include <math.h>

#define ASSERT_FLOAT_EQ(actual, expected, eps) \
  fail_unless (fabs ((actual) - (expected)) < (eps), \
    "Expected %f got %f", (double)(expected), (double)(actual))
#define EPS 1e-9

/* ── TCase "Transform" ─────────────────────────────────────────────── */

GST_START_TEST (test_transform_identity_no_translation)
{
  EdgefirstTransformData t;
  gdouble x = 5.0, y = -3.0, z = 7.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 5.0, EPS);
  ASSERT_FLOAT_EQ (y, -3.0, EPS);
  ASSERT_FLOAT_EQ (z, 7.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_translation_only)
{
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 2.0, z = 3.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.translation[0] = 10.0;
  t.translation[1] = -20.0;
  t.translation[2] = 30.0;

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 11.0, EPS);
  ASSERT_FLOAT_EQ (y, -18.0, EPS);
  ASSERT_FLOAT_EQ (z, 33.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_90deg_about_z)
{
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 0.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[0] = 0.0;
  t.rotation[1] = 0.0;
  t.rotation[2] = sin (G_PI / 4.0);
  t.rotation[3] = cos (G_PI / 4.0);

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 0.0, EPS);
  ASSERT_FLOAT_EQ (y, 1.0, EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_90deg_about_x)
{
  EdgefirstTransformData t;
  gdouble x = 0.0, y = 1.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[0] = sin (G_PI / 4.0);
  t.rotation[1] = 0.0;
  t.rotation[2] = 0.0;
  t.rotation[3] = cos (G_PI / 4.0);

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 0.0, EPS);
  ASSERT_FLOAT_EQ (y, 0.0, EPS);
  ASSERT_FLOAT_EQ (z, 1.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_90deg_about_y)
{
  EdgefirstTransformData t;
  gdouble x = 0.0, y = 0.0, z = 1.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[0] = 0.0;
  t.rotation[1] = sin (G_PI / 4.0);
  t.rotation[2] = 0.0;
  t.rotation[3] = cos (G_PI / 4.0);

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 1.0, EPS);
  ASSERT_FLOAT_EQ (y, 0.0, EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_180deg_about_z)
{
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 0.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[0] = 0.0;
  t.rotation[1] = 0.0;
  t.rotation[2] = 1.0;
  t.rotation[3] = 0.0;

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, -1.0, EPS);
  ASSERT_FLOAT_EQ (y, 0.0, EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_rotation_plus_translation)
{
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 0.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  /* 90 deg about Z */
  t.rotation[2] = sin (G_PI / 4.0);
  t.rotation[3] = cos (G_PI / 4.0);
  t.translation[0] = 5.0;
  t.translation[1] = 10.0;
  t.translation[2] = 0.0;

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, 5.0, EPS);
  ASSERT_FLOAT_EQ (y, 11.0, EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_45deg_about_z)
{
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 0.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[2] = sin (G_PI / 8.0);
  t.rotation[3] = cos (G_PI / 8.0);

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  ASSERT_FLOAT_EQ (x, cos (G_PI / 4.0), EPS);
  ASSERT_FLOAT_EQ (y, sin (G_PI / 4.0), EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_arbitrary_rotation)
{
  /* q = (0.5, 0.5, 0.5, 0.5) = 120deg about (1,1,1)/sqrt(3) */
  EdgefirstTransformData t;
  gdouble x = 1.0, y = 0.0, z = 0.0;

  edgefirst_perception_init ();
  edgefirst_transform_data_set_identity (&t);
  t.rotation[0] = 0.5;
  t.rotation[1] = 0.5;
  t.rotation[2] = 0.5;
  t.rotation[3] = 0.5;

  edgefirst_transform_data_apply (&t, &x, &y, &z);

  /* Cyclic permutation: (1,0,0) -> (0,1,0) */
  ASSERT_FLOAT_EQ (x, 0.0, EPS);
  ASSERT_FLOAT_EQ (y, 1.0, EPS);
  ASSERT_FLOAT_EQ (z, 0.0, EPS);
}
GST_END_TEST;

GST_START_TEST (test_transform_identity_frame_ids)
{
  EdgefirstTransformData t;

  edgefirst_perception_init ();

  /* Set some values first */
  g_strlcpy (t.child_frame_id, "lidar", EDGEFIRST_FRAME_ID_MAX_LEN);
  g_strlcpy (t.parent_frame_id, "base_link", EDGEFIRST_FRAME_ID_MAX_LEN);
  t.timestamp_ns = 123456789;
  t.rotation[0] = 0.5;

  /* set_identity should clear everything */
  edgefirst_transform_data_set_identity (&t);

  fail_unless_equals_string (t.child_frame_id, "");
  fail_unless_equals_string (t.parent_frame_id, "");
  fail_unless_equals_uint64 (t.timestamp_ns, 0);
  ASSERT_FLOAT_EQ (t.rotation[0], 0.0, EPS);
  ASSERT_FLOAT_EQ (t.rotation[1], 0.0, EPS);
  ASSERT_FLOAT_EQ (t.rotation[2], 0.0, EPS);
  ASSERT_FLOAT_EQ (t.rotation[3], 1.0, EPS);
  ASSERT_FLOAT_EQ (t.translation[0], 0.0, EPS);
  ASSERT_FLOAT_EQ (t.translation[1], 0.0, EPS);
  ASSERT_FLOAT_EQ (t.translation[2], 0.0, EPS);
}
GST_END_TEST;

/* ── TCase "Camera" ────────────────────────────────────────────────── */

GST_START_TEST (test_camera_identity_project_center)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;
  gboolean valid;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 640, 480);

  valid = edgefirst_camera_info_meta_project_point (meta, 0.0, 0.0, 1.0, &u, &v);
  fail_unless (valid == TRUE);
  ASSERT_FLOAT_EQ (u, 320.0, EPS);
  ASSERT_FLOAT_EQ (v, 240.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_identity_project_known_values)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 640, 480);

  /* (1, 0, 2) -> u = 640*1/2 + 320 = 640, v = 480*0/2 + 240 = 240 */
  edgefirst_camera_info_meta_project_point (meta, 1.0, 0.0, 2.0, &u, &v);
  ASSERT_FLOAT_EQ (u, 640.0, EPS);
  ASSERT_FLOAT_EQ (v, 240.0, EPS);

  /* (0, 1, 2) -> u = 320, v = 480*1/2 + 240 = 480 */
  edgefirst_camera_info_meta_project_point (meta, 0.0, 1.0, 2.0, &u, &v);
  ASSERT_FLOAT_EQ (u, 320.0, EPS);
  ASSERT_FLOAT_EQ (v, 480.0, EPS);

  /* (-1, -1, 4) -> u = 640*(-1)/4 + 320 = 160, v = 480*(-1)/4 + 240 = 120 */
  edgefirst_camera_info_meta_project_point (meta, -1.0, -1.0, 4.0, &u, &v);
  ASSERT_FLOAT_EQ (u, 160.0, EPS);
  ASSERT_FLOAT_EQ (v, 120.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_project_behind_z_zero)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;
  gboolean valid;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 640, 480);

  valid = edgefirst_camera_info_meta_project_point (meta, 1.0, 1.0, 0.0, &u, &v);
  fail_unless (valid == FALSE);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_project_negative_z)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;
  gboolean valid;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 640, 480);

  valid = edgefirst_camera_info_meta_project_point (meta, 0.0, 0.0, -5.0, &u, &v);
  fail_unless (valid == FALSE);

  valid = edgefirst_camera_info_meta_project_point (meta, 1.0, -1.0, -0.001, &u, &v);
  fail_unless (valid == FALSE);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_set_identity_k_matrix)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 1920, 1080);

  ASSERT_FLOAT_EQ (meta->K[0], 1920.0, EPS);   /* fx */
  ASSERT_FLOAT_EQ (meta->K[1], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->K[2], 960.0, EPS);     /* cx */
  ASSERT_FLOAT_EQ (meta->K[3], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->K[4], 1080.0, EPS);   /* fy */
  ASSERT_FLOAT_EQ (meta->K[5], 540.0, EPS);     /* cy */
  ASSERT_FLOAT_EQ (meta->K[6], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->K[7], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->K[8], 1.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_set_identity_p_matrix)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 1920, 1080);

  /* P = [K | 0] -> 3x4 row-major */
  ASSERT_FLOAT_EQ (meta->P[0], 1920.0, EPS);   /* fx */
  ASSERT_FLOAT_EQ (meta->P[1], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[2], 960.0, EPS);     /* cx */
  ASSERT_FLOAT_EQ (meta->P[3], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[4], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[5], 1080.0, EPS);   /* fy */
  ASSERT_FLOAT_EQ (meta->P[6], 540.0, EPS);     /* cy */
  ASSERT_FLOAT_EQ (meta->P[7], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[8], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[9], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[10], 1.0, EPS);
  ASSERT_FLOAT_EQ (meta->P[11], 0.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_set_identity_r_matrix)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);
  edgefirst_camera_info_meta_set_identity (meta, 1920, 1080);

  ASSERT_FLOAT_EQ (meta->R[0], 1.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[1], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[2], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[3], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[4], 1.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[5], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[6], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[7], 0.0, EPS);
  ASSERT_FLOAT_EQ (meta->R[8], 1.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_camera_project_with_custom_k)
{
  GstBuffer *buf;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;
  gboolean valid;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buf);

  /* Manual K: fx=500, fy=500, cx=320, cy=240 */
  memset (meta->K, 0, sizeof (meta->K));
  meta->K[0] = 500.0;
  meta->K[2] = 320.0;
  meta->K[4] = 500.0;
  meta->K[5] = 240.0;
  meta->K[8] = 1.0;

  /* (2, 3, 5) -> u = 500*2/5 + 320 = 520, v = 500*3/5 + 240 = 540 */
  valid = edgefirst_camera_info_meta_project_point (meta, 2.0, 3.0, 5.0, &u, &v);
  fail_unless (valid == TRUE);
  ASSERT_FLOAT_EQ (u, 520.0, EPS);
  ASSERT_FLOAT_EQ (v, 540.0, EPS);

  gst_buffer_unref (buf);
}
GST_END_TEST;

/* ── TCase "Version" ───────────────────────────────────────────────── */

GST_START_TEST (test_perception_version)
{
  const gchar *version;

  edgefirst_perception_init ();

  version = edgefirst_perception_version ();
  fail_unless (version != NULL, "Version string should not be NULL");
  fail_unless (strlen (version) > 0, "Version string should not be empty");
}
GST_END_TEST;

/* ── TCase "PointFields" ──────────────────────────────────────────── */

GST_START_TEST (test_parse_point_fields_xyz)
{
  EdgefirstPointFieldDesc fields[8];
  guint n;

  edgefirst_perception_init ();

  n = edgefirst_parse_point_fields ("x:F32:0,y:F32:4,z:F32:8", fields, 8);
  fail_unless_equals_int (n, 3);

  fail_unless_equals_string (fields[0].name, "x");
  fail_unless_equals_int (fields[0].datatype, EDGEFIRST_POINT_FIELD_FLOAT32);
  fail_unless_equals_int (fields[0].offset, 0);

  fail_unless_equals_string (fields[1].name, "y");
  fail_unless_equals_int (fields[1].datatype, EDGEFIRST_POINT_FIELD_FLOAT32);
  fail_unless_equals_int (fields[1].offset, 4);

  fail_unless_equals_string (fields[2].name, "z");
  fail_unless_equals_int (fields[2].datatype, EDGEFIRST_POINT_FIELD_FLOAT32);
  fail_unless_equals_int (fields[2].offset, 8);
}
GST_END_TEST;

GST_START_TEST (test_parse_point_fields_roundtrip)
{
  EdgefirstPointFieldDesc fields[8];
  guint n;
  gchar *formatted;

  edgefirst_perception_init ();

  n = edgefirst_parse_point_fields ("x:F32:0,y:F32:4,z:F32:8", fields, 8);
  fail_unless_equals_int (n, 3);

  formatted = edgefirst_format_point_fields (fields, n);
  fail_unless_equals_string (formatted, "x:F32:0,y:F32:4,z:F32:8");
  g_free (formatted);
}
GST_END_TEST;

GST_START_TEST (test_point_field_datatype_size)
{
  edgefirst_perception_init ();

  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_INT8), 1);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_UINT8), 1);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_INT16), 2);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_UINT16), 2);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_INT32), 4);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_UINT32), 4);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_FLOAT32), 4);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (EDGEFIRST_POINT_FIELD_FLOAT64), 8);
  fail_unless_equals_int (edgefirst_point_field_datatype_size (0), 0);
}
GST_END_TEST;

GST_START_TEST (test_parse_point_fields_empty)
{
  EdgefirstPointFieldDesc fields[8];
  guint n;

  edgefirst_perception_init ();

  n = edgefirst_parse_point_fields (NULL, fields, 8);
  fail_unless_equals_int (n, 0);

  n = edgefirst_parse_point_fields ("", fields, 8);
  fail_unless_equals_int (n, 0);
}
GST_END_TEST;

/* ── TCase "Enums" ─────────────────────────────────────────────────── */

GST_START_TEST (test_radar_dimension_all_values)
{
  edgefirst_perception_init ();

  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_UNDEFINED), "UNDEFINED");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_RANGE), "RANGE");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_DOPPLER), "DOPPLER");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_AZIMUTH), "AZIMUTH");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_ELEVATION), "ELEVATION");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_RXCHANNEL), "RXCHANNEL");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_SEQUENCE), "SEQUENCE");
}
GST_END_TEST;

GST_START_TEST (test_radar_dimension_out_of_range)
{
  edgefirst_perception_init ();

  fail_unless_equals_string (edgefirst_radar_dimension_to_string ((EdgefirstRadarDimension) 99), "UNDEFINED");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string ((EdgefirstRadarDimension) -1), "UNDEFINED");
}
GST_END_TEST;

/* ── TCase "Init" ──────────────────────────────────────────────────── */

GST_START_TEST (test_perception_init_idempotent)
{
  GstBuffer *buf;
  EdgefirstPointCloud2Meta *meta;

  edgefirst_perception_init ();
  edgefirst_perception_init ();
  edgefirst_perception_init ();

  /* Meta should still work after triple init */
  buf = gst_buffer_new ();
  meta = edgefirst_buffer_add_pointcloud2_meta (buf);
  fail_unless (meta != NULL);
  meta->point_count = 42;
  fail_unless_equals_int (edgefirst_buffer_get_pointcloud2_meta (buf)->point_count, 42);

  gst_buffer_unref (buf);
}
GST_END_TEST;

/* ── Suite ─────────────────────────────────────────────────────────── */

static Suite *
edgefirst_math_suite (void)
{
  Suite *s = suite_create ("EdgeFirst Math");

  TCase *tc_transform = tcase_create ("Transform");
  tcase_add_test (tc_transform, test_transform_identity_no_translation);
  tcase_add_test (tc_transform, test_transform_translation_only);
  tcase_add_test (tc_transform, test_transform_90deg_about_z);
  tcase_add_test (tc_transform, test_transform_90deg_about_x);
  tcase_add_test (tc_transform, test_transform_90deg_about_y);
  tcase_add_test (tc_transform, test_transform_180deg_about_z);
  tcase_add_test (tc_transform, test_transform_rotation_plus_translation);
  tcase_add_test (tc_transform, test_transform_45deg_about_z);
  tcase_add_test (tc_transform, test_transform_arbitrary_rotation);
  tcase_add_test (tc_transform, test_transform_identity_frame_ids);
  suite_add_tcase (s, tc_transform);

  TCase *tc_camera = tcase_create ("Camera");
  tcase_add_test (tc_camera, test_camera_identity_project_center);
  tcase_add_test (tc_camera, test_camera_identity_project_known_values);
  tcase_add_test (tc_camera, test_camera_project_behind_z_zero);
  tcase_add_test (tc_camera, test_camera_project_negative_z);
  tcase_add_test (tc_camera, test_camera_set_identity_k_matrix);
  tcase_add_test (tc_camera, test_camera_set_identity_p_matrix);
  tcase_add_test (tc_camera, test_camera_set_identity_r_matrix);
  tcase_add_test (tc_camera, test_camera_project_with_custom_k);
  suite_add_tcase (s, tc_camera);

  TCase *tc_version = tcase_create ("Version");
  tcase_add_test (tc_version, test_perception_version);
  suite_add_tcase (s, tc_version);

  TCase *tc_fields = tcase_create ("PointFields");
  tcase_add_test (tc_fields, test_parse_point_fields_xyz);
  tcase_add_test (tc_fields, test_parse_point_fields_roundtrip);
  tcase_add_test (tc_fields, test_point_field_datatype_size);
  tcase_add_test (tc_fields, test_parse_point_fields_empty);
  suite_add_tcase (s, tc_fields);

  TCase *tc_enums = tcase_create ("Enums");
  tcase_add_test (tc_enums, test_radar_dimension_all_values);
  tcase_add_test (tc_enums, test_radar_dimension_out_of_range);
  suite_add_tcase (s, tc_enums);

  TCase *tc_init = tcase_create ("Init");
  tcase_add_test (tc_init, test_perception_init_idempotent);
  suite_add_tcase (s, tc_init);

  return s;
}

GST_CHECK_MAIN (edgefirst_math);
