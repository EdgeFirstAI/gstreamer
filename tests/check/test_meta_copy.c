/*
 * EdgeFirst Perception for GStreamer - Meta Copy Tests
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gst/check/gstcheck.h>
#include <gst/edgefirst/edgefirst.h>
#include <math.h>
#include <string.h>

/* ── TCase "Copy" ──────────────────────────────────────────────────── */

GST_START_TEST (test_pointcloud2_meta_copy)
{
  GstBuffer *src, *dst;
  EdgefirstPointCloud2Meta *meta, *copy;

  edgefirst_perception_init ();

  src = gst_buffer_new ();
  meta = edgefirst_buffer_add_pointcloud2_meta (src);

  /* Populate all fields */
  meta->point_count = 65536;
  g_strlcpy (meta->frame_id, "velodyne_top", EDGEFIRST_FRAME_ID_MAX_LEN);
  meta->ros_timestamp_ns = 1700000000123456789ULL;
  meta->has_transform = TRUE;
  meta->transform.translation[0] = 1.5;
  meta->transform.translation[1] = -0.3;
  meta->transform.translation[2] = 2.1;
  meta->transform.rotation[0] = 0.0;
  meta->transform.rotation[1] = 0.0;
  meta->transform.rotation[2] = sin (G_PI / 4.0);
  meta->transform.rotation[3] = cos (G_PI / 4.0);
  g_strlcpy (meta->transform.child_frame_id, "velodyne", EDGEFIRST_FRAME_ID_MAX_LEN);
  g_strlcpy (meta->transform.parent_frame_id, "base_link", EDGEFIRST_FRAME_ID_MAX_LEN);
  meta->transform.timestamp_ns = 9876543210ULL;

  /* Copy buffer (triggers meta copy) */
  dst = gst_buffer_copy (src);
  copy = edgefirst_buffer_get_pointcloud2_meta (dst);
  fail_unless (copy != NULL);

  fail_unless_equals_int (copy->point_count, 65536);
  fail_unless_equals_string (copy->frame_id, "velodyne_top");
  fail_unless_equals_uint64 (copy->ros_timestamp_ns, 1700000000123456789ULL);
  fail_unless (copy->has_transform == TRUE);
  fail_unless_equals_float (copy->transform.translation[0], 1.5);
  fail_unless_equals_float (copy->transform.translation[1], -0.3);
  fail_unless_equals_float (copy->transform.translation[2], 2.1);
  fail_unless_equals_float (copy->transform.rotation[2], sin (G_PI / 4.0));
  fail_unless_equals_float (copy->transform.rotation[3], cos (G_PI / 4.0));
  fail_unless_equals_string (copy->transform.child_frame_id, "velodyne");
  fail_unless_equals_string (copy->transform.parent_frame_id, "base_link");
  fail_unless_equals_uint64 (copy->transform.timestamp_ns, 9876543210ULL);

  gst_buffer_unref (src);
  gst_buffer_unref (dst);
}
GST_END_TEST;

GST_START_TEST (test_radar_cube_meta_copy)
{
  GstBuffer *src, *dst;
  EdgefirstRadarCubeMeta *meta, *copy;

  edgefirst_perception_init ();

  src = gst_buffer_new ();
  meta = edgefirst_buffer_add_radar_cube_meta (src);

  meta->layout[0] = EDGEFIRST_RADAR_DIM_RANGE;
  meta->layout[1] = EDGEFIRST_RADAR_DIM_DOPPLER;
  meta->layout[2] = EDGEFIRST_RADAR_DIM_AZIMUTH;
  meta->layout[3] = EDGEFIRST_RADAR_DIM_ELEVATION;
  meta->num_dims = 4;
  meta->scales[0] = 0.1f;
  meta->scales[1] = 0.5f;
  meta->scales[2] = 1.0f;
  meta->scales[3] = 2.0f;
  meta->is_complex = TRUE;
  meta->radar_timestamp = 42000000ULL;
  g_strlcpy (meta->frame_id, "radar_front", EDGEFIRST_FRAME_ID_MAX_LEN);

  dst = gst_buffer_copy (src);
  copy = edgefirst_buffer_get_radar_cube_meta (dst);
  fail_unless (copy != NULL);

  fail_unless_equals_int (copy->layout[0], EDGEFIRST_RADAR_DIM_RANGE);
  fail_unless_equals_int (copy->layout[1], EDGEFIRST_RADAR_DIM_DOPPLER);
  fail_unless_equals_int (copy->layout[2], EDGEFIRST_RADAR_DIM_AZIMUTH);
  fail_unless_equals_int (copy->layout[3], EDGEFIRST_RADAR_DIM_ELEVATION);
  fail_unless_equals_int (copy->num_dims, 4);
  fail_unless_equals_float (copy->scales[0], 0.1f);
  fail_unless_equals_float (copy->scales[1], 0.5f);
  fail_unless_equals_float (copy->scales[2], 1.0f);
  fail_unless_equals_float (copy->scales[3], 2.0f);
  fail_unless (copy->is_complex == TRUE);
  fail_unless_equals_uint64 (copy->radar_timestamp, 42000000ULL);
  fail_unless_equals_string (copy->frame_id, "radar_front");

  gst_buffer_unref (src);
  gst_buffer_unref (dst);
}
GST_END_TEST;

GST_START_TEST (test_camera_info_meta_copy)
{
  GstBuffer *src, *dst;
  EdgefirstCameraInfoMeta *meta, *copy;
  gint i;

  edgefirst_perception_init ();

  src = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (src);

  edgefirst_camera_info_meta_set_identity (meta, 1920, 1080);
  meta->distortion_model = EDGEFIRST_DISTORTION_PLUMB_BOB;
  meta->num_distortion_coeffs = 5;
  meta->D[0] = -0.1;
  meta->D[1] = 0.05;
  meta->D[2] = 0.001;
  meta->D[3] = -0.002;
  meta->D[4] = 0.01;
  g_strlcpy (meta->frame_id, "camera_front", EDGEFIRST_FRAME_ID_MAX_LEN);

  dst = gst_buffer_copy (src);
  copy = edgefirst_buffer_get_camera_info_meta (dst);
  fail_unless (copy != NULL);

  fail_unless_equals_int (copy->width, 1920);
  fail_unless_equals_int (copy->height, 1080);
  fail_unless_equals_int (copy->distortion_model, EDGEFIRST_DISTORTION_PLUMB_BOB);
  fail_unless_equals_int (copy->num_distortion_coeffs, 5);

  for (i = 0; i < 9; i++)
    fail_unless_equals_float (copy->K[i], meta->K[i]);
  for (i = 0; i < 5; i++)
    fail_unless_equals_float (copy->D[i], meta->D[i]);
  for (i = 0; i < 9; i++)
    fail_unless_equals_float (copy->R[i], meta->R[i]);
  for (i = 0; i < 12; i++)
    fail_unless_equals_float (copy->P[i], meta->P[i]);

  fail_unless_equals_string (copy->frame_id, "camera_front");

  gst_buffer_unref (src);
  gst_buffer_unref (dst);
}
GST_END_TEST;

GST_START_TEST (test_transform_meta_copy)
{
  GstBuffer *src, *dst;
  EdgefirstTransformMeta *meta, *copy;

  edgefirst_perception_init ();

  src = gst_buffer_new ();
  meta = edgefirst_buffer_add_transform_meta (src);

  edgefirst_transform_data_set_identity (&meta->transform);
  meta->transform.translation[0] = 10.0;
  meta->transform.translation[1] = 20.0;
  meta->transform.translation[2] = 30.0;
  /* 90-deg-Z quaternion */
  meta->transform.rotation[0] = 0.0;
  meta->transform.rotation[1] = 0.0;
  meta->transform.rotation[2] = sin (G_PI / 4.0);
  meta->transform.rotation[3] = cos (G_PI / 4.0);
  g_strlcpy (meta->transform.child_frame_id, "lidar", EDGEFIRST_FRAME_ID_MAX_LEN);
  g_strlcpy (meta->transform.parent_frame_id, "base_link", EDGEFIRST_FRAME_ID_MAX_LEN);
  meta->transform.timestamp_ns = 555555555ULL;

  dst = gst_buffer_copy (src);
  copy = edgefirst_buffer_get_transform_meta (dst);
  fail_unless (copy != NULL);

  fail_unless_equals_float (copy->transform.translation[0], 10.0);
  fail_unless_equals_float (copy->transform.translation[1], 20.0);
  fail_unless_equals_float (copy->transform.translation[2], 30.0);
  fail_unless_equals_float (copy->transform.rotation[2], sin (G_PI / 4.0));
  fail_unless_equals_float (copy->transform.rotation[3], cos (G_PI / 4.0));
  fail_unless_equals_string (copy->transform.child_frame_id, "lidar");
  fail_unless_equals_string (copy->transform.parent_frame_id, "base_link");
  fail_unless_equals_uint64 (copy->transform.timestamp_ns, 555555555ULL);

  gst_buffer_unref (src);
  gst_buffer_unref (dst);
}
GST_END_TEST;

/* ── TCase "MiscMeta" ─────────────────────────────────────────────── */

GST_START_TEST (test_meta_absent_on_empty_buffer)
{
  GstBuffer *buf;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();

  fail_unless (edgefirst_buffer_get_pointcloud2_meta (buf) == NULL);
  fail_unless (edgefirst_buffer_get_radar_cube_meta (buf) == NULL);
  fail_unless (edgefirst_buffer_get_camera_info_meta (buf) == NULL);
  fail_unless (edgefirst_buffer_get_transform_meta (buf) == NULL);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_multiple_meta_types_on_buffer)
{
  GstBuffer *buf;
  EdgefirstPointCloud2Meta *pc;
  EdgefirstRadarCubeMeta *rc;
  EdgefirstCameraInfoMeta *ci;
  EdgefirstTransformMeta *tm;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();

  /* Add all four types */
  pc = edgefirst_buffer_add_pointcloud2_meta (buf);
  rc = edgefirst_buffer_add_radar_cube_meta (buf);
  ci = edgefirst_buffer_add_camera_info_meta (buf);
  tm = edgefirst_buffer_add_transform_meta (buf);

  fail_unless (pc != NULL);
  fail_unless (rc != NULL);
  fail_unless (ci != NULL);
  fail_unless (tm != NULL);

  /* Mark each with distinct values */
  pc->point_count = 111;
  rc->num_dims = 3;
  ci->width = 640;
  tm->transform.translation[0] = 99.0;

  /* Retrieve independently and verify no cross-contamination */
  fail_unless_equals_int (edgefirst_buffer_get_pointcloud2_meta (buf)->point_count, 111);
  fail_unless_equals_int (edgefirst_buffer_get_radar_cube_meta (buf)->num_dims, 3);
  fail_unless_equals_int (edgefirst_buffer_get_camera_info_meta (buf)->width, 640);
  fail_unless_equals_float (edgefirst_buffer_get_transform_meta (buf)->transform.translation[0], 99.0);

  gst_buffer_unref (buf);
}
GST_END_TEST;

GST_START_TEST (test_meta_init_defaults)
{
  GstBuffer *buf;
  EdgefirstPointCloud2Meta *pc;
  EdgefirstRadarCubeMeta *rc;
  EdgefirstCameraInfoMeta *ci;
  EdgefirstTransformMeta *tm;

  edgefirst_perception_init ();

  buf = gst_buffer_new ();

  /* PointCloud2 defaults */
  pc = edgefirst_buffer_add_pointcloud2_meta (buf);
  fail_unless_equals_int (pc->point_count, 0);
  fail_unless_equals_string (pc->frame_id, "");
  fail_unless_equals_uint64 (pc->ros_timestamp_ns, 0);
  fail_unless (pc->has_transform == FALSE);

  /* RadarCube defaults */
  rc = edgefirst_buffer_add_radar_cube_meta (buf);
  fail_unless_equals_int (rc->num_dims, 0);
  fail_unless (rc->is_complex == FALSE);
  fail_unless_equals_uint64 (rc->radar_timestamp, 0);
  fail_unless_equals_string (rc->frame_id, "");

  /* CameraInfo defaults */
  ci = edgefirst_buffer_add_camera_info_meta (buf);
  fail_unless_equals_int (ci->width, 0);
  fail_unless_equals_int (ci->height, 0);
  fail_unless_equals_int (ci->num_distortion_coeffs, 0);
  fail_unless_equals_int (ci->distortion_model, EDGEFIRST_DISTORTION_NONE);
  fail_unless_equals_string (ci->frame_id, "");

  /* Transform defaults (set_identity called in init) */
  tm = edgefirst_buffer_add_transform_meta (buf);
  fail_unless_equals_float (tm->transform.rotation[3], 1.0);
  fail_unless_equals_float (tm->transform.translation[0], 0.0);
  fail_unless_equals_string (tm->transform.child_frame_id, "");

  gst_buffer_unref (buf);
}
GST_END_TEST;

/* ── Suite ─────────────────────────────────────────────────────────── */

static Suite *
edgefirst_meta_copy_suite (void)
{
  Suite *s = suite_create ("EdgeFirst Meta Copy");

  TCase *tc_copy = tcase_create ("Copy");
  tcase_add_test (tc_copy, test_pointcloud2_meta_copy);
  tcase_add_test (tc_copy, test_radar_cube_meta_copy);
  tcase_add_test (tc_copy, test_camera_info_meta_copy);
  tcase_add_test (tc_copy, test_transform_meta_copy);
  suite_add_tcase (s, tc_copy);

  TCase *tc_misc = tcase_create ("MiscMeta");
  tcase_add_test (tc_misc, test_meta_absent_on_empty_buffer);
  tcase_add_test (tc_misc, test_multiple_meta_types_on_buffer);
  tcase_add_test (tc_misc, test_meta_init_defaults);
  suite_add_tcase (s, tc_misc);

  return s;
}

GST_CHECK_MAIN (edgefirst_meta_copy);
