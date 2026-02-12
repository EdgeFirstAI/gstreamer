/*
 * EdgeFirst Perception for GStreamer - Metadata Tests
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gst/check/gstcheck.h>
#include <gst/edgefirst/edgefirst.h>

GST_START_TEST (test_pointcloud2_meta_create)
{
  GstBuffer *buffer;
  EdgefirstPointCloud2Meta *meta;

  edgefirst_perception_init ();

  buffer = gst_buffer_new ();
  fail_unless (buffer != NULL);

  meta = edgefirst_buffer_add_pointcloud2_meta (buffer);
  fail_unless (meta != NULL);

  /* Check default values */
  fail_unless_equals_int (meta->point_count, 0);
  fail_unless_equals_int (meta->has_transform, FALSE);

  /* Set some values */
  meta->point_count = 1000;
  g_strlcpy (meta->frame_id, "velodyne", EDGEFIRST_FRAME_ID_MAX_LEN);

  /* Verify retrieval */
  EdgefirstPointCloud2Meta *retrieved = edgefirst_buffer_get_pointcloud2_meta (buffer);
  fail_unless (retrieved != NULL);
  fail_unless_equals_int (retrieved->point_count, 1000);
  fail_unless_equals_string (retrieved->frame_id, "velodyne");

  gst_buffer_unref (buffer);
}
GST_END_TEST;

GST_START_TEST (test_radar_cube_meta_create)
{
  GstBuffer *buffer;
  EdgefirstRadarCubeMeta *meta;

  edgefirst_perception_init ();

  buffer = gst_buffer_new ();
  fail_unless (buffer != NULL);

  meta = edgefirst_buffer_add_radar_cube_meta (buffer);
  fail_unless (meta != NULL);

  /* Set up a range-doppler cube */
  meta->layout[0] = EDGEFIRST_RADAR_DIM_RANGE;
  meta->layout[1] = EDGEFIRST_RADAR_DIM_DOPPLER;
  meta->num_dims = 2;
  meta->scales[0] = 0.1f;  /* 10cm per range bin */
  meta->scales[1] = 0.5f;  /* 0.5 m/s per doppler bin */
  meta->is_complex = TRUE;

  /* Verify */
  fail_unless_equals_int (meta->layout[0], EDGEFIRST_RADAR_DIM_RANGE);
  fail_unless_equals_int (meta->num_dims, 2);
  fail_unless (meta->is_complex == TRUE);

  gst_buffer_unref (buffer);
}
GST_END_TEST;

GST_START_TEST (test_camera_info_meta_project)
{
  GstBuffer *buffer;
  EdgefirstCameraInfoMeta *meta;
  gdouble u, v;
  gboolean valid;

  edgefirst_perception_init ();

  buffer = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buffer);
  fail_unless (meta != NULL);

  /* Set up identity camera with 640x480 resolution */
  edgefirst_camera_info_meta_set_identity (meta, 640, 480);

  /* Test projection of a point at (1, 0, 2) - should be right of center */
  valid = edgefirst_camera_info_meta_project_point (meta, 1.0, 0.0, 2.0, &u, &v);
  fail_unless (valid == TRUE);
  fail_unless (u > 320.0);  /* Right of center */

  /* Test point behind camera */
  valid = edgefirst_camera_info_meta_project_point (meta, 0.0, 0.0, -1.0, &u, &v);
  fail_unless (valid == FALSE);

  gst_buffer_unref (buffer);
}
GST_END_TEST;

GST_START_TEST (test_transform_apply)
{
  EdgefirstTransformData transform;
  gdouble x, y, z;

  edgefirst_perception_init ();

  edgefirst_transform_data_set_identity (&transform);

  /* Add translation */
  transform.translation[0] = 1.0;
  transform.translation[1] = 2.0;
  transform.translation[2] = 3.0;

  /* Apply to origin */
  x = 0.0; y = 0.0; z = 0.0;
  edgefirst_transform_data_apply (&transform, &x, &y, &z);

  fail_unless_equals_float (x, 1.0);
  fail_unless_equals_float (y, 2.0);
  fail_unless_equals_float (z, 3.0);
}
GST_END_TEST;

GST_START_TEST (test_radar_dimension_to_string)
{
  edgefirst_perception_init ();

  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_RANGE), "RANGE");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_DOPPLER), "DOPPLER");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_AZIMUTH), "AZIMUTH");
  fail_unless_equals_string (edgefirst_radar_dimension_to_string (EDGEFIRST_RADAR_DIM_UNDEFINED), "UNDEFINED");
}
GST_END_TEST;

static Suite *
edgefirst_meta_suite (void)
{
  Suite *s = suite_create ("EdgeFirst Meta");
  TCase *tc_core = tcase_create ("Core");

  tcase_add_test (tc_core, test_pointcloud2_meta_create);
  tcase_add_test (tc_core, test_radar_cube_meta_create);
  tcase_add_test (tc_core, test_camera_info_meta_project);
  tcase_add_test (tc_core, test_transform_apply);
  tcase_add_test (tc_core, test_radar_dimension_to_string);

  suite_add_tcase (s, tc_core);
  return s;
}

GST_CHECK_MAIN (edgefirst_meta);
