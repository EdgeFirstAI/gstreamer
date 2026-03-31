/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirst.h"

static gboolean _initialized = FALSE;
static GMutex _init_mutex;

/**
 * edgefirst_perception_init:
 *
 * Initialize the EdgeFirst Perception library.
 */
void
edgefirst_perception_init (void)
{
  g_mutex_lock (&_init_mutex);

  if (_initialized) {
    g_mutex_unlock (&_init_mutex);
    return;
  }

  /* Register metadata types */
  edgefirst_pointcloud2_meta_get_info ();
  edgefirst_radar_cube_meta_get_info ();
  edgefirst_transform_meta_get_info ();
  edgefirst_camera_info_meta_get_info ();

  /* Ensure detection GTypes are registered */
  edgefirst_detect_box_get_type ();
  edgefirst_detect_box_list_get_type ();
  edgefirst_segmentation_get_type ();
  edgefirst_segmentation_list_get_type ();
  edgefirst_color_mode_get_type ();

  _initialized = TRUE;

  g_mutex_unlock (&_init_mutex);
}

const gchar *
edgefirst_perception_version (void)
{
  return PACKAGE_VERSION;
}
