/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_CAMERA_INFO_META_H__
#define __EDGEFIRST_CAMERA_INFO_META_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst-perception-types.h>

G_BEGIN_DECLS

#define EDGEFIRST_CAMERA_INFO_META_API_TYPE (edgefirst_camera_info_meta_api_get_type())
#define EDGEFIRST_CAMERA_INFO_META_INFO     (edgefirst_camera_info_meta_get_info())

/**
 * EdgefirstCameraInfoMeta:
 * @meta: Parent GstMeta
 * @width: Image width this calibration applies to
 * @height: Image height this calibration applies to
 * @K: Intrinsic camera matrix (3x3, row-major)
 * @D: Distortion coefficients (up to 12)
 * @num_distortion_coeffs: Number of valid distortion coefficients
 * @distortion_model: The distortion model used
 * @R: Rectification matrix (3x3, row-major, identity if not stereo)
 * @P: Projection matrix (3x4, row-major)
 * @frame_id: Coordinate frame identifier
 *
 * Camera intrinsic calibration metadata.
 * Compatible with ROS2 sensor_msgs/CameraInfo.
 */
typedef struct _EdgefirstCameraInfoMeta {
  GstMeta meta;

  guint32 width;
  guint32 height;

  gdouble K[9];

  gdouble D[EDGEFIRST_MAX_DISTORTION_COEFFS];
  guint8 num_distortion_coeffs;

  EdgefirstDistortionModel distortion_model;

  gdouble R[9];

  gdouble P[12];

  gchar frame_id[EDGEFIRST_FRAME_ID_MAX_LEN];
} EdgefirstCameraInfoMeta;

GType edgefirst_camera_info_meta_api_get_type (void);
const GstMetaInfo *edgefirst_camera_info_meta_get_info (void);

/**
 * edgefirst_buffer_add_camera_info_meta:
 * @buffer: a #GstBuffer
 *
 * Adds a #EdgefirstCameraInfoMeta to the buffer.
 *
 * Returns: (transfer none): the #EdgefirstCameraInfoMeta added to @buffer
 */
EdgefirstCameraInfoMeta *edgefirst_buffer_add_camera_info_meta (GstBuffer *buffer);

/**
 * edgefirst_buffer_get_camera_info_meta:
 * @buffer: a #GstBuffer
 *
 * Gets the #EdgefirstCameraInfoMeta from the buffer.
 *
 * Returns: (transfer none) (nullable): the #EdgefirstCameraInfoMeta or %NULL
 */
EdgefirstCameraInfoMeta *edgefirst_buffer_get_camera_info_meta (GstBuffer *buffer);

/**
 * edgefirst_camera_info_meta_set_identity:
 * @meta: a #EdgefirstCameraInfoMeta
 * @width: image width
 * @height: image height
 *
 * Sets the camera info to identity (no distortion, centered principal point).
 */
void edgefirst_camera_info_meta_set_identity (EdgefirstCameraInfoMeta *meta,
    guint32 width, guint32 height);

/**
 * edgefirst_camera_info_meta_project_point:
 * @meta: a #EdgefirstCameraInfoMeta
 * @x: 3D point x coordinate (in camera frame)
 * @y: 3D point y coordinate
 * @z: 3D point z coordinate
 * @u: (out): projected pixel x coordinate
 * @v: (out): projected pixel y coordinate
 *
 * Projects a 3D point to 2D image coordinates using the camera intrinsics.
 * Does not apply distortion.
 *
 * Returns: TRUE if the point is in front of the camera (z > 0)
 */
gboolean edgefirst_camera_info_meta_project_point (const EdgefirstCameraInfoMeta *meta,
    gdouble x, gdouble y, gdouble z, gdouble *u, gdouble *v);

G_END_DECLS

#endif /* __EDGEFIRST_CAMERA_INFO_META_H__ */
