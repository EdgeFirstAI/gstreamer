/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstcamerainfometa.h"
#include <string.h>

GType
edgefirst_camera_info_meta_api_get_type (void)
{
  static GType type = 0;
  static const gchar *tags[] = { NULL };

  if (g_once_init_enter (&type)) {
    GType _type = gst_meta_api_type_register ("EdgefirstCameraInfoMetaAPI", tags);
    g_once_init_leave (&type, _type);
  }
  return type;
}

static gboolean
edgefirst_camera_info_meta_init (GstMeta *meta, gpointer params, GstBuffer *buffer)
{
  EdgefirstCameraInfoMeta *ci_meta = (EdgefirstCameraInfoMeta *) meta;

  ci_meta->width = 0;
  ci_meta->height = 0;
  memset (ci_meta->K, 0, sizeof (ci_meta->K));
  memset (ci_meta->D, 0, sizeof (ci_meta->D));
  ci_meta->num_distortion_coeffs = 0;
  ci_meta->distortion_model = EDGEFIRST_DISTORTION_NONE;
  memset (ci_meta->R, 0, sizeof (ci_meta->R));
  memset (ci_meta->P, 0, sizeof (ci_meta->P));
  ci_meta->frame_id[0] = '\0';

  return TRUE;
}

static void
edgefirst_camera_info_meta_free (GstMeta *meta, GstBuffer *buffer)
{
  /* Nothing to free */
}

static gboolean
edgefirst_camera_info_meta_transform (GstBuffer *dest, GstMeta *meta,
    GstBuffer *buffer, GQuark type, gpointer data)
{
  EdgefirstCameraInfoMeta *src_meta = (EdgefirstCameraInfoMeta *) meta;
  EdgefirstCameraInfoMeta *dest_meta;

  if (GST_META_TRANSFORM_IS_COPY (type)) {
    dest_meta = edgefirst_buffer_add_camera_info_meta (dest);
    if (!dest_meta)
      return FALSE;

    dest_meta->width = src_meta->width;
    dest_meta->height = src_meta->height;
    memcpy (dest_meta->K, src_meta->K, sizeof (src_meta->K));
    memcpy (dest_meta->D, src_meta->D, sizeof (src_meta->D));
    dest_meta->num_distortion_coeffs = src_meta->num_distortion_coeffs;
    dest_meta->distortion_model = src_meta->distortion_model;
    memcpy (dest_meta->R, src_meta->R, sizeof (src_meta->R));
    memcpy (dest_meta->P, src_meta->P, sizeof (src_meta->P));
    memcpy (dest_meta->frame_id, src_meta->frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);

    return TRUE;
  }

  return FALSE;
}

const GstMetaInfo *
edgefirst_camera_info_meta_get_info (void)
{
  static const GstMetaInfo *info = NULL;

  if (g_once_init_enter (&info)) {
    const GstMetaInfo *meta_info = gst_meta_register (
        EDGEFIRST_CAMERA_INFO_META_API_TYPE,
        "EdgefirstCameraInfoMeta",
        sizeof (EdgefirstCameraInfoMeta),
        edgefirst_camera_info_meta_init,
        edgefirst_camera_info_meta_free,
        edgefirst_camera_info_meta_transform);
    g_once_init_leave (&info, meta_info);
  }
  return info;
}

EdgefirstCameraInfoMeta *
edgefirst_buffer_add_camera_info_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstCameraInfoMeta *) gst_buffer_add_meta (buffer,
      EDGEFIRST_CAMERA_INFO_META_INFO, NULL);
}

EdgefirstCameraInfoMeta *
edgefirst_buffer_get_camera_info_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstCameraInfoMeta *) gst_buffer_get_meta (buffer,
      EDGEFIRST_CAMERA_INFO_META_API_TYPE);
}

void
edgefirst_camera_info_meta_set_identity (EdgefirstCameraInfoMeta *meta,
    guint32 width, guint32 height)
{
  g_return_if_fail (meta != NULL);

  meta->width = width;
  meta->height = height;

  /* Identity K matrix with principal point at center */
  memset (meta->K, 0, sizeof (meta->K));
  meta->K[0] = (gdouble) width;   /* fx */
  meta->K[4] = (gdouble) height;  /* fy */
  meta->K[2] = (gdouble) width / 2.0;   /* cx */
  meta->K[5] = (gdouble) height / 2.0;  /* cy */
  meta->K[8] = 1.0;

  /* No distortion */
  memset (meta->D, 0, sizeof (meta->D));
  meta->num_distortion_coeffs = 0;
  meta->distortion_model = EDGEFIRST_DISTORTION_NONE;

  /* Identity R matrix */
  memset (meta->R, 0, sizeof (meta->R));
  meta->R[0] = 1.0;
  meta->R[4] = 1.0;
  meta->R[8] = 1.0;

  /* P = [K | 0] */
  memset (meta->P, 0, sizeof (meta->P));
  meta->P[0] = meta->K[0];
  meta->P[2] = meta->K[2];
  meta->P[5] = meta->K[4];
  meta->P[6] = meta->K[5];
  meta->P[10] = 1.0;
}

gboolean
edgefirst_camera_info_meta_project_point (const EdgefirstCameraInfoMeta *meta,
    gdouble x, gdouble y, gdouble z, gdouble *u, gdouble *v)
{
  g_return_val_if_fail (meta != NULL, FALSE);
  g_return_val_if_fail (u != NULL && v != NULL, FALSE);

  if (z <= 0.0)
    return FALSE;

  /* Project using K matrix: [u, v, 1]^T = K * [x/z, y/z, 1]^T */
  gdouble x_norm = x / z;
  gdouble y_norm = y / z;

  *u = meta->K[0] * x_norm + meta->K[2];
  *v = meta->K[4] * y_norm + meta->K[5];

  return TRUE;
}
