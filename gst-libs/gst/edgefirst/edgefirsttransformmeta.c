/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirsttransformmeta.h"
#include <string.h>
#include <math.h>

GType
edgefirst_transform_meta_api_get_type (void)
{
  static GType type = 0;
  static const gchar *tags[] = { NULL };

  if (g_once_init_enter (&type)) {
    GType _type = gst_meta_api_type_register ("EdgefirstTransformMetaAPI", tags);
    g_once_init_leave (&type, _type);
  }
  return type;
}

static gboolean
edgefirst_transform_meta_init (GstMeta *meta, gpointer params, GstBuffer *buffer)
{
  EdgefirstTransformMeta *t_meta = (EdgefirstTransformMeta *) meta;
  edgefirst_transform_data_set_identity (&t_meta->transform);
  return TRUE;
}

static void
edgefirst_transform_meta_free (GstMeta *meta, GstBuffer *buffer)
{
  /* Nothing to free */
}

static gboolean
edgefirst_transform_meta_transform (GstBuffer *dest, GstMeta *meta,
    GstBuffer *buffer, GQuark type, gpointer data)
{
  EdgefirstTransformMeta *src_meta = (EdgefirstTransformMeta *) meta;
  EdgefirstTransformMeta *dest_meta;

  if (GST_META_TRANSFORM_IS_COPY (type)) {
    dest_meta = edgefirst_buffer_add_transform_meta (dest);
    if (!dest_meta)
      return FALSE;

    memcpy (&dest_meta->transform, &src_meta->transform,
        sizeof (EdgefirstTransformData));
    return TRUE;
  }

  return FALSE;
}

const GstMetaInfo *
edgefirst_transform_meta_get_info (void)
{
  static const GstMetaInfo *info = NULL;

  if (g_once_init_enter (&info)) {
    const GstMetaInfo *meta_info = gst_meta_register (
        EDGEFIRST_TRANSFORM_META_API_TYPE,
        "EdgefirstTransformMeta",
        sizeof (EdgefirstTransformMeta),
        edgefirst_transform_meta_init,
        edgefirst_transform_meta_free,
        edgefirst_transform_meta_transform);
    g_once_init_leave (&info, meta_info);
  }
  return info;
}

EdgefirstTransformMeta *
edgefirst_buffer_add_transform_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstTransformMeta *) gst_buffer_add_meta (buffer,
      EDGEFIRST_TRANSFORM_META_INFO, NULL);
}

EdgefirstTransformMeta *
edgefirst_buffer_get_transform_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstTransformMeta *) gst_buffer_get_meta (buffer,
      EDGEFIRST_TRANSFORM_META_API_TYPE);
}

void
edgefirst_transform_data_set_identity (EdgefirstTransformData *transform)
{
  g_return_if_fail (transform != NULL);

  transform->translation[0] = 0.0;
  transform->translation[1] = 0.0;
  transform->translation[2] = 0.0;

  /* Identity quaternion: (0, 0, 0, 1) */
  transform->rotation[0] = 0.0;
  transform->rotation[1] = 0.0;
  transform->rotation[2] = 0.0;
  transform->rotation[3] = 1.0;

  transform->child_frame_id[0] = '\0';
  transform->parent_frame_id[0] = '\0';
  transform->timestamp_ns = 0;
}

void
edgefirst_transform_data_apply (const EdgefirstTransformData *transform,
    gdouble *x, gdouble *y, gdouble *z)
{
  g_return_if_fail (transform != NULL);
  g_return_if_fail (x != NULL && y != NULL && z != NULL);

  /* Extract quaternion components */
  gdouble qx = transform->rotation[0];
  gdouble qy = transform->rotation[1];
  gdouble qz = transform->rotation[2];
  gdouble qw = transform->rotation[3];

  /* Apply quaternion rotation: v' = q * v * q^(-1)
   * Using the optimized formula for unit quaternions */
  gdouble px = *x, py = *y, pz = *z;

  /* t = 2 * cross(q.xyz, v) */
  gdouble tx = 2.0 * (qy * pz - qz * py);
  gdouble ty = 2.0 * (qz * px - qx * pz);
  gdouble tz = 2.0 * (qx * py - qy * px);

  /* v' = v + q.w * t + cross(q.xyz, t) */
  *x = px + qw * tx + (qy * tz - qz * ty) + transform->translation[0];
  *y = py + qw * ty + (qz * tx - qx * tz) + transform->translation[1];
  *z = pz + qw * tz + (qx * ty - qy * tx) + transform->translation[2];
}
