/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstradarcubemeta.h"
#include <string.h>

GType
edgefirst_radar_cube_meta_api_get_type (void)
{
  static GType type = 0;
  static const gchar *tags[] = {
    GST_META_TAG_MEMORY_STR,
    NULL
  };

  if (g_once_init_enter (&type)) {
    GType _type = gst_meta_api_type_register ("EdgefirstRadarCubeMetaAPI", tags);
    g_once_init_leave (&type, _type);
  }
  return type;
}

static gboolean
edgefirst_radar_cube_meta_init (GstMeta *meta, gpointer params, GstBuffer *buffer)
{
  EdgefirstRadarCubeMeta *rc_meta = (EdgefirstRadarCubeMeta *) meta;

  memset (rc_meta->layout, 0, sizeof (rc_meta->layout));
  rc_meta->num_dims = 0;
  memset (rc_meta->scales, 0, sizeof (rc_meta->scales));
  rc_meta->is_complex = FALSE;
  rc_meta->radar_timestamp = 0;
  rc_meta->frame_id[0] = '\0';

  return TRUE;
}

static void
edgefirst_radar_cube_meta_free (GstMeta *meta, GstBuffer *buffer)
{
  /* Nothing to free - all data is inline */
}

static gboolean
edgefirst_radar_cube_meta_transform (GstBuffer *dest, GstMeta *meta,
    GstBuffer *buffer, GQuark type, gpointer data)
{
  EdgefirstRadarCubeMeta *src_meta = (EdgefirstRadarCubeMeta *) meta;
  EdgefirstRadarCubeMeta *dest_meta;

  if (GST_META_TRANSFORM_IS_COPY (type)) {
    dest_meta = edgefirst_buffer_add_radar_cube_meta (dest);
    if (!dest_meta)
      return FALSE;

    memcpy (dest_meta->layout, src_meta->layout, sizeof (src_meta->layout));
    dest_meta->num_dims = src_meta->num_dims;
    memcpy (dest_meta->scales, src_meta->scales, sizeof (src_meta->scales));
    dest_meta->is_complex = src_meta->is_complex;
    dest_meta->radar_timestamp = src_meta->radar_timestamp;
    memcpy (dest_meta->frame_id, src_meta->frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);

    return TRUE;
  }

  return FALSE;
}

const GstMetaInfo *
edgefirst_radar_cube_meta_get_info (void)
{
  static const GstMetaInfo *info = NULL;

  if (g_once_init_enter (&info)) {
    const GstMetaInfo *meta_info = gst_meta_register (
        EDGEFIRST_RADAR_CUBE_META_API_TYPE,
        "EdgefirstRadarCubeMeta",
        sizeof (EdgefirstRadarCubeMeta),
        edgefirst_radar_cube_meta_init,
        edgefirst_radar_cube_meta_free,
        edgefirst_radar_cube_meta_transform);
    g_once_init_leave (&info, meta_info);
  }
  return info;
}

EdgefirstRadarCubeMeta *
edgefirst_buffer_add_radar_cube_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstRadarCubeMeta *) gst_buffer_add_meta (buffer,
      EDGEFIRST_RADAR_CUBE_META_INFO, NULL);
}

EdgefirstRadarCubeMeta *
edgefirst_buffer_get_radar_cube_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstRadarCubeMeta *) gst_buffer_get_meta (buffer,
      EDGEFIRST_RADAR_CUBE_META_API_TYPE);
}

const gchar *
edgefirst_radar_dimension_to_string (EdgefirstRadarDimension dim)
{
  switch (dim) {
    case EDGEFIRST_RADAR_DIM_RANGE:
      return "RANGE";
    case EDGEFIRST_RADAR_DIM_DOPPLER:
      return "DOPPLER";
    case EDGEFIRST_RADAR_DIM_AZIMUTH:
      return "AZIMUTH";
    case EDGEFIRST_RADAR_DIM_ELEVATION:
      return "ELEVATION";
    case EDGEFIRST_RADAR_DIM_RXCHANNEL:
      return "RXCHANNEL";
    case EDGEFIRST_RADAR_DIM_SEQUENCE:
      return "SEQUENCE";
    case EDGEFIRST_RADAR_DIM_UNDEFINED:
    default:
      return "UNDEFINED";
  }
}
