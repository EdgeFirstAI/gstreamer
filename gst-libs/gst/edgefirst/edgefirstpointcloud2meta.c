/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstpointcloud2meta.h"
#include <string.h>

GType
edgefirst_pointcloud2_meta_api_get_type (void)
{
  static GType type = 0;
  static const gchar *tags[] = {
    GST_META_TAG_MEMORY_STR,
    NULL
  };

  if (g_once_init_enter (&type)) {
    GType _type = gst_meta_api_type_register ("EdgefirstPointCloud2MetaAPI", tags);
    g_once_init_leave (&type, _type);
  }
  return type;
}

static gboolean
edgefirst_pointcloud2_meta_init (GstMeta *meta, gpointer params, GstBuffer *buffer)
{
  EdgefirstPointCloud2Meta *pc_meta = (EdgefirstPointCloud2Meta *) meta;

  pc_meta->point_count = 0;
  pc_meta->frame_id[0] = '\0';
  pc_meta->ros_timestamp_ns = 0;
  pc_meta->has_transform = FALSE;
  memset (&pc_meta->transform, 0, sizeof (EdgefirstTransformData));

  return TRUE;
}

static void
edgefirst_pointcloud2_meta_free (GstMeta *meta, GstBuffer *buffer)
{
  /* Nothing to free - all data is inline */
}

static gboolean
edgefirst_pointcloud2_meta_transform (GstBuffer *dest, GstMeta *meta,
    GstBuffer *buffer, GQuark type, gpointer data)
{
  EdgefirstPointCloud2Meta *src_meta = (EdgefirstPointCloud2Meta *) meta;
  EdgefirstPointCloud2Meta *dest_meta;

  if (GST_META_TRANSFORM_IS_COPY (type)) {
    dest_meta = edgefirst_buffer_add_pointcloud2_meta (dest);
    if (!dest_meta)
      return FALSE;

    dest_meta->point_count = src_meta->point_count;
    memcpy (dest_meta->frame_id, src_meta->frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
    dest_meta->ros_timestamp_ns = src_meta->ros_timestamp_ns;
    dest_meta->has_transform = src_meta->has_transform;
    memcpy (&dest_meta->transform, &src_meta->transform, sizeof (EdgefirstTransformData));

    return TRUE;
  }

  return FALSE;
}

const GstMetaInfo *
edgefirst_pointcloud2_meta_get_info (void)
{
  static const GstMetaInfo *info = NULL;

  if (g_once_init_enter (&info)) {
    const GstMetaInfo *meta_info = gst_meta_register (
        EDGEFIRST_POINTCLOUD2_META_API_TYPE,
        "EdgefirstPointCloud2Meta",
        sizeof (EdgefirstPointCloud2Meta),
        edgefirst_pointcloud2_meta_init,
        edgefirst_pointcloud2_meta_free,
        edgefirst_pointcloud2_meta_transform);
    g_once_init_leave (&info, meta_info);
  }
  return info;
}

EdgefirstPointCloud2Meta *
edgefirst_buffer_add_pointcloud2_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstPointCloud2Meta *) gst_buffer_add_meta (buffer,
      EDGEFIRST_POINTCLOUD2_META_INFO, NULL);
}

EdgefirstPointCloud2Meta *
edgefirst_buffer_get_pointcloud2_meta (GstBuffer *buffer)
{
  g_return_val_if_fail (GST_IS_BUFFER (buffer), NULL);

  return (EdgefirstPointCloud2Meta *) gst_buffer_get_meta (buffer,
      EDGEFIRST_POINTCLOUD2_META_API_TYPE);
}

/* ── Point Field Utilities ──────────────────────────────────────────── */

static const struct {
  guint8 datatype;
  const gchar *short_name;
  const gchar *long_name;
  guint size;
} _datatype_table[] = {
  { EDGEFIRST_POINT_FIELD_INT8,    "I8",  "INT8",    1 },
  { EDGEFIRST_POINT_FIELD_UINT8,   "U8",  "UINT8",   1 },
  { EDGEFIRST_POINT_FIELD_INT16,   "I16", "INT16",   2 },
  { EDGEFIRST_POINT_FIELD_UINT16,  "U16", "UINT16",  2 },
  { EDGEFIRST_POINT_FIELD_INT32,   "I32", "INT32",   4 },
  { EDGEFIRST_POINT_FIELD_UINT32,  "U32", "UINT32",  4 },
  { EDGEFIRST_POINT_FIELD_FLOAT32, "F32", "FLOAT32", 4 },
  { EDGEFIRST_POINT_FIELD_FLOAT64, "F64", "FLOAT64", 8 },
};

const gchar *
edgefirst_point_field_datatype_to_string (guint8 datatype)
{
  for (guint i = 0; i < G_N_ELEMENTS (_datatype_table); i++) {
    if (_datatype_table[i].datatype == datatype)
      return _datatype_table[i].short_name;
  }
  return "UNKNOWN";
}

guint8
edgefirst_point_field_datatype_from_string (const gchar *str)
{
  if (!str)
    return 0;

  for (guint i = 0; i < G_N_ELEMENTS (_datatype_table); i++) {
    if (g_ascii_strcasecmp (str, _datatype_table[i].short_name) == 0 ||
        g_ascii_strcasecmp (str, _datatype_table[i].long_name) == 0)
      return _datatype_table[i].datatype;
  }
  return 0;
}

guint
edgefirst_point_field_datatype_size (guint8 datatype)
{
  for (guint i = 0; i < G_N_ELEMENTS (_datatype_table); i++) {
    if (_datatype_table[i].datatype == datatype)
      return _datatype_table[i].size;
  }
  return 0;
}

guint
edgefirst_parse_point_fields (const gchar *fields_str,
    EdgefirstPointFieldDesc *out_fields, guint max_fields)
{
  gchar **tokens;
  guint count = 0;

  if (!fields_str || !out_fields || max_fields == 0)
    return 0;

  tokens = g_strsplit (fields_str, ",", -1);

  for (guint i = 0; tokens[i] && count < max_fields; i++) {
    gchar **parts;
    gchar *trimmed = g_strstrip (tokens[i]);

    if (trimmed[0] == '\0')
      continue;

    parts = g_strsplit (trimmed, ":", -1);
    guint num_parts = g_strv_length (parts);

    if (num_parts >= 3) {
      EdgefirstPointFieldDesc *f = &out_fields[count];

      g_strlcpy (f->name, g_strstrip (parts[0]), sizeof (f->name));
      f->datatype = edgefirst_point_field_datatype_from_string (
          g_strstrip (parts[1]));
      f->offset = (guint32) g_ascii_strtoull (g_strstrip (parts[2]), NULL, 10);
      f->count = 1;

      if (f->datatype != 0 && f->name[0] != '\0')
        count++;
    }

    g_strfreev (parts);
  }

  g_strfreev (tokens);
  return count;
}

gchar *
edgefirst_format_point_fields (const EdgefirstPointFieldDesc *fields,
    guint num_fields)
{
  GString *s;

  if (!fields || num_fields == 0)
    return g_strdup ("");

  s = g_string_new (NULL);

  for (guint i = 0; i < num_fields; i++) {
    if (i > 0)
      g_string_append_c (s, ',');

    g_string_append_printf (s, "%s:%s:%u",
        fields[i].name,
        edgefirst_point_field_datatype_to_string (fields[i].datatype),
        fields[i].offset);
  }

  return g_string_free (s, FALSE);
}
