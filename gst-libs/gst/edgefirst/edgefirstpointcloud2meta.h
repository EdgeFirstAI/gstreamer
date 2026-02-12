/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_POINTCLOUD2_META_H__
#define __EDGEFIRST_POINTCLOUD2_META_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst-perception-types.h>

G_BEGIN_DECLS

#define EDGEFIRST_POINTCLOUD2_META_API_TYPE (edgefirst_pointcloud2_meta_api_get_type())
#define EDGEFIRST_POINTCLOUD2_META_INFO     (edgefirst_pointcloud2_meta_get_info())

/**
 * EdgefirstTransformData:
 * @translation: Translation vector (x, y, z) in meters
 * @rotation: Rotation quaternion (x, y, z, w)
 * @child_frame_id: Source coordinate frame identifier
 * @parent_frame_id: Target/reference coordinate frame identifier
 * @timestamp_ns: Timestamp for time-varying transforms (nanoseconds)
 *
 * Rigid body transform between coordinate frames.
 * Compatible with ROS2 geometry_msgs/TransformStamped.
 */
typedef struct _EdgefirstTransformData {
  gdouble translation[3];
  gdouble rotation[4];
  gchar child_frame_id[EDGEFIRST_FRAME_ID_MAX_LEN];
  gchar parent_frame_id[EDGEFIRST_FRAME_ID_MAX_LEN];
  guint64 timestamp_ns;
} EdgefirstTransformData;

/**
 * EdgefirstPointCloud2Meta:
 * @meta: Parent GstMeta
 * @point_count: Actual number of valid points in the buffer
 * @frame_id: Coordinate frame identifier for this point cloud
 * @ros_timestamp_ns: Original ROS2 timestamp (nanoseconds since epoch)
 * @has_transform: Whether transform data is valid
 * @transform: Transform to reference frame (if has_transform is TRUE)
 *
 * Metadata for PointCloud2 buffers.
 */
typedef struct _EdgefirstPointCloud2Meta {
  GstMeta meta;

  guint32 point_count;
  gchar frame_id[EDGEFIRST_FRAME_ID_MAX_LEN];
  guint64 ros_timestamp_ns;

  gboolean has_transform;
  EdgefirstTransformData transform;
} EdgefirstPointCloud2Meta;

GType edgefirst_pointcloud2_meta_api_get_type (void);
const GstMetaInfo *edgefirst_pointcloud2_meta_get_info (void);

/**
 * edgefirst_buffer_add_pointcloud2_meta:
 * @buffer: a #GstBuffer
 *
 * Adds a #EdgefirstPointCloud2Meta to the buffer.
 *
 * Returns: (transfer none): the #EdgefirstPointCloud2Meta added to @buffer
 */
EdgefirstPointCloud2Meta *edgefirst_buffer_add_pointcloud2_meta (GstBuffer *buffer);

/**
 * edgefirst_buffer_get_pointcloud2_meta:
 * @buffer: a #GstBuffer
 *
 * Gets the #EdgefirstPointCloud2Meta from the buffer.
 *
 * Returns: (transfer none) (nullable): the #EdgefirstPointCloud2Meta or %NULL
 */
EdgefirstPointCloud2Meta *edgefirst_buffer_get_pointcloud2_meta (GstBuffer *buffer);

/* Point field datatype constants (match ROS2 PointField) */
#define EDGEFIRST_POINT_FIELD_INT8     1
#define EDGEFIRST_POINT_FIELD_UINT8    2
#define EDGEFIRST_POINT_FIELD_INT16    3
#define EDGEFIRST_POINT_FIELD_UINT16   4
#define EDGEFIRST_POINT_FIELD_INT32    5
#define EDGEFIRST_POINT_FIELD_UINT32   6
#define EDGEFIRST_POINT_FIELD_FLOAT32  7
#define EDGEFIRST_POINT_FIELD_FLOAT64  8

/**
 * EdgefirstPointFieldDesc:
 * @name: Field name (e.g. "x", "y", "z", "intensity")
 * @datatype: EDGEFIRST_POINT_FIELD_* constant
 * @offset: Byte offset within a point
 * @count: Number of elements (typically 1)
 *
 * Descriptor for a single field within a point cloud point.
 */
typedef struct {
  gchar name[64];
  guint8 datatype;
  guint32 offset;
  guint32 count;
} EdgefirstPointFieldDesc;

/**
 * edgefirst_parse_point_fields:
 * @fields_str: Caps field string (e.g. "x:F32:0,y:F32:4,z:F32:8")
 * @out_fields: (out caller-allocates): Array to fill
 * @max_fields: Maximum number of fields to parse
 *
 * Parses a point cloud fields caps string into field descriptors.
 *
 * Returns: Number of fields parsed
 */
guint edgefirst_parse_point_fields (const gchar *fields_str,
    EdgefirstPointFieldDesc *out_fields, guint max_fields);

/**
 * edgefirst_format_point_fields:
 * @fields: Array of field descriptors
 * @num_fields: Number of fields
 *
 * Formats field descriptors into a caps-compatible string.
 *
 * Returns: (transfer full): Newly allocated string, caller must g_free()
 */
gchar *edgefirst_format_point_fields (const EdgefirstPointFieldDesc *fields,
    guint num_fields);

/**
 * edgefirst_point_field_datatype_to_string:
 * @datatype: EDGEFIRST_POINT_FIELD_* constant
 *
 * Returns: (transfer none): Short type string (e.g. "F32", "U8")
 */
const gchar *edgefirst_point_field_datatype_to_string (guint8 datatype);

/**
 * edgefirst_point_field_datatype_from_string:
 * @str: Type string (e.g. "F32", "FLOAT32", "U8", "UINT8")
 *
 * Returns: EDGEFIRST_POINT_FIELD_* constant, or 0 if unknown
 */
guint8 edgefirst_point_field_datatype_from_string (const gchar *str);

/**
 * edgefirst_point_field_datatype_size:
 * @datatype: EDGEFIRST_POINT_FIELD_* constant
 *
 * Returns: Size in bytes of the datatype, or 0 if unknown
 */
guint edgefirst_point_field_datatype_size (guint8 datatype);

G_END_DECLS

#endif /* __EDGEFIRST_POINTCLOUD2_META_H__ */
