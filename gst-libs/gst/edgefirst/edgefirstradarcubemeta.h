/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_RADAR_CUBE_META_H__
#define __EDGEFIRST_RADAR_CUBE_META_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst-perception-types.h>

G_BEGIN_DECLS

#define EDGEFIRST_RADAR_CUBE_META_API_TYPE (edgefirst_radar_cube_meta_api_get_type())
#define EDGEFIRST_RADAR_CUBE_META_INFO     (edgefirst_radar_cube_meta_get_info())

/**
 * EdgefirstRadarCubeMeta:
 * @meta: Parent GstMeta
 * @layout: Dimension labels (RANGE, DOPPLER, AZIMUTH, etc.)
 * @num_dims: Number of dimensions in the cube
 * @scales: Scaling factors for physical units (meters/bin, m/s per bin, etc.)
 * @is_complex: TRUE if data contains complex values (real, imaginary pairs)
 * @radar_timestamp: Radar frame timestamp from module
 * @frame_id: Coordinate frame identifier
 *
 * Metadata for RadarCube tensor buffers.
 * The actual tensor data is stored in NNStreamer tensor format.
 */
typedef struct _EdgefirstRadarCubeMeta {
  GstMeta meta;

  EdgefirstRadarDimension layout[EDGEFIRST_RADAR_MAX_DIMS];
  guint8 num_dims;

  gfloat scales[EDGEFIRST_RADAR_MAX_DIMS];

  gboolean is_complex;

  guint64 radar_timestamp;

  gchar frame_id[EDGEFIRST_FRAME_ID_MAX_LEN];
} EdgefirstRadarCubeMeta;

GType edgefirst_radar_cube_meta_api_get_type (void);
const GstMetaInfo *edgefirst_radar_cube_meta_get_info (void);

/**
 * edgefirst_buffer_add_radar_cube_meta:
 * @buffer: a #GstBuffer
 *
 * Adds a #EdgefirstRadarCubeMeta to the buffer.
 *
 * Returns: (transfer none): the #EdgefirstRadarCubeMeta added to @buffer
 */
EdgefirstRadarCubeMeta *edgefirst_buffer_add_radar_cube_meta (GstBuffer *buffer);

/**
 * edgefirst_buffer_get_radar_cube_meta:
 * @buffer: a #GstBuffer
 *
 * Gets the #EdgefirstRadarCubeMeta from the buffer.
 *
 * Returns: (transfer none) (nullable): the #EdgefirstRadarCubeMeta or %NULL
 */
EdgefirstRadarCubeMeta *edgefirst_buffer_get_radar_cube_meta (GstBuffer *buffer);

/**
 * edgefirst_radar_dimension_to_string:
 * @dim: a #EdgefirstRadarDimension
 *
 * Returns a string representation of the dimension.
 *
 * Returns: (transfer none): a static string
 */
const gchar *edgefirst_radar_dimension_to_string (EdgefirstRadarDimension dim);

G_END_DECLS

#endif /* __EDGEFIRST_RADAR_CUBE_META_H__ */
