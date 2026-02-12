/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_TRANSFORM_META_H__
#define __EDGEFIRST_TRANSFORM_META_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst-perception-types.h>
#include <gst/edgefirst/edgefirstpointcloud2meta.h>  /* For EdgefirstTransformData */

G_BEGIN_DECLS

#define EDGEFIRST_TRANSFORM_META_API_TYPE (edgefirst_transform_meta_api_get_type())
#define EDGEFIRST_TRANSFORM_META_INFO     (edgefirst_transform_meta_get_info())

/**
 * EdgefirstTransformMeta:
 * @meta: Parent GstMeta
 * @transform: The rigid body transform
 *
 * Metadata for attaching coordinate frame transforms to buffers.
 */
typedef struct _EdgefirstTransformMeta {
  GstMeta meta;
  EdgefirstTransformData transform;
} EdgefirstTransformMeta;

GType edgefirst_transform_meta_api_get_type (void);
const GstMetaInfo *edgefirst_transform_meta_get_info (void);

/**
 * edgefirst_buffer_add_transform_meta:
 * @buffer: a #GstBuffer
 *
 * Adds a #EdgefirstTransformMeta to the buffer.
 *
 * Returns: (transfer none): the #EdgefirstTransformMeta added to @buffer
 */
EdgefirstTransformMeta *edgefirst_buffer_add_transform_meta (GstBuffer *buffer);

/**
 * edgefirst_buffer_get_transform_meta:
 * @buffer: a #GstBuffer
 *
 * Gets the #EdgefirstTransformMeta from the buffer.
 *
 * Returns: (transfer none) (nullable): the #EdgefirstTransformMeta or %NULL
 */
EdgefirstTransformMeta *edgefirst_buffer_get_transform_meta (GstBuffer *buffer);

/**
 * edgefirst_transform_data_set_identity:
 * @transform: a #EdgefirstTransformData
 *
 * Sets the transform to identity (no translation, no rotation).
 */
void edgefirst_transform_data_set_identity (EdgefirstTransformData *transform);

/**
 * edgefirst_transform_data_apply:
 * @transform: a #EdgefirstTransformData
 * @x: (inout): x coordinate
 * @y: (inout): y coordinate
 * @z: (inout): z coordinate
 *
 * Applies the transform to a 3D point in place.
 */
void edgefirst_transform_data_apply (const EdgefirstTransformData *transform,
    gdouble *x, gdouble *y, gdouble *z);

G_END_DECLS

#endif /* __EDGEFIRST_TRANSFORM_META_H__ */
