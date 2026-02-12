/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_H__
#define __EDGEFIRST_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst-perception-types.h>
#include <gst/edgefirst/edgefirstpointcloud2meta.h>
#include <gst/edgefirst/edgefirstradarcubemeta.h>
#include <gst/edgefirst/edgefirsttransformmeta.h>
#include <gst/edgefirst/edgefirstcamerainfometa.h>

G_BEGIN_DECLS

/**
 * edgefirst_perception_init:
 *
 * Initialize the EdgeFirst Perception library. This registers all
 * custom caps types and metadata types with GStreamer.
 *
 * This function is safe to call multiple times.
 */
void edgefirst_perception_init (void);

/**
 * edgefirst_perception_version:
 *
 * Returns the version string of the EdgeFirst Perception library.
 *
 * Returns: (transfer none): a static version string
 */
const gchar *edgefirst_perception_version (void);

/**
 * GST_CAPS_FEATURE_MEMORY_DMABUF:
 *
 * Caps feature for DMA buffer memory. Re-exported for convenience.
 */
#ifndef GST_CAPS_FEATURE_MEMORY_DMABUF
#define GST_CAPS_FEATURE_MEMORY_DMABUF "memory:DMABuf"
#endif

/**
 * EDGEFIRST_POINTCLOUD2_CAPS:
 *
 * Template caps string for PointCloud2 data.
 */
#define EDGEFIRST_POINTCLOUD2_CAPS \
    "application/x-pointcloud2, " \
    "width = (int) [ 1, MAX ], " \
    "height = (int) [ 1, MAX ], " \
    "point-step = (int) [ 1, MAX ], " \
    "fields = (string) ANY, " \
    "is-bigendian = (boolean) { true, false }, " \
    "is-dense = (boolean) { true, false }"

G_END_DECLS

#endif /* __EDGEFIRST_H__ */
