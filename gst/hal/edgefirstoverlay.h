/*
 * EdgeFirst Perception for GStreamer - Overlay Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_OVERLAY_H__
#define __EDGEFIRST_OVERLAY_H__

#include <gst/gst.h>
#include <gst/edgefirst/edgefirstdetection.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_OVERLAY (edgefirst_overlay_get_type ())
G_DECLARE_FINAL_TYPE (EdgefirstOverlay, edgefirst_overlay,
    EDGEFIRST, OVERLAY, GstElement)

G_END_DECLS

#endif /* __EDGEFIRST_OVERLAY_H__ */
