/*
 * EdgeFirst Perception for GStreamer - Overlay Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_OVERLAY_H__
#define __EDGEFIRST_OVERLAY_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <edgefirst/hal.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_OVERLAY (edgefirst_overlay_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstOverlay, edgefirst_overlay,
    EDGEFIRST, OVERLAY, GstBaseTransform)

/**
 * edgefirst_overlay_set_results:
 * @overlay: the overlay element
 * @boxes: (transfer full) (nullable): detection boxes (takes ownership)
 * @segmentations: (transfer full) (nullable): segmentation masks (takes ownership)
 *
 * Provide the latest detection/segmentation results from the NN branch.
 * Thread-safe — may be called from any thread (e.g. tensor_sink callback).
 * Frees any previously stored results before replacing them.
 */
void edgefirst_overlay_set_results (EdgefirstOverlay *overlay,
    hal_detect_box_list *boxes,
    hal_segmentation_list *segmentations);

G_END_DECLS

#endif /* __EDGEFIRST_OVERLAY_H__ */
