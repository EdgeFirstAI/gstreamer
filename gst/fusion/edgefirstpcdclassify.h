/*
 * EdgeFirst Perception for GStreamer - Point Cloud Classify Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_PCD_CLASSIFY_H__
#define __EDGEFIRST_PCD_CLASSIFY_H__

#include <gst/gst.h>
#include <gst/base/gstaggregator.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_PCD_CLASSIFY (edgefirst_pcd_classify_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstPcdClassify, edgefirst_pcd_classify,
    EDGEFIRST, PCD_CLASSIFY, GstAggregator)

/**
 * EdgefirstPcdClassifyOutputMode:
 * @EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS: Output integer labels
 * @EDGEFIRST_PCD_CLASSIFY_OUTPUT_COLORS: Output RGB colors
 * @EDGEFIRST_PCD_CLASSIFY_OUTPUT_BOTH: Output both labels and colors
 *
 * Output modes for point cloud classification.
 *
 * Since: 0.1
 */
typedef enum {
  EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS = 0,
  EDGEFIRST_PCD_CLASSIFY_OUTPUT_COLORS = 1,
  EDGEFIRST_PCD_CLASSIFY_OUTPUT_BOTH = 2,
} EdgefirstPcdClassifyOutputMode;

GType edgefirst_pcd_classify_output_mode_get_type (void);
#define EDGEFIRST_TYPE_PCD_CLASSIFY_OUTPUT_MODE \
    (edgefirst_pcd_classify_output_mode_get_type())

G_END_DECLS

#endif /* __EDGEFIRST_PCD_CLASSIFY_H__ */
