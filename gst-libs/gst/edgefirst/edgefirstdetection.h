/*
 * EdgeFirst Perception for GStreamer - Detection GObject Types
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_DETECTION_H__
#define __EDGEFIRST_DETECTION_H__

#include <glib-object.h>
#include <edgefirst/hal.h>

G_BEGIN_DECLS

/* ── EdgeFirstDetectBox ──────────────────────────────────────────── */

/**
 * EdgeFirstDetectBox:
 * @x1: left edge, normalized [0,1]
 * @y1: top edge, normalized [0,1]
 * @x2: right edge, normalized [0,1]
 * @y2: bottom edge, normalized [0,1]
 * @class_id: class index
 * @score: confidence score [0,1]
 * @track_id: tracker ID, -1 if not tracked
 *
 * A single bounding box detection result. Coordinates are always in
 * normalized image space [0,1].
 */
typedef struct {
  gfloat x1, y1, x2, y2;
  gint   class_id;
  gfloat score;
  gint64 track_id;
} EdgeFirstDetectBox;

#define EDGEFIRST_TYPE_DETECT_BOX (edgefirst_detect_box_get_type ())
GType edgefirst_detect_box_get_type (void);

EdgeFirstDetectBox *edgefirst_detect_box_copy (const EdgeFirstDetectBox *box);
void                edgefirst_detect_box_free (EdgeFirstDetectBox *box);

/* ── EdgeFirstDetectBoxList ──────────────────────────────────────── */

#define EDGEFIRST_TYPE_DETECT_BOX_LIST (edgefirst_detect_box_list_get_type ())
G_DECLARE_FINAL_TYPE (EdgeFirstDetectBoxList, edgefirst_detect_box_list,
    EDGEFIRST, DETECT_BOX_LIST, GObject)

/**
 * edgefirst_detect_box_list_new:
 * @list: (transfer full): HAL detect box list to wrap (takes ownership)
 *
 * Returns: (transfer full): a new #EdgeFirstDetectBoxList
 */
EdgeFirstDetectBoxList *edgefirst_detect_box_list_new (hal_detect_box_list *list);

/**
 * edgefirst_detect_box_list_new_normalized:
 * @list: (transfer full): HAL detect box list to wrap (takes ownership)
 * @normalized: TRUE if HAL coordinates are already [0,1]
 * @model_w: model input width (used when @normalized is FALSE)
 * @model_h: model input height (used when @normalized is FALSE)
 *
 * Returns: (transfer full): a new #EdgeFirstDetectBoxList
 */
EdgeFirstDetectBoxList *edgefirst_detect_box_list_new_normalized (
    hal_detect_box_list *list,
    gboolean normalized,
    guint model_w,
    guint model_h);

/**
 * edgefirst_detect_box_list_get_length:
 * @self: a #EdgeFirstDetectBoxList
 *
 * Returns: number of detections
 */
guint edgefirst_detect_box_list_get_length (EdgeFirstDetectBoxList *self);

/**
 * edgefirst_detect_box_list_get:
 * @self: a #EdgeFirstDetectBoxList
 * @index: detection index
 *
 * Returns: (transfer full) (nullable): copy of the detection box, or NULL
 */
EdgeFirstDetectBox *edgefirst_detect_box_list_get (EdgeFirstDetectBoxList *self,
                                                    guint                   index);

/* ── EdgeFirstSegmentation ───────────────────────────────────────── */

/**
 * EdgeFirstSegmentation:
 * @x1: bbox left, normalized [0,1]
 * @y1: bbox top, normalized [0,1]
 * @x2: bbox right, normalized [0,1]
 * @y2: bbox bottom, normalized [0,1]
 * @width: mask width in pixels
 * @height: mask height in pixels
 * @mask: (transfer full): per-pixel uint8 sigmoid confidence (0=bg, 255=fg)
 */
typedef struct {
  gfloat  x1, y1, x2, y2;
  guint   width, height;
  GBytes *mask;
} EdgeFirstSegmentation;

#define EDGEFIRST_TYPE_SEGMENTATION (edgefirst_segmentation_get_type ())
GType edgefirst_segmentation_get_type (void);

EdgeFirstSegmentation *edgefirst_segmentation_copy (const EdgeFirstSegmentation *seg);
void                   edgefirst_segmentation_free (EdgeFirstSegmentation *seg);

/* ── EdgeFirstSegmentationList ───────────────────────────────────── */

#define EDGEFIRST_TYPE_SEGMENTATION_LIST (edgefirst_segmentation_list_get_type ())
G_DECLARE_FINAL_TYPE (EdgeFirstSegmentationList, edgefirst_segmentation_list,
    EDGEFIRST, SEGMENTATION_LIST, GObject)

/**
 * edgefirst_segmentation_list_new:
 * @list: (transfer full): HAL segmentation list to wrap (takes ownership)
 *
 * Returns: (transfer full): a new #EdgeFirstSegmentationList
 */
EdgeFirstSegmentationList *edgefirst_segmentation_list_new (hal_segmentation_list *list);

/**
 * edgefirst_segmentation_list_get_length:
 * @self: a #EdgeFirstSegmentationList
 *
 * Returns: number of segmentations
 */
guint edgefirst_segmentation_list_get_length (EdgeFirstSegmentationList *self);

/**
 * edgefirst_segmentation_list_get:
 * @self: a #EdgeFirstSegmentationList
 * @index: segmentation index
 *
 * Returns: (transfer full) (nullable): deep copy of the segmentation, or NULL
 */
EdgeFirstSegmentation *edgefirst_segmentation_list_get (EdgeFirstSegmentationList *self,
                                                         guint                      index);

/**
 * edgefirst_detect_box_list_get_hal:
 * @self: a #EdgeFirstDetectBoxList (may be NULL)
 *
 * Returns the underlying #hal_detect_box_list pointer.  The returned pointer
 * is owned by @self — do not free it.
 */
hal_detect_box_list *edgefirst_detect_box_list_get_hal (EdgeFirstDetectBoxList *self);

/**
 * edgefirst_segmentation_list_get_hal:
 * @self: a #EdgeFirstSegmentationList (may be NULL)
 *
 * Returns the underlying #hal_segmentation_list pointer.  The returned pointer
 * is owned by @self — do not free it.
 */
hal_segmentation_list *edgefirst_segmentation_list_get_hal (EdgeFirstSegmentationList *self);

/* ── EdgeFirstColorMode ──────────────────────────────────────────── */

/**
 * EdgeFirstColorMode:
 * @EDGEFIRST_COLOR_MODE_CLASS: color by class label (default)
 * @EDGEFIRST_COLOR_MODE_INSTANCE: color by detection index
 * @EDGEFIRST_COLOR_MODE_TRACK: color by track ID
 */
typedef enum {
  EDGEFIRST_COLOR_MODE_CLASS    = 0,
  EDGEFIRST_COLOR_MODE_INSTANCE = 1,
  EDGEFIRST_COLOR_MODE_TRACK    = 2,
} EdgeFirstColorMode;

#define EDGEFIRST_TYPE_COLOR_MODE (edgefirst_color_mode_get_type ())
GType edgefirst_color_mode_get_type (void);

G_END_DECLS

#endif /* __EDGEFIRST_DETECTION_H__ */
