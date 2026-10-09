/*
 * EdgeFirst Perception for GStreamer - Detection GObject Types
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_DETECTION_H__
#define __EDGEFIRST_DETECTION_H__

#include <glib-object.h>
#include <edgefirst/detect.h>

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
 * @boxes: (array length=n_boxes) (nullable): HAL detections, copied
 * @n_boxes: number of entries in @boxes
 *
 * Box coordinates are taken to be normalized [0,1].
 *
 * Returns: (transfer full) (nullable): a new #EdgeFirstDetectBoxList, or
 *   NULL when @boxes is NULL and @n_boxes is not zero
 */
EdgeFirstDetectBoxList *edgefirst_detect_box_list_new (const ef_detect_box *boxes,
                                                       guint               n_boxes);

/**
 * edgefirst_detect_box_list_new_normalized:
 * @boxes: (array length=n_boxes) (nullable): HAL detections, copied
 * @n_boxes: number of entries in @boxes
 * @normalized: TRUE if HAL coordinates are already [0,1]
 * @model_w: model input width (used when @normalized is FALSE)
 * @model_h: model input height (used when @normalized is FALSE)
 *
 * Returns: (transfer full) (nullable): a new #EdgeFirstDetectBoxList, or
 *   NULL when @boxes is NULL and @n_boxes is not zero
 */
EdgeFirstDetectBoxList *edgefirst_detect_box_list_new_normalized (
    const ef_detect_box *boxes,
    guint                n_boxes,
    gboolean             normalized,
    guint                model_w,
    guint                model_h);

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
 * @segs: (array length=n_segs) (nullable): HAL segmentations, borrowed
 * @n_segs: number of entries in @segs
 * @owner: (nullable): owner of @segs and of the mask bytes they point to
 * @owner_free: (nullable): releases @owner when the list is finalized
 *
 * Wraps @segs without copying them. @segs and every mask it points to must
 * stay valid until @owner_free is called on @owner.
 *
 * Returns: (transfer full) (nullable): a new #EdgeFirstSegmentationList, or
 *   NULL when @segs is NULL and @n_segs is not zero
 */
EdgeFirstSegmentationList *edgefirst_segmentation_list_new (
    const ef_segmentation *segs,
    guint                  n_segs,
    gpointer               owner,
    GDestroyNotify         owner_free);

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
 * edgefirst_detect_box_list_get_data:
 * @self: (nullable): a #EdgeFirstDetectBoxList
 * @n_boxes: (out): number of boxes
 *
 * The detections as HAL reported them, before any normalization.
 *
 * Returns: (array length=n_boxes) (transfer none) (nullable): the boxes,
 *   owned by @self, or NULL when @self is NULL or empty
 */
const ef_detect_box *edgefirst_detect_box_list_get_data (EdgeFirstDetectBoxList *self,
                                                          guint                  *n_boxes);

/**
 * edgefirst_segmentation_list_get_data:
 * @self: (nullable): a #EdgeFirstSegmentationList
 * @n_segs: (out): number of segmentations
 *
 * Returns: (array length=n_segs) (transfer none) (nullable): the
 *   segmentations, owned by @self, or NULL when @self is NULL or empty
 */
const ef_segmentation *edgefirst_segmentation_list_get_data (EdgeFirstSegmentationList *self,
                                                              guint                     *n_segs);

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
