/*
 * EdgeFirst Perception for GStreamer - Detection GObject Types
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstdetection.h"

/* ── EdgeFirstDetectBox (GBoxed) ─────────────────────────────────── */

EdgeFirstDetectBox *
edgefirst_detect_box_copy (const EdgeFirstDetectBox *box)
{
  return g_slice_dup (EdgeFirstDetectBox, box);
}

void
edgefirst_detect_box_free (EdgeFirstDetectBox *box)
{
  g_slice_free (EdgeFirstDetectBox, box);
}

G_DEFINE_BOXED_TYPE (EdgeFirstDetectBox, edgefirst_detect_box,
    edgefirst_detect_box_copy, edgefirst_detect_box_free)

/* ── EdgeFirstDetectBoxList (GObject) ────────────────────────────── */

struct _EdgeFirstDetectBoxList {
  GObject parent;
  GArray *boxes;         /* ef_detect_box, as HAL reported them */
  gboolean normalized;   /* TRUE = coords already [0,1] */
  guint model_w;         /* model input width for pixel→normalized scaling */
  guint model_h;
};

G_DEFINE_FINAL_TYPE (EdgeFirstDetectBoxList, edgefirst_detect_box_list, G_TYPE_OBJECT)

static void
edgefirst_detect_box_list_finalize (GObject *object)
{
  EdgeFirstDetectBoxList *self = EDGEFIRST_DETECT_BOX_LIST (object);
  g_array_unref (self->boxes);
  G_OBJECT_CLASS (edgefirst_detect_box_list_parent_class)->finalize (object);
}

static void
edgefirst_detect_box_list_class_init (EdgeFirstDetectBoxListClass *klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  obj_class->finalize = edgefirst_detect_box_list_finalize;
}

static void
edgefirst_detect_box_list_init (EdgeFirstDetectBoxList *self)
{
  self->boxes      = g_array_new (FALSE, FALSE, sizeof (ef_detect_box));
  self->normalized = TRUE;
  self->model_w    = 0;
  self->model_h    = 0;
}

EdgeFirstDetectBoxList *
edgefirst_detect_box_list_new (const ef_detect_box *boxes, guint n_boxes)
{
  return edgefirst_detect_box_list_new_normalized (boxes, n_boxes, TRUE, 0, 0);
}

EdgeFirstDetectBoxList *
edgefirst_detect_box_list_new_normalized (const ef_detect_box *boxes,
    guint n_boxes, gboolean normalized, guint model_w, guint model_h)
{
  if (!boxes && n_boxes > 0)
    return NULL;
  EdgeFirstDetectBoxList *self =
      g_object_new (EDGEFIRST_TYPE_DETECT_BOX_LIST, NULL);
  if (n_boxes > 0)
    g_array_append_vals (self->boxes, boxes, n_boxes);
  self->normalized = normalized;
  self->model_w    = model_w;
  self->model_h    = model_h;
  return self;
}

guint
edgefirst_detect_box_list_get_length (EdgeFirstDetectBoxList *self)
{
  g_return_val_if_fail (EDGEFIRST_IS_DETECT_BOX_LIST (self), 0);
  return self->boxes->len;
}

EdgeFirstDetectBox *
edgefirst_detect_box_list_get (EdgeFirstDetectBoxList *self, guint index)
{
  g_return_val_if_fail (EDGEFIRST_IS_DETECT_BOX_LIST (self), NULL);

  if (index >= self->boxes->len)
    return NULL;
  const ef_detect_box *hbox = &g_array_index (self->boxes, ef_detect_box, index);

  EdgeFirstDetectBox *box = g_slice_new (EdgeFirstDetectBox);
  if (self->normalized || self->model_w == 0 || self->model_h == 0) {
    box->x1 = hbox->xmin;
    box->y1 = hbox->ymin;
    box->x2 = hbox->xmax;
    box->y2 = hbox->ymax;
  } else {
    box->x1 = hbox->xmin / (gfloat) self->model_w;
    box->y1 = hbox->ymin / (gfloat) self->model_h;
    box->x2 = hbox->xmax / (gfloat) self->model_w;
    box->y2 = hbox->ymax / (gfloat) self->model_h;
  }
  box->class_id = (gint) hbox->label;
  box->score    = hbox->score;
  box->track_id = -1;  /* track_id populated when tracking is enabled */
  return box;
}

/* ── EdgeFirstSegmentation (GBoxed) ─────────────────────────────── */

EdgeFirstSegmentation *
edgefirst_segmentation_copy (const EdgeFirstSegmentation *seg)
{
  EdgeFirstSegmentation *copy = g_slice_dup (EdgeFirstSegmentation, seg);
  copy->mask = seg->mask ? g_bytes_ref (seg->mask) : NULL;
  return copy;
}

void
edgefirst_segmentation_free (EdgeFirstSegmentation *seg)
{
  if (seg) {
    if (seg->mask)
      g_bytes_unref (seg->mask);
    g_slice_free (EdgeFirstSegmentation, seg);
  }
}

G_DEFINE_BOXED_TYPE (EdgeFirstSegmentation, edgefirst_segmentation,
    edgefirst_segmentation_copy, edgefirst_segmentation_free)

/* ── EdgeFirstSegmentationList (GObject) ─────────────────────────── */

struct _EdgeFirstSegmentationList {
  GObject parent;
  const ef_segmentation *segs;  /* borrowed from owner */
  guint n_segs;
  gpointer owner;
  GDestroyNotify owner_free;
};

G_DEFINE_FINAL_TYPE (EdgeFirstSegmentationList, edgefirst_segmentation_list, G_TYPE_OBJECT)

static void
edgefirst_segmentation_list_finalize (GObject *object)
{
  EdgeFirstSegmentationList *self = EDGEFIRST_SEGMENTATION_LIST (object);
  if (self->owner_free && self->owner)
    self->owner_free (self->owner);
  G_OBJECT_CLASS (edgefirst_segmentation_list_parent_class)->finalize (object);
}

static void
edgefirst_segmentation_list_class_init (EdgeFirstSegmentationListClass *klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  obj_class->finalize = edgefirst_segmentation_list_finalize;
}

static void
edgefirst_segmentation_list_init (EdgeFirstSegmentationList *self)
{
  self->segs       = NULL;
  self->n_segs     = 0;
  self->owner      = NULL;
  self->owner_free = NULL;
}

EdgeFirstSegmentationList *
edgefirst_segmentation_list_new (const ef_segmentation *segs, guint n_segs,
    gpointer owner, GDestroyNotify owner_free)
{
  if (!segs && n_segs > 0)
    return NULL;
  EdgeFirstSegmentationList *self =
      g_object_new (EDGEFIRST_TYPE_SEGMENTATION_LIST, NULL);
  self->segs       = n_segs > 0 ? segs : NULL;
  self->n_segs     = n_segs;
  self->owner      = owner;
  self->owner_free = owner_free;
  return self;
}

guint
edgefirst_segmentation_list_get_length (EdgeFirstSegmentationList *self)
{
  g_return_val_if_fail (EDGEFIRST_IS_SEGMENTATION_LIST (self), 0);
  return self->n_segs;
}

EdgeFirstSegmentation *
edgefirst_segmentation_list_get (EdgeFirstSegmentationList *self, guint index)
{
  g_return_val_if_fail (EDGEFIRST_IS_SEGMENTATION_LIST (self), NULL);

  if (index >= self->n_segs)
    return NULL;
  const ef_segmentation *hseg = &self->segs[index];
  if (!hseg->mask)
    return NULL;

  /* Deep-copy mask bytes into GBytes so this struct is lifetime-independent */
  GBytes *mask = g_bytes_new (hseg->mask, (gsize) hseg->height * hseg->width);

  EdgeFirstSegmentation *seg = g_slice_new (EdgeFirstSegmentation);
  seg->x1     = hseg->xmin;
  seg->y1     = hseg->ymin;
  seg->x2     = hseg->xmax;
  seg->y2     = hseg->ymax;
  seg->width  = hseg->width;
  seg->height = hseg->height;
  seg->mask   = mask;
  return seg;
}

const ef_detect_box *
edgefirst_detect_box_list_get_data (EdgeFirstDetectBoxList *self,
    guint *n_boxes)
{
  if (!self || self->boxes->len == 0) {
    *n_boxes = 0;
    return NULL;
  }
  *n_boxes = self->boxes->len;
  return (const ef_detect_box *) self->boxes->data;
}

const ef_segmentation *
edgefirst_segmentation_list_get_data (EdgeFirstSegmentationList *self,
    guint *n_segs)
{
  if (!self || self->n_segs == 0) {
    *n_segs = 0;
    return NULL;
  }
  *n_segs = self->n_segs;
  return self->segs;
}

/* ── EdgeFirstColorMode (GEnum) ──────────────────────────────────── */

GType
edgefirst_color_mode_get_type (void)
{
  static gsize type_id = 0;
  if (g_once_init_enter (&type_id)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_COLOR_MODE_CLASS,    "Color by class label",     "class"    },
      { EDGEFIRST_COLOR_MODE_INSTANCE, "Color by detection index", "instance" },
      { EDGEFIRST_COLOR_MODE_TRACK,    "Color by track ID",        "track"    },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgeFirstColorMode", values);
    g_once_init_leave (&type_id, (gsize) t);
  }
  return (GType) type_id;
}
