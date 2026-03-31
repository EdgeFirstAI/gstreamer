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
  hal_detect_box_list *list;  /* owned */
  gboolean normalized;   /* TRUE = coords already [0,1] */
  guint model_w;         /* model input width for pixel→normalized scaling */
  guint model_h;
};

G_DEFINE_FINAL_TYPE (EdgeFirstDetectBoxList, edgefirst_detect_box_list, G_TYPE_OBJECT)

static void
edgefirst_detect_box_list_finalize (GObject *object)
{
  EdgeFirstDetectBoxList *self = EDGEFIRST_DETECT_BOX_LIST (object);
  hal_detect_box_list_free (self->list);
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
  self->list       = NULL;
  self->normalized = TRUE;
  self->model_w    = 0;
  self->model_h    = 0;
}

EdgeFirstDetectBoxList *
edgefirst_detect_box_list_new (hal_detect_box_list *list)
{
  return edgefirst_detect_box_list_new_normalized (list, TRUE, 0, 0);
}

EdgeFirstDetectBoxList *
edgefirst_detect_box_list_new_normalized (hal_detect_box_list *list,
    gboolean normalized, guint model_w, guint model_h)
{
  g_return_val_if_fail (list != NULL, NULL);
  EdgeFirstDetectBoxList *self =
      g_object_new (EDGEFIRST_TYPE_DETECT_BOX_LIST, NULL);
  self->list       = list;
  self->normalized = normalized;
  self->model_w    = model_w;
  self->model_h    = model_h;
  return self;
}

guint
edgefirst_detect_box_list_get_length (EdgeFirstDetectBoxList *self)
{
  g_return_val_if_fail (EDGEFIRST_IS_DETECT_BOX_LIST (self), 0);
  return (guint) hal_detect_box_list_len (self->list);
}

EdgeFirstDetectBox *
edgefirst_detect_box_list_get (EdgeFirstDetectBoxList *self, guint index)
{
  struct hal_detect_box hbox;

  g_return_val_if_fail (EDGEFIRST_IS_DETECT_BOX_LIST (self), NULL);

  if (hal_detect_box_list_get (self->list, (size_t) index, &hbox) != 0)
    return NULL;

  EdgeFirstDetectBox *box = g_slice_new (EdgeFirstDetectBox);
  if (self->normalized || self->model_w == 0 || self->model_h == 0) {
    box->x1 = hbox.xmin;
    box->y1 = hbox.ymin;
    box->x2 = hbox.xmax;
    box->y2 = hbox.ymax;
  } else {
    box->x1 = hbox.xmin / (gfloat) self->model_w;
    box->y1 = hbox.ymin / (gfloat) self->model_h;
    box->x2 = hbox.xmax / (gfloat) self->model_w;
    box->y2 = hbox.ymax / (gfloat) self->model_h;
  }
  box->class_id = (gint) hbox.label;
  box->score    = hbox.score;
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
  hal_segmentation_list *list;  /* owned */
};

G_DEFINE_FINAL_TYPE (EdgeFirstSegmentationList, edgefirst_segmentation_list, G_TYPE_OBJECT)

static void
edgefirst_segmentation_list_finalize (GObject *object)
{
  EdgeFirstSegmentationList *self = EDGEFIRST_SEGMENTATION_LIST (object);
  hal_segmentation_list_free (self->list);
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
  self->list = NULL;
}

EdgeFirstSegmentationList *
edgefirst_segmentation_list_new (hal_segmentation_list *list)
{
  g_return_val_if_fail (list != NULL, NULL);
  EdgeFirstSegmentationList *self =
      g_object_new (EDGEFIRST_TYPE_SEGMENTATION_LIST, NULL);
  self->list = list;
  return self;
}

guint
edgefirst_segmentation_list_get_length (EdgeFirstSegmentationList *self)
{
  g_return_val_if_fail (EDGEFIRST_IS_SEGMENTATION_LIST (self), 0);
  return (guint) hal_segmentation_list_len (self->list);
}

EdgeFirstSegmentation *
edgefirst_segmentation_list_get (EdgeFirstSegmentationList *self, guint index)
{
  size_t out_h = 0, out_w = 0;
  float xmin, ymin, xmax, ymax;

  g_return_val_if_fail (EDGEFIRST_IS_SEGMENTATION_LIST (self), NULL);

  const uint8_t *data = hal_segmentation_list_get_mask (self->list,
      (size_t) index, &out_h, &out_w);
  if (!data)
    return NULL;

  if (hal_segmentation_list_get_bbox (self->list, (size_t) index,
          &xmin, &ymin, &xmax, &ymax) != 0)
    return NULL;

  /* Deep-copy mask bytes into GBytes so this struct is lifetime-independent */
  GBytes *mask = g_bytes_new (data, out_h * out_w);

  EdgeFirstSegmentation *seg = g_slice_new (EdgeFirstSegmentation);
  seg->x1     = xmin;
  seg->y1     = ymin;
  seg->x2     = xmax;
  seg->y2     = ymax;
  seg->width  = (guint) out_w;
  seg->height = (guint) out_h;
  seg->mask   = mask;
  return seg;
}

hal_detect_box_list *
edgefirst_detect_box_list_get_hal (EdgeFirstDetectBoxList *self)
{
  return self ? self->list : NULL;
}

hal_segmentation_list *
edgefirst_segmentation_list_get_hal (EdgeFirstSegmentationList *self)
{
  return self ? self->list : NULL;
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
