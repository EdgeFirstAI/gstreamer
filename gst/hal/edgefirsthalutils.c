/*
 * EdgeFirst Perception for GStreamer - HAL C API helpers
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirsthalutils.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_hal_utils_debug);
#define GST_CAT_DEFAULT edgefirst_hal_utils_debug

static void
ensure_debug_category (void)
{
  static gsize initialized = 0;
  if (g_once_init_enter (&initialized)) {
    GST_DEBUG_CATEGORY_INIT (edgefirst_hal_utils_debug, "edgefirsthalutils",
        0, "EdgeFirst HAL tensor import");
    g_once_init_leave (&initialized, 1);
  }
}

/* ── Format table ──────────────────────────────────────────────────── */

/*
 * The single GStreamer <-> HAL pixel format mapping. A GStreamer format is
 * accepted as HAL input exactly when it has a row here.
 */
static const EdgefirstHalFormat hal_formats[] = {
  { EDGEFIRST_HAL_FORMAT_NV12,         GST_VIDEO_FORMAT_NV12,
    EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR, 1 },
  { EDGEFIRST_HAL_FORMAT_YUYV,         GST_VIDEO_FORMAT_YUY2,
    EDGEFIRST_HAL_LAYOUT_PACKED,      2 },
  { EDGEFIRST_HAL_FORMAT_RGB,          GST_VIDEO_FORMAT_RGB,
    EDGEFIRST_HAL_LAYOUT_PACKED,      3 },
  { EDGEFIRST_HAL_FORMAT_RGBA,         GST_VIDEO_FORMAT_RGBA,
    EDGEFIRST_HAL_LAYOUT_PACKED,      4 },
  { EDGEFIRST_HAL_FORMAT_GREY,         GST_VIDEO_FORMAT_GRAY8,
    EDGEFIRST_HAL_LAYOUT_PACKED,      1 },
  { EDGEFIRST_HAL_FORMAT_BGRA,         GST_VIDEO_FORMAT_UNKNOWN,
    EDGEFIRST_HAL_LAYOUT_PACKED,      4 },
  { EDGEFIRST_HAL_FORMAT_PLANAR_RGB,   GST_VIDEO_FORMAT_UNKNOWN,
    EDGEFIRST_HAL_LAYOUT_PLANAR,      3 },
  { EDGEFIRST_HAL_FORMAT_PLANAR_RGBA,  GST_VIDEO_FORMAT_UNKNOWN,
    EDGEFIRST_HAL_LAYOUT_PLANAR,      4 },
  { NULL, GST_VIDEO_FORMAT_UNKNOWN, EDGEFIRST_HAL_LAYOUT_PACKED, 0 },
};

const EdgefirstHalFormat *
edgefirst_hal_formats (void)
{
  return hal_formats;
}

const EdgefirstHalFormat *
edgefirst_hal_format_from_gst (GstVideoFormat fmt)
{
  if (fmt == GST_VIDEO_FORMAT_UNKNOWN)
    return NULL;
  for (const EdgefirstHalFormat *f = hal_formats; f->wire; f++) {
    if (f->gst == fmt)
      return f;
  }
  return NULL;
}

const EdgefirstHalFormat *
edgefirst_hal_format_from_wire (const gchar *wire)
{
  if (!wire)
    return NULL;
  for (const EdgefirstHalFormat *f = hal_formats; f->wire; f++) {
    if (strcmp (f->wire, wire) == 0)
      return f;
  }
  return NULL;
}

guint
edgefirst_hal_format_shape (const EdgefirstHalFormat *fmt,
    guint width, guint height, guint64 *shape)
{
  switch (fmt->layout) {
    case EDGEFIRST_HAL_LAYOUT_PLANAR:
      shape[0] = fmt->channels;
      shape[1] = height;
      shape[2] = width;
      return 3;
    case EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR:
      shape[0] = (guint64) height + (height + 1) / 2;
      shape[1] = width;
      return 2;
    case EDGEFIRST_HAL_LAYOUT_PACKED:
    default:
      shape[0] = height;
      shape[1] = width;
      shape[2] = fmt->channels;
      return 3;
  }
}

gsize
edgefirst_hal_format_row_bytes (const EdgefirstHalFormat *fmt, guint width)
{
  if (fmt->layout == EDGEFIRST_HAL_LAYOUT_PACKED)
    return (gsize) width * fmt->channels;
  return width;
}

/* ── Tensors ───────────────────────────────────────────────────────── */

/*
 * Wrap one plane of a DMA-BUF. The builder adopts the descriptor it is
 * given, so the caller's fd is duplicated first and the duplicate closed if
 * HAL refuses it.
 */
static ef_tensor *
wrap_plane (gint fd, gsize offset, gsize stride, uint32_t dtype,
    const guint64 *shape, guint ndim, const gchar *format)
{
  gint dup_fd = dup (fd);
  if (dup_fd < 0)
    return NULL;

  ef_tensor_builder *b = ef_tensor_builder_new ();
  if (!b) {
    close (dup_fd);
    return NULL;
  }
  ef_tensor_builder_dtype (b, dtype);
  ef_tensor_builder_shape (b, shape, ndim);
  if (format)
    ef_tensor_builder_format (b, format);
  ef_tensor_builder_add_plane (b, dup_fd, offset, stride, 0, 0, 0);

  ef_tensor *t = ef_tensor_builder_wrap (b);
  ef_tensor_builder_free (b);
  if (!t)
    close (dup_fd);
  return t;
}

static gboolean
is_dmabuf (const ef_tensor *t)
{
  return ef_tensor_storage_kind (t) == EF_STORAGE_KIND_DMA_BUF;
}

ef_tensor *
edgefirst_hal_import_image (GstObject *obj, const EdgefirstHalPlane *image,
    const EdgefirstHalPlane *chroma, guint width, guint height,
    const EdgefirstHalFormat *fmt, uint32_t dtype)
{
  ensure_debug_category ();

  if (!chroma) {
    guint64 shape[3];
    guint ndim = edgefirst_hal_format_shape (fmt, width, height, shape);
    ef_tensor *t = wrap_plane (image->fd, image->offset, image->stride,
        dtype, shape, ndim, fmt->wire);
    if (!t) {
      GST_DEBUG_OBJECT (obj, "HAL refused %s import of fd=%d: %s",
          fmt->wire, image->fd, ef_tensor_last_error_message ());
      return NULL;
    }
    if (!is_dmabuf (t)) {
      GST_DEBUG_OBJECT (obj, "fd=%d is not DMA-BUF backed", image->fd);
      ef_tensor_free (t);
      return NULL;
    }
    return t;
  }

  /* Separate luma and chroma planes: HAL composes NV12 from two raw U8
   * plane tensors, with the chroma geometry set before they are joined and
   * the luma geometry after. */
  if (fmt->layout != EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR
      || strcmp (fmt->wire, EDGEFIRST_HAL_FORMAT_NV12) != 0
      || (dtype != EF_DTYPE_U8 && dtype != EF_DTYPE_I8)) {
    GST_DEBUG_OBJECT (obj, "a separate chroma plane needs U8/I8 NV12, got %s",
        fmt->wire);
    return NULL;
  }
  if (chroma->stride > 0 && chroma->stride < width) {
    GST_DEBUG_OBJECT (obj, "chroma stride %" G_GSIZE_FORMAT " < width %u",
        chroma->stride, width);
    return NULL;
  }

  guint64 luma_shape[2] = { height, width };
  guint64 chroma_shape[2] = { (height + 1) / 2, width };
  ef_tensor *luma = wrap_plane (image->fd, 0, 0, EF_DTYPE_U8,
      luma_shape, 2, NULL);
  ef_tensor *uv = wrap_plane (chroma->fd, 0, 0, EF_DTYPE_U8,
      chroma_shape, 2, NULL);
  if (!luma || !uv || !is_dmabuf (luma) || !is_dmabuf (uv)) {
    GST_DEBUG_OBJECT (obj, "NV12 plane import refused (y_fd=%d uv_fd=%d): %s",
        image->fd, chroma->fd, ef_tensor_last_error_message ());
    ef_tensor_free (luma);
    ef_tensor_free (uv);
    return NULL;
  }
  if (chroma->stride > 0)
    ef_tensor_set_row_stride_unchecked (uv, chroma->stride);
  if (chroma->offset > 0)
    ef_tensor_set_plane_offset (uv, chroma->offset);

  ef_tensor *t = ef_tensor_from_planes (luma, uv, fmt->wire);
  if (!t) {
    GST_DEBUG_OBJECT (obj, "HAL refused NV12 planes: %s",
        ef_tensor_last_error_message ());
    /* The planes are consumed: the refusals that leave them with the
     * caller (a null handle, an unknown format, a dtype mismatch, an
     * outstanding retain or map) cannot occur for two fresh wraps. */
    return NULL;
  }

  if (image->stride > 0 && ef_tensor_set_row_stride (t, image->stride) != 0) {
    GST_DEBUG_OBJECT (obj, "HAL refused luma stride %" G_GSIZE_FORMAT ": %s",
        image->stride, ef_tensor_last_error_message ());
    ef_tensor_free (t);
    return NULL;
  }
  if (image->offset > 0)
    ef_tensor_set_plane_offset (t, image->offset);
  if (dtype == EF_DTYPE_I8)
    ef_tensor_set_dtype (t, EF_DTYPE_I8);
  return t;
}

ef_tensor *
edgefirst_hal_wrap_fd (gint fd, uint32_t dtype, const guint64 *shape,
    guint ndim)
{
  return wrap_plane (fd, 0, 0, dtype, shape, ndim, NULL);
}

ef_tensor *
edgefirst_hal_new_tensor (uint32_t dtype, const guint64 *shape, guint ndim)
{
  ef_tensor_builder *b = ef_tensor_builder_new ();
  if (!b)
    return NULL;
  ef_tensor_builder_dtype (b, dtype);
  ef_tensor_builder_shape (b, shape, ndim);
  ef_tensor_builder_storage (b, EF_STORAGE_KIND_MEM);
  ef_tensor *t = ef_tensor_builder_alloc (b);
  ef_tensor_builder_free (b);
  return t;
}

gsize
edgefirst_hal_row_stride (const ef_tensor *t)
{
  ef_tensor_plane plane;
  if (!t || ef_tensor_plane_at (t, 0, &plane) != 0)
    return 0;
  return (gsize) plane.stride;
}

ef_tensor *
edgefirst_hal_create_image (ef_image_processor *processor, guint width,
    guint height, const EdgefirstHalFormat *fmt, uint32_t dtype)
{
  ef_tensor_image_desc *desc = ef_tensor_image_desc_new (width, height,
      fmt->wire, dtype);
  if (!desc)
    return NULL;
  ef_tensor_image_desc_set_access (desc, EF_CPU_ACCESS_READ_WRITE);
  ef_tensor *t = ef_image_processor_create_image_desc (processor, desc);
  ef_tensor_image_desc_free (desc);
  return t;
}
