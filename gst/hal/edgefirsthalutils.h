/*
 * EdgeFirst Perception for GStreamer - HAL C API helpers
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared between the HAL elements: the GStreamer <-> HAL pixel format table,
 * DMA-BUF image import, and the integer codes the HAL C API documents but
 * does not name in its headers.
 */

#ifndef __EDGEFIRST_HAL_UTILS_H__
#define __EDGEFIRST_HAL_UTILS_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <edgefirst/tensor.h>
#include <edgefirst/image.h>

G_BEGIN_DECLS

/* ── Pixel formats ─────────────────────────────────────────────────── */

/*
 * HAL pixel format wire names (`PixelFormat::as_str()`). The HAL C headers
 * take these strings but do not declare them; the format table test checks
 * every name against the linked HAL.
 */
#define EDGEFIRST_HAL_FORMAT_RGB          "rgb8"
#define EDGEFIRST_HAL_FORMAT_RGBA         "rgba8"
#define EDGEFIRST_HAL_FORMAT_BGRA         "bgra8"
#define EDGEFIRST_HAL_FORMAT_GREY         "mono8"
#define EDGEFIRST_HAL_FORMAT_YUYV         "YUYV"
#define EDGEFIRST_HAL_FORMAT_NV12         "NV12"
#define EDGEFIRST_HAL_FORMAT_PLANAR_RGB   "rgb8_planar"
#define EDGEFIRST_HAL_FORMAT_PLANAR_RGBA  "rgba8_planar"

typedef enum {
  EDGEFIRST_HAL_LAYOUT_PACKED,       /* [H, W, C] */
  EDGEFIRST_HAL_LAYOUT_PLANAR,       /* [C, H, W] */
  EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR,  /* [H + ceil(H/2), W] luma + interleaved chroma */
} EdgefirstHalLayout;

/**
 * EdgefirstHalFormat:
 * @wire: HAL pixel format wire name
 * @gst: matching GStreamer video format, or %GST_VIDEO_FORMAT_UNKNOWN for a
 *   format the elements only produce as model input
 * @layout: memory layout of the HAL allocation
 * @channels: channels per pixel position (1 for the luma plane of NV12)
 *
 * One row of the GStreamer <-> HAL format table.
 */
typedef struct {
  const gchar        *wire;
  GstVideoFormat      gst;
  EdgefirstHalLayout  layout;
  guint               channels;
} EdgefirstHalFormat;

/* Every row of the format table, terminated by a row whose @wire is NULL. */
const EdgefirstHalFormat *edgefirst_hal_formats (void);

/* Table row for a GStreamer video format, or NULL when HAL cannot take it. */
const EdgefirstHalFormat *edgefirst_hal_format_from_gst (GstVideoFormat fmt);

/* Table row for a HAL wire name, or NULL when the table does not carry it. */
const EdgefirstHalFormat *edgefirst_hal_format_from_wire (const gchar *wire);

/*
 * The allocation shape HAL uses for a @width x @height image of @fmt.
 * Fills @shape (at least 3 entries) and returns the rank.
 */
guint edgefirst_hal_format_shape (const EdgefirstHalFormat *fmt,
    guint width, guint height, guint64 *shape);

/* Bytes in one tightly packed row of plane 0 (luma for NV12), U8 only. */
gsize edgefirst_hal_format_row_bytes (const EdgefirstHalFormat *fmt,
    guint width);

/* ── Integer codes from the HAL C API documentation ────────────────── */

/* Compute backend codes for ef_image_processor_new_with_backend in image.h */
#define EDGEFIRST_HAL_BACKEND_AUTO    0u
#define EDGEFIRST_HAL_BACKEND_CPU     1u
#define EDGEFIRST_HAL_BACKEND_G2D     2u
#define EDGEFIRST_HAL_BACKEND_OPENGL  3u

/* Rotation and flip codes for ef_image_processor_convert in image.h */
#define EDGEFIRST_HAL_ROTATION_NONE   0u
#define EDGEFIRST_HAL_FLIP_NONE       0u

/* Color mode codes for the ef_image_processor_draw functions in image.h */
#define EDGEFIRST_HAL_COLOR_MODE_CLASS     0u
#define EDGEFIRST_HAL_COLOR_MODE_INSTANCE  1u
#define EDGEFIRST_HAL_COLOR_MODE_TRACK     2u

/* Output type codes for ef_decoder_params_add_output in decoder.h */
#define EDGEFIRST_HAL_OUTPUT_DETECTION          0u
#define EDGEFIRST_HAL_OUTPUT_BOXES              1u
#define EDGEFIRST_HAL_OUTPUT_SCORES             2u
#define EDGEFIRST_HAL_OUTPUT_PROTOS             3u
#define EDGEFIRST_HAL_OUTPUT_SEGMENTATION       4u
#define EDGEFIRST_HAL_OUTPUT_MASK_COEFFICIENTS  5u
#define EDGEFIRST_HAL_OUTPUT_MASK               6u
#define EDGEFIRST_HAL_OUTPUT_CLASSES            7u

/* Decoder family codes for ef_decoder_params_add_output in decoder.h */
#define EDGEFIRST_HAL_DECODER_ULTRALYTICS  0u

/* Dimension name codes for ef_decoder_params_add_output in decoder.h */
#define EDGEFIRST_HAL_DIM_BATCH         0u
#define EDGEFIRST_HAL_DIM_HEIGHT        1u
#define EDGEFIRST_HAL_DIM_WIDTH         2u
#define EDGEFIRST_HAL_DIM_NUM_CLASSES   3u
#define EDGEFIRST_HAL_DIM_NUM_FEATURES  4u
#define EDGEFIRST_HAL_DIM_NUM_BOXES     5u
#define EDGEFIRST_HAL_DIM_NUM_PROTOS    6u
#define EDGEFIRST_HAL_DIM_BOX_COORDS    9u

/* Version codes for ef_decoder_params_set_decoder_version in decoder.h */
#define EDGEFIRST_HAL_DECODER_YOLOV5  0u
#define EDGEFIRST_HAL_DECODER_YOLOV8  1u
#define EDGEFIRST_HAL_DECODER_YOLO11  2u
#define EDGEFIRST_HAL_DECODER_YOLO26  3u

/* NMS mode codes for ef_decoder_params_set_nms in decoder.h */
#define EDGEFIRST_HAL_NMS_CLASS_AGNOSTIC  3u

/* ── Tensors ───────────────────────────────────────────────────────── */

/**
 * EdgefirstHalPlane:
 * @fd: DMA-BUF file descriptor; borrowed, the import duplicates it
 * @offset: byte offset of the plane within @fd, 0 for none
 * @stride: row stride in bytes, 0 to let HAL derive it from the format
 */
typedef struct {
  gint  fd;
  gsize offset;
  gsize stride;
} EdgefirstHalPlane;

/*
 * Import a DMA-BUF image as a HAL tensor. @chroma is NULL for a single
 * allocation, or the separate interleaved chroma plane of an NV12 image.
 * Returns NULL, with the reason logged to @obj, when HAL refuses the import
 * or the memory is not DMA-BUF.
 */
ef_tensor *edgefirst_hal_import_image (GstObject *obj,
    const EdgefirstHalPlane *image, const EdgefirstHalPlane *chroma,
    guint width, guint height, const EdgefirstHalFormat *fmt, uint32_t dtype);

/*
 * Wrap a DMA-BUF holding a tensor of @dtype and @shape. @fd is borrowed.
 * Returns NULL when HAL refuses it.
 */
ef_tensor *edgefirst_hal_wrap_fd (gint fd, uint32_t dtype,
    const guint64 *shape, guint ndim);

/* Allocate a host-memory tensor of @dtype and @shape. */
ef_tensor *edgefirst_hal_new_tensor (uint32_t dtype, const guint64 *shape,
    guint ndim);

/* Row stride of plane 0 in bytes, or 0 when HAL cannot describe it. */
gsize edgefirst_hal_row_stride (const ef_tensor *t);

/*
 * Allocate an image through @processor, letting HAL choose the backing
 * memory, with read-write CPU access.
 */
ef_tensor *edgefirst_hal_create_image (ef_image_processor *processor,
    guint width, guint height, const EdgefirstHalFormat *fmt, uint32_t dtype);

G_END_DECLS

#endif /* __EDGEFIRST_HAL_UTILS_H__ */
