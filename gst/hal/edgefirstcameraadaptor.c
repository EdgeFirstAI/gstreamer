/*
 * EdgeFirst Perception for GStreamer - Camera Adaptor Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-accelerated fused preprocessing: color conversion, resize,
 * letterbox, and quantization in a single element backed by edgefirst-hal.
 * Replaces multi-element chains (videoconvert ! videoscale ! tensor_converter
 * ! tensor_transform) with one step.
 *
 * Outputs NNStreamer-compatible other/tensors caps for direct connection
 * to tensor_filter.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstcameraadaptor.h"
#include "edgefirstcameraadaptor-neon.h"

#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include <edgefirst/hal.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_camera_adaptor_debug);

/* Monotonic clock for pipeline timing */
static inline guint64
_get_time_ns (void)
{
  struct timespec ts;
  clock_gettime (CLOCK_MONOTONIC, &ts);
  return (guint64) ts.tv_sec * 1000000000ULL + (guint64) ts.tv_nsec;
}
#define GST_CAT_DEFAULT edgefirst_camera_adaptor_debug

/* ── GEnum registrations ─────────────────────────────────────────── */

static GType
edgefirst_camera_adaptor_colorspace_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGB, "RGB", "rgb" },
      { EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR, "BGR", "bgr" },
      { EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY, "Grayscale", "gray" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstCameraAdaptorColorspace",
        values);
    g_once_init_leave (&type, t);
  }
  return type;
}

static GType
edgefirst_camera_adaptor_layout_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC, "HWC (interleaved)", "hwc" },
      { EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW, "CHW (planar)", "chw" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstCameraAdaptorLayout", values);
    g_once_init_leave (&type, t);
  }
  return type;
}

static GType
edgefirst_camera_adaptor_dtype_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8, "Unsigned 8-bit", "uint8" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8, "Signed 8-bit", "int8" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32, "32-bit float", "float32" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstCameraAdaptorDtype", values);
    g_once_init_leave (&type, t);
  }
  return type;
}

/* ── Property IDs ────────────────────────────────────────────────── */

enum {
  PROP_0,
  PROP_MODEL_WIDTH,
  PROP_MODEL_HEIGHT,
  PROP_COLORSPACE,
  PROP_LAYOUT,
  PROP_DTYPE,
  PROP_LETTERBOX,
  PROP_FILL_COLOR,
  PROP_LETTERBOX_SCALE,
  PROP_LETTERBOX_TOP,
  PROP_LETTERBOX_BOTTOM,
  PROP_LETTERBOX_LEFT,
  PROP_LETTERBOX_RIGHT,
  PROP_MEAN,
  PROP_STD,
};

/* ── Instance struct ─────────────────────────────────────────────── */

struct _EdgefirstCameraAdaptor {
  GstBaseTransform parent;

  /* Properties */
  guint model_width;
  guint model_height;
  EdgefirstCameraAdaptorColorspace colorspace;
  EdgefirstCameraAdaptorLayout layout;
  EdgefirstCameraAdaptorDtype dtype;
  gboolean letterbox;
  guint32 fill_color;          /* RGBA packed */
  gfloat lb_scale;             /* Letterbox scale (auto-calculated, read-only) */
  gint lb_top;                 /* Letterbox padding top (pixels) */
  gint lb_bottom;              /* Letterbox padding bottom (pixels) */
  gint lb_left;                /* Letterbox padding left (pixels) */
  gint lb_right;               /* Letterbox padding right (pixels) */
  gboolean lb_top_override;    /* User explicitly set padding */
  gboolean lb_bottom_override;
  gboolean lb_left_override;
  gboolean lb_right_override;
  gfloat mean[3];
  gfloat std[3];

  /* Runtime state */
  hal_image_processor *processor;
  GstVideoInfo in_info;
  gboolean in_info_valid;
  hal_crop crop;
  hal_fourcc src_fourcc;       /* Input HAL pixel format */
  hal_fourcc target_fourcc;    /* Output HAL pixel format for convert_ref */
  GstVideoFormat in_vformat;   /* Original GStreamer input format */
  gboolean needs_input_conversion; /* TRUE when in_vformat needs pixel rewrite */
  gboolean crop_valid;

  /* Output dimensions (resolved from properties + input) */
  guint out_width;
  guint out_height;
  guint out_channels;

  /* Two-stage pipeline */
  hal_tensor_image *intermediate;    /* Stage 1 destination (always RGBA) */
  hal_fourcc intermediate_fourcc;    /* HAL_FOURCC_RGBA (or GREY for grayscale) */
  gboolean use_two_stage;            /* TRUE when GPU/G2D hardware is available */

  /* Persistent tensors */
  hal_tensor *work_tensor;     /* U8 tensor, Stage 2 / convert_ref destination */
  hal_tensor *out_float;       /* F32 tensor (only when dtype=float32) */

  /* DMA-BUF state */
  GstAllocator *dmabuf_alloc;
  gboolean downstream_dmabuf;
  GstBuffer *out_dmabuf_buf;   /* Persistent DMA-BUF output (no-pool case) */
};

/* ── Pad templates ───────────────────────────────────────────────── */

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      "video/x-raw(memory:DMABuf), "
        "format={NV12, NV21, NV16, I420, YV12, YUY2, UYVY, "
        "RGB, BGR, RGBA, BGRA, RGBx, BGRx, GRAY8}, "
        "width=[1,MAX], height=[1,MAX]; "
      "video/x-raw, "
        "format={NV12, NV21, NV16, I420, YV12, YUY2, UYVY, "
        "RGB, BGR, RGBA, BGRA, RGBx, BGRx, GRAY8}, "
        "width=[1,MAX], height=[1,MAX]"
    ));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      "other/tensors, num_tensors=(int)1, format=(string)static"
    ));

/* ── Type definition ─────────────────────────────────────────────── */

#define edgefirst_camera_adaptor_parent_class parent_class
G_DEFINE_TYPE (EdgefirstCameraAdaptor, edgefirst_camera_adaptor,
    GST_TYPE_BASE_TRANSFORM);

/* ── Forward declarations ────────────────────────────────────────── */

static void edgefirst_camera_adaptor_set_property (GObject *, guint,
    const GValue *, GParamSpec *);
static void edgefirst_camera_adaptor_get_property (GObject *, guint,
    GValue *, GParamSpec *);
static void edgefirst_camera_adaptor_finalize (GObject *);
static gboolean edgefirst_camera_adaptor_start (GstBaseTransform *);
static gboolean edgefirst_camera_adaptor_stop (GstBaseTransform *);
static GstCaps *edgefirst_camera_adaptor_transform_caps (GstBaseTransform *,
    GstPadDirection, GstCaps *, GstCaps *);
static gboolean edgefirst_camera_adaptor_set_caps (GstBaseTransform *,
    GstCaps *, GstCaps *);
static gboolean edgefirst_camera_adaptor_transform_size (GstBaseTransform *,
    GstPadDirection, GstCaps *, gsize, GstCaps *, gsize *);
static gboolean edgefirst_camera_adaptor_propose_allocation (GstBaseTransform *,
    GstQuery *, GstQuery *);
static gboolean edgefirst_camera_adaptor_decide_allocation (GstBaseTransform *,
    GstQuery *);
static GstFlowReturn edgefirst_camera_adaptor_prepare_output_buffer (
    GstBaseTransform *, GstBuffer *, GstBuffer **);
static GstFlowReturn edgefirst_camera_adaptor_transform (GstBaseTransform *,
    GstBuffer *, GstBuffer *);

/* ── Helpers ─────────────────────────────────────────────────────── */

static hal_fourcc
gst_format_to_hal (GstVideoFormat fmt)
{
  switch (fmt) {
    case GST_VIDEO_FORMAT_NV12:  return HAL_FOURCC_NV12;
    case GST_VIDEO_FORMAT_NV21:  return HAL_FOURCC_NV12;  /* UV swap in copy */
    case GST_VIDEO_FORMAT_I420:  return HAL_FOURCC_NV12;  /* planar→semi in copy */
    case GST_VIDEO_FORMAT_YV12:  return HAL_FOURCC_NV12;  /* planar→semi in copy */
    case GST_VIDEO_FORMAT_NV16:  return HAL_FOURCC_NV16;
    case GST_VIDEO_FORMAT_YUY2:  return HAL_FOURCC_YUYV;
    case GST_VIDEO_FORMAT_UYVY:  return HAL_FOURCC_YUYV;  /* byte swap in copy */
    case GST_VIDEO_FORMAT_RGB:   return HAL_FOURCC_RGB;
    case GST_VIDEO_FORMAT_BGR:   return HAL_FOURCC_RGB;    /* R↔B swap in copy */
    case GST_VIDEO_FORMAT_RGBA:  return HAL_FOURCC_RGBA;
    case GST_VIDEO_FORMAT_BGRA:  return HAL_FOURCC_RGBA;   /* R↔B swap in copy */
    case GST_VIDEO_FORMAT_RGBx:  return HAL_FOURCC_RGBA;   /* x treated as alpha */
    case GST_VIDEO_FORMAT_BGRx:  return HAL_FOURCC_RGBA;   /* R↔B swap in copy */
    case GST_VIDEO_FORMAT_GRAY8: return HAL_FOURCC_GREY;
    default:                     return HAL_FOURCC_RGB;
  }
}

/**
 * Check whether the GStreamer format requires pixel conversion before
 * it matches the HAL fourcc returned by gst_format_to_hal().  Formats
 * that need conversion must go through the system-memory copy path.
 */
static gboolean
format_needs_conversion (GstVideoFormat fmt)
{
  switch (fmt) {
    case GST_VIDEO_FORMAT_NV21:
    case GST_VIDEO_FORMAT_I420:
    case GST_VIDEO_FORMAT_YV12:
    case GST_VIDEO_FORMAT_UYVY:
    case GST_VIDEO_FORMAT_BGR:
    case GST_VIDEO_FORMAT_BGRA:
    case GST_VIDEO_FORMAT_BGRx:
      return TRUE;
    default:
      return FALSE;
  }
}

static gboolean
parse_float_list (const gchar *str, gfloat *out, guint count)
{
  if (!str)
    return FALSE;

  gchar **parts = g_strsplit (str, ",", -1);
  guint n = g_strv_length (parts);
  gboolean ok = (n >= count);

  for (guint i = 0; i < count && i < n; i++)
    out[i] = (gfloat) g_ascii_strtod (g_strstrip (parts[i]), NULL);

  g_strfreev (parts);
  return ok;
}

static gsize
output_element_size (EdgefirstCameraAdaptorDtype dtype)
{
  return (dtype == EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32) ? 4 : 1;
}

static const char *
dtype_to_nnstreamer_string (EdgefirstCameraAdaptorDtype dtype)
{
  switch (dtype) {
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:    return "int8";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32: return "float32";
    default:                                      return "uint8";
  }
}

static void
cleanup_tensors (EdgefirstCameraAdaptor *self)
{
  g_clear_pointer (&self->out_float, hal_tensor_free);
  g_clear_pointer (&self->work_tensor, hal_tensor_free);
  g_clear_pointer (&self->intermediate, hal_tensor_image_free);
  gst_clear_buffer (&self->out_dmabuf_buf);
  self->use_two_stage = FALSE;
}

static void
compute_letterbox (EdgefirstCameraAdaptor *self, guint src_w, guint src_h)
{
  guint dst_w = self->out_width;
  guint dst_h = self->out_height;

  if (!self->letterbox || src_w == 0 || src_h == 0) {
    self->lb_scale = 1.0f;
    self->lb_top = self->lb_bottom = self->lb_left = self->lb_right = 0;
    self->crop_valid = FALSE;
    return;
  }

  /* Scale factor is always auto-calculated from aspect ratio */
  gfloat scale = MIN ((gfloat) dst_w / src_w, (gfloat) dst_h / src_h);
  guint new_w = (guint) (src_w * scale);
  guint new_h = (guint) (src_h * scale);

  self->lb_scale = scale;

  /* Auto-calculate centered padding; user overrides take precedence */
  gint total_h = (gint) (dst_h - new_h);
  gint total_w = (gint) (dst_w - new_w);

  if (!self->lb_top_override)
    self->lb_top = total_h / 2;
  if (!self->lb_bottom_override)
    self->lb_bottom = total_h - self->lb_top;
  if (!self->lb_left_override)
    self->lb_left = total_w / 2;
  if (!self->lb_right_override)
    self->lb_right = total_w - self->lb_left;

  /* Image placement from padding values */
  guint x = (guint) MAX (self->lb_left, 0);
  guint y = (guint) MAX (self->lb_top, 0);
  guint w = dst_w - (guint) MAX (self->lb_left, 0) - (guint) MAX (self->lb_right, 0);
  guint h = dst_h - (guint) MAX (self->lb_top, 0) - (guint) MAX (self->lb_bottom, 0);

  self->crop = hal_crop_new ();
  struct hal_rect dst_rect = hal_rect_new (x, y, w, h);
  hal_crop_set_dst_rect (&self->crop, &dst_rect);

  guint8 r = (self->fill_color >> 24) & 0xFF;
  guint8 g = (self->fill_color >> 16) & 0xFF;
  guint8 b = (self->fill_color >>  8) & 0xFF;
  guint8 a = (self->fill_color      ) & 0xFF;
  hal_crop_set_dst_color (&self->crop, r, g, b, a);

  self->crop_valid = TRUE;

  GST_DEBUG_OBJECT (self, "letterbox: %ux%u → %ux%u in %ux%u "
      "(scale %.4f, T=%d B=%d L=%d R=%d, fill #%02x%02x%02x)",
      src_w, src_h, w, h, dst_w, dst_h,
      self->lb_scale, self->lb_top, self->lb_bottom,
      self->lb_left, self->lb_right, r, g, b);
}

/**
 * Create a HAL tensor image from a GstBuffer.  Handles both DMA-BUF
 * (zero-copy wrap) and system-memory (allocate + copy) paths.
 * Returns a newly-allocated tensor image (caller must free).
 */
static hal_tensor_image *
create_input_image (EdgefirstCameraAdaptor *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width = GST_VIDEO_INFO_WIDTH (info);
  guint height = GST_VIDEO_INFO_HEIGHT (info);
  hal_fourcc fourcc = self->src_fourcc;

  /* Determine tensor shape for this pixel format */
  size_t shape[3];
  size_t ndim;
  size_t row_bytes;

  switch (fourcc) {
    case HAL_FOURCC_RGB:
      shape[0] = height; shape[1] = width; shape[2] = 3; ndim = 3;
      row_bytes = width * 3;
      break;
    case HAL_FOURCC_RGBA:
      shape[0] = height; shape[1] = width; shape[2] = 4; ndim = 3;
      row_bytes = width * 4;
      break;
    case HAL_FOURCC_GREY:
      shape[0] = height; shape[1] = width; shape[2] = 1; ndim = 3;
      row_bytes = width;
      break;
    case HAL_FOURCC_YUYV:
      shape[0] = height; shape[1] = width; shape[2] = 2; ndim = 3;
      row_bytes = width * 2;
      break;
    case HAL_FOURCC_NV12:
      shape[0] = height * 3 / 2; shape[1] = width; ndim = 2;
      row_bytes = width;
      break;
    case HAL_FOURCC_NV16:
      shape[0] = height * 2; shape[1] = width; ndim = 2;
      row_bytes = width;
      break;
    default:
      GST_ERROR_OBJECT (self, "unsupported input fourcc %d", fourcc);
      return NULL;
  }

  /* Try DMA-BUF zero-copy path (only for formats with direct HAL mapping) */
  GstMemory *in_mem = gst_buffer_peek_memory (inbuf, 0);
  if (!self->needs_input_conversion && gst_is_dmabuf_memory (in_mem)) {
    gint stride = GST_VIDEO_INFO_PLANE_STRIDE (info, 0);
    gboolean tight = ((gsize) stride == row_bytes);

    if (tight) {
      int fd = dup (gst_dmabuf_memory_get_fd (in_mem));
      if (fd >= 0) {
        hal_tensor *t = hal_tensor_from_fd (HAL_DTYPE_U8, fd, shape, ndim,
            "input");
        if (t) {
          hal_tensor_image *img = hal_tensor_image_from_tensor (t, fourcc);
          if (img)
            return img;
          /* hal_tensor_image_from_tensor takes ownership only on success;
           * on failure the tensor is still ours to free. */
          hal_tensor_free (t);
        } else {
          close (fd);
        }
      }
      GST_DEBUG_OBJECT (self, "DMA-BUF wrap failed, falling back to copy");
    }
  }

  /* System-memory path: allocate tensor and copy frame data */
  hal_tensor *tensor = hal_tensor_new (HAL_DTYPE_U8, shape, ndim,
      HAL_TENSOR_MEMORY_MEM, "input");
  if (!tensor)
    return NULL;

  hal_tensor_map *tmap = hal_tensor_map_create (tensor);
  if (!tmap) {
    hal_tensor_free (tensor);
    return NULL;
  }

  uint8_t *dst = hal_tensor_map_data (tmap);

  GstVideoFrame frame;
  if (!gst_video_frame_map (&frame, info, inbuf, GST_MAP_READ)) {
    hal_tensor_map_unmap (tmap);
    hal_tensor_free (tensor);
    return NULL;
  }

  GstVideoFormat vfmt = self->in_vformat;

  if (fourcc == HAL_FOURCC_NV12 && vfmt == GST_VIDEO_FORMAT_NV12) {
    /* NV12 native: copy Y + UV planes */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    const guint8 *uv_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height / 2; y++)
      memcpy (uv_dst + y * width, uv_data + y * uv_stride, width);

  } else if (fourcc == HAL_FOURCC_NV12 && vfmt == GST_VIDEO_FORMAT_NV21) {
    /* NV21 → NV12: copy Y plane, swap VU→UV in chroma plane */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    const guint8 *vu_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint vu_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height / 2; y++) {
      const guint8 *vu_row = vu_data + y * vu_stride;
      uint8_t *uv_row = uv_dst + y * width;
      for (guint x = 0; x < width; x += 2) {
        uv_row[x]     = vu_row[x + 1];  /* U */
        uv_row[x + 1] = vu_row[x];      /* V */
      }
    }

  } else if (fourcc == HAL_FOURCC_NV12 &&
      (vfmt == GST_VIDEO_FORMAT_I420 || vfmt == GST_VIDEO_FORMAT_YV12)) {
    /* I420/YV12 → NV12: copy Y, interleave U+V (or V+U) into semi-planar */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    /* I420: plane1=U, plane2=V.  YV12: plane1=V, plane2=U */
    int u_plane = (vfmt == GST_VIDEO_FORMAT_I420) ? 1 : 2;
    int v_plane = (vfmt == GST_VIDEO_FORMAT_I420) ? 2 : 1;
    const guint8 *u_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, u_plane);
    gint u_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, u_plane);
    const guint8 *v_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, v_plane);
    gint v_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, v_plane);
    guint half_w = width / 2;
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height / 2; y++) {
      const guint8 *u_row = u_data + y * u_stride;
      const guint8 *v_row = v_data + y * v_stride;
      uint8_t *uv_row = uv_dst + y * width;
      for (guint x = 0; x < half_w; x++) {
        uv_row[x * 2]     = u_row[x];
        uv_row[x * 2 + 1] = v_row[x];
      }
    }

  } else if (fourcc == HAL_FOURCC_NV16) {
    /* NV16: copy Y + UV planes (full-height UV for 4:2:2) */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    const guint8 *uv_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height; y++)
      memcpy (uv_dst + y * width, uv_data + y * uv_stride, width);

  } else if (fourcc == HAL_FOURCC_YUYV && vfmt == GST_VIDEO_FORMAT_UYVY) {
    /* UYVY → YUYV: swap byte pairs (U-Y-V-Y → Y-U-Y-V) */
    const guint8 *src_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++) {
      const guint8 *srow = src_data + y * stride;
      uint8_t *drow = dst + y * row_bytes;
      for (guint x = 0; x < row_bytes; x += 4) {
        drow[x]     = srow[x + 1];  /* Y0 */
        drow[x + 1] = srow[x];      /* U */
        drow[x + 2] = srow[x + 3];  /* Y1 */
        drow[x + 3] = srow[x + 2];  /* V */
      }
    }

  } else if (fourcc == HAL_FOURCC_RGB && vfmt == GST_VIDEO_FORMAT_BGR) {
    /* BGR → RGB: swap R↔B channels */
    const guint8 *src_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++) {
      const guint8 *srow = src_data + y * stride;
      uint8_t *drow = dst + y * row_bytes;
      for (guint x = 0; x < width; x++) {
        drow[x * 3]     = srow[x * 3 + 2];  /* R←B */
        drow[x * 3 + 1] = srow[x * 3 + 1];  /* G */
        drow[x * 3 + 2] = srow[x * 3];      /* B←R */
      }
    }

  } else if (fourcc == HAL_FOURCC_RGBA &&
      (vfmt == GST_VIDEO_FORMAT_BGRA || vfmt == GST_VIDEO_FORMAT_BGRx)) {
    /* BGRA/BGRx → RGBA: swap R↔B channels */
    const guint8 *src_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++) {
      const guint8 *srow = src_data + y * stride;
      uint8_t *drow = dst + y * row_bytes;
      for (guint x = 0; x < width; x++) {
        drow[x * 4]     = srow[x * 4 + 2];  /* R←B */
        drow[x * 4 + 1] = srow[x * 4 + 1];  /* G */
        drow[x * 4 + 2] = srow[x * 4];      /* B←R */
        drow[x * 4 + 3] = srow[x * 4 + 3];  /* A/x */
      }
    }

  } else {
    /* Direct copy for native formats (NV12 path handled above,
     * plus RGB, RGBA, RGBx, GRAY8, YUYV) */
    const guint8 *src_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    if ((gsize) stride == row_bytes) {
      memcpy (dst, src_data, row_bytes * height);
    } else {
      for (guint y = 0; y < height; y++)
        memcpy (dst + y * row_bytes, src_data + y * stride, row_bytes);
    }
  }

  gst_video_frame_unmap (&frame);
  hal_tensor_map_unmap (tmap);

  hal_tensor_image *img = hal_tensor_image_from_tensor (tensor, fourcc);
  if (!img)
    GST_ERROR_OBJECT (self, "hal_tensor_image_from_tensor failed");
  return img;
}

/* ── GObject lifecycle ───────────────────────────────────────────── */

static void
edgefirst_camera_adaptor_init (EdgefirstCameraAdaptor *self)
{
  self->model_width = 0;
  self->model_height = 0;
  self->colorspace = EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGB;
  self->layout = EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC;
  self->dtype = EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8;
  self->letterbox = FALSE;
  self->fill_color = 0x808080FF;  /* grey, full alpha */
  self->lb_scale = 0.0f;
  self->lb_top = self->lb_bottom = self->lb_left = self->lb_right = 0;
  self->lb_top_override = self->lb_bottom_override = FALSE;
  self->lb_left_override = self->lb_right_override = FALSE;
  self->mean[0] = self->mean[1] = self->mean[2] = 0.0f;
  self->std[0] = self->std[1] = self->std[2] = 1.0f;
  self->in_info_valid = FALSE;

  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (self), FALSE);
}

static void
edgefirst_camera_adaptor_finalize (GObject *object)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (object);

  cleanup_tensors (self);
  g_clear_pointer (&self->processor, hal_image_processor_free);
  gst_clear_object (&self->dmabuf_alloc);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

/* ── Properties ──────────────────────────────────────────────────── */

static void
edgefirst_camera_adaptor_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (object);

  switch (prop_id) {
    case PROP_MODEL_WIDTH:
      self->model_width = g_value_get_uint (value);
      break;
    case PROP_MODEL_HEIGHT:
      self->model_height = g_value_get_uint (value);
      break;
    case PROP_COLORSPACE:
      self->colorspace = g_value_get_enum (value);
      break;
    case PROP_LAYOUT:
      self->layout = g_value_get_enum (value);
      break;
    case PROP_DTYPE:
      self->dtype = g_value_get_enum (value);
      break;
    case PROP_LETTERBOX:
      self->letterbox = g_value_get_boolean (value);
      break;
    case PROP_FILL_COLOR:
      self->fill_color = g_value_get_uint (value);
      break;
    case PROP_LETTERBOX_TOP:
      self->lb_top = g_value_get_int (value);
      self->lb_top_override = TRUE;
      break;
    case PROP_LETTERBOX_BOTTOM:
      self->lb_bottom = g_value_get_int (value);
      self->lb_bottom_override = TRUE;
      break;
    case PROP_LETTERBOX_LEFT:
      self->lb_left = g_value_get_int (value);
      self->lb_left_override = TRUE;
      break;
    case PROP_LETTERBOX_RIGHT:
      self->lb_right = g_value_get_int (value);
      self->lb_right_override = TRUE;
      break;
    case PROP_MEAN:
      parse_float_list (g_value_get_string (value), self->mean, 3);
      break;
    case PROP_STD:
      parse_float_list (g_value_get_string (value), self->std, 3);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
edgefirst_camera_adaptor_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (object);

  switch (prop_id) {
    case PROP_MODEL_WIDTH:
      g_value_set_uint (value, self->model_width);
      break;
    case PROP_MODEL_HEIGHT:
      g_value_set_uint (value, self->model_height);
      break;
    case PROP_COLORSPACE:
      g_value_set_enum (value, self->colorspace);
      break;
    case PROP_LAYOUT:
      g_value_set_enum (value, self->layout);
      break;
    case PROP_DTYPE:
      g_value_set_enum (value, self->dtype);
      break;
    case PROP_LETTERBOX:
      g_value_set_boolean (value, self->letterbox);
      break;
    case PROP_FILL_COLOR:
      g_value_set_uint (value, self->fill_color);
      break;
    case PROP_LETTERBOX_SCALE:
      g_value_set_float (value, self->lb_scale);
      break;
    case PROP_LETTERBOX_TOP:
      g_value_set_int (value, self->lb_top);
      break;
    case PROP_LETTERBOX_BOTTOM:
      g_value_set_int (value, self->lb_bottom);
      break;
    case PROP_LETTERBOX_LEFT:
      g_value_set_int (value, self->lb_left);
      break;
    case PROP_LETTERBOX_RIGHT:
      g_value_set_int (value, self->lb_right);
      break;
    case PROP_MEAN: {
      gchar *str = g_strdup_printf ("%.6f,%.6f,%.6f",
          self->mean[0], self->mean[1], self->mean[2]);
      g_value_take_string (value, str);
      break;
    }
    case PROP_STD: {
      gchar *str = g_strdup_printf ("%.6f,%.6f,%.6f",
          self->std[0], self->std[1], self->std[2]);
      g_value_take_string (value, str);
      break;
    }
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* ── start / stop ────────────────────────────────────────────────── */

static gboolean
edgefirst_camera_adaptor_start (GstBaseTransform *trans)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  self->processor = hal_image_processor_new ();
  if (!self->processor) {
    GST_ELEMENT_ERROR (self, LIBRARY, INIT,
        ("Failed to create HAL image processor"), (NULL));
    return FALSE;
  }

  self->dmabuf_alloc = gst_dmabuf_allocator_new ();

  GST_INFO_OBJECT (self, "HAL image processor created");
  return TRUE;
}

static gboolean
edgefirst_camera_adaptor_stop (GstBaseTransform *trans)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  cleanup_tensors (self);
  g_clear_pointer (&self->processor, hal_image_processor_free);
  gst_clear_object (&self->dmabuf_alloc);
  self->in_info_valid = FALSE;

  return TRUE;
}

/* ── transform_caps ──────────────────────────────────────────────── */

static GstCaps *
edgefirst_camera_adaptor_transform_caps (GstBaseTransform *trans,
    GstPadDirection direction, GstCaps *caps, GstCaps *filter)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);
  GstCaps *result;

  if (direction == GST_PAD_SINK) {
    /* Sink → Src: produce tensor caps from video caps */
    guint width = self->model_width;
    guint height = self->model_height;

    /* If model dimensions not set, try input caps */
    if (width == 0 || height == 0) {
      GstStructure *s = gst_caps_get_structure (caps, 0);
      gint w = 0, h = 0;
      if (s) {
        gst_structure_get_int (s, "width", &w);
        gst_structure_get_int (s, "height", &h);
      }
      if (width == 0 && w > 0)
        width = (guint) w;
      if (height == 0 && h > 0)
        height = (guint) h;
    }

    if (width == 0 || height == 0) {
      /* Can't determine output dimensions yet */
      result = gst_caps_from_string (
          "other/tensors, num_tensors=(int)1, format=(string)static");
    } else {
      guint channels = (self->colorspace ==
          EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) ? 1 : 3;
      const char *type_str = dtype_to_nnstreamer_string (self->dtype);

      /* NNStreamer dimensions: innermost-to-outermost */
      gchar dims[64];
      if (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC)
        g_snprintf (dims, sizeof (dims), "%u:%u:%u:1",
            channels, width, height);
      else
        g_snprintf (dims, sizeof (dims), "%u:%u:%u:1",
            width, height, channels);

      result = gst_caps_new_simple ("other/tensors",
          "num_tensors", G_TYPE_INT, 1,
          "format", G_TYPE_STRING, "static",
          "types", G_TYPE_STRING, type_str,
          "dimensions", G_TYPE_STRING, dims,
          NULL);

      /* Propagate framerate from input */
      GstStructure *in_s = gst_caps_get_structure (caps, 0);
      if (in_s) {
        const GValue *fr = gst_structure_get_value (in_s, "framerate");
        if (fr) {
          GstStructure *out_s = gst_caps_get_structure (result, 0);
          gst_structure_set_value (out_s, "framerate", fr);
        }
      }
    }
  } else {
    /* Src → Sink: accept any supported video format */
    result = gst_static_pad_template_get_caps (&sink_template);
  }

  if (filter) {
    GstCaps *tmp = gst_caps_intersect_full (result, filter,
        GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (result);
    result = tmp;
  }

  GST_DEBUG_OBJECT (self, "transform_caps %s: %" GST_PTR_FORMAT
      " → %" GST_PTR_FORMAT,
      (direction == GST_PAD_SINK) ? "sink→src" : "src→sink",
      caps, result);

  return result;
}

/* ── set_caps ────────────────────────────────────────────────────── */

static gboolean
edgefirst_camera_adaptor_set_caps (GstBaseTransform *trans,
    GstCaps *incaps, GstCaps *outcaps G_GNUC_UNUSED)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  /* Parse input video info */
  if (!gst_video_info_from_caps (&self->in_info, incaps)) {
    GST_ERROR_OBJECT (self, "failed to parse input caps %" GST_PTR_FORMAT,
        incaps);
    return FALSE;
  }
  self->in_info_valid = TRUE;

  guint src_w = GST_VIDEO_INFO_WIDTH (&self->in_info);
  guint src_h = GST_VIDEO_INFO_HEIGHT (&self->in_info);
  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (&self->in_info);
  self->in_vformat = vfmt;
  self->src_fourcc = gst_format_to_hal (vfmt);
  self->needs_input_conversion = format_needs_conversion (vfmt);

  if (self->needs_input_conversion)
    GST_INFO_OBJECT (self, "input %s requires pixel conversion to HAL %d",
        gst_video_format_to_string (vfmt), self->src_fourcc);

  /* Resolve output dimensions */
  self->out_width = self->model_width > 0 ? self->model_width : src_w;
  self->out_height = self->model_height > 0 ? self->model_height : src_h;
  self->out_channels = (self->colorspace ==
      EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) ? 1 : 3;

  /* Determine target fourcc for convert_ref (fallback path) */
  if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) {
    self->target_fourcc = HAL_FOURCC_GREY;
  } else if (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW) {
    self->target_fourcc = HAL_FOURCC_PLANAR_RGB;
  } else {
    self->target_fourcc = HAL_FOURCC_RGB;
  }

  /* Intermediate format for two-stage pipeline.
   * Always RGBA — works with G2D on all platforms and avoids GL importing
   * VPU NV12 DMA-BUFs which deadlocks on Vivante DRM drivers.
   * For CHW output, NEON Stage 2 deinterleaves RGBA into planar. */
  if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) {
    self->intermediate_fourcc = HAL_FOURCC_GREY;
  } else {
    self->intermediate_fourcc = HAL_FOURCC_RGBA;
  }

  /* Compute letterbox geometry */
  compute_letterbox (self, src_w, src_h);

  GST_INFO_OBJECT (self, "configured: %ux%u %s → %ux%ux%u %s %s %s",
      src_w, src_h, gst_video_format_to_string (vfmt),
      self->out_width, self->out_height, self->out_channels,
      dtype_to_nnstreamer_string (self->dtype),
      self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW ? "CHW" : "HWC",
      self->letterbox ? "(letterbox)" : "");

  return TRUE;
}

/* ── transform_size ──────────────────────────────────────────────── */

static gboolean
edgefirst_camera_adaptor_transform_size (GstBaseTransform *trans,
    GstPadDirection direction, GstCaps *caps,
    gsize size G_GNUC_UNUSED,
    GstCaps *othercaps G_GNUC_UNUSED, gsize *othersize)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  if (direction == GST_PAD_SINK) {
    guint w = self->out_width;
    guint h = self->out_height;
    guint c = self->out_channels;

    /* If dimensions aren't resolved yet, try from caps */
    if (w == 0 || h == 0) {
      GstVideoInfo info;
      if (gst_video_info_from_caps (&info, caps)) {
        if (w == 0) w = self->model_width > 0 ? self->model_width :
            (guint) GST_VIDEO_INFO_WIDTH (&info);
        if (h == 0) h = self->model_height > 0 ? self->model_height :
            (guint) GST_VIDEO_INFO_HEIGHT (&info);
      }
      if (c == 0)
        c = (self->colorspace ==
            EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) ? 1 : 3;
    }

    if (w == 0 || h == 0)
      return FALSE;

    *othersize = w * h * c * output_element_size (self->dtype);
    return TRUE;
  }

  /* Reverse direction not supported */
  return FALSE;
}

/* ── Allocation ──────────────────────────────────────────────────── */

static gboolean
edgefirst_camera_adaptor_propose_allocation (
    GstBaseTransform *trans G_GNUC_UNUSED,
    GstQuery *decide_query G_GNUC_UNUSED,
    GstQuery *query G_GNUC_UNUSED)
{
  /* Don't propose a DMA-BUF allocator upstream.  Elements that natively
   * produce DMA-BUF (v4l2 decoders, ISP sources) will provide it anyway,
   * and we detect it in create_input_image().  Proposing a DMA-BUF allocator
   * here breaks intermediate CPU elements (videoconvert, videoscale) that
   * pick it up for their output pool but can't allocate DMA-BUF memory. */
  return TRUE;
}

static gboolean
edgefirst_camera_adaptor_decide_allocation (GstBaseTransform *trans,
    GstQuery *query)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);
  guint w = self->out_width;
  guint h = self->out_height;
  guint c = self->out_channels;

  if (w == 0 || h == 0) {
    GST_ERROR_OBJECT (self, "output dimensions not configured");
    return FALSE;
  }

  /* Clean up any previous tensors */
  cleanup_tensors (self);

  /* Check if downstream provided a buffer pool */
  guint n_pools = gst_query_get_n_allocation_pools (query);
  gboolean has_pool = (n_pools > 0);

  /* Check if downstream accepts DMA-BUF */
  self->downstream_dmabuf = FALSE;
  for (guint i = 0; i < gst_query_get_n_allocation_params (query); i++) {
    GstAllocator *alloc = NULL;
    gst_query_parse_nth_allocation_param (query, i, &alloc, NULL);
    if (alloc && GST_IS_DMABUF_ALLOCATOR (alloc))
      self->downstream_dmabuf = TRUE;
    gst_clear_object (&alloc);
  }

  /* Decide memory type for output tensors */
  gboolean dma_available = hal_is_dma_available ();
  enum hal_tensor_memory mem_type =
      (self->downstream_dmabuf && dma_available) ?
      HAL_TENSOR_MEMORY_DMA : HAL_TENSOR_MEMORY_MEM;

  /* ── Two-stage pipeline: allocate intermediate image for GPU/G2D ── *
   * The intermediate uses DMA when available — GPU/G2D backends need
   * DMA-backed destinations for hardware-accelerated conversion.
   * System memory intermediate only works with G2D (not GPU). */
  self->intermediate = hal_tensor_image_new (w, h,
      self->intermediate_fourcc,
      dma_available ? HAL_TENSOR_MEMORY_DMA : HAL_TENSOR_MEMORY_MEM);
  self->use_two_stage = (self->intermediate != NULL);

  if (self->use_two_stage) {
    GST_INFO_OBJECT (self, "two-stage pipeline: intermediate=RGBA");
  } else {
    GST_INFO_OBJECT (self, "two-stage pipeline unavailable, "
        "falling back to convert_ref");
  }

  /* ── Output tensor shape ── */
  size_t work_shape[3];
  size_t work_ndim = 3;

  if (self->target_fourcc == HAL_FOURCC_PLANAR_RGB) {
    work_shape[0] = c; work_shape[1] = h; work_shape[2] = w;
  } else {
    work_shape[0] = h; work_shape[1] = w; work_shape[2] = c;
  }

  /* ── Allocate work tensor + float tensor ── */
  {
    self->work_tensor = hal_tensor_new (HAL_DTYPE_U8, work_shape, work_ndim,
        mem_type, "work");
    if (!self->work_tensor) {
      GST_ERROR_OBJECT (self, "failed to allocate work tensor");
      return FALSE;
    }

    /* Allocate float output tensor if needed */
    if (self->dtype == EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32) {
      self->out_float = hal_tensor_new (HAL_DTYPE_F32, work_shape, work_ndim,
          mem_type, "output_f32");
      if (!self->out_float) {
        GST_ERROR_OBJECT (self, "failed to allocate float output tensor");
        return FALSE;
      }
    }

    /* Set up persistent DMA-BUF output buffer for the no-pool case */
    if (!has_pool && self->downstream_dmabuf && self->dmabuf_alloc) {
      hal_tensor *final_tensor = self->out_float ? self->out_float :
          self->work_tensor;
      int fd = hal_tensor_clone_fd (final_tensor);
      if (fd >= 0) {
        gsize tensor_size = hal_tensor_size (final_tensor);
        GstMemory *mem = gst_dmabuf_allocator_alloc (self->dmabuf_alloc,
            fd, tensor_size);
        if (mem) {
          self->out_dmabuf_buf = gst_buffer_new ();
          gst_buffer_append_memory (self->out_dmabuf_buf, mem);
          GST_INFO_OBJECT (self, "persistent DMA-BUF output buffer created "
              "(%zu bytes, fd=%d)", tensor_size, fd);
        } else {
          close (fd);
        }
      }
    }
  }

  GST_DEBUG_OBJECT (self, "allocation decided: two_stage=%s mem=%s "
      "pool=%s dmabuf_out=%s",
      self->use_two_stage ? "yes" : "no",
      mem_type == HAL_TENSOR_MEMORY_DMA ? "DMA" : "MEM",
      has_pool ? "yes" : "no",
      self->out_dmabuf_buf ? "yes" : "no");

  /* Don't chain to parent's decide_allocation — the parent tries to set up
   * a buffer pool for the output caps, but other/tensors caps don't have
   * a known buffer size in the allocation query.  We handle output buffer
   * allocation ourselves in prepare_output_buffer. */
  return TRUE;
}

/* ── prepare_output_buffer ───────────────────────────────────────── */

static GstFlowReturn
edgefirst_camera_adaptor_prepare_output_buffer (GstBaseTransform *trans,
    GstBuffer *inbuf G_GNUC_UNUSED, GstBuffer **outbuf)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  if (self->out_dmabuf_buf) {
    /* Reuse persistent DMA-BUF buffer */
    *outbuf = gst_buffer_ref (self->out_dmabuf_buf);
    return GST_FLOW_OK;
  }

  /* Allocate system-memory output buffer with known size.  We can't rely
   * on the base class pool because other/tensors caps don't have a
   * standard pool negotiation path. */
  gsize out_size = self->out_width * self->out_height * self->out_channels
      * output_element_size (self->dtype);
  *outbuf = gst_buffer_new_allocate (NULL, out_size, NULL);
  if (!*outbuf) {
    GST_ERROR_OBJECT (self, "failed to allocate output buffer (%"
        G_GSIZE_FORMAT " bytes)", out_size);
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;
}

/* ── Stage 2: NEON post-processing ────────────────────────────────── */

/**
 * Stage 2 of the two-stage pipeline: NEON post-processing.
 *
 * Reads from the RGBA u8 intermediate image (from GPU/G2D Stage 1) and
 * writes the final model format into the output buffer.
 */
static GstFlowReturn
camera_adaptor_stage2 (EdgefirstCameraAdaptor *self,
    GstBuffer *outbuf, GstBuffer *inbuf)
{
  gsize npixels = self->out_width * self->out_height;
  gboolean bgr = (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR);

  /* Map intermediate image for reading */
  hal_tensor_map *imap = hal_tensor_image_map_create (self->intermediate);
  if (!imap) {
    GST_ERROR_OBJECT (self, "failed to map intermediate image");
    return GST_FLOW_ERROR;
  }
  const uint8_t *isrc = hal_tensor_map_data_const (imap);

  if (self->out_dmabuf_buf) {
    /* ── DMA-BUF output: NEON writes to work_tensor / out_float ── */

    if (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
      /* HWC path: RGBA → interleaved RGB */
      switch (self->dtype) {
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8: {
          hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
          uint8_t *wdst = hal_tensor_map_data (wmap);
          edgefirst_neon_rgba_to_rgb_u8 (isrc, wdst, npixels, bgr);
          hal_tensor_map_unmap (wmap);
          break;
        }
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8: {
          hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
          uint8_t *wdst = hal_tensor_map_data (wmap);
          edgefirst_neon_rgba_to_rgb_i8 (isrc, wdst, npixels, bgr);
          hal_tensor_map_unmap (wmap);
          break;
        }
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32: {
          hal_tensor_map *fmap = hal_tensor_map_create (self->out_float);
          float *fdst = hal_tensor_map_data (fmap);
          edgefirst_neon_rgba_to_rgb_f32 (isrc, fdst, npixels,
              self->mean, self->std, bgr);
          hal_tensor_map_unmap (fmap);
          break;
        }
      }
    } else {
      /* CHW path: RGBA intermediate → planar output via NEON deinterleave */
      switch (self->dtype) {
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8: {
          hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
          uint8_t *wdst = hal_tensor_map_data (wmap);
          edgefirst_neon_rgba_to_planar_u8 (isrc, wdst, npixels, bgr);
          hal_tensor_map_unmap (wmap);
          break;
        }
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8: {
          hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
          uint8_t *wdst = hal_tensor_map_data (wmap);
          edgefirst_neon_rgba_to_planar_i8 (isrc, wdst, npixels, bgr);
          hal_tensor_map_unmap (wmap);
          break;
        }
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32: {
          hal_tensor_map *fmap = hal_tensor_map_create (self->out_float);
          float *fdst = hal_tensor_map_data (fmap);
          edgefirst_neon_rgba_to_planar_f32 (isrc, fdst, npixels,
              self->mean, self->std, bgr);
          hal_tensor_map_unmap (fmap);
          break;
        }
      }
    }

    hal_tensor_map_unmap (imap);
    gst_buffer_copy_into (outbuf, inbuf,
        GST_BUFFER_COPY_TIMESTAMPS | GST_BUFFER_COPY_FLAGS, 0, -1);

  } else {
    /* ── System memory output: NEON writes directly to outbuf ── */
    GstMapInfo omap;
    if (!gst_buffer_map (outbuf, &omap, GST_MAP_WRITE)) {
      hal_tensor_map_unmap (imap);
      GST_ERROR_OBJECT (self, "failed to map output buffer");
      return GST_FLOW_ERROR;
    }

    if (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
      /* HWC path: RGBA → interleaved RGB */
      switch (self->dtype) {
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8:
          edgefirst_neon_rgba_to_rgb_u8 (isrc, omap.data, npixels, bgr);
          break;
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:
          edgefirst_neon_rgba_to_rgb_i8 (isrc, omap.data, npixels, bgr);
          break;
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32:
          edgefirst_neon_rgba_to_rgb_f32 (isrc, (float *) omap.data, npixels,
              self->mean, self->std, bgr);
          break;
      }
    } else {
      /* CHW path: RGBA → planar via NEON deinterleave */
      switch (self->dtype) {
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8:
          edgefirst_neon_rgba_to_planar_u8 (isrc, omap.data, npixels, bgr);
          break;
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:
          edgefirst_neon_rgba_to_planar_i8 (isrc, omap.data, npixels, bgr);
          break;
        case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32:
          edgefirst_neon_rgba_to_planar_f32 (isrc, (float *) omap.data,
              npixels, self->mean, self->std, bgr);
          break;
      }
    }

    hal_tensor_map_unmap (imap);
    gst_buffer_unmap (outbuf, &omap);
  }

  return GST_FLOW_OK;
}

/* ── transform ───────────────────────────────────────────────────── */

static GstFlowReturn
edgefirst_camera_adaptor_transform (GstBaseTransform *trans,
    GstBuffer *inbuf, GstBuffer *outbuf)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);
  guint64 t0 = _get_time_ns ();

  /* ── INPUT: Create HAL tensor image from video frame ── */
  hal_tensor_image *src_img = create_input_image (self, inbuf);
  if (!src_img) {
    GST_ERROR_OBJECT (self, "failed to create input image");
    return GST_FLOW_ERROR;
  }

  guint64 t_input = _get_time_ns ();

  /* ── Two-stage path: GPU/G2D → NEON ── */
  if (self->use_two_stage) {
    /* Stage 1: GPU/G2D hardware — convert into owned intermediate image */
    int ret = hal_image_processor_convert (self->processor, src_img,
        self->intermediate, HAL_ROTATION_NONE, HAL_FLIP_NONE,
        self->crop_valid ? &self->crop : NULL);

    guint64 t_stage1 = _get_time_ns ();

    if (ret != 0) {
      /* Hardware convert failed — permanently fall back to convert_ref.
       * This happens when GPU/G2D are unavailable (headless, no driver). */
      GST_WARNING_OBJECT (self, "HAL convert (stage 1) failed (%d), "
          "falling back to convert_ref for all future frames", ret);
      self->use_two_stage = FALSE;
      /* Fall through to convert_ref path below (src_img still valid) */
    } else {
      hal_tensor_image_free (src_img);

      /* Stage 2: NEON post-processing → output buffer */
      GstFlowReturn flow = camera_adaptor_stage2 (self, outbuf, inbuf);

      guint64 t_done = _get_time_ns ();
      GST_LOG_OBJECT (self,
          "TWO-STAGE input=%.3fms stage1=%.3fms stage2=%.3fms total=%.3fms",
          (t_input - t0) / 1e6, (t_stage1 - t_input) / 1e6,
          (t_done - t_stage1) / 1e6, (t_done - t0) / 1e6);
      return flow;
    }
  }

  /* ── Fallback path: convert_ref + scalar post-processing ── */
  if (!self->work_tensor) {
    hal_tensor_image_free (src_img);
    GST_ERROR_OBJECT (self, "work tensor not allocated");
    return GST_FLOW_ERROR;
  }

  int ret = hal_image_processor_convert_ref (self->processor, src_img,
      self->work_tensor, self->target_fourcc,
      HAL_ROTATION_NONE, HAL_FLIP_NONE,
      self->crop_valid ? &self->crop : NULL);

  hal_tensor_image_free (src_img);

  guint64 t_ref = _get_time_ns ();

  if (ret != 0) {
    GST_ERROR_OBJECT (self, "HAL convert_ref failed (%d)", ret);
    return GST_FLOW_ERROR;
  }

  /* ── POST-PROCESS + OUTPUT (original scalar path) ── */
  gsize out_size = self->out_width * self->out_height * self->out_channels;

  if (self->out_dmabuf_buf) {
    /* DMA-BUF output: post-process in-place on the tensor */
    if (self->dtype == EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8) {
      hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
      uint8_t *data = hal_tensor_map_data (wmap);
      for (gsize i = 0; i < out_size; i++)
        data[i] ^= 0x80;
      if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR &&
          self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
        for (gsize i = 0; i < out_size; i += 3) {
          uint8_t tmp = data[i];
          data[i] = data[i + 2];
          data[i + 2] = tmp;
        }
      }
      hal_tensor_map_unmap (wmap);

    } else if (self->dtype == EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32) {
      hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
      const uint8_t *src = hal_tensor_map_data_const (wmap);
      hal_tensor_map *fmap = hal_tensor_map_create (self->out_float);
      float *dst = hal_tensor_map_data (fmap);
      guint c = self->out_channels;
      for (gsize i = 0; i < out_size; i++) {
        guint ch = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) ?
            (i % c) : (i / (self->out_width * self->out_height));
        dst[i] = ((float) src[i] / 255.0f - self->mean[ch]) / self->std[ch];
      }
      hal_tensor_map_unmap (fmap);
      hal_tensor_map_unmap (wmap);

    } else if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR &&
        self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
      hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
      uint8_t *data = hal_tensor_map_data (wmap);
      for (gsize i = 0; i < out_size; i += 3) {
        uint8_t tmp = data[i];
        data[i] = data[i + 2];
        data[i + 2] = tmp;
      }
      hal_tensor_map_unmap (wmap);
    }

    gst_buffer_copy_into (outbuf, inbuf,
        GST_BUFFER_COPY_TIMESTAMPS | GST_BUFFER_COPY_FLAGS, 0, -1);

  } else {
    /* System memory output: copy from tensor to output buffer */
    GstMapInfo omap;
    if (!gst_buffer_map (outbuf, &omap, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT (self, "failed to map output buffer");
      return GST_FLOW_ERROR;
    }

    hal_tensor_map *wmap = hal_tensor_map_create (self->work_tensor);
    const uint8_t *src = hal_tensor_map_data_const (wmap);

    switch (self->dtype) {
      case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8:
        if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR &&
            self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
          for (gsize i = 0; i < out_size; i += 3) {
            omap.data[i]     = src[i + 2];
            omap.data[i + 1] = src[i + 1];
            omap.data[i + 2] = src[i];
          }
        } else if (self->colorspace ==
            EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR &&
            self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW) {
          gsize plane = self->out_width * self->out_height;
          memcpy (omap.data, src + 2 * plane, plane);
          memcpy (omap.data + plane, src + plane, plane);
          memcpy (omap.data + 2 * plane, src, plane);
        } else {
          memcpy (omap.data, src, out_size);
        }
        break;

      case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:
        for (gsize i = 0; i < out_size; i++)
          omap.data[i] = src[i] ^ 0x80;
        if (self->colorspace == EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR &&
            self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) {
          for (gsize i = 0; i < out_size; i += 3) {
            uint8_t tmp = omap.data[i];
            omap.data[i] = omap.data[i + 2];
            omap.data[i + 2] = tmp;
          }
        }
        break;

      case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32: {
        float *fout = (float *) omap.data;
        guint c = self->out_channels;
        for (gsize i = 0; i < out_size; i++) {
          guint ch = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC) ?
              (i % c) : (i / (self->out_width * self->out_height));
          fout[i] = ((float) src[i] / 255.0f - self->mean[ch]) / self->std[ch];
        }
        break;
      }
    }

    hal_tensor_map_unmap (wmap);
    gst_buffer_unmap (outbuf, &omap);
  }

  {
    guint64 t_done = _get_time_ns ();
    GST_LOG_OBJECT (self,
        "FALLBACK input=%.3fms convert_ref=%.3fms postproc=%.3fms total=%.3fms",
        (t_input - t0) / 1e6, (t_ref - t_input) / 1e6,
        (t_done - t_ref) / 1e6, (t_done - t0) / 1e6);
  }

  return GST_FLOW_OK;
}

/* ── class_init ──────────────────────────────────────────────────── */

static void
edgefirst_camera_adaptor_class_init (EdgefirstCameraAdaptorClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS (klass);

  gobject_class->set_property = edgefirst_camera_adaptor_set_property;
  gobject_class->get_property = edgefirst_camera_adaptor_get_property;
  gobject_class->finalize = edgefirst_camera_adaptor_finalize;

  /* Properties */
  g_object_class_install_property (gobject_class, PROP_MODEL_WIDTH,
      g_param_spec_uint ("model-width", "Model Width",
          "Target width for model input (0 = use input width)",
          0, G_MAXUINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_MODEL_HEIGHT,
      g_param_spec_uint ("model-height", "Model Height",
          "Target height for model input (0 = use input height)",
          0, G_MAXUINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_COLORSPACE,
      g_param_spec_enum ("model-colorspace", "Model Colorspace",
          "Output color space for model input",
          edgefirst_camera_adaptor_colorspace_get_type (),
          EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGB,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LAYOUT,
      g_param_spec_enum ("model-layout", "Model Layout",
          "Tensor memory layout (HWC interleaved or CHW planar)",
          edgefirst_camera_adaptor_layout_get_type (),
          EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_DTYPE,
      g_param_spec_enum ("model-dtype", "Model Data Type",
          "Output tensor data type",
          edgefirst_camera_adaptor_dtype_get_type (),
          EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX,
      g_param_spec_boolean ("letterbox", "Letterbox",
          "Preserve aspect ratio with padding",
          FALSE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_FILL_COLOR,
      g_param_spec_uint ("fill-color", "Fill Color",
          "RGBA fill color for letterbox padding (default 0x808080FF gray)",
          0, G_MAXUINT32, 0x808080FF,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX_SCALE,
      g_param_spec_float ("letterbox-scale", "Letterbox Scale",
          "Scale factor applied to the input image (read-only, auto-calculated)",
          0.0f, G_MAXFLOAT, 0.0f,
          G_PARAM_READABLE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX_TOP,
      g_param_spec_int ("letterbox-top", "Letterbox Top",
          "Top padding in pixels. Auto-calculated when letterbox=true; "
          "set to override for non-centered placement.",
          0, G_MAXINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX_BOTTOM,
      g_param_spec_int ("letterbox-bottom", "Letterbox Bottom",
          "Bottom padding in pixels. Auto-calculated when letterbox=true; "
          "set to override for non-centered placement.",
          0, G_MAXINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX_LEFT,
      g_param_spec_int ("letterbox-left", "Letterbox Left",
          "Left padding in pixels. Auto-calculated when letterbox=true; "
          "set to override for non-centered placement.",
          0, G_MAXINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_LETTERBOX_RIGHT,
      g_param_spec_int ("letterbox-right", "Letterbox Right",
          "Right padding in pixels. Auto-calculated when letterbox=true; "
          "set to override for non-centered placement.",
          0, G_MAXINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_MEAN,
      g_param_spec_string ("model-mean", "Model Mean",
          "Per-channel mean for float32 normalization "
          "(comma-separated, e.g. \"0.485,0.456,0.406\")",
          "0.0,0.0,0.0",
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_STD,
      g_param_spec_string ("model-std", "Model Std",
          "Per-channel std for float32 normalization "
          "(comma-separated, e.g. \"0.229,0.224,0.225\")",
          "1.0,1.0,1.0",
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /* Element metadata */
  gst_element_class_set_static_metadata (element_class,
      "EdgeFirst Camera Adaptor",
      "Filter/Converter/Video",
      "Hardware-accelerated fused image preprocessing for ML inference",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  /* Virtual methods */
  trans_class->start = edgefirst_camera_adaptor_start;
  trans_class->stop = edgefirst_camera_adaptor_stop;
  trans_class->transform_caps = edgefirst_camera_adaptor_transform_caps;
  trans_class->set_caps = edgefirst_camera_adaptor_set_caps;
  trans_class->transform_size = edgefirst_camera_adaptor_transform_size;
  trans_class->propose_allocation = edgefirst_camera_adaptor_propose_allocation;
  trans_class->decide_allocation = edgefirst_camera_adaptor_decide_allocation;
  trans_class->prepare_output_buffer =
      edgefirst_camera_adaptor_prepare_output_buffer;
  trans_class->transform = edgefirst_camera_adaptor_transform;

  trans_class->passthrough_on_same_caps = FALSE;

  GST_DEBUG_CATEGORY_INIT (edgefirst_camera_adaptor_debug,
      "edgefirstcameraadaptor", 0, "EdgeFirst Camera Adaptor");
}
