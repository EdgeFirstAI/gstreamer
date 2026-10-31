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

#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include "edgefirsthalutils.h"
#include <errno.h>
#include <sys/stat.h>
#include <time.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_camera_adaptor_debug);
GST_DEBUG_CATEGORY_STATIC (edgefirst_hal_debug);

/* ── HAL log → GST_DEBUG bridge ──────────────────────────────────── */

static void
hal_log_to_gst (ef_log_level level, const char *target,
    const char *message, void *userdata G_GNUC_UNUSED)
{
  GstDebugLevel gst_level;
  switch (level) {
    case EF_LOG_LEVEL_ERROR: gst_level = GST_LEVEL_ERROR; break;
    case EF_LOG_LEVEL_WARN:  gst_level = GST_LEVEL_WARNING; break;
    case EF_LOG_LEVEL_INFO:  gst_level = GST_LEVEL_INFO; break;
    case EF_LOG_LEVEL_DEBUG: gst_level = GST_LEVEL_DEBUG; break;
    case EF_LOG_LEVEL_TRACE: gst_level = GST_LEVEL_TRACE; break;
    default:                 gst_level = GST_LEVEL_LOG; break;
  }
  gst_debug_log (edgefirst_hal_debug, gst_level, target, "", 0, NULL,
      "%s", message);
}

static ef_log_level
gst_level_to_hal (GstDebugLevel gst_level)
{
  if (gst_level <= GST_LEVEL_ERROR)   return EF_LOG_LEVEL_ERROR;
  if (gst_level <= GST_LEVEL_WARNING) return EF_LOG_LEVEL_WARN;
  if (gst_level <= GST_LEVEL_INFO)    return EF_LOG_LEVEL_INFO;
  if (gst_level <= GST_LEVEL_DEBUG)   return EF_LOG_LEVEL_DEBUG;
  return EF_LOG_LEVEL_TRACE;
}

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
      { EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGBA, "RGBA", "rgba" },
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
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT16, "Unsigned 16-bit", "uint16" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT16, "Signed 16-bit", "int16" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT32, "Unsigned 32-bit", "uint32" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT32, "Signed 32-bit", "int32" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT64, "Unsigned 64-bit", "uint64" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT64, "Signed 64-bit", "int64" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT16, "16-bit float", "float16" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32, "32-bit float", "float32" },
      { EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT64, "64-bit float", "float64" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstCameraAdaptorDtype", values);
    g_once_init_leave (&type, t);
  }
  return type;
}

static GType
edgefirst_camera_adaptor_compute_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO, "Auto (HAL default)", "auto" },
      { EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_OPENGL, "OpenGL", "opengl" },
      { EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_G2D, "G2D", "g2d" },
      { EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_CPU, "CPU", "cpu" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstCameraAdaptorCompute", values);
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
  PROP_COMPUTE,
  PROP_LETTERBOX,
  PROP_FILL_COLOR,
  PROP_LETTERBOX_SCALE,
  PROP_LETTERBOX_TOP,
  PROP_LETTERBOX_BOTTOM,
  PROP_LETTERBOX_LEFT,
  PROP_LETTERBOX_RIGHT,
  PROP_MODEL_MEAN,
  PROP_MODEL_STD,
};

/* ── Instance struct ─────────────────────────────────────────────── */

/* How the letterbox placement reaches HAL. HAL letterboxes natively only
 * into the centred rectangle it computes itself; any other placement is a
 * convert into a view of the output, which HAL offers for packed formats
 * only, so a planar output is staged through a packed image. */
typedef enum {
  LETTERBOX_NONE,     /* stretch to the whole output */
  LETTERBOX_NATIVE,   /* HAL letterbox: the placement HAL computes */
  LETTERBOX_VIEW,     /* packed output: convert into a view of it */
  LETTERBOX_STAGED,   /* planar output: view of a packed U8 stage */
} LetterboxMode;

struct _EdgefirstCameraAdaptor {
  GstBaseTransform parent;

  /* Properties */
  guint model_width;
  guint model_height;
  EdgefirstCameraAdaptorColorspace colorspace;
  EdgefirstCameraAdaptorLayout layout;
  EdgefirstCameraAdaptorDtype dtype;
  EdgefirstCameraAdaptorCompute compute;
  gboolean letterbox;
  guint32 fill_color;
  gfloat lb_scale;
  gint lb_top, lb_bottom, lb_left, lb_right;
  gboolean lb_top_override, lb_bottom_override;
  gboolean lb_left_override, lb_right_override;
  gchar *model_mean;
  gchar *model_std;

  /* Runtime state */
  ef_image_processor *processor;
  GstVideoInfo in_info;
  gboolean in_info_valid;

  /* Letterbox placement: the image is converted into this rectangle of the
   * output and the rest of the output holds fill_rgba. */
  LetterboxMode lb_mode;
  guint dst_x;
  guint dst_y;
  guint dst_w;
  guint dst_h;
  guint8 fill_rgba[4];
  ef_crop native_crop;       /* LETTERBOX_NATIVE */
  ef_tensor *stage;          /* LETTERBOX_STAGED: packed U8 output */
  ef_tensor *stage_view;     /* LETTERBOX_STAGED: placement within stage */

  /* Output dimensions */
  guint out_width, out_height, out_channels;

  /* Tensor caches — import once per DMA-BUF, reuse across frames.
   * Input cache keys are heap-allocated InputCacheKey structs (inode + offset).
   * Using inode instead of fd makes the cache robust to fd number recycling:
   * the kernel dma_buf inode is stable for the lifetime of the buffer. */
  GHashTable *input_cache;   /* InputCacheKey* -> ef_tensor* */
  GHashTable *output_cache;  /* fd -> OutputTensor* (pool has one buffer, offset=0) */
  struct OutputTensor *hal_output;  /* HAL-owned output (when no downstream pool) */

  /* DMA-BUF state */
  gboolean downstream_dmabuf;
  GstBufferPool *downstream_pool;
  gboolean input_is_drm;

  /* Target HAL format/dtype (resolved from properties) */
  const EdgefirstHalFormat *target_format;
  uint32_t target_dtype;
};

/* ── Pad templates ───────────────────────────────────────────────── */

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      GST_VIDEO_DMA_DRM_CAPS_MAKE "; "
      "video/x-raw(memory:DMABuf), "
        "format={NV12, NV21, NV16, I420, YV12, YUY2, UYVY, "
        "RGB, BGR, RGBA, BGRA, RGBx, BGRx, GRAY8}, "
        "width=[1,MAX], height=[1,MAX]; "
      /* Sources like libcamerasrc declare video/x-raw (no memory:DMABuf feature)
       * even when their allocator produces DMABuf-backed memory (linear/mappable
       * DMABuf follows the GStreamer convention of omitting the caps feature).
       * Accept video/x-raw here so caps negotiation succeeds; the actual buffer
       * memory type is checked at runtime via gst_is_dmabuf_memory(). */
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

static const char *
dtype_to_nnstreamer_string (EdgefirstCameraAdaptorDtype dtype)
{
  switch (dtype) {
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8:   return "uint8";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:    return "int8";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT16:  return "uint16";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT16:   return "int16";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT32:  return "uint32";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT32:   return "int32";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT64:  return "uint64";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT64:   return "int64";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT16: return "float16";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32: return "float32";
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT64: return "float64";
    default:                                      return "uint8";
  }
}

static guint
dtype_byte_size (EdgefirstCameraAdaptorDtype dtype)
{
  switch (dtype) {
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8:
      return 1;
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT16:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT16:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT16:
      return 2;
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT32:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT32:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32:
      return 4;
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT64:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT64:
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT64:
      return 8;
    default:
      return 1;
  }
}

static void
resolve_target_format (EdgefirstCameraAdaptor *self)
{
  static const uint32_t dtype_map[] = {
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8]   = EF_DTYPE_U8,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8]    = EF_DTYPE_I8,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT16]  = EF_DTYPE_U16,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT16]   = EF_DTYPE_I16,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT32]  = EF_DTYPE_U32,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT32]   = EF_DTYPE_I32,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT64]  = EF_DTYPE_U64,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT64]   = EF_DTYPE_I64,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT16] = EF_DTYPE_F16,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32] = EF_DTYPE_F32,
    [EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT64] = EF_DTYPE_F64,
  };
  const gchar *wire;
  self->target_dtype = dtype_map[self->dtype];
  switch (self->colorspace) {
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY:
      wire = EDGEFIRST_HAL_FORMAT_GREY;
      break;
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR:
      wire = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW)
          ? EDGEFIRST_HAL_FORMAT_PLANAR_RGB : EDGEFIRST_HAL_FORMAT_BGRA;
      break;
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGBA:
      wire = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW)
          ? EDGEFIRST_HAL_FORMAT_PLANAR_RGBA : EDGEFIRST_HAL_FORMAT_RGBA;
      break;
    default:
      wire = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW)
          ? EDGEFIRST_HAL_FORMAT_PLANAR_RGB : EDGEFIRST_HAL_FORMAT_RGB;
      break;
  }
  self->target_format = edgefirst_hal_format_from_wire (wire);
}

/* ── Tensor cache helpers ─────────────────────────────────────────── */

static void
tensor_cache_value_free (gpointer data)
{
  ef_tensor_free ((ef_tensor *) data);
}

/* An output tensor and, when letterboxing, the view of it that receives
 * the converted image. */
typedef struct OutputTensor {
  ef_tensor *full;
  ef_tensor *view;   /* NULL when the image fills the whole output */
} OutputTensor;

static void
output_tensor_free (gpointer data)
{
  OutputTensor *out = data;
  if (!out)
    return;
  ef_tensor_free (out->view);
  ef_tensor_free (out->full);
  g_free (out);
}

/* Input cache key: identifies a DMA-BUF by its kernel inode number and byte
 * offset.  Using the inode rather than the fd number makes the cache robust
 * to fd recycling: the same physical buffer always has the same inode even if
 * GStreamer closes and re-exports it with a different fd number. */
typedef struct {
  ino_t inode;
  gsize offset;
} InputCacheKey;

static guint
input_cache_key_hash (gconstpointer p)
{
  const InputCacheKey *k = p;
  /* Mix inode and offset into a single 32-bit hash.  Pool sizes are small
   * (4–16 entries), so collision probability is negligible. */
  return (guint) (k->inode ^ (k->inode >> 32) ^ (k->offset << 8));
}

static gboolean
input_cache_key_equal (gconstpointer a, gconstpointer b)
{
  const InputCacheKey *ka = a, *kb = b;
  return ka->inode == kb->inode && ka->offset == kb->offset;
}

static void
init_caches (EdgefirstCameraAdaptor *self)
{
  self->input_cache = g_hash_table_new_full (
      input_cache_key_hash, input_cache_key_equal,
      g_free, tensor_cache_value_free);
  self->output_cache = g_hash_table_new_full (
      g_direct_hash, g_direct_equal, NULL, output_tensor_free);
}

static void
clear_caches (EdgefirstCameraAdaptor *self)
{
  if (self->input_cache)
    g_hash_table_remove_all (self->input_cache);
  if (self->output_cache)
    g_hash_table_remove_all (self->output_cache);
  g_clear_pointer (&self->hal_output, output_tensor_free);
  g_clear_pointer (&self->stage_view, ef_tensor_free);
  g_clear_pointer (&self->stage, ef_tensor_free);
}

static void
destroy_caches (EdgefirstCameraAdaptor *self)
{
  g_clear_pointer (&self->input_cache, g_hash_table_destroy);
  g_clear_pointer (&self->output_cache, g_hash_table_destroy);
  g_clear_pointer (&self->hal_output, output_tensor_free);
  g_clear_pointer (&self->stage_view, ef_tensor_free);
  g_clear_pointer (&self->stage, ef_tensor_free);
}

typedef struct {
  guint x;
  guint y;
  guint w;
  guint h;
} LetterboxRect;

/* The centred placement HAL's letterbox uses (edgefirst-image
 * `letterbox_rect`), so a matching request can use HAL's own letterbox. */
static LetterboxRect
hal_letterbox_rect (guint sw, guint sh, guint dw, guint dh)
{
  guint new_w = dw;
  guint new_h = dh;
  if (sw > 0 && sh > 0) {
    gdouble src_aspect = (gdouble) sw / sh;
    gdouble dst_aspect = (gdouble) dw / dh;
    if (src_aspect > dst_aspect)
      new_h = MAX ((guint) (dw / src_aspect + 0.5), 1u);
    else
      new_w = MAX ((guint) (dh * src_aspect + 0.5), 1u);
  }
  LetterboxRect r = {
    .x = (dw - MIN (new_w, dw)) / 2,
    .y = (dh - MIN (new_h, dh)) / 2,
    .w = new_w,
    .h = new_h,
  };
  return r;
}

static void
compute_letterbox (EdgefirstCameraAdaptor *self, guint src_w, guint src_h)
{
  guint dst_w = self->out_width;
  guint dst_h = self->out_height;

  if (!self->letterbox || src_w == 0 || src_h == 0) {
    self->lb_scale = 1.0f;
    self->lb_top = self->lb_bottom = self->lb_left = self->lb_right = 0;
    self->lb_mode = LETTERBOX_NONE;
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

  self->dst_x = x;
  self->dst_y = y;
  self->dst_w = w;
  self->dst_h = h;

  guint8 r = (self->fill_color >> 24) & 0xFF;
  guint8 g = (self->fill_color >> 16) & 0xFF;
  guint8 b = (self->fill_color >>  8) & 0xFF;
  guint8 a = (self->fill_color      ) & 0xFF;
  self->fill_rgba[0] = r;
  self->fill_rgba[1] = g;
  self->fill_rgba[2] = b;
  self->fill_rgba[3] = a;

  LetterboxRect hal = hal_letterbox_rect (src_w, src_h, dst_w, dst_h);
  if (x == 0 && y == 0 && w == dst_w && h == dst_h) {
    self->lb_mode = LETTERBOX_NONE;
  } else if (x == hal.x && y == hal.y && w == hal.w && h == hal.h) {
    self->lb_mode = LETTERBOX_NATIVE;
    memset (&self->native_crop, 0, sizeof (self->native_crop));
    self->native_crop.letterbox = 1;
    memcpy (self->native_crop.pad, self->fill_rgba, 4);
  } else if (self->target_format->layout == EDGEFIRST_HAL_LAYOUT_PACKED) {
    self->lb_mode = LETTERBOX_VIEW;
  } else {
    self->lb_mode = LETTERBOX_STAGED;
  }

  GST_DEBUG_OBJECT (self, "letterbox: %ux%u → %ux%u in %ux%u "
      "(scale %.4f, T=%d B=%d L=%d R=%d, fill #%02x%02x%02x)",
      src_w, src_h, w, h, dst_w, dst_h,
      self->lb_scale, self->lb_top, self->lb_bottom,
      self->lb_left, self->lb_right, r, g, b);
}

/**
 * Lookup or import an input DMA-BUF as a HAL image tensor.
 * V4L2/ISP pools rotate ~4 fds; after one rotation every frame is a cache hit.
 * Returns a cached tensor (caller must NOT free).
 */
static ef_tensor *
lookup_or_import_input (EdgefirstCameraAdaptor *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width = GST_VIDEO_INFO_WIDTH (info);
  guint height = GST_VIDEO_INFO_HEIGHT (info);
  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (info);
  const EdgefirstHalFormat *pixel_fmt = edgefirst_hal_format_from_gst (vfmt);

  /* Note: for NV12 from v4l2h264dec, each DMA-BUF fd covers only its own
   * plane. The Y fd buffer size includes the aligned height (e.g. 1088 rows).
   * We pass the nominal height here; the HAL derives the actual allocation
   * size from fstat on the fd. */

  if (!pixel_fmt) {
    GST_ERROR_OBJECT (self, "unsupported input format %s",
        gst_video_format_to_string (vfmt));
    return NULL;
  }

  guint n_mem = gst_buffer_n_memory (inbuf);
  if (n_mem < 1) {
    GST_ERROR_OBJECT (self, "input buffer has no memory blocks");
    return NULL;
  }

  GstMemory *mem0 = gst_buffer_peek_memory (inbuf, 0);
  if (!gst_is_dmabuf_memory (mem0))
    return NULL;  /* not DMA-BUF — caller falls back to memcpy path */

  int fd = gst_dmabuf_memory_get_fd (mem0);
  gsize offset = 0;
  gst_memory_get_sizes (mem0, &offset, NULL);

  /* Use the kernel dma_buf inode as the stable buffer identity.  fd numbers
   * are recycled by GStreamer buffer pools and may differ frame-to-frame even
   * for the same underlying physical buffer, which would cause spurious cache
   * misses and EGL re-imports (~5 ms penalty each on i.MX 95 Mali-G310). */
  struct stat st;
  if (fstat (fd, &st) != 0) {
    GST_ERROR_OBJECT (self, "fstat failed for input fd=%d: %s",
        fd, g_strerror (errno));
    return NULL;
  }
  InputCacheKey lookup_key = { .inode = st.st_ino, .offset = offset };

  /* Cache lookup — no allocation needed for lookup, only for insert */
  ef_tensor *cached = g_hash_table_lookup (self->input_cache, &lookup_key);
  if (cached) {
    GST_LOG_OBJECT (self, "input cache hit fd=%d inode=%" G_GUINT64_FORMAT
        " offset=%" G_GSIZE_FORMAT, fd, (guint64) st.st_ino, offset);
    return cached;
  }

  /* Cache miss — import the planes */
  GST_DEBUG_OBJECT (self, "input cache miss fd=%d inode=%" G_GUINT64_FORMAT
      " offset=%" G_GSIZE_FORMAT " importing %ux%u %s",
      fd, (guint64) st.st_ino, offset, width, height,
      gst_video_format_to_string (vfmt));

  EdgefirstHalPlane image = { .fd = fd, .offset = offset };

  /* Set stride if padded (e.g. VPU buffers) */
  GstVideoMeta *vmeta = gst_buffer_get_video_meta (inbuf);
  gint stride = vmeta ? (gint) vmeta->stride[0]
                      : GST_VIDEO_INFO_PLANE_STRIDE (info, 0);
  image.stride = (gsize) MAX (stride, 0);

  EdgefirstHalPlane chroma_plane = { .fd = -1 };
  EdgefirstHalPlane *chroma = NULL;
  if (pixel_fmt->layout == EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR) {
    /* Diagnostic: dump all available NV12 plane info */
    GST_INFO_OBJECT (self, "NV12 import: n_mem=%u vmeta=%p", n_mem, (void *) vmeta);
    if (vmeta) {
      GST_INFO_OBJECT (self, "  vmeta: n_planes=%u, offset[0]=%" G_GSIZE_FORMAT
          " offset[1]=%" G_GSIZE_FORMAT " stride[0]=%d stride[1]=%d",
          vmeta->n_planes,
          vmeta->n_planes > 0 ? vmeta->offset[0] : 0,
          vmeta->n_planes > 1 ? vmeta->offset[1] : 0,
          vmeta->n_planes > 0 ? (int) vmeta->stride[0] : 0,
          vmeta->n_planes > 1 ? (int) vmeta->stride[1] : 0);
    }
    GST_INFO_OBJECT (self, "  GstVideoInfo: offset[0]=%" G_GSIZE_FORMAT
        " offset[1]=%" G_GSIZE_FORMAT " stride[0]=%d stride[1]=%d",
        GST_VIDEO_INFO_PLANE_OFFSET (info, 0),
        GST_VIDEO_INFO_PLANE_OFFSET (info, 1),
        GST_VIDEO_INFO_PLANE_STRIDE (info, 0),
        GST_VIDEO_INFO_PLANE_STRIDE (info, 1));

    if (n_mem >= 2) {
      /* Two DMA-BUF memory blocks → always use each fd for its own plane.
       * vmeta->offset[1] is the logical frame offset (Y_stride × Y_height),
       * NOT an offset within the UV fd's buffer.  Each GstMemory's dma_buf
       * covers only its own plane's data.  This handles both libcamerasrc
       * (separate allocations) and v4l2h264dec (separate dma_bufs for same
       * physical memory) correctly. */
      GstMemory *mem1 = gst_buffer_peek_memory (inbuf, 1);
      if (gst_is_dmabuf_memory (mem1)) {
        int uv_fd = gst_dmabuf_memory_get_fd (mem1);
        gsize mem1_offset = 0;
        gst_memory_get_sizes (mem1, &mem1_offset, NULL);
        gint uv_stride = vmeta && vmeta->n_planes >= 2
            ? (gint) vmeta->stride[1]
            : GST_VIDEO_INFO_PLANE_STRIDE (info, 1);

        chroma_plane.fd = uv_fd;
        chroma_plane.offset = mem1_offset;
        chroma_plane.stride = (gsize) MAX (uv_stride, 0);
        chroma = &chroma_plane;
        GST_INFO_OBJECT (self, "  NV12 two-fd planes: y_fd=%d uv_fd=%d "
            "mem1_offset=%" G_GSIZE_FORMAT " uv_stride=%d",
            fd, uv_fd, mem1_offset, uv_stride);
      }
    } else {
      /* Single-memory NV12: Y and UV in one contiguous DMA-BUF
       * (e.g. v4l2h264dec). UV plane starts at offset from GstVideoMeta
       * or GstVideoInfo. */
      gsize uv_offset = 0;
      gint uv_stride = 0;
      if (vmeta && vmeta->n_planes >= 2 && vmeta->offset[1] > 0) {
        uv_offset = vmeta->offset[1];
        uv_stride = (gint) vmeta->stride[1];
      } else if (GST_VIDEO_INFO_PLANE_OFFSET (info, 1) > 0) {
        uv_offset = (gsize) GST_VIDEO_INFO_PLANE_OFFSET (info, 1);
        uv_stride = GST_VIDEO_INFO_PLANE_STRIDE (info, 1);
      } else {
        /* Last resort: compute from stride * height */
        uv_offset = (gsize) stride * height;
        uv_stride = stride;
        GST_WARNING_OBJECT (self, "  NV12 single-mem: no offset metadata, "
            "computing uv_offset=%zu from stride(%d)*height(%u)",
            (size_t) uv_offset, stride, height);
      }

      if (uv_offset > 0) {
        chroma_plane.fd = fd;
        chroma_plane.offset = uv_offset;
        chroma_plane.stride = (gsize) MAX (uv_stride, 0);
        chroma = &chroma_plane;
        GST_INFO_OBJECT (self, "  NV12 single-mem: uv_offset=%" G_GSIZE_FORMAT
            " uv_stride=%d", uv_offset, uv_stride);
      } else {
        GST_ERROR_OBJECT (self, "  NV12 single-mem: cannot determine UV offset!");
      }
    }

    if (!chroma)
      GST_WARNING_OBJECT (self, "  NV12 import WITHOUT chroma plane — will produce bad output");
  }

  ef_tensor *tensor = edgefirst_hal_import_image (GST_OBJECT (self),
      &image, chroma, width, height, pixel_fmt, EF_DTYPE_U8);
  if (!tensor) {
    GST_ERROR_OBJECT (self, "HAL image import failed for fd=%d", fd);
    return NULL;
  }

  InputCacheKey *heap_key = g_new (InputCacheKey, 1);
  *heap_key = lookup_key;
  g_hash_table_insert (self->input_cache, heap_key, tensor);
  return tensor;
}

/**
 * System-memory fallback: allocate a HAL tensor and memcpy the frame data.
 * Returns a new tensor the caller MUST free with ef_tensor_free().
 */
static ef_tensor *
create_input_from_sysmem (EdgefirstCameraAdaptor *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width = GST_VIDEO_INFO_WIDTH (info);
  guint height = GST_VIDEO_INFO_HEIGHT (info);
  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (info);
  const EdgefirstHalFormat *pixel_fmt = edgefirst_hal_format_from_gst (vfmt);

  static gboolean warned = FALSE;
  if (G_UNLIKELY (!warned)) {
    warned = TRUE;
    GST_WARNING_OBJECT (self, "input is not DMA-BUF; using memcpy fallback "
        "(zero-copy disabled)");
  }

  if (!pixel_fmt) {
    GST_ERROR_OBJECT (self, "unsupported input format %s",
        gst_video_format_to_string (vfmt));
    return NULL;
  }

  ef_tensor *tensor = edgefirst_hal_create_image (self->processor,
      width, height, pixel_fmt, EF_DTYPE_U8);
  if (!tensor) {
    GST_ERROR_OBJECT (self, "HAL image allocation failed: %s",
        ef_tensor_last_error_message ());
    return NULL;
  }

  ef_tensor_view view;
  if (ef_tensor_map (tensor, EF_CPU_ACCESS_WRITE, &view) != 0) {
    ef_tensor_free (tensor);
    return NULL;
  }
  uint8_t *dst = view.ptr;

  GstVideoFrame frame;
  if (!gst_video_frame_map (&frame, info, inbuf, GST_MAP_READ)) {
    ef_tensor_unmap (tensor);
    ef_tensor_free (tensor);
    return NULL;
  }

  /* Row-by-row copy handles stride padding on either side */
  gsize row_bytes = edgefirst_hal_format_row_bytes (pixel_fmt, width);
  gsize dst_stride = edgefirst_hal_row_stride (tensor);
  if (dst_stride < row_bytes)
    dst_stride = row_bytes;

  if (pixel_fmt->layout == EDGEFIRST_HAL_LAYOUT_SEMI_PLANAR) {
    /* Y plane */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * dst_stride, y_data + y * y_stride, width);
    /* UV plane */
    const guint8 *uv_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * dst_stride;
    for (guint y = 0; y < height / 2; y++)
      memcpy (uv_dst + y * dst_stride, uv_data + y * uv_stride, width);
  } else {
    const guint8 *src_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    if ((gsize) stride == row_bytes && dst_stride == row_bytes) {
      memcpy (dst, src_data, row_bytes * height);
    } else {
      for (guint y = 0; y < height; y++)
        memcpy (dst + y * dst_stride, src_data + y * stride, row_bytes);
    }
  }

  gst_video_frame_unmap (&frame);
  ef_tensor_unmap (tensor);
  return tensor;
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
  self->compute = EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO;
  self->letterbox = FALSE;
  self->fill_color = 0x808080FF;  /* grey, full alpha */
  self->lb_scale = 0.0f;
  self->lb_top = self->lb_bottom = self->lb_left = self->lb_right = 0;
  self->lb_top_override = self->lb_bottom_override = FALSE;
  self->lb_left_override = self->lb_right_override = FALSE;
  self->in_info_valid = FALSE;

  init_caches (self);

  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (self), FALSE);
}

static void
edgefirst_camera_adaptor_finalize (GObject *object)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (object);

  destroy_caches (self);
  if (self->downstream_pool) {
    gst_buffer_pool_set_active (self->downstream_pool, FALSE);
    gst_clear_object (&self->downstream_pool);
  }
  g_clear_pointer (&self->processor, ef_image_processor_free);
  g_free (self->model_mean);
  g_free (self->model_std);

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
    case PROP_COMPUTE:
      self->compute = g_value_get_enum (value);
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
    case PROP_MODEL_MEAN:
      g_free (self->model_mean);
      self->model_mean = g_value_dup_string (value);
      if (self->model_mean)
        GST_WARNING_OBJECT (object, "model-mean is not yet implemented; "
            "normalization will be supported in a future HAL version");
      break;
    case PROP_MODEL_STD:
      g_free (self->model_std);
      self->model_std = g_value_dup_string (value);
      if (self->model_std)
        GST_WARNING_OBJECT (object, "model-std is not yet implemented; "
            "normalization will be supported in a future HAL version");
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
    case PROP_COMPUTE:
      g_value_set_enum (value, self->compute);
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
    case PROP_MODEL_MEAN:
      g_value_set_string (value, self->model_mean);
      break;
    case PROP_MODEL_STD:
      g_value_set_string (value, self->model_std);
      break;
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

  /* Map GStreamer compute property to HAL backend enum */
  static const uint32_t compute_map[] = {
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO]   = EDGEFIRST_HAL_BACKEND_AUTO,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_OPENGL] = EDGEFIRST_HAL_BACKEND_OPENGL,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_G2D]    = EDGEFIRST_HAL_BACKEND_G2D,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_CPU]    = EDGEFIRST_HAL_BACKEND_CPU,
  };
  static const char *compute_names[] = {
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO]   = "auto",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_OPENGL] = "opengl",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_G2D]    = "g2d",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_CPU]    = "cpu",
  };
  const char *backend_str = compute_names[self->compute];

  /* Route HAL internal logs through GST_DEBUG ("edgefirst-hal" category) */
  ef_log_init_callback (hal_log_to_gst, NULL,
      gst_level_to_hal (gst_debug_category_get_threshold (edgefirst_hal_debug)));

  GST_INFO_OBJECT (self, "requesting HAL backend: %s", backend_str);
  self->processor = ef_image_processor_new_with_backend (
      compute_map[self->compute]);

  if (!self->processor) {
    GST_ELEMENT_ERROR (self, LIBRARY, INIT,
        ("Failed to create HAL image processor"),
        ("compute=%s", backend_str));
    return FALSE;
  }

  GST_INFO_OBJECT (self, "HAL image processor created (compute=%s)",
      backend_str);
  return TRUE;
}

static gboolean
edgefirst_camera_adaptor_stop (GstBaseTransform *trans)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  clear_caches (self);
  if (self->downstream_pool) {
    gst_buffer_pool_set_active (self->downstream_pool, FALSE);
    gst_clear_object (&self->downstream_pool);
  }
  g_clear_pointer (&self->processor, ef_image_processor_free);
  self->in_info_valid = FALSE;
  self->input_is_drm = FALSE;

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
      guint channels;
      switch (self->colorspace) {
        case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY: channels = 1; break;
        case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGBA: channels = 4; break;
        default:                                        channels = 3; break;
      }
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
    /* Src → Sink: accept the video formats the HAL format table maps.
     * Build DMA_DRM caps with explicit drm-format list so upstream
     * (e.g. v4l2h264dec) only selects formats HAL can process. */
    GValue drm_list = G_VALUE_INIT;
    GValue fmt_list = G_VALUE_INIT;
    g_value_init (&drm_list, GST_TYPE_LIST);
    g_value_init (&fmt_list, GST_TYPE_LIST);
    for (const EdgefirstHalFormat *f = edgefirst_hal_formats (); f->wire; f++) {
      if (f->gst == GST_VIDEO_FORMAT_UNKNOWN)
        continue;

      GValue v = G_VALUE_INIT;
      g_value_init (&v, G_TYPE_STRING);
      g_value_set_static_string (&v, gst_video_format_to_string (f->gst));
      gst_value_list_append_value (&fmt_list, &v);
      g_value_unset (&v);

      guint32 drm_fourcc = gst_video_dma_drm_fourcc_from_format (f->gst);
      if (drm_fourcc != 0) {
        gchar *drm_str =
            gst_video_dma_drm_fourcc_to_string (drm_fourcc, 0);
        if (drm_str) {
          g_value_init (&v, G_TYPE_STRING);
          g_value_take_string (&v, drm_str);
          gst_value_list_append_value (&drm_list, &v);
          g_value_unset (&v);
        }
      }
    }

    /* DMA_DRM caps with restricted drm-format (highest priority) */
    GstCaps *drm_caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "DMA_DRM",
        NULL);
    gst_caps_set_features (drm_caps, 0,
        gst_caps_features_new ("memory:DMABuf", NULL));
    GstStructure *drm_s = gst_caps_get_structure (drm_caps, 0);
    gst_structure_set_value (drm_s, "drm-format", &drm_list);
    g_value_unset (&drm_list);

    /* Non-DMA_DRM DMA-BUF caps (preferred over system memory) */
    GstCaps *raw_caps = gst_caps_new_empty_simple ("video/x-raw");
    GstStructure *raw_s = gst_caps_get_structure (raw_caps, 0);
    gst_structure_set_value (raw_s, "format", &fmt_list);
    g_value_unset (&fmt_list);
    gst_structure_set (raw_s,
        "width", GST_TYPE_INT_RANGE, 1, G_MAXINT,
        "height", GST_TYPE_INT_RANGE, 1, G_MAXINT,
        NULL);

    /* System memory caps: sources like libcamerasrc declare video/x-raw
     * without memory:DMABuf even though their allocator produces
     * DMABuf-backed memory (linear/mappable DMABuf omits the feature per
     * GStreamer convention).  Accept video/x-raw so caps negotiation
     * succeeds; actual memory type is verified at runtime. */
    GstCaps *sys_caps = gst_caps_copy (raw_caps);
    gst_caps_set_features (raw_caps, 0,
        gst_caps_features_new ("memory:DMABuf", NULL));

    result = drm_caps;
    gst_caps_append (result, raw_caps);
    gst_caps_append (result, sys_caps);
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

  /* Parse input video info — try DMA_DRM first, then standard video caps */
  if (gst_video_is_dma_drm_caps (incaps)) {
    GstVideoInfoDmaDrm drm_info;
    gst_video_info_dma_drm_init (&drm_info);
    if (!gst_video_info_dma_drm_from_caps (&drm_info, incaps)) {
      GST_ERROR_OBJECT (self, "failed to parse DMA_DRM caps %" GST_PTR_FORMAT,
          incaps);
      return FALSE;
    }
    /* Convert DRM info to standard GstVideoInfo (strides/offsets for the
     * resolved pixel format).  Falls back to the vinfo inside drm_info
     * if the modifier is non-linear. */
    if (!gst_video_info_dma_drm_to_video_info (&drm_info, &self->in_info)) {
      GstVideoFormat fmt =
          gst_video_dma_drm_fourcc_to_format (drm_info.drm_fourcc);
      if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
        GST_ERROR_OBJECT (self, "unsupported DRM fourcc 0x%08x",
            drm_info.drm_fourcc);
        return FALSE;
      }
      gst_video_info_set_format (&self->in_info, fmt,
          GST_VIDEO_INFO_WIDTH (&drm_info.vinfo),
          GST_VIDEO_INFO_HEIGHT (&drm_info.vinfo));
    }
    self->input_is_drm = TRUE;
    GST_INFO_OBJECT (self, "DMA_DRM input: drm_fourcc=0x%08x modifier=0x%016"
        G_GINT64_MODIFIER "x → %s", drm_info.drm_fourcc, drm_info.drm_modifier,
        gst_video_format_to_string (GST_VIDEO_INFO_FORMAT (&self->in_info)));
  } else if (!gst_video_info_from_caps (&self->in_info, incaps)) {
    GST_ERROR_OBJECT (self, "failed to parse input caps %" GST_PTR_FORMAT,
        incaps);
    return FALSE;
  } else {
    self->input_is_drm = FALSE;
  }
  self->in_info_valid = TRUE;

  guint src_w = GST_VIDEO_INFO_WIDTH (&self->in_info);
  guint src_h = GST_VIDEO_INFO_HEIGHT (&self->in_info);
  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (&self->in_info);

  if (!edgefirst_hal_format_from_gst (vfmt)) {
    GST_ERROR_OBJECT (self, "unsupported input format %s",
        gst_video_format_to_string (vfmt));
    return FALSE;
  }

  /* Resolve output dimensions */
  self->out_width = self->model_width > 0 ? self->model_width : src_w;
  self->out_height = self->model_height > 0 ? self->model_height : src_h;
  switch (self->colorspace) {
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY: self->out_channels = 1; break;
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGBA: self->out_channels = 4; break;
    default:                                        self->out_channels = 3; break;
  }

  /* Resolve target HAL format/dtype from properties */
  resolve_target_format (self);

  /* Invalidate caches — format/resolution may have changed */
  clear_caches (self);

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
      if (c == 0) {
        switch (self->colorspace) {
          case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY: c = 1; break;
          case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGBA: c = 4; break;
          default:                                        c = 3; break;
        }
      }
    }

    if (w == 0 || h == 0)
      return FALSE;

    *othersize = (gsize) w * h * c * dtype_byte_size (self->dtype);
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
    GstQuery *query)
{
  /* Signal that we accept GstVideoMeta — this enables upstream to provide
   * buffers with non-default strides (e.g. VPU height-aligned buffers).
   * Don't propose a DMA-BUF allocator; elements that natively produce
   * DMA-BUF (v4l2 decoders, ISP sources) will provide it anyway. */
  gst_query_add_allocation_meta (query, GST_VIDEO_META_API_TYPE, NULL);
  return TRUE;
}

static gboolean
edgefirst_camera_adaptor_decide_allocation (GstBaseTransform *trans,
    GstQuery *query)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  if (self->out_width == 0 || self->out_height == 0) {
    GST_ERROR_OBJECT (self, "output dimensions not configured");
    return FALSE;
  }

  /* Clean up previous state */
  clear_caches (self);
  if (self->downstream_pool) {
    gst_buffer_pool_set_active (self->downstream_pool, FALSE);
    gst_clear_object (&self->downstream_pool);
  }

  /* Check if downstream accepts DMA-BUF */
  self->downstream_dmabuf = FALSE;
  for (guint i = 0; i < gst_query_get_n_allocation_params (query); i++) {
    GstAllocator *alloc = NULL;
    gst_query_parse_nth_allocation_param (query, i, &alloc, NULL);
    if (alloc && GST_IS_DMABUF_ALLOCATOR (alloc))
      self->downstream_dmabuf = TRUE;
    gst_clear_object (&alloc);
  }

  /* Detect downstream DMA-BUF pool (e.g. Ara-2 pre-registered buffers) */
  guint n_pools = gst_query_get_n_allocation_pools (query);
  if (n_pools > 0) {
    GstBufferPool *pool = NULL;
    guint pool_size = 0, pool_min = 0, pool_max = 0;
    gst_query_parse_nth_allocation_pool (query, 0, &pool, &pool_size,
        &pool_min, &pool_max);
    if (pool) {
      if (!gst_buffer_pool_is_active (pool))
        gst_buffer_pool_set_active (pool, TRUE);
      self->downstream_pool = pool;
      GST_INFO_OBJECT (self, "using downstream DMA-BUF pool "
          "(size=%u min=%u max=%u)", pool_size, pool_min, pool_max);
    }
  }

  GST_INFO_OBJECT (self, "output: %ux%ux%u %s (pool=%s)",
      self->out_width, self->out_height, self->out_channels,
      dtype_to_nnstreamer_string (self->dtype),
      self->downstream_pool ? "DMA-BUF" : "HAL-owned");

  /* Don't chain to parent — other/tensors caps don't have a known
   * buffer size in the allocation query. */
  return TRUE;
}

/* ── Output tensor lookup ────────────────────────────────────────── */

/**
 * Fill an output tensor with the letterbox colour and take the view the
 * image is converted into. Converts only ever write inside the view, so the
 * padding persists for the life of the tensor.
 */
static gboolean
prepare_letterbox_output (EdgefirstCameraAdaptor *self, OutputTensor *out)
{
  if (self->lb_mode != LETTERBOX_VIEW)
    return TRUE;

  /* The fill colour goes through the same HAL conversion as the image, so
   * the padding lands in the output's format, layout and dtype. */
  const EdgefirstHalFormat *rgba =
      edgefirst_hal_format_from_wire (EDGEFIRST_HAL_FORMAT_RGBA);
  const guint fill_size = 16;
  ef_tensor *fill = edgefirst_hal_create_image (self->processor,
      fill_size, fill_size, rgba, EF_DTYPE_U8);
  if (!fill) {
    GST_ERROR_OBJECT (self, "letterbox fill allocation failed: %s",
        ef_tensor_last_error_message ());
    return FALSE;
  }
  ef_tensor_view view;
  if (ef_tensor_map (fill, EF_CPU_ACCESS_WRITE, &view) != 0) {
    ef_tensor_free (fill);
    return FALSE;
  }
  gsize stride = edgefirst_hal_row_stride (fill);
  if (stride < fill_size * 4)
    stride = fill_size * 4;
  for (guint y = 0; y < fill_size; y++)
    for (guint x = 0; x < fill_size; x++)
      memcpy (view.ptr + y * stride + x * 4, self->fill_rgba, 4);
  ef_tensor_unmap (fill);

  int ret = ef_image_processor_convert (self->processor, fill, out->full,
      EDGEFIRST_HAL_ROTATION_NONE, EDGEFIRST_HAL_FLIP_NONE, NULL);
  ef_tensor_free (fill);
  if (ret != 0) {
    GST_ERROR_OBJECT (self, "letterbox fill failed (%d): %s", ret,
        ef_tensor_last_error_message ());
    return FALSE;
  }

  out->view = ef_tensor_view_region (out->full, self->dst_x, self->dst_y,
      self->dst_w, self->dst_h);
  if (!out->view) {
    GST_ERROR_OBJECT (self, "letterbox view %ux%u+%u+%u refused: %s",
        self->dst_w, self->dst_h, self->dst_x, self->dst_y,
        ef_tensor_last_error_message ());
    return FALSE;
  }
  return TRUE;
}

/**
 * Create the packed U8 image a planar output is letterboxed through: the
 * fill colour everywhere, and a view at the placement for the convert.
 */
static gboolean
ensure_letterbox_stage (EdgefirstCameraAdaptor *self)
{
  if (self->stage)
    return TRUE;

  const EdgefirstHalFormat *fmt = edgefirst_hal_format_from_wire (
      self->target_format->channels == 4 ? EDGEFIRST_HAL_FORMAT_RGBA
                                         : EDGEFIRST_HAL_FORMAT_RGB);
  ef_tensor *stage = edgefirst_hal_create_image (self->processor,
      self->out_width, self->out_height, fmt, EF_DTYPE_U8);
  if (!stage) {
    GST_ERROR_OBJECT (self, "letterbox stage allocation failed: %s",
        ef_tensor_last_error_message ());
    return FALSE;
  }

  ef_tensor_view view;
  if (ef_tensor_map (stage, EF_CPU_ACCESS_WRITE, &view) != 0) {
    ef_tensor_free (stage);
    return FALSE;
  }
  gsize row_bytes = edgefirst_hal_format_row_bytes (fmt, self->out_width);
  gsize stride = MAX (edgefirst_hal_row_stride (stage), row_bytes);
  for (guint y = 0; y < self->out_height; y++)
    for (guint x = 0; x < self->out_width; x++)
      memcpy (view.ptr + y * stride + x * fmt->channels, self->fill_rgba,
          fmt->channels);
  ef_tensor_unmap (stage);

  ef_tensor *stage_view = ef_tensor_view_region (stage, self->dst_x,
      self->dst_y, self->dst_w, self->dst_h);
  if (!stage_view) {
    GST_ERROR_OBJECT (self, "letterbox view %ux%u+%u+%u refused: %s",
        self->dst_w, self->dst_h, self->dst_x, self->dst_y,
        ef_tensor_last_error_message ());
    ef_tensor_free (stage);
    return FALSE;
  }
  self->stage = stage;
  self->stage_view = stage_view;
  return TRUE;
}

/**
 * Get or create the output HAL tensor for the current frame.
 * When a downstream pool is available, the output buffer's DMA-BUF fd
 * is imported (cached by fd).  Otherwise a HAL-owned image is allocated
 * once and reused for all frames.
 */
static OutputTensor *
get_output_tensor (EdgefirstCameraAdaptor *self, GstBuffer *outbuf)
{
  if (self->downstream_pool) {
    /* Import the downstream pool buffer's DMA-BUF fd (cached) */
    GstMemory *out_mem = gst_buffer_peek_memory (outbuf, 0);
    if (!gst_is_dmabuf_memory (out_mem)) {
      GST_ERROR_OBJECT (self, "downstream pool buffer is not DMA-BUF");
      return NULL;
    }
    int fd = gst_dmabuf_memory_get_fd (out_mem);
    gpointer key = GINT_TO_POINTER (fd);

    OutputTensor *cached = g_hash_table_lookup (self->output_cache, key);
    if (cached) {
      GST_LOG_OBJECT (self, "output cache hit fd=%d", fd);
      return cached;
    }

    gsize mem_offset = 0;
    gst_memory_get_sizes (out_mem, &mem_offset, NULL);
    GST_DEBUG_OBJECT (self, "output cache miss fd=%d offset=%" G_GSIZE_FORMAT ", importing",
        fd, mem_offset);
    EdgefirstHalPlane plane = { .fd = fd, .offset = mem_offset };
    OutputTensor *out = g_new0 (OutputTensor, 1);
    out->full = edgefirst_hal_import_image (GST_OBJECT (self), &plane, NULL,
        self->out_width, self->out_height,
        self->target_format, self->target_dtype);
    if (!out->full) {
      GST_ERROR_OBJECT (self, "HAL image import failed for output fd=%d", fd);
      output_tensor_free (out);
      return NULL;
    }
    if (!prepare_letterbox_output (self, out)) {
      output_tensor_free (out);
      return NULL;
    }

    g_hash_table_insert (self->output_cache, key, out);
    return out;
  }

  /* No downstream pool — use HAL-owned output (allocated once) */
  if (!self->hal_output) {
    OutputTensor *out = g_new0 (OutputTensor, 1);
    out->full = edgefirst_hal_create_image (self->processor,
        self->out_width, self->out_height,
        self->target_format, self->target_dtype);
    if (!out->full) {
      GST_ERROR_OBJECT (self, "HAL image allocation failed: %s",
          ef_tensor_last_error_message ());
      output_tensor_free (out);
      return NULL;
    }
    if (!prepare_letterbox_output (self, out)) {
      output_tensor_free (out);
      return NULL;
    }
    self->hal_output = out;
    GST_DEBUG_OBJECT (self, "created HAL-owned output %ux%u",
        self->out_width, self->out_height);
  }
  return self->hal_output;
}

/* ── prepare_output_buffer ───────────────────────────────────────── */

static GstFlowReturn
edgefirst_camera_adaptor_prepare_output_buffer (GstBaseTransform *trans,
    GstBuffer *inbuf G_GNUC_UNUSED, GstBuffer **outbuf)
{
  EdgefirstCameraAdaptor *self = EDGEFIRST_CAMERA_ADAPTOR (trans);

  if (self->downstream_pool) {
    /* Acquire from downstream's pre-registered DMA-BUF pool */
    GstFlowReturn ret = gst_buffer_pool_acquire_buffer (
        self->downstream_pool, outbuf, NULL);
    if (ret != GST_FLOW_OK) {
      GST_ERROR_OBJECT (self, "DMA-BUF pool acquire failed: %s",
          gst_flow_get_name (ret));
      return ret;
    }
    return GST_FLOW_OK;
  }

  /* No downstream pool — allocate a minimal buffer.
   * The actual data lives in self->hal_output (HAL-owned). */
  gsize out_size = self->out_width * self->out_height * self->out_channels;
  *outbuf = gst_buffer_new_allocate (NULL, out_size, NULL);
  if (!*outbuf) {
    GST_ERROR_OBJECT (self, "failed to allocate output buffer");
    return GST_FLOW_ERROR;
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

  /* Try DMA-BUF zero-copy import first; fall back to memcpy for system memory */
  gboolean src_owned = FALSE;
  ef_tensor *src = lookup_or_import_input (self, inbuf);
  if (!src) {
    src = create_input_from_sysmem (self, inbuf);
    src_owned = TRUE;
  }
  if (!src) {
    GST_ERROR_OBJECT (self, "failed to get input tensor");
    return GST_FLOW_ERROR;
  }

  OutputTensor *dst = get_output_tensor (self, outbuf);
  if (!dst) {
    GST_ERROR_OBJECT (self, "failed to get output tensor");
    if (src_owned) ef_tensor_free (src);
    return GST_FLOW_ERROR;
  }

  int ret;
  switch (self->lb_mode) {
    case LETTERBOX_NATIVE:
      ret = ef_image_processor_convert (self->processor, src, dst->full,
          EDGEFIRST_HAL_ROTATION_NONE, EDGEFIRST_HAL_FLIP_NONE,
          &self->native_crop);
      break;
    case LETTERBOX_VIEW:
      ret = ef_image_processor_convert (self->processor, src, dst->view,
          EDGEFIRST_HAL_ROTATION_NONE, EDGEFIRST_HAL_FLIP_NONE, NULL);
      break;
    case LETTERBOX_STAGED:
      if (!ensure_letterbox_stage (self)) {
        ret = -1;
        break;
      }
      ret = ef_image_processor_convert (self->processor, src,
          self->stage_view, EDGEFIRST_HAL_ROTATION_NONE,
          EDGEFIRST_HAL_FLIP_NONE, NULL);
      if (ret == 0)
        ret = ef_image_processor_convert (self->processor, self->stage,
            dst->full, EDGEFIRST_HAL_ROTATION_NONE, EDGEFIRST_HAL_FLIP_NONE,
            NULL);
      break;
    case LETTERBOX_NONE:
    default:
      ret = ef_image_processor_convert (self->processor, src, dst->full,
          EDGEFIRST_HAL_ROTATION_NONE, EDGEFIRST_HAL_FLIP_NONE, NULL);
      break;
  }

  if (src_owned) ef_tensor_free (src);

  if (ret != 0) {
    GST_ERROR_OBJECT (self, "HAL convert failed (%d): %s", ret,
        ef_tensor_last_error_message ());
    return GST_FLOW_ERROR;
  }

  /* When using HAL-owned output (no downstream DMA-BUF pool), the convert
   * wrote into self->hal_output, not outbuf. Copy the result out. */
  if (!self->downstream_pool && self->hal_output) {
    ef_tensor_view view;
    if (ef_tensor_map (self->hal_output->full, EF_CPU_ACCESS_READ, &view) == 0) {
      GstMapInfo map;
      if (gst_buffer_map (outbuf, &map, GST_MAP_WRITE)) {
        gsize copy_size = MIN (map.size,
            (gsize) self->out_width * self->out_height * self->out_channels
            * dtype_byte_size (self->dtype));
        memcpy (map.data, view.ptr, MIN (copy_size, view.len));
        gst_buffer_unmap (outbuf, &map);
      }
      ef_tensor_unmap (self->hal_output->full);
    }
  }

  gst_buffer_copy_into (outbuf, inbuf,
      GST_BUFFER_COPY_TIMESTAMPS | GST_BUFFER_COPY_FLAGS, 0, -1);

  guint64 t_done = _get_time_ns ();
  GST_LOG_OBJECT (self, "convert %.3fms", (t_done - t0) / 1e6);

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

  g_object_class_install_property (gobject_class, PROP_COMPUTE,
      g_param_spec_enum ("compute", "Compute Backend",
          "HAL image processing backend. OpenGL is faster for resize/letterbox "
          "on most platforms. Auto uses the HAL default (G2D > OpenGL > CPU).",
          edgefirst_camera_adaptor_compute_get_type (),
          EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO,
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

  g_object_class_install_property (gobject_class, PROP_MODEL_MEAN,
      g_param_spec_string ("model-mean", "Model Mean",
          "Per-channel mean for float normalization (comma-separated, e.g. "
          "\"0.485,0.456,0.406\")",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_MODEL_STD,
      g_param_spec_string ("model-std", "Model Std",
          "Per-channel standard deviation for float normalization "
          "(comma-separated, e.g. \"0.229,0.224,0.225\")",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

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
  GST_DEBUG_CATEGORY_INIT (edgefirst_hal_debug,
      "edgefirst-hal", 0, "EdgeFirst HAL (routed from libedgefirst_hal)");
}
