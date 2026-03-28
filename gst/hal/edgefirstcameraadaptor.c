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
#include <edgefirst/hal.h>
#include <time.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_camera_adaptor_debug);
GST_DEBUG_CATEGORY_STATIC (edgefirst_hal_debug);

/* ── HAL log → GST_DEBUG bridge ──────────────────────────────────── */

static void
hal_log_to_gst (enum hal_log_level level, const char *target,
    const char *message, void *userdata G_GNUC_UNUSED)
{
  GstDebugLevel gst_level;
  switch (level) {
    case HAL_LOG_LEVEL_ERROR: gst_level = GST_LEVEL_ERROR; break;
    case HAL_LOG_LEVEL_WARN:  gst_level = GST_LEVEL_WARNING; break;
    case HAL_LOG_LEVEL_INFO:  gst_level = GST_LEVEL_INFO; break;
    case HAL_LOG_LEVEL_DEBUG: gst_level = GST_LEVEL_DEBUG; break;
    case HAL_LOG_LEVEL_TRACE: gst_level = GST_LEVEL_TRACE; break;
    default:                  gst_level = GST_LEVEL_LOG; break;
  }
  gst_debug_log (edgefirst_hal_debug, gst_level, target, "", 0, NULL,
      "%s", message);
}

static enum hal_log_level
gst_level_to_hal (GstDebugLevel gst_level)
{
  if (gst_level <= GST_LEVEL_ERROR)   return HAL_LOG_LEVEL_ERROR;
  if (gst_level <= GST_LEVEL_WARNING) return HAL_LOG_LEVEL_WARN;
  if (gst_level <= GST_LEVEL_INFO)    return HAL_LOG_LEVEL_INFO;
  if (gst_level <= GST_LEVEL_DEBUG)   return HAL_LOG_LEVEL_DEBUG;
  return HAL_LOG_LEVEL_TRACE;
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
  EdgefirstCameraAdaptorCompute compute;
  gboolean letterbox;
  guint32 fill_color;
  gfloat lb_scale;
  gint lb_top, lb_bottom, lb_left, lb_right;
  gboolean lb_top_override, lb_bottom_override;
  gboolean lb_left_override, lb_right_override;

  /* Runtime state */
  hal_image_processor *processor;
  GstVideoInfo in_info;
  gboolean in_info_valid;
  hal_crop crop;
  gboolean crop_valid;

  /* Output dimensions */
  guint out_width, out_height, out_channels;

  /* Tensor caches — import once per fd, reuse across frames */
  GHashTable *input_cache;   /* fd (int) -> hal_tensor* */
  GHashTable *output_cache;  /* fd (int) -> hal_tensor* */
  hal_tensor *hal_output;    /* HAL-owned output (when no downstream pool) */

  /* DMA-BUF state */
  gboolean downstream_dmabuf;
  GstBufferPool *downstream_pool;
  gboolean input_is_drm;

  /* Target HAL format/dtype (resolved from properties) */
  enum hal_pixel_format target_format;
  enum hal_dtype target_dtype;
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

static enum hal_pixel_format
gst_format_to_hal_pixel (GstVideoFormat fmt)
{
  switch (fmt) {
    case GST_VIDEO_FORMAT_NV12:  return HAL_PIXEL_FORMAT_NV12;
    case GST_VIDEO_FORMAT_YUY2:  return HAL_PIXEL_FORMAT_YUYV;
    case GST_VIDEO_FORMAT_RGB:   return HAL_PIXEL_FORMAT_RGB;
    case GST_VIDEO_FORMAT_RGBA:  return HAL_PIXEL_FORMAT_RGBA;
    case GST_VIDEO_FORMAT_GRAY8: return HAL_PIXEL_FORMAT_GREY;
    default:                     return (enum hal_pixel_format) -1;
  }
}

static const char *
dtype_to_nnstreamer_string (EdgefirstCameraAdaptorDtype dtype)
{
  switch (dtype) {
    case EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8: return "int8";
    default:                                   return "uint8";
  }
}

static void
resolve_target_format (EdgefirstCameraAdaptor *self)
{
  self->target_dtype = (self->dtype == EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8)
      ? HAL_DTYPE_I8 : HAL_DTYPE_U8;
  switch (self->colorspace) {
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY:
      self->target_format = HAL_PIXEL_FORMAT_GREY;
      break;
    case EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR:
      self->target_format = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW)
          ? HAL_PIXEL_FORMAT_PLANAR_RGB : HAL_PIXEL_FORMAT_BGRA;
      break;
    default:
      self->target_format = (self->layout == EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW)
          ? HAL_PIXEL_FORMAT_PLANAR_RGB : HAL_PIXEL_FORMAT_RGB;
      break;
  }
}

/* ── Tensor cache helpers ─────────────────────────────────────────── */

static void
tensor_cache_value_free (gpointer data)
{
  hal_tensor_free ((hal_tensor *) data);
}

static void
init_caches (EdgefirstCameraAdaptor *self)
{
  self->input_cache = g_hash_table_new_full (
      g_direct_hash, g_direct_equal, NULL, tensor_cache_value_free);
  self->output_cache = g_hash_table_new_full (
      g_direct_hash, g_direct_equal, NULL, tensor_cache_value_free);
}

static void
clear_caches (EdgefirstCameraAdaptor *self)
{
  if (self->input_cache)
    g_hash_table_remove_all (self->input_cache);
  if (self->output_cache)
    g_hash_table_remove_all (self->output_cache);
  if (self->hal_output) {
    hal_tensor_free (self->hal_output);
    self->hal_output = NULL;
  }
}

static void
destroy_caches (EdgefirstCameraAdaptor *self)
{
  g_clear_pointer (&self->input_cache, g_hash_table_destroy);
  g_clear_pointer (&self->output_cache, g_hash_table_destroy);
  if (self->hal_output) {
    hal_tensor_free (self->hal_output);
    self->hal_output = NULL;
  }
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
 * Lookup or import an input DMA-BUF as a HAL image tensor.
 * V4L2/ISP pools rotate ~4 fds; after one rotation every frame is a cache hit.
 * Returns a cached tensor (caller must NOT free).
 */
static hal_tensor *
lookup_or_import_input (EdgefirstCameraAdaptor *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width = GST_VIDEO_INFO_WIDTH (info);
  guint height = GST_VIDEO_INFO_HEIGHT (info);
  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (info);
  enum hal_pixel_format pixel_fmt = gst_format_to_hal_pixel (vfmt);

  if ((int) pixel_fmt == -1) {
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
  if (!gst_is_dmabuf_memory (mem0)) {
    GST_ERROR_OBJECT (self, "input buffer is not DMA-BUF");
    return NULL;
  }

  int fd = gst_dmabuf_memory_get_fd (mem0);
  gpointer key = GINT_TO_POINTER (fd);

  /* Cache lookup */
  hal_tensor *cached = g_hash_table_lookup (self->input_cache, key);
  if (cached) {
    GST_LOG_OBJECT (self, "input cache hit fd=%d", fd);
    return cached;
  }

  /* Cache miss — import via PlaneDescriptor */
  GST_DEBUG_OBJECT (self, "input cache miss fd=%d, importing %ux%u %s",
      fd, width, height, gst_video_format_to_string (vfmt));

  struct hal_plane_descriptor *pd = hal_plane_descriptor_new (fd);
  if (!pd) {
    GST_ERROR_OBJECT (self, "hal_plane_descriptor_new failed for fd=%d", fd);
    return NULL;
  }

  /* Set stride if padded (e.g. VPU buffers) */
  GstVideoMeta *vmeta = gst_buffer_get_video_meta (inbuf);
  gint stride = vmeta ? (gint) vmeta->stride[0]
                      : GST_VIDEO_INFO_PLANE_STRIDE (info, 0);
  hal_plane_descriptor_set_stride (pd, (size_t) stride);

  struct hal_plane_descriptor *chroma = NULL;
  if (n_mem >= 2 && pixel_fmt == HAL_PIXEL_FORMAT_NV12) {
    GstMemory *mem1 = gst_buffer_peek_memory (inbuf, 1);
    if (gst_is_dmabuf_memory (mem1)) {
      int uv_fd = gst_dmabuf_memory_get_fd (mem1);
      chroma = hal_plane_descriptor_new (uv_fd);
      if (chroma) {
        gint uv_stride = vmeta ? (gint) vmeta->stride[1]
                               : GST_VIDEO_INFO_PLANE_STRIDE (info, 1);
        hal_plane_descriptor_set_stride (chroma, (size_t) uv_stride);
      }
    }
  }

  /* hal_import_image CONSUMES pd and chroma */
  hal_tensor *tensor = hal_import_image (self->processor,
      pd, chroma, width, height, pixel_fmt, HAL_DTYPE_U8);
  if (!tensor) {
    GST_ERROR_OBJECT (self, "hal_import_image failed for fd=%d", fd);
    return NULL;
  }

  g_hash_table_insert (self->input_cache, key, tensor);
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
  g_clear_pointer (&self->processor, hal_image_processor_free);

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
  static const enum hal_compute_backend compute_map[] = {
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO]   = HAL_COMPUTE_BACKEND_AUTO,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_OPENGL] = HAL_COMPUTE_BACKEND_OPENGL,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_G2D]    = HAL_COMPUTE_BACKEND_G2D,
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_CPU]    = HAL_COMPUTE_BACKEND_CPU,
  };
  static const char *compute_names[] = {
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_AUTO]   = "auto",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_OPENGL] = "opengl",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_G2D]    = "g2d",
    [EDGEFIRST_CAMERA_ADAPTOR_COMPUTE_CPU]    = "cpu",
  };
  const char *backend_str = compute_names[self->compute];

  GST_INFO_OBJECT (self, "requesting HAL backend: %s", backend_str);
  self->processor = hal_image_processor_new_with_backend (
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
  g_clear_pointer (&self->processor, hal_image_processor_free);
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
    /* Src → Sink: accept supported video formats.
     * Build DMA_DRM caps with explicit drm-format list so upstream
     * (e.g. v4l2h264dec) only selects formats HAL can process. */
    static const GstVideoFormat supported_fmts[] = {
      GST_VIDEO_FORMAT_NV12,
      GST_VIDEO_FORMAT_YUY2,
      GST_VIDEO_FORMAT_RGB,
      GST_VIDEO_FORMAT_RGBA,
      GST_VIDEO_FORMAT_GRAY8,
    };

    /* Build drm-format string list from supported GstVideoFormats */
    GValue drm_list = G_VALUE_INIT;
    g_value_init (&drm_list, GST_TYPE_LIST);
    for (guint i = 0; i < G_N_ELEMENTS (supported_fmts); i++) {
      guint32 drm_fourcc =
          gst_video_dma_drm_fourcc_from_format (supported_fmts[i]);
      if (drm_fourcc != 0) {
        gchar *drm_str =
            gst_video_dma_drm_fourcc_to_string (drm_fourcc, 0);
        if (drm_str) {
          GValue v = G_VALUE_INIT;
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

    /* Non-DRM DMA-BUF caps (preferred over system memory) */
    GstCaps *raw_caps = gst_caps_from_string (
        "video/x-raw(memory:DMABuf), "
          "format={NV12, YUY2, RGB, RGBA, GRAY8}, "
          "width=[1,MAX], height=[1,MAX]");

    /* System memory caps: sources like libcamerasrc declare video/x-raw
     * without memory:DMABuf even though their allocator produces
     * DMABuf-backed memory (linear/mappable DMABuf omits the feature per
     * GStreamer convention).  Accept video/x-raw so caps negotiation
     * succeeds; actual memory type is verified at runtime. */
    GstCaps *sys_caps = gst_caps_from_string (
        "video/x-raw, "
          "format={NV12, YUY2, RGB, RGBA, GRAY8}, "
          "width=[1,MAX], height=[1,MAX]");

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

  if ((int) gst_format_to_hal_pixel (vfmt) == -1) {
    GST_ERROR_OBJECT (self, "unsupported input format %s",
        gst_video_format_to_string (vfmt));
    return FALSE;
  }

  /* Resolve output dimensions */
  self->out_width = self->model_width > 0 ? self->model_width : src_w;
  self->out_height = self->model_height > 0 ? self->model_height : src_h;
  self->out_channels = (self->colorspace ==
      EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) ? 1 : 3;

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
      if (c == 0)
        c = (self->colorspace ==
            EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY) ? 1 : 3;
    }

    if (w == 0 || h == 0)
      return FALSE;

    *othersize = w * h * c;
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
 * Get or create the output HAL tensor for the current frame.
 * When a downstream pool is available, the output buffer's DMA-BUF fd
 * is imported (cached by fd).  Otherwise a HAL-owned image is allocated
 * once and reused for all frames.
 */
static hal_tensor *
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

    hal_tensor *cached = g_hash_table_lookup (self->output_cache, key);
    if (cached) {
      GST_LOG_OBJECT (self, "output cache hit fd=%d", fd);
      return cached;
    }

    GST_DEBUG_OBJECT (self, "output cache miss fd=%d, importing", fd);
    struct hal_plane_descriptor *pd = hal_plane_descriptor_new (fd);
    if (!pd) {
      GST_ERROR_OBJECT (self, "hal_plane_descriptor_new failed for output fd=%d", fd);
      return NULL;
    }

    hal_tensor *tensor = hal_import_image (self->processor,
        pd, NULL, self->out_width, self->out_height,
        self->target_format, self->target_dtype);
    if (!tensor) {
      GST_ERROR_OBJECT (self, "hal_import_image failed for output fd=%d", fd);
      return NULL;
    }

    g_hash_table_insert (self->output_cache, key, tensor);
    return tensor;
  }

  /* No downstream pool — use HAL-owned output (allocated once) */
  if (!self->hal_output) {
    self->hal_output = hal_image_processor_create_image (self->processor,
        self->out_width, self->out_height,
        self->target_format, self->target_dtype);
    if (!self->hal_output) {
      GST_ERROR_OBJECT (self, "hal_image_processor_create_image failed");
      return NULL;
    }
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

  hal_tensor *src = lookup_or_import_input (self, inbuf);
  if (!src) {
    GST_ERROR_OBJECT (self, "failed to get input tensor");
    return GST_FLOW_ERROR;
  }

  hal_tensor *dst = get_output_tensor (self, outbuf);
  if (!dst) {
    GST_ERROR_OBJECT (self, "failed to get output tensor");
    return GST_FLOW_ERROR;
  }

  int ret = hal_image_processor_convert (self->processor, src, dst,
      HAL_ROTATION_NONE, HAL_FLIP_NONE,
      self->crop_valid ? &self->crop : NULL);

  if (ret != 0) {
    GST_ERROR_OBJECT (self, "hal_image_processor_convert failed (%d)", ret);
    return GST_FLOW_ERROR;
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
