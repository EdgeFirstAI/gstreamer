/*
 * EdgeFirst Perception for GStreamer - Overlay Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dual-sink GstElement: accepts video + other/tensors, runs HAL >= 0.16.3
 * decode->materialize->draw pipeline, emits new-detection signal.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstoverlay.h"

#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include <edgefirst/hal.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#if HAVE_NNSTREAMER
#include <nnstreamer_tensor_quant_meta.h>
#endif

/* HAL >= 0.16.3 is a hard dependency — all APIs below are always available. */

GST_DEBUG_CATEGORY_STATIC (edgefirst_overlay_debug);
#define GST_CAT_DEFAULT edgefirst_overlay_debug

static inline guint64
_get_time_ns (void)
{
  struct timespec ts;
  clock_gettime (CLOCK_MONOTONIC, &ts);
  return (guint64) ts.tv_sec * 1000000000ULL + (guint64) ts.tv_nsec;
}

/* ── Property IDs ────────────────────────────────────────────────── */

enum {
  PROP_0,
  PROP_MODEL_CONFIG,
  PROP_SCORE_THRESHOLD,
  PROP_IOU_THRESHOLD,
  PROP_DECODER_VERSION,
  PROP_MODEL_WIDTH,
  PROP_MODEL_HEIGHT,
  PROP_MODEL_SYNC,
  PROP_MODEL_SYNC_TIMEOUT,
  PROP_LETTERBOX,
  PROP_OPACITY,
  PROP_COLOR_MODE,
  PROP_CLASS_COLORS,
  PROP_COMPUTE,
  PROP_NORMALIZED,
};

/* Tri-state for normalized box coordinates */
typedef enum {
  OVERLAY_NORMALIZED_AUTO,    /* Infer from model-config or decoder version */
  OVERLAY_NORMALIZED_TRUE,    /* Boxes in [0,1] range */
  OVERLAY_NORMALIZED_FALSE,   /* Boxes in pixel coordinates */
} OverlayNormalized;

static GType
overlay_normalized_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { OVERLAY_NORMALIZED_AUTO, "Auto (infer from model config/decoder)", "auto" },
      { OVERLAY_NORMALIZED_TRUE, "Normalized [0,1] coordinates", "true" },
      { OVERLAY_NORMALIZED_FALSE, "Pixel coordinates", "false" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstOverlayNormalized", values);
    g_once_init_leave (&type, t);
  }
  return type;
}

/* Compute backend enum — mirrors EdgefirstCameraAdaptorCompute for consistency */
typedef enum {
  OVERLAY_COMPUTE_AUTO,
  OVERLAY_COMPUTE_OPENGL,
  OVERLAY_COMPUTE_G2D,
  OVERLAY_COMPUTE_CPU,
} OverlayCompute;

static GType
overlay_compute_get_type (void)
{
  static GType type = 0;
  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { OVERLAY_COMPUTE_AUTO, "Auto (HAL default)", "auto" },
      { OVERLAY_COMPUTE_OPENGL, "OpenGL", "opengl" },
      { OVERLAY_COMPUTE_G2D, "G2D", "g2d" },
      { OVERLAY_COMPUTE_CPU, "CPU", "cpu" },
      { 0, NULL, NULL },
    };
    GType t = g_enum_register_static ("EdgefirstOverlayCompute", values);
    g_once_init_leave (&type, t);
  }
  return type;
}

static const enum hal_compute_backend overlay_compute_map[] = {
  [OVERLAY_COMPUTE_AUTO]   = HAL_COMPUTE_BACKEND_AUTO,
  [OVERLAY_COMPUTE_OPENGL] = HAL_COMPUTE_BACKEND_OPENGL,
  [OVERLAY_COMPUTE_G2D]    = HAL_COMPUTE_BACKEND_G2D,
  [OVERLAY_COMPUTE_CPU]    = HAL_COMPUTE_BACKEND_CPU,
};

/* ── Signal IDs ──────────────────────────────────────────────────── */

enum {
  SIGNAL_NEW_DETECTION,
  N_SIGNALS,
};
static guint signals[N_SIGNALS];

/* ── Instance struct ─────────────────────────────────────────────── */

struct _EdgefirstOverlay {
  GstElement parent;

  /* Fixed pads */
  GstPad *video_sinkpad;
  GstPad *tensors_sinkpad;
  GstPad *srcpad;

  /* HAL state */
  hal_image_processor   *processor;
  hal_decoder           *decoder;
  hal_tensor            *display_images[2]; /* double-buffered RGBA render targets */
  guint                  display_buf_idx;   /* index of buffer to render into this frame */
  gboolean               display_is_dmabuf;
  GstAllocator          *dmabuf_allocator;

  /* Video info */
  GstVideoInfo           in_info;
  gboolean               in_info_valid;
  enum hal_pixel_format  src_pixel_format;
  guint                  display_w, display_h;

  /* Thread-safe decoded state */
  GMutex                 lock;
  GCond                  decode_cond;
  EdgeFirstDetectBoxList    *boxes_obj;       /* NULL until first tensor */
  EdgeFirstSegmentationList *segs_obj;        /* NULL for det-only models */
  GstClockTime           decode_ts;
  gboolean               flushing;
  gboolean               normalized;         /* TRUE if decoder outputs [0,1] coords */

  /* Cached tensor shapes from CAPS event (for chain function).
   * tensor_shapes: NNStreamer-squeezed shapes (row-major after reversal).
   * hal_shapes/hal_ndims: HAL-convention shapes used for tensor creation,
   * including batch dim and any rearrangement done in auto-config. */
  size_t   tensor_ndims[16];
  size_t   tensor_shapes[16][8];
  size_t   hal_ndims[16];
  size_t   hal_shapes[16][8];
  enum HalOutputType tensor_types[16];
  gint     tensor_count;
  enum hal_dtype tensor_dtypes[16];
  gboolean caps_parsed;           /* TRUE after tensors CAPS event parsed */
  gboolean has_split_boxes;
  gboolean has_protos;

  /* Properties */
  gchar     *model_config;
  gfloat     score_threshold;
  gfloat     iou_threshold;
  gchar     *decoder_version;
  guint      model_width;
  guint      model_height;
  gboolean   model_sync;
  guint      model_sync_timeout_ms;
  gchar     *letterbox_str;
  gboolean   has_letterbox;
  gfloat     letterbox[4];
  gfloat     opacity;
  EdgeFirstColorMode color_mode;
  gchar     *class_colors;
  OverlayCompute compute;
  OverlayNormalized normalized_prop;  /* user-facing tri-state property */
};

/* ── Pad templates ───────────────────────────────────────────────── */

static GstStaticPadTemplate video_sink_template =
    GST_STATIC_PAD_TEMPLATE ("video",
        GST_PAD_SINK, GST_PAD_ALWAYS,
        GST_STATIC_CAPS (
            "video/x-raw(memory:DMABuf), "
                "format={NV12,YUY2,RGB,RGBA,GRAY8}, width=[1,MAX], height=[1,MAX]; "
            "video/x-raw, "
                "format={NV12,YUY2,RGB,RGBA,GRAY8}, width=[1,MAX], height=[1,MAX]"));

static GstStaticPadTemplate tensors_sink_template =
    GST_STATIC_PAD_TEMPLATE ("tensors",
        GST_PAD_SINK, GST_PAD_ALWAYS,
        GST_STATIC_CAPS ("other/tensors"));

/* Output is always plain video/x-raw RGBA. DMA-BUF backing is transparent —
 * downstream elements (e.g. waylandsink) discover DMA-BUF at buffer-import
 * time via gst_is_dmabuf_memory(), not through caps features. */
static GstStaticPadTemplate src_template =
    GST_STATIC_PAD_TEMPLATE ("src",
        GST_PAD_SRC, GST_PAD_ALWAYS,
        GST_STATIC_CAPS (
            "video/x-raw, format=RGBA, width=[1,MAX], height=[1,MAX]"));

/* ── Type definition ─────────────────────────────────────────────── */

#define edgefirst_overlay_parent_class parent_class
G_DEFINE_TYPE (EdgefirstOverlay, edgefirst_overlay, GST_TYPE_ELEMENT)

/* Forward declarations */
static void            edgefirst_overlay_finalize    (GObject *);
static void            edgefirst_overlay_set_property (GObject *, guint, const GValue *, GParamSpec *);
static void            edgefirst_overlay_get_property (GObject *, guint, GValue *, GParamSpec *);
static GstStateChangeReturn edgefirst_overlay_change_state (GstElement *, GstStateChange);
static GstFlowReturn   edgefirst_overlay_video_chain   (GstPad *, GstObject *, GstBuffer *);
static GstFlowReturn   edgefirst_overlay_tensors_chain (GstPad *, GstObject *, GstBuffer *);
static gboolean        edgefirst_overlay_video_event   (GstPad *, GstObject *, GstEvent *);
static gboolean        edgefirst_overlay_tensors_event (GstPad *, GstObject *, GstEvent *);
static gboolean        edgefirst_overlay_src_query     (GstPad *, GstObject *, GstQuery *);

static void
edgefirst_overlay_class_init (EdgefirstOverlayClass *klass)
{
  GObjectClass    *obj_class = G_OBJECT_CLASS (klass);
  GstElementClass *el_class  = GST_ELEMENT_CLASS (klass);

  GST_DEBUG_CATEGORY_INIT (edgefirst_overlay_debug, "edgefirstoverlay", 0,
      "EdgeFirst Overlay");

  obj_class->finalize     = edgefirst_overlay_finalize;
  obj_class->set_property = edgefirst_overlay_set_property;
  obj_class->get_property = edgefirst_overlay_get_property;
  el_class->change_state  = edgefirst_overlay_change_state;

  /* Properties */
  g_object_class_install_property (obj_class, PROP_MODEL_CONFIG,
      g_param_spec_string ("model-config", "Model Config",
          "Path to edgefirst.yaml/.json or inline JSON. NULL = auto-configure from tensor caps.",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_SCORE_THRESHOLD,
      g_param_spec_float ("score-threshold", "Score Threshold",
          "NMS score threshold", 0.0f, 1.0f, 0.25f,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_IOU_THRESHOLD,
      g_param_spec_float ("iou-threshold", "IoU Threshold",
          "NMS IoU threshold for duplicate suppression", 0.0f, 1.0f, 0.45f,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_DECODER_VERSION,
      g_param_spec_string ("decoder-version", "Decoder Version",
          "YOLO version for auto-config: yolov5, yolov8, yolo11, yolo26",
          "yolov8", G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_MODEL_WIDTH,
      g_param_spec_uint ("model-width", "Model Width",
          "Model input width (0 = infer from tensor caps)", 0, G_MAXUINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_MODEL_HEIGHT,
      g_param_spec_uint ("model-height", "Model Height",
          "Model input height (0 = infer from tensor caps)", 0, G_MAXUINT, 0,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_MODEL_SYNC,
      g_param_spec_boolean ("model-sync", "Model Sync",
          "Wait for matching tensor timestamp before drawing (B mode)",
          FALSE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_MODEL_SYNC_TIMEOUT,
      g_param_spec_uint ("model-sync-timeout", "Model Sync Timeout",
          "Max ms to wait in model-sync mode (0 = wait indefinitely)",
          0, G_MAXUINT, 500,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_LETTERBOX,
      g_param_spec_string ("letterbox", "Letterbox",
          "Normalized letterbox rect \"x0,y0,x1,y1\" (NULL = no correction)",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_OPACITY,
      g_param_spec_float ("opacity", "Opacity",
          "Mask opacity [0.0-1.0]", 0.0f, 1.0f, 0.6f,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_COLOR_MODE,
      g_param_spec_enum ("color-mode", "Color Mode",
          "How to assign colors to detections",
          EDGEFIRST_TYPE_COLOR_MODE, EDGEFIRST_COLOR_MODE_CLASS,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_CLASS_COLORS,
      g_param_spec_string ("class-colors", "Class Colors",
          "Comma-separated RGBA hex values per class, e.g. \"FF0000FF,00FF00FF\"",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_COMPUTE,
      g_param_spec_enum ("compute", "Compute Backend",
          "HAL image processing backend (auto, opengl, g2d, cpu)",
          overlay_compute_get_type (),
          OVERLAY_COMPUTE_AUTO,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (obj_class, PROP_NORMALIZED,
      g_param_spec_enum ("normalized", "Normalized Coordinates",
          "Whether box coordinates are in normalized [0,1] range. "
          "Auto infers from model-config or defaults to true for YOLOv8.",
          overlay_normalized_get_type (),
          OVERLAY_NORMALIZED_AUTO,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /* new-detection signal */
  signals[SIGNAL_NEW_DETECTION] = g_signal_new ("new-detection",
      G_TYPE_FROM_CLASS (klass),
      G_SIGNAL_RUN_LAST,
      0, NULL, NULL, NULL,
      G_TYPE_NONE, 2,
      EDGEFIRST_TYPE_DETECT_BOX_LIST,
      EDGEFIRST_TYPE_SEGMENTATION_LIST);

  gst_element_class_set_static_metadata (el_class,
      "EdgeFirst Overlay", "Filter/Video",
      "GPU-accelerated detection/segmentation overlay with dual-pad input",
      "Au-Zone Technologies <info@au-zone.com>");

  gst_element_class_add_static_pad_template (el_class, &video_sink_template);
  gst_element_class_add_static_pad_template (el_class, &tensors_sink_template);
  gst_element_class_add_static_pad_template (el_class, &src_template);
}

static void
edgefirst_overlay_init (EdgefirstOverlay *self)
{
  /* Create pads */
  self->video_sinkpad = gst_pad_new_from_static_template (&video_sink_template, "video");
  gst_pad_set_chain_function (self->video_sinkpad, edgefirst_overlay_video_chain);
  gst_pad_set_event_function (self->video_sinkpad, edgefirst_overlay_video_event);
  gst_element_add_pad (GST_ELEMENT (self), self->video_sinkpad);

  self->tensors_sinkpad = gst_pad_new_from_static_template (&tensors_sink_template, "tensors");
  gst_pad_set_chain_function (self->tensors_sinkpad, edgefirst_overlay_tensors_chain);
  gst_pad_set_event_function (self->tensors_sinkpad, edgefirst_overlay_tensors_event);
  gst_element_add_pad (GST_ELEMENT (self), self->tensors_sinkpad);

  self->srcpad = gst_pad_new_from_static_template (&src_template, "src");
  gst_pad_set_query_function (self->srcpad, edgefirst_overlay_src_query);
  gst_element_add_pad (GST_ELEMENT (self), self->srcpad);

  /* Default property values */
  self->score_threshold       = 0.25f;
  self->iou_threshold         = 0.45f;
  self->decoder_version       = g_strdup ("yolov8");
  self->model_sync_timeout_ms = 500;
  self->opacity               = 0.6f;
  self->color_mode            = EDGEFIRST_COLOR_MODE_CLASS;
  self->decode_ts             = GST_CLOCK_TIME_NONE;
  self->dmabuf_allocator      = gst_dmabuf_allocator_new ();
}

/* ── finalize, set_property, get_property ────────────────────────── */

static void
edgefirst_overlay_finalize (GObject *object)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  g_free (self->model_config);
  g_free (self->decoder_version);
  g_free (self->letterbox_str);
  g_free (self->class_colors);
  gst_object_unref (self->dmabuf_allocator);

  /* HAL and GObjects freed in stop(); clear in case finalize is called early */
  g_clear_object (&self->boxes_obj);
  g_clear_object (&self->segs_obj);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
edgefirst_overlay_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  switch (prop_id) {
    case PROP_MODEL_CONFIG:
      g_free (self->model_config);
      self->model_config = g_value_dup_string (value);
      break;
    case PROP_SCORE_THRESHOLD:
      self->score_threshold = g_value_get_float (value);
      break;
    case PROP_IOU_THRESHOLD:
      self->iou_threshold = g_value_get_float (value);
      break;
    case PROP_DECODER_VERSION:
      g_free (self->decoder_version);
      self->decoder_version = g_value_dup_string (value);
      break;
    case PROP_MODEL_WIDTH:
      self->model_width = g_value_get_uint (value);
      break;
    case PROP_MODEL_HEIGHT:
      self->model_height = g_value_get_uint (value);
      break;
    case PROP_MODEL_SYNC:
      self->model_sync = g_value_get_boolean (value);
      break;
    case PROP_MODEL_SYNC_TIMEOUT:
      self->model_sync_timeout_ms = g_value_get_uint (value);
      break;
    case PROP_LETTERBOX:
      g_free (self->letterbox_str);
      self->letterbox_str = g_value_dup_string (value);
      self->has_letterbox = FALSE;
      if (self->letterbox_str) {
        gint n = sscanf (self->letterbox_str, "%f,%f,%f,%f",
            &self->letterbox[0], &self->letterbox[1],
            &self->letterbox[2], &self->letterbox[3]);
        self->has_letterbox = (n == 4);
        if (!self->has_letterbox)
          GST_WARNING_OBJECT (self, "Invalid letterbox string: %s",
              self->letterbox_str);
      }
      break;
    case PROP_OPACITY:
      self->opacity = g_value_get_float (value);
      break;
    case PROP_COLOR_MODE:
      self->color_mode = (EdgeFirstColorMode) g_value_get_enum (value);
      break;
    case PROP_CLASS_COLORS:
      g_free (self->class_colors);
      self->class_colors = g_value_dup_string (value);
      break;
    case PROP_COMPUTE:
      self->compute = (OverlayCompute) g_value_get_enum (value);
      break;
    case PROP_NORMALIZED:
      self->normalized_prop = (OverlayNormalized) g_value_get_enum (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
  }
}

static void
edgefirst_overlay_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  switch (prop_id) {
    case PROP_MODEL_CONFIG:       g_value_set_string  (value, self->model_config);        break;
    case PROP_SCORE_THRESHOLD:    g_value_set_float   (value, self->score_threshold);     break;
    case PROP_IOU_THRESHOLD:      g_value_set_float   (value, self->iou_threshold);       break;
    case PROP_DECODER_VERSION:    g_value_set_string  (value, self->decoder_version);     break;
    case PROP_MODEL_WIDTH:        g_value_set_uint    (value, self->model_width);         break;
    case PROP_MODEL_HEIGHT:       g_value_set_uint    (value, self->model_height);        break;
    case PROP_MODEL_SYNC:         g_value_set_boolean (value, self->model_sync);          break;
    case PROP_MODEL_SYNC_TIMEOUT: g_value_set_uint    (value, self->model_sync_timeout_ms); break;
    case PROP_LETTERBOX:          g_value_set_string  (value, self->letterbox_str);       break;
    case PROP_OPACITY:            g_value_set_float   (value, self->opacity);             break;
    case PROP_COLOR_MODE:         g_value_set_enum    (value, (gint) self->color_mode);   break;
    case PROP_CLASS_COLORS:       g_value_set_string  (value, self->class_colors);        break;
    case PROP_COMPUTE:            g_value_set_enum    (value, (gint) self->compute);      break;
    case PROP_NORMALIZED:         g_value_set_enum    (value, (gint) self->normalized_prop); break;
    default: G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec); break;
  }
}

/* ── start / stop ────────────────────────────────────────────────── */

/* Map "yolov8" string property to HalDecoderVersion enum */
static enum HalDecoderVersion
parse_decoder_version (const gchar *str)
{
  if (!str || g_ascii_strcasecmp (str, "yolov8") == 0)
    return HAL_DECODER_VERSION_YOLOV8;
  if (g_ascii_strcasecmp (str, "yolo11") == 0)
    return HAL_DECODER_VERSION_YOLO11;
  if (g_ascii_strcasecmp (str, "yolov5") == 0)
    return HAL_DECODER_VERSION_YOLOV5;
  if (g_ascii_strcasecmp (str, "yolo26") == 0)
    return HAL_DECODER_VERSION_YOLO26;
  return HAL_DECODER_VERSION_YOLOV8;
}

/* Map GstVideoFormat to HAL pixel format */
static enum hal_pixel_format
gst_format_to_hal (GstVideoFormat fmt)
{
  switch (fmt) {
    case GST_VIDEO_FORMAT_NV12:  return HAL_PIXEL_FORMAT_NV12;
    case GST_VIDEO_FORMAT_YUY2:  return HAL_PIXEL_FORMAT_YUYV;
    case GST_VIDEO_FORMAT_RGB:   return HAL_PIXEL_FORMAT_RGB;
    case GST_VIDEO_FORMAT_RGBA:  return HAL_PIXEL_FORMAT_RGBA;
    case GST_VIDEO_FORMAT_GRAY8: return HAL_PIXEL_FORMAT_GREY;
    default:                     return HAL_PIXEL_FORMAT_RGB;
  }
}

static gboolean
overlay_start (EdgefirstOverlay *self)
{
  enum hal_compute_backend backend = overlay_compute_map[self->compute];
  if (backend == HAL_COMPUTE_BACKEND_AUTO)
    self->processor = hal_image_processor_new ();
  else
    self->processor = hal_image_processor_new_with_backend (backend);
  if (!self->processor) {
    GST_ELEMENT_ERROR (self, RESOURCE, FAILED,
        ("Failed to create HAL image processor: %s", strerror (errno)), (NULL));
    return FALSE;
  }
  GST_INFO_OBJECT (self, "HAL image processor created (compute=%s)",
      self->compute == OVERLAY_COMPUTE_AUTO ? "auto" :
      self->compute == OVERLAY_COMPUTE_OPENGL ? "opengl" :
      self->compute == OVERLAY_COMPUTE_G2D ? "g2d" : "cpu");

  if (self->model_config) {
    struct hal_decoder_params *params = hal_decoder_params_new ();
    if (g_file_test (self->model_config, G_FILE_TEST_EXISTS))
      hal_decoder_params_set_config_file (params, self->model_config);
    else
      hal_decoder_params_set_config_json (params, self->model_config, 0);
    hal_decoder_params_set_score_threshold (params, self->score_threshold);
    hal_decoder_params_set_iou_threshold (params, self->iou_threshold);

    self->decoder = hal_decoder_new (params);
    hal_decoder_params_free (params);

    if (!self->decoder) {
      GST_ELEMENT_ERROR (self, RESOURCE, FAILED,
          ("Failed to create HAL decoder from config \"%s\": %s",
           self->model_config, strerror (errno)), (NULL));
      hal_image_processor_free (self->processor);
      self->processor = NULL;
      return FALSE;
    }
    /* model-config provides normalized flag; property override takes precedence */
    if (self->normalized_prop == OVERLAY_NORMALIZED_AUTO)
      self->normalized = hal_decoder_normalized_boxes (self->decoder);
    else
      self->normalized = (self->normalized_prop == OVERLAY_NORMALIZED_TRUE);
  } else {
    self->decoder   = NULL;   /* deferred: auto-configure on tensors CAPS */
    /* Default: YOLOv8 Ultralytics outputs normalized coords */
    self->normalized = (self->normalized_prop != OVERLAY_NORMALIZED_FALSE);
  }

  g_mutex_init (&self->lock);
  g_cond_init (&self->decode_cond);
  self->flushing  = FALSE;
  self->decode_ts = GST_CLOCK_TIME_NONE;

  return TRUE;
}

static void
overlay_stop (EdgefirstOverlay *self)
{
  g_mutex_lock (&self->lock);
  self->flushing = TRUE;
  g_cond_broadcast (&self->decode_cond);
  g_mutex_unlock (&self->lock);

  hal_decoder_free (self->decoder);
  self->decoder = NULL;
  self->tensor_count = 0;

  hal_image_processor_free (self->processor);
  self->processor = NULL;

  for (int i = 0; i < 2; i++) {
    hal_tensor_free (self->display_images[i]);
    self->display_images[i] = NULL;
  }

  g_mutex_lock (&self->lock);
  g_clear_object (&self->boxes_obj);
  g_clear_object (&self->segs_obj);
  g_mutex_unlock (&self->lock);

  g_mutex_clear (&self->lock);
  g_cond_clear (&self->decode_cond);
}

static GstStateChangeReturn
edgefirst_overlay_change_state (GstElement *element, GstStateChange transition)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (element);
  GstStateChangeReturn ret;

  switch (transition) {
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      if (!overlay_start (self))
        return GST_STATE_CHANGE_FAILURE;
      break;
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);

  switch (transition) {
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      overlay_stop (self);
      break;
    default:
      break;
  }

  return ret;
}

/* ── auto_config_decoder (tensors CAPS handler helper) ───────────── */

/* Parse NNStreamer tensor dtype string to HAL dtype */
static enum hal_dtype
nnstreamer_type_to_hal (const gchar *type_str)
{
  if (!type_str) return HAL_DTYPE_F32;
  if (g_strcmp0 (type_str, "float32") == 0) return HAL_DTYPE_F32;
  if (g_strcmp0 (type_str, "uint8")   == 0) return HAL_DTYPE_U8;
  if (g_strcmp0 (type_str, "int8")    == 0) return HAL_DTYPE_I8;
  if (g_strcmp0 (type_str, "int16")   == 0) return HAL_DTYPE_I16;
  if (g_strcmp0 (type_str, "int32")   == 0) return HAL_DTYPE_I32;
  return HAL_DTYPE_F32;
}

/* Parse NNStreamer dim string "C:W:H:B" (innermost first) to shape[] (row-major).
 * Returns ndim, fills shape[0..ndim-1] in row-major order (outermost first).
 * Strips trailing :1 dimensions, then squeezes interior unit dimensions
 * (keeps min 2 dims). This handles Ara-2 DVM output like "32:1:8400:1"
 * → [8400, 32] instead of [8400, 1, 32]. */
static size_t
parse_nnstreamer_dims (const gchar *dim_str, size_t *shape, size_t max_ndim)
{
  gchar **parts = g_strsplit (dim_str, ":", (gint) max_ndim + 1);
  size_t n = 0;
  while (parts[n] && n < max_ndim) n++;
  /* Strip trailing :1 dimensions (outermost batch dims) */
  while (n > 1 && g_strcmp0 (parts[n - 1], "1") == 0) n--;
  /* NNStreamer: innermost first → reverse to row-major */
  size_t raw[8];
  for (size_t i = 0; i < n && i < 8; i++)
    raw[i] = (size_t) g_ascii_strtoull (parts[n - 1 - i], NULL, 10);
  g_strfreev (parts);
  /* Squeeze interior unit (=1) dimensions, keeping at least 2 dims */
  size_t out = 0;
  for (size_t i = 0; i < n; i++) {
    if (raw[i] != 1 || out == 0)
      shape[out++] = raw[i];
  }
  if (out < 2 && n >= 2)
    shape[out++] = 1;
  return out;
}

/* Phase 1: Parse tensor caps and compute HAL-convention shapes.
 * Called from CAPS event on the tensors pad. Does NOT create the decoder
 * because quantization metadata is only available on the first buffer. */
static gboolean
overlay_parse_tensor_caps (EdgefirstOverlay *self, GstCaps *caps)
{
  GstStructure *s = gst_caps_get_structure (caps, 0);
  gint num_tensors = 0;
  const gchar *dims_str = NULL, *types_str = NULL;

  if (!gst_structure_get_int (s, "num_tensors", &num_tensors) || num_tensors <= 0)
    return FALSE;

  dims_str  = gst_structure_get_string (s, "dimensions");
  types_str = gst_structure_get_string (s, "types");

  if (!dims_str || !types_str)
    return FALSE;

  gchar **dim_parts  = g_strsplit (dims_str,  ",", num_tensors + 1);
  gchar **type_parts = g_strsplit (types_str, ",", num_tensors + 1);

#define MAX_NDIM 8
  /* ── Pass 1: detect flags ────────────────────────────────────── */
  gboolean has_protos      = FALSE;
  size_t   proto_channels  = 0;
  gboolean has_split_boxes = FALSE;

  self->tensor_count = (num_tensors < 16) ? num_tensors : 16;

  for (gint i = 0; i < self->tensor_count; i++) {
    self->tensor_ndims[i] = parse_nnstreamer_dims (dim_parts[i],
        self->tensor_shapes[i], MAX_NDIM);
    self->tensor_dtypes[i] = nnstreamer_type_to_hal (type_parts[i]);
  }

  /* Detect protos: a 3D tensor where the first two dims are spatial (both > 1)
   * and the third is channels, e.g. [160, 160, 32]. Distinguish from detection
   * tensors like [1, 8400, 4] where dim[0]=1 (batch). */
  for (gint i = 0; i < self->tensor_count; i++) {
    size_t nd = self->tensor_ndims[i];
    if (nd == 3 && self->tensor_shapes[i][0] > 1 && self->tensor_shapes[i][1] > 1) {
      has_protos = TRUE;
      proto_channels = self->tensor_shapes[i][2];
      break;
    }
  }

  /* Detect split-box format: look for a tensor whose box-coordinate dimension == 4.
   * Two conventions exist after parse_nnstreamer_dims reversal:
   *   TFLite (features-first): [1,4,8400] → [4,8400]   — features at shape[0]
   *   Ara-2  (anchors-first):  [1,8400,4] → [8400,4]   — features at shape[nd-1]
   * Check both ends to handle either framework. */
  for (gint i = 0; i < self->tensor_count; i++) {
    size_t nd = self->tensor_ndims[i];
    if (nd < 2) continue;
    /* Skip the protos tensor */
    if (nd == 3 && self->tensor_shapes[i][0] > 1 && self->tensor_shapes[i][1] > 1)
      continue;
    size_t d0    = self->tensor_shapes[i][0];
    size_t dlast = self->tensor_shapes[i][nd - 1];
    if (d0 == 4 || dlast == 4) {
      has_split_boxes = TRUE;
      break;
    }
  }

  /* ── Pass 2: compute HAL-convention shapes and output types ──── */
  /* Track per-tensor output type for quant application later */

  static const char *type_names[HAL_OUTPUT_TYPE_CLASSES + 1] = {
    [HAL_OUTPUT_TYPE_DETECTION]         = "DETECTION",
    [HAL_OUTPUT_TYPE_BOXES]             = "BOXES",
    [HAL_OUTPUT_TYPE_SCORES]            = "SCORES",
    [HAL_OUTPUT_TYPE_PROTOS]            = "PROTOS",
    [HAL_OUTPUT_TYPE_SEGMENTATION]      = "SEGMENTATION",
    [HAL_OUTPUT_TYPE_MASK_COEFFICIENTS] = "MASK_COEFF",
    [HAL_OUTPUT_TYPE_MASK]              = "MASK",
    [HAL_OUTPUT_TYPE_CLASSES]           = "CLASSES",
  };

  for (gint i = 0; i < self->tensor_count; i++) {
    size_t ndim = self->tensor_ndims[i];
    size_t out_shape[MAX_NDIM] = {0};
    memcpy (out_shape, self->tensor_shapes[i], ndim * sizeof (size_t));

    enum HalOutputType type;

    /* Protos: 3D with spatial dims > 1, e.g. [160, 160, 32] → HAL [1, C, H, W] */
    gboolean is_protos = (ndim == 3 && out_shape[0] > 1 && out_shape[1] > 1);

    if (is_protos) {
      size_t H = out_shape[0], W = out_shape[1], C = out_shape[2];
      out_shape[0] = 1; out_shape[1] = C; out_shape[2] = H; out_shape[3] = W;
      type = HAL_OUTPUT_TYPE_PROTOS;
      ndim = 4;
    } else if (has_split_boxes) {
      /* Split-box output: determine features vs anchors using min-dimension heuristic.
       * Features (4, 80, 32, …) are always << anchors (8400), so the smaller
       * dimension is features regardless of which end it sits on:
       *   TFLite (features-first): [4,8400]   → d0=4  < dlast=8400 → nf=4
       *   Ara-2  (anchors-first):  [8400,4]   → d0=8400 > dlast=4  → nf=4 */
      size_t d0    = (ndim >= 3 && out_shape[0] == 1) ? out_shape[1] : out_shape[0];
      size_t dlast = out_shape[ndim - 1];
      size_t nf = (d0 <= dlast) ? d0 : dlast;  /* features: smaller dim */
      size_t nb = (d0 <= dlast) ? dlast : d0;  /* anchors:  larger dim  */

      /* Normalize to HAL [1, features, anchors] — matches add_split() in reference binary */
      out_shape[0] = 1; out_shape[1] = nf; out_shape[2] = nb;
      ndim = 3;

      if (nf == 4)
        type = HAL_OUTPUT_TYPE_BOXES;
      else if (has_protos && proto_channels > 0 && nf == proto_channels)
        type = HAL_OUTPUT_TYPE_MASK_COEFFICIENTS;
      else
        type = HAL_OUTPUT_TYPE_SCORES;
    } else {
      /* Fused detection — always 3D [1, features, boxes].
       * NNStreamer reversed 2D: [feat, boxes]. */
      type = HAL_OUTPUT_TYPE_DETECTION;
      if (ndim == 2) {
        size_t nf = out_shape[0], nb = out_shape[1];
        out_shape[0] = 1; out_shape[1] = nf; out_shape[2] = nb;
      }
      ndim = 3;
    }

    self->hal_ndims[i] = ndim;
    self->tensor_types[i] = type;
    for (size_t j = 0; j < ndim && j < 8; j++)
      self->hal_shapes[i][j] = out_shape[j];

    gchar shape_str[128];
    gint pos = g_snprintf (shape_str, sizeof (shape_str), "[%zu", out_shape[0]);
    for (size_t j = 1; j < ndim && pos < (gint) sizeof (shape_str) - 8; j++)
      pos += g_snprintf (shape_str + pos, sizeof (shape_str) - pos, ",%zu", out_shape[j]);
    g_strlcat (shape_str, "]", sizeof (shape_str));
    GST_INFO_OBJECT (self, "  [%d] %-11s ndim=%zu dtype=%d shape=%s",
        i, (type <= HAL_OUTPUT_TYPE_CLASSES && type_names[type]) ? type_names[type] : "?",
        ndim, self->tensor_dtypes[i], shape_str);
  }

  self->has_split_boxes = has_split_boxes;
  self->has_protos = has_protos;

  g_strfreev (dim_parts);
  g_strfreev (type_parts);

  self->caps_parsed = TRUE;
  GST_INFO_OBJECT (self, "parsed tensor caps: %d tensors, has_split=%d, has_protos=%d",
      self->tensor_count, has_split_boxes, has_protos);
  return TRUE;
}

/* Phase 2: Create the HAL decoder from cached shapes + quant params from buffer.
 * Called from the first tensor chain invocation where the buffer is available
 * for GstNnsTensorQuantMeta extraction. */
static gboolean
overlay_create_decoder (EdgefirstOverlay *self, GstBuffer *buf)
{
  struct hal_decoder_params *params = hal_decoder_params_new ();
  hal_decoder_params_set_score_threshold (params, self->score_threshold);
  hal_decoder_params_set_iou_threshold   (params, self->iou_threshold);
  hal_decoder_params_set_decoder_version (params, parse_decoder_version (self->decoder_version));
  hal_decoder_params_set_nms (params, HAL_NMS_CLASS_AGNOSTIC);

  /* Infer model input size from protos spatial dims (160*4=640) or model_width property */
  guint model_input_size = self->model_width > 0 ? self->model_width : 640;
  for (gint i = 0; i < self->tensor_count; i++) {
    if (self->tensor_types[i] == HAL_OUTPUT_TYPE_PROTOS && self->hal_ndims[i] == 4) {
      guint inferred = (guint) self->hal_shapes[i][2] * 4;  /* proto H * stride */
      if (inferred > 0 && self->model_width == 0) {
        model_input_size = inferred;
        GST_INFO_OBJECT (self, "inferred model input size %u from protos", model_input_size);
      }
      break;
    }
  }

  /* Extract quantization parameters from buffer if available */
#if HAVE_NNSTREAMER
  GstNnsTensorQuantMeta *qm = gst_buffer_get_nns_tensor_quant_meta (buf);
#else
  void *qm = NULL;
#endif

  gint boxes_idx = -1;  /* track which decoder output is boxes/detection */

  for (gint i = 0; i < self->tensor_count; i++) {
    enum HalOutputType type = self->tensor_types[i];
    size_t ndim = self->hal_ndims[i];

    /* Build dim names from type */
    enum HalDimName dims[MAX_NDIM];
    switch (type) {
      case HAL_OUTPUT_TYPE_PROTOS:
        dims[0] = HAL_DIM_NAME_BATCH; dims[1] = HAL_DIM_NAME_NUM_PROTOS;
        dims[2] = HAL_DIM_NAME_HEIGHT; dims[3] = HAL_DIM_NAME_WIDTH;
        break;
      case HAL_OUTPUT_TYPE_BOXES:
        dims[0] = HAL_DIM_NAME_BATCH; dims[1] = HAL_DIM_NAME_BOX_COORDS;
        dims[2] = HAL_DIM_NAME_NUM_BOXES;
        break;
      case HAL_OUTPUT_TYPE_MASK_COEFFICIENTS:
        dims[0] = HAL_DIM_NAME_BATCH; dims[1] = HAL_DIM_NAME_NUM_PROTOS;
        dims[2] = HAL_DIM_NAME_NUM_BOXES;
        break;
      case HAL_OUTPUT_TYPE_SCORES:
        dims[0] = HAL_DIM_NAME_BATCH; dims[1] = HAL_DIM_NAME_NUM_CLASSES;
        dims[2] = HAL_DIM_NAME_NUM_BOXES;
        break;
      default: /* DETECTION */
        dims[0] = HAL_DIM_NAME_BATCH; dims[1] = HAL_DIM_NAME_NUM_FEATURES;
        dims[2] = HAL_DIM_NAME_NUM_BOXES;
        break;
    }

    gint idx = hal_decoder_params_add_output (params, type,
        HAL_DECODER_TYPE_ULTRALYTICS, self->hal_shapes[i], dims, ndim);

    if (idx >= 0 && (type == HAL_OUTPUT_TYPE_BOXES || type == HAL_OUTPUT_TYPE_DETECTION)) {
      gboolean is_norm = (self->normalized_prop != OVERLAY_NORMALIZED_FALSE);
      hal_decoder_params_output_set_normalized (params, idx, is_norm ? 1 : 0);
      boxes_idx = idx;
    }

    /* Set quantization: prefer actual quant meta, fall back to identity */
    if (self->tensor_dtypes[i] != HAL_DTYPE_F32 && idx >= 0) {
      gfloat scale = 1.0f;
      gint   zp    = 0;

#if HAVE_NNSTREAMER
      if (qm && (guint) i < qm->num_tensors) {
        const NnsTensorQuantInfo *qi = &qm->quant[i];
        if (qi->scheme != NNS_QUANT_NONE && qi->num_params > 0) {
          scale = (gfloat) qi->scales[0];
          zp    = (gint) qi->zero_points[0];
          GST_DEBUG_OBJECT (self, "  [%d] quant from meta: scale=%g zp=%d", i, scale, zp);
        }
      }
#endif

      /* For boxes, scale must account for model input size to normalize
       * pixel-space coordinates to [0,1]. See yolov8n_seg_ara2.cpp:273 */
      if ((type == HAL_OUTPUT_TYPE_BOXES || type == HAL_OUTPUT_TYPE_DETECTION)
          && model_input_size > 0 && scale != 1.0f) {
        scale /= (gfloat) model_input_size;
        GST_DEBUG_OBJECT (self, "  [%d] boxes scale adjusted by /%u → %g",
            i, model_input_size, scale);
      }

      hal_decoder_params_output_set_quantization (params, idx, scale, zp);
    }
  }

  self->decoder = hal_decoder_new (params);
  hal_decoder_params_free (params);

  if (!self->decoder) {
    GST_WARNING_OBJECT (self,
        "auto-config decoder creation failed (%s); running in pass-through mode",
        strerror (errno));
    return FALSE;
  }

  if (self->normalized_prop == OVERLAY_NORMALIZED_AUTO)
    self->normalized = hal_decoder_normalized_boxes (self->decoder);
  else
    self->normalized = (self->normalized_prop == OVERLAY_NORMALIZED_TRUE);

  GST_INFO_OBJECT (self, "decoder created: %d tensors, normalized=%d, "
      "quant_meta=%s", self->tensor_count, self->normalized,
      qm ? "yes" : "no");
  return TRUE;
}

/* ── Helper: parse_class_colors ──────────────────────────────────── */

/* Parse comma-separated RGBA hex color values into uint8_t arrays.
 * E.g. "FF0000FF,00FF00FF" → {{255,0,0,255}, {0,255,0,255}, ...}
 * Returns number of colors parsed. Caller must g_free the output. */
static gsize
parse_class_colors (const gchar *str, uint8_t (**out_colors)[4])
{
  *out_colors = NULL;
  if (!str || str[0] == '\0')
    return 0;

  gchar **parts = g_strsplit (str, ",", -1);
  guint n = g_strv_length (parts);

  uint8_t (*colors)[4] = g_malloc (n * sizeof (*colors));
  gsize count = 0;

  for (guint i = 0; i < n; i++) {
    gchar *s = g_strstrip (parts[i]);
    if (strlen (s) != 8)
      continue;
    guint32 val = (guint32) g_ascii_strtoull (s, NULL, 16);
    colors[count][0] = (val >> 24) & 0xFF;
    colors[count][1] = (val >> 16) & 0xFF;
    colors[count][2] = (val >>  8) & 0xFF;
    colors[count][3] = (val      ) & 0xFF;
    count++;
  }

  g_strfreev (parts);
  *out_colors = colors;
  return count;
}

/* ── Helper: create_input_image ──────────────────────────────────── */

/* Create a HAL input tensor from a GstBuffer.
 * Tries DMABuf zero-copy via hal_import_image first; falls back to
 * hal_image_processor_create_image + memcpy.
 * Returns a new hal_tensor that the caller must free with hal_tensor_free(). */
static hal_tensor *
create_input_image (EdgefirstOverlay *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width  = (guint) self->display_w;
  guint height = (guint) self->display_h;
  enum hal_pixel_format pixel_fmt = self->src_pixel_format;

  /* Determine packed row_bytes for memcpy fallback */
  size_t row_bytes;
  switch (pixel_fmt) {
    case HAL_PIXEL_FORMAT_RGB:   row_bytes = width * 3; break;
    case HAL_PIXEL_FORMAT_RGBA:  row_bytes = width * 4; break;
    case HAL_PIXEL_FORMAT_GREY:  row_bytes = width;     break;
    case HAL_PIXEL_FORMAT_YUYV:  row_bytes = width * 2; break;
    case HAL_PIXEL_FORMAT_NV12:  row_bytes = width;     break;
    default:
      GST_ERROR_OBJECT (self, "unsupported input pixel format %d", pixel_fmt);
      return NULL;
  }

  /* Prefer GstVideoMeta for plane layout — it is the authoritative record of
   * the actual buffer strides and offsets set by the producer (e.g.
   * v4l2h264dec, which pads the Y plane to a macroblock boundary). The
   * GstVideoInfo parsed from caps only carries width/height and computes
   * *tight-packed* strides, so it silently corrupts buffers that have row
   * or plane padding. Reading the wrong UV offset manifests as a colored
   * band at the top of the output (UV sampled from Y-plane padding bytes). */
  GstVideoMeta *vm = gst_buffer_get_video_meta (inbuf);
  gint    y_stride  = vm ? vm->stride[0] : GST_VIDEO_INFO_PLANE_STRIDE (info, 0);
  gint    uv_stride = vm ? vm->stride[1] : GST_VIDEO_INFO_PLANE_STRIDE (info, 1);
  gsize   y_offset  = vm ? vm->offset[0] : GST_VIDEO_INFO_PLANE_OFFSET (info, 0);
  gsize   uv_offset = vm ? vm->offset[1] : GST_VIDEO_INFO_PLANE_OFFSET (info, 1);

  /* Try DMA-BUF zero-copy path */
  guint n_mem = gst_buffer_n_memory (inbuf);
  GstMemory *in_mem = gst_buffer_peek_memory (inbuf, 0);
  if (gst_is_dmabuf_memory (in_mem)) {
    int fd = gst_dmabuf_memory_get_fd (in_mem);
    struct hal_plane_descriptor *pd = hal_plane_descriptor_new (fd);
    if (pd) {
      hal_plane_descriptor_set_stride (pd, (size_t) y_stride);
      if (y_offset > 0)
        hal_plane_descriptor_set_offset (pd, y_offset);

      struct hal_plane_descriptor *chroma = NULL;
      if (pixel_fmt == HAL_PIXEL_FORMAT_NV12) {
        int   uv_fd = fd;
        gsize chroma_offset = uv_offset;
        if (n_mem >= 2) {
          /* UV in its own GstMemory block: use that fd, offset starts at 0 */
          GstMemory *mem1 = gst_buffer_peek_memory (inbuf, 1);
          if (gst_is_dmabuf_memory (mem1)) {
            uv_fd = gst_dmabuf_memory_get_fd (mem1);
            chroma_offset = 0;
          }
        }
        /* Always attach a chroma descriptor for NV12 so HAL uses the exact
         * UV offset we pass rather than computing it from (Y stride × height),
         * which is only correct for unpadded layouts. */
        chroma = hal_plane_descriptor_new (uv_fd);
        if (chroma) {
          hal_plane_descriptor_set_stride (chroma, (size_t) uv_stride);
          if (chroma_offset > 0)
            hal_plane_descriptor_set_offset (chroma, chroma_offset);
        }
      }

      /* hal_import_image CONSUMES pd and chroma */
      hal_tensor *t = hal_import_image (self->processor,
          pd, chroma, (size_t) width, (size_t) height, pixel_fmt, HAL_DTYPE_U8);
      if (t)
        return t;

      GST_DEBUG_OBJECT (self, "hal_import_image failed for fd=%d", fd);
    }
  }

  /* Memcpy fallback: warn once so pipeline operators can identify
   * zero-copy regressions; subsequent occurrences at trace level. */
  static gboolean memcpy_warned = FALSE;
  if (G_UNLIKELY (!memcpy_warned)) {
    memcpy_warned = TRUE;
    GST_INFO_OBJECT (self, "Input is not DMABuf or hal_import_image failed; "
        "using memcpy path");
  } else {
    GST_LOG_OBJECT (self, "memcpy fallback: copying input frame");
  }

  /* System-memory path: allocate HAL image and copy frame data into it */
  guint64 t0_input = _get_time_ns ();
  hal_tensor *tensor = hal_image_processor_create_image (self->processor,
      (size_t) width, (size_t) height, pixel_fmt, HAL_DTYPE_U8);
  guint64 t1_input = _get_time_ns ();
  GST_INFO_OBJECT (self, "hal_image_processor_create_image (input %zux%zu fmt=%d) "
      "took %.1f ms", (size_t) width, (size_t) height, pixel_fmt,
      (t1_input - t0_input) / 1e6);
  if (!tensor)
    return NULL;

  struct hal_tensor_map *tmap = hal_tensor_map_create (tensor);
  if (!tmap) {
    hal_tensor_free (tensor);
    return NULL;
  }

  uint8_t *dst = (uint8_t *) hal_tensor_map_data (tmap);

  GstVideoFrame frame;
  if (!gst_video_frame_map (&frame, info, inbuf, GST_MAP_READ)) {
    hal_tensor_map_unmap (tmap);
    hal_tensor_free (tensor);
    return NULL;
  }

  if (pixel_fmt == HAL_PIXEL_FORMAT_NV12) {
    /* Y plane */
    const guint8 *y_data = (const guint8 *) GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    /* UV plane */
    const guint8 *uv_data = (const guint8 *) GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height / 2; y++)
      memcpy (uv_dst + y * width, uv_data + y * uv_stride, width);
  } else {
    const guint8 *src_data = (const guint8 *) GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
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
  return tensor;
}

/* ── Helper: overlay_set_video_caps ──────────────────────────────── */

static gboolean
overlay_set_video_caps (EdgefirstOverlay *self, GstCaps *caps)
{
  if (!self->processor) {
    GST_DEBUG_OBJECT (self,
        "CAPS before READY_TO_PAUSED — deferring until processor is ready");
    return FALSE;
  }

  GstVideoInfo info;
  if (!gst_video_info_from_caps (&info, caps)) {
    GST_ERROR_OBJECT (self, "Failed to parse video caps");
    return FALSE;
  }

  self->in_info       = info;
  self->in_info_valid = TRUE;
  self->src_pixel_format = gst_format_to_hal (GST_VIDEO_INFO_FORMAT (&info));
  self->display_w     = (guint) GST_VIDEO_INFO_WIDTH (&info);
  self->display_h     = (guint) GST_VIDEO_INFO_HEIGHT (&info);

  /* Apply class colors if configured */
  if (self->class_colors) {
    uint8_t (*colors)[4] = NULL;
    gsize n = parse_class_colors (self->class_colors, &colors);
    if (n > 0) {
      hal_image_processor_set_class_colors (self->processor, colors, n);
      GST_INFO_OBJECT (self, "set %zu class colors", n);
    }
    g_free (colors);
  }

  /* Allocate two display images for double-buffering.  The DMA-BUF clone path
   * shares physical memory with display_image — double-buffering lets frame N's
   * fd remain valid while frame N+1 is rendered into the other buffer. */
  for (int i = 0; i < 2; i++) {
    hal_tensor_free (self->display_images[i]);
    guint64 t0 = _get_time_ns ();
    self->display_images[i] = hal_image_processor_create_image (self->processor,
        (size_t) self->display_w, (size_t) self->display_h,
        HAL_PIXEL_FORMAT_RGBA, HAL_DTYPE_U8);
    guint64 t1 = _get_time_ns ();
    GST_INFO_OBJECT (self, "create_image buf[%d] %ux%u RGBA: %.1f ms",
        i, self->display_w, self->display_h, (t1 - t0) / 1e6);
    if (!self->display_images[i]) {
      GST_ERROR_OBJECT (self, "Failed to allocate display image[%d] (%ux%u)",
          i, self->display_w, self->display_h);
      return FALSE;
    }
  }
  self->display_buf_idx = 0;

  /* Probe DMA-BUF availability once; both images were allocated identically. */
  int test_fd = hal_tensor_dmabuf_clone (self->display_images[0]);
  if (test_fd >= 0) {
    close (test_fd);
    self->display_is_dmabuf = TRUE;
  } else {
    self->display_is_dmabuf = FALSE;
  }

  GST_INFO_OBJECT (self, "display image %ux%u RGBA, dmabuf=%s (double-buffered)",
      self->display_w, self->display_h,
      self->display_is_dmabuf ? "yes" : "no");

  /* Forward plain video/x-raw RGBA caps downstream with framerate from input.
   * Do NOT advertise memory:DMABuf in caps — DMA-BUF backing is transparent.
   * waylandsink (and other sinks) discover DMA-BUF at buffer-import time via
   * gst_is_dmabuf_memory(), not through caps negotiation. This matches the
   * pattern used by the working yolov8n_seg_ara2 appsrc → waylandsink path. */
  GstCaps *out_caps = gst_caps_new_simple ("video/x-raw",
      "format",    G_TYPE_STRING, "RGBA",
      "width",     G_TYPE_INT,    (gint) self->display_w,
      "height",    G_TYPE_INT,    (gint) self->display_h,
      "framerate", GST_TYPE_FRACTION,
                   GST_VIDEO_INFO_FPS_N (&info),
                   GST_VIDEO_INFO_FPS_D (&info),
      NULL);

  GstEvent *caps_event = gst_event_new_caps (out_caps);
  gst_caps_unref (out_caps);
  gst_pad_push_event (self->srcpad, caps_event);
  return TRUE;
}

/* ── Event handlers ──────────────────────────────────────────────── */

static gboolean
edgefirst_overlay_video_event (GstPad *pad, GstObject *parent, GstEvent *event)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (parent);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_CAPS: {
      GstCaps *caps;
      gst_event_parse_caps (event, &caps);
      gboolean ok = overlay_set_video_caps (self, caps);
      gst_event_unref (event);
      return ok;
    }
    case GST_EVENT_SEGMENT:
      return gst_pad_push_event (self->srcpad, event);
    case GST_EVENT_EOS:
    case GST_EVENT_FLUSH_START:
      g_mutex_lock (&self->lock);
      self->flushing = TRUE;
      g_cond_broadcast (&self->decode_cond);
      g_mutex_unlock (&self->lock);
      return gst_pad_push_event (self->srcpad, event);
    case GST_EVENT_FLUSH_STOP:
      g_mutex_lock (&self->lock);
      self->flushing = FALSE;
      g_mutex_unlock (&self->lock);
      return gst_pad_push_event (self->srcpad, event);
    default:
      return gst_pad_event_default (pad, parent, event);
  }
}

static gboolean
edgefirst_overlay_tensors_event (GstPad *pad G_GNUC_UNUSED,
    GstObject *parent, GstEvent *event)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (parent);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_CAPS: {
      GstCaps *caps;
      gst_event_parse_caps (event, &caps);
      /* Parse tensor shapes from caps; decoder creation deferred to
       * first buffer where quant meta is available. */
      if (!self->caps_parsed && !self->decoder)
        overlay_parse_tensor_caps (self, caps);
      gst_event_unref (event);
      return TRUE;
    }
    case GST_EVENT_EOS:
      /* Tensors EOS: wake up any model-sync waits so they can timeout and
       * emit frames, but do NOT set flushing — the video chain must continue
       * draining all video frames until the video EOS arrives. */
      g_mutex_lock (&self->lock);
      g_cond_broadcast (&self->decode_cond);
      g_mutex_unlock (&self->lock);
      gst_event_unref (event);
      return TRUE;
    case GST_EVENT_FLUSH_START:
      g_mutex_lock (&self->lock);
      self->flushing = TRUE;
      g_cond_broadcast (&self->decode_cond);
      g_mutex_unlock (&self->lock);
      gst_event_unref (event);
      return TRUE;
    case GST_EVENT_FLUSH_STOP:
      g_mutex_lock (&self->lock);
      self->flushing = FALSE;
      g_mutex_unlock (&self->lock);
      gst_event_unref (event);
      return TRUE;
    default:
      gst_event_unref (event);
      return TRUE;
  }
}

static gboolean
edgefirst_overlay_src_query (GstPad *pad, GstObject *parent, GstQuery *query)
{
  return gst_pad_query_default (pad, parent, query);
}

/* ── Video chain ─────────────────────────────────────────────────── */

static GstFlowReturn
edgefirst_overlay_video_chain (GstPad *pad G_GNUC_UNUSED,
    GstObject *parent, GstBuffer *buf)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (parent);

  if (!self->in_info_valid)
    return GST_FLOW_NOT_NEGOTIATED;

  /* ── model-sync wait ─────────────────────────────────────────── */
  if (self->model_sync) {
    GstClockTime pts = GST_BUFFER_PTS (buf);
    g_mutex_lock (&self->lock);
    if (pts != GST_CLOCK_TIME_NONE && self->decode_ts != GST_CLOCK_TIME_NONE) {
      if (self->model_sync_timeout_ms > 0) {
        gint64 deadline = g_get_monotonic_time () +
            (gint64) self->model_sync_timeout_ms * G_TIME_SPAN_MILLISECOND;
        while (self->decode_ts < pts && !self->flushing)
          if (!g_cond_wait_until (&self->decode_cond, &self->lock, deadline))
            break;
      } else {
        while (self->decode_ts < pts && !self->flushing)
          g_cond_wait (&self->decode_cond, &self->lock);
      }
    }
    if (self->flushing) {
      g_mutex_unlock (&self->lock);
      gst_buffer_unref (buf);
      return GST_FLOW_FLUSHING;
    }
  }

  /* Snapshot decoded results under lock.
   * When model_sync=TRUE the mutex is still held from the wait loop above;
   * when model_sync=FALSE we acquire it here. Either way we release it below. */
  EdgeFirstDetectBoxList    *boxes_snap = NULL;
  EdgeFirstSegmentationList *segs_snap  = NULL;
  {
    if (!self->model_sync) g_mutex_lock (&self->lock);
    boxes_snap = self->boxes_obj ? g_object_ref (self->boxes_obj) : NULL;
    segs_snap  = self->segs_obj  ? g_object_ref (self->segs_obj)  : NULL;
    g_mutex_unlock (&self->lock);
  }

  /* ── Convert input to RGBA display_image ─────────────────────── */
  /* Pick the current back buffer; front buffer (previous frame) is in flight. */
  hal_tensor *display_image = self->display_images[self->display_buf_idx];

  hal_tensor *src_img = create_input_image (self, buf);
  if (!src_img) {
    g_clear_object (&boxes_snap);
    g_clear_object (&segs_snap);
    gst_buffer_unref (buf);
    return GST_FLOW_ERROR;
  }

  guint64 t0_convert = _get_time_ns ();
  int convert_ret = hal_image_processor_convert (self->processor, src_img,
      display_image, HAL_ROTATION_NONE, HAL_FLIP_NONE, NULL);
  guint64 t1_convert = _get_time_ns ();
  GST_INFO_OBJECT (self, "hal_image_processor_convert took %.1f ms",
      (t1_convert - t0_convert) / 1e6);
  hal_tensor_free (src_img);

  if (convert_ret != 0) {
    GST_ERROR_OBJECT (self, "HAL convert failed (%d)", convert_ret);
    g_clear_object (&boxes_snap);
    g_clear_object (&segs_snap);
    gst_buffer_unref (buf);
    return GST_FLOW_ERROR;
  }

  /* ── Draw decoded masks ───────────────────────────────────────── */
  hal_detect_box_list    *boxes_hal = edgefirst_detect_box_list_get_hal (boxes_snap);
  hal_segmentation_list  *segs_hal  = edgefirst_segmentation_list_get_hal (segs_snap);

  if (boxes_hal || segs_hal) {
    guint64 t0_draw = _get_time_ns ();
    hal_image_processor_draw_decoded_masks (self->processor, display_image,
        boxes_hal, segs_hal, NULL, self->opacity,
        self->has_letterbox ? self->letterbox : NULL,
        (enum hal_color_mode) self->color_mode);
    guint64 t1_draw = _get_time_ns ();
    GST_INFO_OBJECT (self, "draw_decoded_masks: %.1f ms", (t1_draw - t0_draw) / 1e6);
  }

  g_clear_object (&boxes_snap);
  g_clear_object (&segs_snap);

  /* ── Wrap display_image as output GstBuffer ───────────────────── */
  GstBuffer *outbuf = NULL;
  guint w = self->display_w, h = self->display_h;
  gboolean dmabuf_ok = FALSE;

  /* HAL >= 0.16.3 pads the row stride to 64-byte alignment on Mali Valhall;
   * query the actual stride for correct DMA-BUF size and GstVideoMeta. */
  gsize row_stride = hal_tensor_row_stride (display_image);

  if (self->display_is_dmabuf) {
    int fd = hal_tensor_dmabuf_clone (display_image);
    if (fd >= 0) {
      outbuf = gst_buffer_new ();
      gst_buffer_append_memory (outbuf,
          gst_dmabuf_allocator_alloc (self->dmabuf_allocator, fd, row_stride * h));
      /* Advance to the other buffer so the next frame renders there while
       * downstream (waylandsink) holds the current frame's DMA-BUF fd. */
      self->display_buf_idx = (self->display_buf_idx + 1) % 2;
      dmabuf_ok = TRUE;
    }
  }

  if (!dmabuf_ok) {
    /* System-memory fallback: memcpy snapshot is safe with a single buffer
     * because the copy completes before the buffer is pushed downstream. */
    outbuf = gst_buffer_new_allocate (NULL, (gsize) w * h * 4, NULL);
    struct hal_tensor_map *tmap = hal_tensor_map_create (display_image);
    if (tmap) {
      GstMapInfo map;
      if (gst_buffer_map (outbuf, &map, GST_MAP_WRITE)) {
        const guint8 *src = (const guint8 *) hal_tensor_map_data_const (tmap);
        if (row_stride == (gsize) w * 4) {
          memcpy (map.data, src, (gsize) w * h * 4);
        } else {
          /* Strip per-row padding to produce a dense (w × 4) output buffer. */
          for (guint y = 0; y < h; y++)
            memcpy ((guint8 *) map.data + y * w * 4, src + y * row_stride, (gsize) w * 4);
        }
        gst_buffer_unmap (outbuf, &map);
      }
      hal_tensor_map_unmap (tmap);
    }
  }

  /* Attach GstVideoMeta so downstream elements (especially waylandsink) can
   * read stride/offset for DMA-BUF import without relying on caps.
   * DMA-BUF path: pass the actual (possibly padded) row stride.
   * System-memory path: output is dense — default stride (w × 4) is correct. */
  if (dmabuf_ok) {
    gsize offsets[1] = {0};
    gint  strides[1] = {(gint) row_stride};
    gst_buffer_add_video_meta_full (outbuf, GST_VIDEO_FRAME_FLAG_NONE,
        GST_VIDEO_FORMAT_RGBA, w, h, 1, offsets, strides);
  } else {
    gst_buffer_add_video_meta (outbuf, GST_VIDEO_FRAME_FLAG_NONE,
        GST_VIDEO_FORMAT_RGBA, w, h);
  }

  GST_BUFFER_PTS (outbuf)      = GST_BUFFER_PTS (buf);
  GST_BUFFER_DURATION (outbuf) = GST_BUFFER_DURATION (buf);

  gst_buffer_unref (buf);
  return gst_pad_push (self->srcpad, outbuf);
}

/* ── Helper: gst_memory_to_hal_tensor ────────────────────────────── */

/* Create a HAL tensor wrapping a GstMemory block.
 * Tries DMABuf zero-copy first; falls back to hal_tensor_new + memcpy.
 * shape/ndim are used for the memcpy fallback; for DMABuf, the fd carries
 * the data. */
static hal_tensor *
gst_memory_to_hal_tensor (GstMemory *mem,
    enum hal_dtype dtype, const size_t *shape, size_t ndim)
{
  hal_tensor *t = NULL;

  if (gst_is_dmabuf_memory (mem)) {
    int fd = gst_dmabuf_memory_get_fd (mem);
    t = hal_tensor_from_fd (dtype, fd, shape, ndim, NULL);
  }

  if (!t) {
    t = hal_tensor_new (dtype, shape, ndim, HAL_TENSOR_MEMORY_MEM, NULL);
    if (!t) return NULL;

    struct hal_tensor_map *tmap = hal_tensor_map_create (t);
    if (!tmap) { hal_tensor_free (t); return NULL; }

    GstMapInfo map;
    gst_memory_map (mem, &map, GST_MAP_READ);
    memcpy (hal_tensor_map_data (tmap), map.data, map.size);
    gst_memory_unmap (mem, &map);
    hal_tensor_map_unmap (tmap);
  }

  return t;
}

/* ── Tensors chain ────────────────────────────────────────────────── */

static GstFlowReturn
edgefirst_overlay_tensors_chain (GstPad *pad G_GNUC_UNUSED,
    GstObject *parent, GstBuffer *buf)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (parent);

  /* Create decoder on first buffer if caps were parsed but decoder not yet built.
   * This defers creation so we can extract GstNnsTensorQuantMeta from the buffer. */
  if (!self->decoder && self->caps_parsed) {
    overlay_create_decoder (self, buf);
  }

  if (!self->decoder) {
    gst_buffer_unref (buf);
    return GST_FLOW_OK;   /* pass-through: no decoder configured */
  }

  g_mutex_lock (&self->lock);
  if (self->flushing) {
    g_mutex_unlock (&self->lock);
    gst_buffer_unref (buf);
    return GST_FLOW_FLUSHING;
  }
  g_mutex_unlock (&self->lock);

  /* ── Wrap GstBuffer memories as HAL tensors ─────────────────────── */
  guint n_mem = gst_buffer_n_memory (buf);
  hal_tensor **outputs = g_new0 (hal_tensor *, n_mem);
  gboolean ok = TRUE;

  for (guint i = 0; i < n_mem; i++) {
    GstMemory *mem = gst_buffer_peek_memory (buf, i);

    /* Use HAL-convention shapes from auto-config if available.
     * These match the shapes registered with the decoder, so
     * find_outputs_with_shape will match correctly. */
    size_t  ndim;
    size_t  shape[8];
    enum hal_dtype dtype = HAL_DTYPE_F32;

    if (i < (guint) self->tensor_count && self->hal_ndims[i] > 0) {
      ndim  = self->hal_ndims[i];
      dtype = self->tensor_dtypes[i];
      for (size_t j = 0; j < ndim; j++)
        shape[j] = self->hal_shapes[i][j];
    } else {
      size_t byte_size = gst_memory_get_sizes (mem, NULL, NULL);
      shape[0] = byte_size;
      ndim = 1;
    }

    outputs[i] = gst_memory_to_hal_tensor (mem, dtype, shape, ndim);
    if (!outputs[i]) {
      GST_WARNING_OBJECT (self, "Failed to wrap tensor %u as HAL tensor", i);
      ok = FALSE;
      break;
    }
  }

  if (!ok) {
    for (guint i = 0; i < n_mem; i++) hal_tensor_free (outputs[i]);
    g_free (outputs);
    gst_buffer_unref (buf);
    return GST_FLOW_ERROR;
  }

  /* ── Decode ─────────────────────────────────────────────────────── */
  hal_detect_box_list *new_boxes = NULL;
  struct hal_proto_data *proto   = NULL;

  guint64 t0_decode = _get_time_ns ();
  proto = hal_decoder_decode_proto (self->decoder,
      (const struct hal_tensor *const *) outputs, n_mem, &new_boxes);
  guint64 t1_decode = _get_time_ns ();

  for (guint i = 0; i < n_mem; i++) hal_tensor_free (outputs[i]);
  g_free (outputs);

  if (!new_boxes) {
    if (proto) hal_proto_data_free (proto);
    gst_buffer_unref (buf);
    GST_INFO_OBJECT (self, "decode: %.1f ms (0 boxes, errno=%d)",
        (t1_decode - t0_decode) / 1e6, errno);
    errno = 0;
    return GST_FLOW_OK;   /* zero detections this frame */
  }

  guint n_boxes = (guint) hal_detect_box_list_len (new_boxes);
  GST_INFO_OBJECT (self, "decode: %.1f ms (%u boxes%s)",
      (t1_decode - t0_decode) / 1e6, n_boxes,
      proto ? ", has protos" : "");

  /* ── Materialize masks ───────────────────────────────────────────── */
  hal_segmentation_list *new_segs = NULL;
  if (proto) {
    guint64 t0_mat = _get_time_ns ();
    new_segs = hal_image_processor_materialize_masks (self->processor,
        new_boxes, proto, self->has_letterbox ? self->letterbox : NULL);
    guint64 t1_mat = _get_time_ns ();
    GST_INFO_OBJECT (self, "materialize_masks: %.1f ms (%s)",
        (t1_mat - t0_mat) / 1e6, new_segs ? "ok" : "none");
    hal_proto_data_free (proto);
  }

  /* ── Wrap in GObjects ────────────────────────────────────────────── */
  guint mw = self->model_width  > 0 ? self->model_width  : 0;
  guint mh = self->model_height > 0 ? self->model_height : 0;
  EdgeFirstDetectBoxList    *boxes_obj = edgefirst_detect_box_list_new_normalized (
      new_boxes, self->normalized, mw, mh);
  EdgeFirstSegmentationList *segs_obj  = new_segs
      ? edgefirst_segmentation_list_new (new_segs) : NULL;

  /* ── Update stored state under lock ─────────────────────────────── */
  g_mutex_lock (&self->lock);
  g_clear_object (&self->boxes_obj);
  g_clear_object (&self->segs_obj);
  self->boxes_obj = g_object_ref (boxes_obj);
  self->segs_obj  = segs_obj ? g_object_ref (segs_obj) : NULL;
  if (GST_BUFFER_PTS (buf) != GST_CLOCK_TIME_NONE)
    self->decode_ts = GST_BUFFER_PTS (buf);
  g_cond_signal (&self->decode_cond);
  g_mutex_unlock (&self->lock);

  /* ── Emit signal ─────────────────────────────────────────────────── */
  g_signal_emit (self, signals[SIGNAL_NEW_DETECTION], 0, boxes_obj, segs_obj);

  g_object_unref (boxes_obj);
  g_clear_object (&segs_obj);

  gst_buffer_unref (buf);
  return GST_FLOW_OK;
}
