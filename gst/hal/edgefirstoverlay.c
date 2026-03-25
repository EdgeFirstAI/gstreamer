/*
 * EdgeFirst Perception for GStreamer - Overlay Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPU-accelerated segmentation mask + bounding box overlay renderer.
 * Accepts video input, converts to RGBA via HAL image processor, and
 * draws detection/segmentation results using hal_image_processor_draw_masks().
 *
 * The NN branch calls edgefirst_overlay_set_results() to provide the
 * latest inference results; the display thread reads them each frame.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstoverlay.h"

#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include <string.h>
#include <unistd.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_overlay_debug);
#define GST_CAT_DEFAULT edgefirst_overlay_debug

/* ── Property IDs ────────────────────────────────────────────────── */

enum {
  PROP_0,
  PROP_CLASS_COLORS,
};

/* ── Instance struct ─────────────────────────────────────────────── */

struct _EdgefirstOverlay {
  GstBaseTransform parent;

  /* HAL state */
  hal_image_processor *processor;
  hal_tensor *display_image;         /* Persistent RGBA at display resolution */

  /* Input video info */
  GstVideoInfo in_info;
  gboolean in_info_valid;
  enum hal_pixel_format src_pixel_format;

  /* Thread-safe result storage (NN thread writes, display thread reads) */
  GMutex lock;
  hal_detect_box_list *boxes;
  hal_segmentation_list *segmentations;

  /* Properties */
  gchar *class_colors;
};

/* ── Pad templates ───────────────────────────────────────────────── */

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      "video/x-raw(memory:DMABuf), "
        "format={NV12, YUY2, RGB, RGBA, GRAY8}, "
        "width=[1,MAX], height=[1,MAX]; "
      "video/x-raw, "
        "format={NV12, YUY2, RGB, RGBA, GRAY8}, "
        "width=[1,MAX], height=[1,MAX]"
    ));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (
      "video/x-raw, format=RGBA, width=[1,MAX], height=[1,MAX]"
    ));

/* ── Type definition ─────────────────────────────────────────────── */

#define edgefirst_overlay_parent_class parent_class
G_DEFINE_TYPE (EdgefirstOverlay, edgefirst_overlay, GST_TYPE_BASE_TRANSFORM);

/* ── Forward declarations ────────────────────────────────────────── */

static void edgefirst_overlay_set_property (GObject *, guint,
    const GValue *, GParamSpec *);
static void edgefirst_overlay_get_property (GObject *, guint,
    GValue *, GParamSpec *);
static void edgefirst_overlay_finalize (GObject *);
static gboolean edgefirst_overlay_start (GstBaseTransform *);
static gboolean edgefirst_overlay_stop (GstBaseTransform *);
static GstCaps *edgefirst_overlay_transform_caps (GstBaseTransform *,
    GstPadDirection, GstCaps *, GstCaps *);
static gboolean edgefirst_overlay_set_caps (GstBaseTransform *,
    GstCaps *, GstCaps *);
static gboolean edgefirst_overlay_transform_size (GstBaseTransform *,
    GstPadDirection, GstCaps *, gsize, GstCaps *, gsize *);
static gboolean edgefirst_overlay_decide_allocation (GstBaseTransform *,
    GstQuery *);
static GstFlowReturn edgefirst_overlay_prepare_output_buffer (
    GstBaseTransform *, GstBuffer *, GstBuffer **);
static GstFlowReturn edgefirst_overlay_transform (GstBaseTransform *,
    GstBuffer *, GstBuffer *);

/* ── Helpers ─────────────────────────────────────────────────────── */

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

/**
 * Create a HAL image tensor from a GstBuffer.
 * Handles DMA-BUF zero-copy (via hal_import_image) and
 * system-memory (allocate via hal_image_processor_create_image + copy) paths.
 * Returns a newly-allocated tensor (caller must free).
 */
static hal_tensor *
create_input_image (EdgefirstOverlay *self, GstBuffer *inbuf)
{
  GstVideoInfo *info = &self->in_info;
  guint width = GST_VIDEO_INFO_WIDTH (info);
  guint height = GST_VIDEO_INFO_HEIGHT (info);
  enum hal_pixel_format pixel_fmt = self->src_pixel_format;

  /* Determine row_bytes for packed copy fallback */
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

  /* Try DMA-BUF zero-copy path */
  guint n_mem = gst_buffer_n_memory (inbuf);
  GstMemory *in_mem = gst_buffer_peek_memory (inbuf, 0);
  if (gst_is_dmabuf_memory (in_mem)) {
    int fd = gst_dmabuf_memory_get_fd (in_mem);
    struct hal_plane_descriptor *pd = hal_plane_descriptor_new (fd);
    if (pd) {
      gint stride = GST_VIDEO_INFO_PLANE_STRIDE (info, 0);
      hal_plane_descriptor_set_stride (pd, (size_t) stride);

      struct hal_plane_descriptor *chroma = NULL;
      if (n_mem >= 2 && pixel_fmt == HAL_PIXEL_FORMAT_NV12) {
        GstMemory *mem1 = gst_buffer_peek_memory (inbuf, 1);
        if (gst_is_dmabuf_memory (mem1)) {
          int uv_fd = gst_dmabuf_memory_get_fd (mem1);
          chroma = hal_plane_descriptor_new (uv_fd);
          if (chroma) {
            gint uv_stride = GST_VIDEO_INFO_PLANE_STRIDE (info, 1);
            hal_plane_descriptor_set_stride (chroma, (size_t) uv_stride);
          }
        }
      }

      /* hal_import_image CONSUMES pd and chroma */
      hal_tensor *t = hal_import_image (self->processor,
          pd, chroma, width, height, pixel_fmt, HAL_DTYPE_U8);
      if (t)
        return t;

      GST_DEBUG_OBJECT (self, "hal_import_image failed, falling back to copy");
    }
  }

  /* System-memory path: allocate HAL image and copy frame data into it */
  hal_tensor *tensor = hal_image_processor_create_image (self->processor,
      width, height, pixel_fmt, HAL_DTYPE_U8);
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

  if (pixel_fmt == HAL_PIXEL_FORMAT_NV12) {
    /* Y plane */
    const guint8 *y_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 0);
    gint y_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 0);
    for (guint y = 0; y < height; y++)
      memcpy (dst + y * width, y_data + y * y_stride, width);
    /* UV plane */
    const guint8 *uv_data = GST_VIDEO_FRAME_PLANE_DATA (&frame, 1);
    gint uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE (&frame, 1);
    uint8_t *uv_dst = dst + height * width;
    for (guint y = 0; y < height / 2; y++)
      memcpy (uv_dst + y * width, uv_data + y * uv_stride, width);
  } else {
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

  return tensor;
}

/**
 * Parse comma-separated RGBA hex color values into uint8_t arrays.
 * E.g. "FF0000FF,00FF00FF,0000FFFF" → {{255,0,0,255}, {0,255,0,255}, ...}
 * Returns number of colors parsed.  Caller must g_free the output.
 */
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

/* ── GObject lifecycle ───────────────────────────────────────────── */

static void
edgefirst_overlay_init (EdgefirstOverlay *self)
{
  self->in_info_valid = FALSE;
  self->class_colors = NULL;
  g_mutex_init (&self->lock);
  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (self), FALSE);
}

static void
edgefirst_overlay_finalize (GObject *object)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  g_clear_pointer (&self->display_image, hal_tensor_free);
  g_clear_pointer (&self->processor, hal_image_processor_free);
  g_clear_pointer (&self->boxes, hal_detect_box_list_free);
  g_clear_pointer (&self->segmentations, hal_segmentation_list_free);
  g_free (self->class_colors);
  g_mutex_clear (&self->lock);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

/* ── Properties ──────────────────────────────────────────────────── */

static void
edgefirst_overlay_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  switch (prop_id) {
    case PROP_CLASS_COLORS:
      g_free (self->class_colors);
      self->class_colors = g_value_dup_string (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
edgefirst_overlay_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (object);

  switch (prop_id) {
    case PROP_CLASS_COLORS:
      g_value_set_string (value, self->class_colors);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* ── start / stop ────────────────────────────────────────────────── */

static gboolean
edgefirst_overlay_start (GstBaseTransform *trans)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);

  self->processor = hal_image_processor_new ();
  if (!self->processor) {
    GST_ELEMENT_ERROR (self, LIBRARY, INIT,
        ("Failed to create HAL image processor"), (NULL));
    return FALSE;
  }

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

  GST_INFO_OBJECT (self, "HAL image processor created");
  return TRUE;
}

static gboolean
edgefirst_overlay_stop (GstBaseTransform *trans)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);

  g_clear_pointer (&self->display_image, hal_tensor_free);
  g_clear_pointer (&self->processor, hal_image_processor_free);
  self->in_info_valid = FALSE;

  g_mutex_lock (&self->lock);
  g_clear_pointer (&self->boxes, hal_detect_box_list_free);
  g_clear_pointer (&self->segmentations, hal_segmentation_list_free);
  g_mutex_unlock (&self->lock);

  return TRUE;
}

/* ── transform_caps ──────────────────────────────────────────────── */

static GstCaps *
edgefirst_overlay_transform_caps (GstBaseTransform *trans,
    GstPadDirection direction, GstCaps *caps, GstCaps *filter)
{
  GstCaps *result;

  if (direction == GST_PAD_SINK) {
    /* Sink → Src: always produce RGBA with matching dimensions/framerate */
    result = gst_caps_new_empty ();

    for (guint i = 0; i < gst_caps_get_size (caps); i++) {
      GstStructure *s = gst_caps_get_structure (caps, i);
      GstStructure *out = gst_structure_new ("video/x-raw",
          "format", G_TYPE_STRING, "RGBA", NULL);

      /* Copy width, height, framerate if present */
      const GValue *w = gst_structure_get_value (s, "width");
      const GValue *h = gst_structure_get_value (s, "height");
      const GValue *fr = gst_structure_get_value (s, "framerate");

      if (w) gst_structure_set_value (out, "width", w);
      if (h) gst_structure_set_value (out, "height", h);
      if (fr) gst_structure_set_value (out, "framerate", fr);

      gst_caps_append_structure (result, out);
    }
  } else {
    /* Src → Sink: accept any of our supported input formats */
    result = gst_static_pad_template_get_caps (&sink_template);
  }

  if (filter) {
    GstCaps *tmp = gst_caps_intersect_full (result, filter,
        GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (result);
    result = tmp;
  }

  GST_DEBUG_OBJECT (trans, "transform_caps %s: %" GST_PTR_FORMAT
      " → %" GST_PTR_FORMAT,
      direction == GST_PAD_SINK ? "sink→src" : "src→sink", caps, result);

  return result;
}

/* ── set_caps ────────────────────────────────────────────────────── */

static gboolean
edgefirst_overlay_set_caps (GstBaseTransform *trans, GstCaps *incaps,
    GstCaps *outcaps)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);

  if (!gst_video_info_from_caps (&self->in_info, incaps)) {
    GST_ERROR_OBJECT (self, "failed to parse input caps");
    return FALSE;
  }
  self->in_info_valid = TRUE;

  GstVideoFormat vfmt = GST_VIDEO_INFO_FORMAT (&self->in_info);
  self->src_pixel_format = gst_format_to_hal (vfmt);

  GST_INFO_OBJECT (self, "caps set: %dx%d %s → RGBA",
      GST_VIDEO_INFO_WIDTH (&self->in_info),
      GST_VIDEO_INFO_HEIGHT (&self->in_info),
      gst_video_format_to_string (vfmt));

  (void) outcaps;
  return TRUE;
}

/* ── transform_size ──────────────────────────────────────────────── */

static gboolean
edgefirst_overlay_transform_size (GstBaseTransform *trans,
    GstPadDirection direction, GstCaps *caps, gsize size,
    GstCaps *othercaps, gsize *othersize)
{
  if (direction == GST_PAD_SINK) {
    /* Output is RGBA: width * height * 4 */
    GstStructure *s = gst_caps_get_structure (othercaps, 0);
    gint w = 0, h = 0;
    gst_structure_get_int (s, "width", &w);
    gst_structure_get_int (s, "height", &h);
    if (w > 0 && h > 0) {
      *othersize = (gsize) w * h * 4;
      return TRUE;
    }
  }

  (void) trans;
  (void) caps;
  (void) size;
  return FALSE;
}

/* ── decide_allocation ───────────────────────────────────────────── */

static gboolean
edgefirst_overlay_decide_allocation (GstBaseTransform *trans, GstQuery *query)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);

  if (!self->in_info_valid) {
    GST_ERROR_OBJECT (self, "input caps not set before decide_allocation");
    return FALSE;
  }

  guint w = GST_VIDEO_INFO_WIDTH (&self->in_info);
  guint h = GST_VIDEO_INFO_HEIGHT (&self->in_info);

  /* Allocate persistent RGBA display image via processor */
  g_clear_pointer (&self->display_image, hal_tensor_free);
  self->display_image = hal_image_processor_create_image (self->processor,
      w, h, HAL_PIXEL_FORMAT_RGBA, HAL_DTYPE_U8);
  if (!self->display_image) {
    GST_ERROR_OBJECT (self, "failed to create RGBA display image (%ux%u)",
        w, h);
    return FALSE;
  }

  GST_INFO_OBJECT (self, "RGBA display image allocated (%ux%u, "
      "processor-selected backend)", w, h);

  /* Don't chain to parent — we handle output allocation in
   * prepare_output_buffer. */
  (void) query;
  return TRUE;
}

/* ── prepare_output_buffer ───────────────────────────────────────── */

static GstFlowReturn
edgefirst_overlay_prepare_output_buffer (GstBaseTransform *trans,
    GstBuffer *inbuf G_GNUC_UNUSED, GstBuffer **outbuf)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);
  guint w = GST_VIDEO_INFO_WIDTH (&self->in_info);
  guint h = GST_VIDEO_INFO_HEIGHT (&self->in_info);
  gsize size = (gsize) w * h * 4;

  *outbuf = gst_buffer_new_allocate (NULL, size, NULL);
  if (!*outbuf) {
    GST_ERROR_OBJECT (self, "failed to allocate output buffer (%zu bytes)",
        size);
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;
}

/* ── transform ───────────────────────────────────────────────────── */

static GstFlowReturn
edgefirst_overlay_transform (GstBaseTransform *trans,
    GstBuffer *inbuf, GstBuffer *outbuf)
{
  EdgefirstOverlay *self = EDGEFIRST_OVERLAY (trans);

  /* 1. Wrap input buffer as HAL tensor */
  hal_tensor *src_img = create_input_image (self, inbuf);
  if (!src_img) {
    GST_ERROR_OBJECT (self, "failed to create input image");
    return GST_FLOW_ERROR;
  }

  /* 2. GPU convert to RGBA at display resolution */
  int ret = hal_image_processor_convert (self->processor, src_img,
      self->display_image, HAL_ROTATION_NONE, HAL_FLIP_NONE, NULL);

  hal_tensor_free (src_img);

  if (ret != 0) {
    GST_ERROR_OBJECT (self, "HAL convert failed (%d)", ret);
    return GST_FLOW_ERROR;
  }

  /* 3. Draw overlays if results available */
  g_mutex_lock (&self->lock);
  hal_detect_box_list *boxes = self->boxes;
  hal_segmentation_list *segs = self->segmentations;

  if (boxes || segs) {
    ret = hal_image_processor_draw_masks (self->processor,
        self->display_image, boxes, segs);
    if (ret != 0) {
      GST_WARNING_OBJECT (self, "draw_masks failed (%d)", ret);
    }
  }
  g_mutex_unlock (&self->lock);

  /* 4. Copy RGBA display image data into output GstBuffer */
  hal_tensor_map *tmap = hal_tensor_map_create (self->display_image);
  if (!tmap) {
    GST_ERROR_OBJECT (self, "failed to map display image");
    return GST_FLOW_ERROR;
  }

  const uint8_t *rgba_data = hal_tensor_map_data_const (tmap);
  gsize rgba_size = (gsize) GST_VIDEO_INFO_WIDTH (&self->in_info) *
      GST_VIDEO_INFO_HEIGHT (&self->in_info) * 4;

  GstMapInfo omap;
  if (!gst_buffer_map (outbuf, &omap, GST_MAP_WRITE)) {
    hal_tensor_map_unmap (tmap);
    GST_ERROR_OBJECT (self, "failed to map output buffer");
    return GST_FLOW_ERROR;
  }

  memcpy (omap.data, rgba_data, rgba_size);
  gst_buffer_unmap (outbuf, &omap);
  hal_tensor_map_unmap (tmap);

  return GST_FLOW_OK;
}

/* ── Public API: set_results ─────────────────────────────────────── */

void
edgefirst_overlay_set_results (EdgefirstOverlay *overlay,
    hal_detect_box_list *boxes, hal_segmentation_list *segmentations)
{
  g_return_if_fail (EDGEFIRST_IS_OVERLAY (overlay));

  g_mutex_lock (&overlay->lock);

  g_clear_pointer (&overlay->boxes, hal_detect_box_list_free);
  g_clear_pointer (&overlay->segmentations, hal_segmentation_list_free);
  overlay->boxes = boxes;
  overlay->segmentations = segmentations;

  g_mutex_unlock (&overlay->lock);
}

/* ── class_init ──────────────────────────────────────────────────── */

static void
edgefirst_overlay_class_init (EdgefirstOverlayClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS (klass);

  gobject_class->set_property = edgefirst_overlay_set_property;
  gobject_class->get_property = edgefirst_overlay_get_property;
  gobject_class->finalize = edgefirst_overlay_finalize;

  /* Properties */
  g_object_class_install_property (gobject_class, PROP_CLASS_COLORS,
      g_param_spec_string ("class-colors", "Class Colors",
          "Comma-separated RGBA hex values for mask colors "
          "(e.g. \"FF0000FF,00FF00FF,0000FFFF\")",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /* Element metadata */
  gst_element_class_set_static_metadata (element_class,
      "EdgeFirst Overlay",
      "Filter/Effect/Video",
      "GPU-accelerated segmentation mask and bounding box overlay renderer",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  /* Virtual methods */
  trans_class->start = edgefirst_overlay_start;
  trans_class->stop = edgefirst_overlay_stop;
  trans_class->transform_caps = edgefirst_overlay_transform_caps;
  trans_class->set_caps = edgefirst_overlay_set_caps;
  trans_class->transform_size = edgefirst_overlay_transform_size;
  trans_class->decide_allocation = edgefirst_overlay_decide_allocation;
  trans_class->prepare_output_buffer = edgefirst_overlay_prepare_output_buffer;
  trans_class->transform = edgefirst_overlay_transform;

  trans_class->passthrough_on_same_caps = FALSE;

  GST_DEBUG_CATEGORY_INIT (edgefirst_overlay_debug,
      "edgefirstoverlay", 0, "EdgeFirst Overlay");
}
