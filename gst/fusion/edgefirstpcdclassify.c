/*
 * EdgeFirst Perception for GStreamer - Point Cloud Classify Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * Projects camera segmentation masks onto point clouds, assigning
 * semantic labels or colors to each point based on the corresponding pixel.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstpcdclassify.h"
#include <gst/edgefirst/edgefirst.h>
#include <gst/video/video.h>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_pcd_classify_debug);
#define GST_CAT_DEFAULT edgefirst_pcd_classify_debug

enum {
  PROP_0,
  PROP_OUTPUT_MODE,
};

struct _EdgefirstPcdClassify {
  GstAggregator parent;

  /* Properties */
  EdgefirstPcdClassifyOutputMode output_mode;

  /* Pad references */
  GstAggregatorPad *cloud_pad;
  GstAggregatorPad *mask_pad;
};

static GstStaticPadTemplate cloud_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink_cloud",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (EDGEFIRST_POINTCLOUD2_CAPS)
    );

static GstStaticPadTemplate mask_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink_mask",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw, format = (string) GRAY8")
    );

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (EDGEFIRST_POINTCLOUD2_CAPS)
    );

#define edgefirst_pcd_classify_parent_class parent_class
G_DEFINE_TYPE (EdgefirstPcdClassify, edgefirst_pcd_classify, GST_TYPE_AGGREGATOR);

static void edgefirst_pcd_classify_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec);
static void edgefirst_pcd_classify_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec);
static void edgefirst_pcd_classify_finalize (GObject *object);

static GstFlowReturn edgefirst_pcd_classify_aggregate (GstAggregator *agg,
    gboolean timeout);

GType
edgefirst_pcd_classify_output_mode_get_type (void)
{
  static GType type = 0;

  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS,
        "EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS", "labels" },
      { EDGEFIRST_PCD_CLASSIFY_OUTPUT_COLORS,
        "EDGEFIRST_PCD_CLASSIFY_OUTPUT_COLORS", "colors" },
      { EDGEFIRST_PCD_CLASSIFY_OUTPUT_BOTH,
        "EDGEFIRST_PCD_CLASSIFY_OUTPUT_BOTH", "both" },
      { 0, NULL, NULL },
    };
    GType _type = g_enum_register_static ("EdgefirstPcdClassifyOutputMode",
        values);
    g_once_init_leave (&type, _type);
  }
  return type;
}

static void
edgefirst_pcd_classify_class_init (EdgefirstPcdClassifyClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstAggregatorClass *agg_class = GST_AGGREGATOR_CLASS (klass);

  gobject_class->set_property = edgefirst_pcd_classify_set_property;
  gobject_class->get_property = edgefirst_pcd_classify_get_property;
  gobject_class->finalize = edgefirst_pcd_classify_finalize;

  g_object_class_install_property (gobject_class, PROP_OUTPUT_MODE,
      g_param_spec_enum ("output-mode", "Output Mode",
          "Classification output mode",
          EDGEFIRST_TYPE_PCD_CLASSIFY_OUTPUT_MODE,
          EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_set_static_metadata (element_class,
      "EdgeFirst Point Cloud Classify",
      "Filter/Video",
      "Project segmentation masks onto point clouds",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &cloud_sink_template);
  gst_element_class_add_static_pad_template (element_class, &mask_sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  agg_class->aggregate = edgefirst_pcd_classify_aggregate;

  GST_DEBUG_CATEGORY_INIT (edgefirst_pcd_classify_debug, "edgefirstpcdclassify", 0,
      "EdgeFirst Point Cloud Classify");
}

static void
edgefirst_pcd_classify_init (EdgefirstPcdClassify *self)
{
  self->output_mode = EDGEFIRST_PCD_CLASSIFY_OUTPUT_LABELS;
  self->cloud_pad = NULL;
  self->mask_pad = NULL;
}

static void
edgefirst_pcd_classify_finalize (GObject *object)
{
  EdgefirstPcdClassify *self = EDGEFIRST_PCD_CLASSIFY (object);

  gst_clear_object (&self->cloud_pad);
  gst_clear_object (&self->mask_pad);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
edgefirst_pcd_classify_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstPcdClassify *self = EDGEFIRST_PCD_CLASSIFY (object);

  switch (prop_id) {
    case PROP_OUTPUT_MODE:
      self->output_mode = g_value_get_enum (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
edgefirst_pcd_classify_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstPcdClassify *self = EDGEFIRST_PCD_CLASSIFY (object);

  switch (prop_id) {
    case PROP_OUTPUT_MODE:
      g_value_set_enum (value, self->output_mode);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
ensure_pad_refs (EdgefirstPcdClassify *self)
{
  if (!self->cloud_pad) {
    self->cloud_pad = GST_AGGREGATOR_PAD (
        gst_element_get_static_pad (GST_ELEMENT (self), "sink_cloud"));
  }
  if (!self->mask_pad) {
    self->mask_pad = GST_AGGREGATOR_PAD (
        gst_element_get_static_pad (GST_ELEMENT (self), "sink_mask"));
  }
}

static GstFlowReturn
edgefirst_pcd_classify_aggregate (GstAggregator *agg, gboolean timeout)
{
  EdgefirstPcdClassify *self = EDGEFIRST_PCD_CLASSIFY (agg);
  GstBuffer *cloud_buf = NULL;
  GstBuffer *mask_buf = NULL;
  GstBuffer *out_buf = NULL;
  EdgefirstCameraInfoMeta *cam_meta;
  EdgefirstTransformMeta *tf_meta;
  EdgefirstPointCloud2Meta *pcd_meta;
  GstMapInfo cloud_map, mask_map, out_map;
  GstCaps *cloud_caps;
  GstStructure *s;
  const gchar *fields_str;
  EdgefirstPointFieldDesc fields[32];
  guint num_fields;
  gint point_step, cloud_width, cloud_height;
  gint x_off = -1, y_off = -1, z_off = -1;
  guint32 point_count;
  gint new_point_step;
  GstFlowReturn ret = GST_FLOW_OK;

  (void) timeout;
  ensure_pad_refs (self);

  /* Pop buffers from both pads */
  cloud_buf = gst_aggregator_pad_pop_buffer (self->cloud_pad);
  mask_buf = gst_aggregator_pad_pop_buffer (self->mask_pad);

  if (!cloud_buf) {
    if (gst_aggregator_pad_is_eos (self->cloud_pad)) {
      if (mask_buf)
        gst_buffer_unref (mask_buf);
      return GST_FLOW_EOS;
    }
    if (mask_buf)
      gst_buffer_unref (mask_buf);
    return GST_FLOW_OK;
  }

  if (!mask_buf) {
    gst_buffer_unref (cloud_buf);
    if (gst_aggregator_pad_is_eos (self->mask_pad))
      return GST_FLOW_EOS;
    return GST_FLOW_OK;
  }

  /* Get metadata */
  cam_meta = edgefirst_buffer_get_camera_info_meta (mask_buf);
  tf_meta = edgefirst_buffer_get_transform_meta (cloud_buf);
  pcd_meta = edgefirst_buffer_get_pointcloud2_meta (cloud_buf);

  if (!cam_meta) {
    GST_WARNING_OBJECT (self, "Mask buffer missing CameraInfoMeta, passing cloud through");
    gst_aggregator_finish_buffer (agg, cloud_buf);
    gst_buffer_unref (mask_buf);
    return GST_FLOW_OK;
  }

  /* Parse cloud caps */
  cloud_caps = gst_pad_get_current_caps (GST_PAD (self->cloud_pad));
  if (!cloud_caps) {
    GST_WARNING_OBJECT (self, "No caps on cloud pad");
    gst_buffer_unref (cloud_buf);
    gst_buffer_unref (mask_buf);
    return GST_FLOW_ERROR;
  }

  s = gst_caps_get_structure (cloud_caps, 0);
  gst_structure_get_int (s, "point-step", &point_step);
  gst_structure_get_int (s, "width", &cloud_width);
  gst_structure_get_int (s, "height", &cloud_height);
  fields_str = gst_structure_get_string (s, "fields");

  num_fields = edgefirst_parse_point_fields (fields_str, fields, 32);

  /* Find x, y, z offsets */
  for (guint i = 0; i < num_fields; i++) {
    if (g_strcmp0 (fields[i].name, "x") == 0) x_off = (gint) fields[i].offset;
    else if (g_strcmp0 (fields[i].name, "y") == 0) y_off = (gint) fields[i].offset;
    else if (g_strcmp0 (fields[i].name, "z") == 0) z_off = (gint) fields[i].offset;
  }

  if (x_off < 0 || y_off < 0 || z_off < 0) {
    GST_WARNING_OBJECT (self, "Point cloud missing x/y/z fields");
    gst_caps_unref (cloud_caps);
    gst_buffer_unref (cloud_buf);
    gst_buffer_unref (mask_buf);
    return GST_FLOW_ERROR;
  }

  point_count = (guint32) cloud_width * (guint32) cloud_height;
  if (pcd_meta && pcd_meta->point_count > 0)
    point_count = pcd_meta->point_count;

  /* Output: original point data + 1 label byte per point */
  new_point_step = point_step + 1;
  out_buf = gst_buffer_new_allocate (NULL,
      (gsize) point_count * new_point_step, NULL);

  if (!gst_buffer_map (cloud_buf, &cloud_map, GST_MAP_READ)) {
    GST_ERROR_OBJECT (self, "Failed to map cloud buffer");
    gst_buffer_unref (cloud_buf);
    gst_buffer_unref (mask_buf);
    gst_buffer_unref (out_buf);
    gst_caps_unref (cloud_caps);
    return GST_FLOW_ERROR;
  }

  if (!gst_buffer_map (mask_buf, &mask_map, GST_MAP_READ)) {
    GST_ERROR_OBJECT (self, "Failed to map mask buffer");
    gst_buffer_unmap (cloud_buf, &cloud_map);
    gst_buffer_unref (cloud_buf);
    gst_buffer_unref (mask_buf);
    gst_buffer_unref (out_buf);
    gst_caps_unref (cloud_caps);
    return GST_FLOW_ERROR;
  }

  if (!gst_buffer_map (out_buf, &out_map, GST_MAP_WRITE)) {
    GST_ERROR_OBJECT (self, "Failed to map output buffer");
    gst_buffer_unmap (cloud_buf, &cloud_map);
    gst_buffer_unmap (mask_buf, &mask_map);
    gst_buffer_unref (cloud_buf);
    gst_buffer_unref (mask_buf);
    gst_buffer_unref (out_buf);
    gst_caps_unref (cloud_caps);
    return GST_FLOW_ERROR;
  }

  /* Validate cloud buffer size */
  {
    gsize expected_cloud_size = (gsize) point_count * point_step;
    if (expected_cloud_size > cloud_map.size) {
      GST_WARNING_OBJECT (self, "Point cloud buffer too small: expected %"
          G_GSIZE_FORMAT " got %" G_GSIZE_FORMAT,
          expected_cloud_size, cloud_map.size);
      point_count = (guint32) (cloud_map.size / point_step);
    }
  }

  /* Classify each point */
  for (guint32 i = 0; i < point_count; i++) {
    const guint8 *src_point = cloud_map.data + (gsize) i * point_step;
    guint8 *dst_point = out_map.data + (gsize) i * new_point_step;
    gfloat fx, fy, fz;
    gdouble x, y, z, u, v;
    guint8 label = 0;

    /* Copy original point data */
    memcpy (dst_point, src_point, point_step);

    /* Read xyz as float */
    memcpy (&fx, src_point + x_off, sizeof (gfloat));
    memcpy (&fy, src_point + y_off, sizeof (gfloat));
    memcpy (&fz, src_point + z_off, sizeof (gfloat));

    x = (gdouble) fx;
    y = (gdouble) fy;
    z = (gdouble) fz;

    /* Apply transform if available */
    if (tf_meta)
      edgefirst_transform_data_apply (&tf_meta->transform, &x, &y, &z);

    /* Project to image plane */
    if (edgefirst_camera_info_meta_project_point (cam_meta, x, y, z, &u, &v)) {
      gint px = (gint) (u + 0.5);
      gint py = (gint) (v + 0.5);

      if (px >= 0 && px < (gint) cam_meta->width &&
          py >= 0 && py < (gint) cam_meta->height) {
        gsize mask_offset = (gsize) py * cam_meta->width + (gsize) px;
        if (mask_offset < mask_map.size)
          label = mask_map.data[mask_offset];
      }
    }

    dst_point[point_step] = label;
  }

  gst_buffer_unmap (cloud_buf, &cloud_map);
  gst_buffer_unmap (mask_buf, &mask_map);
  gst_buffer_unmap (out_buf, &out_map);

  /* Copy metadata from cloud buffer to output */
  gst_buffer_copy_into (out_buf, cloud_buf,
      GST_BUFFER_COPY_FLAGS | GST_BUFFER_COPY_TIMESTAMPS | GST_BUFFER_COPY_META,
      0, -1);

  /* Update output caps with label field */
  {
    EdgefirstPointFieldDesc out_fields[33];
    gchar *new_fields_str;
    GstCaps *out_caps;

    memcpy (out_fields, fields, sizeof (EdgefirstPointFieldDesc) * num_fields);

    g_strlcpy (out_fields[num_fields].name, "label", 64);
    out_fields[num_fields].datatype = EDGEFIRST_POINT_FIELD_UINT8;
    out_fields[num_fields].offset = (guint32) point_step;
    out_fields[num_fields].count = 1;

    new_fields_str = edgefirst_format_point_fields (out_fields, num_fields + 1);

    out_caps = gst_caps_new_simple ("application/x-pointcloud2",
        "width", G_TYPE_INT, cloud_width,
        "height", G_TYPE_INT, cloud_height,
        "point-step", G_TYPE_INT, new_point_step,
        "fields", G_TYPE_STRING, new_fields_str,
        "is-bigendian", G_TYPE_BOOLEAN, FALSE,
        "is-dense", G_TYPE_BOOLEAN, TRUE,
        NULL);

    gst_aggregator_set_src_caps (agg, out_caps);
    gst_caps_unref (out_caps);
    g_free (new_fields_str);
  }

  gst_caps_unref (cloud_caps);
  gst_buffer_unref (cloud_buf);
  gst_buffer_unref (mask_buf);

  ret = gst_aggregator_finish_buffer (agg, out_buf);

  return ret;
}
