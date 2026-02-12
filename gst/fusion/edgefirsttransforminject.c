/*
 * EdgeFirst Perception for GStreamer - Transform Inject Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * Attaches transform and/or camera calibration metadata to buffers.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirsttransforminject.h"
#include <gst/edgefirst/edgefirst.h>
#include <json-glib/json-glib.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_transform_inject_debug);
#define GST_CAT_DEFAULT edgefirst_transform_inject_debug

enum {
  PROP_0,
  PROP_CALIBRATION_FILE,
  PROP_FRAME_ID,
  PROP_PARENT_FRAME_ID,
};

struct _EdgefirstTransformInject {
  GstBaseTransform parent;

  /* Properties */
  gchar *calibration_file;
  gchar *frame_id;
  gchar *parent_frame_id;

  /* Loaded calibration data */
  gboolean has_camera_info;
  EdgefirstCameraInfoMeta camera_info;

  gboolean has_transform;
  EdgefirstTransformData transform;
};

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY
    );

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY
    );

#define edgefirst_transform_inject_parent_class parent_class
G_DEFINE_TYPE (EdgefirstTransformInject, edgefirst_transform_inject,
    GST_TYPE_BASE_TRANSFORM);

static void edgefirst_transform_inject_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec);
static void edgefirst_transform_inject_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec);
static void edgefirst_transform_inject_finalize (GObject *object);

static gboolean edgefirst_transform_inject_start (GstBaseTransform *trans);
static GstFlowReturn edgefirst_transform_inject_transform_ip (GstBaseTransform *trans,
    GstBuffer *buffer);

static void
edgefirst_transform_inject_class_init (EdgefirstTransformInjectClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS (klass);

  gobject_class->set_property = edgefirst_transform_inject_set_property;
  gobject_class->get_property = edgefirst_transform_inject_get_property;
  gobject_class->finalize = edgefirst_transform_inject_finalize;

  g_object_class_install_property (gobject_class, PROP_CALIBRATION_FILE,
      g_param_spec_string ("calibration-file", "Calibration File",
          "Path to YAML/JSON calibration file",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_FRAME_ID,
      g_param_spec_string ("frame-id", "Frame ID",
          "Coordinate frame identifier for this sensor",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_PARENT_FRAME_ID,
      g_param_spec_string ("parent-frame-id", "Parent Frame ID",
          "Reference/parent coordinate frame identifier",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_set_static_metadata (element_class,
      "EdgeFirst Transform Inject",
      "Filter/Metadata",
      "Attach transform and calibration metadata to buffers",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  trans_class->start = edgefirst_transform_inject_start;
  trans_class->transform_ip = edgefirst_transform_inject_transform_ip;
  trans_class->passthrough_on_same_caps = FALSE;

  GST_DEBUG_CATEGORY_INIT (edgefirst_transform_inject_debug, "edgefirsttransforminject", 0,
      "EdgeFirst Transform Inject");
}

static void
edgefirst_transform_inject_init (EdgefirstTransformInject *self)
{
  self->calibration_file = NULL;
  self->frame_id = NULL;
  self->parent_frame_id = NULL;
  self->has_camera_info = FALSE;
  self->has_transform = FALSE;

  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (self), TRUE);
}

static void
edgefirst_transform_inject_finalize (GObject *object)
{
  EdgefirstTransformInject *self = EDGEFIRST_TRANSFORM_INJECT (object);

  g_free (self->calibration_file);
  g_free (self->frame_id);
  g_free (self->parent_frame_id);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
edgefirst_transform_inject_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstTransformInject *self = EDGEFIRST_TRANSFORM_INJECT (object);

  switch (prop_id) {
    case PROP_CALIBRATION_FILE:
      g_free (self->calibration_file);
      self->calibration_file = g_value_dup_string (value);
      break;
    case PROP_FRAME_ID:
      g_free (self->frame_id);
      self->frame_id = g_value_dup_string (value);
      break;
    case PROP_PARENT_FRAME_ID:
      g_free (self->parent_frame_id);
      self->parent_frame_id = g_value_dup_string (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
edgefirst_transform_inject_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstTransformInject *self = EDGEFIRST_TRANSFORM_INJECT (object);

  switch (prop_id) {
    case PROP_CALIBRATION_FILE:
      g_value_set_string (value, self->calibration_file);
      break;
    case PROP_FRAME_ID:
      g_value_set_string (value, self->frame_id);
      break;
    case PROP_PARENT_FRAME_ID:
      g_value_set_string (value, self->parent_frame_id);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
json_array_to_double (JsonArray *arr, gdouble *out, guint count)
{
  for (guint i = 0; i < count && i < json_array_get_length (arr); i++)
    out[i] = json_array_get_double_element (arr, i);
}

static gboolean
parse_calibration_file (EdgefirstTransformInject *self, GError **error)
{
  JsonParser *parser;
  JsonNode *root_node;
  JsonObject *root;
  gboolean ret = FALSE;

  parser = json_parser_new ();
  if (!json_parser_load_from_file (parser, self->calibration_file, error))
    goto done;

  root_node = json_parser_get_root (parser);
  if (!root_node || !JSON_NODE_HOLDS_OBJECT (root_node)) {
    g_set_error (error, GST_CORE_ERROR, GST_CORE_ERROR_FAILED,
        "Calibration file root is not a JSON object");
    goto done;
  }

  root = json_node_get_object (root_node);

  /* Parse camera_info section */
  if (json_object_has_member (root, "camera_info")) {
    JsonObject *ci = json_object_get_object_member (root, "camera_info");
    EdgefirstCameraInfoMeta *meta = &self->camera_info;

    memset (meta, 0, sizeof (*meta));
    meta->width = (guint32) json_object_get_int_member (ci, "width");
    meta->height = (guint32) json_object_get_int_member (ci, "height");

    if (json_object_has_member (ci, "K"))
      json_array_to_double (json_object_get_array_member (ci, "K"), meta->K, 9);

    if (json_object_has_member (ci, "D")) {
      JsonArray *d_arr = json_object_get_array_member (ci, "D");
      guint n = json_array_get_length (d_arr);
      if (n > EDGEFIRST_MAX_DISTORTION_COEFFS)
        n = EDGEFIRST_MAX_DISTORTION_COEFFS;
      json_array_to_double (d_arr, meta->D, n);
      meta->num_distortion_coeffs = (guint8) n;
    }

    if (json_object_has_member (ci, "distortion_model")) {
      const gchar *model = json_object_get_string_member (ci, "distortion_model");
      if (g_strcmp0 (model, "plumb_bob") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_PLUMB_BOB;
      else if (g_strcmp0 (model, "equidistant") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_EQUIDISTANT;
      else if (g_strcmp0 (model, "rational_polynomial") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_RATIONAL;
      else
        meta->distortion_model = EDGEFIRST_DISTORTION_NONE;
    }

    if (json_object_has_member (ci, "R"))
      json_array_to_double (json_object_get_array_member (ci, "R"), meta->R, 9);

    if (json_object_has_member (ci, "P"))
      json_array_to_double (json_object_get_array_member (ci, "P"), meta->P, 12);

    self->has_camera_info = TRUE;
    GST_INFO_OBJECT (self, "Loaded camera_info: %ux%u", meta->width, meta->height);
  }

  /* Parse transform section */
  if (json_object_has_member (root, "transform")) {
    JsonObject *tf = json_object_get_object_member (root, "transform");
    EdgefirstTransformData *t = &self->transform;

    edgefirst_transform_data_set_identity (t);

    if (json_object_has_member (tf, "translation")) {
      JsonArray *arr = json_object_get_array_member (tf, "translation");
      json_array_to_double (arr, t->translation, 3);
    }

    if (json_object_has_member (tf, "rotation")) {
      JsonArray *arr = json_object_get_array_member (tf, "rotation");
      json_array_to_double (arr, t->rotation, 4);
    }

    if (json_object_has_member (tf, "child_frame_id")) {
      g_strlcpy (t->child_frame_id,
          json_object_get_string_member (tf, "child_frame_id"),
          EDGEFIRST_FRAME_ID_MAX_LEN);
    }

    if (json_object_has_member (tf, "parent_frame_id")) {
      g_strlcpy (t->parent_frame_id,
          json_object_get_string_member (tf, "parent_frame_id"),
          EDGEFIRST_FRAME_ID_MAX_LEN);
    }

    /* Property overrides */
    if (self->frame_id)
      g_strlcpy (t->child_frame_id, self->frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
    if (self->parent_frame_id)
      g_strlcpy (t->parent_frame_id, self->parent_frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);

    self->has_transform = TRUE;
    GST_INFO_OBJECT (self, "Loaded transform: %s -> %s",
        t->child_frame_id, t->parent_frame_id);
  }

  ret = TRUE;

done:
  g_object_unref (parser);
  return ret;
}

static gboolean
edgefirst_transform_inject_start (GstBaseTransform *trans)
{
  EdgefirstTransformInject *self = EDGEFIRST_TRANSFORM_INJECT (trans);

  if (self->calibration_file) {
    GError *error = NULL;

    GST_INFO_OBJECT (self, "Loading calibration from: %s", self->calibration_file);

    if (!parse_calibration_file (self, &error)) {
      GST_ELEMENT_ERROR (self, RESOURCE, READ,
          ("Failed to load calibration file: %s", self->calibration_file),
          ("%s", error ? error->message : "Unknown error"));
      g_clear_error (&error);
      return FALSE;
    }
  }

  return TRUE;
}

static GstFlowReturn
edgefirst_transform_inject_transform_ip (GstBaseTransform *trans,
    GstBuffer *buffer)
{
  EdgefirstTransformInject *self = EDGEFIRST_TRANSFORM_INJECT (trans);

  /* GstBaseTransform guarantees the buffer is writable when
   * passthrough_on_same_caps = FALSE and in_place = TRUE. */

  if (self->has_camera_info) {
    EdgefirstCameraInfoMeta *meta = edgefirst_buffer_add_camera_info_meta (buffer);
    if (meta) {
      *meta = self->camera_info;
      if (self->frame_id) {
        g_strlcpy (meta->frame_id, self->frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
      }
    }
  }

  if (self->has_transform) {
    EdgefirstTransformMeta *meta = edgefirst_buffer_add_transform_meta (buffer);
    if (meta) {
      meta->transform = self->transform;
      if (self->frame_id) {
        g_strlcpy (meta->transform.child_frame_id, self->frame_id,
            EDGEFIRST_FRAME_ID_MAX_LEN);
      }
      if (self->parent_frame_id) {
        g_strlcpy (meta->transform.parent_frame_id, self->parent_frame_id,
            EDGEFIRST_FRAME_ID_MAX_LEN);
      }
    }
  }

  return GST_FLOW_OK;
}
