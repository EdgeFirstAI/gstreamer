/*
 * EdgeFirst Perception for GStreamer - Zenoh Publisher Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstzenohpub.h"
#include "edgefirstzenoh-enums.h"
#include <gst/edgefirst/edgefirst.h>
#include <gst/video/video.h>
#include <edgefirst/schemas.h>
#include <zenoh.h>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_zenoh_pub_debug);
#define GST_CAT_DEFAULT edgefirst_zenoh_pub_debug

enum {
  PROP_0,
  PROP_TOPIC,
  PROP_MESSAGE_TYPE,
  PROP_SESSION,
  PROP_RELIABLE,
};

struct _EdgefirstZenohPub {
  GstBaseSink parent;

  /* Properties */
  gchar *topic;
  EdgefirstZenohPubMessageType message_type;
  gchar *session_config;
  gboolean reliable;

  /* Zenoh session and publisher handles */
  z_owned_session_t session;
  z_owned_publisher_t publisher;
  gboolean session_valid;
};

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (EDGEFIRST_POINTCLOUD2_CAPS "; "
        "other/tensors, num-tensors = (int) 1; "
        "video/x-raw")
    );

#define edgefirst_zenoh_pub_parent_class parent_class
G_DEFINE_TYPE (EdgefirstZenohPub, edgefirst_zenoh_pub, GST_TYPE_BASE_SINK);

static void edgefirst_zenoh_pub_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec);
static void edgefirst_zenoh_pub_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec);
static void edgefirst_zenoh_pub_finalize (GObject *object);

static gboolean edgefirst_zenoh_pub_start (GstBaseSink *sink);
static gboolean edgefirst_zenoh_pub_stop (GstBaseSink *sink);
static GstFlowReturn edgefirst_zenoh_pub_render (GstBaseSink *sink, GstBuffer *buffer);

static void
edgefirst_zenoh_pub_class_init (EdgefirstZenohPubClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);

  gobject_class->set_property = edgefirst_zenoh_pub_set_property;
  gobject_class->get_property = edgefirst_zenoh_pub_get_property;
  gobject_class->finalize = edgefirst_zenoh_pub_finalize;

  g_object_class_install_property (gobject_class, PROP_TOPIC,
      g_param_spec_string ("topic", "Topic",
          "Zenoh key expression to publish to",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_MESSAGE_TYPE,
      g_param_spec_enum ("message-type", "Message Type",
          "Type of message to publish",
          EDGEFIRST_TYPE_ZENOH_PUB_MESSAGE_TYPE,
          EDGEFIRST_ZENOH_PUB_POINTCLOUD2,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_SESSION,
      g_param_spec_string ("session", "Session",
          "Zenoh locator or path to configuration file",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_RELIABLE,
      g_param_spec_boolean ("reliable", "Reliable",
          "Use reliable QoS for message delivery",
          TRUE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_set_static_metadata (element_class,
      "EdgeFirst Zenoh Publisher",
      "Sink/Network",
      "Publish GStreamer buffers to Zenoh topics",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &sink_template);

  basesink_class->start = GST_DEBUG_FUNCPTR (edgefirst_zenoh_pub_start);
  basesink_class->stop = GST_DEBUG_FUNCPTR (edgefirst_zenoh_pub_stop);
  basesink_class->render = GST_DEBUG_FUNCPTR (edgefirst_zenoh_pub_render);

  GST_DEBUG_CATEGORY_INIT (edgefirst_zenoh_pub_debug, "edgefirstzenohpub", 0,
      "EdgeFirst Zenoh Publisher");
}

static void
edgefirst_zenoh_pub_init (EdgefirstZenohPub *self)
{
  self->topic = NULL;
  self->message_type = EDGEFIRST_ZENOH_PUB_POINTCLOUD2;
  self->session_config = NULL;
  self->reliable = TRUE;
  self->session_valid = FALSE;
}

static void
edgefirst_zenoh_pub_finalize (GObject *object)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (object);

  g_free (self->topic);
  g_free (self->session_config);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
edgefirst_zenoh_pub_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (object);

  switch (prop_id) {
    case PROP_TOPIC:
      g_free (self->topic);
      self->topic = g_value_dup_string (value);
      break;
    case PROP_MESSAGE_TYPE:
      self->message_type = g_value_get_enum (value);
      break;
    case PROP_SESSION:
      g_free (self->session_config);
      self->session_config = g_value_dup_string (value);
      break;
    case PROP_RELIABLE:
      self->reliable = g_value_get_boolean (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
edgefirst_zenoh_pub_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (object);

  switch (prop_id) {
    case PROP_TOPIC:
      g_value_set_string (value, self->topic);
      break;
    case PROP_MESSAGE_TYPE:
      g_value_set_enum (value, self->message_type);
      break;
    case PROP_SESSION:
      g_value_set_string (value, self->session_config);
      break;
    case PROP_RELIABLE:
      g_value_set_boolean (value, self->reliable);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* ── Zenoh session helpers ─────────────────────────────────────────── */

static gboolean
open_zenoh_session (EdgefirstZenohPub *self)
{
  z_owned_config_t config;

  if (self->session_config && g_file_test (self->session_config, G_FILE_TEST_EXISTS)) {
    if (zc_config_from_file (&config, self->session_config) != Z_OK) {
      GST_ERROR_OBJECT (self, "Failed to load Zenoh config from: %s",
          self->session_config);
      return FALSE;
    }
  } else if (self->session_config) {
    z_config_default (&config);
    zc_config_insert_json5 (z_loan_mut (config), "connect/endpoints",
        self->session_config);
  } else {
    z_config_default (&config);
  }

  if (z_open (&self->session, z_move (config), NULL) != Z_OK) {
    GST_ERROR_OBJECT (self, "Failed to open Zenoh session");
    return FALSE;
  }

  self->session_valid = TRUE;
  return TRUE;
}

/* ── Publish helpers ───────────────────────────────────────────────── */

static GstFlowReturn
publish_pointcloud2 (EdgefirstZenohPub *self, GstBuffer *buffer)
{
  RosPointCloud2 *pcd;
  EdgefirstPointCloud2Meta *meta;
  GstMapInfo map;
  GstCaps *caps;
  GstStructure *s;
  const gchar *fields_str;
  EdgefirstPointFieldDesc fields[32];
  guint num_fields;
  uint8_t *out_bytes = NULL;
  size_t out_len = 0;

  pcd = ros_point_cloud2_new ();
  if (!pcd)
    return GST_FLOW_ERROR;

  /* Get caps info */
  caps = gst_pad_get_current_caps (GST_BASE_SINK_PAD (self));
  if (caps) {
    s = gst_caps_get_structure (caps, 0);

    gint w = 0, h = 0, ps = 0;
    gboolean bigendian = FALSE, dense = FALSE;

    gst_structure_get_int (s, "width", &w);
    gst_structure_get_int (s, "height", &h);
    gst_structure_get_int (s, "point-step", &ps);
    gst_structure_get_boolean (s, "is-bigendian", &bigendian);
    gst_structure_get_boolean (s, "is-dense", &dense);

    ros_point_cloud2_set_width (pcd, (uint32_t) w);
    ros_point_cloud2_set_height (pcd, (uint32_t) h);
    ros_point_cloud2_set_point_step (pcd, (uint32_t) ps);
    ros_point_cloud2_set_row_step (pcd, (uint32_t) (w * ps));
    ros_point_cloud2_set_is_bigendian (pcd, bigendian);
    ros_point_cloud2_set_is_dense (pcd, dense);

    /* Add fields */
    fields_str = gst_structure_get_string (s, "fields");
    num_fields = edgefirst_parse_point_fields (fields_str, fields, 32);

    for (guint i = 0; i < num_fields; i++) {
      RosPointField *f = ros_point_field_new ();
      if (f) {
        ros_point_field_set_name (f, fields[i].name);
        ros_point_field_set_datatype (f, fields[i].datatype);
        ros_point_field_set_offset (f, fields[i].offset);
        ros_point_field_set_count (f, fields[i].count);
        ros_point_cloud2_add_field (pcd, f);
        ros_point_field_free (f);
      }
    }

    gst_caps_unref (caps);
  }

  /* Set header from meta */
  meta = edgefirst_buffer_get_pointcloud2_meta (buffer);
  if (meta) {
    RosHeader *hdr = ros_point_cloud2_get_header_mut (pcd);
    if (hdr && meta->frame_id[0] != '\0')
      ros_header_set_frame_id (hdr, meta->frame_id);
    if (hdr && meta->ros_timestamp_ns > 0) {
      RosTime *stamp = ros_header_get_stamp_mut (hdr);
      if (stamp) {
        ros_time_set_sec (stamp,
            (int32_t) (meta->ros_timestamp_ns / G_GUINT64_CONSTANT (1000000000)));
        ros_time_set_nanosec (stamp,
            (uint32_t) (meta->ros_timestamp_ns % G_GUINT64_CONSTANT (1000000000)));
      }
    }
  }

  /* Set point data */
  if (gst_buffer_map (buffer, &map, GST_MAP_READ)) {
    ros_point_cloud2_set_data (pcd, map.data, map.size);
    gst_buffer_unmap (buffer, &map);
  }

  /* Serialize and publish */
  if (ros_point_cloud2_serialize (pcd, &out_bytes, &out_len) == 0) {
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf (&payload, out_bytes, out_len);
    z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    free (out_bytes);
  } else {
    GST_WARNING_OBJECT (self, "Failed to serialize PointCloud2");
  }

  ros_point_cloud2_free (pcd);
  return GST_FLOW_OK;
}

static GstFlowReturn
publish_radarcube (EdgefirstZenohPub *self, GstBuffer *buffer)
{
  EdgeFirstRadarCube *cube;
  EdgefirstRadarCubeMeta *meta;
  GstMapInfo map;
  size_t required_size = 0;
  uint8_t *out_bytes = NULL;

  cube = edgefirst_radarcube_new ();
  if (!cube)
    return GST_FLOW_ERROR;

  meta = edgefirst_buffer_get_radar_cube_meta (buffer);
  if (meta) {
    /* Set layout */
    uint8_t layout[EDGEFIRST_RADAR_MAX_DIMS];
    for (guint8 i = 0; i < meta->num_dims; i++)
      layout[i] = (uint8_t) meta->layout[i];
    edgefirst_radarcube_set_layout (cube, layout, meta->num_dims);

    /* Set scales */
    edgefirst_radarcube_set_scales (cube, meta->scales, meta->num_dims);

    edgefirst_radarcube_set_is_complex (cube, meta->is_complex);
    edgefirst_radarcube_set_timestamp (cube, meta->radar_timestamp);

    /* Set header */
    RosHeader *hdr = edgefirst_radarcube_get_header_mut (cube);
    if (hdr && meta->frame_id[0] != '\0')
      ros_header_set_frame_id (hdr, meta->frame_id);
  }

  /* Set cube data */
  if (gst_buffer_map (buffer, &map, GST_MAP_READ)) {
    edgefirst_radarcube_set_cube (cube,
        (const int16_t *) map.data, map.size / sizeof (int16_t));
    gst_buffer_unmap (buffer, &map);
  }

  /* Serialize: query size first, then serialize */
  edgefirst_radarcube_serialize (cube, NULL, 0, &required_size);
  if (required_size > 0) {
    out_bytes = g_malloc (required_size);
    if (edgefirst_radarcube_serialize (cube, out_bytes, required_size, NULL) == 0) {
      z_owned_bytes_t payload;
      z_bytes_copy_from_buf (&payload, out_bytes, required_size);
      z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    } else {
      GST_WARNING_OBJECT (self, "Failed to serialize RadarCube");
    }
    g_free (out_bytes);
  }

  edgefirst_radarcube_free (cube);
  return GST_FLOW_OK;
}

static const char *
gst_format_to_ros_encoding (GstVideoFormat format)
{
  switch (format) {
    case GST_VIDEO_FORMAT_RGB:       return "rgb8";
    case GST_VIDEO_FORMAT_BGR:       return "bgr8";
    case GST_VIDEO_FORMAT_RGBA:      return "rgba8";
    case GST_VIDEO_FORMAT_BGRA:      return "bgra8";
    case GST_VIDEO_FORMAT_GRAY8:     return "mono8";
    case GST_VIDEO_FORMAT_GRAY16_LE: return "mono16";
    case GST_VIDEO_FORMAT_UYVY:      return "yuv422";
    default:
      g_warning ("Unsupported GstVideoFormat %d for ROS encoding", format);
      return NULL;
  }
}

static GstFlowReturn
publish_image (EdgefirstZenohPub *self, GstBuffer *buffer)
{
  RosImage *img;
  GstMapInfo map;
  GstCaps *caps;
  GstVideoInfo info;
  uint8_t *out_bytes = NULL;
  size_t out_len = 0;

  img = ros_image_new ();
  if (!img)
    return GST_FLOW_ERROR;

  caps = gst_pad_get_current_caps (GST_BASE_SINK_PAD (self));
  if (caps) {
    const char *encoding;

    gst_video_info_from_caps (&info, caps);
    encoding = gst_format_to_ros_encoding (GST_VIDEO_INFO_FORMAT (&info));
    if (!encoding) {
      GST_WARNING_OBJECT (self, "Unsupported video format for ROS encoding");
      gst_caps_unref (caps);
      ros_image_free (img);
      return GST_FLOW_ERROR;
    }

    ros_image_set_width (img, (uint32_t) GST_VIDEO_INFO_WIDTH (&info));
    ros_image_set_height (img, (uint32_t) GST_VIDEO_INFO_HEIGHT (&info));
    ros_image_set_step (img, (uint32_t) GST_VIDEO_INFO_WIDTH (&info)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&info, 0));
    ros_image_set_encoding (img, encoding);
    gst_caps_unref (caps);
  }

  if (gst_buffer_map (buffer, &map, GST_MAP_READ)) {
    ros_image_set_data (img, map.data, map.size);
    gst_buffer_unmap (buffer, &map);
  }

  if (ros_image_serialize (img, &out_bytes, &out_len) == 0) {
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf (&payload, out_bytes, out_len);
    z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    free (out_bytes);
  } else {
    GST_WARNING_OBJECT (self, "Failed to serialize Image");
  }

  ros_image_free (img);
  return GST_FLOW_OK;
}

/* ── Start / Stop / Render ─────────────────────────────────────────── */

static gboolean
edgefirst_zenoh_pub_start (GstBaseSink *sink)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (sink);
  z_view_keyexpr_t ke;

  if (!self->topic) {
    GST_ERROR_OBJECT (self, "No topic specified");
    return FALSE;
  }

  GST_INFO_OBJECT (self, "Starting Zenoh publisher on topic: %s", self->topic);

  if (!open_zenoh_session (self))
    return FALSE;

  z_view_keyexpr_from_str (&ke, self->topic);

  if (z_declare_publisher (z_loan (self->session), &self->publisher,
          z_loan (ke), NULL) != Z_OK) {
    GST_ERROR_OBJECT (self, "Failed to create publisher for: %s", self->topic);
    z_drop (z_move (self->session));
    self->session_valid = FALSE;
    return FALSE;
  }

  return TRUE;
}

static gboolean
edgefirst_zenoh_pub_stop (GstBaseSink *sink)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (sink);

  GST_INFO_OBJECT (self, "Stopping Zenoh publisher");

  z_drop (z_move (self->publisher));

  if (self->session_valid) {
    z_drop (z_move (self->session));
    self->session_valid = FALSE;
  }

  return TRUE;
}

static GstFlowReturn
edgefirst_zenoh_pub_render (GstBaseSink *sink, GstBuffer *buffer)
{
  EdgefirstZenohPub *self = EDGEFIRST_ZENOH_PUB (sink);

  GST_LOG_OBJECT (self, "Publishing buffer of size %" G_GSIZE_FORMAT,
      gst_buffer_get_size (buffer));

  switch (self->message_type) {
    case EDGEFIRST_ZENOH_PUB_POINTCLOUD2:
      return publish_pointcloud2 (self, buffer);
    case EDGEFIRST_ZENOH_PUB_RADARCUBE:
      return publish_radarcube (self, buffer);
    case EDGEFIRST_ZENOH_PUB_IMAGE:
      return publish_image (self, buffer);
    case EDGEFIRST_ZENOH_PUB_DMABUFFER:
      GST_WARNING_OBJECT (self, "DMA buffer publishing not yet implemented");
      return GST_FLOW_OK;
    default:
      GST_ERROR_OBJECT (self, "Unknown message type: %d", self->message_type);
      return GST_FLOW_ERROR;
  }
}
