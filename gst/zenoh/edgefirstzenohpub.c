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

/* ── CDR little-endian encoding helpers ────────────────────────────── */

static inline void
cdr_pad_to (GByteArray *b, size_t align)
{
  static const guint8 zeros[8] = { 0 };
  size_t pad = (align - (b->len % align)) % align;
  if (pad)
    g_byte_array_append (b, zeros, (guint) pad);
}

static inline void
cdr_write_u8 (GByteArray *b, guint8 v)
{
  g_byte_array_append (b, &v, 1);
}

static inline void
cdr_write_i32 (GByteArray *b, gint32 v)
{
  cdr_pad_to (b, 4);
  g_byte_array_append (b, (const guint8 *) &v, 4);
}

static inline void
cdr_write_u32 (GByteArray *b, guint32 v)
{
  cdr_pad_to (b, 4);
  g_byte_array_append (b, (const guint8 *) &v, 4);
}

static inline void
cdr_write_u64 (GByteArray *b, guint64 v)
{
  cdr_pad_to (b, 8);
  g_byte_array_append (b, (const guint8 *) &v, 8);
}

static inline void
cdr_write_string (GByteArray *b, const char *s)
{
  guint32 len = (guint32) strlen (s ? s : "") + 1;   /* includes null */
  cdr_write_u32 (b, len);
  g_byte_array_append (b, (const guint8 *) (s ? s : ""), len);
}

/* ── Publish helpers ───────────────────────────────────────────────── */

/* Encode sensor_msgs/PointCloud2 to CDR little-endian.
 * Returns allocated bytes (free with g_free); sets *out_len. */
static guint8 *
encode_pointcloud2_cdr (int32_t stamp_sec, uint32_t stamp_nanosec,
                        const char *frame_id,
                        uint32_t height, uint32_t width,
                        uint32_t point_step, uint32_t row_step,
                        const EdgefirstPointFieldDesc *fields, guint num_fields,
                        gboolean is_bigendian, gboolean is_dense,
                        const uint8_t *data, size_t data_len,
                        size_t *out_len)
{
  static const guint8 cdr_le_header[4] = { 0x00, 0x01, 0x00, 0x00 };
  GByteArray *b = g_byte_array_new ();

  g_byte_array_append (b, cdr_le_header, 4);

  /* Header.stamp + frame_id */
  cdr_write_i32 (b, stamp_sec);
  cdr_write_u32 (b, stamp_nanosec);
  cdr_write_string (b, frame_id);

  cdr_write_u32 (b, height);
  cdr_write_u32 (b, width);

  /* fields: sequence<PointField> */
  cdr_write_u32 (b, num_fields);
  for (guint i = 0; i < num_fields; i++) {
    cdr_write_string (b, fields[i].name);
    cdr_write_u32 (b, fields[i].offset);
    cdr_write_u8 (b, (guint8) fields[i].datatype);
    cdr_write_u32 (b, fields[i].count);
  }

  cdr_write_u8 (b, is_bigendian ? 1 : 0);
  cdr_write_u32 (b, point_step);
  cdr_write_u32 (b, row_step);

  /* data: sequence<uint8> */
  cdr_write_u32 (b, (guint32) data_len);
  if (data_len)
    g_byte_array_append (b, data, (guint) data_len);

  cdr_write_u8 (b, is_dense ? 1 : 0);

  *out_len = b->len;
  return g_byte_array_free (b, FALSE);
}

/* Encode edgefirst_msgs/RadarCube to CDR little-endian.
 * Field order: Header (stamp, frame_id), timestamp, layout[], cube[], is_complex.
 * Returns allocated bytes (free with g_free); sets *out_len. */
static guint8 *
encode_radarcube_cdr (int32_t stamp_sec, uint32_t stamp_nanosec,
                      const char *frame_id,
                      guint64 timestamp,
                      const guint8 *layout, guint layout_len,
                      const gint16 *cube, guint cube_len,
                      gboolean is_complex,
                      size_t *out_len)
{
  static const guint8 cdr_le_header[4] = { 0x00, 0x01, 0x00, 0x00 };
  GByteArray *b = g_byte_array_new ();

  g_byte_array_append (b, cdr_le_header, 4);

  /* Header */
  cdr_write_i32 (b, stamp_sec);
  cdr_write_u32 (b, stamp_nanosec);
  cdr_write_string (b, frame_id);

  /* timestamp (uint64, 8-byte aligned) */
  cdr_write_u64 (b, timestamp);

  /* layout: sequence<uint8> */
  cdr_write_u32 (b, layout_len);
  if (layout_len)
    g_byte_array_append (b, layout, layout_len);

  /* cube: sequence<int16> (element count + int16 bytes) */
  cdr_write_u32 (b, cube_len);
  if (cube_len)
    g_byte_array_append (b, (const guint8 *) cube, cube_len * sizeof (gint16));

  /* is_complex */
  cdr_write_u8 (b, is_complex ? 1 : 0);

  *out_len = b->len;
  return g_byte_array_free (b, FALSE);
}

static GstFlowReturn
publish_pointcloud2 (EdgefirstZenohPub *self, GstBuffer *buffer)
{
  EdgefirstPointCloud2Meta *meta;
  GstMapInfo map;
  GstCaps *caps;
  GstStructure *s;
  const gchar *fields_str = NULL;
  EdgefirstPointFieldDesc fields[32];
  guint num_fields = 0;
  guint8 *out_bytes = NULL;
  size_t out_len = 0;
  int32_t stamp_sec = 0;
  uint32_t stamp_nanosec = 0;
  const gchar *frame_id = "";
  gint w = 0, h = 0, ps = 0;
  gboolean bigendian = FALSE, dense = FALSE;

  /* Get caps info */
  caps = gst_pad_get_current_caps (GST_BASE_SINK_PAD (self));
  if (caps) {
    s = gst_caps_get_structure (caps, 0);
    gst_structure_get_int (s, "width", &w);
    gst_structure_get_int (s, "height", &h);
    gst_structure_get_int (s, "point-step", &ps);
    gst_structure_get_boolean (s, "is-bigendian", &bigendian);
    gst_structure_get_boolean (s, "is-dense", &dense);
    fields_str = gst_structure_get_string (s, "fields");
    num_fields = edgefirst_parse_point_fields (fields_str, fields, 32);
    gst_caps_unref (caps);
  }

  /* Get header from meta */
  meta = edgefirst_buffer_get_pointcloud2_meta (buffer);
  if (meta) {
    if (meta->frame_id[0] != '\0')
      frame_id = meta->frame_id;
    if (meta->ros_timestamp_ns > 0) {
      stamp_sec = (int32_t) (meta->ros_timestamp_ns / G_GUINT64_CONSTANT (1000000000));
      stamp_nanosec = (uint32_t) (meta->ros_timestamp_ns % G_GUINT64_CONSTANT (1000000000));
    }
  }

  if (!gst_buffer_map (buffer, &map, GST_MAP_READ))
    return GST_FLOW_ERROR;

  out_bytes = encode_pointcloud2_cdr (stamp_sec, stamp_nanosec, frame_id,
      (uint32_t) h, (uint32_t) w, (uint32_t) ps,
      (uint32_t) (w * ps), fields, num_fields,
      bigendian, dense, map.data, map.size, &out_len);

  gst_buffer_unmap (buffer, &map);

  if (out_bytes) {
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf (&payload, out_bytes, out_len);
    z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    g_free (out_bytes);
  } else {
    GST_WARNING_OBJECT (self, "Failed to encode PointCloud2");
  }

  return GST_FLOW_OK;
}

static GstFlowReturn
publish_radarcube (EdgefirstZenohPub *self, GstBuffer *buffer)
{
  EdgefirstRadarCubeMeta *meta;
  GstMapInfo map;
  guint8 *out_bytes = NULL;
  size_t out_len = 0;
  guint8 layout[EDGEFIRST_RADAR_MAX_DIMS];
  guint layout_len = 0;
  guint64 timestamp = 0;
  const gchar *frame_id = "";
  gboolean is_complex = FALSE;

  meta = edgefirst_buffer_get_radar_cube_meta (buffer);
  if (meta) {
    layout_len = meta->num_dims;
    for (guint8 i = 0; i < meta->num_dims; i++)
      layout[i] = (guint8) meta->layout[i];
    timestamp = meta->radar_timestamp;
    is_complex = meta->is_complex;
    if (meta->frame_id[0] != '\0')
      frame_id = meta->frame_id;
  }

  if (!gst_buffer_map (buffer, &map, GST_MAP_READ))
    return GST_FLOW_ERROR;

  out_bytes = encode_radarcube_cdr (0, 0, frame_id, timestamp,
      layout, layout_len,
      (const gint16 *) map.data, (guint) (map.size / sizeof (gint16)),
      is_complex, &out_len);

  gst_buffer_unmap (buffer, &map);

  if (out_bytes) {
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf (&payload, out_bytes, out_len);
    z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    g_free (out_bytes);
  } else {
    GST_WARNING_OBJECT (self, "Failed to encode RadarCube");
  }

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
  GstMapInfo map;
  GstCaps *caps;
  GstVideoInfo info;
  const char *encoding = NULL;
  uint8_t *out_bytes = NULL;
  size_t out_len = 0;
  uint32_t width = 0, height = 0, step = 0;

  caps = gst_pad_get_current_caps (GST_BASE_SINK_PAD (self));
  if (caps) {
    gst_video_info_from_caps (&info, caps);
    encoding = gst_format_to_ros_encoding (GST_VIDEO_INFO_FORMAT (&info));
    if (!encoding) {
      GST_WARNING_OBJECT (self, "Unsupported video format for ROS encoding");
      gst_caps_unref (caps);
      return GST_FLOW_ERROR;
    }
    width = (uint32_t) GST_VIDEO_INFO_WIDTH (&info);
    height = (uint32_t) GST_VIDEO_INFO_HEIGHT (&info);
    step = width * (uint32_t) GST_VIDEO_INFO_COMP_PSTRIDE (&info, 0);
    gst_caps_unref (caps);
  }

  if (!encoding) {
    GST_WARNING_OBJECT (self, "No caps available for image encoding");
    return GST_FLOW_ERROR;
  }

  if (!gst_buffer_map (buffer, &map, GST_MAP_READ))
    return GST_FLOW_ERROR;

  if (ros_image_encode (&out_bytes, &out_len,
          0, 0, "", height, width, encoding, 0, step,
          map.data, map.size) == 0) {
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf (&payload, out_bytes, out_len);
    z_publisher_put (z_loan (self->publisher), z_move (payload), NULL);
    ros_bytes_free (out_bytes, out_len);
  } else {
    GST_WARNING_OBJECT (self, "Failed to encode Image");
  }

  gst_buffer_unmap (buffer, &map);
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
