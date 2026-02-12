/*
 * EdgeFirst Perception for GStreamer - Zenoh Subscriber Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "edgefirstzenohsub.h"
#include "edgefirstzenoh-enums.h"
#include "transform-cache.h"
#include <gst/edgefirst/edgefirst.h>
#include <gst/video/video.h>
#include <edgefirst/schemas.h>
#include <zenoh.h>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC (edgefirst_zenoh_sub_debug);
#define GST_CAT_DEFAULT edgefirst_zenoh_sub_debug

#define BUFFER_QUEUE_MAX 16

/* Queue item carrying both buffer and caps from callback to streaming thread */
typedef struct {
  GstBuffer *buf;
  GstCaps *caps;   /* may be NULL */
} EdgefirstQueueItem;

enum {
  PROP_0,
  PROP_TOPIC,
  PROP_MESSAGE_TYPE,
  PROP_SESSION,
  PROP_RELIABLE,
};

struct _EdgefirstZenohSub {
  GstPushSrc parent;

  /* Properties */
  gchar *topic;
  EdgefirstZenohSubMessageType message_type;
  gchar *session_config;
  gboolean reliable;

  /* Runtime state */
  gboolean started;
  GMutex lock;
  GCond cond;
  GQueue *buffer_queue;   /* of EdgefirstQueueItem* */
  GstCaps *last_caps;     /* set on streaming thread only */

  /* Transform cache for /tf_static */
  EdgefirstTransformCache *transform_cache;

  /* Zenoh session and subscriber handles */
  z_owned_session_t session;
  z_owned_subscriber_t subscriber;
  z_owned_subscriber_t tf_subscriber;
  gboolean session_valid;
};

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (EDGEFIRST_POINTCLOUD2_CAPS "; "
        "other/tensors, num-tensors = (int) 1; "
        "video/x-raw")
    );

#define edgefirst_zenoh_sub_parent_class parent_class
G_DEFINE_TYPE (EdgefirstZenohSub, edgefirst_zenoh_sub, GST_TYPE_PUSH_SRC);

static void edgefirst_zenoh_sub_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec);
static void edgefirst_zenoh_sub_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec);
static void edgefirst_zenoh_sub_finalize (GObject *object);

static gboolean edgefirst_zenoh_sub_start (GstBaseSrc *src);
static gboolean edgefirst_zenoh_sub_stop (GstBaseSrc *src);
static GstFlowReturn edgefirst_zenoh_sub_create (GstPushSrc *src, GstBuffer **buf);

/* ── Forward declarations for callbacks ────────────────────────────── */

static void zenoh_sub_data_handler (z_loaned_sample_t *sample, void *context);
static void zenoh_sub_tf_handler (z_loaned_sample_t *sample, void *context);

/* ── Helper: push buffer + caps to queue ───────────────────────────── */

static void
queue_item_free (EdgefirstQueueItem *item)
{
  gst_buffer_unref (item->buf);
  gst_clear_caps (&item->caps);
  g_free (item);
}

static void
push_to_queue (EdgefirstZenohSub *self, GstBuffer *buffer, GstCaps *caps)
{
  EdgefirstQueueItem *item = g_new0 (EdgefirstQueueItem, 1);
  item->buf = buffer;
  item->caps = caps;   /* takes ownership, may be NULL */

  g_mutex_lock (&self->lock);

  /* Drop oldest if queue is full */
  while (g_queue_get_length (self->buffer_queue) >= BUFFER_QUEUE_MAX) {
    EdgefirstQueueItem *old = g_queue_pop_head (self->buffer_queue);
    GST_DEBUG_OBJECT (self, "Dropping oldest buffer from queue");
    queue_item_free (old);
  }

  g_queue_push_tail (self->buffer_queue, item);
  g_cond_signal (&self->cond);
  g_mutex_unlock (&self->lock);
}

/* ── Class init ────────────────────────────────────────────────────── */

static void
edgefirst_zenoh_sub_class_init (EdgefirstZenohSubClass *klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS (klass);
  GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS (klass);

  gobject_class->set_property = edgefirst_zenoh_sub_set_property;
  gobject_class->get_property = edgefirst_zenoh_sub_get_property;
  gobject_class->finalize = edgefirst_zenoh_sub_finalize;

  g_object_class_install_property (gobject_class, PROP_TOPIC,
      g_param_spec_string ("topic", "Topic",
          "Zenoh key expression to subscribe to",
          NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_MESSAGE_TYPE,
      g_param_spec_enum ("message-type", "Message Type",
          "Type of message expected on the topic",
          EDGEFIRST_TYPE_ZENOH_SUB_MESSAGE_TYPE,
          EDGEFIRST_ZENOH_MSG_POINTCLOUD2,
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
      "EdgeFirst Zenoh Subscriber",
      "Source/Network",
      "Subscribe to Zenoh topics and produce GStreamer buffers",
      "Au-Zone Technologies <support@au-zone.com>");

  gst_element_class_add_static_pad_template (element_class, &src_template);

  basesrc_class->start = GST_DEBUG_FUNCPTR (edgefirst_zenoh_sub_start);
  basesrc_class->stop = GST_DEBUG_FUNCPTR (edgefirst_zenoh_sub_stop);
  pushsrc_class->create = GST_DEBUG_FUNCPTR (edgefirst_zenoh_sub_create);

  GST_DEBUG_CATEGORY_INIT (edgefirst_zenoh_sub_debug, "edgefirstzenohsub", 0,
      "EdgeFirst Zenoh Subscriber");
}

static void
edgefirst_zenoh_sub_init (EdgefirstZenohSub *self)
{
  self->topic = NULL;
  self->message_type = EDGEFIRST_ZENOH_MSG_POINTCLOUD2;
  self->session_config = NULL;
  self->reliable = TRUE;
  self->started = FALSE;
  self->session_valid = FALSE;

  g_mutex_init (&self->lock);
  g_cond_init (&self->cond);
  self->buffer_queue = g_queue_new ();
  self->last_caps = NULL;
  self->transform_cache = edgefirst_transform_cache_new ();

  gst_base_src_set_live (GST_BASE_SRC (self), TRUE);
  gst_base_src_set_format (GST_BASE_SRC (self), GST_FORMAT_TIME);
}

static void
edgefirst_zenoh_sub_finalize (GObject *object)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (object);

  g_free (self->topic);
  g_free (self->session_config);
  g_mutex_clear (&self->lock);
  g_cond_clear (&self->cond);
  g_queue_free_full (self->buffer_queue, (GDestroyNotify) queue_item_free);
  gst_clear_caps (&self->last_caps);
  edgefirst_transform_cache_free (self->transform_cache);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
edgefirst_zenoh_sub_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (object);

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
edgefirst_zenoh_sub_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (object);

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
open_zenoh_session (EdgefirstZenohSub *self)
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

/* ── Data deserialization handlers ─────────────────────────────────── */

static GstBuffer *
handle_pointcloud2 (EdgefirstZenohSub *self, const uint8_t *data, size_t len,
    GstCaps **out_caps)
{
  RosPointCloud2 *pcd;
  GstBuffer *buffer;
  const uint8_t *cloud_data;
  size_t cloud_data_len;
  EdgefirstPointCloud2Meta *meta;
  size_t num_ros_fields;
  EdgefirstPointFieldDesc gst_fields[32];
  guint num_gst_fields = 0;
  gchar *fields_str;
  GstCaps *caps;

  pcd = ros_point_cloud2_deserialize (data, len);
  if (!pcd) {
    GST_WARNING_OBJECT (self, "Failed to deserialize PointCloud2");
    return NULL;
  }

  cloud_data = ros_point_cloud2_get_data (pcd, &cloud_data_len);
  if (!cloud_data || cloud_data_len == 0) {
    ros_point_cloud2_free (pcd);
    return NULL;
  }

  buffer = gst_buffer_new_allocate (NULL, cloud_data_len, NULL);
  gst_buffer_fill (buffer, 0, cloud_data, cloud_data_len);

  /* Attach metadata */
  meta = edgefirst_buffer_add_pointcloud2_meta (buffer);
  if (meta) {
    meta->point_count = ros_point_cloud2_get_width (pcd)
        * ros_point_cloud2_get_height (pcd);

    const RosHeader *hdr = ros_point_cloud2_get_header_mut (pcd);
    if (hdr) {
      char *frame_id = ros_header_get_frame_id (hdr);
      if (frame_id) {
        g_strlcpy (meta->frame_id, frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
        free (frame_id);
      }
      const RosTime *stamp = ros_header_get_stamp (hdr);
      if (stamp) {
        meta->ros_timestamp_ns = (guint64) ros_time_get_sec (stamp) * G_GUINT64_CONSTANT (1000000000)
            + ros_time_get_nanosec (stamp);
      }
    }

    /* Lookup transform from cache */
    if (meta->frame_id[0] != '\0') {
      meta->has_transform = edgefirst_transform_cache_lookup (
          self->transform_cache, meta->frame_id, NULL, &meta->transform);
    }
  }

  /* Build field descriptors from ROS fields */
  num_ros_fields = ros_point_cloud2_get_num_fields (pcd);
  for (size_t i = 0; i < num_ros_fields && num_gst_fields < 32; i++) {
    const RosPointField *rf = ros_point_cloud2_get_field_at (pcd, i);
    char *name = ros_point_field_get_name (rf);
    if (name) {
      g_strlcpy (gst_fields[num_gst_fields].name, name, 64);
      free (name);
    }
    gst_fields[num_gst_fields].datatype = ros_point_field_get_datatype (rf);
    gst_fields[num_gst_fields].offset = ros_point_field_get_offset (rf);
    gst_fields[num_gst_fields].count = ros_point_field_get_count (rf);
    num_gst_fields++;
  }

  fields_str = edgefirst_format_point_fields (gst_fields, num_gst_fields);

  caps = gst_caps_new_simple ("application/x-pointcloud2",
      "width", G_TYPE_INT, (gint) ros_point_cloud2_get_width (pcd),
      "height", G_TYPE_INT, (gint) ros_point_cloud2_get_height (pcd),
      "point-step", G_TYPE_INT, (gint) ros_point_cloud2_get_point_step (pcd),
      "fields", G_TYPE_STRING, fields_str,
      "is-bigendian", G_TYPE_BOOLEAN, (gboolean) ros_point_cloud2_get_is_bigendian (pcd),
      "is-dense", G_TYPE_BOOLEAN, (gboolean) ros_point_cloud2_get_is_dense (pcd),
      NULL);

  *out_caps = caps;
  g_free (fields_str);

  ros_point_cloud2_free (pcd);
  return buffer;
}

static GstBuffer *
handle_radarcube (EdgefirstZenohSub *self, const uint8_t *data, size_t len,
    GstCaps **out_caps)
{
  EdgeFirstRadarCube *cube;
  GstBuffer *buffer;
  const int16_t *cube_data;
  size_t cube_data_len;
  EdgefirstRadarCubeMeta *meta;
  const uint8_t *layout;
  const uint16_t *shape;
  const float *scales;
  size_t layout_len, shape_len, scales_len;

  cube = edgefirst_radarcube_deserialize (data, len);
  if (!cube) {
    GST_WARNING_OBJECT (self, "Failed to deserialize RadarCube");
    return NULL;
  }

  cube_data = edgefirst_radarcube_get_cube (cube, &cube_data_len);
  if (!cube_data || cube_data_len == 0) {
    edgefirst_radarcube_free (cube);
    return NULL;
  }

  buffer = gst_buffer_new_allocate (NULL,
      cube_data_len * sizeof (int16_t), NULL);
  gst_buffer_fill (buffer, 0, cube_data, cube_data_len * sizeof (int16_t));

  /* Attach metadata */
  meta = edgefirst_buffer_add_radar_cube_meta (buffer);
  if (meta) {
    layout = edgefirst_radarcube_get_layout (cube, &layout_len);
    shape = edgefirst_radarcube_get_shape (cube, &shape_len);
    scales = edgefirst_radarcube_get_scales (cube, &scales_len);

    meta->num_dims = (guint8) MIN (layout_len, EDGEFIRST_RADAR_MAX_DIMS);
    for (guint8 i = 0; i < meta->num_dims; i++)
      meta->layout[i] = (EdgefirstRadarDimension) layout[i];

    for (guint8 i = 0; i < MIN (scales_len, EDGEFIRST_RADAR_MAX_DIMS); i++)
      meta->scales[i] = scales[i];

    meta->is_complex = edgefirst_radarcube_get_is_complex (cube);
    meta->radar_timestamp = edgefirst_radarcube_get_timestamp (cube);

    /* Get frame_id from header */
    const RosHeader *hdr = edgefirst_radarcube_get_header (cube);
    if (hdr) {
      char *frame_id = ros_header_get_frame_id (hdr);
      if (frame_id) {
        g_strlcpy (meta->frame_id, frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
        free (frame_id);
      }
    }

    /* Set caps for tensor data */
    {
      GString *dim_str = g_string_new (NULL);

      for (guint8 i = 0; i < MIN (shape_len, EDGEFIRST_RADAR_MAX_DIMS); i++) {
        if (i > 0)
          g_string_append_c (dim_str, ':');
        g_string_append_printf (dim_str, "%u", shape[i]);
      }

      *out_caps = gst_caps_new_simple ("other/tensors",
          "num-tensors", G_TYPE_INT, 1,
          "types", G_TYPE_STRING, "int16",
          "dimensions", G_TYPE_STRING, dim_str->str,
          "format", G_TYPE_STRING, "static",
          NULL);

      g_string_free (dim_str, TRUE);
    }
  }

  edgefirst_radarcube_free (cube);
  return buffer;
}

static GstVideoFormat
ros_encoding_to_gst_format (const char *encoding)
{
  if (!encoding)
    return GST_VIDEO_FORMAT_UNKNOWN;

  if (g_strcmp0 (encoding, "rgb8") == 0)
    return GST_VIDEO_FORMAT_RGB;
  if (g_strcmp0 (encoding, "bgr8") == 0)
    return GST_VIDEO_FORMAT_BGR;
  if (g_strcmp0 (encoding, "rgba8") == 0)
    return GST_VIDEO_FORMAT_RGBA;
  if (g_strcmp0 (encoding, "bgra8") == 0)
    return GST_VIDEO_FORMAT_BGRA;
  if (g_strcmp0 (encoding, "mono8") == 0)
    return GST_VIDEO_FORMAT_GRAY8;
  if (g_strcmp0 (encoding, "mono16") == 0)
    return GST_VIDEO_FORMAT_GRAY16_LE;
  if (g_strcmp0 (encoding, "yuv422") == 0)
    return GST_VIDEO_FORMAT_UYVY;

  return GST_VIDEO_FORMAT_UNKNOWN;
}

static GstBuffer *
handle_image (EdgefirstZenohSub *self, const uint8_t *data, size_t len,
    GstCaps **out_caps)
{
  RosImage *img;
  GstBuffer *buffer;
  const uint8_t *img_data;
  size_t img_data_len;
  char *encoding;
  GstVideoFormat format;
  GstVideoInfo info;

  img = ros_image_deserialize (data, len);
  if (!img) {
    GST_WARNING_OBJECT (self, "Failed to deserialize Image");
    return NULL;
  }

  img_data = ros_image_get_data (img, &img_data_len);
  if (!img_data || img_data_len == 0) {
    ros_image_free (img);
    return NULL;
  }

  encoding = ros_image_get_encoding (img);
  format = ros_encoding_to_gst_format (encoding);
  free (encoding);

  if (format == GST_VIDEO_FORMAT_UNKNOWN) {
    GST_WARNING_OBJECT (self, "Unsupported image encoding");
    ros_image_free (img);
    return NULL;
  }

  gst_video_info_set_format (&info, format,
      ros_image_get_width (img), ros_image_get_height (img));

  buffer = gst_buffer_new_allocate (NULL, img_data_len, NULL);
  gst_buffer_fill (buffer, 0, img_data, img_data_len);

  *out_caps = gst_video_info_to_caps (&info);

  ros_image_free (img);
  return buffer;
}

static GstBuffer *
handle_camera_info (EdgefirstZenohSub *self, const uint8_t *data, size_t len)
{
  RosCameraInfo *ci;
  GstBuffer *buffer;
  EdgefirstCameraInfoMeta *meta;
  const double *arr;
  size_t d_len;

  ci = ros_camera_info_deserialize (data, len);
  if (!ci) {
    GST_WARNING_OBJECT (self, "Failed to deserialize CameraInfo");
    return NULL;
  }

  /* CameraInfo doesn't carry pixel data, create a minimal buffer */
  buffer = gst_buffer_new ();
  meta = edgefirst_buffer_add_camera_info_meta (buffer);
  if (meta) {
    meta->width = ros_camera_info_get_width (ci);
    meta->height = ros_camera_info_get_height (ci);

    arr = ros_camera_info_get_k (ci);
    if (arr)
      memcpy (meta->K, arr, sizeof (meta->K));

    arr = ros_camera_info_get_d (ci, &d_len);
    if (arr && d_len > 0) {
      guint n = MIN (d_len, EDGEFIRST_MAX_DISTORTION_COEFFS);
      memcpy (meta->D, arr, n * sizeof (gdouble));
      meta->num_distortion_coeffs = (guint8) n;
    }

    char *model = ros_camera_info_get_distortion_model (ci);
    if (model) {
      if (g_strcmp0 (model, "plumb_bob") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_PLUMB_BOB;
      else if (g_strcmp0 (model, "equidistant") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_EQUIDISTANT;
      else if (g_strcmp0 (model, "rational_polynomial") == 0)
        meta->distortion_model = EDGEFIRST_DISTORTION_RATIONAL;
      free (model);
    }

    arr = ros_camera_info_get_r (ci);
    if (arr)
      memcpy (meta->R, arr, sizeof (meta->R));

    arr = ros_camera_info_get_p (ci);
    if (arr)
      memcpy (meta->P, arr, sizeof (meta->P));

    const RosHeader *hdr = ros_camera_info_get_header (ci);
    if (hdr) {
      char *frame_id = ros_header_get_frame_id (hdr);
      if (frame_id) {
        g_strlcpy (meta->frame_id, frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
        free (frame_id);
      }
    }
  }

  ros_camera_info_free (ci);
  return buffer;
}

/* ── Zenoh callbacks ───────────────────────────────────────────────── */

static void
zenoh_sub_data_handler (z_loaned_sample_t *sample, void *context)
{
  EdgefirstZenohSub *self = (EdgefirstZenohSub *) context;
  const z_loaned_bytes_t *payload;
  z_bytes_slice_iterator_t iter;
  z_view_slice_t slice;
  GstBuffer *buffer = NULL;
  GstCaps *caps = NULL;

  payload = z_sample_payload (sample);

  /* Get raw bytes from payload */
  iter = z_bytes_get_slice_iterator (payload);
  if (!z_bytes_slice_iterator_next (&iter, &slice))
    return;

  const uint8_t *data = z_slice_data (z_view_slice_loan (&slice));
  size_t len = z_slice_len (z_view_slice_loan (&slice));

  if (!data || len == 0)
    return;

  /* message_type is read without a lock; it must be set before READY state */
  switch (self->message_type) {
    case EDGEFIRST_ZENOH_MSG_POINTCLOUD2:
      buffer = handle_pointcloud2 (self, data, len, &caps);
      break;
    case EDGEFIRST_ZENOH_MSG_RADARCUBE:
      buffer = handle_radarcube (self, data, len, &caps);
      break;
    case EDGEFIRST_ZENOH_MSG_IMAGE:
      buffer = handle_image (self, data, len, &caps);
      break;
    case EDGEFIRST_ZENOH_MSG_CAMERA_INFO:
      buffer = handle_camera_info (self, data, len);
      break;
    case EDGEFIRST_ZENOH_MSG_TRANSFORM:
      /* Transform messages handled via TF subscriber */
      break;
  }

  if (buffer) {
    buffer->pts = gst_util_get_timestamp ();
    push_to_queue (self, buffer, caps);
    caps = NULL;   /* ownership transferred */
  }
  gst_clear_caps (&caps);
}

static void
zenoh_sub_tf_handler (z_loaned_sample_t *sample, void *context)
{
  EdgefirstZenohSub *self = (EdgefirstZenohSub *) context;
  const z_loaned_bytes_t *payload;
  z_bytes_slice_iterator_t iter;
  z_view_slice_t slice;

  payload = z_sample_payload (sample);

  iter = z_bytes_get_slice_iterator (payload);
  if (!z_bytes_slice_iterator_next (&iter, &slice))
    return;

  const uint8_t *data = z_slice_data (z_view_slice_loan (&slice));
  size_t len = z_slice_len (z_view_slice_loan (&slice));

  if (!data || len == 0)
    return;

  RosTransformStamped *tf = ros_transform_stamped_deserialize (data, len);
  if (!tf)
    return;

  EdgefirstTransformData td;
  edgefirst_transform_data_set_identity (&td);

  /* Extract translation and rotation */
  const RosTransform *transform = ros_transform_stamped_get_transform (tf);
  if (transform) {
    const RosVector3 *trans = ros_transform_get_translation (transform);
    if (trans) {
      td.translation[0] = ros_vector3_get_x (trans);
      td.translation[1] = ros_vector3_get_y (trans);
      td.translation[2] = ros_vector3_get_z (trans);
    }

    const RosQuaternion *rot = ros_transform_get_rotation (transform);
    if (rot) {
      td.rotation[0] = ros_quaternion_get_x (rot);
      td.rotation[1] = ros_quaternion_get_y (rot);
      td.rotation[2] = ros_quaternion_get_z (rot);
      td.rotation[3] = ros_quaternion_get_w (rot);
    }
  }

  /* Extract frame IDs */
  char *child_frame_id = ros_transform_stamped_get_child_frame_id (tf);
  if (child_frame_id) {
    g_strlcpy (td.child_frame_id, child_frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
    free (child_frame_id);
  }

  const RosHeader *hdr = ros_transform_stamped_get_header (tf);
  if (hdr) {
    char *frame_id = ros_header_get_frame_id (hdr);
    if (frame_id) {
      g_strlcpy (td.parent_frame_id, frame_id, EDGEFIRST_FRAME_ID_MAX_LEN);
      free (frame_id);
    }
    const RosTime *stamp = ros_header_get_stamp (hdr);
    if (stamp) {
      td.timestamp_ns = (guint64) ros_time_get_sec (stamp) * G_GUINT64_CONSTANT (1000000000)
          + ros_time_get_nanosec (stamp);
    }
  }

  edgefirst_transform_cache_insert (self->transform_cache, &td);

  GST_DEBUG_OBJECT (self, "Cached transform: %s -> %s",
      td.child_frame_id, td.parent_frame_id);

  ros_transform_stamped_free (tf);
}

/* ── Start / Stop / Create ─────────────────────────────────────────── */

static gboolean
edgefirst_zenoh_sub_start (GstBaseSrc *src)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (src);
  z_owned_closure_sample_t callback;
  z_view_keyexpr_t ke;

  if (!self->topic) {
    GST_ERROR_OBJECT (self, "No topic specified");
    return FALSE;
  }

  GST_INFO_OBJECT (self, "Starting Zenoh subscriber on topic: %s", self->topic);

  if (!open_zenoh_session (self))
    return FALSE;

  /* Subscribe to main topic */
  z_closure_sample (&callback, zenoh_sub_data_handler, NULL, self);
  z_view_keyexpr_from_str (&ke, self->topic);

  if (z_declare_subscriber (z_loan (self->session), &self->subscriber,
          z_loan (ke), z_move (callback), NULL) != Z_OK) {
    GST_ERROR_OBJECT (self, "Failed to create subscriber for: %s", self->topic);
    z_drop (z_move (self->session));
    self->session_valid = FALSE;
    return FALSE;
  }

  /* Subscribe to rt/tf_static for transforms */
  {
    z_owned_closure_sample_t tf_callback;
    z_view_keyexpr_t tf_ke;

    z_closure_sample (&tf_callback, zenoh_sub_tf_handler, NULL, self);
    z_view_keyexpr_from_str (&tf_ke, "rt/tf_static");

    if (z_declare_subscriber (z_loan (self->session), &self->tf_subscriber,
            z_loan (tf_ke), z_move (tf_callback), NULL) != Z_OK) {
      GST_WARNING_OBJECT (self, "Failed to subscribe to rt/tf_static");
      /* Non-fatal: transforms are optional */
    }
  }

  self->started = TRUE;
  return TRUE;
}

static gboolean
edgefirst_zenoh_sub_stop (GstBaseSrc *src)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (src);

  GST_INFO_OBJECT (self, "Stopping Zenoh subscriber");

  g_mutex_lock (&self->lock);
  self->started = FALSE;
  g_cond_signal (&self->cond);
  g_mutex_unlock (&self->lock);

  z_drop (z_move (self->tf_subscriber));
  z_drop (z_move (self->subscriber));

  if (self->session_valid) {
    z_drop (z_move (self->session));
    self->session_valid = FALSE;
  }

  /* Drain the queue */
  g_mutex_lock (&self->lock);
  while (!g_queue_is_empty (self->buffer_queue)) {
    EdgefirstQueueItem *item = g_queue_pop_head (self->buffer_queue);
    queue_item_free (item);
  }
  g_mutex_unlock (&self->lock);

  gst_clear_caps (&self->last_caps);

  return TRUE;
}

static GstFlowReturn
edgefirst_zenoh_sub_create (GstPushSrc *src, GstBuffer **buf)
{
  EdgefirstZenohSub *self = EDGEFIRST_ZENOH_SUB (src);
  EdgefirstQueueItem *item = NULL;

  g_mutex_lock (&self->lock);

  while (self->started && g_queue_is_empty (self->buffer_queue)) {
    g_cond_wait (&self->cond, &self->lock);
  }

  if (!self->started) {
    g_mutex_unlock (&self->lock);
    return GST_FLOW_FLUSHING;
  }

  item = g_queue_pop_head (self->buffer_queue);
  g_mutex_unlock (&self->lock);

  if (!item) {
    return GST_FLOW_ERROR;
  }

  /* Set caps on the streaming thread only when they change */
  if (item->caps) {
    if (!self->last_caps || !gst_caps_is_equal (self->last_caps, item->caps)) {
      gst_base_src_set_caps (GST_BASE_SRC (self), item->caps);
      gst_caps_replace (&self->last_caps, item->caps);
    }
    gst_caps_unref (item->caps);
  }

  *buf = item->buf;
  g_free (item);
  return GST_FLOW_OK;
}
