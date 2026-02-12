/*
 * EdgeFirst Perception for GStreamer - Zenoh Enum GTypes
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include "edgefirstzenoh-enums.h"
#include "edgefirstzenohsub.h"
#include "edgefirstzenohpub.h"

GType
edgefirst_zenoh_sub_message_type_get_type (void)
{
  static GType type = 0;

  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_ZENOH_MSG_POINTCLOUD2, "EDGEFIRST_ZENOH_MSG_POINTCLOUD2", "pointcloud2" },
      { EDGEFIRST_ZENOH_MSG_RADARCUBE, "EDGEFIRST_ZENOH_MSG_RADARCUBE", "radarcube" },
      { EDGEFIRST_ZENOH_MSG_IMAGE, "EDGEFIRST_ZENOH_MSG_IMAGE", "image" },
      { EDGEFIRST_ZENOH_MSG_CAMERA_INFO, "EDGEFIRST_ZENOH_MSG_CAMERA_INFO", "camera-info" },
      { EDGEFIRST_ZENOH_MSG_TRANSFORM, "EDGEFIRST_ZENOH_MSG_TRANSFORM", "transform" },
      { 0, NULL, NULL },
    };
    GType _type = g_enum_register_static ("EdgefirstZenohSubMessageType", values);
    g_once_init_leave (&type, _type);
  }
  return type;
}

GType
edgefirst_zenoh_pub_message_type_get_type (void)
{
  static GType type = 0;

  if (g_once_init_enter (&type)) {
    static const GEnumValue values[] = {
      { EDGEFIRST_ZENOH_PUB_POINTCLOUD2, "EDGEFIRST_ZENOH_PUB_POINTCLOUD2", "pointcloud2" },
      { EDGEFIRST_ZENOH_PUB_RADARCUBE, "EDGEFIRST_ZENOH_PUB_RADARCUBE", "radarcube" },
      { EDGEFIRST_ZENOH_PUB_IMAGE, "EDGEFIRST_ZENOH_PUB_IMAGE", "image" },
      { EDGEFIRST_ZENOH_PUB_DMABUFFER, "EDGEFIRST_ZENOH_PUB_DMABUFFER", "dmabuffer" },
      { 0, NULL, NULL },
    };
    GType _type = g_enum_register_static ("EdgefirstZenohPubMessageType", values);
    g_once_init_leave (&type, _type);
  }
  return type;
}
