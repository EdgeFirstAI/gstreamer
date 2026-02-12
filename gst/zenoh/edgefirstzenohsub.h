/*
 * EdgeFirst Perception for GStreamer - Zenoh Subscriber Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_ZENOH_SUB_H__
#define __EDGEFIRST_ZENOH_SUB_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_ZENOH_SUB (edgefirst_zenoh_sub_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstZenohSub, edgefirst_zenoh_sub, EDGEFIRST, ZENOH_SUB, GstPushSrc)

/**
 * EdgefirstZenohSubMessageType:
 * @EDGEFIRST_ZENOH_MSG_POINTCLOUD2: PointCloud2 message
 * @EDGEFIRST_ZENOH_MSG_RADARCUBE: RadarCube message
 *
 * Message types supported by the Zenoh subscriber.
 */
typedef enum {
  EDGEFIRST_ZENOH_MSG_POINTCLOUD2 = 0,
  EDGEFIRST_ZENOH_MSG_RADARCUBE = 1,
  EDGEFIRST_ZENOH_MSG_IMAGE = 2,
  EDGEFIRST_ZENOH_MSG_CAMERA_INFO = 3,
  EDGEFIRST_ZENOH_MSG_TRANSFORM = 4,
} EdgefirstZenohSubMessageType;

G_END_DECLS

#endif /* __EDGEFIRST_ZENOH_SUB_H__ */
