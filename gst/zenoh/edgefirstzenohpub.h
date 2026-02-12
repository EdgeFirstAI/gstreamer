/*
 * EdgeFirst Perception for GStreamer - Zenoh Publisher Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_ZENOH_PUB_H__
#define __EDGEFIRST_ZENOH_PUB_H__

#include <gst/gst.h>
#include <gst/base/gstbasesink.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_ZENOH_PUB (edgefirst_zenoh_pub_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstZenohPub, edgefirst_zenoh_pub, EDGEFIRST, ZENOH_PUB, GstBaseSink)

/**
 * EdgefirstZenohPubMessageType:
 * @EDGEFIRST_ZENOH_PUB_POINTCLOUD2: PointCloud2 message
 * @EDGEFIRST_ZENOH_PUB_RADARCUBE: RadarCube message
 * @EDGEFIRST_ZENOH_PUB_IMAGE: sensor_msgs/Image message
 * @EDGEFIRST_ZENOH_PUB_DMABUFFER: edgefirst/DmaBuffer message
 *
 * Message types supported by the Zenoh publisher.
 */
typedef enum {
  EDGEFIRST_ZENOH_PUB_POINTCLOUD2 = 0,
  EDGEFIRST_ZENOH_PUB_RADARCUBE = 1,
  EDGEFIRST_ZENOH_PUB_IMAGE = 2,
  EDGEFIRST_ZENOH_PUB_DMABUFFER = 3,
} EdgefirstZenohPubMessageType;

G_END_DECLS

#endif /* __EDGEFIRST_ZENOH_PUB_H__ */
