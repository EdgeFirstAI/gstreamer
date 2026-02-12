/*
 * EdgeFirst Perception for GStreamer - Camera Adaptor Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_CAMERA_ADAPTOR_H__
#define __EDGEFIRST_CAMERA_ADAPTOR_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_CAMERA_ADAPTOR (edgefirst_camera_adaptor_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstCameraAdaptor, edgefirst_camera_adaptor,
    EDGEFIRST, CAMERA_ADAPTOR, GstBaseTransform)

typedef enum {
  EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_RGB,
  EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_BGR,
  EDGEFIRST_CAMERA_ADAPTOR_COLORSPACE_GRAY,
} EdgefirstCameraAdaptorColorspace;

typedef enum {
  EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_HWC,
  EDGEFIRST_CAMERA_ADAPTOR_LAYOUT_CHW,
} EdgefirstCameraAdaptorLayout;

typedef enum {
  EDGEFIRST_CAMERA_ADAPTOR_DTYPE_UINT8,
  EDGEFIRST_CAMERA_ADAPTOR_DTYPE_INT8,
  EDGEFIRST_CAMERA_ADAPTOR_DTYPE_FLOAT32,
} EdgefirstCameraAdaptorDtype;

G_END_DECLS

#endif /* __EDGEFIRST_CAMERA_ADAPTOR_H__ */
