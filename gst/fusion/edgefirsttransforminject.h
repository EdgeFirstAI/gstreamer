/*
 * EdgeFirst Perception for GStreamer - Transform Inject Element
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_TRANSFORM_INJECT_H__
#define __EDGEFIRST_TRANSFORM_INJECT_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define EDGEFIRST_TYPE_TRANSFORM_INJECT (edgefirst_transform_inject_get_type())
G_DECLARE_FINAL_TYPE (EdgefirstTransformInject, edgefirst_transform_inject,
    EDGEFIRST, TRANSFORM_INJECT, GstBaseTransform)

G_END_DECLS

#endif /* __EDGEFIRST_TRANSFORM_INJECT_H__ */
