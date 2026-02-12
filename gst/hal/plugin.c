/*
 * EdgeFirst Perception for GStreamer - HAL Plugin
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst.h>
#include "edgefirstcameraadaptor.h"

static gboolean
plugin_init (GstPlugin *plugin)
{
  gboolean ret = TRUE;

  /* Initialize the core library */
  edgefirst_perception_init ();

  ret &= gst_element_register (plugin, "edgefirstcameraadaptor",
      GST_RANK_NONE, EDGEFIRST_TYPE_CAMERA_ADAPTOR);

  return ret;
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    edgefirsthal,
    "EdgeFirst HAL hardware-accelerated image processing elements",
    plugin_init,
    PACKAGE_VERSION,
    "Apache 2.0",
    PACKAGE_NAME,
    "https://edgefirst.ai"
)
