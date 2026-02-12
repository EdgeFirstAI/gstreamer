/*
 * EdgeFirst Perception for GStreamer - Fusion Plugin
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst.h>
#include "edgefirstpcdclassify.h"
#include "edgefirsttransforminject.h"

static gboolean
plugin_init (GstPlugin *plugin)
{
  gboolean ret = TRUE;

  /* Initialize the core library */
  edgefirst_perception_init ();

  ret &= gst_element_register (plugin, "edgefirstpcdclassify",
      GST_RANK_NONE, EDGEFIRST_TYPE_PCD_CLASSIFY);

  ret &= gst_element_register (plugin, "edgefirsttransforminject",
      GST_RANK_NONE, EDGEFIRST_TYPE_TRANSFORM_INJECT);

  return ret;
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    edgefirstfusion,
    "EdgeFirst Perception fusion and processing elements",
    plugin_init,
    PACKAGE_VERSION,
    "Apache 2.0",
    PACKAGE_NAME,
    "https://edgefirst.ai"
)
