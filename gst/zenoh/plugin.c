/*
 * EdgeFirst Perception for GStreamer - Zenoh Bridge Plugin
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/edgefirst/edgefirst.h>
#include "edgefirstzenohsub.h"
#include "edgefirstzenohpub.h"

static gboolean
plugin_init (GstPlugin *plugin)
{
  gboolean ret = TRUE;

  /* Initialize the core library */
  edgefirst_perception_init ();

  ret &= gst_element_register (plugin, "edgefirstzenohsub",
      GST_RANK_NONE, EDGEFIRST_TYPE_ZENOH_SUB);

  ret &= gst_element_register (plugin, "edgefirstzenohpub",
      GST_RANK_NONE, EDGEFIRST_TYPE_ZENOH_PUB);

  return ret;
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    edgefirstzenoh,
    "EdgeFirst Perception Zenoh bridge elements",
    plugin_init,
    PACKAGE_VERSION,
    "Apache 2.0",
    PACKAGE_NAME,
    "https://edgefirst.ai"
)
