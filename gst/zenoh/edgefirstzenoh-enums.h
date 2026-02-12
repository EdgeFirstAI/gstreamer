/*
 * EdgeFirst Perception for GStreamer - Zenoh Enum GTypes
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_ZENOH_ENUMS_H__
#define __EDGEFIRST_ZENOH_ENUMS_H__

#include <glib-object.h>

G_BEGIN_DECLS

GType edgefirst_zenoh_sub_message_type_get_type (void);
#define EDGEFIRST_TYPE_ZENOH_SUB_MESSAGE_TYPE (edgefirst_zenoh_sub_message_type_get_type())

GType edgefirst_zenoh_pub_message_type_get_type (void);
#define EDGEFIRST_TYPE_ZENOH_PUB_MESSAGE_TYPE (edgefirst_zenoh_pub_message_type_get_type())

G_END_DECLS

#endif /* __EDGEFIRST_ZENOH_ENUMS_H__ */
