/*
 * EdgeFirst Perception for GStreamer - Transform Cache
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_TRANSFORM_CACHE_H__
#define __EDGEFIRST_TRANSFORM_CACHE_H__

#include <glib.h>
#include <gst/edgefirst/edgefirstpointcloud2meta.h>

G_BEGIN_DECLS

typedef struct _EdgefirstTransformCache EdgefirstTransformCache;

/**
 * edgefirst_transform_cache_new:
 *
 * Creates a new transform cache.
 *
 * Returns: (transfer full): a new #EdgefirstTransformCache
 */
EdgefirstTransformCache *edgefirst_transform_cache_new (void);

/**
 * edgefirst_transform_cache_free:
 * @cache: a #EdgefirstTransformCache
 *
 * Frees the transform cache.
 */
void edgefirst_transform_cache_free (EdgefirstTransformCache *cache);

/**
 * edgefirst_transform_cache_insert:
 * @cache: a #EdgefirstTransformCache
 * @transform: the transform to insert
 *
 * Inserts a transform into the cache, keyed by child_frame_id.
 */
void edgefirst_transform_cache_insert (EdgefirstTransformCache *cache,
    const EdgefirstTransformData *transform);

/**
 * edgefirst_transform_cache_lookup:
 * @cache: a #EdgefirstTransformCache
 * @child_frame_id: the source frame
 * @parent_frame_id: the target frame
 * @transform_out: (out): location to store the transform
 *
 * Looks up a transform from child_frame to parent_frame.
 *
 * Returns: TRUE if the transform was found
 */
gboolean edgefirst_transform_cache_lookup (EdgefirstTransformCache *cache,
    const gchar *child_frame_id,
    const gchar *parent_frame_id,
    EdgefirstTransformData *transform_out);

/**
 * edgefirst_transform_cache_clear:
 * @cache: a #EdgefirstTransformCache
 *
 * Clears all transforms from the cache.
 */
void edgefirst_transform_cache_clear (EdgefirstTransformCache *cache);

G_END_DECLS

#endif /* __EDGEFIRST_TRANSFORM_CACHE_H__ */
