/*
 * EdgeFirst Perception for GStreamer - Transform Cache
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "transform-cache.h"
#include <string.h>

struct _EdgefirstTransformCache {
  GHashTable *transforms;  /* child_frame_id → EdgefirstTransformData */
  GMutex lock;
};

EdgefirstTransformCache *
edgefirst_transform_cache_new (void)
{
  EdgefirstTransformCache *cache = g_new0 (EdgefirstTransformCache, 1);

  cache->transforms = g_hash_table_new_full (g_str_hash, g_str_equal,
      g_free, g_free);
  g_mutex_init (&cache->lock);

  return cache;
}

void
edgefirst_transform_cache_free (EdgefirstTransformCache *cache)
{
  if (!cache)
    return;

  g_hash_table_destroy (cache->transforms);
  g_mutex_clear (&cache->lock);
  g_free (cache);
}

void
edgefirst_transform_cache_insert (EdgefirstTransformCache *cache,
    const EdgefirstTransformData *transform)
{
  EdgefirstTransformData *copy;

  g_return_if_fail (cache != NULL);
  g_return_if_fail (transform != NULL);

  copy = g_new (EdgefirstTransformData, 1);
  memcpy (copy, transform, sizeof (EdgefirstTransformData));

  g_mutex_lock (&cache->lock);
  g_hash_table_replace (cache->transforms,
      g_strdup (transform->child_frame_id), copy);
  g_mutex_unlock (&cache->lock);
}

gboolean
edgefirst_transform_cache_lookup (EdgefirstTransformCache *cache,
    const gchar *child_frame_id,
    const gchar *parent_frame_id,
    EdgefirstTransformData *transform_out)
{
  EdgefirstTransformData *found;
  gboolean result = FALSE;

  g_return_val_if_fail (cache != NULL, FALSE);
  g_return_val_if_fail (child_frame_id != NULL, FALSE);
  g_return_val_if_fail (transform_out != NULL, FALSE);

  g_mutex_lock (&cache->lock);

  found = g_hash_table_lookup (cache->transforms, child_frame_id);
  if (found) {
    /* If parent_frame_id is specified, verify it matches */
    if (parent_frame_id == NULL ||
        g_strcmp0 (found->parent_frame_id, parent_frame_id) == 0) {
      memcpy (transform_out, found, sizeof (EdgefirstTransformData));
      result = TRUE;
    }
  }

  g_mutex_unlock (&cache->lock);

  return result;
}

void
edgefirst_transform_cache_clear (EdgefirstTransformCache *cache)
{
  g_return_if_fail (cache != NULL);

  g_mutex_lock (&cache->lock);
  g_hash_table_remove_all (cache->transforms);
  g_mutex_unlock (&cache->lock);
}
