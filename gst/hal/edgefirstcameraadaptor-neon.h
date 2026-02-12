/*
 * EdgeFirst Perception for GStreamer - NEON Post-Processing Kernels
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * SIMD-optimized kernels for Stage 2 post-processing in the two-stage
 * GPU/NEON camera adaptor pipeline.  Each kernel has a NEON fast path
 * on AArch64 and a portable scalar C fallback.
 */

#ifndef __EDGEFIRST_CAMERA_ADAPTOR_NEON_H__
#define __EDGEFIRST_CAMERA_ADAPTOR_NEON_H__

#include <glib.h>
#include <stddef.h>
#include <stdint.h>

G_BEGIN_DECLS

/*
 * HWC kernels — RGBA u8 intermediate (from G2D/GPU) to model format.
 *
 * All HWC kernels accept a `bgr` flag; when TRUE, R and B channels
 * are swapped during the store (zero extra cost on NEON).
 */

/**
 * RGBA u8 → RGB u8 (HWC).  Strips alpha channel.
 *
 * @param src    Source RGBA data (npixels * 4 bytes)
 * @param dst    Destination RGB data (npixels * 3 bytes)
 * @param npixels Number of pixels
 * @param bgr    If TRUE, output is BGR instead of RGB
 */
void edgefirst_neon_rgba_to_rgb_u8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr);

/**
 * RGBA u8 → RGB i8 (HWC).  Strips alpha and applies XOR 0x80
 * (uint8→int8 offset conversion).
 *
 * @param src    Source RGBA data (npixels * 4 bytes)
 * @param dst    Destination RGB i8 data (npixels * 3 bytes)
 * @param npixels Number of pixels
 * @param bgr    If TRUE, output is BGR instead of RGB
 */
void edgefirst_neon_rgba_to_rgb_i8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr);

/**
 * RGBA u8 → RGB f32 (HWC).  Strips alpha, widens to float, and
 * applies ImageNet-style normalization: out = (pixel/255 - mean) / std.
 *
 * @param src    Source RGBA data (npixels * 4 bytes)
 * @param dst    Destination RGB f32 data (npixels * 3 floats)
 * @param npixels Number of pixels
 * @param mean   Per-channel mean [3] (R, G, B)
 * @param std    Per-channel std [3] (R, G, B)
 * @param bgr    If TRUE, output is BGR instead of RGB
 */
void edgefirst_neon_rgba_to_rgb_f32 (const uint8_t *src, float *dst,
    size_t npixels, const float mean[3], const float std[3], gboolean bgr);

/*
 * CHW kernels — RGBA u8 intermediate to planar (CHW) model format.
 *
 * These deinterleave RGBA into separate R, G, B planes with optional
 * dtype conversion.  Always uses RGBA intermediate (works with G2D on
 * all platforms) instead of requiring GPU PLANAR_RGB output.
 *
 * All CHW kernels accept a `bgr` flag; when TRUE, R and B planes
 * are swapped during the store.
 */

/**
 * RGBA u8 → planar u8 (CHW).  Deinterleaves and strips alpha.
 *
 * @param src     Source RGBA data (npixels * 4 bytes)
 * @param dst     Destination planar data (3 * npixels bytes, CHW layout)
 * @param npixels Number of pixels (width * height)
 * @param bgr     If TRUE, output planes are B, G, R instead of R, G, B
 */
void edgefirst_neon_rgba_to_planar_u8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr);

/**
 * RGBA u8 → planar i8 (CHW).  Deinterleaves, strips alpha, and
 * applies XOR 0x80 (uint8→int8 offset conversion).
 *
 * @param src     Source RGBA data (npixels * 4 bytes)
 * @param dst     Destination planar i8 data (3 * npixels bytes, CHW layout)
 * @param npixels Number of pixels (width * height)
 * @param bgr     If TRUE, output planes are B, G, R instead of R, G, B
 */
void edgefirst_neon_rgba_to_planar_i8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr);

/**
 * RGBA u8 → planar f32 (CHW).  Deinterleaves, strips alpha, widens
 * to float, and applies normalization: out = (pixel/255 - mean) / std.
 *
 * @param src     Source RGBA data (npixels * 4 bytes)
 * @param dst     Destination planar f32 data (3 * npixels floats, CHW layout)
 * @param npixels Number of pixels (width * height)
 * @param mean    Per-channel mean [3] (R, G, B)
 * @param std     Per-channel std [3] (R, G, B)
 * @param bgr     If TRUE, output planes are B, G, R instead of R, G, B
 */
void edgefirst_neon_rgba_to_planar_f32 (const uint8_t *src, float *dst,
    size_t npixels, const float mean[3], const float std[3], gboolean bgr);

/*
 * Legacy CHW kernels — PLANAR_RGB u8 intermediate (from GPU) to model format.
 * Used when GPU PLANAR_RGB output is available (e.g. i.MX 95).
 */

/**
 * Planar u8 → planar i8.  Linear XOR 0x80 across entire buffer.
 */
void edgefirst_neon_planar_u8_to_i8 (const uint8_t *src, uint8_t *dst,
    size_t nbytes);

/**
 * Planar u8 → planar f32.  Widens to float and normalizes per-plane.
 */
void edgefirst_neon_planar_u8_to_f32 (const uint8_t *src, float *dst,
    size_t count, float mean, float std);

G_END_DECLS

#endif /* __EDGEFIRST_CAMERA_ADAPTOR_NEON_H__ */
