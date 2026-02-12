/*
 * EdgeFirst Perception for GStreamer - NEON Post-Processing Kernels
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * SIMD-optimized kernels for Stage 2 post-processing.  Each function
 * has a NEON intrinsics path (AArch64 baseline) and a scalar C fallback
 * for desktop/CI builds.
 */

#include "edgefirstcameraadaptor-neon.h"

#include <string.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

/* ── HWC: RGBA u8 → RGB u8 ──────────────────────────────────────── */

void
edgefirst_neon_rgba_to_rgb_u8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr)
{
#ifdef __ARM_NEON
  size_t i = 0;
  /* Process 16 pixels per iteration */
  for (; i + 16 <= npixels; i += 16) {
    uint8x16x4_t rgba = vld4q_u8 (src + i * 4);
    uint8x16x3_t rgb;
    if (bgr) {
      rgb.val[0] = rgba.val[2];  /* B */
      rgb.val[1] = rgba.val[1];  /* G */
      rgb.val[2] = rgba.val[0];  /* R */
    } else {
      rgb.val[0] = rgba.val[0];  /* R */
      rgb.val[1] = rgba.val[1];  /* G */
      rgb.val[2] = rgba.val[2];  /* B */
    }
    vst3q_u8 (dst + i * 3, rgb);
  }
  /* Scalar tail */
  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    uint8_t *d = dst + i * 3;
    if (bgr) {
      d[0] = p[2]; d[1] = p[1]; d[2] = p[0];
    } else {
      d[0] = p[0]; d[1] = p[1]; d[2] = p[2];
    }
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    uint8_t *d = dst + i * 3;
    if (bgr) {
      d[0] = p[2]; d[1] = p[1]; d[2] = p[0];
    } else {
      d[0] = p[0]; d[1] = p[1]; d[2] = p[2];
    }
  }
#endif
}

/* ── HWC: RGBA u8 → RGB i8 ──────────────────────────────────────── */

void
edgefirst_neon_rgba_to_rgb_i8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr)
{
#ifdef __ARM_NEON
  const uint8x16_t xor_val = vdupq_n_u8 (0x80);
  size_t i = 0;

  for (; i + 16 <= npixels; i += 16) {
    uint8x16x4_t rgba = vld4q_u8 (src + i * 4);
    uint8x16x3_t rgb;
    if (bgr) {
      rgb.val[0] = veorq_u8 (rgba.val[2], xor_val);
      rgb.val[1] = veorq_u8 (rgba.val[1], xor_val);
      rgb.val[2] = veorq_u8 (rgba.val[0], xor_val);
    } else {
      rgb.val[0] = veorq_u8 (rgba.val[0], xor_val);
      rgb.val[1] = veorq_u8 (rgba.val[1], xor_val);
      rgb.val[2] = veorq_u8 (rgba.val[2], xor_val);
    }
    vst3q_u8 (dst + i * 3, rgb);
  }

  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    uint8_t *d = dst + i * 3;
    if (bgr) {
      d[0] = p[2] ^ 0x80; d[1] = p[1] ^ 0x80; d[2] = p[0] ^ 0x80;
    } else {
      d[0] = p[0] ^ 0x80; d[1] = p[1] ^ 0x80; d[2] = p[2] ^ 0x80;
    }
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    uint8_t *d = dst + i * 3;
    if (bgr) {
      d[0] = p[2] ^ 0x80; d[1] = p[1] ^ 0x80; d[2] = p[0] ^ 0x80;
    } else {
      d[0] = p[0] ^ 0x80; d[1] = p[1] ^ 0x80; d[2] = p[2] ^ 0x80;
    }
  }
#endif
}

/* ── HWC: RGBA u8 → RGB f32 (normalized) ────────────────────────── */

void
edgefirst_neon_rgba_to_rgb_f32 (const uint8_t *src, float *dst,
    size_t npixels, const float mean[3], const float std[3], gboolean bgr)
{
  int ch_r = bgr ? 2 : 0;
  int ch_b = bgr ? 0 : 2;

#ifdef __ARM_NEON
  float inv_std[3], scale[3];
  for (int c = 0; c < 3; c++) {
    inv_std[c] = 1.0f / std[c];
    scale[c] = inv_std[c] / 255.0f;
  }

  float32x4_t v_scale[3], v_mean[3], v_inv_std[3];
  for (int c = 0; c < 3; c++) {
    v_scale[c] = vdupq_n_f32 (scale[c]);
    v_mean[c] = vdupq_n_f32 (mean[c]);
    v_inv_std[c] = vdupq_n_f32 (inv_std[c]);
  }

  size_t i = 0;
  /* Process 8 pixels per iteration */
  for (; i + 8 <= npixels; i += 8) {
    uint8x8x4_t rgba = vld4_u8 (src + i * 4);

    /* Select channels based on BGR flag */
    uint8x8_t ch[3];
    ch[0] = bgr ? rgba.val[2] : rgba.val[0];
    ch[1] = rgba.val[1];
    ch[2] = bgr ? rgba.val[0] : rgba.val[2];

    float32x4x3_t out_lo, out_hi;
    for (int c = 0; c < 3; c++) {
      uint16x8_t wide = vmovl_u8 (ch[c]);
      uint16x4_t lo16 = vget_low_u16 (wide);
      uint16x4_t hi16 = vget_high_u16 (wide);
      float32x4_t flo = vcvtq_f32_u32 (vmovl_u16 (lo16));
      float32x4_t fhi = vcvtq_f32_u32 (vmovl_u16 (hi16));
      /* out = pixel * scale - mean * inv_std = (pixel/255 - mean) / std */
      out_lo.val[c] = vmlsq_f32 (vmulq_f32 (flo, v_scale[c]),
          v_mean[c], v_inv_std[c]);
      out_hi.val[c] = vmlsq_f32 (vmulq_f32 (fhi, v_scale[c]),
          v_mean[c], v_inv_std[c]);
    }

    vst3q_f32 (dst + i * 3, out_lo);
    vst3q_f32 (dst + (i + 4) * 3, out_hi);
  }

  /* Scalar tail */
  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    float *d = dst + i * 3;
    d[0] = ((float) p[ch_r] / 255.0f - mean[0]) / std[0];
    d[1] = ((float) p[1]    / 255.0f - mean[1]) / std[1];
    d[2] = ((float) p[ch_b] / 255.0f - mean[2]) / std[2];
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    float *d = dst + i * 3;
    d[0] = ((float) p[ch_r] / 255.0f - mean[0]) / std[0];
    d[1] = ((float) p[1]    / 255.0f - mean[1]) / std[1];
    d[2] = ((float) p[ch_b] / 255.0f - mean[2]) / std[2];
  }
#endif
}

/* ── CHW: RGBA u8 → planar u8 (deinterleave, strip alpha) ────────── */

void
edgefirst_neon_rgba_to_planar_u8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr)
{
  uint8_t *plane0 = dst;
  uint8_t *plane1 = dst + npixels;
  uint8_t *plane2 = dst + npixels * 2;

#ifdef __ARM_NEON
  size_t i = 0;

  for (; i + 16 <= npixels; i += 16) {
    uint8x16x4_t rgba = vld4q_u8 (src + i * 4);
    if (bgr) {
      vst1q_u8 (plane0 + i, rgba.val[2]);  /* B */
      vst1q_u8 (plane1 + i, rgba.val[1]);  /* G */
      vst1q_u8 (plane2 + i, rgba.val[0]);  /* R */
    } else {
      vst1q_u8 (plane0 + i, rgba.val[0]);  /* R */
      vst1q_u8 (plane1 + i, rgba.val[1]);  /* G */
      vst1q_u8 (plane2 + i, rgba.val[2]);  /* B */
    }
  }

  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    if (bgr) {
      plane0[i] = p[2]; plane1[i] = p[1]; plane2[i] = p[0];
    } else {
      plane0[i] = p[0]; plane1[i] = p[1]; plane2[i] = p[2];
    }
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    if (bgr) {
      plane0[i] = p[2]; plane1[i] = p[1]; plane2[i] = p[0];
    } else {
      plane0[i] = p[0]; plane1[i] = p[1]; plane2[i] = p[2];
    }
  }
#endif
}

/* ── CHW: RGBA u8 → planar i8 (deinterleave + XOR 0x80) ─────────── */

void
edgefirst_neon_rgba_to_planar_i8 (const uint8_t *src, uint8_t *dst,
    size_t npixels, gboolean bgr)
{
  uint8_t *plane0 = dst;
  uint8_t *plane1 = dst + npixels;
  uint8_t *plane2 = dst + npixels * 2;

#ifdef __ARM_NEON
  const uint8x16_t xor_val = vdupq_n_u8 (0x80);
  size_t i = 0;

  for (; i + 16 <= npixels; i += 16) {
    uint8x16x4_t rgba = vld4q_u8 (src + i * 4);
    if (bgr) {
      vst1q_u8 (plane0 + i, veorq_u8 (rgba.val[2], xor_val));  /* B */
      vst1q_u8 (plane1 + i, veorq_u8 (rgba.val[1], xor_val));  /* G */
      vst1q_u8 (plane2 + i, veorq_u8 (rgba.val[0], xor_val));  /* R */
    } else {
      vst1q_u8 (plane0 + i, veorq_u8 (rgba.val[0], xor_val));  /* R */
      vst1q_u8 (plane1 + i, veorq_u8 (rgba.val[1], xor_val));  /* G */
      vst1q_u8 (plane2 + i, veorq_u8 (rgba.val[2], xor_val));  /* B */
    }
  }

  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    if (bgr) {
      plane0[i] = p[2] ^ 0x80; plane1[i] = p[1] ^ 0x80; plane2[i] = p[0] ^ 0x80;
    } else {
      plane0[i] = p[0] ^ 0x80; plane1[i] = p[1] ^ 0x80; plane2[i] = p[2] ^ 0x80;
    }
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    if (bgr) {
      plane0[i] = p[2] ^ 0x80; plane1[i] = p[1] ^ 0x80; plane2[i] = p[0] ^ 0x80;
    } else {
      plane0[i] = p[0] ^ 0x80; plane1[i] = p[1] ^ 0x80; plane2[i] = p[2] ^ 0x80;
    }
  }
#endif
}

/* ── CHW: RGBA u8 → planar f32 (deinterleave + normalize) ────────── */

void
edgefirst_neon_rgba_to_planar_f32 (const uint8_t *src, float *dst,
    size_t npixels, const float mean[3], const float std[3], gboolean bgr)
{
  float *plane0 = dst;
  float *plane1 = dst + npixels;
  float *plane2 = dst + npixels * 2;
  int ch_r = bgr ? 2 : 0;
  int ch_b = bgr ? 0 : 2;

#ifdef __ARM_NEON
  float inv_std[3], scale[3];
  for (int c = 0; c < 3; c++) {
    inv_std[c] = 1.0f / std[c];
    scale[c] = inv_std[c] / 255.0f;
  }

  float32x4_t v_scale[3], v_mean[3], v_inv_std[3];
  for (int c = 0; c < 3; c++) {
    v_scale[c] = vdupq_n_f32 (scale[c]);
    v_mean[c] = vdupq_n_f32 (mean[c]);
    v_inv_std[c] = vdupq_n_f32 (inv_std[c]);
  }

  size_t i = 0;
  for (; i + 8 <= npixels; i += 8) {
    uint8x8x4_t rgba = vld4_u8 (src + i * 4);

    uint8x8_t ch[3];
    ch[0] = bgr ? rgba.val[2] : rgba.val[0];
    ch[1] = rgba.val[1];
    ch[2] = bgr ? rgba.val[0] : rgba.val[2];

    for (int c = 0; c < 3; c++) {
      uint16x8_t wide = vmovl_u8 (ch[c]);
      float32x4_t flo = vcvtq_f32_u32 (vmovl_u16 (vget_low_u16 (wide)));
      float32x4_t fhi = vcvtq_f32_u32 (vmovl_u16 (vget_high_u16 (wide)));
      flo = vmlsq_f32 (vmulq_f32 (flo, v_scale[c]), v_mean[c], v_inv_std[c]);
      fhi = vmlsq_f32 (vmulq_f32 (fhi, v_scale[c]), v_mean[c], v_inv_std[c]);
      float *plane = (c == 0) ? plane0 : (c == 1) ? plane1 : plane2;
      vst1q_f32 (plane + i, flo);
      vst1q_f32 (plane + i + 4, fhi);
    }
  }

  for (; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    plane0[i] = ((float) p[ch_r] / 255.0f - mean[0]) / std[0];
    plane1[i] = ((float) p[1]    / 255.0f - mean[1]) / std[1];
    plane2[i] = ((float) p[ch_b] / 255.0f - mean[2]) / std[2];
  }
#else
  for (size_t i = 0; i < npixels; i++) {
    const uint8_t *p = src + i * 4;
    plane0[i] = ((float) p[ch_r] / 255.0f - mean[0]) / std[0];
    plane1[i] = ((float) p[1]    / 255.0f - mean[1]) / std[1];
    plane2[i] = ((float) p[ch_b] / 255.0f - mean[2]) / std[2];
  }
#endif
}

/* ── Legacy CHW: Planar u8 → planar i8 (linear XOR 0x80) ────────── */

void
edgefirst_neon_planar_u8_to_i8 (const uint8_t *src, uint8_t *dst,
    size_t nbytes)
{
#ifdef __ARM_NEON
  const uint8x16_t xor_val = vdupq_n_u8 (0x80);
  size_t i = 0;

  for (; i + 16 <= nbytes; i += 16) {
    uint8x16_t v = vld1q_u8 (src + i);
    vst1q_u8 (dst + i, veorq_u8 (v, xor_val));
  }

  for (; i < nbytes; i++)
    dst[i] = src[i] ^ 0x80;
#else
  for (size_t i = 0; i < nbytes; i++)
    dst[i] = src[i] ^ 0x80;
#endif
}

/* ── CHW: Planar u8 → planar f32 (normalized, per-plane) ────────── */

void
edgefirst_neon_planar_u8_to_f32 (const uint8_t *src, float *dst,
    size_t count, float mean, float std)
{
#ifdef __ARM_NEON
  float inv_std = 1.0f / std;
  float scale = inv_std / 255.0f;
  float32x4_t v_scale = vdupq_n_f32 (scale);
  float32x4_t v_mean = vdupq_n_f32 (mean);
  float32x4_t v_inv_std = vdupq_n_f32 (inv_std);
  size_t i = 0;

  /* Process 8 elements per iteration */
  for (; i + 8 <= count; i += 8) {
    uint8x8_t v8 = vld1_u8 (src + i);
    uint16x8_t v16 = vmovl_u8 (v8);
    float32x4_t flo = vcvtq_f32_u32 (vmovl_u16 (vget_low_u16 (v16)));
    float32x4_t fhi = vcvtq_f32_u32 (vmovl_u16 (vget_high_u16 (v16)));
    flo = vmlsq_f32 (vmulq_f32 (flo, v_scale), v_mean, v_inv_std);
    fhi = vmlsq_f32 (vmulq_f32 (fhi, v_scale), v_mean, v_inv_std);
    vst1q_f32 (dst + i, flo);
    vst1q_f32 (dst + i + 4, fhi);
  }

  for (; i < count; i++)
    dst[i] = ((float) src[i] / 255.0f - mean) / std;
#else
  for (size_t i = 0; i < count; i++)
    dst[i] = ((float) src[i] / 255.0f - mean) / std;
#endif
}
