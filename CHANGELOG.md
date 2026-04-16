# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-04-16

### Fixed

- **NV12 plane offset corruption** — `edgefirstoverlay` now uses `GstVideoMeta`
  (authoritative buffer layout) instead of `GstVideoInfo` (tight-packed assumption)
  for plane strides and offsets. Fixes a ~10-pixel magenta band at the top of the
  output when decoding H.264 via `v4l2h264dec` on i.MX 8M Plus (Y plane height
  padded to 1088 for macroblock alignment).
- **Split-box tensor detection** — the min-dimension heuristic now handles both
  TFLite features-first (`[4, 8400]`) and Ara-2 anchors-first (`[8400, 4]`) tensor
  shapes, enabling auto-config for YOLOv8 segmentation across both frameworks.
- **Box quantization scale** — the pixel-space scale ÷ model_input_size adjustment
  is now gated on `normalized=false`. TFLite uint8 models that already quantize
  boxes in [0, 1] are no longer collapsed to sub-pixel slivers. Ara-2 DVM pipelines
  must set `normalized=false` until future DVM exports adopt normalized coordinates.

### Added

- **Auto-letterbox** — when the `letterbox` property is unset, edgefirstoverlay
  computes an aspect-preserving fit from the input video dimensions and the inferred
  model input size, matching what `edgefirstcameraadaptor letterbox=true` produces.
  The explicit `letterbox` property still overrides when set.

### Known Issues

- TFLite NHWC protos produce vertical-stripe mask artifacts on i.MX 8M Plus due to
  a layout mismatch between the GStreamer overlay's NCHW shape labeling and the
  physical NHWC memory order. Detection boxes and auto-letterbox work correctly;
  only mask rendering is affected. Requires a HAL-side fix to handle NHWC protos.

## [0.2.0] - 2026-04-13

### Added

- **Dual-sink edgefirstoverlay** — redesigned as a GstElement with separate `video`
  and `tensors` sink pads. Accepts raw video + NNStreamer tensor output, runs
  HAL decode → materialize → draw pipeline, and emits `new-detection` signal.
- Double-buffered DMA-BUF display image for zero-copy waylandsink export.
- Per-element timing instrumentation (decode, materialize, draw) via GST_INFO.
- Deferred decoder creation to first tensor buffer for quantization meta support.
- GPU-aligned row stride for DMA-BUF export (Mali Valhall / i.MX 95 compatibility).

### Fixed

- Wayland display caps negotiation — output plain `video/x-raw` RGBA with framerate
  (waylandsink discovers DMA-BUF at buffer-import time, not through caps features).
- NV12 two-fd DMA-BUF plane handling in edgefirstcameraadaptor.
- NNStreamer dimension ordering for split-tensor detection (anchors-first convention).
- DMA-BUF cache key using buffer inode for stable identity.

## [0.1.1] - 2026-02-17

### Added

- NOTICE file with first-level dependency attributions
- SBOM generation via scancode-toolkit (CycloneDX JSON format)
- License policy checker and NOTICE file validator
- GitHub Actions workflows for SBOM generation and release publishing
- Shared library versioning (soversion 0) for libedgefirst-gstreamer-1.0.so

## [0.1.0] - 2026-02-17

Initial release of EdgeFirst Perception for GStreamer.

### Added

- **Core library** (`libedgefirst-gstreamer-1.0.so`):
  - `EdgefirstPointCloud2Meta` for LiDAR point cloud buffers
  - `EdgefirstRadarCubeMeta` for radar cube tensor metadata
  - `EdgefirstCameraInfoMeta` for camera intrinsic calibration
  - `EdgefirstTransformMeta` for extrinsic transform data
  - Pinhole projection and quaternion transform math utilities
  - Point field parsing and formatting (`edgefirst_parse_point_fields()`)

- **Zenoh bridge plugin** (`libgstedgefirst-zenoh.so`):
  - `edgefirstzenohsub` — subscribe to Zenoh topics, produce GStreamer buffers
    (PointCloud2, RadarCube, Image)
  - `edgefirstzenohpub` — publish GStreamer buffers to Zenoh topics
  - CDR serialization/deserialization via edgefirst-schemas C API
  - Transform cache for `/tf_static` topic with automatic metadata attachment
  - Thread-safe buffer queue with caps carried alongside buffers

- **Fusion plugin** (`libgstedgefirst-fusion.so`):
  - `edgefirstpcdclassify` — project camera segmentation masks onto point
    clouds with configurable output mode (labels, colors, both)
  - `edgefirsttransforminject` — attach calibration metadata from JSON files
    to passing buffers (intrinsic and/or extrinsic)

- **HAL preprocessing plugin** (`libgstedgefirsthal.so`):
  - `edgefirstcameraadaptor` — hardware-accelerated fused image preprocessing
    for ML inference (color conversion, resize, letterbox, quantization)
  - DMA-BUF zero-copy support with automatic tier negotiation
  - NEON-optimized kernels for HWC/CHW layout conversion and int8/float32
    quantization with scalar fallbacks
  - NNStreamer-compatible `other/tensors` output

- **Test suites** covering core metadata, math, fusion elements, and HAL
  elements (documented in TESTING.md)
- **Documentation**: ARCHITECTURE.md, TESTING.md, README.md with pipeline
  examples
- **Examples**: Camera adaptor and radar inference pipeline scripts, sample
  calibration JSON and Zenoh config
