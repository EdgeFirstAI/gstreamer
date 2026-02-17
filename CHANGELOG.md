# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
