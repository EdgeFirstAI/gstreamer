# AI Assistant Instructions

Instructions for AI coding assistants (GitHub Copilot, Claude Code, etc.) working in this repository.

## Project Context

**edgefirst-gstreamer** — EdgeFirst Perception for GStreamer — is a C11 GStreamer plugin suite for spatial perception: LiDAR point clouds, automotive radar cubes, camera ML preprocessing, and sensor fusion with Zenoh/CDR messaging bridge. Licensed Apache-2.0.

Part of the **EdgeFirst Perception** ecosystem (github.com/EdgeFirstAI) — edge-optimized vision, spatial perception, and AI/ML middleware commercially supported by Au-Zone Technologies (au-zone.com / edgefirst.ai). Documentation at doc.edgefirst.ai.

**Sibling projects:**
- **EdgeFirst Perception for Zenoh** — together with this project forms the core Maivin Middleware (doc.edgefirst.ai/latest/perception/). Services are styled after ROS2 using CDR-encoded messages over Zenoh.
- **EdgeFirst HAL** (github.com/EdgeFirstAI/hal) — low-level pre/post-processing algorithms, tracking, and hardware acceleration. This project heavily depends on HAL.
- **EdgeFirst Perception for ROS2** (planned) — will build on the Zenoh stack with first-class ROS2 RMW support.

**Target platforms:** NXP i.MX 8M Plus, NXP i.MX 95, Ara-2/Ara240, Raspberry Pi 5, NVIDIA Jetson. More being added over time.

## Build Commands

```sh
# Setup dependencies (downloads zenoh-c, edgefirst-schemas, edgefirst-hal)
source env.sh

# Full build
meson setup builddir
meson compile -C builddir

# Minimal build (core + fusion only, no external sensor deps)
meson setup builddir -Dzenoh=disabled -Dhal=disabled
meson compile -C builddir

# Clean rebuild
rm -rf builddir && source env.sh && meson setup builddir && meson compile -C builddir
```

## Testing

```sh
# All tests
meson test -C builddir

# Single test suite
meson test -C builddir meta
meson test -C builddir meta_copy
meson test -C builddir math
meson test -C builddir fusion_elements
meson test -C builddir hal_elements    # requires edgefirst-hal

# Verbose with GStreamer debug
GST_DEBUG=edgefirst*:5 meson test -C builddir -v
```

Test fixtures live in `tests/fixtures/`. The `FIXTURE_DIR` macro is set at compile time for tests that need them.

## Architecture

Four libraries with layered dependencies:

- **Core** (`gst-libs/gst/edgefirst/` → `libedgefirst-gstreamer-1.0.so`): GstMeta types (`EdgefirstPointCloud2Meta`, `EdgefirstRadarCubeMeta`, `EdgefirstCameraInfoMeta`, `EdgefirstTransformMeta`), caps registration, math utilities. Required by all plugins.
- **Zenoh** (`gst/zenoh/` → `libgstedgefirst-zenoh.so`): `edgefirstzenohsub` (GstPushSrc), `edgefirstzenohpub` (GstBaseSink). CDR ser/deser via edgefirst-schemas. Maintains a transform cache from `/tf_static`.
- **Fusion** (`gst/fusion/` → `libgstedgefirst-fusion.so`): `edgefirstpcdclassify` (GstAggregator, multi-pad), `edgefirsttransforminject` (GstBaseTransform, in-place, passthrough disabled).
- **HAL** (`gst/hal/` → `libgstedgefirsthal.so`): `edgefirstcameraadaptor` (GstBaseTransform) — fused color convert + resize + letterbox + quantization. NEON kernels in `edgefirstcameraadaptor-neon.{c,h}`.

## Naming Conventions

- **Namespace**: `Edgefirst*` / `edgefirst_*` / `EDGEFIRST_*` (NOT `Gst`/`gst_`/`GST_` for EdgeFirst types)
- **Source files**: `edgefirst*.{c,h}` (no `gst` prefix on source files)
- **Debug categories**: match element names, e.g. `edgefirstzenohsub`. Bulk enable: `GST_DEBUG=edgefirst*:5`
- **Plugin .so outputs**: only the shared library files keep `libgst*` prefix

## Code Style

- C11, GNU-based clang-format (see `.clang-format`): 2-space indent, 100-col limit, Linux brace style, pointer right-aligned (`char *foo`)
- EditorConfig: UTF-8, LF, 2-space indent for `.c`, `.h`, `meson.build`
- GStreamer logging conventions: `GST_LOG` per-buffer, `GST_DEBUG` state/config, `GST_INFO` lifecycle, `GST_WARNING` recoverable, `GST_ERROR` fatal

## Key Design Decisions

- `edgefirsttransforminject` has `passthrough_on_same_caps = FALSE` — passthrough skips `transform_ip`, which would silently disable metadata injection
- GFX module (pcdoverlay) must use bare `GstAggregator`, NOT `GstGLBaseMixer` — the latter enforces video caps on all sink pads, incompatible with `application/x-pointcloud2`
- Camera adaptor uses RGBA-always intermediate stage; HAL cascade: G2D → CPU shortcut → OpenGL → CPU fallback
- NNStreamer tensor dimensions are innermost-to-outermost: HWC → `"C:W:H:1"`, CHW → `"W:H:C:1"`

## Dependencies

Required: GStreamer >= 1.20, GLib >= 2.56, Meson >= 0.62. Optional (feature-gated): zenoh-c, edgefirst-schemas (≥1.5), edgefirst-hal, json-glib-1.0, NNStreamer (≥2.0, runtime only).

`source env.sh` downloads pre-compiled deps to `deps/host/` and sets `PKG_CONFIG_PATH` + `LD_LIBRARY_PATH`.

## Reference Documentation

- Architecture details: `ARCHITECTURE.md` (project root)
- Test suite details: `TESTING.md` (project root)
- Documentation portal: doc.edgefirst.ai
