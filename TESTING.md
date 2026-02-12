# Testing edgefirst-gstreamer

## Prerequisites

- GStreamer >= 1.20 development packages
- GStreamer Check (`gstreamer-check-1.0`) for test harness
- Meson >= 0.62 build system

On Debian/Ubuntu:

```sh
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-base meson ninja-build
```

## Build Configurations

The project supports three build configurations, controlled by Meson feature
options. Tests are built automatically when `gstreamer-check-1.0` is available.

### Core + Fusion (no external sensor dependencies)

```sh
meson setup builddir -Dzenoh=disabled -Dhal=disabled
meson compile -C builddir
```

### Core + Fusion + HAL

```sh
source env.sh
meson setup builddir -Dzenoh=disabled -Dhal=enabled
meson compile -C builddir
```

### Full build (requires zenoh-c, edgefirst-schemas, edgefirst-hal)

```sh
source env.sh
meson setup builddir
meson compile -C builddir
```

### Clean rebuild

```sh
rm -rf builddir
source env.sh
meson setup builddir
meson compile -C builddir
```

## Running Tests

### All tests

```sh
meson test -C builddir
```

### Verbose output (shows individual test assertions)

```sh
meson test -C builddir -v
```

### Single test suite

```sh
meson test -C builddir meta
meson test -C builddir meta_copy
meson test -C builddir math
meson test -C builddir fusion_elements
meson test -C builddir hal_elements
```

### GStreamer debug output during tests

```sh
GST_DEBUG=edgefirst*:5 meson test -C builddir -v
```

## Test Suites

### `meta` -- Metadata Type Tests

**File**: `tests/check/test_meta.c` (5 tests)

| Test | Description |
|------|-------------|
| `test_pointcloud2_meta_create` | Create buffer, attach EdgefirstPointCloud2Meta, verify fields |
| `test_radar_cube_meta_create` | Create buffer, attach EdgefirstRadarCubeMeta, verify fields |
| `test_camera_info_meta_project` | Pinhole projection of a 3D point using EdgefirstCameraInfoMeta |
| `test_transform_apply` | Apply quaternion rotation via EdgefirstTransformData |
| `test_radar_dimension_to_string` | Verify all EdgefirstRadarDimension enum string conversions |

### `meta_copy` -- Metadata Copy/Transform Tests

**File**: `tests/check/test_meta_copy.c` (7 tests)

| Test | Description |
|------|-------------|
| `test_pointcloud2_meta_copy` | Copy buffer, verify EdgefirstPointCloud2Meta is preserved |
| `test_radar_cube_meta_copy` | Copy buffer, verify EdgefirstRadarCubeMeta is preserved |
| `test_camera_info_meta_copy` | Copy buffer, verify EdgefirstCameraInfoMeta is preserved |
| `test_transform_meta_copy` | Copy buffer, verify EdgefirstTransformMeta is preserved |
| `test_meta_absent_on_empty_buffer` | Verify no metadata on a fresh buffer |
| `test_multiple_meta_types_on_buffer` | Attach multiple meta types to one buffer |
| `test_meta_init_defaults` | Verify default values after metadata initialization |

### `math` -- Mathematical Operations Tests

**File**: `tests/check/test_math.c` (26 tests)

**Transform rotation tests (10):**

| Test | Description |
|------|-------------|
| `test_transform_identity_no_translation` | Identity quaternion, zero translation |
| `test_transform_translation_only` | Identity quaternion with non-zero translation |
| `test_transform_90deg_about_z` | 90-degree rotation about Z axis |
| `test_transform_90deg_about_x` | 90-degree rotation about X axis |
| `test_transform_90deg_about_y` | 90-degree rotation about Y axis |
| `test_transform_180deg_about_z` | 180-degree rotation about Z axis |
| `test_transform_rotation_plus_translation` | Combined rotation and translation |
| `test_transform_45deg_about_z` | 45-degree rotation about Z axis |
| `test_transform_arbitrary_rotation` | Non-axis-aligned rotation |
| `test_transform_identity_frame_ids` | Verify frame ID string handling |

**Camera projection tests (8):**

| Test | Description |
|------|-------------|
| `test_camera_identity_project_center` | Project point at optical center |
| `test_camera_identity_project_known_values` | Project with known focal length/principal point |
| `test_camera_project_behind_z_zero` | Handle point at z=0 (degenerate case) |
| `test_camera_project_negative_z` | Handle point behind camera |
| `test_camera_set_identity_k_matrix` | Verify identity intrinsic matrix setup |
| `test_camera_set_identity_p_matrix` | Verify identity projection matrix setup |
| `test_camera_set_identity_r_matrix` | Verify identity rectification matrix setup |
| `test_camera_project_with_custom_k` | Projection with non-trivial intrinsics |

**Version tests (1):**

| Test | Description |
|------|-------------|
| `test_perception_version` | `edgefirst_perception_version()` returns non-empty string |

**Point field parsing tests (4):**

| Test | Description |
|------|-------------|
| `test_parse_point_fields_xyz` | Parse standard x:F32, y:F32, z:F32 fields |
| `test_parse_point_fields_roundtrip` | Parse and re-format fields string matches original |
| `test_point_field_datatype_size` | All datatype sizes correct (I8→1, F64→8, etc.) |
| `test_parse_point_fields_empty` | NULL and empty strings return 0 fields |

**Enum tests (2):**

| Test | Description |
|------|-------------|
| `test_radar_dimension_all_values` | All EdgefirstRadarDimension values have string names |
| `test_radar_dimension_out_of_range` | Out-of-range dimension returns "UNDEFINED" |

**Initialization tests (1):**

| Test | Description |
|------|-------------|
| `test_perception_init_idempotent` | `edgefirst_perception_init()` can be called multiple times |

### `fusion_elements` -- Fusion Plugin Element Tests

**File**: `tests/check/test_fusion_elements.c` (14 tests)

| Test | Description |
|------|-------------|
| `test_pcd_classify_create` | Element factory creates edgefirstpcdclassify |
| `test_transform_inject_create` | Element factory creates edgefirsttransforminject |
| `test_pcd_classify_output_mode_property` | Get/set output-mode enum property |
| `test_transform_inject_properties` | Get/set calibration-file, frame-id, parent-frame-id |
| `test_pcd_classify_pad_templates` | Verify sink_cloud, sink_mask, src pad templates and caps |
| `test_transform_inject_pad_templates` | Verify sink/src pad templates (ANY caps) |
| `test_transform_inject_not_passthrough` | Confirm passthrough is disabled (metadata injection) |
| `test_transform_inject_is_in_place` | Confirm in-place transform mode |
| `test_pcd_classify_element_metadata` | Verify element description and classification strings |
| `test_transform_inject_element_metadata` | Verify element description and classification strings |
| `test_transform_inject_load_calibration` | Load valid calibration JSON during READY→PAUSED |
| `test_transform_inject_load_invalid` | Reject invalid calibration JSON (state change fails) |
| `test_transform_inject_state_null_to_ready` | State transition NULL to READY succeeds |
| `test_pcd_classify_state_null_to_ready` | State transition NULL to READY succeeds |

### `hal_elements` -- HAL Plugin Element Tests

**File**: `tests/check/test_hal_elements.c` (22 tests)

**Element tests (7):**

| Test | Description |
|------|-------------|
| `test_camera_adaptor_create` | Element factory creates edgefirstcameraadaptor |
| `test_camera_adaptor_properties` | Get/set all properties (model-width, model-height, model-colorspace, model-layout, model-dtype, letterbox, fill-color, model-mean, model-std) |
| `test_camera_adaptor_pad_templates` | Verify sink and src pad templates present |
| `test_camera_adaptor_element_metadata` | Verify element classification and author strings |
| `test_camera_adaptor_not_passthrough` | Confirm passthrough_on_same_caps is FALSE |
| `test_camera_adaptor_not_in_place` | Confirm element is not in-place (different input/output formats) |
| `test_camera_adaptor_state_null_to_ready` | State transitions NULL→READY→NULL succeed |

**Pipeline tests (6):**

| Test | Description |
|------|-------------|
| `test_camera_adaptor_rgb_passthrough` | 320x240 RGB → 320x240 uint8 HWC (230400 bytes) |
| `test_camera_adaptor_resize` | 640x480 RGB → 320x320 uint8 HWC (307200 bytes) |
| `test_camera_adaptor_letterbox` | 640x480 RGB → 320x320 uint8 HWC letterbox (307200 bytes) |
| `test_camera_adaptor_int8` | 320x240 RGB → int8 tensor (230400 bytes) |
| `test_camera_adaptor_chw_layout` | 320x240 RGB → uint8 CHW tensor (230400 bytes) |
| `test_camera_adaptor_float32` | 320x240 RGB → float32 tensor (921600 bytes) |

**NEON kernel tests (6):**

| Test | Description |
|------|-------------|
| `test_neon_rgba_to_rgb_u8` | RGBA to RGB uint8 conversion, alpha dropped |
| `test_neon_rgba_to_rgb_u8_bgr` | RGBA to BGR uint8 conversion (R/B swap) |
| `test_neon_rgba_to_rgb_i8` | RGBA to RGB int8 conversion (XOR 0x80 quantization) |
| `test_neon_planar_u8_to_i8` | Planar uint8 to int8 conversion (XOR 0x80) |
| `test_neon_rgba_to_rgb_f32` | RGBA to RGB float32 with mean/std normalization |
| `test_neon_large_buffer` | 640x640 pixel buffer tests NEON/scalar boundary handling |

**Pipeline content tests (3):**

| Test | Description |
|------|-------------|
| `test_camera_adaptor_chw_int8` | CHW + int8 (Ara-2 workflow): 320x240 → 230400 bytes |
| `test_camera_adaptor_bgr_hwc` | BGR HWC uint8: 320x240 → 230400 bytes |
| `test_camera_adaptor_letterbox_properties` | Verify letterbox-scale, letterbox-top, letterbox-left after negotiation |

**Note**: HAL tests require edgefirst-hal to be available at build time. The
pipeline tests use `videotestsrc` and work without hardware. NEON kernel tests
use scalar fallback implementations on non-ARM platforms.

## Planned Tests (Not Yet Implemented)

The following test suites will be added as their corresponding modules are implemented:

| Suite | File | Module | Tests |
|-------|------|--------|-------|
| CDR round-trips | `tests/check/test_cdr.c` | Zenoh | PointCloud2, RadarCube, Image, CameraInfo, Transform serialization |
| Zenoh elements | `tests/check/test_zenoh_elements.c` | Zenoh | Element creation, properties, error handling |
| Transform cache | `tests/check/test_transform_cache.c` | Zenoh | Insert/lookup, overwrite, thread safety |
| Fusion pipeline | `tests/check/test_fusion.c` | Fusion | Calibration JSON parsing, projection math |
| GFX elements | `tests/check/test_gfx.c` | GFX | (roadmap — GFX module not yet in git) |
| Pipeline integration | `tests/pipelines/test_pipeline_*.c` | All | End-to-end pipeline tests |

## Test Fixtures

Test fixture files in `tests/fixtures/`:

| File | Status | Purpose |
|------|--------|---------|
| `test_calibration.json` | Exists | Camera intrinsics + extrinsic transform |
| `test_calibration_invalid.json` | Exists | Malformed JSON for error handling tests |
| `test_calibration_transform_only.json` | Planned | Transform-only calibration |
