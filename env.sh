#!/bin/bash
# env.sh - Setup development dependencies for local build
#
# Usage: source env.sh
#
# Downloads and extracts pre-compiled zenoh-c and edgefirst-schemas
# libraries for the host architecture, then sets PKG_CONFIG_PATH and
# LD_LIBRARY_PATH so that meson can find them.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$SCRIPT_DIR/deps/host"

ZENOH_C_VERSION="1.7.2"
SCHEMAS_VERSION="1.5.3"
HAL_VERSION="0.6.2"

ARCH="$(uname -m)"
case "$ARCH" in
  x86_64)
    ZENOH_C_TRIPLE="x86_64-unknown-linux-gnu"
    SCHEMAS_PLATFORM="linux_x86_64"
    ;;
  aarch64)
    ZENOH_C_TRIPLE="aarch64-unknown-linux-gnu"
    SCHEMAS_PLATFORM="linux_aarch64"
    ;;
  *)
    echo "error: unsupported architecture: $ARCH" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

ZENOH_C_ARCHIVE="zenoh-c-${ZENOH_C_VERSION}-${ZENOH_C_TRIPLE}-standalone.zip"
SCHEMAS_ARCHIVE="edgefirst-schemas-${SCHEMAS_PLATFORM}-${SCHEMAS_VERSION}.zip"
HAL_ARCHIVE="edgefirst-hal-capi-${HAL_VERSION}-${ARCH}-linux.tar.gz"

ZENOH_C_URL="https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_C_VERSION}/${ZENOH_C_ARCHIVE}"
SCHEMAS_URL="https://github.com/EdgeFirstAI/schemas/releases/download/v${SCHEMAS_VERSION}/${SCHEMAS_ARCHIVE}"
HAL_URL="https://github.com/EdgeFirstAI/hal/releases/download/v${HAL_VERSION}/${HAL_ARCHIVE}"

ZENOH_C_DIR="$DEPS_DIR/zenoh-c"
SCHEMAS_DIR="$DEPS_DIR/edgefirst-schemas"
HAL_DIR="$DEPS_DIR/edgefirst-hal"

# --- zenoh-c ---
if [ ! -f "$ZENOH_C_DIR/lib/libzenohc.so" ]; then
  mkdir -p "$ZENOH_C_DIR"
  echo "Downloading zenoh-c ${ZENOH_C_VERSION} (${ZENOH_C_TRIPLE})..."
  curl -sL "$ZENOH_C_URL" -o "/tmp/${ZENOH_C_ARCHIVE}"
  unzip -qo "/tmp/${ZENOH_C_ARCHIVE}" -d "$ZENOH_C_DIR"
  rm -f "/tmp/${ZENOH_C_ARCHIVE}"

  # Fix hardcoded prefix in pkg-config file
  sed -i "s|^prefix=.*|prefix=${ZENOH_C_DIR}|" "$ZENOH_C_DIR/lib/pkgconfig/zenohc.pc"
  echo "  -> ${ZENOH_C_DIR}"
fi

# --- edgefirst-schemas ---
if [ ! -f "$SCHEMAS_DIR/lib/libedgefirst_schemas.so" ]; then
  mkdir -p "$SCHEMAS_DIR"
  echo "Downloading edgefirst-schemas ${SCHEMAS_VERSION} (${SCHEMAS_PLATFORM})..."
  curl -sL "$SCHEMAS_URL" -o "/tmp/${SCHEMAS_ARCHIVE}"
  # Archive has a top-level directory; strip it
  unzip -qo "/tmp/${SCHEMAS_ARCHIVE}" -d "/tmp"
  cp -a "/tmp/edgefirst-schemas-${SCHEMAS_PLATFORM}-${SCHEMAS_VERSION}"/. "$SCHEMAS_DIR/"
  rm -rf "/tmp/${SCHEMAS_ARCHIVE}" "/tmp/edgefirst-schemas-${SCHEMAS_PLATFORM}-${SCHEMAS_VERSION}"

  # Generate pkg-config file (not shipped in the archive)
  mkdir -p "$SCHEMAS_DIR/lib/pkgconfig"
  cat > "$SCHEMAS_DIR/lib/pkgconfig/edgefirst-schemas.pc" <<PCEOF
prefix=${SCHEMAS_DIR}
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: edgefirst-schemas
Description: EdgeFirst message schemas C API
Version: ${SCHEMAS_VERSION}
Cflags: -I\${includedir}
Libs: -L\${libdir} -ledgefirst_schemas
PCEOF
  echo "  -> ${SCHEMAS_DIR}"
fi

# --- edgefirst-hal ---
if [ ! -f "$HAL_DIR/lib/libedgefirst_hal.so" ]; then
  mkdir -p "$HAL_DIR"
  echo "Downloading edgefirst-hal ${HAL_VERSION} (${ARCH})..."
  curl -sL "$HAL_URL" -o "/tmp/${HAL_ARCHIVE}"
  # Archive has a top-level directory; strip it
  tar xzf "/tmp/${HAL_ARCHIVE}" -C "/tmp"
  cp -a "/tmp/edgefirst-hal-capi-${HAL_VERSION}-${ARCH}-linux"/. "$HAL_DIR/"
  rm -rf "/tmp/${HAL_ARCHIVE}" "/tmp/edgefirst-hal-capi-${HAL_VERSION}-${ARCH}-linux"

  # Generate pkg-config file
  mkdir -p "$HAL_DIR/lib/pkgconfig"
  cat > "$HAL_DIR/lib/pkgconfig/edgefirst-hal.pc" <<PCEOF
prefix=${HAL_DIR}
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: edgefirst-hal
Description: EdgeFirst Hardware Abstraction Layer C API
Version: ${HAL_VERSION}
Cflags: -I\${includedir}
Libs: -L\${libdir} -ledgefirst_hal
PCEOF
  echo "  -> ${HAL_DIR}"
fi

# --- Environment ---
export PKG_CONFIG_PATH="${ZENOH_C_DIR}/lib/pkgconfig:${SCHEMAS_DIR}/lib/pkgconfig:${HAL_DIR}/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="${ZENOH_C_DIR}/lib:${SCHEMAS_DIR}/lib:${HAL_DIR}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "Development environment ready (${ARCH})."
