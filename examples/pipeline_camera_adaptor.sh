#!/bin/bash
# pipeline_camera_adaptor.sh - Camera preprocessing for ML inference
#
# Demonstrates edgefirstcameraadaptor with videotestsrc (works without hardware).
# Shows multiple configurations: uint8, int8, float32, letterbox.
#
# Usage: ./pipeline_camera_adaptor.sh [config]
#   config: uint8 (default), int8, float32, letterbox

set -e

CONFIG="${1:-uint8}"

case "$CONFIG" in
  uint8)
    echo "==> uint8 HWC: 640x480 RGB -> 320x320 uint8 tensor"
    gst-launch-1.0 \
      videotestsrc num-buffers=30 ! video/x-raw,format=RGB,width=640,height=480 \
      ! edgefirstcameraadaptor model-width=320 model-height=320 \
          model-dtype=uint8 model-colorspace=rgb model-layout=hwc \
      ! fakesink dump=true
    ;;

  int8)
    echo "==> int8 HWC: 640x480 RGB -> 320x320 int8 tensor (XOR 0x80 quantization)"
    gst-launch-1.0 \
      videotestsrc num-buffers=30 ! video/x-raw,format=RGB,width=640,height=480 \
      ! edgefirstcameraadaptor model-width=320 model-height=320 \
          model-dtype=int8 model-colorspace=rgb model-layout=hwc \
      ! fakesink dump=true
    ;;

  float32)
    echo "==> float32 HWC: 640x480 RGB -> 320x320 float32 tensor (ImageNet normalization)"
    gst-launch-1.0 \
      videotestsrc num-buffers=30 ! video/x-raw,format=RGB,width=640,height=480 \
      ! edgefirstcameraadaptor model-width=320 model-height=320 \
          model-dtype=float32 model-colorspace=rgb model-layout=hwc \
          model-mean="0.485,0.456,0.406" model-std="0.229,0.224,0.225" \
      ! fakesink dump=true
    ;;

  letterbox)
    echo "==> Letterbox: 640x480 RGB -> 320x320 uint8 (aspect-preserving + gray padding)"
    gst-launch-1.0 \
      videotestsrc num-buffers=30 ! video/x-raw,format=RGB,width=640,height=480 \
      ! edgefirstcameraadaptor model-width=320 model-height=320 \
          model-dtype=uint8 letterbox=true fill-color=0x808080FF \
      ! fakesink dump=true
    ;;

  *)
    echo "Unknown config: $CONFIG"
    echo "Usage: $0 [uint8|int8|float32|letterbox]"
    exit 1
    ;;
esac
