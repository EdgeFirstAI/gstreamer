#!/bin/bash
# pipeline_radar_inference.sh - Radar cube inference pipeline
#
# Subscribes to a Zenoh RadarCube topic, runs inference via NNStreamer
# tensor_filter, and publishes results back to Zenoh.
#
# Prerequisites:
#   - Zenoh router running (zenohd)
#   - Radar publisher on the configured topic (e.g., EdgeFirst radarpub)
#   - NNStreamer with TensorFlow Lite support
#   - A radar detection model (e.g., radar_detector.tflite)
#
# Usage: ./pipeline_radar_inference.sh [model] [in_topic] [out_topic]
#   model:     Path to TFLite model (default: radar_detector.tflite)
#   in_topic:  Input Zenoh topic (default: rt/radar/cube)
#   out_topic: Output Zenoh topic (default: rt/radar/detections)

set -e

MODEL="${1:-radar_detector.tflite}"
IN_TOPIC="${2:-rt/radar/cube}"
OUT_TOPIC="${3:-rt/radar/detections}"
SESSION="tcp/127.0.0.1:7447"

if [ ! -f "$MODEL" ]; then
  echo "error: model file not found: $MODEL"
  echo "Provide a TFLite radar detection model."
  exit 1
fi

echo "==> Radar inference: ${IN_TOPIC} -> ${MODEL} -> ${OUT_TOPIC}"

gst-launch-1.0 \
  edgefirstzenohsub topic="${IN_TOPIC}" message-type=radarcube session="${SESSION}" \
  ! tensor_filter framework=tensorflow2-lite model="${MODEL}" \
  ! tensor_decoder mode=bounding_boxes \
  ! edgefirstzenohpub topic="${OUT_TOPIC}" message-type=image session="${SESSION}"
