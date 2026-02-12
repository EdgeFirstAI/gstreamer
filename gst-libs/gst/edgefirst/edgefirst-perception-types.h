/*
 * EdgeFirst Perception for GStreamer
 * Copyright (C) 2026 Au-Zone Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EDGEFIRST_PERCEPTION_TYPES_H__
#define __EDGEFIRST_PERCEPTION_TYPES_H__

#include <glib.h>

G_BEGIN_DECLS

/**
 * EdgefirstRadarDimension:
 * @EDGEFIRST_RADAR_DIM_UNDEFINED: Undefined dimension
 * @EDGEFIRST_RADAR_DIM_RANGE: Range dimension (distance)
 * @EDGEFIRST_RADAR_DIM_DOPPLER: Doppler dimension (velocity)
 * @EDGEFIRST_RADAR_DIM_AZIMUTH: Azimuth dimension (horizontal angle)
 * @EDGEFIRST_RADAR_DIM_ELEVATION: Elevation dimension (vertical angle)
 * @EDGEFIRST_RADAR_DIM_RXCHANNEL: Receive channel dimension
 * @EDGEFIRST_RADAR_DIM_SEQUENCE: Sequence/frame dimension
 *
 * Dimension labels for radar cube data, matching EdgeFirst RadarCube message.
 */
typedef enum {
  EDGEFIRST_RADAR_DIM_UNDEFINED  = 0,
  EDGEFIRST_RADAR_DIM_RANGE      = 1,
  EDGEFIRST_RADAR_DIM_DOPPLER    = 2,
  EDGEFIRST_RADAR_DIM_AZIMUTH    = 3,
  EDGEFIRST_RADAR_DIM_ELEVATION  = 4,
  EDGEFIRST_RADAR_DIM_RXCHANNEL  = 5,
  EDGEFIRST_RADAR_DIM_SEQUENCE   = 6,
} EdgefirstRadarDimension;

/**
 * EdgefirstDistortionModel:
 * @EDGEFIRST_DISTORTION_NONE: No distortion
 * @EDGEFIRST_DISTORTION_PLUMB_BOB: Brown-Conrady model (k1,k2,p1,p2,k3)
 * @EDGEFIRST_DISTORTION_EQUIDISTANT: Fisheye model (k1,k2,k3,k4)
 * @EDGEFIRST_DISTORTION_RATIONAL: Rational polynomial model (8 coefficients)
 *
 * Camera distortion models for intrinsic calibration.
 */
typedef enum {
  EDGEFIRST_DISTORTION_NONE        = 0,
  EDGEFIRST_DISTORTION_PLUMB_BOB   = 1,
  EDGEFIRST_DISTORTION_EQUIDISTANT = 2,
  EDGEFIRST_DISTORTION_RATIONAL    = 3,
} EdgefirstDistortionModel;

/**
 * EDGEFIRST_RADAR_MAX_DIMS:
 *
 * Maximum number of dimensions in a radar cube.
 */
#define EDGEFIRST_RADAR_MAX_DIMS 8

/**
 * EDGEFIRST_MAX_DISTORTION_COEFFS:
 *
 * Maximum number of distortion coefficients.
 */
#define EDGEFIRST_MAX_DISTORTION_COEFFS 12

/**
 * EDGEFIRST_FRAME_ID_MAX_LEN:
 *
 * Maximum length of a coordinate frame identifier.
 */
#define EDGEFIRST_FRAME_ID_MAX_LEN 64

G_END_DECLS

#endif /* __EDGEFIRST_PERCEPTION_TYPES_H__ */
