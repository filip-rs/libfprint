/*
 * Keypoint features for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <glib.h>

#define ELANPRESS_SIFT_DESC_LEN 128

typedef struct
{
  /* position in input image pixels */
  gfloat x, y;
  /* dominant gradient direction in radians, measured in image coordinates
   * (y pointing down): rotating an image by R(t) = [[cos t, -sin t],
   * [sin t, cos t]] adds t to the angle of every keypoint on it */
  gfloat angle;
  /* blur, in input image pixels, of the scale the keypoint was found at */
  gfloat scale;
  /* contrast of the extremum, for keeping the strongest when capped */
  gfloat response;
  guint8 desc[ELANPRESS_SIFT_DESC_LEN];
} ElanpressKeypoint;

typedef struct
{
  ElanpressKeypoint *kp;
  guint              n;
} ElanpressFeatures;

ElanpressFeatures *elanpress_sift_features (const guint8 *img,
                                            int           width,
                                            int           height,
                                            guint         max_keypoints);
void               elanpress_features_free (ElanpressFeatures *features);

G_DEFINE_AUTOPTR_CLEANUP_FUNC (ElanpressFeatures, elanpress_features_free)
