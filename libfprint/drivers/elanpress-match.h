/*
 * Image processing and correlation matching for ELAN press-type sensors
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

#include "elanpress-sift.h"

/* frames dropped at the start (finger settling) and end (possible lift)
 * of a touch when enough frames are available */
#define ELANPRESS_SKIP_OLDEST 2
#define ELANPRESS_SKIP_NEWEST 1

/* A press is matched against an enrolled image by pairing up keypoints whose
 * descriptors are each other's clear nearest neighbour, then counting how
 * many of those pairs agree on a single rigid placement of one image on the
 * other. Unrelated fingers still pair up a handful of keypoints by chance,
 * but their placements disagree, so they reach only a few such pairs.
 *
 * Measured on labelled presses from the reference sensor (40 genuine, 30
 * from five other fingers, against 8 to 12 enrolled images): the other
 * fingers reached at most 4 pairs, while 97-99% of genuine presses reached 12
 * or more and half of them over 60. Correlating whole images instead (the
 * previous matcher) left the two overlapping on the same data: at 0.68 it
 * rejected about 1 genuine press in 10 and still accepted the odd impostor. */
#define ELANPRESS_MATCH_MIN_PAIRS 12

/* the nearest descriptor must be closer than this fraction of the distance
 * to the second nearest to count as a pair (Lowe's ratio test) */
#define ELANPRESS_MATCH_RATIO 0.8

/* how far a keypoint may land from its partner under the placement, in
 * pixels: the first value gathers support for a placement proposed by a
 * single pair, the second decides the final count after refitting it */
#define ELANPRESS_MATCH_LOOSE_PX 6.0
#define ELANPRESS_MATCH_TOL_PX 3.0

/* proposed placements refined by least squares, best supported first */
#define ELANPRESS_MATCH_REFINED 8

/* keypoints kept per image, strongest first */
#define ELANPRESS_MAX_KEYPOINTS 300

/* Presence is read out of the image, because the status byte cmd_pre_scan
 * returns answers once per power-up and then wedges at "finger present".
 *
 * A pixel counts as covered when it sits this far above the background, and
 * the frame counts as touched once that many of them are. Counting covered
 * pixels rather than averaging the delta over the whole sensor is what keeps
 * a lightly resting finger from being diluted by the untouched area around
 * it: a finger landing on a third of the pad drives those pixels hard but
 * moves the whole-sensor mean very little, so a mean test only notices once
 * the finger is pressed hard or slid across to cover more of the pad. */
#define ELANPRESS_TOUCH_PIXEL_DELTA 1200
#define ELANPRESS_TOUCH_MIN_COVERAGE 0.12

/* what a frame looked like against the background, for tuning the two
 * constants above from fp_dbg output */
typedef struct
{
  gdouble mean_delta;
  gdouble coverage;
} ElanpressTouchStats;

/* a processed touch whose contrast (pixel std-dev) falls below this carries
 * too little ridge detail to enroll or to match against: a light or partial
 * press correlates poorly against everything, genuine or not. A well-imaged
 * press lands around 30-50. Rejecting these is what makes a bad press fail
 * rather than score like a stranger's finger. */
#define ELANPRESS_MIN_QUALITY_STDDEV 10.0

void     elanpress_rotate_frame (const guint8 *raw, unsigned short *out,
                                 int w, int h);
guint8 * elanpress_process_frames (GSList *frames, int num_frames,
                                   const unsigned short *background,
                                   unsigned int size);
ElanpressFeatures * elanpress_image_features (const guint8 *img, int w, int h);
guint    elanpress_match_features (const ElanpressFeatures *probe,
                                   const ElanpressFeatures *enrolled);
gboolean elanpress_frame_has_touch (const unsigned short *frame,
                                    const unsigned short *background,
                                    unsigned int size,
                                    ElanpressTouchStats *stats);
gdouble  elanpress_image_quality (const guint8 *img, unsigned int size);
