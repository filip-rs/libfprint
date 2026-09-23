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

/* frames dropped at the start (finger settling) and end (possible lift)
 * of a touch when enough frames are available */
#define ELANPRESS_SKIP_OLDEST 2
#define ELANPRESS_SKIP_NEWEST 1

/* matching parameters, tuned offline on labelled touches from an ASUS
 * 04f3:0c6e (16-image enrollment, 11 genuine vs 10 impostor probes of
 * 7 other fingers): genuine scored 0.81-0.93, impostors at most 0.66.
 * Translation-only NCC without masking/high-pass overlapped (FRR 36%). */
#define ELANPRESS_NCC_THRESHOLD 0.74
#define ELANPRESS_NCC_MAX_DX 60
#define ELANPRESS_NCC_MAX_DY 20
#define ELANPRESS_NCC_MIN_OVERLAP_PX 4000
#define ELANPRESS_NCC_STEP 4
#define ELANPRESS_ROT_MAX_DEG 20
#define ELANPRESS_ROT_STEP_DEG 5
#define ELANPRESS_HP_SIGMA 4.0f
#define ELANPRESS_MASK_SIGMA 2.0f
#define ELANPRESS_MASK_LEVEL 25

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
gdouble  elanpress_ncc_best (const guint8 *a, const guint8 *b,
                             int w, int h);
gboolean elanpress_frame_has_touch (const unsigned short *frame,
                                    const unsigned short *background,
                                    unsigned int size,
                                    ElanpressTouchStats *stats);
gdouble  elanpress_image_quality (const guint8 *img, unsigned int size);
