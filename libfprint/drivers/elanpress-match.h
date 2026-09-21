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

/* matching parameters, validated against captures of the Windows driver:
 * genuine presses of the same finger region correlate at 0.59-0.91 while
 * impostor images stay below 0.47 */
#define ELANPRESS_NCC_THRESHOLD 0.55
#define ELANPRESS_NCC_MAX_DX 60
#define ELANPRESS_NCC_MAX_DY 20
#define ELANPRESS_NCC_MIN_OVERLAP_PX 1500

/* mean positive per-pixel delta against the background frame above which a
 * frame counts as "touched". The status byte cmd_pre_scan returns answers
 * once per power-up and then wedges at "finger present" for good, so presence
 * has to be read out of the image instead. Live capture measured ~450 with no
 * finger on the sensor and ~4800 with one pressed. */
#define ELANPRESS_TOUCH_MIN_MEAN_DELTA 1500

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
                                    unsigned int size);
gdouble  elanpress_image_quality (const guint8 *img, unsigned int size);
