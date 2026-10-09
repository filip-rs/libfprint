/*
 * Driver for ELAN press-type (touch) fingerprint sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * These sensors stream raw frames like the swipe sensors handled by the
 * elan driver, but the finger rests on the sensor instead of being swiped
 * across it. The imaged area is far too small for reliable minutiae
 * matching, so enrollment stores the images themselves and matching pairs
 * up local keypoint features between them, like the Windows driver does.
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

#include "fpi-device.h"
#include "fpi-usb-transfer.h"

#define ELANPRESS_VEND_ID 0x04f3

#define ELANPRESS_EP_CMD_OUT (0x1 | FPI_USB_ENDPOINT_OUT)
#define ELANPRESS_EP_CMD_IN (0x3 | FPI_USB_ENDPOINT_IN)
#define ELANPRESS_EP_IMG_IN (0x2 | FPI_USB_ENDPOINT_IN)

#define ELANPRESS_CMD_LEN 2
#define ELANPRESS_CMD_TIMEOUT 5000
#define ELANPRESS_FRAME_TIMEOUT 2000

/* interval between polls while frames of a touch are being collected */
#define ELANPRESS_POLL_INTERVAL_MS 30

/* interval between polls while waiting for a finger to land or lift. Each
 * poll pulls a whole frame rather than the single status byte it used to, so
 * the transfer already paces the loop; a further delay on top of it only
 * adds latency to noticing the finger. */
#define ELANPRESS_IDLE_POLL_INTERVAL_MS 5

/* consecutive untouched frames that end a touch. Presence is inferred from
 * the image itself (see ELANPRESS_TOUCH_MIN_MEAN_DELTA) rather than asked of
 * cmd_pre_scan, whose status byte answers once per power-up and then wedges
 * at "finger present" for good, stranding capture in a wait-for-lift loop
 * that only a full power cycle could clear. Requiring a run of clear frames
 * keeps sensor noise from ending a touch early, and keeps a finger left
 * resting on the sensor from being counted as the next press. */
#define ELANPRESS_FINGER_OFF_FRAMES 10

/* consecutive clear polls after which the background frame is retaken while
 * waiting for a press. The background is the only reference for telling a
 * touch apart, so one captured while a finger happened to be resting would
 * make every later frame look untouched and strand capture just as the
 * wedged status byte used to. Retaking it on a long clear stretch recovers
 * from that by itself, and otherwise just tracks the sensor's drift. */
#define ELANPRESS_BG_REFRESH_FRAMES 50

/* a touch is imaged once, ELANPRESS_SETTLE_MS after the finger is first
 * detected. The sensor's frames are nearly noise-free (consecutive frames of
 * a resting finger correlate at 0.999), so averaging several buys nothing,
 * but it does smear the ridges whenever the finger shifts while resting: on
 * recorded presses that happened in about one press in ten, between 170 ms
 * and 900 ms after landing, and averaged frames from such presses failed to
 * match where a single frame of the same press did. */
#define ELANPRESS_MAX_FRAMES 1

/* number of touches stored during enrollment. The sensor sees a different
 * part of the finger on each press, and a press only matches an enrolled one
 * it overlaps, so more touches cover more of the finger: measured on
 * recorded presses, 1 genuine press in 40 failed to match against 8 stored
 * touches, 1 in 80 against 12 and 1 in 170 against 16. */
#define ELANPRESS_ENROLL_STAGES 12

/* time between detecting the finger and imaging it. The first frames catch it
 * still landing, with only part of it in contact; from about 100 ms on the
 * image is as good as any later one. */
#define ELANPRESS_SETTLE_MS 200

/* separate presses averaged into one verify/identify decision.
 *
 * Resampling until some press cleared the threshold made the verdict the best
 * of everything tried: the best of several presses, each scored against the
 * best of the enrolled images, each of those the best over a few thousand
 * candidate alignments. A maximum taken over that many chances lifts a
 * stranger's score about as readily as the owner's, which is the likely
 * reason the driver has been reported as accepting any finger.
 *
 * Dropping that loop is what fixed it, not the averaging. A single press
 * already sits clearly either side of the threshold (see
 * ELANPRESS_MATCH_MIN_PAIRS), so averaging further presses only buys margin
 * against an unusually poor genuine one, and on a lockscreen that margin is
 * better spent on convenience: a false reject there costs one more press,
 * whereas a second mandatory press costs one every single time. Raise this
 * if the score distributions on some other sensor turn out to sit closer
 * together. */
#define ELANPRESS_MATCH_SAMPLES 1

/* extra presses allowed when one yields no usable image at all: too few
 * frames, or too little contrast to be worth scoring. These do not count
 * towards the average, because a press that produced nothing is not evidence
 * either way, whereas one that produced a poor score is. */
#define ELANPRESS_MATCH_MAX_RETRIES 3

/* version tag for the serialized print data */
#define ELANPRESS_PRINT_VERSION 1

G_DECLARE_FINAL_TYPE (FpiDeviceElanPress, fpi_device_elanpress, FPI,
                      DEVICE_ELANPRESS, FpDevice);

static const FpIdEntry elanpress_id_table[] = {
  {.vid = ELANPRESS_VEND_ID, .pid = 0x0c6e, },
  /* 04f3:0c4f Acer Aspire A515-45: press sensor; stock elan driver mislabels it swipe */
  {.vid = ELANPRESS_VEND_ID, .pid = 0x0c4f, },
  {.vid = 0, .pid = 0, },
};
