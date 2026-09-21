/*
 * Driver for ELAN press-type (touch) fingerprint sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * These sensors stream raw frames like the swipe sensors handled by the
 * elan driver, but the finger rests on the sensor instead of being swiped
 * across it. The imaged area is far too small for reliable minutiae
 * matching, so enrollment stores the images themselves and matching is
 * done by normalized cross-correlation, like the Windows driver does.
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

/* frames captured per touch; the finger is static so these only serve to
 * average out noise.
 *
 * Enrollment keeps the stricter minimum so the stored templates stay clean.
 * Verification accepts a single frame: a touch that yields fewer frames than
 * the minimum is discarded and silently retried, which to the user looks like
 * the sensor ignoring the finger rather than rejecting it. A quick tap that
 * only produces one noisy frame still correlates far above the threshold. */
#define ELANPRESS_MIN_FRAMES_ENROLL 3
#define ELANPRESS_MIN_FRAMES_VERIFY 1
#define ELANPRESS_MAX_FRAMES 10

/* number of touches stored during enrollment */
#define ELANPRESS_ENROLL_STAGES 8

/* grace period after the finger is first detected, before the frames that
 * will actually be matched are captured. The sensor images a small window of
 * the finger, so the correlation score is dominated by how well the touch is
 * centred rather than by whose finger it is; these sensors are often mounted
 * where the user cannot see them, and this gives them a moment to adjust. */
#define ELANPRESS_SETTLE_MS 400

/* separate presses averaged into one verify/identify decision.
 *
 * Resampling until something cleared the threshold made the verdict the best
 * of everything tried: the best of several presses, each scored against the
 * best of the enrolled images, each of those the best over a few thousand
 * candidate alignments. A maximum taken over that many chances lifts a
 * stranger's score about as readily as the owner's, which is the likely
 * reason the driver has been reported as accepting any finger.
 *
 * Averaging independent presses cancels placement noise instead of rewarding
 * a lucky one, so the genuine and impostor score distributions separate
 * rather than both drifting upward. The cost is one extra press per verify:
 * set this to 1 to go back to deciding on a single press. */
#define ELANPRESS_MATCH_SAMPLES 2

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
  {.vid = 0, .pid = 0, },
};
