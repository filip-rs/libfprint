/*
 * Image processing and correlation matching for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * The imaged area of these sensors is far too small for reliable minutiae
 * matching, so prints store the enrolled images themselves and matching is
 * done by zero-mean normalized cross-correlation over translations, like
 * the Windows driver does.
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

#include "elanpress-match.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int
elanpress_cmp_short (const void *a, const void *b)
{
  return (int) (*(unsigned short *) a - *(unsigned short *) b);
}

/* the wire format is column-major with frame-height-tall columns; rotate
 * to row-major (same layout as the elan driver uses for these sensors) */
void
elanpress_rotate_frame (const guint8 *raw, unsigned short *out, int w, int h)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      out[y * w + x] = ((const guint16 *) raw)[y + x * h];
}

/* Elantech's recommended 2-step non-linear normalization of the 14-bit
 * background-subtracted frame onto 8 bits (see the elan driver) */
static void
elanpress_normalize (const unsigned short *in, guint8 *out, unsigned int size)
{
  g_autofree unsigned short *sorted = g_malloc (size * sizeof (short));
  unsigned short lvl0, lvl1, lvl2, lvl3;
  unsigned short px;

  memcpy (sorted, in, size * sizeof (short));
  qsort (sorted, size, sizeof (short), elanpress_cmp_short);

  lvl0 = sorted[0];
  lvl1 = sorted[size * 3 / 10];
  lvl2 = sorted[size * 65 / 100];
  lvl3 = sorted[size - 1];

  for (unsigned int i = 0; i < size; i++)
    {
      px = in[i];
      if (px < lvl1)
        px = (px - lvl0) * 99 / MAX (lvl1 - lvl0, 1);
      else if (px < lvl2)
        px = 99 + ((px - lvl1) * 56 / MAX (lvl2 - lvl1, 1));
      else
        px = 155 + MIN ((px - lvl2) * 100 / MAX (lvl3 - lvl2, 1), 100);
      out[i] = (guint8) px;
    }
}

/* average the stable frames of a touch (newest-first list), subtract the
 * background and normalize; returns NULL if the finger wasn't properly on
 * the sensor */
guint8 *
elanpress_process_frames (GSList *frames, int num_frames,
                          const unsigned short *background,
                          unsigned int size)
{
  g_autofree unsigned int *sum = g_malloc0 (size * sizeof (unsigned int));
  g_autofree unsigned short *avg = g_malloc (size * sizeof (unsigned short));
  guint8 *out;
  int skip_oldest = 0, skip_newest = 0, used = 0, i;
  unsigned long total = 0;
  GSList *l;

  if (num_frames >= 6)
    {
      skip_oldest = ELANPRESS_SKIP_OLDEST;
      skip_newest = ELANPRESS_SKIP_NEWEST;
    }
  else if (num_frames >= 4)
    {
      skip_oldest = 1;
      skip_newest = 1;
    }

  for (l = frames, i = 0; l; l = l->next, i++)
    {
      const unsigned short *frame = l->data;

      if (i < skip_newest || i >= num_frames - skip_oldest)
        continue;
      for (unsigned int j = 0; j < size; j++)
        sum[j] += frame[j];
      used++;
    }

  if (used == 0)
    return NULL;

  for (unsigned int j = 0; j < size; j++)
    {
      unsigned int px = sum[j] / used;
      unsigned short bg = background ? background[j] : 0;

      avg[j] = px > bg ? px - bg : 0;
      total += avg[j];
    }

  if (total == 0)
    {
      g_debug ("touch darker than background, ignoring");
      return NULL;
    }

  out = g_malloc (size);
  elanpress_normalize (avg, out, size);
  return out;
}

/* standard deviation of the pixel values of a normalized touch image, as a
 * cheap proxy for how much ridge contrast it actually holds. Flat captures
 * correlate poorly against everything, genuine or not, so it is better to
 * reject them at capture time than to store one or match against it. */
gdouble
elanpress_image_quality (const guint8 *img, unsigned int size)
{
  gdouble sum = 0, sum_sq = 0, mean;

  if (!img || size == 0)
    return 0.0;

  for (unsigned int i = 0; i < size; i++)
    sum += img[i];
  mean = sum / size;

  for (unsigned int i = 0; i < size; i++)
    {
      gdouble d = img[i] - mean;

      sum_sq += d * d;
    }

  return sqrt (sum_sq / size);
}

/* counts how many pixels sit clearly above the background frame, to infer a
 * touch without asking cmd_pre_scan (see the capture state machine) */
gboolean
elanpress_frame_has_touch (const unsigned short *frame,
                           const unsigned short *background,
                           unsigned int size,
                           ElanpressTouchStats *stats)
{
  gint64 sum = 0;
  unsigned int covered = 0;

  if (stats)
    {
      stats->mean_delta = 0;
      stats->coverage = 0;
    }

  /* without a reference frame presence cannot be told apart from an evenly
   * lit sensor, so report no touch rather than guess */
  if (!frame || !background || size == 0)
    return FALSE;

  for (unsigned int i = 0; i < size; i++)
    {
      int d = (int) frame[i] - (int) background[i];

      if (d <= 0)
        continue;

      sum += d;
      if (d >= ELANPRESS_TOUCH_PIXEL_DELTA)
        covered++;
    }

  if (stats)
    {
      stats->mean_delta = (gdouble) sum / size;
      stats->coverage = (gdouble) covered / size;
    }

  return covered >= size * ELANPRESS_TOUCH_MIN_COVERAGE;
}

/* separable gaussian blur with mirrored borders */
static void
elanpress_blur (const float *in, float *out, int w, int h, float sigma)
{
  int r = (int) ceilf (3 * sigma);
  g_autofree float *k = g_new (float, 2 * r + 1);
  g_autofree float *tmp = g_new (float, w * h);
  float ksum = 0;

  for (int i = -r; i <= r; i++)
    ksum += k[i + r] = expf (-(i * i) / (2 * sigma * sigma));
  for (int i = 0; i < 2 * r + 1; i++)
    k[i] /= ksum;

#define MIRROR(v, n) ((v) < 0 ? -(v) - 1 : (v) >= (n) ? 2 * (n) - (v) - 1 : (v))
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float acc = 0;
        for (int i = -r; i <= r; i++)
          acc += k[i + r] * in[y * w + MIRROR (x + i, w)];
        tmp[y * w + x] = acc;
      }
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float acc = 0;
        for (int i = -r; i <= r; i++)
          acc += k[i + r] * tmp[MIRROR (y + i, h) * w + x];
        out[y * w + x] = acc;
      }
#undef MIRROR
}

/* high-pass the image so only ridge detail is correlated (the contact blob
 * and pressure gradient look alike on every finger), and mask out the parts
 * of the sensor the finger didn't touch */
static void
elanpress_prep (const guint8 *img, float *hp, guint8 *mask, int w, int h)
{
  int n = w * h;
  g_autofree float *f = g_new (float, n);
  g_autofree float *blur = g_new (float, n);

  for (int i = 0; i < n; i++)
    f[i] = img[i];

  elanpress_blur (f, blur, w, h, ELANPRESS_MASK_SIGMA);
  for (int i = 0; i < n; i++)
    mask[i] = blur[i] > ELANPRESS_MASK_LEVEL;

  elanpress_blur (f, blur, w, h, ELANPRESS_HP_SIGMA);
  for (int i = 0; i < n; i++)
    hp[i] = f[i] - blur[i];
}

/* rotate about the centre into a same-sized canvas; pixels that come from
 * outside the source (or from masked-out source) are masked out */
static void
elanpress_rotate (const float *in, const guint8 *min, float *out, guint8 *mout,
                  int w, int h, float deg)
{
  float c = cosf (deg * G_PI / 180), s = sinf (deg * G_PI / 180);
  float cx = (w - 1) / 2.0f, cy = (h - 1) / 2.0f;

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float sx = c * (x - cx) + s * (y - cy) + cx;
        float sy = -s * (x - cx) + c * (y - cy) + cy;
        int x0 = (int) floorf (sx), y0 = (int) floorf (sy);
        float fx = sx - x0, fy = sy - y0;
        int o = y * w + x;

        if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h ||
            !min[(int) roundf (sy) * w + (int) roundf (sx)])
          {
            out[o] = 0;
            mout[o] = 0;
            continue;
          }
        out[o] = (1 - fy) * ((1 - fx) * in[y0 * w + x0] + fx * in[y0 * w + x0 + 1]) +
                 fy * ((1 - fx) * in[(y0 + 1) * w + x0] + fx * in[(y0 + 1) * w + x0 + 1]);
        mout[o] = 1;
      }
}

/* zero-mean NCC of b shifted by (dx, dy) against a, over the pixels both
 * masks cover; -1 when that overlap is too small to mean anything */
static gdouble
elanpress_ncc_at (const float *a, const guint8 *ma, const float *b,
                  const guint8 *mb, int w, int h, int dx, int dy)
{
  int x0 = MAX (0, dx), x1 = MIN (w, w + dx);
  int y0 = MAX (0, dy), y1 = MIN (h, h + dy);
  gdouble sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0, num, den;
  long n = 0;

  for (int y = y0; y < y1; y++)
    for (int x = x0; x < x1; x++)
      {
        int ia = y * w + x, ib = (y - dy) * w + (x - dx);

        if (!ma[ia] || !mb[ib])
          continue;
        sa += a[ia];
        sb += b[ib];
        saa += a[ia] * a[ia];
        sbb += b[ib] * b[ib];
        sab += a[ia] * b[ib];
        n++;
      }

  if (n < ELANPRESS_NCC_MIN_OVERLAP_PX)
    return -1.0;

  num = sab - sa * sb / n;
  den = (saa - sa * sa / n) * (sbb - sb * sb / n);
  if (den <= 0)
    return -1.0;

  return num / sqrt (den);
}

/* best masked NCC of the probe a against enrolled image b over rotations of
 * the probe and translations: coarse grid, then refinement around the best
 * hit of each rotation */
gdouble
elanpress_ncc_best (const guint8 *a, const guint8 *b, int w, int h)
{
  int n = w * h;
  g_autofree float *ha = g_new (float, n), *hb = g_new (float, n), *ra = g_new (float, n);
  g_autofree guint8 *ma = g_new (guint8, n), *mb = g_new (guint8, n), *rm = g_new (guint8, n);
  gdouble best = -1.0;

  elanpress_prep (a, ha, ma, w, h);
  elanpress_prep (b, hb, mb, w, h);

  for (int deg = -ELANPRESS_ROT_MAX_DEG; deg <= ELANPRESS_ROT_MAX_DEG;
       deg += ELANPRESS_ROT_STEP_DEG)
    {
      gdouble abest = -1.0, c;
      int bdx = 0, bdy = 0;

      elanpress_rotate (ha, ma, ra, rm, w, h, deg);

      for (int dy = -ELANPRESS_NCC_MAX_DY; dy <= ELANPRESS_NCC_MAX_DY; dy += ELANPRESS_NCC_STEP)
        for (int dx = -ELANPRESS_NCC_MAX_DX; dx <= ELANPRESS_NCC_MAX_DX; dx += ELANPRESS_NCC_STEP)
          {
            c = elanpress_ncc_at (hb, mb, ra, rm, w, h, dx, dy);
            if (c > abest)
              {
                abest = c;
                bdx = dx;
                bdy = dy;
              }
          }

      for (int dy = bdy - ELANPRESS_NCC_STEP / 2; dy <= bdy + ELANPRESS_NCC_STEP / 2; dy++)
        for (int dx = bdx - ELANPRESS_NCC_STEP / 2; dx <= bdx + ELANPRESS_NCC_STEP / 2; dx++)
          abest = MAX (abest, elanpress_ncc_at (hb, mb, ra, rm, w, h, dx, dy));

      best = MAX (best, abest);
    }

  return best;
}
