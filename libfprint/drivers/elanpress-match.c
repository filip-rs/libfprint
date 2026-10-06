/*
 * Image processing and keypoint matching for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * The imaged area of these sensors is far too small for reliable minutiae
 * matching, so prints store the enrolled images themselves and a press is
 * matched against them by pairing up local keypoint features (see
 * elanpress-sift.c), like the Windows driver does.
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

ElanpressFeatures *
elanpress_image_features (const guint8 *img, int w, int h)
{
  return elanpress_sift_features (img, w, h, ELANPRESS_MAX_KEYPOINTS);
}

static gint
elanpress_desc_dist2 (const guint8 *a, const guint8 *b)
{
  gint sum = 0;

  for (int k = 0; k < ELANPRESS_SIFT_DESC_LEN; k++)
    {
      gint d = (gint) a[k] - (gint) b[k];
      sum += d * d;
    }
  return sum;
}

typedef struct
{
  gfloat px, py;        /* keypoint on the probe */
  gfloat ex, ey;        /* its partner on the enrolled image */
  gfloat rot;           /* rotation the pair's orientations imply */
  guint  ei;            /* partner index, to count distinct partners */
} ElanpressPair;

typedef struct
{
  gfloat c, s, tx, ty;
} ElanpressPlacement;

/* the placement a single pair implies: its rotation, and the translation
 * that then lands the probe keypoint exactly on its partner */
static void
elanpress_pair_placement (const ElanpressPair *p, ElanpressPlacement *m)
{
  m->c = cosf (p->rot);
  m->s = sinf (p->rot);
  m->tx = p->ex - (m->c * p->px - m->s * p->py);
  m->ty = p->ey - (m->s * p->px + m->c * p->py);
}

static gboolean
elanpress_pair_fits (const ElanpressPair *p, const ElanpressPlacement *m, gfloat tol2)
{
  gfloat dx = m->c * p->px - m->s * p->py + m->tx - p->ex;
  gfloat dy = m->s * p->px + m->c * p->py + m->ty - p->ey;

  return dx * dx + dy * dy < tol2;
}

/* least-squares rotation and translation taking the fitting pairs' probe
 * keypoints onto their partners */
static gboolean
elanpress_refit (const ElanpressPair *pairs, const gboolean *fit, guint n,
                 ElanpressPlacement *m)
{
  gdouble mpx = 0, mpy = 0, mex = 0, mey = 0, sxx = 0, sxy = 0, a;
  guint cnt = 0;

  for (guint i = 0; i < n; i++)
    if (fit[i])
      {
        mpx += pairs[i].px;
        mpy += pairs[i].py;
        mex += pairs[i].ex;
        mey += pairs[i].ey;
        cnt++;
      }
  if (cnt < 2)
    return FALSE;
  mpx /= cnt;
  mpy /= cnt;
  mex /= cnt;
  mey /= cnt;

  for (guint i = 0; i < n; i++)
    if (fit[i])
      {
        gdouble px = pairs[i].px - mpx, py = pairs[i].py - mpy;
        gdouble ex = pairs[i].ex - mex, ey = pairs[i].ey - mey;

        sxx += px * ex + py * ey;
        sxy += px * ey - py * ex;
      }

  a = atan2 (sxy, sxx);
  m->c = cos (a);
  m->s = sin (a);
  m->tx = mex - (m->c * mpx - m->s * mpy);
  m->ty = mey - (m->s * mpx + m->c * mpy);
  return TRUE;
}

/* fitting pairs, counted once per distinct keypoint on either side: a
 * keypoint with two dominant directions yields two keypoints at one spot,
 * and two probe keypoints can share a partner */
static guint
elanpress_count_distinct (const ElanpressPair *pairs, const gboolean *fit, guint n)
{
  guint probe = 0, enrolled = 0;

  for (guint i = 0; i < n; i++)
    {
      gboolean seen_p = FALSE, seen_e = FALSE;

      if (!fit[i])
        continue;
      for (guint j = 0; j < i; j++)
        {
          if (!fit[j])
            continue;
          if (pairs[j].px == pairs[i].px && pairs[j].py == pairs[i].py)
            seen_p = TRUE;
          if (pairs[j].ex == pairs[i].ex && pairs[j].ey == pairs[i].ey)
            seen_e = TRUE;
        }
      probe += !seen_p;
      enrolled += !seen_e;
    }
  return MIN (probe, enrolled);
}

/* the number of keypoint pairs that agree on one rigid placement of the
 * probe on the enrolled image.
 *
 * Each pair's keypoint orientations already fix the rotation, so every pair
 * proposes a complete placement on its own. Trying all of them, rather than
 * sampling pairs of pairs as RANSAC would, makes the result deterministic
 * and does not miss a genuine placement when most pairs are chance ones. The
 * best supported proposals are then refined by least squares over the pairs
 * they gather. */
guint
elanpress_match_features (const ElanpressFeatures *probe,
                          const ElanpressFeatures *enrolled)
{
  const gfloat loose2 = ELANPRESS_MATCH_LOOSE_PX * ELANPRESS_MATCH_LOOSE_PX;
  const gfloat tol2 = ELANPRESS_MATCH_TOL_PX * ELANPRESS_MATCH_TOL_PX;
  const gdouble ratio2 = ELANPRESS_MATCH_RATIO * ELANPRESS_MATCH_RATIO;
  g_autofree ElanpressPair *pairs = NULL;
  g_autofree guint *support = NULL;
  g_autofree gboolean *fit = NULL, *next = NULL;
  guint n = 0, best = 0;

  if (!probe || !enrolled || probe->n < 2 || enrolled->n < 2)
    return 0;

  pairs = g_new (ElanpressPair, probe->n);
  for (guint i = 0; i < probe->n; i++)
    {
      const ElanpressKeypoint *kp = &probe->kp[i];
      gint d1 = G_MAXINT, d2 = G_MAXINT;
      guint b1 = 0;

      for (guint j = 0; j < enrolled->n; j++)
        {
          gint d = elanpress_desc_dist2 (kp->desc, enrolled->kp[j].desc);

          if (d < d1)
            {
              d2 = d1;
              d1 = d;
              b1 = j;
            }
          else if (d < d2)
            {
              d2 = d;
            }
        }
      if (d2 == G_MAXINT || d1 >= ratio2 * d2)
        continue;

      pairs[n].px = kp->x;
      pairs[n].py = kp->y;
      pairs[n].ex = enrolled->kp[b1].x;
      pairs[n].ey = enrolled->kp[b1].y;
      pairs[n].rot = enrolled->kp[b1].angle - kp->angle;
      pairs[n].ei = b1;
      n++;
    }
  if (n < 3)
    return 0;

  support = g_new0 (guint, n);
  fit = g_new (gboolean, n);
  next = g_new (gboolean, n);

  /* support for the placement each pair proposes */
  for (guint k = 0; k < n; k++)
    {
      ElanpressPlacement m;

      elanpress_pair_placement (&pairs[k], &m);
      for (guint i = 0; i < n; i++)
        support[k] += elanpress_pair_fits (&pairs[i], &m, loose2);
    }

  /* refine the best supported proposals; a genuine placement is proposed by
   * every one of its pairs, so it is among the first few */
  for (int round = 0; round < ELANPRESS_MATCH_REFINED; round++)
    {
      ElanpressPlacement m;
      guint top = 0, cnt;

      for (guint k = 1; k < n; k++)
        if (support[k] > support[top])
          top = k;
      if (support[top] < 3)
        break;
      support[top] = 0;

      elanpress_pair_placement (&pairs[top], &m);
      for (guint i = 0; i < n; i++)
        fit[i] = elanpress_pair_fits (&pairs[i], &m, loose2);

      for (int step = 0; step < 3; step++)
        {
          gboolean same = TRUE;
          guint was = 0, now = 0;

          if (!elanpress_refit (pairs, fit, n, &m))
            break;
          for (guint i = 0; i < n; i++)
            {
              next[i] = elanpress_pair_fits (&pairs[i], &m, tol2);
              same &= next[i] == fit[i];
              was += fit[i];
              now += next[i];
            }
          memcpy (fit, next, n * sizeof (gboolean));
          if (same || now < was)
            break;
        }

      cnt = elanpress_count_distinct (pairs, fit, n);
      best = MAX (best, cnt);
    }

  return best;
}
