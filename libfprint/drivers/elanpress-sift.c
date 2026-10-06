/*
 * Keypoint features for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * The sensor images a 150 x 52 pixel window of the finger, so two presses
 * rarely show the same patch in the same place, and minutiae are far too few
 * to match on. Like the Windows driver, prints are matched on local ridge
 * features instead: keypoints are the extrema of a difference-of-gaussians
 * scale space, and each is described by histograms of gradient orientation
 * around it, relative to its own dominant direction (D. Lowe, "Distinctive
 * image features from scale-invariant keypoints", IJCV 2004). The parameters
 * are the ones the Windows driver uses for this sensor.
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

#include "elanpress-sift.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* scale space: scales sampled per octave, blur of each octave's first scale,
 * and the blur the input is assumed to already have */
#define SIFT_INTERVALS 3
#define SIFT_SIGMA 1.6f
#define SIFT_INIT_SIGMA 0.5f
#define SIFT_LAYERS (SIFT_INTERVALS + 3)

/* extrema below this contrast (image scaled to [0, 1]) are noise, and those
 * with a principal curvature ratio above SIFT_EDGE_RATIO lie on an edge,
 * where their position along it is ill defined */
#define SIFT_CONTRAST_THR 0.04f
#define SIFT_EDGE_RATIO 10.0f
#define SIFT_BORDER 5
#define SIFT_MAX_INTERP_STEPS 5

/* orientation: 36-bin gradient histogram over a gaussian window of 1.5x the
 * keypoint's scale; every peak within 80% of the highest gives a keypoint */
#define SIFT_ORI_BINS 36
#define SIFT_ORI_SIG_FCTR 1.5f
#define SIFT_ORI_RADIUS (3.0f * SIFT_ORI_SIG_FCTR)
#define SIFT_ORI_PEAK_RATIO 0.8f

/* descriptor: 4 x 4 cells of 8 orientation bins, each cell 3x the scale */
#define SIFT_DESC_WIDTH 4
#define SIFT_DESC_BINS 8
#define SIFT_DESC_SCL_FCTR 3.0f
#define SIFT_DESC_MAG_THR 0.2f
#define SIFT_DESC_INT_FCTR 512.0f

typedef struct
{
  int     w, h;
  gfloat *px;
} SiftImage;

/* gradient magnitude and direction of one gaussian scale */
typedef struct
{
  int     w, h;
  gfloat *mag;
  gfloat *ang;
} SiftGrad;

typedef struct
{
  int        n_octaves;
  SiftImage *gauss;     /* n_octaves x SIFT_LAYERS */
  SiftImage *dog;       /* n_octaves x (SIFT_LAYERS - 1) */
  SiftGrad  *grad;      /* n_octaves x SIFT_LAYERS, filled in on demand */
} SiftPyramid;

#define GAUSS(p, o, l) (&(p)->gauss[(o) * SIFT_LAYERS + (l)])
#define DOG(p, o, l) (&(p)->dog[(o) * (SIFT_LAYERS - 1) + (l)])

static void
sift_image_alloc (SiftImage *im, int w, int h)
{
  im->w = w;
  im->h = h;
  im->px = g_new (gfloat, (gsize) w * h);
}

static inline gfloat
px_at (const SiftImage *im, int x, int y)
{
  return im->px[y * im->w + x];
}

static inline int
reflect101 (int i, int n)
{
  if (n == 1)
    return 0;
  while (i < 0 || i >= n)
    i = i < 0 ? -i : 2 * n - 2 - i;
  return i;
}

/* separable gaussian blur, borders reflected about the edge pixel */
static void
sift_blur (const SiftImage *src, SiftImage *dst, gfloat sigma)
{
  int r = MAX (1, (int) (sigma * 4.0f + 0.5f));
  int w = src->w, h = src->h;
  g_autofree gfloat *k = g_new (gfloat, 2 * r + 1);
  g_autofree gfloat *row = g_new (gfloat, w + 2 * r);
  g_autofree gfloat *tmp = g_new (gfloat, (gsize) w * h);
  gfloat sum = 0;

  for (int i = -r; i <= r; i++)
    sum += k[i + r] = expf (-(gfloat) (i * i) / (2.0f * sigma * sigma));
  for (int i = 0; i <= 2 * r; i++)
    k[i] /= sum;

  for (int y = 0; y < h; y++)
    {
      const gfloat *in = src->px + (gsize) y * w;
      gfloat *out = tmp + (gsize) y * w;

      for (int x = -r; x < w + r; x++)
        row[x + r] = in[reflect101 (x, w)];
      for (int x = 0; x < w; x++)
        {
          gfloat acc = 0;
          for (int i = 0; i <= 2 * r; i++)
            acc += k[i] * row[x + i];
          out[x] = acc;
        }
    }

  for (int y = 0; y < h; y++)
    {
      gfloat *out = dst->px + (gsize) y * w;

      memset (out, 0, sizeof (gfloat) * w);
      for (int i = -r; i <= r; i++)
        {
          const gfloat *in = tmp + (gsize) reflect101 (y + i, h) * w;
          gfloat kv = k[i + r];
          for (int x = 0; x < w; x++)
            out[x] += kv * in[x];
        }
    }
}

/* the first octave works on the image doubled in size, which roughly
 * quadruples the keypoints found at the smallest scales (Lowe, 3.3) */
static void
sift_upsample (const guint8 *img, int w, int h, SiftImage *dst)
{
  sift_image_alloc (dst, 2 * w, 2 * h);
  for (int Y = 0; Y < 2 * h; Y++)
    {
      gfloat fy = (Y + 0.5f) * 0.5f - 0.5f;
      int y0 = (int) floorf (fy);
      gfloat wy = fy - y0;
      int ya = CLAMP (y0, 0, h - 1), yb = CLAMP (y0 + 1, 0, h - 1);

      for (int X = 0; X < 2 * w; X++)
        {
          gfloat fx = (X + 0.5f) * 0.5f - 0.5f;
          int x0 = (int) floorf (fx);
          gfloat wx = fx - x0;
          int xa = CLAMP (x0, 0, w - 1), xb = CLAMP (x0 + 1, 0, w - 1);
          gfloat top = (1 - wx) * img[ya * w + xa] + wx * img[ya * w + xb];
          gfloat bot = (1 - wx) * img[yb * w + xa] + wx * img[yb * w + xb];

          dst->px[Y * dst->w + X] = ((1 - wy) * top + wy * bot) / 255.0f;
        }
    }
}

static void
sift_pyramid_build (SiftPyramid *pyr, const guint8 *img, int w, int h)
{
  gfloat sig[SIFT_LAYERS];
  gfloat k = powf (2.0f, 1.0f / SIFT_INTERVALS);
  int ow = 2 * w, oh = 2 * h;
  SiftImage base;

  /* each octave needs room for the border plus the 3x3 extremum test */
  pyr->n_octaves = 0;
  while (MIN (ow, oh) >= 2 * SIFT_BORDER + 3)
    {
      pyr->n_octaves++;
      ow /= 2;
      oh /= 2;
    }
  pyr->gauss = g_new0 (SiftImage, pyr->n_octaves * SIFT_LAYERS);
  pyr->dog = g_new0 (SiftImage, pyr->n_octaves * (SIFT_LAYERS - 1));
  pyr->grad = g_new0 (SiftGrad, pyr->n_octaves * SIFT_LAYERS);

  /* incremental blur taking scale i-1 to scale i */
  sig[0] = SIFT_SIGMA;
  for (int i = 1; i < SIFT_LAYERS; i++)
    {
      gfloat prev = powf (k, i - 1) * SIFT_SIGMA;
      gfloat total = prev * k;
      sig[i] = sqrtf (total * total - prev * prev);
    }

  sift_upsample (img, w, h, &base);
  for (int o = 0; o < pyr->n_octaves; o++)
    {
      SiftImage *g0 = GAUSS (pyr, o, 0);

      if (o == 0)
        {
          gfloat init = 2.0f * SIFT_INIT_SIGMA;

          sift_image_alloc (g0, base.w, base.h);
          sift_blur (&base, g0, sqrtf (MAX (SIFT_SIGMA * SIFT_SIGMA - init * init, 0.01f)));
        }
      else
        {
          /* scale SIFT_INTERVALS of the previous octave has twice the blur
           * of its first one, so every other pixel of it starts this one */
          const SiftImage *src = GAUSS (pyr, o - 1, SIFT_INTERVALS);

          sift_image_alloc (g0, src->w / 2, src->h / 2);
          for (int y = 0; y < g0->h; y++)
            for (int x = 0; x < g0->w; x++)
              g0->px[y * g0->w + x] = px_at (src, 2 * x, 2 * y);
        }

      for (int i = 1; i < SIFT_LAYERS; i++)
        {
          sift_image_alloc (GAUSS (pyr, o, i), g0->w, g0->h);
          sift_blur (GAUSS (pyr, o, i - 1), GAUSS (pyr, o, i), sig[i]);
        }

      for (int i = 0; i < SIFT_LAYERS - 1; i++)
        {
          SiftImage *d = DOG (pyr, o, i);
          const SiftImage *a = GAUSS (pyr, o, i), *b = GAUSS (pyr, o, i + 1);

          sift_image_alloc (d, g0->w, g0->h);
          for (int p = 0; p < g0->w * g0->h; p++)
            d->px[p] = b->px[p] - a->px[p];
        }
    }
  g_free (base.px);
}

static void
sift_pyramid_clear (SiftPyramid *pyr)
{
  for (int i = 0; i < pyr->n_octaves * SIFT_LAYERS; i++)
    {
      g_free (pyr->gauss[i].px);
      g_free (pyr->grad[i].mag);
      g_free (pyr->grad[i].ang);
    }
  for (int i = 0; i < pyr->n_octaves * (SIFT_LAYERS - 1); i++)
    g_free (pyr->dog[i].px);
  g_free (pyr->gauss);
  g_free (pyr->dog);
  g_free (pyr->grad);
}

/* gradients of a gaussian scale, direction in [0, 2 pi) with y pointing
 * down; the outermost pixels have none */
static const SiftGrad *
sift_grad (SiftPyramid *pyr, int o, int l)
{
  SiftGrad *g = &pyr->grad[o * SIFT_LAYERS + l];
  const SiftImage *im = GAUSS (pyr, o, l);

  if (g->mag)
    return g;

  g->w = im->w;
  g->h = im->h;
  g->mag = g_new0 (gfloat, (gsize) im->w * im->h);
  g->ang = g_new0 (gfloat, (gsize) im->w * im->h);
  for (int y = 1; y < im->h - 1; y++)
    for (int x = 1; x < im->w - 1; x++)
      {
        gfloat dx = px_at (im, x + 1, y) - px_at (im, x - 1, y);
        gfloat dy = px_at (im, x, y + 1) - px_at (im, x, y - 1);
        gfloat a = atan2f (dy, dx);

        g->mag[y * im->w + x] = sqrtf (dx * dx + dy * dy);
        g->ang[y * im->w + x] = a < 0 ? a + 2 * G_PI : a;
      }
  return g;
}

static gboolean
sift_is_extremum (const SiftPyramid *pyr, int o, int l, int x, int y, gfloat v)
{
  for (int dl = -1; dl <= 1; dl++)
    {
      const SiftImage *im = DOG (pyr, o, l + dl);

      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
          {
            gfloat n = px_at (im, x + dx, y + dy);

            if (dl == 0 && dx == 0 && dy == 0)
              continue;
            if (v > 0 ? n > v : n < v)
              return FALSE;
          }
    }
  return TRUE;
}

/* solve the 3x3 symmetric system H X = b; FALSE if H is singular */
static gboolean
sift_solve3 (const gfloat H[3][3], const gfloat b[3], gfloat X[3])
{
  gfloat c00 = H[1][1] * H[2][2] - H[1][2] * H[2][1];
  gfloat c01 = H[1][2] * H[2][0] - H[1][0] * H[2][2];
  gfloat c02 = H[1][0] * H[2][1] - H[1][1] * H[2][0];
  gfloat det = H[0][0] * c00 + H[0][1] * c01 + H[0][2] * c02;
  gfloat inv[3][3];

  if (fabsf (det) < 1e-20f)
    return FALSE;

  inv[0][0] = c00;
  inv[0][1] = H[0][2] * H[2][1] - H[0][1] * H[2][2];
  inv[0][2] = H[0][1] * H[1][2] - H[0][2] * H[1][1];
  inv[1][0] = c01;
  inv[1][1] = H[0][0] * H[2][2] - H[0][2] * H[2][0];
  inv[1][2] = H[0][2] * H[1][0] - H[0][0] * H[1][2];
  inv[2][0] = c02;
  inv[2][1] = H[0][1] * H[2][0] - H[0][0] * H[2][1];
  inv[2][2] = H[0][0] * H[1][1] - H[0][1] * H[1][0];
  for (int i = 0; i < 3; i++)
    X[i] = (inv[i][0] * b[0] + inv[i][1] * b[1] + inv[i][2] * b[2]) / det;
  return TRUE;
}

/* fit a quadratic to the scale space around a sampled extremum to locate it
 * below the pixel and scale grid (Lowe, 4), then reject it if its contrast is
 * too low or it lies on an edge. Moves (*l, *x, *y) to the nearest sample
 * and returns the remaining offsets. */
static gboolean
sift_refine (const SiftPyramid *pyr, int o, int *l, int *x, int *y,
             gfloat *ox, gfloat *oy, gfloat *ol, gfloat *contrast)
{
  const SiftImage *cur = DOG (pyr, o, *l);
  gfloat H[3][3], dD[3], X[3] = { 0, 0, 0 };
  gfloat dxx = 0, dyy = 0, dxy = 0;
  int step;

  for (step = 0; step < SIFT_MAX_INTERP_STEPS; step++)
    {
      const SiftImage *prev = DOG (pyr, o, *l - 1), *next = DOG (pyr, o, *l + 1);
      int c = *x, r = *y;
      gfloat v2;

      cur = DOG (pyr, o, *l);
      v2 = 2 * px_at (cur, c, r);
      dD[0] = (px_at (cur, c + 1, r) - px_at (cur, c - 1, r)) * 0.5f;
      dD[1] = (px_at (cur, c, r + 1) - px_at (cur, c, r - 1)) * 0.5f;
      dD[2] = (px_at (next, c, r) - px_at (prev, c, r)) * 0.5f;

      dxx = px_at (cur, c + 1, r) + px_at (cur, c - 1, r) - v2;
      dyy = px_at (cur, c, r + 1) + px_at (cur, c, r - 1) - v2;
      dxy = (px_at (cur, c + 1, r + 1) - px_at (cur, c - 1, r + 1) -
             px_at (cur, c + 1, r - 1) + px_at (cur, c - 1, r - 1)) * 0.25f;
      H[0][0] = dxx;
      H[1][1] = dyy;
      H[2][2] = px_at (next, c, r) + px_at (prev, c, r) - v2;
      H[0][1] = H[1][0] = dxy;
      H[0][2] = H[2][0] = (px_at (next, c + 1, r) - px_at (next, c - 1, r) -
                           px_at (prev, c + 1, r) + px_at (prev, c - 1, r)) * 0.25f;
      H[1][2] = H[2][1] = (px_at (next, c, r + 1) - px_at (next, c, r - 1) -
                           px_at (prev, c, r + 1) + px_at (prev, c, r - 1)) * 0.25f;

      if (!sift_solve3 ((const gfloat (*)[3]) H, dD, X))
        return FALSE;
      X[0] = -X[0];
      X[1] = -X[1];
      X[2] = -X[2];

      if (fabsf (X[0]) < 0.5f && fabsf (X[1]) < 0.5f && fabsf (X[2]) < 0.5f)
        break;
      if (fabsf (X[0]) > 1e4f || fabsf (X[1]) > 1e4f || fabsf (X[2]) > 1e4f)
        return FALSE;

      *x += (int) roundf (X[0]);
      *y += (int) roundf (X[1]);
      *l += (int) roundf (X[2]);
      if (*l < 1 || *l > SIFT_INTERVALS ||
          *x < SIFT_BORDER || *x >= cur->w - SIFT_BORDER ||
          *y < SIFT_BORDER || *y >= cur->h - SIFT_BORDER)
        return FALSE;
    }
  if (step >= SIFT_MAX_INTERP_STEPS)
    return FALSE;

  *contrast = px_at (cur, *x, *y) + 0.5f * (dD[0] * X[0] + dD[1] * X[1] + dD[2] * X[2]);
  if (fabsf (*contrast) * SIFT_INTERVALS < SIFT_CONTRAST_THR)
    return FALSE;

  {
    gfloat tr = dxx + dyy, det = dxx * dyy - dxy * dxy;

    if (det <= 0 ||
        tr * tr * SIFT_EDGE_RATIO >= (SIFT_EDGE_RATIO + 1) * (SIFT_EDGE_RATIO + 1) * det)
      return FALSE;
  }

  *ox = X[0];
  *oy = X[1];
  *ol = X[2];
  *contrast = fabsf (*contrast);
  return TRUE;
}

/* smoothed histogram of gradient directions around (cx, cy); returns the
 * highest bin */
static gfloat
sift_ori_hist (const SiftGrad *g, int cx, int cy, int radius, gfloat sigma,
               gfloat hist[SIFT_ORI_BINS])
{
  const int n = SIFT_ORI_BINS;
  gfloat raw[SIFT_ORI_BINS + 4] = { 0 };
  gfloat scale = -1.0f / (2.0f * sigma * sigma);
  gfloat maxv = 0;

  for (int i = -radius; i <= radius; i++)
    {
      int y = cy + i;

      if (y <= 0 || y >= g->h - 1)
        continue;
      for (int j = -radius; j <= radius; j++)
        {
          int x = cx + j;
          int bin;

          if (x <= 0 || x >= g->w - 1)
            continue;
          bin = (int) roundf (n * g->ang[y * g->w + x] / (2 * G_PI));
          bin = ((bin % n) + n) % n;
          raw[bin + 2] += expf ((i * i + j * j) * scale) * g->mag[y * g->w + x];
        }
    }

  raw[1] = raw[n + 1];
  raw[0] = raw[n];
  raw[n + 2] = raw[2];
  raw[n + 3] = raw[3];
  for (int i = 0; i < n; i++)
    {
      hist[i] = (raw[i] + raw[i + 4]) * (1.0f / 16) +
                (raw[i + 1] + raw[i + 3]) * (4.0f / 16) +
                raw[i + 2] * (6.0f / 16);
      maxv = MAX (maxv, hist[i]);
    }
  return maxv;
}

/* gradient directions relative to the keypoint's, binned into a 4 x 4 grid
 * that turns with it and spreads by trilinear interpolation (Lowe, 6) */
static void
sift_descriptor (const SiftGrad *g, gfloat px, gfloat py, gfloat angle,
                 gfloat sigma, guint8 out[ELANPRESS_SIFT_DESC_LEN])
{
  const int d = SIFT_DESC_WIDTH, n = SIFT_DESC_BINS;
  gfloat hist[(SIFT_DESC_WIDTH + 2) * (SIFT_DESC_WIDTH + 2) * (SIFT_DESC_BINS + 2)] = { 0 };
  gfloat v[ELANPRESS_SIFT_DESC_LEN];
  int cx = (int) roundf (px), cy = (int) roundf (py);
  gfloat hist_width = SIFT_DESC_SCL_FCTR * sigma;
  gfloat cos_t = cosf (angle) / hist_width, sin_t = sinf (angle) / hist_width;
  gfloat bins_per_rad = n / (2 * G_PI);
  gfloat wscale = -1.0f / (d * d * 0.5f);
  int radius = (int) roundf (hist_width * G_SQRT2 * (d + 1) * 0.5f);
  gfloat nrm2 = 0, thr;

  radius = MIN (radius, (int) sqrtf ((gfloat) (g->w * g->w + g->h * g->h)));

  for (int i = -radius; i <= radius; i++)
    for (int j = -radius; j <= radius; j++)
      {
        /* the sample's position in the keypoint's own frame, in cells */
        gfloat c_rot = j * cos_t + i * sin_t;
        gfloat r_rot = -j * sin_t + i * cos_t;
        gfloat rbin = r_rot + d / 2 - 0.5f;
        gfloat cbin = c_rot + d / 2 - 0.5f;
        int x = cx + j, y = cy + i;
        gfloat mag, obin, v_r1, v_r0, v_rc11, v_rc10, v_rc01, v_rc00;
        int r0, c0, o0, idx;

        if (rbin <= -1 || rbin >= d || cbin <= -1 || cbin >= d ||
            y <= 0 || y >= g->h - 1 || x <= 0 || x >= g->w - 1)
          continue;

        mag = g->mag[y * g->w + x] * expf ((c_rot * c_rot + r_rot * r_rot) * wscale);
        obin = (g->ang[y * g->w + x] - angle) * bins_per_rad;
        while (obin < 0)
          obin += n;
        while (obin >= n)
          obin -= n;

        r0 = (int) floorf (rbin);
        c0 = (int) floorf (cbin);
        o0 = (int) floorf (obin);
        rbin -= r0;
        cbin -= c0;
        obin -= o0;

        v_r1 = mag * rbin;
        v_r0 = mag - v_r1;
        v_rc11 = v_r1 * cbin;
        v_rc10 = v_r1 - v_rc11;
        v_rc01 = v_r0 * cbin;
        v_rc00 = v_r0 - v_rc01;

        idx = ((r0 + 1) * (d + 2) + c0 + 1) * (n + 2) + o0;
        hist[idx] += v_rc00 * (1 - obin);
        hist[idx + 1] += v_rc00 * obin;
        hist[idx + (n + 2)] += v_rc01 * (1 - obin);
        hist[idx + (n + 3)] += v_rc01 * obin;
        hist[idx + (d + 2) * (n + 2)] += v_rc10 * (1 - obin);
        hist[idx + (d + 2) * (n + 2) + 1] += v_rc10 * obin;
        hist[idx + (d + 3) * (n + 2)] += v_rc11 * (1 - obin);
        hist[idx + (d + 3) * (n + 2) + 1] += v_rc11 * obin;
      }

  for (int i = 0; i < d; i++)
    for (int j = 0; j < d; j++)
      {
        int idx = ((i + 1) * (d + 2) + (j + 1)) * (n + 2);

        /* fold the wrap-around orientation bins back */
        hist[idx] += hist[idx + n];
        hist[idx + 1] += hist[idx + n + 1];
        for (int k = 0; k < n; k++)
          v[(i * d + j) * n + k] = hist[idx + k];
      }

  /* unit length, then cap single gradients that dominate (lighting,
   * saturation) and renormalize */
  for (int k = 0; k < ELANPRESS_SIFT_DESC_LEN; k++)
    nrm2 += v[k] * v[k];
  thr = sqrtf (nrm2) * SIFT_DESC_MAG_THR;
  nrm2 = 0;
  for (int k = 0; k < ELANPRESS_SIFT_DESC_LEN; k++)
    {
      v[k] = MIN (v[k], thr);
      nrm2 += v[k] * v[k];
    }
  nrm2 = SIFT_DESC_INT_FCTR / MAX (sqrtf (nrm2), FLT_EPSILON);
  for (int k = 0; k < ELANPRESS_SIFT_DESC_LEN; k++)
    out[k] = (guint8) CLAMP ((int) roundf (v[k] * nrm2), 0, 255);
}

static gint
sift_cmp_response (gconstpointer a, gconstpointer b)
{
  const ElanpressKeypoint *ka = a, *kb = b;

  return (ka->response < kb->response) - (ka->response > kb->response);
}

ElanpressFeatures *
elanpress_sift_features (const guint8 *img, int width, int height, guint max_keypoints)
{
  SiftPyramid pyr;
  GArray *kps = g_array_new (FALSE, FALSE, sizeof (ElanpressKeypoint));
  ElanpressFeatures *f = g_new0 (ElanpressFeatures, 1);
  gfloat prelim = 0.5f * SIFT_CONTRAST_THR / SIFT_INTERVALS;

  sift_pyramid_build (&pyr, img, width, height);

  for (int o = 0; o < pyr.n_octaves; o++)
    for (int layer = 1; layer <= SIFT_INTERVALS; layer++)
      {
        const SiftImage *dog = DOG (&pyr, o, layer);

        for (int y = SIFT_BORDER; y < dog->h - SIFT_BORDER; y++)
          for (int x = SIFT_BORDER; x < dog->w - SIFT_BORDER; x++)
            {
              gfloat v = px_at (dog, x, y);
              gfloat hist[SIFT_ORI_BINS], ox, oy, ol, contrast, sigma, omax;
              gfloat octave_scale = ldexpf (1.0f, o - 1);
              int l = layer, c = x, r = y;
              const SiftGrad *g;

              if (fabsf (v) <= prelim || !sift_is_extremum (&pyr, o, layer, x, y, v))
                continue;
              if (!sift_refine (&pyr, o, &l, &c, &r, &ox, &oy, &ol, &contrast))
                continue;

              sigma = SIFT_SIGMA * powf (2.0f, (l + ol) / SIFT_INTERVALS);
              g = sift_grad (&pyr, o, l);
              omax = sift_ori_hist (g, c, r, (int) roundf (SIFT_ORI_RADIUS * sigma),
                                    SIFT_ORI_SIG_FCTR * sigma, hist);

              for (int j = 0; j < SIFT_ORI_BINS; j++)
                {
                  int lb = j > 0 ? j - 1 : SIFT_ORI_BINS - 1;
                  int rb = j < SIFT_ORI_BINS - 1 ? j + 1 : 0;
                  ElanpressKeypoint kp;
                  gfloat bin;

                  if (!(hist[j] > hist[lb] && hist[j] > hist[rb] &&
                        hist[j] >= omax * SIFT_ORI_PEAK_RATIO))
                    continue;

                  /* parabolic peak interpolation */
                  bin = j + 0.5f * (hist[lb] - hist[rb]) / (hist[lb] - 2 * hist[j] + hist[rb]);
                  if (bin < 0)
                    bin += SIFT_ORI_BINS;
                  else if (bin >= SIFT_ORI_BINS)
                    bin -= SIFT_ORI_BINS;

                  kp.x = (c + ox) * octave_scale;
                  kp.y = (r + oy) * octave_scale;
                  kp.scale = sigma * octave_scale;
                  kp.angle = bin * (2 * G_PI / SIFT_ORI_BINS);
                  kp.response = contrast;
                  sift_descriptor (g, c + ox, r + oy, kp.angle, sigma, kp.desc);
                  g_array_append_val (kps, kp);
                }
            }
      }

  sift_pyramid_clear (&pyr);

  if (max_keypoints && kps->len > max_keypoints)
    {
      g_array_sort (kps, sift_cmp_response);
      g_array_set_size (kps, max_keypoints);
    }

  f->n = kps->len;
  f->kp = (ElanpressKeypoint *) g_array_free (kps, FALSE);
  return f;
}

void
elanpress_features_free (ElanpressFeatures *features)
{
  if (!features)
    return;
  g_free (features->kp);
  g_free (features);
}
