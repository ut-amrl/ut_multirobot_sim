/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/* image conversion */

#ifndef CONV_H
#define CONV_H

#include <climits>
#include "image.h"
#include "imutil.h"
#include "misc.h"

#define	RED_WEIGHT	0.299
#define GREEN_WEIGHT	0.587
#define BLUE_WEIGHT	0.114

static DistanceTransform::image<uchar> *imageRGBtoGRAY(
    DistanceTransform::image<rgb> *input) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<uchar> *output =
      new DistanceTransform::image<uchar>(width, height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = (uchar)
          (imRef(input, x, y).r * RED_WEIGHT +
          imRef(input, x, y).g * GREEN_WEIGHT +
          imRef(input, x, y).b * BLUE_WEIGHT);
    }
  }
  return output;
}

static DistanceTransform::image<rgb> *imageGRAYtoRGB(
    DistanceTransform::image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<rgb> *output =
      new DistanceTransform::image<rgb>(width, height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y).r = imRef(input, x, y);
      imRef(output, x, y).g = imRef(input, x, y);
      imRef(output, x, y).b = imRef(input, x, y);
    }
  }
  return output;
}

static DistanceTransform::image<float> *imageUCHARtoFLOAT(
    DistanceTransform::image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<float> *output =
      new DistanceTransform::image<float>(width, height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}

static DistanceTransform::image<float> *imageINTtoFLOAT(
    DistanceTransform::image<int> *input) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<float> *output =
      new DistanceTransform::image<float>(width, height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}

static DistanceTransform::image<uchar> *imageFLOATtoUCHAR(
    DistanceTransform::image<float> *input, float min, float max) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<uchar> *output =
      new DistanceTransform::image<uchar>(width, height, false);

  if (max == min)
    return output;

  float scale = UCHAR_MAX / (max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}

static DistanceTransform::image<uchar> *imageFLOATtoUCHAR(
    DistanceTransform::image<float> *input) {
  float min, max;
  min_max(input, &min, &max);
  return imageFLOATtoUCHAR(input, min, max);
}

static DistanceTransform::image<long> *imageUCHARtoLONG(
    DistanceTransform::image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<long> *output =
      new DistanceTransform::image<long>(width, height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}

static DistanceTransform::image<uchar> *imageLONGtoUCHAR(
    DistanceTransform::image<long> *input, long min, long max) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<uchar> *output =
      new DistanceTransform::image<uchar>(width, height, false);

  if (max == min)
    return output;

  float scale = UCHAR_MAX / (float)(max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}

static DistanceTransform::image<uchar> *imageLONGtoUCHAR(
    DistanceTransform::image<long> *input) {
  long min, max;
  min_max(input, &min, &max);
  return imageLONGtoUCHAR(input, min, max);
}

static DistanceTransform::image<uchar> *imageSHORTtoUCHAR(
    DistanceTransform::image<short> *input, short min, short max) {
  int width = input->width();
  int height = input->height();
  DistanceTransform::image<uchar> *output =
      new DistanceTransform::image<uchar>(width, height, false);

  if (max == min)
    return output;

  float scale = UCHAR_MAX / (float)(max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}

static DistanceTransform::image<uchar> *imageSHORTtoUCHAR(
    DistanceTransform::image<short> *input) {
  short min, max;
  min_max(input, &min, &max);
  return imageSHORTtoUCHAR(input, min, max);
}


#endif
