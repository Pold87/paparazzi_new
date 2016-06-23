/*
 * Copyright (C) Volker Strobel
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of t/home/pold/paparazzi/sw/airborne/modules/computer_vision
he GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef TREXTON_SETTINGS_H
#define TREXTON_SETTINGS_H

#include <stdio.h>
#include "lib/vision/image.h"


/* /\* The video device *\/ */
/* #ifndef TREXTON_DEVICE */
/* #define TREXTON_DEVICE /dev/video2      ///< The video device */
/* #endif */

/* The video device size (width, height) */
#ifndef TREXTON_DEVICE_SIZE
#define TREXTON_DEVICE_SIZE 640,480     ///< The video device size (width, height)
/* /\* #define TREXTON_DEVICE_SIZE 640,480     ///< The video device size (width, height) *\/ */
/* /\* #define TREXTON_DEVICE_SIZE 1280, 720     ///< The video device size (width, height) *\/ */
#endif

/* The video device buffers (the amount of V4L2 buffers) */
/* #ifndef TREXTON_DEVICE_BUFFERS */
/* #define TREXTON_DEVICE_BUFFERS 15       ///< The video device buffers (the amount of V4L2 buffers) */
/*#endif*/

#define TREXTON_DEBUG false

/* #define VIEWVIDEO_HOST 192.168.1.255 */
/* #define VIEWVIDEO_PORT_OUT 5000 */
/* #define VIEWVIDEO_BROADCAST true */

/* Analysis */
#define MEASURE_TIME true

/* treXton settings */
#define PATCH_SIZE  5
#define TOTAL_PATCH_SIZE 25
#define NUM_TEXTONS 48
//#define NUM_TEXTONS 33
#define MAX_TEXTONS 1000
#define MAX_POSSIBLE_DIST 50000

/* Maximum lines read from histogram CSV */
#define NUM_HISTOGRAMS 54
#define NUM_CLASSES 4
#define PREDICT true
#define EVALUATE true
#define NUM_TEST_HISTOGRAMS 625
#define SAVE_HISTOGRAM false
#define USE_WEBCAM true
#define HISTOGRAM_PATH "../treXton/saved.csv"
#define USE_CONVERSIONS false
#define TEXTON_STANDARIZE true
#define COLOR_CHANNELS 3
#define NUM_COLOR_BINS 48
#define SEND_VIDEO false
#define USE_COLOR false
#define USE_ALL false
#define CHANNELS 1

#if USE_COLOR
  #define SIZE_HIST NUM_COLOR_BINS*COLOR_CHANNELS
#elif USE_ALL
  #define SIZE_HIST NUM_TEXTONS*CHANNELS+NUM_COLOR_BINS*COLOR_CHANNELS
#else
  #define SIZE_HIST NUM_TEXTONS*CHANNELS
#endif

/* Total patch size is width of patch times height of patch */
/* static uint8_t patch_size = PATCH_SIZE; */
/* static uint8_t total_patch_size = 25; //pow(patch_size, 2); */
/* static int knn = 5; */



#endif
