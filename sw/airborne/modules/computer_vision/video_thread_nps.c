/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * Dummy C implementation for simulation
 * The V4L2 could also work in simulation, but must be adapted a bit.
 */

// Own header
#include "video_thread.h"
#include "image_conversions.h"
#include "cv.h"
#include <stdio.h>
#include "readpng.h"

// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE,
  .fps = 30,
  .take_shot = FALSE,
  .shot_number = 0
};

int i = 0;

int use_yuv = 1;

// All dummy functions
void video_thread_init(void) {}
void video_thread_periodic(void)
{

  /* TODO: use setting variable here */
  //char image_folder[] = "/home/pold/Downloads/for_pres/";
  char image_folder[] = "/home/pold/Documents/MASTER_THESIS/final_evaluation/fast_flight/";
  //char image_folder[] = "/home/pold/from_bebop/png/";

    struct image_t img, yuv_img;
    image_create(&img, 640, 480, IMAGE_RGB);
    image_create(&yuv_img, 640, 480, IMAGE_YUV422);

    char image_path[2048];
    sprintf(image_path, "%simg_%05d.png", image_folder, i);
    printf("Image path: %s\n", image_path);
    read_png_file(image_path, &img);

    if (use_yuv) {
       RGBtoYUV422(&img, &yuv_img);
       cv_run(&yuv_img);
    } else {
       cv_run(&img);
    }

    image_free(&img);
    image_free(&yuv_img);
    i++;
}

void video_thread_start(void) {}
void video_thread_stop(void) {}
void video_thread_take_shot(bool_t take __attribute__((unused))) {}
