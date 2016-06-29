#ifndef TEXTON_HELPERS_H
#define TEXTON_HELPERS_H

/*
 * Copyright (C) Volker Strobel
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
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

/**
 * @file "modules/trexton/trexton_helpers.h"
 * @author Volker Strobel
 * treXton regression localization
 */


#include <stdio.h>
#include "lib/vision/image.h"
#include "texton_settings.h"
#include "../particle_filter/particle_filter.h"


/* A marker has a distance and an ID that can be mapped to its name */
struct marker {

  int id;
  double dist;

};

/* A position has a x and y coordinate */
struct position {

  double x;
  double y;
  double dist; /* Distance to the current histogram */

};

float euclidean_dist_float(float x[], float y[], int s);
double euclidean_dist(double x[], double y[], int s);
double euclidean_dist_int(int x[], int y[], int s);
double chi_square_dist(int x[], int y[], int s);
double chi_square_dist_double(double x[], double y[], int s);
double bhat_dist(int x[], int y[], int s);
double bhat_dist_double(double x[], double y[], int s);

/**
*
 * Find the arg maximum of an integer array
 * @param arr An integer array
 * @param size Size of the integer array
 *
 * @return Index of the maximum of the array
 */
int arg_max(int arr[], int size);

/**
 * \brief Find the maximum of an integer array
 *
 * @param arr An integer array
 * @param size Size of the integer array
 *
 * @return Maximum of the array
 */
int max(int arr[], int size);
void extract_one_patch(struct image_t *img, double *patch, int x, int y, uint8_t patch_size, uint8_t channel);
void get_texton_histogram(struct image_t *img, float texton_histogram[], double textons[][TREXTON_TOTAL_PATCH_SIZE]);
void make_histogram(uint8_t *texton_ids, float texton_hist[]);
void save_histogram_int(int hist[], FILE *fp, int width);
void save_histogram_double(double hist[], FILE *fp, int width);
void save_histogram_float(float hist[], FILE *fp, int width);
void save_histogram_both(double hist_color[], float hist_textons[], FILE *fp_all, int width_color, int width_textons);
void concat_histograms(double color_hist[], double texton_histogram[], double both_hists[], int width_color, int width_textons);
uint8_t label_image_patch(double *patch, double textons[][TREXTON_TOTAL_PATCH_SIZE], uint8_t channel);
uint8_t predict_class(int *texton_hist);
int measurement_comp (const void *elem1, const void *elem2);
void save_image(struct image_t *img, char* filename);
void get_color_histogram(struct image_t *img, double color_hist[], int num_bins);
uint8_t get_interlace(struct image_t *img);
int get_bin(int val, int bincount);
void get_idfs(double idfs[], double regression_histograms_color);
void  make_double_hist(int texton_histogram[], double color_hist[], int size);
void rotate_flow(double flow_x, double flow_y, double *flow_rot_x, double *flow_rot_y, double degrees);
double rotate_flow_x(double flow_x, double flow_y, double degrees);
double rotate_flow_y(double flow_x, double flow_y, double degrees);

#endif
