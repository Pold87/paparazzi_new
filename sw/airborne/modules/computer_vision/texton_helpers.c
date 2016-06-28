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

#include "texton_helpers.h" /* Utilities for extracting textons and saving histograms */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

/* Calculate Euclidean distance between two double arrays */
double euclidean_dist(double x[], double y[], int s)
{
  double sum = 0;
  double dist;
  int i;
  for(i = 0; i < s; i++)
    {
       /* printf("%f ", x[i]); */

      sum += pow((x[i] - y[i]), 2.0);
    }
      dist = sqrt(sum);
      /* printf("dist is %f\n", dist); */
  return dist;
}

/* Calculate Euclidean distance between two double arrays */
float euclidean_dist_float(float x[], float y[], int s)
{
  float sum = 0;
  float dist;
  int i;
  for(i = 0; i < s; i++)
    {
       /* printf("%f ", x[i]); */

      sum += pow((x[i] - y[i]), 2.0);
    }
      dist = sqrt(sum);
      /* printf("dist is %f\n", dist); */
  return dist;
}


double euclidean_dist_int(int x[], int y[], int s)
{
  uint64_t sum = 0;
  double dist;
  int i;


  for(i = 0; i < s; i++)
    {
      /* printf("Regression histograms: first %d second %d\n", x[i], y[i]); */
      sum += pow((x[i] - y[i]), 2.0);
    }
  dist = sqrt(sum);

  /* printf("Euclidean dist %f \n", dist); */
  return dist;
}


double chi_square_dist(int x[], int y[], int s) {

  int i;
  double dist = 0, curr_dist;
  for(i = 0; i < s; i++) {
    curr_dist = pow(x[i] - y[i], 2) / (x[i] + y[i] + 1.0e-8);
    dist += curr_dist;
  }
  
  dist = dist / 2;
  return dist;

}

double chi_square_dist_double(double x[], double y[], int s) {

  int i;
  double dist = 0, curr_dist;
  for(i = 0; i < s; i++) {
    curr_dist = pow(x[i] - y[i], 2) / (x[i] + y[i] + 1.0e-8);
    dist += curr_dist;
  }
  
  dist = dist / 2;
  return dist;

}



double bhat_dist(int x[], int y[], int s) {

  double sum = 0;

  int i;
  for (i = 0; i < s; i++) {
    sum += sqrt(((double) x[i]) * ((double) y[i]));
  }


  double dist = - log(sum);

  return dist;
  
}

double bhat_dist_double(double x[], double y[], int s) {

  double sum = 0;

  int i;
  for (i = 0; i < s; i++) {
    sum += sqrt(((double) x[i]) * ((double) y[i]));
  }

  double dist = - log(sum);

  return dist;
  
}

/* Find the arg maximum of an array */
int arg_max(int arr[], int size){

  /* am: argmax, m: max */
  int am = 0, m = 0, i;
  for (i = 0; i < size; i++) {
    if (arr[i] > m) {
      m = arr[i];
      am = i;
    }
  }
  return am;
}

int max(int arr[], int size){

  /* am: argmax, m: max */
  int m = 0, i;
  for (i = 0; i < size; i++) {
    if (arr[i] > m) {
      m = arr[i];
    }
  }

  return m;

}


void save_image(struct image_t *img, char* filename) {


  /* Get image buffer */
  double *buf = img->buf;

  FILE *fp = fopen(filename, "w");

  int interlace;
  if (img->type == IMAGE_GRAYSCALE)
     interlace = 1;
  else if (img->type == IMAGE_YUV422)
     interlace = 2;
  else if (img->type == IMAGE_STD)
     interlace = 1;
  else {
     printf("[save_image] No image type specified");
     while(1);
  }
  
  int x, y;
  uint32_t pos;
    
  for (y = 0; y < img->h; y++) {
    for (x = 0; x < img->w; x++) {
      pos =  (x + (img->w * y)) * interlace;
      fprintf(fp, "%f", buf[pos]);
      if (x != (img->w - 1))
	fprintf(fp, ",");
    }
    fprintf(fp, "\n");
  }



  /* for (i = 0; i < img->w * img->h * interlace; i += interlace) { */

  /*   fprintf(fp, "%f", buf[i]); */
  /*   printf("%d ", i); */
  /*   if (i % ((img->w * interlace) + 1) != 0 || i == 0) */
  /*     fprintf(fp, ","); */
  /*   if (i % ((img->w * interlace) + 1) == 0 && i != 0) */
  /*     fprintf(fp, "\n"); */
  /* } */

  fclose(fp);
}


/* Determine the interlace of grayscale pixels in an image */
uint8_t get_interlace(struct image_t *img){
  
  uint8_t interlace;

  switch(img->type) {

  case IMAGE_GRAYSCALE :
    interlace = 1;
    break;

  case IMAGE_YUV422 :
    interlace = 2;
    break;

  case IMAGE_OPPONENT :
    interlace = 3;
    break;
    
  case IMAGE_STD :
    interlace = 1;
    break;

  case IMAGE_RGB :
    interlace = 3;
    break;
    
  default :
    printf("No valid color space defined");
    interlace = -1;
  }

  return interlace;
}

/* Extract an image patch of size 'patch_size' (for example, 5 x 5)
   and fill 'patch' with the flattened patch */
void extract_one_patch(struct image_t *img, double *patch, int x, int y, uint8_t patch_size, uint8_t channel)
{

  /* position of x, y */
  uint8_t interlace = get_interlace(img);
  
  /* ABSOLUTE TODO !! CHECK TYPE OF BUFFER */
  /* Get image buffer */
  uint8_t *buf = (uint8_t*) img->buf;

  /* double *buf = img->buf; */

  /* TODO: check the + 1 here */
  uint32_t pos =  (x + (img->w * y)) * interlace + channel + 1;

  /* printf("pos is %d\n", pos); */

  double mean = 0.0;
  double M2 = 0;
  int i;
  double delta = 0;
  double min_val = 1000;
  double max_val = -1000;

  /* Extract patches  */
  for (i = 0; i < TOTAL_PATCH_SIZE; i++) {

    /* TODO: SOMETHINGS TERRIBLY WRONG WITH THE RANDOM !!! POSITIONS */
    
    /* Assign pixel values */
    double double_pix = (double)buf[pos];
    patch[i] = double_pix;

    
    #if TEXTON_STANDARIZE
    if (double_pix > max_val)
      max_val = double_pix;

    if (double_pix < min_val)
      min_val = double_pix;
    
    delta = double_pix - mean;
    mean += delta / (i + 1);
    M2 += delta * (double_pix - mean);
    #endif

    
    /* if (channel == 0) */
    /*   printf("Patch is %f\n", double_pix); */
    
    /* Check, if texton extraction should continue in a new line */
    if (i % patch_size == 0 && i != 0)
      pos += ((img->h * interlace) - (patch_size * interlace));
    else
      pos += interlace; 
  }

  
  /* double var = M2 / (TOTAL_PATCH_SIZE - 1); */
  /* double std = sqrt(var); */
  /* printf("M2 is %f, std is %f min %f  max %f", M2, std, min_val, max_val); */
  /*  Normalize*/

  /* for (i = 0; i < total_patch_size; i++) { */
  /*   printf("%f ", patch[i]); */
  /* } */
  /* printf("\n"); */

  
  /* for (i = 0; i < total_patch_size; i++) { */
  /*   if (std < 0.0001) { */
  /*     patch[i] = 0.0; */
  /*   } else { */
  /*     patch[i] = (patch[i] - mean) / std; */
  /*   } */
  /* } */

  #if TEXTON_STANDARIZE
  for (i = 0; i < TOTAL_PATCH_SIZE; i++) {
    if ((max_val - min_val) < 80) {
      patch[i] = 0.0;
    } else {
      patch[i] = (patch[i] - min_val) / (max_val - min_val);
    }
  }
  #endif
  

  /* /\* int i; *\/ */
  /* for (i = 0; i < total_patch_size; i++) { */
  /*   printf("patch is %f\n", patch[i]); */
  /* } */


}

/* Get the texton histogram of an image */
void get_texton_histogram(struct image_t *img, float texton_histogram[], double textons[][TOTAL_PATCH_SIZE]) {

  printf("Type of in get_texton_hist  is %d\n", img->type);
    uint8_t texton_ids[MAX_TEXTONS * CHANNELS]; /*  texton IDs */
    double patch[TOTAL_PATCH_SIZE];
    printf("Total patch is is %d\n", TOTAL_PATCH_SIZE);
    uint8_t texton_id;

    /* Set random seed */
    //srand (time(NULL));
    srand(42);

    
    /* Extract image patches */

      FILE *fp_xy;
      fp_xy = fopen("xy_pos.csv", "w");

    int i;
    for (i = 0; i < MAX_TEXTONS; i++) {

       /* Extract random locations of patchsize x patchsize */
      int max_x = img->w - PATCH_SIZE;
      int max_y = img->h - PATCH_SIZE;

      int rand_x = rand();
      int rand_y = rand();

      /* int between 0 and max - 1 */
      int x = rand_x % max_x;
      int y = rand_y % max_y;

      // Print coords of x y positions to file
      fprintf(fp_xy, "%d,%d\n", x, y);

      int channel;
      for (channel = 0; channel < CHANNELS; channel++) {
         extract_one_patch(img, patch, x, y, PATCH_SIZE, channel);

	/* int u; */
	/* printf("patch is"); */
	/* for (u = 0; u < TOTAL_PATCH_SIZE; u++) { */
	/*   printf("%f ", patch[u]); */
	/* } */
	/* printf("\n"); */
	
	/* Find nearest texton ...*/
	texton_id = label_image_patch(patch, textons, channel);

	/* ... and fill the feature vector */
	texton_ids[i + channel * MAX_TEXTONS] = texton_id;
	
      }

    }

    fclose(fp_xy);

    /* Build the histogram of textons from the texton ID array */
    make_histogram(texton_ids, texton_histogram);

    /* /\* Set black to zero *\/ */
    /* texton_histogram[1] = 0.0; */
    
    #if TREXTON_DEBUG
      printf("Texton histogram\n");
      for (i = 0; i < NUM_TEXTONS * CHANNELS; i++)
	printf("%f ", texton_histogram[i]);
      printf("\n");
    #endif
 }

/* Vector quantization */
int get_bin(int val, int bincount){

  int bin_size = 256 / bincount;
  int bin = val / bin_size;

  return bin;

}


/* Get color histogram of an YUV422 image; there's an alternative function that gets the color histogram of the extracted textons  */
void get_color_histogram(struct image_t *img, double color_hist[], int num_bins){

  uint8_t interlace = get_interlace(img);
  uint8_t *buf = (uint8_t *)img->buf;

  /* The histogram is composes of Y hists, U hists, and then V hists */
  
  int i;  
  for (i = 0; i < img->w * img->h * interlace; i = i + 2 * interlace) {   

    int U = buf[i];
    int u_bin = get_bin(U, num_bins);
    color_hist[num_bins + u_bin] += (1.0 / 153600.0);
    
    int Y1 = buf[i+1];
    int y1_bin = get_bin(Y1, num_bins);
    color_hist[y1_bin] += (1.0 / 153600.0);
    
    int V = buf[i+2];
    int v_bin = get_bin(V, num_bins);
    color_hist[2 * num_bins + v_bin] += (1.0 / 153600.0);    
    
    int Y2 = buf[i+3];
    int y2_bin = get_bin(Y2, num_bins);
    color_hist[y2_bin] += (1.0 / 153600.0);
    
  }
}

/* Create a histogram showing the frequency of values in an array */
void make_histogram(uint8_t *texton_ids, float texton_hist[]) {

  int i = 0;
  /* Set all to 0 (TODO: there might be a better option!!) */
  for (i = 0; i < NUM_TEXTONS * CHANNELS; i++) {
    texton_hist[i] =  0.0;
  }
  
  for (i = 0; i < MAX_TEXTONS * CHANNELS; i++) {
    texton_hist[texton_ids[i]] += (1.0 / (MAX_TEXTONS * CHANNELS));
  }
}


void concat_histograms(double color_hist[], double texton_histogram[], double both_hists[], int width_color, int width_textons){

    int j;
    for (j = 0; j < width_color + width_textons; j++) {

      if (j < width_color)
	both_hists[j] = color_hist[j];
      else
	both_hists[j] = texton_histogram[j - width_color];
    }

}

/* Write histogram to a new line to a given file pointer */
void save_histogram_int(int hist[], FILE *fp, int width) {

   if (!fp) {
    fprintf(stderr, "[treXton - save_histogram] Can't open file.\n");
   }

    int j;
    for (j = 0; j < width; j++) {
      fprintf(fp, "%d", hist[j]);
      if (j != width - 1)
	fprintf(fp, ",");
      printf("%d ", hist[j]);
    }
    fprintf(fp, "\n");
    printf("\n");
   
}

/* Write histogram to a new line to a given file pointer */
void save_histogram_double(double hist[], FILE *fp, int width) {

   if (!fp) {
    fprintf(stderr, "[treXton - save_histogram] Can't open file.\n");
   }

    int j;
    for (j = 0; j < width; j++) {
      fprintf(fp, "%f", hist[j]);
      if (j != width - 1)
	fprintf(fp, ",");
      printf("%f ", hist[j]);
    }
    fprintf(fp, "\n");
    printf("\n");
}



/* Write histogram to a new line to a given file pointer */
void save_histogram_float(float hist[], FILE *fp, int width) {

   if (!fp) {
    fprintf(stderr, "[treXton - save_histogram] Can't open file.\n");
   }

    int j;
    for (j = 0; j < width; j++) {
      fprintf(fp, "%f", hist[j]);
      if (j != width - 1)
	fprintf(fp, ",");
      printf("%f ", hist[j]);
    }
    fprintf(fp, "\n");
    printf("\n");
}


void save_histogram_both(double hist_color[], double hist_textons[], FILE *fp_all, int width_color, int width_textons) {
   if (!fp_all) {
    fprintf(stderr, "[treXton - save_histogram_both] Can't open file.\n");
   }

    int j;
    for (j = 0; j < width_color + width_textons; j++) {

      if (j < width_color)
	fprintf(fp_all, "%f", hist_color[j]);
      else
	fprintf(fp_all, "%f", hist_textons[j - width_color]);
      if (j != width_color + width_textons - 1)
	fprintf(fp_all, ",");
    }
    fprintf(fp_all, "\n");

}


/* Compare an image patch to all existing textons using Euclidean
   distance */
uint8_t label_image_patch(double patch[], double textons[][TOTAL_PATCH_SIZE], uint8_t channel){

  /* Total patch size is width of patch times height of patch */
  uint8_t patch_size = PATCH_SIZE;

  int num_textons = NUM_TEXTONS;
  uint8_t total_patch_size = pow(patch_size, 2);

  double dist; /* Current distance between patch and texton */
  int id = 0; /* ID of closest texton */
  double min_dist = MAX_POSSIBLE_DIST; /* Minimum distance between patch and texton */

  /* Label the patch */
  int i;
  for (i = 0; i < num_textons; i++) {
    dist = euclidean_dist(patch, textons[i + num_textons * channel], total_patch_size);
    /* printf("i is %d dist is %f\n", i, dist); */
    if (dist < min_dist) {
      min_dist = dist;
      id = i +  num_textons * channel; /* Set id */
    }
  }
  return id;
}


void make_double_hist(int int_hist[], double double_hist[], int size) {

  int i;
  for (i = 0; i < size; i++) {
    double_hist[i] = ((double) int_hist[i]) ;

  }
    
}

/* void get_idfs(double idfs[], double regression_histograms_color){ */

/*   int r, c; /\* Row, column *\/ */
/*   for (r = 0; r < NUM_HISTOGRAMS; r++) { */
/*     for (c = 0; c < SIZE_HIST; c++) { */
      
/*     } */

/*   } */

  
/* } */


void rotate_flow(double flow_x, double flow_y, double *flow_rot_x, double *flow_rot_y, double degrees) {

  double radians = degrees * 3.1415926535 / 180;

  *flow_rot_x = flow_x * cos(radians) - flow_y * sin(radians);
  *flow_rot_y = flow_y * cos(radians) + flow_x * sin(radians);
  
}

double rotate_flow_x(double flow_x, double flow_y, double degrees) {

  double radians = degrees * 3.1415926535 / 180;

  double flow_rot_x = flow_x * cos(radians) - flow_y * sin(radians);
  return flow_rot_x;
}

double rotate_flow_y(double flow_x, double flow_y, double degrees) {

  double radians = degrees * 3.1415926535 / 180;

  double flow_rot_y = flow_y * cos(radians) + flow_x * sin(radians);
  return flow_rot_y;
}


int measurement_comp (const void *elem1, const void *elem2)
{

  struct measurement *m1 = (struct measurement *) elem1;
  struct measurement *m2 = (struct measurement *) elem2;

  double f = m1->dist;
  double s =  m2->dist;
  if (f > s) return 1;
  if (f < s) return -1;
  return 0;
}
