#include "readcsv.h"
/* #include "trexton.h" */
#include "csv.h"
#include <errno.h>
#include <math.h>
#include "modules/particle_filter/particle_filter.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lib/vision/image.h"

#include "texton_helpers.h"

#define CSV_APPEND_NULL 8 /* Ensure that all fields are null-ternimated */

static int width = 0;
static int row = 0;
static int col = 0;
static int max_lines;


void cb_write_to_int_arr(void *s, size_t i, void *arr) {
   i = i;
  /* Save in texton array */
  int* int_arr = (int*) arr;
  unsigned char *str = s;

  /* Convert string to int */
  size_t j;
  int acc = 0;
  for (j = 0; j < i; j++) {
    acc = acc * 10;
    acc += str[j] - '0';
 }

  if (max_lines > 0)
    int_arr[row * width + col] = acc;
  col++;
}

void cb_write_to_double_arr(void *s, size_t i, void *arr) {
   i = i;
  /* Save in texton array */
  double* double_arr = (double*) arr;
  if (max_lines > 0)
    double_arr[row * width + col] = strtof(s, NULL);
  col++;
}

void cb_write_to_float_arr(void *s, size_t i, void *arr) {
   i = i;
  /* Save in texton array */
  float* float_arr = (float*) arr;
  if (max_lines > 0)
    float_arr[row * width + col] = strtof(s, NULL);
  /* TODO use sprintf!!! */
  col++;
}


void cb_write_to_position_arr(void *s, size_t i, void *arr) {

   /* Save in texton array */
  unsigned char *str = s;

  /* Convert string to int */
  size_t j;
  int acc = 0;
  for (j = 0; j < i; j++) {
     acc = acc * 10;
     if (str[j] == '-')
        continue;
     if (str[0] == '-') {
        acc -= str[j] - '0';
     } else {
        acc += str[j] - '0';
     }
 }


  /* Save in texton array */
  struct measurement* pos_arr = (struct measurement*) arr;
  if (max_lines > 0) {

    /* the first column has the x coordinate */
    if (col == 0)
      pos_arr[row].x = (float) acc;
    /* and the second one the y coordinate */
    if (col == 1)
      pos_arr[row].y = (float) acc;
    if (col == 2)
       pos_arr[row].dist = (float) acc;
  }
  /* and the third one the number of matches  */
  col++;
}

void cb_write_to_optitrack_position_arr(void *s, size_t i, void *arr) {

  /* Save in texton array */
  struct measurement* pos_arr = (struct measurement*) arr;
  if (max_lines > 0) {
    
    /* the first column has the x coordinate */
    if (col == 0)
      pos_arr[row].x = (float) atoi(s);
    /* and the second one the y coordinate */
    if (col == 1)
      pos_arr[row].y = (float) atoi(s);
  }
  col++;
}



void cb_end_of_line(int c, void *outfile) {

   c = c;
  /* Set row and column counter */
  max_lines--;
  col = 0;
  row++;
}

uint8_t read_csv_into_array(void *array, char *filename, void (*cb)(void *, size_t, void *)) {
  
  /* Reset counter variables */
  col = 0;
  row = 0;

  char buf[1024];
  struct csv_parser p;
  
  size_t i;
  csv_init(&p, 0);

  /* Read input CSV file */
  FILE *infile;
  printf("[read_csv_into_array] filename: %s\n", filename);
  infile = fopen(filename, "rb");

  /* Check if CSV file exists */
  if (infile == NULL) {
    fprintf(stderr, "Failed to open file %s: %s\n", filename, strerror(errno));
    exit(EXIT_FAILURE);
  }

  /* Read CSV into buffer */
  while ((i = fread(buf, 1, 1024, infile)) > 0 && max_lines > 0) {
    if (csv_parse(&p, buf, i, *cb, cb_end_of_line, array) != i) {
      fprintf(stderr, "Error: %s\n", csv_strerror(csv_error(&p)));
      fclose(infile);
      exit(EXIT_FAILURE);
    }
  }

  csv_fini(&p, cb, cb_end_of_line, array);
  csv_free(&p);
  fclose(infile);

  return 0;
}


uint8_t read_textons_from_csv(double textons[][TREXTON_TOTAL_PATCH_SIZE], char *filename) {
  
  printf("\n%s\n", filename);
  fflush(stdout); // Prints to screen or whatever your standard out is
  max_lines = TREXTON_NUM_TEXTONS * TREXTON_CHANNELS;
  width = TREXTON_TOTAL_PATCH_SIZE;
  uint8_t r = read_csv_into_array(textons, filename, cb_write_to_double_arr);
  
  return r;


}

uint8_t read_histograms_from_csv(float histograms[][TREXTON_SIZE_HIST], char *filename, int used_width) {
   
  printf("\n%s\n", filename);
  fflush(stdout); // Prints to screen or whatever your standard out is
  max_lines = TREXTON_NUM_HISTOGRAMS;
  width = used_width;
  uint8_t r = read_csv_into_array(histograms, filename, cb_write_to_float_arr);

  return r;

 }



uint8_t read_color_histograms_from_csv(double *histograms, char *filename, int used_width) {
   
  printf("\n%s\n", filename);
  fflush(stdout); // Prints to screen or whatever your standard out is
  max_lines = TREXTON_NUM_HISTOGRAMS;
  width = used_width;
  uint8_t r = read_csv_into_array(histograms, filename, cb_write_to_double_arr);

  return r;

 }


uint8_t read_test_histograms_from_csv(int *histograms, char *filename) {
   
  printf("\n%s\n", filename);
  fflush(stdout); // Prints to screen or whatever your standard out is
  max_lines = TREXTON_NUM_TEST_HISTOGRAMS;
  width = TREXTON_NUM_TEXTONS * TREXTON_CHANNELS;
  uint8_t r = read_csv_into_array(histograms, filename, cb_write_to_int_arr);

  return r;

}


 uint8_t read_positions_from_csv(struct measurement *measurements, char *filename) {
   
  printf("[read_positions_from_csv] filename is \n%s\n", filename);
  fflush(stdout); 

  max_lines = TREXTON_NUM_HISTOGRAMS;
  width = 3; /* CSV header is x, y, matches */
  uint8_t r = read_csv_into_array(measurements, filename, cb_write_to_position_arr);

  /* int i; */
  /* for (i = 0; i < 100; i++) { */
  /*   printf("pos in func is %f", positions[i].x); */
  /* } */


  return r;

 }

 uint8_t read_optitrack_positions_from_csv(struct measurement *measurements, char *filename) {
   
  printf("[read_optitrack_positions_from_csv] filename is \n%s\n", filename);
  fflush(stdout); 

  max_lines = TREXTON_NUM_HISTOGRAMS;
  width = 2; /* CSV header is x, y */
  uint8_t r = read_csv_into_array(measurements, filename, cb_write_to_optitrack_position_arr);

  return r;

 }


 uint8_t read_test_positions_from_csv(struct measurement *measurements, char *filename) {
   
  printf("[read_positions_from_csv] filename is \n%s\n", filename);
  fflush(stdout); 

  max_lines = TREXTON_NUM_TEST_HISTOGRAMS;
  width = 4; /* CSV header is id, x, y, matches */
  uint8_t r = read_csv_into_array(measurements, filename, cb_write_to_position_arr);

  /* int i; */
  /* for (i = 0; i < 100; i++) { */
  /*   printf("pos in func is %f", positions[i].x); */
  /* } */


  return r;

 }



uint8_t read_weights_from_csv(double *weights, char *filename) {
   
  printf("\n%s\n", filename);
  fflush(stdout); // Prints to screen or whatever your standard out is
  max_lines = TREXTON_NUM_TEXTONS * TREXTON_CHANNELS;
  width = 1;
  uint8_t r = read_csv_into_array(weights, filename, cb_write_to_double_arr);

  return r;

}
