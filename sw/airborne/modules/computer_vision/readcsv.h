#ifndef READ_CSV_H_
#define READ_CSV_H_

#include <stdio.h>
#include "lib/vision/image.h"
#include "../particle_filter/particle_filter.h"
#include "texton_settings.h"

/* #include "trexton.h" */

void cb_write_to_int_arr(void *s, size_t i, void *arr);
void cb_write_to_float_arr(void *s, size_t i, void *arr);
void cb_write_to_double_arr(void *s, size_t i, void *arr);
void cb_write_to_position_arr(void *s, size_t i, void *arr);

void cb_end_of_line(int c, void *outfile);

uint8_t read_csv_into_array(void *array, char *filename, void (*cb)(void *, size_t, void *));
uint8_t read_textons_from_csv(double textons[][TREXTON_TOTAL_PATCH_SIZE], char *filename);
uint8_t read_histograms_from_csv(float histograms[][TREXTON_SIZE_HIST], char *filename, int used_width);
uint8_t read_color_histograms_from_csv(double *histograms, char *filename, int used_width);
uint8_t read_positions_from_csv(struct measurement *measurements, char *filename);
uint8_t read_weights_from_csv(double *weights, char *filename);
uint8_t read_test_histograms_from_csv(int *histograms, char *filename);
uint8_t read_test_positions_from_csv(struct measurement *measurements, char *filename);

#endif // READ_CSV_H_
