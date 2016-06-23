#include "particle_filter.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[]) {

  /* Settings */
  int use_smoothing = 0;
  int use_flow = 1;
  int use_multiple_predictions = 0;
  int use_variance = 0;

  if (argc < 2) {
    printf("Please specify: test, train, or preds and data set size");
    return -1;
  }

  int SIZE_SIFT = atoi(argv[2]);

  /* Create arrays for measurements */
  struct measurement measurements[SIZE_SIFT];
  struct measurement measurements_2[SIZE_SIFT]; /* If multiply regressors are used */
  struct measurement measurements_3[SIZE_SIFT]; /* If multiply regressors are used */

  struct measurement opticalflow[SIZE_SIFT];

  char *filename_out;

  /* For test set */
  if (strcmp(argv[1], "test") == 0) {   
    filename_out = "sift_filtered_test_2.csv";
    //char filename_in[] = "/home/pold/Documents/Internship/datasets/board_test_pos.csv";
    char filename_in[] = "/home/pold/Documents/Internship/datasets/board_test_2_pos.csv";
    read_measurements_from_csv(measurements, filename_in, SIZE_SIFT);
  }

  /* For training set */
  else if (strcmp(argv[1], "train") == 0) {
    filename_out = "sift_filtered_train_vel.csv";
    char filename_in[] = "/home/pold/Documents/Internship/datasets/board_train_pos.csv";
    read_measurements_from_csv(measurements, filename_in, SIZE_SIFT);
  }

  /* For predictions */
  else if (strcmp(argv[1], "preds") == 0) {
    filename_out = "predictions_filtered_lasso.csv";
    char filename_in[] = "/home/pold/Documents/treXton/predictions_cross.csv";
    char filename_in_2[] = "/home/pold/Documents/treXton/predictions.csv";

    char filename_optical_flow[] = "/home/pold/Documents/trexton_pprz/edgeflow_diff.csv";
    //char filename_in_3[] = "/home/pold/Documents/Internship/treXton/predictions_3.csv";
    read_predictions_from_csv(measurements, filename_in, SIZE_SIFT, 1);
    read_predictions_from_csv(measurements_2, filename_in_2, SIZE_SIFT, 0);

    read_predictions_from_csv(opticalflow, filename_optical_flow, SIZE_SIFT, 0);
    /* read_predictions_from_csv(measurements_3, filename_in_3, SIZE_SIFT); */


  } else {
    printf("No argument specified");
    return -1;
  }
  
  /* Create and initialize particles */
  struct particle particles[N];
  init_particles(particles);

  struct particle particles_backward[N];
  init_particles(particles_backward);

  /* Run particle filter for every measurement */
  int i = 0, j = 0, k;
  FILE *fp = fopen(filename_out, "w");
  fprintf(fp, "x,y\n");

  /* Tables for dynamic programming */
  struct particle ps_forward[SIZE_SIFT];
  struct particle ps_backward[SIZE_SIFT];
  struct particle uncertainties[SIZE_SIFT];

  /* Fill in dynamic programming table */
  for (i = 0; i < SIZE_SIFT; i++) {
    printf(" iteration: %d\n", i);
    fflush(stdout);

    /* Use predictions form multiple regressors */ 
    if (use_multiple_predictions) {
      particle_filter_multiple(particles, &measurements[i], &measurements_2[i], use_variance);
    } else {
      particle_filter(particles, &measurements[i], &opticalflow[i], use_variance, use_flow);
    }


    /* Forward-backward smoothing */
    if (use_smoothing) {
      k = SIZE_SIFT - i;
      particle_filter(particles_backward, &measurements[k], &opticalflow[i], use_variance, use_flow);
      struct particle p_backward = weighted_average(particles_backward, N);
      ps_backward[k] = p_backward;
    }


    struct particle p_forward = weighted_average(particles, N);
    uncertainties[i] = calc_uncertainty(particles, p_forward, N);

    printf("Uncertainty: x: %f y: %f", uncertainties[i].x, uncertainties[i].y);
    
    ps_forward[i] = p_forward;

  }

  for (i = 0; i < SIZE_SIFT; i++) {

    struct particle p;

    printf("measurement %f %f\n", measurements[i].x, measurements[i].y);

    if (use_smoothing)
      p = weight_forward_backward(ps_forward[i], ps_backward[i], i, (SIZE_SIFT - i));
    
    else
      p = ps_forward[i];

    printf("x: %f y: %f", p.x, p.y);
    fprintf(fp, "%f,%f\n" , p.x, p.y);
  }

  fclose(fp);
  return 0;
}

