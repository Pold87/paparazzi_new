#include "particle_filter.h"
#include <stdio.h>
#include <time.h>
#define SIZE_SIFT 588

int main(int argc, char *argv[])
{

  printf("Red: Measurement (output from treXton)\nGreen: Ground truth\nBlue: Particles (size represents their weight)\nYellow: Best estimate from particle filter (weighted average)");
  
  /* Create and initilize particles */
  struct particle particles[N];
  init_particles(particles);

  struct particle particles_backward[N];
  init_particles(particles_backward);


  struct measurement measurements[M];
  create_fake_measurements(measurements);
  /* read_measurements_from_csv(measurements, "/home/pold/Documents/Internship/treXton_pprz/predictions.csv", M); */
  /*zs_to_csv(measurements, "fake_measurements.csv");*/

  struct sift sift[M];
  read_sift_from_csv(sift, "/home/pold/Documents/Internship/image_recorder/sift_targets.csv");

  /* Run particle filter for every measurement */
  int i = 0, j = 0;
  FILE *gnuplot;
  gnuplot = popen("gnuplot", "w");
  fprintf(gnuplot, "set palette model RGB defined (0 'blue', 1 'green', 2 'red', 3 'yellow')\n");
  fprintf(gnuplot, "unset colorbox\n");
  fprintf(gnuplot, "set xrange[0:1024]\n");
  fprintf(gnuplot, "set yrange[0:1024]\n");

  for (i = 0; i < M; i++) {
    printf("iteration: %d\n", i);
    particle_filter(particles, &measurements[i]);

    int k;
    for (k = M; k > i; k--) {
      particle_filter(particles_backward, &measurements[k]);
    }
    struct particle p_backward = weighted_average(particles_backward, N);
    struct particle p_forward = weighted_average(particles, N);
    struct particle p;

    p = weight_forward_backward(p_forward, p_backward, i, (M - i));

    fprintf(gnuplot, "plot '-' with points pt 7 ps variable palette\n");
    for (j = 0; j < N; j++) {
      fprintf(gnuplot, "%f %f %f 0\n", particles[j].x, particles[j].y, particles[j].w * 10);
      /* printf("%f %f\n", particles[j].x, particles[j].y); */
    }

    printf("measurement (z) - x: %f , y: %f\n", measurements[i].x, measurements[i].y);
    
    fprintf(gnuplot, "%f %f 4 2\n", measurements[i].x, measurements[i].y);
    /* Bimodal*/
    /* fprintf(gnuplot, "%d %d 4 1\n",  ((int) 200 * i) % ((int) 400), ((int) 200 * i) % ((int) 400)); */
    /* delta_x = delta_y = 5 */
    fprintf(gnuplot, "%d %d 4 1\n", 5 * i, 5 * i);
    fprintf(gnuplot, "%f %f 4 3\n", p.x, p.y); /* The best prediction (weighted average) */
    printf("%f %f 4 3\n", p_backward.x, p_backward.y); /* The best prediction (weighted average) */
    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    fprintf(gnuplot, "refresh;\n");

    struct timespec ts;
    int milliseconds = 100;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
    /* pclose(gnuplot); */
    /* Print all particles */
    /* for (j = 0; j < N; j++) { */
    /*   printf("x is: %f y is: %f\n", particles[j].x, particles[j].y); */
    /* } */
  }
  return 0;
}
