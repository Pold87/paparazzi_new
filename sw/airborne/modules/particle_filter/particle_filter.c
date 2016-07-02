/*
Particle filter without control input
 */

#include <stdio.h>
#include <stdlib.h>
#include "particle_filter.h"
#include "random_number_gen.h"
#include <math.h>
#include <time.h>

#define PI 3.14159265358979323846

#define max_x 640
#define max_y 480

int informed_prior = 0;
int global_k = 0;
FILE *gnuplot;

static double process_noise_x;
static double process_noise_y;

void init_particles(struct particle particles[N]){

  srand(time(NULL)); /* Set random seed */
  int i;
  for (i = 0; i < N; i++) {

    /* Initialize with informed prior */
    if (informed_prior) {
      particles[i].x = randu(1060, 1080);
      printf("%f", particles[i].x);
      particles[i].y = randu(275, 285);
      particles[i].w = 1;
      particles[i].prev_w = 1;
    }

    /* Initialize with random x, y-positions */
    else {

      particles[i].prev_x = randu(0, max_x);
      particles[i].prev_y = randu(0, max_y);

      particles[i].vel_x = randu(0, 5);
      particles[i].vel_y = randu(0, 5);

      particles[i].x = randu(0, max_x);
      particles[i].y = randu(0, max_y);

      particles[i].w = 1;
      particles[i].prev_w = 1;
      }
  }
}


double normpdf(double x, double mu, double sigma) {

  double density, a;
  static const double inv_sqrt_2pi = 0.3989422804014327;

  a = (x - mu) / sigma;
  density = inv_sqrt_2pi / sigma * exp(-0.5 * a * a);

  return density;

}

void weighted_sample(struct particle ps[], struct particle res[], double weights[], int samples){

  double sum = 0, x;
  int i;
  /* Calculate sum */
  for (i = 0; i < N; i++) {
    sum += weights[i];
  }
  printf("sum is %f", sum);

  i = 0;
  int m;
  double w = weights[0];
  struct particle v;
  for (m = 0; m < samples; m++) {
    x = sum * (1 - pow(randu(0, 1), (1.0 / samples)));
    //printf("%f is\n", x);
    sum -= x;
    while (x > w) {
            x -= w;
            i += 1;
            w = weights[i];
      v = ps[i];
    }
    w -= x;
    res[m] = v;
  }
}


double fmax(double a, double b) {

  if (a > b)
    return a;
  else
    return b;
}

double fmin(double a, double b) {

  if (a < b)
    return a;
  else
    return b;
}


double array_max(double arr[], int size){

  double m = -1;
  int i;
  for (i = 0; i < size; i++) {
    if (arr[i] > m)
      m = arr[i];
  }
  return m;
}


/* Returns total weight */
double resampling_wheel(struct particle ps[], struct particle res[], double weights[], int samples) {

    /* p: particles */
    /* w: weights */
    /* N: Desired number of particles */

  int idx = randu(0, 1) * samples;
  /* printf("index is %d\n", idx); */
  double beta = 0.0;
  double mw = array_max(weights, samples);
  /* printf("MAX IS %f\n", mw); */

  double total = 0;
  
  int i;
  for (i = 0; i < N; i++) {
    /* printf("[resampling wheel] weights: %f\n", weights[i]); */
    beta += randu(0, 1) * 2.0 * mw;
    while (beta > weights[idx]){
      /* printf(" beta is %f\n", beta); */
      /* printf(" weights[ifx] is %f\n", weights[idx]); */
      beta -= weights[idx];
      idx = (idx + 1) % N;
    }
    /* printf("idx outside is %d\n", idx); */
    /* printf("ps[ids] outside is %f\n", ps[idx].x); */
    res[i] = ps[idx];
    total += res[i].w;
  }

  return total;
}


void particle_filter(struct particle xs[N], struct measurement z[], struct measurement *flow,
		     int use_variance, int use_flow, int num_predictions) {

  double w[N]; /* The weights of particles */

  if (use_flow) {
    process_noise_x = 10;
    process_noise_y = 10;
  } else {
    process_noise_x = 25.0;
    process_noise_y = 25.0;
  }

  double measurement_noise_x;
  double measurement_noise_y;

  if (use_variance) {

    measurement_noise_x = z[0].dist * 1000;
    measurement_noise_y = z[0].dist * 1000;
    printf("Measurement noise x is %f", measurement_noise_x);

  } else {

    measurement_noise_x = 100;
    measurement_noise_y = 100;
  }


  /* IMPORTANT REMOVE AGAIN */
  int debug_flow = 0;
  
  
  /* Obtaining new belief state (iterate over all particles) */
  int i = 0;

  for (i = 0; i < N; i++) {

    /* Process noise incorporates the movement of the UAV */
    /* According to p(x_t | x_(t-1)) */

    double updated_x;
    double updated_y;

    xs[i].prev_x = xs[i].x;
    xs[i].prev_y = xs[i].y;
    
    if (use_flow) {

       updated_x = xs[i].x + flow->x;
       updated_y = xs[i].y + flow->y;

    } else {
       updated_x = xs[i].x;
       updated_y = xs[i].y;
    }

    if (debug_flow) {

      process_noise_x = 0.1;
      process_noise_y = 0.1;
      measurement_noise_x = 200;
      measurement_noise_y = 200;

    } 

    /* Add some random process noise */
    xs[i].x = randn(updated_x, process_noise_x);
    xs[i].y = randn(updated_y, process_noise_y);

    /* Calculate weight */
    double p_x, p_y;

      int pred;
      double total_likelihood = 0.00000001;

      double phis[num_predictions];
      phis[0] = 0.4;
      phis[1] = 0.3;
      phis[2] = 0.2;
      phis[3] = 0.08;
      phis[4] = 0.02;
      double phi;
      for (pred = 0; pred < num_predictions; pred++) {

         phi = phis[pred];

         /* TODO: instead of fixed measurement noise use confidence */
         //p_x = normpdf(z[pred].x, xs[i].x, measurement_noise_x);
         //p_y = normpdf(z[pred].y, xs[i].y, measurement_noise_y);
	
         p_x = normpdf(xs[i].x, z[pred].x, z[pred].dist * 1400.0);
         p_y = normpdf(xs[i].y, z[pred].y, z[pred].dist * 1400.0);

         total_likelihood += phi * p_x * p_y;

      }

      xs[i].prev_w = xs[i].w;

      /* TODO: see if weight array is necessary */
      
      w[i] = total_likelihood;
      xs[i].w = w[i]; /* Set weight of particle */
	
  }

  

  /* Importance resampling: (iterate over all particles) */
  struct particle res[N];
  double total = resampling_wheel(xs, res, w, N);

  /* Copy results */
  for (i = 0; i < N; i++) {
//     printf("xs: %f, %f;  res: %f, %f", xs[i].x, xs[i].y, res[i].x, res[i].y);
     xs[i] = res[i];
  }

  /* Normalize weights (should be 1!) */
  for (i = 0; i < N; i++) {
    xs[i].w = xs[i].w / total;
  }  
}


struct particle weighted_average(struct particle ps[], int size) {

  int i;
  double total_weight = 0;
  double x = 0;
  double y = 0;

  for (i = 0; i < size; i++) {
    total_weight += ps[i].w;
    x += ps[i].x * ps[i].w;
    y += ps[i].y * ps[i].w;
  }

  struct particle p;
  p.x = x / total_weight;
  p.y = y / total_weight;

  return p;
}

struct particle map_estimate(struct particle ps[], int size) {

  int i, j;

  double t_max;
  int t_argmax;
  t_max = -1.0;
  t_argmax = 0;

  /* See paper MAP Estimation in Particle Filter Tracking */
  for (i = 0; i < size; i++) {

    double s, a, b, t;
    s = 0;
    for (j = 0; j < size; j++) {
       a = normpdf(ps[i].x, ps[j].prev_x, process_noise_x);
       b = normpdf(ps[i].y, ps[j].prev_y, process_noise_y);
       s += a * b * ps[j].prev_w;
    }

    t = ps[i].w * s;
    if (t > t_max) {
       t_max = t;
       t_argmax = i;
    }
  }

  return ps[t_argmax];
}

struct particle weight_forward_backward(struct particle p_forward, struct particle p_backward, int i, int k) {

  double forward_x = p_forward.x;
  double backward_x = p_backward.x;
  double forward_y = p_forward.y;
  double backward_y = p_backward.y;

  double combined_x = (i * forward_x + k * backward_x) / (i + k);
  double combined_y = (i * forward_y + k * backward_y) / (i + k);

  struct particle p;
  p.x = combined_x;
  p.y = combined_y;

  return p;

}

struct particle calc_uncertainty(struct particle ps[], struct particle weighted_mean, int size) {

  int i;
  float total_weight = 0;
  float x = 0;
  float y = 0;

    for (i = 0; i < size; i++) {
    total_weight += ps[i].w;
    x += ps[i].w * pow(ps[i].x - weighted_mean.x, 2);
    y += ps[i].w * pow(ps[i].y - weighted_mean.y, 2);
  }

  struct particle p;
  p.x = x / total_weight;
  p.y = y / total_weight;

  printf("\nUncertainty in x (STD) is: %f\n", sqrt(p.x));
  printf("Uncertainty in y (STD) is: %f\n", sqrt(p.y));

  return p;

}

void init_visualize(void){
  gnuplot = popen("gnuplot", "w");
  fprintf(gnuplot, "set palette model RGB defined (0 'blue', 1 'green', 2 'red', 3 'yellow')\n");
  fprintf(gnuplot, "unset colorbox\n");
  /* fprintf(gnuplot, "set xrange[0:1280]\n"); */
  /* fprintf(gnuplot, "set yrange[0:720]\n"); */
  fprintf(gnuplot, "set xrange[-500:500]\n");
  fprintf(gnuplot, "set yrange[-500:500]\n");
}

void visualize_simple(double x, double y) {
    /* fprintf(gnuplot, "plot '/home/pold/Documents/Internship/draug/img/sparse_board.jpg' binary filetype=jpg with rgbimage, '-' with points pt 7 ps variable palette\n"); */
    fprintf(gnuplot, "plot '-' with points pt 7 ps variable palette\n");
    fprintf(gnuplot, "%f %f 4 2\n", x, y);
    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    fprintf(gnuplot, "refresh;\n");
}


void visualize_optitrack(int x, int y, int opti_x, int opti_y, double uncertainty) {
    /* fprintf(gnuplot, "plot '/home/pold/Documents/Internship/draug/img/sparse_board.jpg' binary filetype=jpg with rgbimage, '-' with points pt 7 ps variable palette\n"); */
    fprintf(gnuplot, "set cbrange [0:1]\n");
    fprintf(gnuplot, "set palette defined (0.0 0 0 0.5, 0.1 0 0 1, 0.2 0 0.5 1, 0.3 0 1 1, 0.4 0.5 1 0.5, 0.5 1 1 0, 0.6 1 0.5 0, 0.7 1 0 0, 0.8 0.5 0 0)\n");
    fprintf(gnuplot, "set colorbox\n");
    printf("color is %f", uncertainty / 1000.0);
    fprintf(gnuplot, "plot '-' with points pt 7 ps variable palette\n");
    fprintf(gnuplot, "%d %d 4 %f\n", x, y, uncertainty/ 1000.0);
    fprintf(gnuplot, "%d %d 4 0.0\n", opti_x, opti_y);
    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    fprintf(gnuplot, "refresh;\n");
}

void visualize(struct particle particles[N], struct measurement *z, struct particle *pos)
{

    fprintf(gnuplot, "plot '/home/pold/Documents/Internship/draug/img/sparse_board.jpg' binary filetype=jpg with rgbimage, '-' with points pt 7 ps variable palette\n");
    int j;
    for (j = 0; j < N; j++) {
      fprintf(gnuplot, "%f %f %f 0\n", particles[j].x, particles[j].y, particles[j].w * 10);
      /* printf("%f %f\n", particles[j].x, particles[j].y); */
    }

    fprintf(gnuplot, "%f %f 4 2\n", z->x, z->y);
    fprintf(gnuplot, "%f %f 4 3\n", pos->x, pos->y); /* The best prediction (weighted average) */
    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    fprintf(gnuplot, "refresh;\n");
}
