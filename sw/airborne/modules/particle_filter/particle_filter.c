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

/* Initialize particles either uniformly at random or with an informed
 * prior, if the starting positions of the UAV is known */
void init_particles(struct particle particles[N]){

  srand(time(NULL)); /* Set random seed */
  int i;
  for (i = 0; i < N; i++) {

    /* Initialize with informed prior */
    if (informed_prior) {
      particles[i].x = randu(1060, 1080);
      particles[i].y = randu(275, 285);
      particles[i].w = 1;
      particles[i].prev_w = 1;
    }

    /* Initialize with random x, y-positions */
    else {

       /* TODO: randu seems to be biased (see particle filter video) */
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


/* Sebastian Thrun's resampling wheel */
/* Returns total weight */
double resampling_wheel(struct particle ps[], struct particle res[], double weights[], int samples) {

    /* p: particles */
    /* w: weights */
    /* N: Desired number of particles */

  int idx = randu(0, 1) * samples;
  double beta = 0.0;
  double mw = array_max(weights, samples);
  double total = 0;

  int i;
  for (i = 0; i < N; i++) {
    beta += randu(0, 1) * 2.0 * mw;
    while (beta > weights[idx]){
      beta -= weights[idx];
      idx = (idx + 1) % N;
    }
    res[i] = ps[idx];
    total += res[i].w;
  }

  return total;
}


void particle_filter(struct particle xs[N], struct measurement z[], struct measurement *flow,
         int use_variance, int use_flow, int num_predictions) {

  double w[N]; /* The weights of particles */
  double rho;

  /* IMPORTANT REMOVE AGAIN */
  int debug_flow = 0;

  if (use_flow) {
    process_noise_x = 10;
    process_noise_y = 10;
  } else {
    process_noise_x = 15.0;
    process_noise_y = 25.0;
  }

  double measurement_noise_x;
  double measurement_noise_y;

  if (use_variance) {

    measurement_noise_x = z[0].dist * 1000;
    measurement_noise_y = z[0].dist * 1000;
    printf("Measurement noise x is %f", measurement_noise_x);

  } else {
    measurement_noise_x = 92;
    measurement_noise_y = 134;
    rho = 0.0;
  }

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
    double p;
    double total_likelihood = 0.00000001;
    int pred;
    for (pred = 0; pred < num_predictions; pred++) {

       p = binormpdf(xs[i].x, xs[i].y, z[pred].x, z[pred].y,
                     measurement_noise_x, measurement_noise_y, rho);
       total_likelihood += p;
    }

      /* Update particles */
      xs[i].prev_w = xs[i].w;
      w[i] = total_likelihood;
      xs[i].w = w[i]; /* Set weight of particle */

  }

  /* Importance resampling: (iterate over all particles) */
  struct particle res[N];
  double total = resampling_wheel(xs, res, w, N);

  /* Copy results */
  for (i = 0; i < N; i++) {
     xs[i] = res[i];
  }

  /* Normalize weights such that their sum adds up to  1 */
  for (i = 0; i < N; i++) {
    xs[i].w = xs[i].w / total;
  }
}

/* Calculate the weighted average (expectation) of the particles */
/* Returns the results as a particle with the calculated x, y
 * position */
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

/* Maximum a posteriori estimate of the x, y position using the
 * particles */
/* See paper MAP Estimation in Particle Filter Tracking */
struct particle map_estimate(struct particle ps[], int size) {

  int i, j;

  double t_max;
  int t_argmax;
  t_max = -1.0;
  t_argmax = 0;

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

/* Calculate the uncertainty of the particle filter (variance of the particles) */
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

/* Helper functions */

/* Maximum value of an array with non-negative values */
double array_max(double arr[], int size){

  double m = -1;
  int i;
  for (i = 0; i < size; i++) {
    if (arr[i] > m)
      m = arr[i];
  }
  return m;
}


/* Gaussian probability density function */
double normpdf(double x, double mu, double sigma) {

  double density, a;
  static const double inv_sqrt_2pi = 0.3989422804014327;

  a = (x - mu) / sigma;
  density = inv_sqrt_2pi / sigma * exp(-0.5 * a * a);

  return density;

}


/* Bivariate Gaussian probability density function */
double binormpdf(double x1, double x2, double mu1, double mu2, double sigma1, double sigma2, double rho) {

   double p;
   double z, z1, z2, z3;

   z1 = ((x1 - mu1) * (x1 - mu1)) / (sigma1 * sigma1);
   z2 = (2 * rho * (x1 - mu1) * (x2 - mu2)) / (sigma1 * sigma2);
   z3 = ((x2 - mu2) * (x2 - mu2)) / (sigma2  * sigma2);
   z = z1 - z2 + z3;

   p = (1 / (2 * PI * sigma1 * sigma2 * sqrt(1 - (rho * rho)))) * exp(- (z / (2 * (1 - (rho * rho)))));

   return p;

}
