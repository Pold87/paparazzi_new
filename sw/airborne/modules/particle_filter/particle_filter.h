#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#define N 1000 /* Total number of particles */
#define M 200 /* Number of fake measurements */

struct particle {

  float prev_x;
  float prev_y;
  
  float x;
  float y;
  float heading;
  float vel_x;
  float vel_y;
  float w; /* Weight (fitness) of the particle */
  float prev_w; /* Weight  at t- 1 of the particle */
};

struct measurement {

  float x;
  float y;
  float uncertainty;
  float dist;
  float var_x; /* Variance of predictions in x direction */
  float var_y; /* Variance of predictions in y direction */
  int hist_num;
};

struct sift {

  double x;
  double y;
  int matches;
};


/**
 * @brief Calculates the value of x of a normal distribution with mean mu and
 * standard deviation sigma.
 *
 * @param x
 * @param mu Mean of the normal distribution
 * @param sigma Standard deviation of the normal distribution
 *
 * @return Value of the normal distribution at position x
 */
double normpdf(double x, double mu, double sigma);

double array_max(double arr[], int size);
void resampling_wheel(struct particle ps[], struct particle res[], double weights[], int samples);

void init_particles(struct particle particles[N]);
/* void particle_filter(struct particle particles[N], struct measurement *z, struct measurement *flow, int use_variance, int use_flow); */
void particle_filter(struct particle xs[N], struct measurement z[], struct measurement *flow,
		     int use_variance, int use_flow, int num_predictions);
void particle_filter_multiple(struct particle particles[N], struct measurement *z, struct measurement *z2, int use_variance);
void read_measurements_from_csv(struct measurement zs[], char filename[], int size);
struct particle weighted_average(struct particle ps[], int size);
struct particle map_estimate(struct particle ps[], int size);
void weighted_sample(struct particle ps[], struct particle res[], double weights[], int samples);
struct particle weight_forward_backward(struct particle p_forward, struct particle p_backward, int i, int k);
struct particle calc_uncertainty(struct particle particles[], struct particle weighted_mean, int size);
void init_visualize(void);
void visualize(struct particle xs[N], struct measurement *z, struct particle *pos);
void visualize_simple(double x, double y);
void visualize_optitrack(int x, int y, int opti_x, int opti_y, double uncertainty);


#endif
