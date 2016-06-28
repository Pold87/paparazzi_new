#include "texton_settings.h"
#include "texton_helpers.h"
#include "trexton_regression.h"
#include "image_conversions.h"
#include "readcsv.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>

#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/textons.h"

#include "subsystems/gps.h"
#include "subsystems/abi.h"

#include "opticflow/opticflow_calculator.h"
#include "subsystems/datalink/telemetry.h"

#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include "state.h"

/* Histogram paths */
static char histogram_filename[] = "/home/pold/paparazzi_clean2/mat_train_hists_texton.csv";
/* static char histogram_filename_testset[] = "mat_test_hists_str8.csv"; */
/* static) char position_filename[] =  "board_train_pos.csv"; */
static char position_filename[] =  "cyberzoo_pos_optitrack.csv";
//static char position_filename[] =  "final.csv";
static char test_position_filename[] =  "cyberzoo_pos_optitrack.csv";
static struct measurement all_positions[NUM_HISTOGRAMS];
static struct measurement all_test_positions[NUM_TEST_HISTOGRAMS];

static float regression_histograms[NUM_HISTOGRAMS][SIZE_HIST];
static float regression_histograms_color[NUM_HISTOGRAMS][SIZE_HIST];
/* static int histograms_testset[NUM_TEST_HISTOGRAMS][SIZE_HIST]; */

static int current_test_histogram = 0;
static int use_variance = 0;

/* Create  particles */
struct particle particles[N];

/* The main opticflow variables */
struct opticflow_t opticflow;                      ///< Opticflow calculations
static struct opticflow_result_t opticflow_result; ///< The opticflow result
static struct opticflow_state_t opticflow_state;   ///< State of the drone to communicate with the opticflow
static pthread_mutex_t opticflow_mutex;            ///< Mutex lock fo thread safety
static bool opticflow_got_result; ///< When we have an optical flow calculation

static struct UdpSocket video_sock; /* UDP socket for sending RTP video */

static int image_num = 0;

/* The trexton camera V4L2 device */
static struct v4l2_device *trexton_dev;

/* File that contains the filters */
/* IMPORTANT: needs three decimal places !!! */
//static char *texton_filename = "textons.csv";
static char *texton_filename = "textons_malik.csv";

/* Array with the textons */
double textons[NUM_TEXTONS * CHANNELS][TOTAL_PATCH_SIZE];

static int k = 1; /* Number of nearest neighbors for kNN */

/* Uncertainty measurements */
static float entropy = 0;
static float uncertainty_x = 0;
static float uncertainty_y = 0;

float texton_histogram[NUM_TEXTONS * CHANNELS] = {0.0};

#define USE_FLOW false

/* Local function declarations */
void trexton_init(void);
struct image_t* trexton_func(struct image_t* img);


int global_x = 0;
int global_y = 0;

int global_ground_truth_x = 0;
int global_ground_truth_y = 0;

// For target landing
int closest = 2000;

void trexton_init(void)
{

  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TREXTON, send_trexton_position);

  /* Print current working directory */
   char cwd[1024];
   if (getcwd(cwd, sizeof(cwd)) != NULL)
       fprintf(stdout, "Current working dir: %s\n", cwd);
   else
       perror("getcwd() error");

  /* Initialize GPS settings  */
  /* init_positions(); */

  /* Get textons -- that is the clustering centers */
  read_textons_from_csv(textons, texton_filename);

  /* Remove predictions file */
  remove("particle_filter_preds.csv");
  remove("edgeflow_diff.csv");
  remove("knn.csv");
  remove("saved.csv");

  // Set the opticflow state to 0
  opticflow_state.phi = 0;
  opticflow_state.theta = 0;
  opticflow_state.agl = 0;

  // Initialize the opticflow calculation
  opticflow_calc_init(&opticflow, 640, 480);

  opticflow_got_result = FALSE;

#if PREDICT
  /* Read histograms */

  #if USE_COLOR
    read_color_histograms_from_csv(regression_histograms_color, histogram_filename, SIZE_HIST);
  #else
    read_histograms_from_csv(regression_histograms, histogram_filename, SIZE_HIST);
  #endif

  /* Print color histograms */
  /* int i, j; */
  /* for (i = 0; i < NUM_HISTOGRAMS; i++) { */
  /*   for (j = 0; j < NUM_COLOR_BINS * COLOR_CHANNELS; j++) { */
  /*     printf("%f ", regression_histograms_color[i][j]); */
  /*   } */
  /*   printf("\n"); */
  /* } */


  /* Read x, y, position from SIFT */
  read_positions_from_csv(all_positions, position_filename);
  read_test_positions_from_csv(all_test_positions, test_position_filename);

#endif

#if EVALUATE
  /* read_test_histograms_from_csv(histograms_testset, histogram_filename_testset); */

  /* Write header for predictions file*/
  remove("predictions.csv");
  FILE *fp_predictions;
  fp_predictions = fopen("predictions.csv", "a");
  //fprintf(fp_predictions, "id,x,y,dist\n");
  fclose(fp_predictions);

#endif

  /* Initialize particles*/
  init_particles(particles);

  /* Debugging read poitions from csv */
  /* int i; */
  /* for (i = 0; i < 10; i++) { */
  /*   printf("is is %d pos x is %f, pos y is %f\n", i, all_positions[i].x, all_positions[i].y); */
  /* } */

  printf("Adding function");
  cv_add(trexton_func);
  printf("Still there");

}

struct image_t* trexton_func(struct image_t* img) {

  printf("Starting to fly");
  printf("Type of in IMAGE  is %d\n", img->type);

  int u;
  for (u = 0; u < 20; u++) {
     printf("textons: %f ", texton_distribution[u]);
  }


  #if MEASURE_TIME
    /* clock_t start = clock(); */;
    static struct timeval t0, t1, tot;
    long elapsed;
    gettimeofday(&tot, 0);
  #endif

  /* Calculate the texton histogram -- that is the frequency of
     characteristic image patches -- for this image */

  #if USE_WEBCAM
    /* Get the image from the camera */
    /* v4l2_image_get(trexton_dev, &img); */

    #if USE_CONVERSIONS

      /* Create RGB image */
      struct image_t rgb_img, opp_img, std_img;
      image_create(&rgb_img, 320, 240, IMAGE_RGB);
      image_create(&opp_img, 320, 240, IMAGE_OPPONENT);
      image_create(&std_img, 320, 240, IMAGE_STD);
      YUV422toRGB(img, &rgb_img);
      double means[8];
      RGBtoOpponent(&rgb_img, &opp_img, means);
      image_grayscale_standardize(&opp_img, &std_img, means);
      image_grayscale_standardize(img, &std_img, means);
      printf("Means are %f, %f, %f %f\n", means[0], means[1], means[2], means[3]);
      
      uint8_t *rgb_buf = (uint8_t *)rgb_img.buf;
      uint8_t *opp_buf = (uint8_t *)opp_img.buf;
      double *std_buf = (double *)std_img.buf;
      
      printf("RGB: %d %d\n", rgb_buf[0], rgb_buf[1]);
      printf("Opponent: %d %d %d %d\n", opp_buf[0], opp_buf[1], opp_buf[2], opp_buf[3]);
      printf("STD: %f %f %f %f\n", std_buf[0], std_buf[1], std_buf[2], std_buf[3]);
    #endif
  #endif

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed first part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif

  //get_texton_histogram(img, texton_histogram, textons);

  #if USE_COLOR
    double color_hist[COLOR_CHANNELS*NUM_COLOR_BINS] = {0.0};
    get_color_histogram(img, color_hist, NUM_COLOR_BINS);
  #endif


#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed get histogram: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


#if SEND_VIDEO
  /* Send JPG image over RTP/UDP */

  // Create a new JPEG image
  struct image_t img_jpeg;
  image_create(&img_jpeg,
               trexton_dev->w,
               trexton_dev->h,
               IMAGE_JPEG);

  jpeg_encode_image(img, &img_jpeg, 70, FALSE);
  rtp_frame_send(&video_sock, /* UDP device */
                 &img_jpeg,
                 0, /* Format 422 */
                 70, /* Jpeg-Quality */
                 0,  /* DRI Header */
                 0); /* 90kHz time increment */
#endif

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed second part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


#if SAVE_HISTOGRAM
  FILE *fp_hist;
  fp_hist = fopen("saved.csv", "a");
  save_histogram_float(texton_distribution, fp_hist, NUM_TEXTONS);
  fclose(fp_hist);
#endif
#if PREDICT

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed after get histogram: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


  #if USE_COLOR
  int i;
  printf("COLOR\n");
  for (i = 0; i < COLOR_CHANNELS * NUM_COLOR_BINS; i++) {
    printf("%f ", color_hist[i]);
  }
  printf("\n");
  #else
  int i;
  /* printf("TEXTON\n"); */
  /* for (i = 0; i < SIZE_HIST; i++) { */
  /*   printf("%f ", texton_histogram[i]); */
  /* } */
  /* printf("\n"); */
  #endif

  /* TreXton prediction */
  struct measurement pos[k];
  /* For textons */
  /* pos = predict_position(texton_histogram, NUM_TEXTONS * CHANNELS); */

  /* For colors */
  #if USE_COLOR
  predict_position(pos, color_hist, COLOR_CHANNELS * NUM_COLOR_BINS);
  #else
  /* For textons */
  predict_position(pos, texton_distribution, NUM_TEXTONS * CHANNELS);
  #endif

 
  /* For colors */
  /* pos = predict_fann(texton_histogram, NUM_COLOR_BINS * COLOR_CHANNELS); */
  /* pos = linear_regression_prediction(texton_histogram); */
  printf("\nPOSITION[0] IS x:%f y:%f\n", pos[0].x, pos[0].y);
  fflush(stdout);

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed predict position: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif
  //pos = all_test_positions[current_test_histogram];
  /* save_image(&img, "afterpredict.csv"); */

  /* Optical flow prediction */
  /* TODO */

  /* Particle filter update */
  struct measurement flow;

#if USE_FLOW
  // Copy the state
  pthread_mutex_lock(&opticflow_mutex);
  struct opticflow_state_t temp_state;
  memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
  pthread_mutex_unlock(&opticflow_mutex);
  // Do the optical flow calculation
  struct opticflow_result_t temp_result;

  printf("Now calculating opticflow");
  fflush(stdout);
  /* edgeflow_calc_frame(&opticflow, &temp_state, &img, &temp_result); */
  
  opticflow_calc_frame(&opticflow, &temp_state, img, &temp_result);
  printf("\n opticflow result: x:%d y:%d\n", temp_result.flow_x, temp_result.flow_y);
  fflush(stdout);
  // Copy the result if finished
  pthread_mutex_lock(&opticflow_mutex);
  memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
  opticflow_got_result = TRUE;
  pthread_mutex_unlock(&opticflow_mutex);

  /* if (image_num == 10) { */
  /*   save_image(&std_img, "mainpic.csv");  */
  /* } */

  /* Mind the change of x, y ! */
  /* TODO: use subpixel factor instead of 1000 */
  flow.x =  9.0 * ((double) opticflow_result.flow_x) / 1000.0;
  flow.y =  9.0 * ((double) opticflow_result.flow_y) / 1000.0;

  printf("flow is %f", flow.x);
  particle_filter(particles, pos, &flow, use_variance, 1, k);
  opticflow_got_result = FALSE;
#else
  particle_filter(particles, pos, &flow, use_variance, 0, k);
#endif

  printf("I finished the particle filter");
  fflush(stdout);
  
#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed flow part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif

  /* TODO: compare weighted average and MAP estimate */
  struct particle avg = weighted_average(particles, N);
  struct particle var = calc_uncertainty(particles, avg, N);

  uncertainty_x = var.x;
  uncertainty_y = var.y;
  
  struct particle p_forward = map_estimate(particles, N);
  //struct particle p_forward = weighted_average(particles, N);
  /* printf("\nRaw: %f,%f\n", pos.x, pos.y); */
  printf("Particle filter: %f,%f\n", p_forward.x, p_forward.y);

  global_x = (int) p_forward.x;
  global_y = (int) p_forward.y;

  global_ground_truth_x = all_test_positions[image_num].x;
  global_ground_truth_y = all_test_positions[image_num].y;

  FILE *fp_predictions;
  FILE *fp_particle_filter;
  FILE *fp_edge;
  fp_predictions = fopen("predictions.csv", "a");
  fp_particle_filter = fopen("particle_filter_preds.csv", "a");
  fp_edge = fopen("edgeflow_diff.csv", "a");
  fprintf(fp_edge, "%f,%f\n", flow.x, flow.y);
  fprintf(fp_particle_filter, "%f,%f\n", p_forward.x, p_forward.y);
  fprintf(fp_predictions, "%f,%f\n", pos[0].x, pos[0].y);
  fclose(fp_predictions);
  fclose(fp_particle_filter);
  fclose(fp_edge);
#endif

  /* Send postion to ground station */
  /*send_trexton_position()*/
  /* send_pos_to_ground_station((int) p_forward.x, (int) p_forward.y); */
  current_test_histogram++;

#if !EVALUATE

#if USE_CONVERSIONS
  /* Free the image */
  image_free(&rgb_img);
  image_free(&opp_img);
  image_free(&std_img);
#endif
#if USE_WEBCAM
  /* v4l2_image_free(trexton_dev, &img); */
#else
  image_free(&rgb_img);
#endif

#endif

#if SEND_VIDEO
  /* Free RTP stream image */
  image_free(&img_jpeg);
#endif

  image_num = image_num + 1;

#if MEASURE_TIME
  /* clock_t end = clock(); */
  /* float seconds = (float) (end - start) / CLOCKS_PER_SEC; */
  /* printf("%.10f\n", seconds); */
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - tot.tv_sec) * 1000000 + t1.tv_usec - tot.tv_usec;

  printf("TOTAL ELAPSED (entire function) %ld ms\n", elapsed / 1000);
#endif

  return img;
  
}

/**
 * Predict the x, y position of the UAV using the texton histogram.
 *
 * @param texton_hist The texton histogram
 *
 * @return The x, y, position of the MAV, computed by means of the input histogram
 */
void predict_position(struct measurement pos[], float hist[], int hist_size)
{

  printf("Closest is %d\n", closest);
  int h = 0; /* Histogram iterator variable */

  struct measurement measurements[NUM_HISTOGRAMS];
  float dist;

  /* Compare current texton histogram to all saved histograms for
     a certain class */
  for (h = 0; h < NUM_HISTOGRAMS; h++) {
    /* dist = euclidean_dist_int(hist, regression_histograms[h], hist_size); */
#if USE_COLOR
    dist = chi_square_dist_double(hist, regression_histograms_color[h], hist_size);
#else
    dist = euclidean_dist_float(hist, regression_histograms[h], hist_size);
#endif

    /* printf("dist is %d %f\n", h, dist); */
    /* printf("all pos is %f\n", all_positions[h].x); */

    struct measurement z;
    z.x = all_positions[h].x;
    z.y = all_positions[h].y;
    z.hist_num = h;
    z.dist = dist;
    measurements[h] = z;

    /* printf("H is %d, dist is %f\n", h, dist); */

  }

  /* Sort distances */
  qsort(measurements, sizeof(measurements) / sizeof(*measurements), sizeof(*measurements), measurement_comp);

  /* Return average over first positions for accurate regression: */

  int l;
  /* struct  measurement mean_pos; */
  /* mean_pos.x = 0; */
  /* mean_pos.y = 0; */
  /* mean_pos.dist = 0; */

  /* FILE *fp_knn; */
  /* fp_knn = fopen("knn.csv", "a"); */
  
  /* for (l = 0; l < k; l++) { */
  /*   printf("\nmeasurement: x: %f, y: %f dist: %f num_histogram: %d\n", measurements[l].x, measurements[l].y, */
  /*          measurements[l].dist, measurements[l].hist_num); */
  /*   printf("\n\nnum_histogram: %d\n\n", measurements[l].hist_num); */
  /*   fflush(stdout); */
  /*   mean_pos.x += measurements[l].x / k; */
  /*   mean_pos.y += measurements[l].y / k; */
  /*   mean_pos.dist += measurements[l].dist / k; */

  /*   fprintf(fp_knn, "%f,%f", measurements[l].x, measurements[l].y); */
  /*   if (l != k - 1) */
  /*     fprintf(fp_knn, ","); */
  /* } */

  /* Fill struct with nearest neighbors */
  for (l = 0; l < k; l++) {
    struct measurement cur_pos;
    pos[l] = cur_pos;
    pos[l].x = measurements[l].x;
    pos[l].y = measurements[l].y;
    pos[l].dist = measurements[l].dist;
      
  }

  /* fprintf(fp_knn, "\n"); */
  /* fclose(fp_knn); */
}

static void send_trexton_position(struct transport_tx *trans, struct link_device *dev)
 {
   /* printf("global x is %d global y is %d\n", global_x, global_y); */
   fflush(stdout);
   struct NedCoor_i *ned = stateGetPositionNed_i();

   /* For using Optitrack or the simulator */
   /* pprz_msg_send_TREXTON(trans, dev, AC_ID, &global_x, &global_y, &ned->x, &ned->y, &entropy, &uncertainty_x, &uncertainty_y); */

   int i;
   /* printf("\n"); */
   /* printf("THE HISTOGRAM IS\n"); */
   /* for (i = 0; i < N; i++) { */
   /*   printf("%f ", texton_histogram[i]); */
   /* } */
   /* printf("\n"); */

   
   /* For comparing ground truth to estimates */
   pprz_msg_send_TREXTON(trans, dev, AC_ID, &global_x, &global_y,
			 &global_ground_truth_x, &global_ground_truth_y,
			 &entropy, &uncertainty_x, &uncertainty_y,
			 &texton_histogram[0],
			 &texton_histogram[1],
			 &texton_histogram[2],
			 &texton_histogram[3],
			 &texton_histogram[4],
			 &texton_histogram[5],
			 &texton_histogram[6],
			 &texton_histogram[7],
			 &texton_histogram[8],
			 &texton_histogram[9],
			 &texton_histogram[10],
			 &texton_histogram[11],
			 &texton_histogram[12],
			 &texton_histogram[13],
			 &texton_histogram[14],
			 &texton_histogram[15],
			 &texton_histogram[16],
			 &texton_histogram[17],
			 &texton_histogram[18],
			 &texton_histogram[19],
			 &texton_histogram[20],
			 &texton_histogram[21],
			 &texton_histogram[22],
			 &texton_histogram[23],
			 &texton_histogram[24],
			 &texton_histogram[25],
			 &texton_histogram[26],
			 &texton_histogram[27],
			 &texton_histogram[28],
			 &texton_histogram[29],
			 &texton_histogram[30],
			 &texton_histogram[31],
			 &texton_histogram[32]);
 }


/* Initialize GPS settings  */
void init_positions(void)
{
  //gps.fix = GPS_FIX_NONE;
  /* gps.fix = GPS_FIX_3D; */
  /* gps_available = TRUE; */
  /* gps.gspeed = 700; // To enable course setting */
  /* gps.cacc = 0; // To enable course setting */

  /* // CyberZoo ref point is in: */
  /* // https://github.com/tudelft/infinium_video/blob/volker/coordTransforms.py */
  /* gps.ecef_pos.x = 392433249; */
  /* gps.ecef_pos.y = 30036183; */
  /* gps.ecef_pos.z = 500219779; */

}


int targetPos_X = 78;
int targetPos_Y = 56;
int accuracy = 25;

/* Check if the current position is in the circle spanned by the
   target position and the desired accuracy */
uint8_t isInTargetPos(uint8_t wp_id)
{

  //  int targetPos_X_i = waypoint_get_x_int(wp_id);
  //int targetPos_Y_i = waypoint_get_y_int(wp_id);


  int targetPos_X_i = targetPos_X;
  int targetPos_Y_i = targetPos_Y;


  printf("target pos x %d, target pos y %d", targetPos_X_i, targetPos_Y_i);
  int dist_x = targetPos_X_i - global_x;
  int dist_y = targetPos_Y_i - global_y;
  int d = sqrt(dist_x * dist_x + dist_y * dist_y);

  uint8_t inside = d < accuracy;

  if (d < closest)
    closest = d;

  printf("\nglob x: %d, glob y:%d, dist is %d closest %d\n", global_x, global_y, d, closest);

  return inside;
  
}

float tolerated_x_dev = 100.0;
float tolerated_y_dev = 100.0;

/* Check if the particle filter is certain that the current estimates
   are valid */
uint8_t isCertain(void)
{

  uint8_t isCertain = 0;
  if (uncertainty_x < tolerated_x_dev && uncertainty_y < tolerated_y_dev)
    isCertain = 1;

  return isCertain;
  
}
