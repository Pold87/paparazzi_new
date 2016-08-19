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

#include "subsystems/gps.h"
#include "subsystems/abi.h"

/* #include "cv.h" */
#include "textons.h"
#include "opticflow_module.h"
#include "subsystems/datalink/telemetry.h"

#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include "state.h"

#include "subsystems/abi.h"

/* Default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_ID
#define OPTICFLOW_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)


/* For getting opticflow */
static abi_event opticflow_ev;

#define VIDEO_CAPTURE_JPEG_QUALITY 99

//uint8_t save_histogram = TREXTON_SAVE_HISTOGRAM;
uint8_t save_histogram = 0;
uint8_t stream_video = TREXTON_SEND_VIDEO;
uint8_t use_color = TREXTON_USE_COLOR;
uint8_t predict = TREXTON_PREDICT;
//uint8_t use_flow = TREXTON_USE_FLOW;
uint8_t use_flow = 0;
//uint8_t evaluate = TREXTON_EVALUATE;
uint8_t evaluate = 1;
uint8_t use_particle_filter = 1; // Use particle filter or plain kNN predictions
uint8_t use_optitrack = TREXTON_USE_OPTITRACK;
//uint8_t use_optitrack = 1;
uint8_t trexton_save_img_and_histogram = TREXTON_SAVE_IMG_AND_HISTOGRAM;
uint8_t trexton_save_img = TREXTON_SAVE_IMG;

static int k = TREXTON_K; /* Number of nearest neighbors for kNN */
static int use_variance = 0;

/* Histograms and their paths */
static char histogram_filename[] = "histograms_flight.csv";
static char position_filename[] =  "positions.csv";
//static char position_filename[] =  "sift_rotated.csv";
static char test_position_filename[] =  "cyberzoo_pos_optitrack.csv";
static struct measurement all_positions[TREXTON_NUM_HISTOGRAMS];
static struct measurement all_test_positions[TREXTON_NUM_TEST_HISTOGRAMS];
static float regression_histograms[TREXTON_NUM_HISTOGRAMS][TREXTON_SIZE_HIST];
static float regression_histograms_color[TREXTON_NUM_HISTOGRAMS][TREXTON_SIZE_HIST];
static int current_test_histogram = 0;
static float texton_dist_copy[20];

/* The distributions */
double color_hist[TREXTON_COLOR_CHANNELS*TREXTON_NUM_COLOR_BINS] = {0.0};

/* Global variables */
struct particle particles[N];
struct particle p_forward; /* Particle filter result */
struct measurement flow; /* Optical flow result */
struct measurement pos[TREXTON_K]; /* Estimates of kNN */
struct particle var; /* Variance of particles */
static int image_num = 0;
static int image_tot = 0;

/* Landing sign */
//int targetPos_X = 242;
//int targetPos_Y = 84;

/* Red magazine */
//int targetPos_X = 424;
//int targetPos_Y = 504;

/* Other corner */
int targetPos_X = 59;
int targetPos_Y = 392;


int accuracy = 70;
float global_ground_truth_x = 0;
float global_ground_truth_y = 0;
int closest = 2000; // For target landing: what was the closest position to the goal
float tolerated_x_dev = 80.0;
float tolerated_y_dev = 80.0;

/* File pointers for saving predictions */
static FILE *fp_predictions = NULL;
static FILE *fp_ground_truth = NULL;
static FILE *fp_particle_filter = NULL;
static FILE *fp_edge = NULL;
static FILE *fp_hist = NULL; // Histograms saved during flight
static FILE *fp_pos = NULL; // Positions saved during flight


static FILE *fp_knn1 = NULL; // nearest neighbor
static FILE *fp_knn2 = NULL; // second nearest neighbor
static FILE *fp_knn3 = NULL; // ...
static FILE *fp_knn4 = NULL; // ...
static FILE *fp_knn5 = NULL; // ...


static int opti_offset_x = 199;
static int opti_offset_y = 729;

/* Others */
static struct UdpSocket video_sock; /* UDP socket for sending RTP video */
static struct v4l2_device *trexton_dev; /* The trexton camera V4L2 device */
static void send_trexton_position(struct transport_tx *trans, struct link_device *dev);
static void save_predictions(void);
void send_video(struct image_t* img);
uint8_t isCertain(void);
static void save_img_to_uav(struct image_t *img);
//static void opticflow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float dist);
static void opticflow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, float vel_body_x, float vel_body_y, float dist, float noise);

/**
 * Main function of treXton (texton regression) that estimates a UAV's position based on texton distributions
 * @param[out] *img The output image
 * @param[in] *img The input image (YUV422)
 */
struct image_t *trexton_func(struct image_t *img) {

  //printf("register");
  /* TODO: maybe move to init */
  register_periodic_telemetry(DefaultPeriodic, 24, send_trexton_position);

   /* Use color histograms or just textons? */
   if (use_color) {
      get_color_histogram(img, color_hist, TREXTON_NUM_COLOR_BINS);
   }

   /* Stream video to ground station */
   if (stream_video) {
      send_video(img);
   }


   /* Copy histogram */
   int u;
   for (u = 0; u < TREXTON_NUM_TEXTONS; u++) {
     texton_dist_copy[u] =  texton_distribution[u];
   }

   /* Save img to path on UAV */
   if (trexton_save_img || trexton_save_img_and_histogram) {
     save_img_to_uav(img);
   }

   /* Save extracted histogram to file (the histogram is received from the texton module) */
   if (save_histogram || trexton_save_img_and_histogram) {

     /* Save positions */
     /* struct NedCoor_i *ned = stateGetPositionNed_i(); */
     /* fp_pos = fopen("positions.csv", "a"); */
     /* fprintf(fp_pos, "%d,%d,%d\n", ned->x, ned->y, 100); */
     /* fclose(fp_pos); */

     /* Save histograms */
     fp_hist = fopen("histograms_flight.csv", "a");
     save_histogram_float(texton_distribution, fp_hist, TREXTON_NUM_TEXTONS);
     fclose(fp_hist);
   }

   /* Increment image number (e.g. used for file name in save path) */
   if (save_histogram || trexton_save_img || trexton_save_img_and_histogram)
     image_num++;

  /* Predict the positions of the UAV and save them*/
  if (predict) {

     /* For colors */
     if (use_color) {
        predict_position(pos, color_hist, TREXTON_COLOR_CHANNELS * TREXTON_NUM_COLOR_BINS);
     } else {
        /* For textons */
        predict_position(pos, texton_distribution, TREXTON_NUM_TEXTONS * TREXTON_CHANNELS);

  printf("kNN: %f,%f\n", pos[0].x, pos[0].y);

     }

     if (use_particle_filter) {

       /* Use optical flow for particle filter updates */
       /* TODO: use opticalflow_result here */

       if (use_flow) {
   particle_filter(particles, pos, &flow, use_variance, 1, k);
   printf("Flow is: x:%f y:%f\n", flow.x, flow.y);
   flow.x = 0;
   flow.y = 0;
       } else {
   particle_filter(particles, pos, &flow, use_variance, 0, k);
           int i;
        printf("\n");
        for (i = 0; i < N; i++) {
           printf("particles: %f, %f", particles[i].x, particles[i].y);
        }
       }

        /* TODO: compare weighted average and MAP estimate */
        /*p_forward = weighted_average(particles, N);*/

        static FILE *fp_particles = NULL;
        char fp_particles_name[256];
        sprintf(fp_particles_name, "particles/particles_%05d.csv", image_tot);
        fp_particles = fopen(fp_particles_name, "w");

        for (int a = 0; a < N; a++) {
           fprintf(fp_particles, "%f,%f\n", particles[a].x, particles[a].y);
        }

        //var = calc_uncertainty(particles, avg, N);
        p_forward = map_estimate(particles, N);
  //p_forward = avg;
        printf("Particle filter: %f,%f\n", p_forward.x, p_forward.y);

     }
  }

  // Check for landing (DEBUG)
  uint8_t do_landing = isInTargetPos();
  uint8_t is_certain = isCertain();

  if (do_landing) {
    printf("\n\n\n LANDING!!!\n\n\n\n\n");
  }

  current_test_histogram++;

  /* Get Optitrack positions */
  struct NedCoor_i *ned = stateGetPositionNed_i();
  printf("\nOPTITRACK is: x: %d y: %d\n", ned->x + 199, ned->y + 729);

  if (evaluate) {
    /* Set global variables */
    //    global_ground_truth_x = all_test_positions[image_num].x;
    //    global_ground_truth_y = all_test_positions[image_num].y;
    save_predictions();
  }

  image_tot++;
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

  struct measurement measurements[TREXTON_NUM_HISTOGRAMS];
  float dist;

  /* Compare current texton histogram to all saved histograms for
     a certain class */
  for (h = 0; h < TREXTON_NUM_HISTOGRAMS; h++) {
    /* dist = euclidean_dist_int(hist, regression_histograms[h], hist_size); */
     if (use_color)
        dist = chi_square_dist_double(hist, regression_histograms_color[h], hist_size);
     else
        dist = euclidean_dist_float(hist, regression_histograms[h], hist_size);

    struct measurement z;

    //TODO: CHECK IF READ POSITION REALLY WORKS

    z.x = all_positions[h].x;
    z.y = all_positions[h].y;
    z.hist_num = h;
    z.dist = dist;
    measurements[h] = z;

  }

  /* Sort distances */
  qsort(measurements, sizeof(measurements) / sizeof(*measurements), sizeof(*measurements), measurement_comp);

  int l;

  /* Fill struct with nearest neighbors */
  for (l = 0; l < k; l++) {
    struct measurement cur_pos;
    pos[l] = cur_pos;
    pos[l].x = measurements[l].x;
    pos[l].y = measurements[l].y;
    pos[l].dist = measurements[l].dist;

  }

}

/* Check if the current position is in the circle spanned by the
   target position and the desired accuracy */
//uint8_t isInTargetPos(uint8_t wp_id)
uint8_t isInTargetPos(void)
{

  //int targetPos_X_i = waypoint_get_x_int(wp_id);
  //int targetPos_Y_i = waypoint_get_y_int(wp_id);

  int targetPos_X_i = targetPos_X;
  int targetPos_Y_i = targetPos_Y;

  //printf("target pos x %d, target pos y %d", targetPos_X_i, targetPos_Y_i);

  int dist_x = targetPos_X_i - p_forward.x;
  int dist_y = targetPos_Y_i - p_forward.y;

  /* int dist_x = targetPos_X_i - pos[0].x; */
  /* int dist_y = targetPos_Y_i - pos[0].y; */

  int d = sqrt(dist_x * dist_x + dist_y * dist_y);

  uint8_t inside = d < accuracy;

  if (d < closest)
    closest = d;

  printf("\nglob x: %f, glob y:%f, dist is %d closest %d\n",
         p_forward.x, p_forward.y, d, closest);

  return inside;

}

/* Check if the particle filter is certain that the current estimates
   are valid */
uint8_t isCertain(void)
{

  uint8_t isCertain = 0;
  if (sqrt(var.x) < tolerated_x_dev && sqrt(var.y) < tolerated_y_dev)
    isCertain = 1;

  return isCertain;

}

/* Send video to ground stations */
void send_video(struct image_t* img) {
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
   image_free(&img_jpeg);
}


/* Save the predictions to files (for evaluating them) */
void save_predictions(void) {
  struct NedCoor_i *ned = stateGetPositionNed_i();

  /* Best unfiltered prediction */
  fp_predictions = fopen("knn.csv", "a");
  fprintf(fp_predictions, "%f,%f,%f\n", pos[0].x, pos[0].y, pos[0].dist);
  fclose(fp_predictions);

  /* /\* Predictions of the nearest neighbors with their distance *\/ */
  /* fp_knn1 = fopen("knn1.csv", "a"); */
  /* fp_knn2 = fopen("knn2.csv", "a"); */
  /* fp_knn3 = fopen("knn3.csv", "a"); */
  /* fp_knn4 = fopen("knn4.csv", "a"); */
  /* fp_knn5 = fopen("knn5.csv", "a"); */

  /* fprintf(fp_knn1, "%f,%f,%f\n", pos[0].x, pos[0].y, pos[0].dist); */
  /* fprintf(fp_knn2, "%f,%f,%f\n", pos[1].x, pos[1].y, pos[1].dist); */
  /* fprintf(fp_knn3, "%f,%f,%f\n", pos[2].x, pos[2].y, pos[2].dist); */
  /* fprintf(fp_knn4, "%f,%f,%f\n", pos[3].x, pos[3].y, pos[3].dist); */
  /* fprintf(fp_knn5, "%f,%f,%f\n", pos[4].x, pos[4].y, pos[4].dist); */

  /* fclose(fp_knn1); */
  /* fclose(fp_knn2); */
  /* fclose(fp_knn3); */
  /* fclose(fp_knn4); */
  /* fclose(fp_knn5); */

   /* Filtered predictions */
   fp_particle_filter = fopen("particle_filter_preds.csv", "a");
   fprintf(fp_particle_filter, "%d,%f,%f\n", image_tot, p_forward.x, p_forward.y);
   fclose(fp_particle_filter);

   /* Ground truth of optitrack */
   fp_ground_truth = fopen("optitrack_preds.csv", "a");
   fprintf(fp_ground_truth, "%d,%f,%f\n", image_tot, ned->x + + ((float) opti_offset_x), ned->y + + ((float) opti_offset_y));
   fclose(fp_ground_truth);

   /* Save opticalfow */
   //fp_edge = fopen("edgeflow_diff.csv", "a");
   //fprintf(fp_edge, "%f,%f\n", flow.x, flow.y);
   //fclose(fp_edge);
}


/* INITIALIZE */
void trexton_init(void)
{

  var.x = 40;
  var.y = 40;

  AbiBindMsgVELOCITY_ESTIMATE(OPTICFLOW_ID, &opticflow_ev, opticflow_cb);

  printf("treXton init\n");

  /* Remove predictions file */
  //remove("particle_filter_preds.csv");
  //remove("edgeflow_diff.csv");
  //remove("knn.csv");
  //remove("histograms_flight.csv");
  //remove("texton_img.csv");

  /* if (!predict) */
  /*   remove("positions.csv"); */

if (predict) {

     /* Read histograms */
     if (use_color) {
        read_color_histograms_from_csv(regression_histograms_color,
                                       histogram_filename, TREXTON_SIZE_HIST);
     } else {
        read_histograms_from_csv(regression_histograms,
                                 histogram_filename, TREXTON_SIZE_HIST);
     }


     if (use_optitrack)
       /* Read x, y, position from Optitraick */
       read_optitrack_positions_from_csv(all_positions, position_filename);
     else
       /* Read x, y, position from SIFT */
       read_positions_from_csv(all_positions, position_filename);
  }

struct measurement mypos; /* Estimates of kNN */
int i;

for (i = 0; i < TREXTON_NUM_HISTOGRAMS; i++) {
   mypos = all_positions[i];
   printf("Raw: %f,%f\n", mypos.x, mypos.y);
}


 /* if (evaluate) { */
 /*   read_test_positions_from_csv(all_test_positions, test_position_filename); */
 /* } */

  /* Initialize particles*/
  init_particles(particles);

  /* Add treXton to computer vision module */
  cv_add(trexton_func);
}


static void save_img_to_uav(struct image_t *img) {

  // Declare storage for image location
  char save_name[128];

  sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(TREXTON_VIDEO_CAPTURE_PATH), image_num);

  // Open file
  FILE *fp = fopen(save_name, "w");
  if (fp == NULL) {
    printf("[video_capture] Could not write shot %s.\n", save_name);
    /* TODO: return -1 */
  }

  // Create jpg image from raw frame
  struct image_t img_jpeg;
  image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
  jpeg_encode_image(img, &img_jpeg, VIDEO_CAPTURE_JPEG_QUALITY, true);

  // Save it to the file and close it
  fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
  fclose(fp);

  // Free image
  image_free(&img_jpeg);

}


/* static void opticflow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float dist) */
/* { */
/*   printf("CALLBACK !!\n"); */
/*   flow.x = ((float) flow_x) / 1000.0; */
/*   flow.y = ((float) flow_y) / 1000.0; */

/*   /\* flow.x = ((float) vel_body_x) / 1.0; *\/ */
/*   /\* flow.y = ((float) vel_body_y) / 1.0;  *\/ */
/* } */

static void opticflow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, float vel_body_x, float vel_body_y, float dist, float noise)
{
  printf("CALLBACK !!\n");
  flow.x = vel_body_x;
  flow.y = vel_body_y;

  /* flow.x = ((float) vel_body_x) / 1.0; */
  /* flow.y = ((float) vel_body_y) / 1.0;  */
}


static void send_trexton_position(struct transport_tx *trans, struct link_device *dev) {

  struct NedCoor_i *ned = stateGetPositionNed_i();

   /* For using Optitrack or the simulator */
   /* pprz_msg_send_TREXTON(trans, dev, AC_ID, &global_x, &global_y, &ned->x, &ned->y, &entropy, &uncertainty_x, &uncertainty_y); */

   float uncertainty_x = var.x;
   float uncertainty_y = var.y;
   float entropy = 0.0;
   float global_x = p_forward.x;
   float global_y = p_forward.y;

   global_ground_truth_x = ((float) ned->x) + ((float) opti_offset_x);
   global_ground_truth_y = ((float) ned->y) + ((float) opti_offset_y);


   //float global_x = pos[0].x;
   //float global_y = pos[0].y;

   /* For comparing ground truth to estimates */
   pprz_msg_send_TREXTON(trans, dev, AC_ID, &global_x, &global_y,
       &global_ground_truth_x, &global_ground_truth_y,
       &entropy, &uncertainty_x, &uncertainty_y,
       &texton_dist_copy[0],
       &texton_dist_copy[1],
       &texton_dist_copy[2],
       &texton_dist_copy[3],
       &texton_dist_copy[4],
       &texton_dist_copy[5],
       &texton_dist_copy[6],
       &texton_dist_copy[7],
       &texton_dist_copy[8],
       &texton_dist_copy[9],
       &texton_dist_copy[10],
       &texton_dist_copy[11],
       &texton_dist_copy[12],
       &texton_dist_copy[13],
       &texton_dist_copy[14],
       &texton_dist_copy[15],
       &texton_dist_copy[16],
       &texton_dist_copy[17],
       &texton_dist_copy[18],
       &texton_dist_copy[19]);
 }
