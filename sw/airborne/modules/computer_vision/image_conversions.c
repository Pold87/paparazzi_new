#include <stdio.h>
#include "image_conversions.h"
#include "lib/vision/image.h"
#include <math.h>

/* http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1301930 */
double fmax(double a, double b);
double fmin(double a, double b);

void YUV422toRGB(struct image_t *img_yuv, struct image_t *img_rgb){

  int rgb_pos = 0;
  int i;

  uint8_t *yuv_buf = (uint8_t *)img_yuv->buf;
  uint8_t *rgb_buf = (uint8_t *)img_rgb->buf;

  for (i = 0; i < img_yuv->w * img_yuv->h * 2; i = i + 4) {
  
  uint8_t Y1 = yuv_buf[i+1];
  uint8_t U = yuv_buf[i];
  uint8_t Y2 = yuv_buf[i+3];
  uint8_t V = yuv_buf[i+2];


  int C1 = Y1 - 16;
  int C2 = Y2 - 16;
  int D = U - 128;
  int E = V - 128;

  int R1 = (298 * C1 + 409 * E + 128) >> 8;
  /* printf("R1: %d ", R1); */
  R1 = clamp(R1);

  int G1 = (298  * C1 - 100 * D - 208 * E + 128) >> 8;
  /* printf("G1: %d ", G1); */
  G1 = clamp(G1);
  int B1 = (298 * C1 + 516 * D + 128) >> 8;
  B1 = clamp(B1);
  /* printf("B1: %d\n", B1); */
  
  int R2 = clamp((298 * C2 + 409 * E + 128) >> 8);
  int G2 = clamp((298  * C2 - 100 * D - 208 * E + 128) >> 8);
  int B2 = clamp((298 * C2 + 516 * D + 128) >> 8);

  rgb_buf[rgb_pos++] = R1;
  rgb_buf[rgb_pos++] = G1;
  rgb_buf[rgb_pos++] = B1;

  rgb_buf[rgb_pos++] = R2;
  rgb_buf[rgb_pos++] = G2;
  rgb_buf[rgb_pos++] = B2;
  }
}


void RGBtoOpponent(struct image_t *img_rgb, struct image_t *img_opp, double means[8]){

   uint8_t *rgb_buf = (uint8_t *)img_rgb->buf;
   uint8_t *opp_buf = (uint8_t *)img_opp->buf;

   /* double sum_E = 0, sum_E_L = 0, sum_E_LL = 0; */

   int n = 0;
   double E_mean = 0.0, E_L_mean = 0.0, E_LL_mean = 0.0;
   double E_M2 = 0.0, E_L_M2 = 0.0, E_LL_M2 = 0.0;
   double delta_E, delta_E_L, delta_E_LL;

   double E_max = -1000;
   double E_min = 1000;
   
   int i;
   for (i = 0; i < img_rgb->w * img_rgb->h * 3; i = i + 3) {
     /* Intensity, blue-yellow, green-red */
     int E, E_L, E_LL;

     E = 0.06 * rgb_buf[i] + 0.63 * rgb_buf[i+1] + 0.27 * rgb_buf[i+2];
     E_L = 0.30 * rgb_buf[i] + 0.04 * rgb_buf[i+1] -0.35 * rgb_buf[i+2];
     E_LL = 0.34 * rgb_buf[i] - 0.60 * rgb_buf[i+1] + 0.17 * rgb_buf[i+2];

     n++;
     delta_E = E - E_mean;
     delta_E_L = E_L - E_L_mean;
     delta_E_LL = E_LL - E_LL_mean;

     E_mean += delta_E / n;
     E_M2 += delta_E * (E - E_mean);
     
     E_L_mean += delta_E_L / n;
     E_L_M2 += delta_E_L * (E_L - E_L_mean);
     
     E_LL_mean += delta_E_LL / n;
     E_LL_M2 += delta_E_LL * (E_LL - E_LL_mean);

     if (E > E_max) {
       E_max = E;
     }

     if (E < E_min) {
       E_min = E;
     }
     
     opp_buf[i] = E;
     opp_buf[i+1] = E_L + 128;
     opp_buf[i+2] = E_LL + 128;

   }

   means[0] = E_mean;
   means[1] = E_L_mean;
   means[2] = E_LL_mean;

   means[3] = E_M2 / (n - 1);
   means[4] = E_L_M2 / (n - 1);
   means[5] = E_LL_M2 / (n - 1);

   means[6] = E_max;
   means[7] = E_min;
   
 }

/* Convert RGB image to YUV422 image (u1y1 v1y2 u3y3)  */
void RGBtoYUV422(struct image_t *img_rgb, struct image_t *img_yuv){
   uint8_t *rgb_buf = (uint8_t *)img_rgb->buf;
   uint8_t *yuv_buf = (uint8_t *)img_yuv->buf;

   int i;
   uint32_t yuv_pos = 0;
   for (i = 0; i < img_rgb->w * img_rgb->h * 3; i = i + 3) {
     /* Intensity, blue-yellow, green-red */
     int Y, U, V;

     Y = 0.299 * rgb_buf[i] + 0.587 * rgb_buf[i+1] + 0.114 * rgb_buf[i+2];
     Y = clamp(Y);
     
     U = (rgb_buf[i+2] - Y) * 0.493 + 128;
     U = clamp(U);
     
     /* printf("U is: %d \n", U); */
     V = (rgb_buf[i] - Y) * 0.877 + 128;
     V = clamp(V);
     /* printf("V is %d \n", V); */
     
     /* Set U oder V value? */
     if ((i % 2) == 0)
       yuv_buf[yuv_pos++] = U;
     else
       yuv_buf[yuv_pos++] = V;
     
     /* Y value is set every time */
     yuv_buf[yuv_pos++] = Y;

   }

}

void image_grayscale_standardize(struct image_t *img, struct image_t *img_standardized, double means[8]){

  uint8_t *buf = img->buf;
  double *std_buf = img_standardized->buf;

  int interlace = 3;
  int i;
  int pos_std = 0;
  if (means[3] == 0) {
    means[3] += 0.00001;
  }
  for (i = 0; i < img->w * img->h * interlace; i = i + interlace) {   

    /* printf("means: %f, %f", means[7], means[6]); */
    /* std_buf[pos_std++] = ((double) buf[i] - means[0]) / (sqrt(means[3] * 10)); */
    if (means[6] != 0.0) {
      std_buf[pos_std++] =  ((double) buf[i] - means[7]) / (means[6] - means[7]);      
    }
    
    if (i == 0) {
      printf("buf[i]: %d, means: %f sqrt %f std_buf: %f\n", buf[i], means[0], sqrt(means[3]), std_buf[pos_std-1]);
    }
   }
}


/* Maximum of two numbers */
double fmax(double a, double b) {

  if (a > b)
    return a;
  else
    return b;
}

/* Minimum of two numbers */
double fmin(double a, double b) {

  if (a < b)
    return a;
  else
    return b;
}

int clamp(int num){
  return fmin(fmax(0, num), 255);
}

double double_clamp(double num){
  return double_min(double_max(0.0, num), 255.0);
}

double double_min(double x, double y){
  if (x < y)
    return x;
  else
    return y;
}

double double_max(double x, double y){
  if (x < y)
    return y;
  else
    return x;
}
