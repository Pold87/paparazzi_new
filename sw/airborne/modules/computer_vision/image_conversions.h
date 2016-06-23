#ifndef IMAGE_CONVERSIONS_H
#define IMAGE_CONVERSIONS_H

#include "lib/vision/image.h"

void YUV422toRGB(struct image_t *img_yuv, struct image_t *img_rgb);
void RGBtoOpponent(struct image_t *img_rgb, struct image_t *img_opp, double means[3]);
void RGBtoYUV422(struct image_t *img_rgb, struct image_t *img_yuv);
int clamp(int num);
double double_clamp(double num);
double double_min(double x, double y);
double double_max(double x, double y);

void image_grayscale_standardize(struct image_t *img, struct image_t *img_standardized, double means[6]);

#endif
