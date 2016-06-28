#include <stdlib.h>
#include <stdio.h>
#include <png.h>
#include "lib/vision/image.h"
#include "readpng.h"
#include "image_conversions.h"


void read_png_file(char *filename, struct image_t *img) {

  int width, height;
  png_byte color_type;
  png_byte bit_depth;
  png_bytep *row_pointers;

  FILE *fp = fopen(filename, "rb");

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if(!png) abort();

  png_infop info = png_create_info_struct(png);
  if(!info) abort();

  //if(setjmp(png_jmpbuf(png))) abort();

  png_init_io(png, fp);
  
  png_read_info(png, info);

  width      = png_get_image_width(png, info);
  height     = png_get_image_height(png, info);
  color_type = png_get_color_type(png, info);
  bit_depth  = png_get_bit_depth(png, info);

  // Read any color_type into 8bit depth, RGBA format.
  // See http://www.libpng.org/pub/png/libpng-manual.txt

  if(bit_depth == 16)
    png_set_strip_16(png);

  if(color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png);

  // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand_gray_1_2_4_to_8(png);

  if(png_get_valid(png, info, PNG_INFO_tRNS))
    png_set_tRNS_to_alpha(png);

  // These color_type don't have an alpha channel then fill it with 0xff.
  if(color_type == PNG_COLOR_TYPE_RGB ||
     color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

  if(color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_gray_to_rgb(png);

  png_read_update_info(png, info);

  row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  for(int y = 0; y < height; y++) {
    row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
  }

  png_read_image(png, row_pointers);

  png_destroy_read_struct(&png, &info, NULL);
  png=NULL;
  info=NULL;

  int read_opponent = 0;

  if (read_opponent) {
     /* Read as opponent color space  */
     for(int y = 0; y < height; y++) {
        png_bytep row = row_pointers[y];

        for(int x = 0; x < width; x++) {
           png_bytep px = &(row[x * 4]);

           /* Intensity, blue-yellow, green-red */
           int E;
           //int E_L, E_LL;
           E = 0.06 * px[0] + 0.63 * px[1] + 0.27 * px[2];
           //E_L = 0.30 * px[0] + 0.04 * px[1] -0.35 * px[2];
           //E_LL = 0.34 * px[0] - 0.60 * px[1] + 0.17 * px[2];
           ((uint8_t*) img->buf)[y * width + x] = (uint8_t) E;
        }
     }
  } else {

     /* Read as RGB */
     int rgb_pos = 0;

     for(int y = 0; y < height; y++) {
       fflush(stdout);
       png_bytep row = row_pointers[y];
       for(int x = 0; x < width; x++) {
          png_bytep px = &(row[x * 4]);
          ((uint8_t*) img->buf)[rgb_pos++] = (uint8_t) px[0];
          ((uint8_t*) img->buf)[rgb_pos++] = (uint8_t) px[1];
          ((uint8_t*) img->buf)[rgb_pos++] = (uint8_t) px[2];
       }
    }
  }

  fclose(fp);
}
