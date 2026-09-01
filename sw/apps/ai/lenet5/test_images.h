#pragma once

#include <stdint.h>


#define TEST_IMAGES 64


typedef struct {
  int label;             /* ground truth */
  uint8_t image[32*32];  /* pixel data */
} image_t;


extern image_t test_images[TEST_IMAGES];
