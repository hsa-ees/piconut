#pragma once

#include <stdint.h>


typedef struct {
  int8_t conv1_weight[1][5][5][6];
  int8_t conv1_bias[6];
  int8_t conv2_weight[6][5][5][16];
  int8_t conv2_bias[16];
  int8_t fc1_weight[400][120];
  int8_t fc1_bias[120];
  int8_t fc2_weight[120][84];
  int8_t fc2_bias[84];
  int8_t fc3_weight[84][10];
  int8_t fc3_bias[10];
} model_int8f8_t;


extern const model_int8f8_t model_int8f8;
