#pragma once

#include <stdint.h>


typedef struct {
  float conv1_weight[1][5][5][6];
  float conv1_bias[6];
  float conv2_weight[6][5][5][16];
  float conv2_bias[16];
  float fc1_weight[400][120];
  float fc1_bias[120];
  float fc2_weight[120][84];
  float fc2_bias[84];
  float fc3_weight[84][10];
  float fc3_bias[10];
} model_float_t;


extern const model_float_t model_float;
