#pragma once


typedef struct {
  int16_t conv1_weight[6][1][5][5];
  int16_t conv1_bias[6];
  int16_t conv2_weight[16][6][5][5];
  int16_t conv2_bias[16];
  int16_t fc1_weight[120][400];
  int16_t fc1_bias[120];
  int16_t fc2_weight[84][120];
  int16_t fc2_bias[84];
  int16_t fc3_weight[10][84];
  int16_t fc3_bias[10];
} model_int16f8_t;


extern const model_int16f8_t model_int16f8;
