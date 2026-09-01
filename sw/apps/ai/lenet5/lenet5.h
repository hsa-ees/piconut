#pragma once

#ifndef __ASSEMBLER__   // This file can also be included by assembler sources (*.S).
#include <stdint.h>
#endif // __ASSEMBLER__





/************************ Configuration: Data Type ****************************/


/***** Defined data types *****/


#define DTYPE_FLOAT      0
  // 132 error(s), accuracy = 98.68% [2025-04-07, model_float]
#define DTYPE_INT32F16   1
#define DTYPE_INT32F8    2
  // 144 error(s), accuracy = 98.56%, in 0..63: no errors [2025-12-23]
  // 157 error(s), accuracy = 98.43% [2025-05-25, model_int8f8, after fixing bug in 'run_conv()']
  // 126 error(s), accuracy = 98.74% [2025-04-08, model_int8f8]
  // 129 error(s), accuracy = 98.71% [2025-04-07]
#define DTYPE_INT32F7    3
  // 130 error(s), accuracy = 98.70% [2025-04-07]
#define DTYPE_INT32F6    4
  // 137 error(s), accuracy = 98.63% [2025-04-07]
#define DTYPE_INT32F5    5
  // 140 error(s), accuracy = 98.60% [2025-04-07]
#define DTYPE_INT32F4    6
  // 253 error(s), accuracy = 97.47% [2025-04-07]
#define DTYPE_INT16F8    7
  // 131 error(s), accuracy = 98.69% (*) [2025-04-07]
  // 125 error(s), accuracy = 98.75% (*) [2025-04-08, model_int8f8]
#define DTYPE_INT16F7    8
  // 133 error(s), accuracy = 98.67% (*) [2025-04-07]
#define DTYPE_INT16F6    9
  // 132 error(s), accuracy = 98.68% (*) [2025-04-07]
#define DTYPE_INT16F5   10
  // 140 error(s), accuracy = 98.60% [2025-04-07]
  // 145 error(s), accuracy = 98.55% [2025-04-08, model_int8f8]
#define DTYPE_INT16F4   11
  // 171 error(s), accuracy = 98.29%, in 0..63: 1 error (18. 8 != 3) [2025-12-23]
  // 253 error(s), accuracy = 97.47% [2025-04-07]

// (*) Requires right-shifting after each multiplication to avoid overflows.



/***** Select Data Type *****/


#define WITH_VEXT 1
//~ #define DTYPE_SELECTED DTYPE_INT32F8
#define DTYPE_SELECTED DTYPE_INT16F4



/***** Data Type Properties *****/


#if DTYPE_SELECTED == DTYPE_FLOAT

#define DTYPE float     // physical type
#define DTYPE_STR "float"

#define DTYPE_FROM_FLOAT(a) (a)
#define DTYPE_TO_FLOAT(a) (a)
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a) / 256.0)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a)


#elif DTYPE_SELECTED == DTYPE_INT32F16
  // [2025-04-04] This has 0 of 64 test images mispredicted.
  //    It requires right-shifting after each multiplication. Otherwise, overflows occur.

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f16"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 16)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 16))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) (a)) << 8)

#define DTYPE_MUL(a, b) ((int32_t) (((int64_t) (a) * (int64_t) (b) + (1 << 15)) >> 16))
#define DTYPE_POSTMUL(a)


#elif DTYPE_SELECTED == DTYPE_INT32F8
  // [2025-04-04] This has 0 of 64 test images mispredicted.

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f8"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 8)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 8))
#define DTYPE_FROM_INTXF8(a) ((DTYPE) (a))

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 7)) >> 8
//~ #define DTYPE_MUL(a, b) (((a) * (b) + (1 << 7)) >> 8)
//~ #define DTYPE_POSTMUL(a)


#elif DTYPE_SELECTED == DTYPE_INT32F7

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f7"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 7)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 7))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a) >> 1)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 6)) >> 7


#elif DTYPE_SELECTED == DTYPE_INT32F6

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f6"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 6)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 6))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 1)) >> 2)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 5)) >> 6


#elif DTYPE_SELECTED == DTYPE_INT32F5
  // [2025-04-04] This has 0 of 64 test images mispredicted.

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f5"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 5)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 5))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 2)) >> 3)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 4)) >> 5


#elif DTYPE_SELECTED == DTYPE_INT32F4
  // [2025-04-04] This has 1 of 64 test images mispredicted (#7: 4 != 9).

#define DTYPE int32_t     // physical type
#define DTYPE_STR "int32f4"

#define DTYPE_FROM_FLOAT(a) ((int32_t) ((a) * (float) (1 << 4)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 4))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 3)) >> 4)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 3)) >> 4


#elif DTYPE_SELECTED == DTYPE_INT16F8
  // [2025-04-04] This has 0 of 64 test images mispredicted.
  //    It requires right-shifting after each multiplication. Otherwise, overflows occur.

#define DTYPE int16_t     // physical type
#define DTYPE_STR "int16f8"

#define DTYPE_FROM_FLOAT(a) ((int16_t) ((a) * (float) (1 << 8)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 8))
#define DTYPE_FROM_INTXF8(a) ((DTYPE) a)

//~ #define DTYPE_MUL(a, b) ((a) * (b))
//~ #define DTYPE_POSTMUL(a) a = ((a) + (1 << 7)) >> 8
#define DTYPE_MUL(a, b) (((a) * (b) + (1 << 7)) >> 8)
#define DTYPE_POSTMUL(a)


#elif DTYPE_SELECTED == DTYPE_INT16F7
  // [2025-04-07] This has 0 of 64 test images mispredicted.
  //    It requires right-shifting after each multiplication. Otherwise, overflows occur.

#define DTYPE int16_t     // physical type
#define DTYPE_STR "int16f7"

#define DTYPE_FROM_FLOAT(a) ((int16_t) ((a) * (float) (1 << 7)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 7))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 0)) >> 1)

//~ #define DTYPE_MUL(a, b) ((a) * (b))
//~ #define DTYPE_POSTMUL(a) a = ((a) + (1 << 6)) >> 7
#define DTYPE_MUL(a, b) (((a) * (b) + (1 << 6)) >> 7)
#define DTYPE_POSTMUL(a)



#elif DTYPE_SELECTED == DTYPE_INT16F6
  // [2025-04-07] This has 0 of 64 test images mispredicted.
  //    It requires right-shifting after each multiplication. Otherwise, overflows occur.

#define DTYPE int16_t     // physical type
#define DTYPE_STR "int16f6"

#define DTYPE_FROM_FLOAT(a) ((int16_t) ((a) * (float) (1 << 6)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 6))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 1)) >> 2)

//~ #define DTYPE_MUL(a, b) ((a) * (b))
//~ #define DTYPE_POSTMUL(a) a = ((a) + (1 << 5)) >> 6
#define DTYPE_MUL(a, b) (((a) * (b) + (1 << 5)) >> 6)
#define DTYPE_POSTMUL(a)



#elif DTYPE_SELECTED == DTYPE_INT16F5
  // [2025-04-07] This has 0 of 64 test images mispredicted.

#define DTYPE int16_t     // physical type
#define DTYPE_STR "int16f5"

#define DTYPE_FROM_FLOAT(a) ((int16_t) ((a) * (float) (1 << 5)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 5))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 2)) >> 3)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 4)) >> 5
//~ #define DTYPE_MUL(a, b) (((a) * (b) + (1 << 4)) >> 5)
//~ #define DTYPE_POSTMUL(a)



#elif DTYPE_SELECTED == DTYPE_INT16F4
  // [2025-04-04] This has 1 of 64 test images mispredicted (#7: 4 != 9).

#define DTYPE int16_t     // physical type
#define DTYPE_STR "int16f4"

#define DTYPE_FROM_FLOAT(a) ((int16_t) ((a) * (float) (1 << 4)))
#define DTYPE_TO_FLOAT(a) ((float) (a) / (float) (1 << 4))
#define DTYPE_FROM_INTXF8(a) (((DTYPE) a + (1 << 3)) >> 4)

#define DTYPE_MUL(a, b) ((a) * (b))
#define DTYPE_POSTMUL(a) a = ((a) + (1 << 3)) >> 4
//~ #define DTYPE_MUL(a, b) (((a) * (b) + (1 << 3)) >> 4)
//~ #define DTYPE_POSTMUL(a)



#else
#error "No valid DTYPE selected"
#endif





/************************ LeNet5 Parameters ***********************************/


// Kernel dimension for both convolutional layers ...
#define KERNEL_DIM     5

// Input image: channels and dimension ...
#define INPUT_CHS      1
#define INPUT_DIM     32

// Conv1 layer: channels, dimension and dimension after pooling ...
#define CONV1_CHS      6
#define CONV1_DIM (INPUT_DIM - KERNEL_DIM + 1)      // 28
#define CONV1_POOL_DIM (CONV1_DIM >> 1)             // 14

// Conv2 layer: channels, dimension and dimension after pooling ...
#define CONV2_CHS     16
#define CONV2_DIM (CONV1_POOL_DIM - KERNEL_DIM + 1) // 10
#define CONV2_POOL_DIM (CONV2_DIM >> 1)             //  5

// Fully-connected layers: number of outputs ...
#define FC1_OUTS     120    // number of inputs is 400 (= CONV2_CHS * CONV2_POOL_DIM^2)
#define FC2_OUTS      84    // number of inputs is 120 (= FC1_OUTS)
#define FC3_OUTS      10    // number of inputs is 84 (= FC2_OUTS)





/************************ Model and Features **********************************/


#ifndef __ASSEMBLER__   // This file can also be included by assembler sources (*.S).


typedef struct model_s {
  DTYPE conv1_weight[INPUT_CHS][KERNEL_DIM][KERNEL_DIM][CONV1_CHS];
  DTYPE conv1_bias[CONV1_CHS];
  DTYPE conv2_weight[CONV1_CHS][KERNEL_DIM][KERNEL_DIM][CONV2_CHS];
  DTYPE conv2_bias[CONV2_CHS];
  DTYPE fc1_weight[CONV2_CHS*CONV2_POOL_DIM*CONV2_POOL_DIM][FC1_OUTS];
  DTYPE fc1_bias[FC1_OUTS];
  DTYPE fc2_weight[FC1_OUTS][FC2_OUTS];
  DTYPE fc2_bias[FC2_OUTS];
  DTYPE fc3_weight[FC2_OUTS][FC3_OUTS];
  DTYPE fc3_bias[FC3_OUTS];
} model_t;


typedef struct features_s {
  DTYPE input[INPUT_CHS][INPUT_DIM][INPUT_DIM];                 //   1 x 32 x 32
  DTYPE conv1_out[CONV1_CHS][CONV1_DIM][CONV1_DIM];             //   6 x 28 x 28  -- conv1 (before + after ReLU)
  DTYPE conv1_pool[CONV1_CHS][CONV1_POOL_DIM][CONV1_POOL_DIM];  //   6 x 14 x 14  -- conv1 after pooling
  DTYPE conv2_out[CONV2_CHS][CONV2_DIM][CONV2_DIM];             //  16 x 10 x 10  -- conv2 (before + after ReLU)
  DTYPE conv2_pool[CONV2_CHS][CONV2_POOL_DIM][CONV2_POOL_DIM];  //  16 x  5 x  5  -- conv2 after pooling
  DTYPE fc1_out[FC1_OUTS];                                      // 120 x  1 x  1  -- fc1 (before + after ReLU)
  DTYPE fc2_out[FC2_OUTS];                                      //  84 x  1 x  1  -- fc2 (before + after ReLU)
  DTYPE fc3_out[FC3_OUTS];                                      //  10            -- fc3 (before + after ReLU)
} features_t;





/************************ Helpers *********************************************/


#define ELEMENTS(array) (sizeof(array) / sizeof(DTYPE))
#define ELEMENTS_FLOAT(array) (sizeof(array) / sizeof(float))





/************************ Functions *******************************************/


void lenet5_forward (features_t *ft, model_t *model);

void lenet5_load_input (features_t *ft, uint8_t *input);
int  lenet5_get_result (features_t *ft);

int lenet5_predict (features_t *ft, model_t *model, uint8_t input[32*32]);


#endif // __ASSEMBLER__
