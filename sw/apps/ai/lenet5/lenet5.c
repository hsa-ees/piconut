#include "lenet5.h"
#include "model_float.h"

#include <stdio.h>
#include <memory.h>

#include "pn_support.h"





/***************************** Assembler Imports ******************************/


#if ARCH_RV32 == 1

/* Functions from 'lenet5_asm.S' ... */

#if DTYPE_SELECTED == DTYPE_INT32F8

void run_relu_asm_32f8 (DTYPE *data, int count);
void run_relu_vec_32f8 (DTYPE *data, int count);

void run_maxpool_vec_32f8 (DTYPE *out, DTYPE *in, int channels, int dim);

void run_conv1_vec_32f8 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
void run_conv2_vec_32f8 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);

void run_fc_vec_32f8 (DTYPE *out, int out_dim, DTYPE *in, int in_dim, DTYPE *weight, DTYPE *bias);

#endif
#if DTYPE_SELECTED == DTYPE_INT16F4

void run_relu_vec_16f4 (DTYPE *data, int count);

void run_maxpool_vec_16f4 (DTYPE *out, DTYPE *in, int channels, int dim);

void run_conv1_vec_16f4 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
void run_conv2_vec_16f4 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);

void run_fc_vec_16f4 (DTYPE *out, int out_dim, DTYPE *in, int in_dim, DTYPE *weight, DTYPE *bias);

#endif

#endif


#if DTYPE_SELECTED == DTYPE_INT32F8

#define run_relu_vec        run_relu_vec_32f8

#define run_maxpool_vec     run_maxpool_vec_32f8

#define run_conv1_vec       run_conv1_vec_32f8
#define run_conv2_vec       run_conv2_vec_32f8

#define run_fc_vec          run_fc_vec_32f8

#endif


#if DTYPE_SELECTED == DTYPE_INT16F4

#define run_relu_vec        run_relu_vec_16f4

#define run_maxpool_vec     run_maxpool_vec_16f4

#define run_conv1_vec       run_conv1_vec_16f4
#define run_conv2_vec       run_conv2_vec_16f4

#define run_fc_vec          run_fc_vec_16f4

#endif


#if !WITH_VEXT

#undef run_relu_vec
#define run_relu_vec        run_relu

#undef run_maxpool_vec
#define run_maxpool_vec     run_maxpool

#undef run_conv1_vec
#define run_conv1_vec       run_conv1

#undef run_conv2_vec
#define run_conv2_vec       run_conv2

#undef run_fc_vec
#define run_fc_vec          run_fc

#endif





/***************************** Layers: ReLU ***********************************/


/* static */ void run_relu (DTYPE *data, int count) {
  int i;

  for (i = 0; i < count; i++)
    if (data[i] < 0) data[i] = DTYPE_FROM_FLOAT (0.0);
}





/***************************** Layers: Max Pooling ****************************/


/* static */ void run_maxpool (DTYPE *out, DTYPE *in, int channels, int dim) {
  // We expect a memory layout of dim * dim values and channels stored separately ([channels][dim][dim]).
  // 'dim' is the output dimension, the input arrays are expected to have dimensions of '2*dim'.
  //    DTYPE in[channels][2*dim][2*dim];
  //    DTYPE out[channels][dim][dim];
  int x, y;
  DTYPE val00, val01, val10, val11, *src, *dst;

  for (y = 0; y < channels * dim; y++)
    for (x = 0; x < dim; x++) {
      src = in + 4 * dim * y + 2 * x;
      dst = out + dim * y + x;
      val00 = src[0];
      val01 = src[1];
      val10 = src[2 * dim];
      val11 = src[2 * dim + 1];
      if (val01 > val00) val00 = val01;
      if (val11 > val10) val10 = val11;
      if (val10 > val00) val00 = val10;
      dst[0] = val00;
    }
}





/***************************** Layers: Convolution *****************************/


static void run_conv (DTYPE *in, int in_chs, int in_dim,
                      DTYPE *out, int out_chs, int out_dim,
                      int kernel_dim,
                      DTYPE *weight, DTYPE *bias) {
  // We expect 'in', 'out', 'weight' and 'bias' in the following formats:
  //   DTYPE in[in_chs][in_dim][in_dim];
  //   DTYPE out[out_chs][out_dim][out_dim];
  //   DTYPE weight[in_chs][kernel_dim][kernel_dim][out_chs];
  //   DTYPE bias[out_chs];
  // where: out_dim = in_dim - kernel_dim + 1
  DTYPE *ch_in, *ch_out, *ch_weight;
  int i, o, x, y, wx, wy;

  // Preset output ...
  for (i = 0; i < out_chs * out_dim * out_dim; i++)
    out[i] = DTYPE_FROM_FLOAT (0.0);

  // Accumulate ...
  for (i = 0; i < in_chs; i++)
    for (o = 0; o < out_chs; o++) {
      ch_in = in + i * in_dim * in_dim;
      ch_out = out + o * out_dim * out_dim;
      ch_weight = weight + i * kernel_dim * kernel_dim * out_chs + o;

      // Run a convolution for a single channel and output ...
      for (y = 0; y < out_dim; y++)
        for (x = 0; x < out_dim; x++)
          for (wy = 0; wy < kernel_dim; wy++)
            for (wx = 0; wx < kernel_dim; wx++) {
              ch_out[y * out_dim + x] += DTYPE_MUL (
                  ch_in[(y + wy) * in_dim + (x + wx)],
                  ch_weight[(wy * kernel_dim + wx) * out_chs]
                );
              //~ printf ("i=%i, o=%i, y=%i, x=%i, wy=%i, wx=%i, ch_out[x,y]=%08x\n", i, o, y, x, wy, wx, ch_out[y * out_dim + x]);
              //~ printf ("  ch_in[...] = %08x, ch_weight[...] = %08x\n", ch_in[(y + wy) * in_dim + (x + wx)], ch_weight[wy * kernel_dim + wx]);
            }
    }

  // Post-process outputs ...
  for (o = 0; o < out_chs; o++) {
    ch_out = out + o * out_dim * out_dim;
    for (x = 0; x < out_dim * out_dim; x++) {
      DTYPE_POSTMUL (ch_out[x]);
      ch_out[x] += bias[o];
      //~ printf ("o=%i, x=%i, ch_out[x]=%08x\n", o, x, ch_out[x]);
    }
  }
}


/* static */ void run_conv1 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias) {
  run_conv (in, INPUT_CHS, INPUT_DIM,
            out, CONV1_CHS, CONV1_DIM,
            KERNEL_DIM,
            weight, bias);
}


/* static */ void run_conv2 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias) {
  run_conv (in, CONV1_CHS, CONV1_POOL_DIM,
            out, CONV2_CHS, CONV2_DIM,
            KERNEL_DIM,
            weight, bias);
}





/***************************** Layers: Fully Connected ************************/


/* static */ void run_fc (DTYPE *in, int in_dim, DTYPE *out, int out_dim, DTYPE *weight, DTYPE *bias) {
  // We expect weights to be stored in lines of 'in_dim' values:
  //   DTYPE weight[in_dim][out_dim];
  int i, o;

  // Preset output ...
  for (o = 0; o < out_dim; o++) out[o] = DTYPE_MUL (bias[o], DTYPE_FROM_FLOAT (1.0));

  // Accumulate ...
  for (i = 0; i < in_dim; i++)
    for (o = 0; o < out_dim; o++)
      out[o] += DTYPE_MUL (in[i], weight[i * out_dim + o]);

  // Post-processing ...
  for (o = 0; o < out_dim; o++)
    DTYPE_POSTMUL (out[o]);
}





/***************************** Inference Main *********************************/


/***** Macros to run with validation or measurement *****/

/* Arguments to any of the run macros are:
 *  INF     : Informative string naming the processing step (for log outputs)
 *  OPT_FN  : Optimized function (used for RISC-V only)
 *  REF_FN  : Reference function
 *  FT_OUT  : Ouput array, must be a field name defines in 'features_t'
 *  ARGS    : Arguments for OPT_FN and REF_FN
 */


#if ARCH_RV32 == 1

// Run with validation against a reference function ...
#define VALIDATE(INF, OPT_FN, REF_FN, FT_OUT, ARGS) {                     \
    printf ("-- " INF ": validating ...\n");                              \
    DTYPE FT_OUT##_org [ELEMENTS (ft->FT_OUT)];                           \
    DTYPE FT_OUT##_ref [ELEMENTS (ft->FT_OUT)];                           \
    memcpy (FT_OUT##_org, (DTYPE *) ft->FT_OUT, sizeof (FT_OUT##_org));   \
    REF_FN ARGS;                                                          \
    memcpy (FT_OUT##_ref, (DTYPE *) ft->FT_OUT, sizeof (FT_OUT##_ref));   \
    memcpy ((DTYPE *) ft->FT_OUT, FT_OUT##_org, sizeof (FT_OUT##_org));   \
    OPT_FN ARGS;                                                          \
    for (int i = 0; i < ELEMENTS (ft->FT_OUT); i++)                       \
      if (((DTYPE *) ft->FT_OUT) [i] != FT_OUT##_ref[i])                  \
        printf ("### ERROR: " INF " %i: 0x%08x / 0x%08x\n", i,            \
                (unsigned) ((DTYPE *) ft->FT_OUT) [i],                    \
                (unsigned) FT_OUT##_ref[i]);                              \
    printf ("-- " INF ": ... done.\n");                                   \
  }


// Run with performance measurment ...
#define MEASURE(INF, OPT_FN, REF_FN, FT_OUT, ARGS)                        \
  pn_perf_start ();                                                       \
  OPT_FN ARGS;                                                            \
  pn_perf_stop ();                                                        \
  printf ("-- " INF ": %s\n", pn_perf_get (NULL));

// Just run ...
#define JUSTRUN(INF, OPT_FN, REF_FN, FT_OUT, ARGS) OPT_FN ARGS


#else   // ARCH_RV32 == 1

#define VALIDATE(INF, OPT_FN, REF_FN, FT_OUT, ARGS) REF_FN ARGS
#define MEASURE(INF, OPT_FN, REF_FN, FT_OUT, ARGS) REF_FN ARGS
#define JUSTRUN(INF, OPT_FN, REF_FN, FT_OUT, ARGS) REF_FN ARGS

#endif  // ARCH_RV32 == 1



/***** Default Run Macro *****/

//~ #define RUN JUSTRUN
  // option: just run (default)
#define RUN MEASURE
  // option: measure everything
//~ #define RUN VALIDATE
  // option: Validate everything



/***** lenet5_forward() *****/


void lenet5_forward (features_t *ft, model_t *model) {

  // Convolution Layer 1 + ReLu + Pooling ...
  //~ for (int i = 0; i < ELEMENTS(model->conv1_bias); i++) ((DTYPE *) model->conv1_bias)[i] = DTYPE_FROM_FLOAT(0.0);
  //~ for (int i = 0; i < ELEMENTS(model->conv1_weight); i++) ((DTYPE *) model->conv1_weight)[i] = DTYPE_FROM_FLOAT(0.0);
  //~ ((DTYPE *) model->conv1_weight)[(0*5+2)*6] = DTYPE_FROM_FLOAT(1.0);
  //~ for (int i = 0; i < ELEMENTS(ft->input); i++) ((DTYPE *) ft->input)[i] = DTYPE_FROM_FLOAT(0.0);
  RUN ("Conv-1   ", run_conv1_vec, run_conv1, conv1_out,
        ((DTYPE *) ft->input, (DTYPE *) ft->conv1_out,
         (DTYPE *) model->conv1_weight, (DTYPE *) model->conv1_bias)
    );
  RUN ("ReLU-C1  ", run_relu_vec, run_relu, conv1_out,
        ((DTYPE *) ft->conv1_out, ELEMENTS (ft->conv1_out))
    );
  RUN ("MaxPool-1", run_maxpool_vec, run_maxpool, conv1_pool,
        ((DTYPE *) ft->conv1_pool, (DTYPE *) ft->conv1_out, CONV1_CHS, CONV1_POOL_DIM)
    );

  // Convolution Layer 2 + ReLu + Pooling ...
  //~ for (int i = 0; i < ELEMENTS(model->conv2_bias); i++) ((DTYPE *) model->conv2_bias)[i] = DTYPE_FROM_FLOAT(0.0);
  //~ for (int i = 0; i < ELEMENTS(model->conv2_weight); i++) ((DTYPE *) model->conv2_weight)[i] = DTYPE_FROM_FLOAT(0.0);
  //~ ((DTYPE *) model->conv2_weight)[(0*5+0)*16+0] = DTYPE_FROM_FLOAT(1.0);
  //~ for (int i = 0; i < ELEMENTS(ft->conv1_pool); i++) ((DTYPE *) ft->conv1_pool)[i] = DTYPE_FROM_FLOAT(0.0);
  RUN ("Conv-2   ", run_conv2_vec, run_conv2, conv2_out,
        ((DTYPE *) ft->conv1_pool, (DTYPE *) ft->conv2_out,
         (DTYPE *) model->conv2_weight, (DTYPE *) model->conv2_bias)
    );
  RUN ("ReLU-C2  ", run_relu_vec, run_relu, conv2_out,
        ((DTYPE *) ft->conv2_out, ELEMENTS (ft->conv2_out))
    );
  RUN ("MaxPool-2", run_maxpool_vec, run_maxpool, conv2_pool,
        ((DTYPE *) ft->conv2_pool, (DTYPE *) ft->conv2_out, CONV2_CHS, CONV2_POOL_DIM)
    );

  // Fully-Connected Layer 1 + ReLu ...
  RUN ("Full-1   ", run_fc_vec, run_fc, fc1_out,
        ((DTYPE *) ft->conv2_pool, CONV2_CHS * CONV2_POOL_DIM * CONV2_POOL_DIM,
         (DTYPE *) ft->fc1_out, FC1_OUTS,
         (DTYPE *) model->fc1_weight, (DTYPE *) model->fc1_bias)
    );
  RUN ("ReLU-F1  ", run_relu_vec, run_relu, fc1_out,
        ((DTYPE *) ft->fc1_out, ELEMENTS (ft->fc1_out))
    );

  // Fully-Connected Layer 2 + ReLu ...
  RUN ("Full-2   ", run_fc_vec, run_fc, fc2_out,
        ((DTYPE *) ft->fc1_out, FC1_OUTS,
         (DTYPE *) ft->fc2_out, FC2_OUTS,
         (DTYPE *) model->fc2_weight, (DTYPE *) model->fc2_bias)
    );
  RUN ("ReLU-F2  ", run_relu_vec, run_relu, fc2_out,
        ((DTYPE *) ft->fc2_out, ELEMENTS (ft->fc2_out))
    );

  // Fully-Connected Layer 3 ...
  RUN ("Full-3   ", run_fc_vec, run_fc, fc3_out,
      ((DTYPE *) ft->fc2_out, FC2_OUTS,
       (DTYPE *) ft->fc3_out, FC3_OUTS,
       (DTYPE *) model->fc3_weight, (DTYPE *) model->fc3_bias)
    );
}





/***************************** Top-Level Functions ****************************/


void lenet5_load_input (features_t *ft, uint8_t *input) {
  DTYPE (*layer0)[INPUT_DIM][INPUT_DIM] = ft->input;
  int x, y;

  for (y = 0; y < 32; y++)
    for (x = 0; x < 32; x++) {
      layer0[0][y][x] = DTYPE_FROM_INTXF8 ((((int) input[INPUT_DIM * y + x]) << 1) - 256);
      //~ layer0[0][y][x] = DTYPE_FROM_FLOAT (((float) input[INPUT_DIM * y + x]) * (2.0/255) - 1.0);
      //~ printf ("### input[%i][%i] = %08x\n", y, x, (uint32_t) (layer0[0][y][x]));
    }
}


int lenet5_get_result (features_t *ft) {
  int i, iMax;

  iMax = 0;
  for (i = 1; i < FC3_OUTS; i++)
    if (ft->fc3_out[i] > ft->fc3_out[iMax]) iMax = i;
  return iMax;
}


int lenet5_predict (features_t *ft, model_t *model, uint8_t input[32*32]) {
  //~ bzero (ft, sizeof (*ft));
  memset (ft, 0xee, sizeof (*ft));    // preset everything with 0xee to make errors visible
  lenet5_load_input (ft, input);
  //~ pn_perf_start ();
  lenet5_forward (ft, model);
  //~ pn_perf_stop ();
  return lenet5_get_result (ft);
}
