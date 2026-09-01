#include "lenet5.h"

#include "model_int16f8.h"
#include "model_int8f8.h"

#if ARCH_RV32 == 1
#include "test_images.h"
#else
#include "model_float.h"
#include "test_images_10000.h"
#endif

#include "pn_support.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>





/***************************** Options ****************************************/


#define VERBOSE 1

#if ARCH_RV32 == 1
#define IMAGES  1
#else
#define IMAGES  TEST_IMAGES
#endif

#define DUMP_FEATURES 0





/***************************** Helpers ****************************************/


#if ARCH_RV32 != 1


static inline void dtype_from_float (DTYPE *out, const float *in, int count) {
  for (int i = 0; i < count; i++) out[i] = DTYPE_FROM_FLOAT (in[i]);
}


static inline void dtype_to_float (const DTYPE *in, float *out, int count) {
  for (int i = 0; i < count; i++) out[i] = DTYPE_TO_FLOAT (in[i]);
}


static inline void model_from_float (model_t *model, const model_float_t *model_float) {
  assert (ELEMENTS(*model) == ELEMENTS(*model_float));
  dtype_from_float ((DTYPE *) model, (float *) model_float, ELEMENTS(*model));
}


#endif // ARCH_RV32 != 1


static inline void model_from_int16f8 (model_t *model, const model_int16f8_t *model_int16f8) {
  int16_t *src;
  DTYPE *dst;

  src = (int16_t *) model_int16f8;
  dst = (DTYPE *) model;
  for (int i = 0; i < ELEMENTS(*model); i++) {
    //~ printf ("i=%i, dst[i] = %08x\n", i, DTYPE_FROM_INT16F8(*src));
    *(dst++) = DTYPE_FROM_INTXF8(*(src++));
  }
}


static inline void model_from_int8f8 (model_t *model, const model_int8f8_t *model_int8f8) {
  int8_t *src;
  DTYPE *dst;

  src = (int8_t *) model_int8f8;
  dst = (DTYPE *) model;
  for (int i = 0; i < ELEMENTS(*model); i++) {
    //~ printf ("i=%i, dst[i] = %08x\n", i, DTYPE_FROM_INTXF8(*src));
    *(dst++) = DTYPE_FROM_INTXF8(*(src++));
  }
}


#if DUMP_FEATURES
static void dump_features (features_t *ft, const char *filename_fmt, int idx) {
  char filename[256];
  float ft_float[ELEMENTS(features_t)];
  FILE *f;

  snprintf (filename, sizeof (filename), filename_fmt, idx);
  filename[sizeof (filename) - 1] = '\0';
  dtype_to_float ((DTYPE *) ft, (float *) ft_float, ELEMENTS_FLOAT (ft_float));
  printf ("    writing features to '%s' (" DTYPE_STR " converted to float).\n", filename);
  f = fopen (filename, "wb");
  fwrite (&ft_float, 1, sizeof (ft_float), f);
  fclose (f);
}
#endif


static void check_vext () {
#if ARCH_RV32 == 1 && WITH_VEXT
  uint32_t vl;

  printf ("Checking V extension: ");
  printf ("VLEN = %i, ", pn_get_vlen ());
  asm volatile ("vsetvli %0, zero, e32, ta, ma" : "=r" (vl));
  printf ("VLmax (e32) = %i, ", (int) vl);
  asm volatile ("vsetvli %0, zero, e16, ta, ma" : "=r" (vl));
  printf ("VLmax (e16) = %i, ", (int) vl);
  asm volatile ("vsetvli %0, zero, e8, ta, ma" : "=r" (vl));
  printf ("VLmax (e8) = %i\n", (int) vl);
#endif
}


/*
#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })
*/





/***************************** Main *******************************************/


int main () {
  static features_t ft;   // static to save stack space
  static model_t model;   // static to save stack space
  int i, truth, predicted, images, errors;

  // Init performance measurement ...
  printf ("Performance counter validation:");
  for (i = 0; i < 2; i++) {
    pn_perf_start ();
    pn_perf_stop ();
    printf (" (%s)", pn_perf_get (NULL));
  }
  printf ("\n");

  check_vext ();
  //~ printf ("VLEN = %i\n", pn_get_vlen ());

  // Initialization ...
  images = IMAGES;
  errors = 0;
  //~ model_from_int8f8 (&model, &model_int8f8);
  model_from_int16f8 (&model, &model_int16f8);
  //~ model_from_float (&model, &model_float);

  // Inference loop ...
  if (VERBOSE) printf ("Initialization complete. Running inference on %i image(s) ...\n", images);

  #pragma omp parallel for private(ft, truth, predicted) reduction(+:errors)
  for (i = 0; i < images; i++) {
    truth = test_images[i].label;
    predicted = lenet5_predict (&ft, &model, test_images[i].image);
    // Print per-run status ...
    if (predicted == truth) {
      if (VERBOSE && ARCH_RV32) printf ("%4i. %i\n", i, predicted);
      //~ if (VERBOSE && ARCH_RV32) printf ("%4i. %i   (%s)\n", i, predicted, pn_perf_get (NULL));
    }
    else {
      if (VERBOSE) printf ("%4i. %i != %i\n", i, predicted, truth);
      //~ if (VERBOSE) printf ("%4i. %i != %i   (%s)\n", i, predicted, truth, pn_perf_get (NULL));
      errors++;
    }

    // Dump features to files ...
#if DUMP_FEATURES
    dump_features (&ft, "lenet2l/features-%02i.dump", i);
#endif
  }

  // Done ...
  printf ("%i error(s), accuracy = %.2f%%\n", errors, 100.0 - 100 * (float) errors / (float) images);
  return 0;
}
