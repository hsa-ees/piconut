#include "pn_support.h"

#include <stdio.h>
#include <string.h>


#if ARCH_RV32 == 1
#include <pn_riscv_defs.h>
#endif





/************************ Performance Measurement *****************************/


static pn_perf_t pn_perf_last, pn_perf_delta;   /* private */
static const char *pn_perf_msg;                 /* private */


void pn_perf_read (pn_perf_t *perf) {
#if ARCH_RV32 == 1
  uint32_t lo, hi1, hi2;

  // Read instruction counter ...
  do {
    hi1 = read_csr (instreth);
    lo = read_csr (instret);
    hi2 = read_csr (instreth);
  }
  while (hi1 != hi2);
  perf->instret = (((uint64_t) hi1) << 32) | lo;

  // Read cycles counter ...
  do {
    hi1 = read_csr (cycleh);
    lo = read_csr (cycle);
    hi2 = read_csr (cycleh);
  }
  while (hi1 != hi2);
  perf->cycle = (((uint64_t) hi1) << 32) | lo;
#else
  bzero (perf, sizeof (*perf));
#endif // ARCH_RV32 == 1
}


const char *pn_perf_measure (pn_perf_t *last, pn_perf_t *delta) {
  static char ret[80];
  pn_perf_t perf;

  pn_perf_read (&perf);
  delta->instret = perf.instret - last->instret;
  delta->cycle = perf.cycle - last->cycle;
  snprintf (ret, sizeof (ret), "#i=%lu #c=%lu", (unsigned long) delta->instret, (unsigned long) delta->cycle);
  pn_perf_read (last);  // read again to skip the time required for this routine
  return ret;
}


void pn_perf_start () {
  pn_perf_read (&pn_perf_last);
}


void pn_perf_stop () {
  pn_perf_msg = pn_perf_measure (&pn_perf_last, &pn_perf_delta);
}


const char *pn_perf_get (pn_perf_t *delta) {
  if (delta) *delta = pn_perf_delta;
  return pn_perf_msg;
}





/************************ Vector Extension ************************************/


int pn_get_vlen () {
#if ARCH_RV32 == 1
  return read_csr (vlenb) * 8;
#else
  return 0;
#endif
}
