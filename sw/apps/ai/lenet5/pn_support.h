#pragma once


#include <stdint.h>





/************************ Performance Measurement *****************************/


typedef struct {
  uint64_t instret;
  uint64_t cycle;
} pn_perf_t;


void pn_perf_read (pn_perf_t *perf);
  /**< @brief Read performance-related counters.
   */

const char *pn_perf_measure (pn_perf_t *last, pn_perf_t *delta);
  /**< @brief Read the performance-related counters and update `last`.
   *
   * The returned string is valid until the next call of  @ref pn_perf_measure()
   * or @ref pn_perf_stop().
   *
   * @return Elapsed time as `*delta` and a readable string for printing.
   */

void pn_perf_start ();
  /**< @brief Start performance measurement and store the current counters in a global variable.
   */

void pn_perf_stop ();
  /**< @brief Stop performance measurement and store the results in a global variable.
   *
  * If @ref pn_perf_start() has not been called before, the behavior is unspecified.
   */

const char *pn_perf_get (pn_perf_t *delta);
  /**< @brief Retrieve the results of a previous measurement by @ref pn_perf_start() and @ref pn_perf_stop().
   *
   * The returned values are valid until the next call of  @ref pn_perf_start(), @ref pn_perf_stop()
   * or @ref pn_perf_measure().
   *
   * @return Elapsed time as `*delta` and a readable string for printing.
   */





/************************ Vector Extension ************************************/


int pn_get_vlen ();
  /**< @brief Get the VLEN value of the vector extension.
   *
   * Note [2025-04-30]: Presently, spike/pk raise a illegal instruction exception
   *   when executing this, see also: https://github.com/riscv-software-src/riscv-isa-sim/issues/1016
   */
