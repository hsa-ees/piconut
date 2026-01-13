/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Tristan Kundrat <tristan.kundrat@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

/**
 * @fn SC_MODULE(m_counter)
 * @author Tristan Kundrat
 * @brief Configurable counter module
 *
 *
 * While enabled, this module counts from 0 to frequency_divisor, resets to 0
 * after and continues counting. This count can be accessed by the parameter count.
 * When disabled, the count is set to 0.
 * Every clock cycle, count is incremented by 1 (if enabled).
 * The amplitude is reset to 0, when count is reset to 0.
 * If enabled, every clock cycle the amplitude is recalculated as follows:
 * <br> if negative_step is 0:
 * <br> amplitude = amplitude + step_height
 * <br> if negative_step is 1:
 * <br> amplitude = amplitude - step_height
 * 
 *
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] enable enable of the module
 * @param[in] frequency_divisor f_clk / f
 * @param[in] step_height increment value per clock cycle
 * @param[in] negative_step decrement amplitude instead of increment
 * @param[out] amplitude current amplitude of output signal
 * @param[out] count current count of underlying counter
 *
 *
 */

#include <systemc.h>
#include <piconut.h>

#ifndef __COUNTER_H
#define __COUNTER_H

SC_MODULE (m_counter) {
  sc_in_clk PN_NAME(clk);
  sc_in<bool> PN_NAME(reset);
  sc_in<bool> PN_NAME(enable);
  sc_in<sc_uint<32> > PN_NAME(frequency_divisor);
  sc_in<sc_uint<16> > PN_NAME(step_height);
  sc_in<bool> PN_NAME(negative_step);
  sc_out<sc_uint<24> > PN_NAME(amplitude);
  sc_out<sc_uint<32> > PN_NAME(count);

  SC_CTOR(m_counter) {
    SC_CTHREAD(proc_clk, clk.pos());
    reset_signal_is(reset, true);
  }

  void proc_clk();
};

#endif // __COUNTER_H
