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
 * @fn SC_MODULE(m_trianglewave)
 * @author Tristan Kundrat
 * @brief Audio module generating a trianglewave signal
 *
 *
 * This module generates a trianglewave signal with the frequency f and amplitude A.
 * The input parameters are calculated like so (f_clk is the systems clock frequency):
 * <br> frequency_divisor = f_clk / f
 * <br> step_height = A * 2 / frequency_divisor
 *
 *
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] enable enable of the module
 * @param[in] frequency_divisor f_clk/f
 * @param[in] step_height amplitude * 2 / frequency_divisor
 * @param[out] audio_signal 16 bit unsigned audio output
 *
 *
 */

#ifndef __TRIANGLEWAVE_H
#define __TRIANGLEWAVE_H

#include <piconut.h>

#include <sysc/kernel/sc_module.h>

#include "counter.h"

SC_MODULE (m_trianglewave) {
public:
  sc_in_clk PN_NAME(clk);
  sc_in<bool> PN_NAME(reset);
  sc_in<bool> PN_NAME(enable);
  sc_in<sc_uint<32> > PN_NAME(frequency_divisor);
  sc_in<sc_uint<16> > PN_NAME(step_height);
  sc_out<sc_uint<16> > PN_NAME(audio_signal);


  SC_CTOR(m_trianglewave) {

    SC_METHOD(proc_comb);
    sensitive << amplitude << count << frequency_divisor;

    init_submodules();
  }

  // Processes
  void proc_comb();

  // Submodules
  m_counter *counter;

  // Methods
  void init_submodules();

protected:
  // Registers

  // Signals
  sc_signal<sc_uint<32> > PN_NAME(count);
  sc_signal<sc_uint<24> > PN_NAME(amplitude);
  sc_signal<bool> PN_NAME(negative_step);
};

#endif // __TRIANGLEWAVE_H
