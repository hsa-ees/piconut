/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
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
 * @fn SC_MODULE(m_color_translator)
 * @author Martin Erichsen
 *
 * The module translates an input color to 24-bit RGB. color_out[23:16],
 * color_out[15:8], color_out[7:0] correspond to the 8-bit wide red, green and
 * blue components respectively. `color_mode` selects the input color's format.
 *
 * Conversion requires one clock cycle.
 *
 * Color map modes are currently not supported and output black (0x000000).
 *
 * @par Ports:
 * @param[in] clk        clock of the module
 * @param[in] reset      reset of the module
 * @param[in] color_mode color mode of the input color
 * @param[in] color_in   input color
 * @param[out] color_out output color as 24-bit RGB
 *
 */

#ifndef __COLOR_TRANSLATOR_H__
#define __COLOR_TRANSLATOR_H__

#include "wb_graphics_config.h"

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_color_translator)
{
public:
    // Ports
    sc_in_clk PN_NAME(clk);     // Clock signal of the module
    sc_in<bool> PN_NAME(reset); // Reset signal of the module

    sc_in<sc_uint<5>> PN_NAME(color_mode);
    sc_in<sc_uint<32>> PN_NAME(color_in);
    sc_out<sc_uint<24>> PN_NAME(color_out);

    // Constructor
    SC_CTOR(m_color_translator)
    {
        SC_METHOD(proc_comb_translate);
        sensitive << color_mode << color_in << color_out;

        SC_CTHREAD(proc_clk_update, clk.pos());
        reset_signal_is(reset, true);
    }

    // Functions
    /**
     * @brief this function is used to generate a tracefile
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace
     */
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_comb_translate();
    void proc_clk_update();

protected:
    // Registers
    sc_signal<sc_uint<24>> PN_NAME(color_out_next);
};
#endif // __COLOR_TRANSLATOR_H__
