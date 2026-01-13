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
 * @fn SC_MODULE(m_vga_color_generator)
 * @authors Beaurel I. Ngaleu
 * @author Martin Erichsen
 * @brief Generator for VGA compatible 4-bit wide color
 *
 * Generates 4-bit wide red, green and blue signals from a 24-bit RGB signal
 * for with 4-bit video DACs. `blank_enable` forces the output to black (all
 * three channels to 0).
 *
 * Requires one clock cycle to output.
 *
 * @par Ports:
 * @param[in] clk          clock of the module
 * @param[in] reset        reset of the module
 * @param[in] blank_enable enable output blanking
 * @param[in] video_in     input color
 * @param[out] vga_red     4-bit red channel intensity
 * @param[out] vga_green   4-bit green channel intensity
 * @param[out] vga_blue    4-bit blue channel intensity
 */

#ifndef __VGA_COLOR_GENERATOR_H__
#define __VGA_COLOR_GENERATOR_H__

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_vga_color_generator)
{
public:
    // Ports
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Blanking Control Signal
    sc_in<bool> PN_NAME(blank_enable);

    // Input Color Signal
    sc_in<sc_uint<24>> PN_NAME(video_in);

    // Output Color Signals
    sc_out<sc_uint<4>> PN_NAME(vga_red);
    sc_out<sc_uint<4>> PN_NAME(vga_green);
    sc_out<sc_uint<4>> PN_NAME(vga_blue);

    // Constructor
    SC_CTOR(m_vga_color_generator)
    {
        SC_METHOD(proc_comb_convert);
        sensitive << video_in << blank_enable;

        SC_CTHREAD(proc_clk_update, clk.pos())
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
    void proc_comb_convert();
    void proc_clk_update();

protected:
    // Internal Signals
    sc_signal<sc_uint<4>> PN_NAME(vga_red_next);
    sc_signal<sc_uint<4>> PN_NAME(vga_green_next);
    sc_signal<sc_uint<4>> PN_NAME(vga_blue_next);
};

#endif // __VGA_COLOR_GENERATOR_H__
