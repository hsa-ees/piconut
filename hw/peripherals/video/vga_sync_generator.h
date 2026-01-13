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
 * @fn SC_MODULE(m_vga_sync_generator)
 * @author Martin Erichsen
 * @brief Generator for VGA compatible hsync and vsync signals
 *
 * Outputs horizontal and vertical sync signals. Both hsync and vsync are
 * inverted, so they are at logic 0 when asserted during synchronization and
 * logic 1 otherwise.
 *
 * Either sync signal is asserted or deasserted the next clock cycle their
 * address value matches `vga_*_begin` or `vga_*_end`. They do not change
 * otherwise.
 *
 * While `enable` is not set, both sync signals are forced to deasserted state
 * on the next clock cycle.
 *
 * @par Ports:
 * @param[in] clk             clock of the module
 * @param[in] reset           reset of the module
 * @param[in] vga_hsync_begin horizontal address to assert hsync after
 * @param[in] vga_hsync_end   horizontal address to deassert hsync after
 * @param[in] vga_vsync_begin vertical address to assert vsync after
 * @param[in] vga_vsync_end   vertical address to deassert vsync after
 * @param[in] enable          synchronous enable
 * @param[in] column          current column coutner address
 * @param[in] line            current line counter address
 * @param[out] vga_hsync      horizontal synchronization signal
 * @param[out] vga_vsync      vertical synchronization signal
 */

#ifndef __VGA_SYNC_GENERATOR_H__
#define __VGA_SYNC_GENERATOR_H__

#include <piconut.h>

#include "wb_graphics_config.h"


SC_MODULE(m_vga_sync_generator)
{
public:
    // Ports
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);
    sc_in<bool> PN_NAME(enable);

    // Horizontal and vertical synchronization pulse timings
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_begin);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_end);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_begin);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_end);

    // Counters
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line);

    // Sync Signals
    sc_out<bool> PN_NAME(vga_hsync);
    sc_out<bool> PN_NAME(vga_vsync);

    // Constructor
    SC_CTOR(m_vga_sync_generator)
    {
        SC_METHOD(proc_comb_sync);
        sensitive << column << line << enable
                  << vga_hsync << vga_vsync
                  << vga_hsync_begin << vga_hsync_end
                  << vga_vsync_begin << vga_vsync_end;

        SC_CTHREAD(proc_clk, clk.pos());
        reset_signal_is(reset, true);
    }

    // Functions
    /**
     * @brief this function is used to generate a tracefile
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_comb_sync();
    void proc_clk();

protected:
    // Next Sync Signals
    sc_signal<bool> PN_NAME(vga_hsync_next);
    sc_signal<bool> PN_NAME(vga_vsync_next);
};

#endif // __VGA_SYNC_GENERATOR_H__
