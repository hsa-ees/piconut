/**
 * @file baudgen.h
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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
 * @fn SC_MODULE(m_baudgen)
 * @author Lukas Bauer
 * @brief clockdivider to generate the baudtick for the UART
 *
 * This clock divider can be configured by setting a divider value
 * in the 16-bit `div` register. If the module is enabled, the baud generator
 * counts up until the divider value is reached. Then, the `baudtick` is set
 * to high, and the counter is reset.
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] en_i enable for baudgen module
 * @param[in] clear_i clearing the baudgen (synchronous reset)
 * @param[in] div_i <16> divider value for baudgen
 * @param[out] baudtick_o generated baudtick
 *
 *
 */

#ifndef __BAUDGEN_H__
#define __BAUDGEN_H__

#include <piconut.h>
#include <systemc.h>


SC_MODULE(m_baudgen) {
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk   PN_NAME(clk);           // clock signal of the module
    sc_in<bool> PN_NAME(reset);         // reset for the module

    sc_in<bool> PN_NAME(en_i);          // enable for baudgen module
    sc_in<bool> PN_NAME(clear_i);       // clearing the baudgen (synchronous reset)
    sc_in<sc_uint<16>> PN_NAME(div_i);  // divider value for baudgen

    sc_out<bool> PN_NAME(baudtick_o);   // generated baudtick



    /* Constructor... */
    SC_CTOR(m_baudgen){
        SC_CTHREAD (proc_clk_module, clk.pos ());
        reset_signal_is(reset, true);
    }

    /* Functions...*/
    /**
     * @brief this function is used to generate a tracefile
     * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
     * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void pn_trace (sc_trace_file * tf, int level = 1);

    /** Processes...
    */

    void proc_clk_module();

protected:
    /** Registers...
     * This are examples of Module Registers
     * you may add your own Registers here*/

    sc_signal<sc_uint<16>> PN_NAME(baud_cnt);       // register containing the counter value

};
#endif // __BAUDGEN_H__