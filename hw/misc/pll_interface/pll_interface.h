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


#ifndef __PLL_INTERFACE_H__
#define __PLL_INTERFACE_H__

#include <systemc.h>
#include <piconut.h> // contains all PN_<> Macros and is part of the PicoNut

SC_MODULE(m_pll_interface) {
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk   PN_NAME(clk_in);
    sc_out<bool> PN_NAME(clk_out);

    /* Constructor... */
    SC_CTOR(m_pll_interface){
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


    std::string __SC_TOOL_VERILOG_MOD__ =
    R"(
    module m_pll_interface (
        input clk_in,
        output clk_out);

        wire locked;

        pll pll_inst (
           .clk_in(clk_in),
           .clk_out(clk_out),
           .locked(locked)
        );
    endmodule
    )";

protected:
    /** Registers...
     * This are examples of Module Registers
     * you may add your own Registers here*/



};
#endif // __PLL_INTERFACE_H__