
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2023    Felix Wagner <felix.wagner@hs-augsburg.de>
                2025    Lukas Bauer  <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains genereal implementations of simple block rams.
    As synthesising these for different verndors may necessitate different
    implementations, these implementations will be held in seperate files and
    included here.

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
#ifndef __DUAL_PORT_RAM_H__
#define __DUAL_PORT_RAM_H__
#include <piconut.h>

// NOTE: The RAM_SIZE is only a place holder so that the template does compile without
//       errors. The RAM_SIZE should be replaced with the actual size of the RAM
//       variable that is configured in the global configuration of the processor
#define RAM_SIZE 123

SC_MODULE (m_dual_port_block_ram) {
public:
    // Ports A
    sc_in_clk           PN_NAME(clka);   // clock input for port A
    sc_in<bool>         PN_NAME(wea);   // write enable for port A
    sc_in<bool>         PN_NAME(ena);   // enable bram for port A

    sc_in<sc_uint<32>>  PN_NAME(addra); // address for bram access for port A
    sc_in<sc_uint<32>>  PN_NAME(dia);   // data in for bram for port A
    sc_out<sc_uint<32>> PN_NAME(doa);   // data out for bram for port A

    // Ports B
    sc_in_clk           PN_NAME(clkb);   // clock input for port B
    sc_in<bool>         PN_NAME(web);   // write enable for port B
    sc_in<bool>         PN_NAME(enb);   // enable bram for port B

    sc_in<sc_uint<32>>  PN_NAME(addrb); // address for bram access for port B
    sc_in<sc_uint<32>>  PN_NAME(dib);   // data in for bram for port B
    sc_out<sc_uint<32>> PN_NAME(dob);   // data out for bram for port B


    // Constructor...
    SC_CTOR(m_dual_port_block_ram){
        SC_CTHREAD (proc_clk_ram_port_a, clka.pos ());
        SC_CTHREAD (proc_clk_ram_port_b, clkb.pos ());
    }

    // Functions...
    void pn_trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_ram_port_a();
    void proc_clk_ram_port_b();

    // This module has an alternative verilog implementation this is used
    // for the ICSC synthesis instead of the SystemC implementation
    // to ensure that the signature of the bram is recognized by the synthesis tool
    // used for the board
    std::string __SC_TOOL_VERILOG_MOD__ =
    R"(
    `include "../verilog/dualPortRam_template.v";
    )";

protected:

    // This is the ram use in the simulation of the bram
    sc_uint<32> ram[RAM_SIZE];
};

#endif // __DUAL_PORT_RAM_H__