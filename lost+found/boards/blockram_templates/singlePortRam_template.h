
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
#ifndef __SINGLE_PORT_RAM_H__
#define __SINGLE_PORT_RAM_H__
#include <piconut.h>

// NOTE: The RAM_SIZE is only a place holder so that the template does compile without
//       errors. The RAM_SIZE should be replaced with the actual size of the RAM
//       variable that is configured in the global configuration of the processor
#define RAM_SIZE 123



// **************** Single Port Block Ram ******************
// This class implements a read first single port block ram
SC_MODULE (m_single_port_block_ram) {
public:
    // Ports ...
    sc_in_clk           PN_NAME(clk);   // clock input
    sc_in<bool>         PN_NAME(wea);   // write enable
    sc_in<bool>         PN_NAME(ena);   // enable bram

    sc_in<sc_uint<32>>  PN_NAME(addra); // address for bram access
    sc_in<sc_uint<32>>  PN_NAME(dia);   // data in for bram
    sc_out<sc_uint<32>> PN_NAME(doa);   // data out for bram

    // Constructor...
    SC_CTOR(m_single_port_block_ram){
        SC_CTHREAD (proc_clk_ram, clk.pos ());
    }

    // Functions...
    void pn_trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_ram ();


    // This module has an alternative verilog implementation this is used
    // for the ICSC synthesis instead of the SystemC implementation
    // to ensure that the signature of the bram is recognized by the synthesis tool
    // used for the board
    std::string __SC_TOOL_VERILOG_MOD__ =
    R"(
    `include "../verilog/singlePortRam_template.v";
    )";

protected:
    // This is the ram use in the simulation of the bram
    sc_uint<32> ram[RAM_SIZE];

};
#endif // __SINGLE_PORT_RAM_H__