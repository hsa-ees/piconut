/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                2023-2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de
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
#include "boot_blockRam.h"


// **************** Dual Port Block Ram with Byte Enable ******************
void m_dual_port_byte_enable_block_ram::Trace(sc_trace_file *tf, int level)
{

    // Port A
    PN_TRACE (tf, clka);
    PN_TRACE (tf, wea);
    PN_TRACE (tf, ena);
    PN_TRACE (tf, addra);
    PN_TRACE (tf, dia);
    PN_TRACE (tf, doa);

    //Port B
    PN_TRACE (tf, clkb);
    PN_TRACE (tf, web);
    PN_TRACE (tf, enb);
    PN_TRACE (tf, addrb);
    PN_TRACE (tf, dib);
    PN_TRACE (tf, dob);

}

/**
 * This module implements a wrapper for a dual port block ram with byte enable.
 * The SystemC processes is implemented for the use in a SystemC simulation.
 *
 * The implementation is a conversion from the verilog templates that are used
 * by the synthesis tools to generate the block ram.
 *
 * For the ICSC synthesis the implementation is replaced by a verilog template
 * this is done by setting the __SC_TOOL_VERILOG_MOD__ in the header file.
 *
 */

void m_dual_port_byte_enable_block_ram::proc_clk_ram_port_a () {

    sc_uint<32> data_var;

    while (1) {

        wait();

        if (ena == 1){
            // checks if the wea byte enables are set and if this is the case
            // write the byte that is selected by the byte enables
            for (uint8_t i = 0; i < 4; i++) {
                if (wea.read()[i] == 1) {
                        data_var &= ~(0xffU << (8 * i));
                        data_var |= ( dia.read () & (0xffU << (8 * i)));
                }
            }
            ram[addra.read()] = data_var;

            // if the enable signal is set read the data from the bram and write it to the output
            data_var = ram[addra.read ()];
            doa = data_var;
        }

    }
}

void m_dual_port_byte_enable_block_ram::proc_clk_ram_port_b () {

    sc_uint<32> data_var;

    while (1) {

        wait ();

        if (enb == 1){
            // checks if the wea byte enables are set and if this is the case
            // write the byte that is selected by the byte enables
            for (uint8_t i = 0; i < 4; i++) {
                if (web.read()[i] == 1) {
                        data_var &= ~(0xffU << (8 * i));
                        data_var |= ( dib.read () & (0xffU << (8 * i)));
                }
            }
            ram[addra.read()] = data_var;

            // if the enable signal is set read the data from the bram and write it to the output
            data_var = ram[addrb.read ()];
            dob = data_var;
        }

    }
}