/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
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


#include "blockram_memu.h"
#include <stdint.h>

void m_blockram_memu::Trace(sc_trace_file *tf, int level)
{

    // Ports ...
    PN_TRACE (tf, clka);
    PN_TRACE (tf, clkb);

    PN_TRACE (tf, wea);
    PN_TRACE (tf, ena);

    PN_TRACE (tf, addra);
    PN_TRACE (tf, dia);
    PN_TRACE (tf, doa);

    PN_TRACE (tf, web);
    PN_TRACE (tf, enb);

    PN_TRACE (tf, addrb);
    PN_TRACE (tf, dib);
    PN_TRACE (tf, dob);

}

void m_blockram_memu::proc_clk_ram_port_a () {

    sc_uint<32> combData;

    while (1) {

        wait ();

        // Port A

        if (ena.read() == 1){

            if((((uint32_t (addra.read())) - ((CFG_START_ADDRESS)>>2)) < 0x00) || (((uint32_t) (addra.read()) - ((CFG_START_ADDRESS)>>2)) >= ((CFG_MEMU_BRAM_SIZE*4)))) {
                PN_ERROR("Wrong memory access!");
            }
            combData = ram[(uint32_t)(addra.read ()) - (((uint32_t)(CFG_START_ADDRESS)>>2))];

            // check if nucleus writes into code area
            if (wea.read() == 1) {
                if (addra.read() > ((uint32_t)(CFG_START_ADDRESS) + 0x0001FFFF))
                PN_INFO("!!! WRITING CODE ERROR !!!");
            }

            doa = combData;

            for (uint i = 0; i < 4; i++) {
                if (wea.read()[i] == 1) {
                combData &= ~(0xffU << (8 * i));
                combData |= ( dia.read () & (0xffU << (8 * i)));
                }
            }
            ram[(uint32_t)addra.read() - ((uint32_t)(CFG_START_ADDRESS)>>2)] = combData;
        }
    }
}


void m_blockram_memu::proc_clk_ram_port_b () {

    sc_uint<32> combData;

    while (1) {
        wait();
        // Port B

        if (enb == 1){

            combData = ram[addrb.read ()- CFG_START_ADDRESS];
            dob = combData;

            for (uint i = 0; i < 4; i++) {

                if (web.read()[i] == 1) {
                    combData &= ~(0xffU << (8 * i));
                    combData |= ( dib.read () & (0xffU << (8 * i)));
                }
            }
            ram[addra.read()- CFG_START_ADDRESS] = combData;
        }
    }
}