/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
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

#include "extender.h"

void m_extender::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, data_in);
    PN_TRACE(tf, funct3_in);
    PN_TRACE(tf, bsel_in);
    PN_TRACE(tf, extend_out);
    PN_TRACE(tf, assembled_word);
}

void m_extender::proc_cmb_assemble_word()
{
    assembled_word = 0x0;
    // read bssel and pick the correct bit range from the input data
    switch(bsel_in.read())
    {
        case BYTE_0: // 0001
            assembled_word = (data_in.read().range(7, 0));
            break;
        case BYTE_1: // 0010
            assembled_word = (data_in.read().range(15, 8));
            break;
        case BYTE_2: // 0100
            assembled_word = (data_in.read().range(23, 16));
            break;
        case BYTE_3: // 1000
            assembled_word = (data_in.read().range(31, 24));
            break;
        case HALF_LOWER: // 0011
            assembled_word = (data_in.read().range(15, 0));
            break;
        case HALF_UPPER: // 1100
            assembled_word = (data_in.read().range(31, 16));
            break;
        case WORD: // 1111
            assembled_word = data_in.read();
            break;
        default:
            break;
    }
}

void m_extender::proc_cmb_sign_extend()
{
    extend_out = 0x0;

    // read funct3 block of the current instruction and sign extend if necessary
    switch(funct3_in.read())
    {
        case FUNCT3_LB:                       // lb
            if(assembled_word.read()[7] == 1) // sign extension
            {
                extend_out = (sc_uint<24>(0xFFFFFF), assembled_word.read().range(7, 0));
            }
            else
            {
                extend_out = assembled_word.read();
            }
            break;

        case FUNCT3_LH:                        // lh
            if(assembled_word.read()[15] == 1) // sign extension
            {
                extend_out = (sc_uint<16>(0xFFFF), assembled_word.read().range(15, 0));
            }
            else
            {
                extend_out = assembled_word.read();
            }
            break;
        default:                                // lbu,lhu,lw
            extend_out = assembled_word.read(); // no sign extension
            break;
    }
}
