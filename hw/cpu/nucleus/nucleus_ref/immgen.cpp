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

#include "immgen.h"

void m_immgen::pn_trace(sc_trace_file* tf, int level)
{

    PN_TRACE(tf, data_in);
    PN_TRACE(tf, imm_out);
    PN_TRACE(tf, type_format);
}


void m_immgen::proc_cmb_immgen()
{
    sc_uint<3> immediate_type = 0x0;
    sc_uint<5> imm_identifier = 0x0;

    // Truncated opcode[6:0]
    imm_identifier = data_in.read().range(6, 2);

    // Evaluate opcode[6:2] to determine instruction type
    switch(imm_identifier)
    {
        case OP_ALUI:
        case OP_LOAD:
        case OP_JALR:
        case OP_SYSTEM:
            immediate_type = IMMG_I_type;
            break;

        case OP_STORE:
            immediate_type = IMMG_S_Type;
            break;

        case OP_BRANCH:
            immediate_type = IMMG_B_Type;
            break;

        case OP_JAL:
            immediate_type = IMMG_J_type;
            break;

        case OP_LUI:
        case OP_AUIPC:
            immediate_type = IMMG_U_type;
            break;

        default:
            immediate_type = 0x0;
            break;
    }

    /** Generate immediate value based on instruction type

     */

    switch(immediate_type)
    {
        case IMMG_I_type:

            // If MSB = 1, set upper 21 bits to 1
            if(data_in.read()[31] == 1)
            {
                imm_out = (sc_uint<21>(0x1FFFFF), data_in.read().range(31, 20));
            }
            // If MSB = 0, set upper 21 bits to 0
            else
            {
                imm_out = (sc_uint<21>(0x0), data_in.read().range(30, 20));
            }
            break;

        case IMMG_S_Type:

            // If MSB = 1, set upper 21 bits to 1
            if(data_in.read()[31] == 1)
            {
                imm_out = (sc_uint<21>(0x1FFFFF), data_in.read().range(30, 25), data_in.read().range(11, 8), data_in.read()[7]);
            }
            // If MSB = 0, set upper 21 bits to 0
            else
            {
                imm_out = (sc_uint<21>(0x0), data_in.read().range(31, 25), data_in.read().range(11, 8), data_in.read()[7]);
            }
            break;

        case IMMG_B_Type:
            // If MSB = 1, set upper 20 bits to 1
            if(data_in.read()[31] == 1)
            {
                imm_out = (sc_uint<19>(0x7FFFF), data_in.read()[31], data_in.read()[7], data_in.read().range(30, 25), data_in.read().range(11, 8), sc_uint<1>(0));
            }
            // If MSB = 0, set upper 20 bits to 0
            else
            {
                imm_out = (sc_uint<19>(0x0), data_in.read()[31], data_in.read()[7], data_in.read().range(30, 25), data_in.read().range(11, 8), sc_uint<1>(0));
            }
            break;

        case IMMG_U_type:
            imm_out = (data_in.read()[31], data_in.read().range(30, 20), data_in.read().range(19, 12), sc_uint<12>(0));
            // PN_INFOF(("U-Type imm_out: 0x%08x", (uint32_t)imm_out));
            break;

        case IMMG_J_type:
            // If MSB = 1, set upper 12 bits to 1
            if(data_in.read()[31] == 1)
            {
                imm_out = (sc_uint<12>(0xFFF), data_in.read().range(19, 12), data_in.read()[20], data_in.read().range(30, 25), data_in.read().range(24, 21), sc_uint<1>(0));
            }
            // If MSB = 0, set upper 12 bits to 0
            else
            {
                imm_out = (sc_uint<12>(0x0), data_in.read().range(19, 12), data_in.read()[20], data_in.read().range(30, 25), data_in.read().range(24, 21), sc_uint<1>(0));
            }

            break;

        default:
            PN_WARNING("immgen: Invalid instruction type");
            imm_out = 0x0;
            break;
    }
}