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

#include "byteselector.h"

void m_byteselector::Trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, adr_in);
    PN_TRACE(tf, funct3_in);
    PN_TRACE(tf, bsel_out);
    PN_TRACE(tf, invalid_out);
}



void m_byteselector::proc_cmb_byteselector()
{
    // Set default values.
    bsel_out = 0x0;
    invalid_out = 0x0;

    // Read funct3 from IR[14:12]
    switch(funct3_in.read())
    {
        case LB_SB:
        case LBU:
            // Read alu_out[1:0]
            switch(adr_in.read())
            {
                case BYTE_0_ALIGNED:
                    bsel_out = BYTE_0;
                    break;
                case BYTE_1_ALIGNED:
                    bsel_out = BYTE_1;
                    break;
                case BYTE_2_ALIGNED:
                    bsel_out = BYTE_2;
                    break;
                case BYTE_3_ALIGNED:
                    bsel_out = BYTE_3;
                    break;
                default:
                    bsel_out = 0x0;
                    // invalid alignment
                    invalid_out = 0x1;
                    break;
            }
            break;

        case LH_SH:
        case LHU:
            switch(adr_in.read())
            {
                case BYTE_1_ALIGNED:
                    bsel_out = 0x0;
                    // invalid alignment
                    invalid_out = 0x1;
                    break;
                case BYTE_3_ALIGNED:
                    bsel_out = 0x0;
                    // invalid alignment
                    invalid_out = 0x1;
                    break;
                case BYTE_0_ALIGNED:
                    bsel_out = HALF_LOWER;
                    break;
                case BYTE_2_ALIGNED:
                    bsel_out = HALF_UPPER;
                    break;
                default:
                    bsel_out = 0x0;
                    invalid_out = 0x1;
                    break;
            }
            break;

        case LW_SW:
            switch(adr_in.read())
            {
                case BYTE_0_ALIGNED:
                    bsel_out = WORD;
                    break;
                case BYTE_1_ALIGNED:
                    // invalid alignment
                    invalid_out = 0x1;
                    bsel_out = 0x0;
                    break;
                case BYTE_2_ALIGNED:
                    // invalid alignment
                    invalid_out = 0x1;
                    bsel_out = 0x0;
                    break;
                case BYTE_3_ALIGNED:
                    // invalid alignment
                    invalid_out = 0x1;
                    bsel_out = 0x0;
                    break;
                default:
                    // invalid alignment
                    invalid_out = 0x1;
                    bsel_out = 0x0;
                    break;
            }
            break;

        default:
            invalid_out = 0x1;
            bsel_out = 0x0;
            break;
    }
}
