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

#include "datahandler.h"


void m_datahandler::Trace(sc_trace_file* tf, int level)
{
    /* Ports ... */
    PN_TRACE(tf, bsel_in);
    PN_TRACE(tf, data_in);
    PN_TRACE(tf, data_out);
}

void m_datahandler::proc_cmb_datahandler()
{
    data_out = 0x0; // Default

    // Read bsel
    switch(bsel_in.read())
    {
        case BYTE_0: // '0001'
            data_out.write((sc_uint<24>(0), data_in.read().range(7, 0)));
            break;
        case BYTE_1: // '0010'
            data_out.write((sc_uint<16>(0), data_in.read().range(7, 0), sc_uint<8>(0)));
            break;
        case BYTE_2: // '0100'
            data_out.write((sc_uint<8>(0), data_in.read().range(7, 0), sc_uint<16>(0)));
            break;
        case BYTE_3: // '1000'
            data_out.write((data_in.read().range(7, 0), sc_uint<24>(0)));
            break;
        case HALF_LOWER: // '0011'
            data_out.write((sc_uint<16>(0), data_in.read().range(15, 0)));
            break;
        case HALF_UPPER: // '1100'
            data_out.write((data_in.read().range(15, 0), sc_uint<16>(0)));
            break;
        case WORD: // '1111'
            data_out.write(data_in.read());
            break;
        default: // Invalid bsel
            data_out = 0x0;
            break;
    }
}