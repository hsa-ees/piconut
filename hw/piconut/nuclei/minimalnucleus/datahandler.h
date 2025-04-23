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

 /** datahndler (Data Handler)
 *
 * The purpose of this model is to arrange the outgoing data to the DPort interface.
 *
 * Input ports:
 *  - data_in<32>: The data to be formatted and written to memory via the DPort interface.
 *  - bsel_in<4>:     Coming from the byteselector module, this signal is used to determine the proper arrangement
 *                 of the data.
 * Output ports:
 *  - data_out<32>: The formatted data to be written to memory via the DPort interface.
 *
 * Working principle:
 *  The RISC-V standard specifies, that data to be written to memory is via the lb,lbu,lh and lhu instructions
 *  sits in the *low* 8 (lb/lbu) or 16 (lh/lhu) bits of the source register. The IPort/DPort, utilizing the byteselect
 *  signal, does address memory bytewise. Therefore, the outgoing data must be arranged according to the instruction,
 *  the byteselect signal and - by extension - by the instruction word and the target memory address.
 *
 * Example:
 *  - Assume register x6 holds the value 0x000000AB.
 *  - Assume register x7 holds the value 0x10000000.
 *  - Assume the instruction is sb x6, 1(x7).
 *  - The targeted effective memory address is therefore 0x10000001.
 *  - Because the address is offset by 1 and the instruction is lb, the byteselect signal is set to '0010' by
 *    the byteselector module.
 *  - Therefore the correct value of the data_out signal to the DPort interfaces wdata input is 0x0000AB00.
 *  - The datahandler module handles this conversion.
 *
 *  Table:
 *  data_in     | bsel   | data_out
 * -------------|--------|----------
 * 0x12345678   | 0001   | 0x00000078
 * 0x12345678   | 0010   | 0x00007800
 * 0x12345678   | 0100   | 0x00780000
 * 0x12345678   | 1000   | 0x78000000
 *
 * 0x12345678   | 0011   | 0x00005678
 * 0x12345678   | 1100   | 0x56780000
 *
 * 0x12345678   | 1111   | 0x12345678
 *
 * All other bsel values are invalid.
 */

#ifndef __WRITE_DATA_HANDLER_H__
#define __WRITE_DATA_HANDLER_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>
#include <typedef.h>

SC_MODULE(m_datahandler)
{
public:
    sc_in<sc_uint<4>> bsel_in;
    sc_in<sc_uint<32>> data_in;
    sc_out<sc_uint<32>> data_out;

    /* Constructor... */
    SC_CTOR(m_datahandler)
    {
        SC_METHOD(proc_cmb_datahandler);
        sensitive << bsel_in << data_in;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_datahandler();

protected:
};

#endif //__CONTROLLER_H__