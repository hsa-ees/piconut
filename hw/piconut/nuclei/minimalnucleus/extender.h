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

/** Extender
 *
 * The extender module is responsible for shifting the incoming data from the DPort interface to the correct position
 * and sign extending that data. Therefore it is only involved in load operations.
 *
 * Because the main memory of the system is byte-addressable and returns requested data in as it is stored in memory,
 * the extender is necessary to align the data correctly.
 *
 * Input ports:
 *  - data_in<32>: The data to be extended.
 *  - funct3_in<3>: The funct3 block of the current instruction. (IR[14:12])
 *  - bsel_in<4>: The byte select signal coming from the byteselector module.
 *
 * Output ports:
 *  - extend_out<32>: The extended data.
 *
 * The working principle is similar to the datahandler module: Depending on the bsel signal, the data is shifted to the
 * correct position. The funct3 signal is used to determine if the data needs to be sign extended. Sign extension is done
 * by evaluating the most significant bit of the data and concatonating it with an appropriate amount of 1s if
 * necessary.
 *
 */

#ifndef __EXTENDER_H__
#define __EXTENDER_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>
#include <typedef.h>

typedef enum
{
    FUNCT3_LB = 0x0,
    FUNCT3_LH = 0x1,
    FUNCT3_LW = 0x2,
} e_extender_funct3;

SC_MODULE(m_extender)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);
    sc_in<sc_uint<32>> PN_NAME(data_in);
    sc_in<sc_uint<4>> PN_NAME(bsel_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);

    sc_out<sc_uint<32>> PN_NAME(extend_out);

    /* Constructor... */
    SC_CTOR(m_extender)
    {
        SC_METHOD(proc_cmb_assemble_word);
        sensitive << data_in << bsel_in << assembled_word;
        SC_METHOD(proc_cmb_sign_extend);
        sensitive << assembled_word << funct3_in;
        SC_METHOD(proc_cmb_output);
        sensitive << extender_reg << extended_word;
        SC_CTHREAD(proc_clk_reg, clk.pos());
        reset_signal_is(reset, true);
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_assemble_word();
    void proc_cmb_sign_extend();
    void proc_clk_reg();
    void proc_cmb_output();

protected:
    sc_signal<sc_uint<32>> PN_NAME(assembled_word);
    sc_signal<sc_uint<32>> PN_NAME(extender_reg);
    sc_signal<sc_uint<32>> PN_NAME(extended_word);
};

#endif //__REGFILE_H__