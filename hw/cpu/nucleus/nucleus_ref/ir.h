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

/** IR (Instruction Register)
 *
 * The IR module is a simple register that holds the current instruction.
 *
 * Input ports:
 *  - ir_in<32>:         The value to be loaded into the register from the data line of the IPort-Interface.
 *  - en_load_in<1>:     If en_load_in = 1, the value on ir_in is loaded into the register with the next rising edge,
 *                       else the current value is held.
 *
 * Output ports:
 *  - ir_out<32>:   Permanent output of the current register value.
 */

#ifndef __IR_H__
#define __IR_H__

#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

SC_MODULE(m_ir)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<sc_uint<32>> PN_NAME(ir_in);
    sc_in<bool> PN_NAME(en_load_in);

    sc_out<sc_uint<32>> PN_NAME(ir_out);

    /* Constructor... */
    SC_CTOR(m_ir)
    {
        SC_CTHREAD(proc_clk_ir, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_ir);
        sensitive << ir_reg;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_ir();
    void proc_clk_ir();

protected:
    sc_signal<sc_uint<32>> ir_reg;
};

#endif //__IR_H__