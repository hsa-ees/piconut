/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2023-2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de
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
#ifndef __BOOTRAM_H__
#define __BOOTRAM_H__

#include <piconut.h>

#include "boot_blockRam.h"


// States for translator
typedef enum  {
    BOOT_IDLE,
    BOOT_WAIT,
    BOOT_SETUP,
    BOOT_SETUP_WE,
    BOOT_ACK,
    BOOT_ACK_WE
}e_stage_wb_2_bram;

SC_MODULE (m_boot_ram){
public:
    // Ports ...
    // Ports (WISHBONE slave)...
    sc_in_clk             PN_NAME(clk_i);     // clock input
    sc_in<bool>           PN_NAME(rst_i);     // reset

    sc_in<bool>           PN_NAME(stb_i);     // strobe input
    sc_in<bool>           PN_NAME(cyc_i);     // cycle valid input
    sc_in<bool>           PN_NAME(we_i);      // indicates write transfer
    sc_in<sc_uint<32/8> > PN_NAME(sel_i);     // byte select inputs
    sc_out<bool>          PN_NAME(ack_o);     // normal termination
    sc_out<bool>          PN_NAME(err_o);     // termination w/ error
    sc_out<bool>          PN_NAME(rty_o);     // termination w/ retry
    sc_in<sc_uint<32> >   PN_NAME(adr_i);     // address bus inputs
    sc_in<sc_uint<32> >   PN_NAME(dat_i);     // input data bus
    sc_out<sc_uint<32> >  PN_NAME(dat_o);     // output data bus


    // Constructor...
    SC_CTOR(m_boot_ram){

        init_submodules();
        SC_CTHREAD (proc_clk_state_ram, clk_i.pos ());
        reset_signal_is (rst_i, true);
        SC_METHOD (proc_comb_state_ram);
        sensitive << state << stb_i << cyc_i << adr_i << we_i;
        SC_METHOD (proc_comb_output_ram);
        sensitive << adr_i << c_set_sel << sel_i;


    }

    // Instance of BlockRAM
    m_dual_port_byte_enable_block_ram* boot_bram;

    // Functions...
    void pn_trace (sc_trace_file * tf, int level = 1);
    void init_submodules();

    // Processes...
    void proc_clk_state_ram ();
    void proc_comb_state_ram();
    void proc_comb_output_ram();

protected:

    sc_signal<bool> PN_NAME(ena);
    sc_signal<sc_uint<32> > PN_NAME(addr); // address signal for port a
    sc_signal<sc_uint<4> > PN_NAME(wea); // write enable signal for port a
    sc_signal<sc_uint<3> > PN_NAME(state); // state of the translator
    sc_signal<sc_uint<3> > PN_NAME(next_state); // next state of the translator
    sc_signal<bool> PN_NAME(c_set_sel); // set sel signal
};


#endif // __BOOTRAM_H__
