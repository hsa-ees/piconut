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

#include "bootram.h"

// Traces of Module
void m_boot_ram::Trace(sc_trace_file *tf, int level)
{

    // Ports ...
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);

    PN_TRACE(tf, stb_i);
    PN_TRACE(tf, cyc_i);
    PN_TRACE(tf, we_i);
    PN_TRACE(tf, sel_i);
    PN_TRACE(tf, ack_o);
    PN_TRACE(tf, err_o);
    PN_TRACE(tf, rty_o);
    PN_TRACE(tf, adr_i);
    PN_TRACE(tf, dat_i);
    PN_TRACE(tf, dat_o);

    boot_bram->Trace(tf, level);
}

/**
 *  This module implements a moore machine that  translates the wishbone bus
 *  to a block ram interface.
 *
 *  It reacts to a write or read access to the wishbone slave interface.
 *
 *  It supports a 32 bit address and daata bus and a 4 bit byte select.
 *  It supports single word read and write access not burst access.
 *
 *
 *
*/

void m_boot_ram::init_submodules()
{

    // Instanciates the Blockram module and connects it to the translator
    boot_bram = sc_new<m_dual_port_byte_enable_block_ram>("boot_bram");

    boot_bram->clka(clk_i);
    boot_bram->clkb(clk_i);
    boot_bram->dia(dat_i);
    boot_bram->doa(dat_o);
    boot_bram->addra(addr);
    boot_bram->wea(wea);
    boot_bram->ena(ena);
    boot_bram->web(vh_const<sc_uint<4>> (0));
    boot_bram->enb(vh_const<bool> (0));
    boot_bram->dob(vh_open);
    boot_bram->addrb(vh_const<sc_uint<32>> (0));
    boot_bram->dib(vh_const<sc_uint<32>> (0));
}

void m_boot_ram::proc_clk_state_ram()
{
    // simple state transition method

    state = BOOT_IDLE;

    wait();

    while (true)
    {

        state = next_state;
        wait();
    }


}

void m_boot_ram::proc_comb_state_ram()
{

    // defaults
    ack_o = 0;
    rty_o = 0;
    err_o = 0;
    ena = 0;
    c_set_sel = 0;
    next_state = BOOT_IDLE;

    switch (state.read())
    {
    case BOOT_IDLE:

        // when the module is addressed at the correct address go to wait state
        // TODO: Make the address range configurable
        if (cyc_i && stb_i && adr_i.read() >= (0xC0000000UL) && adr_i.read() <= (0xC0020000UL)){
            next_state = BOOT_WAIT;
        }
        break;

    case BOOT_WAIT:

        // determine if it is a read or a write access that is sent over the wishbone bus
        // and go to the corresponding state
        if(we_i)
            next_state = BOOT_SETUP_WE;
        else
            next_state = BOOT_SETUP;
        break;

    case BOOT_SETUP:

        // start the read process at the blockram interface and allow the data to be read
        ena = 1;
        next_state = BOOT_ACK;
        break;

    case BOOT_SETUP_WE:

        // allow selection of bytes to write and start write process at the blockram interface
        ena = 1;
        c_set_sel = 1;
        next_state = BOOT_ACK_WE;
        break;

    case BOOT_ACK:

        // send out acknowlegment for read data received from the blockram
        ena = 1;
        ack_o = 1;

        // only go back to idle if wishbone access was ended
        if (!cyc_i && !stb_i)
            next_state = BOOT_IDLE;

        break;

    case BOOT_ACK_WE:

            // send out acknolegement for write data received from the blockram
            ena = 1;
            c_set_sel = 1;
            ack_o = 1;

            // only go back to idle if wishbone access was ended
            if (!cyc_i && !stb_i)
                next_state = BOOT_IDLE;

            break;


    default:
        next_state = BOOT_IDLE;
    }

    // create enable by combining cyc and stb because the blockram does only have an enable
    // signal
    ena = cyc_i && stb_i;


}


void m_boot_ram::proc_comb_output_ram(){


    // get the absolute adress in the blockram from the normal addres
    // TODO: Make the address range configurable
    addr = (adr_i.read() & ~ 0xC0000000UL) >> 2;

    // set the wea when a write is occurring to write to the correct bytes of the word
    if (c_set_sel == 1){
        wea = sel_i;
    }else{
        wea = 0;
    }
}