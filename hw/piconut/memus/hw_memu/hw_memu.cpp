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

#include "hw_memu.h"


/** Not for synthesis...
 * This an example use of the preprocesser directive
 * you can use this directive to differentiate between simulation- and synthesizable-Code */

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_hw_memu::Trace(sc_trace_file *tf, int level)
{

    /* Ports ... */
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, current_state);

    PN_TRACE(tf, ip_adr);
    PN_TRACE(tf, ip_rdata);
    PN_TRACE(tf, ip_stb);
    PN_TRACE(tf, ip_adr);
    PN_TRACE(tf, ip_ack)

    PN_TRACE(tf, dp_adr);
    PN_TRACE(tf, dp_rdata);
    PN_TRACE(tf, dp_wdata);
    PN_TRACE(tf, dp_stb)
    PN_TRACE(tf, dp_ack)

    PN_TRACE(tf, wea);
    PN_TRACE(tf, web);
    PN_TRACE(tf, ena);
    PN_TRACE(tf, enb);
    PN_TRACE(tf, addra);
    PN_TRACE(tf, addrb);
    PN_TRACE(tf, dia);
    PN_TRACE(tf, dib);
    PN_TRACE(tf, doa);
    PN_TRACE(tf, dob);

    // wishbone
    PN_TRACE(tf, wb_ack_i);
    PN_TRACE(tf, wb_adr_o);
    PN_TRACE(tf, wb_cyc_o);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_stb_o);
    PN_TRACE(tf, wb_we_o);

    blockram_memu->Trace(tf, level);

    /* Registers... */
}

// **************** Helpers *********************

/* This is an example Helper function
it checks if the address is smaller than the CFG_NUT_MEM_SIZE */
// static inline bool AdrIsCacheable(sc_uint<32> adr) { return ((adr ^ CFG_NUT_RESET_ADDR) < CFG_NUT_MEM_SIZE); }

// **************** m_hw_memu ******************

// modell fürs blockram eine eigene Testbench
// Steuerwerk wird mit gesamter Testbench getestet
// Blockram => simulation (systemC) keine Acks nur ENable, WriteEnable, DataIn, DataOut gleiche Modellierung

// process for state transition , should be combinatory
void m_hw_memu::proc_comb_transition()
{

    // defaults, only overwrite in other states
    next_state = current_state;
    ip_ack = 0;
    ip_ack = 0x0;
    ip_rdata = 0x0;

    dp_ack = 0;
    dp_ack = 0;
    dp_ack = 0x0;
    dp_rdata = 0x0;

    addra = 0x0;
    addrb = 0x0;

    // defaults for blockram
    dia = 0x0;
    dib = 0x0;
    ena = 0x0;
    enb = 0x0;
    wea = 0x0;
    web = 0x0;

    // set defaults for wishbone bus signals
    wb_adr_o = 0x0;
    wb_cyc_o = 0x0;
    wb_dat_o = 0x0;
    wb_stb_o = 0x0;
    wb_we_o = 0x0;
    wb_sel_o = 0x0;

    switch (current_state.read())
    {

    case idle:
        // nothing more happens in idle case


        // state decision based on the occuring bus signals
        if (((uint32_t)dp_adr.read() - CFG_START_ADDRESS) > (0x101FFFFF) && ((uint32_t)dp_adr.read() - CFG_START_ADDRESS) == 0x20000000)
        {
            // PN_INFO("TEST");
        }

        // IPort controls Memu
        if (ip_stb.read() == 1)
        {

            next_state = ip_read_1; // IPort read request received
        }

        // DPort controls Memu
        else if ((dp_we.read() == 1) && (dp_stb.read() == 1) && (((uint32_t)dp_adr.read() - CFG_START_ADDRESS) <= (0x101FFFFF)))
        {

            next_state = dp_write_1;

        }
        else if ((dp_stb.read() == 1) && (dp_we.read() == 0) && (((uint32_t)dp_adr.read() - CFG_START_ADDRESS) <= (0x101FFFFF)))
        {

            next_state = dp_read_1;
        }

        // wishbone state transition
        else if ((dp_we.read() == 1) && (dp_stb.read() == 1) && ((dp_adr.read() - CFG_START_ADDRESS) > (0x101FFFFF)))
        {

            next_state = wb_write_1;
        }
        else if ((dp_stb.read() == 1) && ((dp_adr.read() - CFG_START_ADDRESS) > (0x101FFFFF)))
        {

            next_state = wb_read_1;
        }

        break;

    case dp_read_1:
        // things to do in DataPort reads from memory

        // adress blockram A with address from the nucleus core
        addra = (dp_adr.read() >> 2);
        ena = 0x1;

        next_state = dp_read_2;

        break;

    case dp_read_2:

        // hold up the signals to the blockram to hand the doa data to the nucleus core
        ena = 0x1;
        addra = (dp_adr.read() >> 2);
        // hold up the signals and tell the nucleus core that the transaction is complete
        // using the ACK signal
        dp_rdata = doa.read();
        dp_ack = 0x1;

        next_state = idle;

        break;

    case dp_write_1:
        // things to do when dataport writes to memory

        // adress blockram and write data to it
        // not now... maybe later: lock port A or B when accessing it? e.g. addra_dp = 1; addra_ip = 0; => portA used by DP
        addra = (dp_adr.read() >> 2);
        dia = dp_wdata.read();
        wea = dp_bsel.read();
        ena = 0x1;

        // set next state to dp_write_2
        next_state = dp_write_2;

        break;

    case dp_write_2:

        // set ACK to 1 to tell the nucleus core that the transaction is complete
        // ena = 0x1;
        dp_ack = 0x1;

        next_state = idle;

        break;

    case ip_read_1:

        // activate blockram and hand over the address
        ena = 0x1;                  // activate A
        addra = ip_adr.read() >> 2; // access memory adress

        next_state = ip_read_2;

        break;

    case ip_read_2:

        // hold signals for the blockram
        ena = 0x1;
        ip_rdata = doa.read();

        // set ack to 1 because data is valid on rdata of IPort
        addra = ip_adr.read() >> 2;
        ip_ack = 0x1;

        next_state = idle;
        break;

    case wb_write_1:

        // read dp_wdata to send data to uart module via wishbone
        wb_dat_o = dp_wdata.read();
        wb_adr_o = dp_adr.read();
        wb_stb_o = 0x1;
        wb_we_o = 0x1;
        wb_cyc_o = 0x1;
        wb_sel_o = 0xF;

        next_state = wb_write_2;

        break;

    case wb_write_2:

        // hold up the signals and wait until uart module sends an ACK
        wb_dat_o = dp_wdata.read();
        wb_adr_o = dp_adr.read();
        wb_stb_o = 0x1;
        wb_we_o = 0x1;
        wb_cyc_o = 0x1;
        wb_sel_o = 0xF;


        if (wb_ack_i.read() == 1)
        {
            next_state = wb_write_3;
        }
        else
        {
            next_state = wb_write_2;
        }

        break;

    case wb_write_3:

        // send the nucleus core an ACK after the WB-action is complete
        wb_dat_o = dp_wdata.read();
        wb_adr_o = dp_adr.read();
        wb_stb_o = 0x1;
        wb_we_o = 0x1;
        wb_cyc_o = 0x1;
        wb_sel_o = 0xF;


        // ack for core
        dp_ack = 0x1;
        next_state = idle;
        break;

    case wb_read_1:

        // for future implementation to read data from WishBone-Bus. E.g Read from GPIO Module
        wb_adr_o = dp_adr.read();
        wb_stb_o = 0x1;
        wb_cyc_o = 0x1;
        wb_sel_o = 0xF;


        if (wb_ack_i.read() == 0x1)
        {
            next_state = wb_read_2;
        }
        break;

    case wb_read_2:
        // read data to somewhere...

        wb_cyc_o = 0x1;
        wb_stb_o = 0x1;
        wb_adr_o = dp_adr.read();
        dp_rdata = wb_dat_i.read();
        dp_ack = 0x1;
        wb_sel_o = 0xF;


        next_state = idle;
        break;

    default:
        next_state = idle;
        break;

        // more states
    }
}

// process clk sensitive for state changes
void m_hw_memu::proc_clk_state()
{

    current_state = idle;

    while (true)
    {
        wait();

        // current_state is the one for the switch case
        // next_state is provided by the current state
        current_state = next_state;
    }
}


// initialize BlockramMemu

void m_hw_memu::init_submodules()
{

    blockram_memu = sc_new<m_blockram_memu>("blockram");
    blockram_memu->clka(clk);
    blockram_memu->clkb(clk);
    blockram_memu->wea(wea);
    blockram_memu->web(web);
    blockram_memu->ena(ena);
    blockram_memu->enb(enb);
    blockram_memu->addra(addra);
    blockram_memu->addrb(addrb);
    blockram_memu->dia(dia);
    blockram_memu->dib(dib);
    blockram_memu->doa(doa);
    blockram_memu->dob(dob);
}