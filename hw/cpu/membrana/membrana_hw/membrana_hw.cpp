/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Niklas Sirch <niklas.sirch1@tha.de>
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

#include "membrana_hw.h"
#include "membrana_hw_emem.h"


void m_membrana_hw::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, current_state);

    // registers
    PN_TRACE(tf, reg_a_ext);
    PN_TRACE(tf, reg_nuclei);

    PN_TRACE_BUS(tf, ip_adr, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, ip_bsel, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, ip_rdata, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, ip_stb, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, ip_adr, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, ip_ack, PN_CFG_CPU_CORES)

    PN_TRACE_BUS(tf, dp_adr, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_bsel, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_rdata, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_wdata, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_stb, PN_CFG_CPU_CORES)
    PN_TRACE_BUS(tf, dp_ack, PN_CFG_CPU_CORES)
    PN_TRACE_BUS(tf, dp_we, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_lr_sc, PN_CFG_CPU_CORES);
    PN_TRACE_BUS(tf, dp_amo, PN_CFG_CPU_CORES);

    // wishbone
    PN_TRACE(tf, wb_master.ack_i);
    PN_TRACE(tf, wb_master.adr_o);
    PN_TRACE(tf, wb_master.cyc_o);
    PN_TRACE(tf, wb_master.dat_i);
    PN_TRACE(tf, wb_master.dat_o);
    PN_TRACE(tf, wb_master.stb_o);
    PN_TRACE(tf, wb_master.we_o);
    PN_TRACE(tf, wb_master.rty_i);
    PN_TRACE(tf, wb_master.err_i);

    // blockram ports
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

    emem->pn_trace(tf, level);
}

// **************** Helpers *********************

/* This is an example Helper function
it checks if the address is smaller than the CFG_NUT_MEM_SIZE */
// static inline bool AdrIsCacheable(sc_uint<32> adr) { return ((adr ^ CFG_NUT_RESET_ADDR) < CFG_NUT_MEM_SIZE); }

// **************** m_membrana_hw ******************

// modell fürs blockram eine eigene Testbench
// Steuerwerk wird mit gesamter Testbench getestet
// Blockram => simulation (systemC) keine Acks nur ENable, WriteEnable, DataIn, DataOut gleiche Modellierung

// process for state transition , should be combinatory
void m_membrana_hw::proc_comb_transition()
{
    // defaults, only overwrite in other states
    next_state = current_state;

    addra = 0x0;
    addrb = 0x0;

    // defaults for blockram
    dia = 0x0;
    dib = 0x0;
    ena = 0x0;
    enb = 0x0;
    wea = 0x0;
    web = 0x0;

    for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
    {
        // set default values for all nuclei
        dp_ack[i] = 0;
        dp_rdata[i] = 0x0;
        ip_ack[i] = 0;
        ip_rdata[i] = 0x0;
    }

    wb_master.adr_o = 0x0;
    wb_master.dat_o = 0x0;
    wb_master.stb_o = 0x0;
    wb_master.we_o = 0x0;
    wb_master.sel_o = 0x0;
    wb_master.cyc_o = 0x0;

    // register variables
    c_membrana_a_ext local_next_a_ext = reg_a_ext.read();
    sc_uint<NUM_NUCLEI_WIDTH> current_nucleus = reg_nuclei.read();
    next_nucleus = current_nucleus;

    switch(current_state.read())
    {

        case idle:
            // nothing more happens in idle case

            // state decision based on the occurring bus signals for core
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++) // At the moment PicoNut is single core, so we don't care that other cores are neglected
            {
                // IPort controls membrana
                if(ip_stb[i].read() == 1)
                {
                    if(is_adr_internal(ip_adr[i].read()))
                    {
                        next_state = ip_br_read_1;
                    }
                    else
                    {
                        next_state = ip_wb_read_1;
                    }
                    next_nucleus = i;
                    break;
                }

                // DPort controls membrana
                else if((dp_we[i].read() == 1) && (dp_stb[i].read() == 1))
                {
                    next_state = write_check;
                    next_nucleus = i;
                    break;
                }
                else if((dp_stb[i].read() == 1) && (dp_we[i].read() == 0) && !is_nucleus_dp_wb_addr(i))
                {
                    next_state = dp_read_check;
                    next_nucleus = i;
                    break;
                }
                else if((dp_stb[i].read() == 1) && (dp_we[i].read() == 0) && is_nucleus_dp_wb_addr(i))
                {
                    next_state = wb_read_check;
                    next_nucleus = i;
                    break;
                }
            }

            break;

        case dp_read_check: {

            // address blockram A with address from the nucleus core
            addra = (dp_adr[current_nucleus].read() >> 2);
            ena = 0x1;

#if PN_CFG_CPU_CORES > 1
            // helps exit switch from loop
            bool blocked_by_other_amo = false;
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(reg_a_ext.read().amo_pending[i] == 1)
                {
                    if(reg_a_ext.read().amo_pending_adr[i] == dp_adr[current_nucleus].read())
                    {
                        blocked_by_other_amo = true;
                        break;
                    }
                }
            }

            if(blocked_by_other_amo)
            {
                // For multi-core systems the next core should not be this one
                next_state = idle;
            }
            else
#endif
                // If reserve is marked, put address in reservation set.
                if(dp_lr_sc[current_nucleus].read() == 1)
                {
                    next_state = dp_read_lr;
                }
                else if(dp_amo[current_nucleus].read() == 1)
                {
                    next_state = dp_read_set_amo;
                }
                else
                {
                    next_state = dp_read_rdata;
                }

            break;
        }

        case dp_read_lr:
            // keep signals up
            addra = (dp_adr[current_nucleus].read() >> 2);
            ena = 0x1;

            local_next_a_ext.res_adr[current_nucleus] = dp_adr[current_nucleus].read();
            local_next_a_ext.res_valid[current_nucleus] = 1;

            next_state = dp_read_rdata;

            break;

        case dp_read_set_amo:
            // keep signals up
            addra = (dp_adr[current_nucleus].read() >> 2);
            ena = 0x1;

            local_next_a_ext.amo_pending[current_nucleus] = 1;
            local_next_a_ext.amo_pending_adr[current_nucleus] = dp_adr[current_nucleus].read();

            next_state = dp_read_rdata;

            break;

        case dp_read_rdata:

            // hold up the signals to the blockram to hand the doa data to the nucleus core
            ena = 0x1;
            addra = (dp_adr[current_nucleus].read() >> 2);
            // hold up the signals and tell the nucleus core that the transaction is complete
            // using the ACK signal
            dp_rdata[current_nucleus] = doa.read();
            dp_ack[current_nucleus] = 0x1;

            next_state = idle;

            break;

        case write_check: {
            // things to do when dataport writes to memory

            for(unsigned int i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(
                    // Core should invalidate other reservations in `sc`
                    // (This will also invalidate its own reservation in `sc` but only in the next cycle)
                    reg_a_ext.read().res_adr[i] == dp_adr[current_nucleus].read())
                {
                    // Break out from moore automaton as this would require 2^PN_CFG_CPU_CORES-1 states (one store could invalidate all reservations)
                    local_next_a_ext.res_valid[i] = 0; // invalidate reservation
                }
            }

#if PN_CFG_CPU_CORES > 1
            // helps exit switch from loop
            bool blocked_by_other_amo = false;
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(reg_a_ext.read().amo_pending[i] == 1)
                {
                    if(reg_a_ext.read().amo_pending_adr[i] == dp_adr[current_nucleus].read())
                    {
                        blocked_by_other_amo = true;
                        break;
                    }
                }
            }

            if(blocked_by_other_amo)
            {
                // For multi-core systems the next nucleus should not be this one
                next_state = idle;
            }
            else
#endif
                if(dp_lr_sc[current_nucleus].read() == 1)
            {
                if(reg_a_ext.read().res_valid[current_nucleus] == 1)
                {
                    // if the core has a reservation, write to the blockram
                    next_state = write_sc_success;
                }
                else
                {
                    // if the core has no reservation, fail the write
                    next_state = write_sc_fail;
                }
            }
            else if(dp_amo[current_nucleus].read() == 1)
            {
                next_state = write_clear_amo;
            }
            else
            {
                if(is_nucleus_dp_wb_addr(current_nucleus))
                {
                    next_state = wb_write_1;
                }
                else
                {
                    next_state = dp_write_1;
                }
            }

            break;
        }

        case write_clear_amo:

            local_next_a_ext.amo_pending[current_nucleus] = 0; // clear AMO pending

            if(is_nucleus_dp_wb_addr(current_nucleus))
            {
                next_state = wb_write_1;
            }
            else
            {
                next_state = dp_write_1;
            }
            break;

        case write_sc_success:
            // reset the a_ext register for the current core

            local_next_a_ext.res_valid[current_nucleus] = 0;

            if(is_nucleus_dp_wb_addr(current_nucleus))
            {
                next_state = wb_write_1;
            }
            else
            {
                next_state = dp_write_1;
            }

            break;

        case write_sc_fail:
            local_next_a_ext.res_valid[current_nucleus] = 0;

            dp_rdata[current_nucleus] = 0x1; // write status 1 (fail) through rdata
            dp_ack[current_nucleus] = 0x1;

            next_state = idle;

            break;

        case dp_write_1:
            // do write to blockram (for sc also write status to rdata)

            // address blockram and write data to it
            // not now... maybe later: lock port A or B when accessing it? e.g. addra_dp = 1; addra_ip = 0; => portA used by DP
            addra = (dp_adr[current_nucleus].read() >> 2);
            dia = dp_wdata[current_nucleus].read();
            wea = dp_bsel[current_nucleus].read();
            ena = 0x1;

            next_state = dp_write_2;

            break;

        case dp_write_2:

            // set ACK to 1 to tell the nucleus core that the transaction is complete
            dp_ack[current_nucleus] = 0x1;

            next_state = idle;

            break;

        case ip_br_read_1:

            // activate blockram and hand over the address
            ena = 0x1;                                   // activate A
            addra = ip_adr[current_nucleus].read() >> 2; // access memory adress

            next_state = ip_br_read_2;

            break;

        case ip_br_read_2:

            // hold signals for the blockram
            ena = 0x1;
            ip_rdata[current_nucleus] = doa.read();

            // set ack to 1 because data is valid on rdata of IPort
            addra = ip_adr[current_nucleus].read() >> 2;
            ip_ack[current_nucleus] = 0x1;

            next_state = idle;
            break;

        case wb_write_1:

            // read dp_wdata to send data to uart module via wishbone
            wb_master.dat_o = dp_wdata[current_nucleus].read();
            wb_master.adr_o = dp_adr[current_nucleus].read();
            wb_master.stb_o = 0x1;
            wb_master.we_o = 0x1;
            wb_master.cyc_o = 0x1;
            wb_master.sel_o = 0xF;

            next_state = wb_write_2;

            break;

        case wb_write_2:

            // hold up the signals and wait until uart module sends an ACK
            wb_master.dat_o = dp_wdata[current_nucleus].read();
            wb_master.adr_o = dp_adr[current_nucleus].read();
            wb_master.stb_o = 0x1;
            wb_master.we_o = 0x1;
            wb_master.cyc_o = 0x1;
            wb_master.sel_o = 0xF;

            if(wb_master.ack_i.read() == 1)
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
            wb_master.dat_o = dp_wdata[current_nucleus].read();
            wb_master.adr_o = dp_adr[current_nucleus].read();
            wb_master.stb_o = 0x1;
            wb_master.we_o = 0x1;
            wb_master.cyc_o = 0x1;
            wb_master.sel_o = 0xF;

            // ack for nucleus
            dp_ack[current_nucleus] = 0x1;
            next_state = idle;
            break;

        case wb_read_check: {
            // read check for A-Extension

            // (This is a separate state from the dp one, as a merged state would
            // require 1 more cycle for a blockram read.
            // A solution would be to address the blockram with wishbone addresses
            // but this would be undefined behaviour as it would address non existing.)

#if PN_CFG_CPU_CORES > 1
            // helps exit switch from loop
            bool blocked_by_other_amo = false;
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(reg_a_ext.read().amo_pending[i] == 1)
                {
                    if(reg_a_ext.read().amo_pending_adr[i] == dp_adr[core].read())
                    {
                        blocked_by_other_amo = true;
                        break;
                    }
                }
            }

            if(blocked_by_other_amo)
            {
                // For multi-core systems the next core should not be this one
                next_state = idle;
            }
            else
#endif
                // If reserve is marked, put address in reservation set.
                if(dp_lr_sc[current_nucleus].read() == 1)
                {
                    next_state = wb_read_lr;
                }
                else if(dp_amo[current_nucleus].read() == 1)
                {
                    next_state = wb_read_set_amo;
                }
                else
                {
                    next_state = wb_read_1;
                }

            break;
        }

        case wb_read_lr:

            local_next_a_ext.res_adr[current_nucleus] = dp_adr[current_nucleus].read();
            local_next_a_ext.res_valid[current_nucleus] = 1;

            next_state = wb_read_1;

            break;

        case wb_read_set_amo:

            local_next_a_ext.amo_pending[current_nucleus] = 1;
            local_next_a_ext.amo_pending_adr[current_nucleus] = dp_adr[current_nucleus].read();

            next_state = wb_read_1;

            break;

        case wb_read_1:

            // for future implementation to read data from WishBone-Bus. E.g Read from GPIO Module
            wb_master.adr_o = dp_adr[current_nucleus].read();
            wb_master.stb_o = 0x1;
            wb_master.cyc_o = 0x1;
            wb_master.sel_o = 0xF;

            if(wb_master.ack_i.read() == 0x1)
            {
                next_state = wb_read_2;
            }
            break;

        case wb_read_2:
            // read data to somewhere...

            wb_master.cyc_o = 0x1;
            wb_master.stb_o = 0x1;
            wb_master.adr_o = dp_adr[current_nucleus].read();
            wb_master.sel_o = 0xF;
            dp_rdata[current_nucleus] = wb_master.dat_i.read();
            dp_ack[current_nucleus] = 0x1;

            next_state = idle;
            break;

        case ip_wb_read_1:

            wb_master.adr_o = ip_adr[current_nucleus].read();
            wb_master.stb_o = 0x1;
            wb_master.cyc_o = 0x1;
            wb_master.sel_o = 0xF;

            if(wb_master.ack_i.read() == 0x1)
            {
                next_state = ip_wb_read_2;
            }
            break;

        case ip_wb_read_2:

            wb_master.cyc_o = 0x1;
            wb_master.stb_o = 0x1;
            wb_master.adr_o = ip_adr[current_nucleus].read();
            wb_master.sel_o = 0xF;
            ip_rdata[current_nucleus] = wb_master.dat_i.read();
            ip_ack[current_nucleus] = 0x1;

            next_state = idle;
            break;

        default:
            PN_ERROR("Invalid state");
            next_state = idle;
            break;

            // more states
    }

    // any changes to local_next_a_ext will be written to the register
    next_a_ext = local_next_a_ext;
}

// process clk sensitive for state changes
void m_membrana_hw::proc_clk_state()
{
    current_state = idle;
    reg_nuclei = 0; // default core is first
    reg_a_ext = c_membrana_a_ext();

    while(true)
    {
        wait();

        // current_state is the one for the switch case
        // next_state is provided by the current state
        current_state = next_state;
        reg_nuclei = next_nucleus;
        reg_a_ext = next_a_ext;
    }
}

// initialize Blockram

void m_membrana_hw::init_submodules()
{

    emem = sc_new<m_membrana_hw_emem>("emem");
    emem->clka(clk);
    emem->clkb(clk);
    emem->wea(wea);
    emem->web(web);
    emem->ena(ena);
    emem->enb(enb);
    emem->addra(addra);
    emem->addrb(addrb);
    emem->dia(dia);
    emem->dib(dib);
    emem->doa(doa);
    emem->dob(dob);
}
