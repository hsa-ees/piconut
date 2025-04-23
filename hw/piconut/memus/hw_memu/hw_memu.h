
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

/**
 * @fn SC_MODULE(m_hw_memu)
 * @author Claus Janicher
 * @brief Hardware-MemoryUnit as Interface between Nucleus and systembus.
 *
 * This module is the hardware-memory unit which is used for the hardware implementation of the piconut processor.
 * It is the interface between the Nucleus and the Systembus (Wishbone). Furthermore, it has a built in ram (Blockram) that servers as memory.
 *
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] ip_stb Instructionport Strobe
 * @param[in] ip_adr <32> Bit Instructionport Address
 * @param[in] ip_bsel Instructionport Byteselect
 * @param[out] ip_rdata Instructionport Read-Data
 * @param[out] ip_ack Instructionport ACK
 * @param[in] dp_stb Dataport strobe
 * @param[in] dp_bsel Dataport Byteselect
 * @param[in] dp_adr <32> Bit Dataport Address
 * @param[in] dp_we Dataport Write-Enable
 * @param[in] dp_wdata Dataport Write-Data
 * @param[out] dp_rdata Dataport Read-Data
 * @param[out] dp_ack Dataport ACK
 * @param[out] wb_adr_o Wishbone Adress
 * @param[out] wb_dat_o Wishbone Data out
 * @param[in] wb_dat_i Wishbone Data in
 * @param[out] wb_we_o Wishbone Write-Enable out
 * @param[out] wb_stb_o Wishbone Strobe out
 * @param[out] wb_cyc_o Wishbone Cycle out
 * @param[in] wb_ack_i Wishbone ACK input
 * @param[out] wb_sel_o Wishbone Byte select
 *
 *
 */



#ifndef __HW_MEMU_H__
#define __HW_MEMU_H__

#include <systemc.h>
#include <base.h> // contains all PN_<> Macros and is part of the PicoNut
#include "blockram_memu.h"
#include "elab_alloc.h"
#include "piconut-config.h"

// states for the automat
// caps all upper letters
typedef enum
{
    idle,
    dp_read_1,
    dp_read_2,
    dp_write_1,
    dp_write_2,
    ip_read_1,
    ip_read_2,
    wb_write_1, // wishbone write to uart
    wb_write_2, // wishbone write to uart
    wb_write_3,
    wb_read_1,
    wb_read_2,
} State;

SC_MODULE(m_hw_memu)
{

public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // signals InstructionPort
    sc_in<bool> PN_NAME(ip_stb); // strobe
    // sc_in<bool>         ip_we{"write enable"} // write enable DPort only?
    sc_in<sc_uint<4>> PN_NAME(ip_bsel); // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_in<sc_uint<32>> PN_NAME(ip_adr); // adress <=30
    // sc_in<sc_uint<32>>     ip_wdata{"write data"} // write data from core to memU 32/64/128 bit width DPort only?
    sc_out<bool> PN_NAME(ip_ack);          // acknowledge
    sc_out<sc_uint<32>> PN_NAME(ip_rdata); // hands data to the core 32/64/128 bit width

    // signals DataPort
    sc_in<bool> PN_NAME(dp_stb);           // strobe
    sc_in<bool> PN_NAME(dp_we);            // write enable DPort only?
    sc_in<sc_uint<4>> PN_NAME(dp_bsel);    // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_in<sc_uint<32>> PN_NAME(dp_adr);    // adress <=30
    sc_in<sc_uint<32>> PN_NAME(dp_wdata);  // write data from core to memU 32/64/128 bit width DPort only?
    sc_out<bool> PN_NAME(dp_ack);          // acknowledge
    sc_out<sc_uint<32>> PN_NAME(dp_rdata); // hands data to the core 32/64/128 bit width

    // ports for wishbone-master-interface
    sc_out<sc_uint<32>> PN_NAME(wb_adr_o);
    sc_out<sc_uint<32>> PN_NAME(wb_dat_o);
    sc_in<sc_uint<32>> PN_NAME(wb_dat_i);
    sc_out<bool> PN_NAME(wb_we_o);
    sc_out<bool> PN_NAME(wb_stb_o);
    sc_in<bool> PN_NAME(wb_ack_i);
    sc_out<bool> PN_NAME(wb_cyc_o);
    sc_out<sc_uint<4>> PN_NAME(wb_sel_o);

    // class constructor
    SC_CTOR(m_hw_memu)
    {

        SC_CTHREAD(proc_clk_state, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_comb_transition);
        sensitive << current_state;
        sensitive << dp_wdata;
        sensitive << dp_stb;
        sensitive << dp_bsel;
        sensitive << dp_adr;
        sensitive << dp_we;
        sensitive << ip_adr;
        sensitive << ip_stb;
        sensitive << doa;
        sensitive << dob;
        sensitive << wb_ack_i;
        sensitive << wb_dat_i;
        sensitive << wb_sel_o;

        init_submodules();
    }

    /* Functions...*/
    /**
     * @brief this function is used to generate a tracefile
     * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
     * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void Trace(sc_trace_file * tf, int level = 1);

    /** Processes...
     * This is an example of a combinatorical and a sequential process method
     * You may add further processes here if needed*/
    void proc_clk_state();

    void proc_comb_transition();

private:
    // for blockram initalisation
    m_blockram_memu *blockram_memu;

    void init_submodules();

    // signals

    // connection signals of blockram
protected:
    sc_signal<sc_uint<4>> PN_NAME(wea); // write enable a
    sc_signal<sc_uint<4>> PN_NAME(web); // write enable b
    sc_signal<bool> PN_NAME(ena);
    sc_signal<bool> PN_NAME(enb);
    sc_signal<sc_uint<32>> PN_NAME(addra);
    sc_signal<sc_uint<32>> PN_NAME(addrb);
    sc_signal<sc_uint<32>> PN_NAME(dia);
    sc_signal<sc_uint<32>> PN_NAME(dib);
    sc_signal<sc_uint<32>> PN_NAME(doa);
    sc_signal<sc_uint<32>> PN_NAME(dob);

    // current state
    sc_signal<sc_uint<4>> PN_NAME(current_state);
    sc_signal<sc_uint<4>> PN_NAME(next_state);

};

#endif // __HW_MEMU_H__