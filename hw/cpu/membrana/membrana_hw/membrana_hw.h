/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Claus Janicher <claus.janicher@tha.de>
                     Niklas Sirch <niklas.sirch1@tha.de>
                     Johannes Hofmann <johannes.hofmann1@tha.de>
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
 * @fn SC_MODULE(m_membrana_hw)
 * @author Claus Janicher
 * @brief Hardware-Membrana as the interface between the nucleus and the systembus.
 *
 * This module is the hardware-membrana unit which is used for the hardware implementation of the piconut processor.
 * It is the interface between the Nucleus and the Systembus (Wishbone). Furthermore, it has a built in ram (Blockram) that serves as memory.
 *
 * The interface for the module is expanded for a multi-nucleus system but not yet implemented.
 * Open points:
 *  - fair next nucleus logic
 *  - on failed AMO use another nucleus
 *  - testing
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
 * @param[in] dp_lr_sc Dataport Load-Reserved/Store-Conditional (vector, per core)
 * @param[in] dp_amo Dataport Atomic Memory Operation (vector, per core)
 * @param[out] wb_adr_o Wishbone Address
 * @param[out] wb_dat_o Wishbone Data out
 * @param[in] wb_dat_i Wishbone Data in
 * @param[out] wb_we_o Wishbone Write-Enable out
 * @param[out] wb_stb_o Wishbone Strobe out
 * @param[out] wb_cyc_o Wishbone Cycle out
 * @param[in] wb_ack_i Wishbone ACK input
 * @param[out] wb_sel_o Wishbone Byte select
 *
 */

#ifndef __MEMBRANA_HW_H__
#define __MEMBRANA_HW_H__

#include <piconut.h>


#define NUM_NUCLEI_WIDTH 6

class c_membrana_a_ext
{
public:
    // yosys couldn't comprehend that registers and the next signals with same
    // length really had the same length so we use a class here
    sc_uint<32> res_adr[PN_CFG_CPU_CORES];  // reserved addresses
    sc_uint<1> res_valid[PN_CFG_CPU_CORES]; // reserve validity indicator bits
    sc_uint<32> amo_pending_adr[PN_CFG_CPU_CORES];
    sc_uint<1> amo_pending[PN_CFG_CPU_CORES];

    c_membrana_a_ext()
    {
        for(uint i = 0; i < PN_CFG_CPU_CORES; ++i)
        {
            res_adr[i] = 0;
            res_valid[i] = 0;
            amo_pending_adr[i] = 0;
            amo_pending[i] = 0;
        }
    }

    bool operator==(const c_membrana_a_ext& t) const
    {
        for(uint i = 0; i < PN_CFG_CPU_CORES; ++i)
        {
            if(res_adr[i] != t.res_adr[i])
                return false;
            if(res_valid[i] != t.res_valid[i])
                return false;
            if(amo_pending[i] != t.amo_pending[i])
                return false;
            if(amo_pending_adr[i] != t.amo_pending_adr[i])
                return false;
        }
        return true;
    }

    friend ::std::ostream& operator<<(
        ::std::ostream& os,
        const c_membrana_a_ext& s)
    {
        os << "{ ";
        for(uint i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            os << "res_adr(" << i << ")=" << s.res_adr[i] << ", ";
        }
        for(uint i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            os << "res_valid(" << i << ")=" << s.res_valid[i] << ", ";
        }
        for(uint i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            os << "amo_pending_adr(" << i << ")=" << s.amo_pending_adr[i] << ", ";
        }
        for(uint i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            os << "amo_pending(" << i << ")=" << s.amo_pending[i];
            os << ((i == PN_CFG_CPU_CORES - 1) ? "" : ", ");
        }
        os << " }";
        return os;
    }

    friend void sc_trace(
        sc_trace_file* tf,
        const c_membrana_a_ext& t,
        const std::string& name)
    {
        for(uint i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            PN_TRACE_R(tf, t, res_adr[i], name);
            PN_TRACE_R(tf, t, res_valid[i], name);
            PN_TRACE_R(tf, t, amo_pending[i], name);
            PN_TRACE_R(tf, t, amo_pending_adr[i], name);
        }
    }
};

// states for the automat
// caps all upper letters
typedef enum
{
    idle,
    dp_read_check,
    dp_read_lr,
    dp_read_set_amo,
    dp_read_rdata,
    write_check,
    write_clear_amo,
    write_sc_success,
    write_sc_fail,
    dp_write_1,
    dp_write_2,
    ip_br_read_1,
    ip_br_read_2,
    wb_write_1,
    wb_write_2,
    wb_write_3,
    wb_read_check,
    wb_read_lr,
    wb_read_set_amo,
    wb_read_1,
    wb_read_2,
    ip_wb_read_1,
    ip_wb_read_2,
    // At max 2^STATE_WIDTH states possible
} State;

#define STATE_WIDTH 5

SC_MODULE(m_membrana_hw), pn_module_if
{

public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // signals InstructionPort
    sc_vector<sc_in<bool>> PN_NAME_VEC(ip_stb, PN_CFG_CPU_CORES);           // strobe
    sc_vector<sc_in<sc_uint<4>>> PN_NAME_VEC(ip_bsel, PN_CFG_CPU_CORES);    // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(ip_adr, PN_CFG_CPU_CORES);    // adress <=30
    sc_vector<sc_out<bool>> PN_NAME_VEC(ip_ack, PN_CFG_CPU_CORES);          // acknowledge
    sc_vector<sc_out<sc_uint<32>>> PN_NAME_VEC(ip_rdata, PN_CFG_CPU_CORES); // hands data to the core 32/64/128 bit width

    // signals DataPort
    sc_vector<sc_in<bool>> PN_NAME_VEC(dp_stb, PN_CFG_CPU_CORES);           // strobe
    sc_vector<sc_in<bool>> PN_NAME_VEC(dp_we, PN_CFG_CPU_CORES);            // write enable DPort only?
    sc_vector<sc_in<sc_uint<4>>> PN_NAME_VEC(dp_bsel, PN_CFG_CPU_CORES);    // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(dp_adr, PN_CFG_CPU_CORES);    // adress <=30
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(dp_wdata, PN_CFG_CPU_CORES);  // write data from core to membrana 32/64/128 bit width DPort only?
    sc_vector<sc_out<bool>> PN_NAME_VEC(dp_ack, PN_CFG_CPU_CORES);          // acknowledge
    sc_vector<sc_out<sc_uint<32>>> PN_NAME_VEC(dp_rdata, PN_CFG_CPU_CORES); // hands data to the core 32/64/128 bit width
    sc_vector<sc_in<bool>> PN_NAME_VEC(dp_lr_sc, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>> PN_NAME_VEC(dp_amo, PN_CFG_CPU_CORES);

    pn_wishbone_master_t wb_master;

    // class constructor
    SC_CTOR(m_membrana_hw)
        : wb_master{
              .alen = 32,
              .dlen = 32}
    {
        pn_add_wishbone_master(&wb_master);

#if !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)

        SC_CTHREAD(proc_clk_state, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_comb_transition);
        sensitive << current_state;
        for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            sensitive << dp_wdata[i];
            sensitive << dp_stb[i];
            sensitive << dp_bsel[i];
            sensitive << dp_adr[i];
            sensitive << dp_we[i];
            sensitive << dp_lr_sc[i];
            sensitive << dp_amo[i];

            sensitive << ip_adr[i];
            sensitive << ip_stb[i];
        }
        sensitive << reg_a_ext;
        sensitive << reg_nuclei;
        sensitive << doa;
        sensitive << dob;
        sensitive << wb_master.ack_i;
        sensitive << wb_master.dat_i;
        sensitive << wb_master.sel_o;

        init_submodules();
    }

    /* Functions...*/
    /**
     * @brief this function is used to generate a tracefile
     * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
     * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void pn_trace(sc_trace_file * tf, int level = 1);

    /** Processes...
     * This is an example of a combinatorial and a sequential process method
     * You may add further processes here if needed*/
    void proc_clk_state();

    void proc_comb_transition();

#else // !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)
    }

    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)

    // for embedded memory initialisation
    class m_membrana_hw_emem* emem;

    void init_submodules();

    // TBD(jh): remove function is_nucleus_dp_wb_addr() and use is_adr_internal() instead.
    inline bool is_nucleus_dp_wb_addr(sc_uint<NUM_NUCLEI_WIDTH> nucleus)
    {
        return ((uint32_t)dp_adr[nucleus].read() - PN_CFG_CPU_RESET_ADR) > (0x101FFFFF);
    }
    bool is_adr_internal(sc_uint<32> adr)
    {
        return adr >= PN_CFG_CPU_RESET_ADR &&
               adr <= PN_CFG_CPU_RESET_ADR + 0x3FFFFF;
    }

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

    /**
     * Register to persist the data needed for a-extension atomicity
     */
    sc_signal<c_membrana_a_ext> PN_NAME(reg_a_ext);
    sc_signal<c_membrana_a_ext> PN_NAME(next_a_ext);

    /**
     * Register to persist the current nucleus between membrana states
     */
    sc_signal<sc_uint<NUM_NUCLEI_WIDTH>> PN_NAME(reg_nuclei);
    sc_signal<sc_uint<NUM_NUCLEI_WIDTH>> PN_NAME(next_nucleus);

    // current state
    sc_signal<sc_uint<STATE_WIDTH>> PN_NAME(current_state);
    sc_signal<sc_uint<STATE_WIDTH>> PN_NAME(next_state);

#else  // !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)
    PN_PRESYNTHESIZED;
#endif // !PN_PRESYNTHESIZED_H_ONLY(MEMBRANA_HW)
};

#undef STATE_WIDTH

#endif // __MEMBRANA_HW_H__
