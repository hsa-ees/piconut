/**
 * @file csr.h
 * @brief This file contains the definition of the csr module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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
 * @fn SC_MODULE(m_csr)
 *
 * This module implements the CSR's which have effect in the nucleus itself, like
 * for debug purposes, processor status etc. . The registers are connected to the
 * CSR-bus for basic read/write operations. The read/write protection,
 * if it exists, is implemented seperately for each register in its own CThread.
 *
 * Present CSR's in this module:
 * | Address | Name      |
 * |---------|-----------|
 * | 0x300   | mstatus   |
 * | 0x301   | misa      |
 * | 0x7b0   | dcsr      |
 * | 0x7b1   | dpc       |
 * | 0x7b2   | dscratch0 |
 * | 0x7b3   | dscratch1 |
 *
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] csr_bus_read_en_in CSR-bus read enable.
 * @param[in] csr_bus_write_en_in CSR-bus write enable.
 * @param[in] csr_bus_adr_in <`CFG_CSR_BUS_ADR_WIDTH`> CSR-bus address.
 * @param[in] csr_bus_wdata_in <`CFG_CSR_BUS_DATA_WIDTH`> CSR-bus write data.
 * @param[out] csr_bus_rdata_out <`CFG_CSR_BUS_DATA_WIDTH`> CSR-bus read data.
 * @param[in] pc_in <32> Program counter.
 * @param[in] debug_level_enter_ebreak_in Debug level enter request caused by ebreak.
 * @param[in] debug_level_enter_haltrequest_in Debug level enter request caused by halt request.
 * @param[in] debug_level_enter_step_in Debug level enter request caused by step.
 * @param[in] debug_level_leave_in Debug level leave request.
 * @param[out] dpc_out <32> Full CSR Debug-Program-Counter.
 * @param[out] debug_level_enter_out Debug level enter request.
 * @param[out] debug_step_out Debug step signal.
 *
 */

#ifndef __CSR_H__
#define __CSR_H__

#include <base.h>
#include <piconut-config.h>

#include <systemc.h>

#include <cstdint>

// Csr addresses according to risc-v priveleged spec.
typedef enum
{
    // Machine trap setup
    csr_adr_mstatus = 0x300,
    csr_adr_misa = 0x301,

    // Debug mode
    csr_adr_dcsr = 0x7b0,
    csr_adr_dpc = 0x7b1,
    csr_adr_dscratch0 = 0x7b2,
    csr_adr_dscratch1 = 0x7b3,

} e_csr_address;

SC_MODULE(m_csr)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Csr bus
    sc_in<bool> PN_NAME(csr_bus_read_en_in);
    sc_in<bool> PN_NAME(csr_bus_write_en_in);
    sc_in<sc_uint<CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(csr_bus_adr_in);
    sc_in<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_wdata_in);
    sc_out<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_rdata_out);

    /* ---------------  Input signals ---------------  */
    // Program counter
    sc_in<sc_uint<32>> PN_NAME(pc_in);

    // Debug
    sc_in<bool> PN_NAME(debug_level_enter_ebreak_in);
    sc_in<bool> PN_NAME(debug_level_enter_haltrequest_in);
    sc_in<bool> PN_NAME(debug_level_enter_step_in);
    sc_in<bool> PN_NAME(debug_level_leave_in);

    /* --------------- Output signals ---------------  */
    // Dpc
    sc_out<sc_uint<32>> PN_NAME(dpc_out);

    // Debug
    sc_out<bool> PN_NAME(debug_level_enter_out);
    sc_out<bool> PN_NAME(debug_step_out);

    /* Constructor... */
    SC_CTOR(m_csr)
    {
        // Csr bus
        SC_CTHREAD(proc_clk_bus_read, clk.pos());
        reset_signal_is(reset, true);

        // Registers
        SC_CTHREAD(proc_clk_mstatus, clk.pos());
        reset_signal_is(reset, true);
        SC_CTHREAD(proc_clk_misa, clk.pos());
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_dcsr, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_dcsr);
        sensitive << dcsr_reg
                  << debug_level_reg;
        SC_CTHREAD(proc_clk_dpc, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_dpc);
        sensitive << dpc_reg;
        SC_CTHREAD(proc_clk_dscratch0, clk.pos());
        reset_signal_is(reset, true);
        SC_CTHREAD(proc_clk_dscratch1, clk.pos());
        reset_signal_is(reset, true);

        // Control signals
        SC_CTHREAD(proc_clk_debug_level, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_cmb_debug_level_enter);
        sensitive << s_debug_level_enter;

        // Internal signals
        SC_METHOD(proc_cmb_debug_mode_enter);
        sensitive << debug_level_enter_ebreak_in
                  << debug_level_enter_haltrequest_in
                  << debug_level_enter_step_in;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    // Csr bus
    void proc_clk_bus_read();

    // Registers
    void proc_clk_mstatus();
    void proc_clk_misa();

    void proc_clk_dcsr();
    void proc_cmb_dcsr();
    void proc_clk_dpc();
    void proc_cmb_dpc();
    void proc_clk_dscratch0();
    void proc_clk_dscratch1();

    // Privilege level
    void proc_cmb_privilege_level();

    // Debug level
    void proc_clk_debug_level();
    void proc_cmb_debug_level();

    void proc_cmb_debug_level_enter();

    // Internal status signals
    void proc_cmb_debug_mode_enter();

protected:
    // Registers
    enum class e_mstatus_fieldpos : int
    {
        WPRI = 0,
        SIE = 1,
        WPRI1 = 2,
        MIE = 3,
        WPRI2 = 4,
        SPIE = 5,
        UBE = 6,
        MPIE = 7,
        SPP = 8,
        VS = 9,
        MPP = 11,
        FS = 13,
        XS = 15,
        MPRV = 17,
        SUM = 18,
        MXR = 19,
        TVM = 20,
        TW = 21,
        TSR = 22,
        WPRI3 = 23,
        SD = 31,
    };

    enum class e_misa_mxl : int
    {
        BITS_32 = 1,
        BITS_64 = 2,
        BITS_128 = 3,
    };

    enum class e_misa_extensions : int
    {
        A = 0,
        B = 1,
        C = 2,
        D = 3,
        E = 4,
        F = 5,
        H = 7,
        I = 8,
        M = 12,
        N = 13,
        P = 15,
        Q = 16,
        S = 18,
        U = 20,
        V = 21,
        X = 23,
    };

    enum class e_dcsr_fieldpos : int
    {
        PRV = 0,
        STEP = 2,
        NMIP = 3,
        MPRVEN = 4,
        CAUSE = 6,
        STOPTIME = 9,
        STOPCOUNT = 10,
        STEPIE = 11,
        EBREAKU = 12,
        EBREAKS = 13,
        EBREAKM = 15,
        XDEBUGVER = 28,
    };

    enum class e_dcsr_prv : int
    {
        USER = 0,
        SUPERVISOR = 1,
        MACHINE = 3,
    };

    enum class e_dcsr_cause : int
    {
        NONE = 0,
        EBREAK = 1,
        TRIGGER = 2, // Not implemented
        HALTREQUEST = 3,
        SINGLESTEP = 4,       // Not implemented
        RESETHALTREQUEST = 5, // Not implemented
    };

    enum class e_dcsr_xdebugver : int
    {
        NOT_SUPPORTED = 0,
        AS_DESCRIBED = 4,
        CUSTOM = 15,
    };

protected:
    // Registers
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(mstatus_reg);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(misa_reg);

    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(dcsr_reg);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(dpc_reg);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(dscratch0_reg);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(dscratch1_reg);

    // Internal registers
    sc_signal<bool> PN_NAME(debug_level_reg);

    // Internal status signals
    sc_signal<bool> PN_NAME(s_debug_level_enter);
};

#endif //__PC_H__