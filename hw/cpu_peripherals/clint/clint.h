/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Alexander Beck
                2025 Christian Zellinger
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
 * @fn SC_MODULE(m_clint)
 * @author [Alexander Beck, Christian Zellinger]
 *
 * The Wishbone CLINT (Core Local Interruptor) module provides timer and software interrupt
 * functionality for the PicoNut processor, following the RISC-V specification.
 *
 * It implements the following registers:
 * - `MSIP` (0x0000): Machine Software Interrupt Pending (32-bit)
 * - `MTIMECMP` (0x4000): Machine Timer Compare (64-bit, split into two 32-bit words)
 * - `MTIME` (0xBFF8): Machine Timer (64-bit, split into two 32-bit words)
 *
 * The module increments the `MTIME` register on every clock cycle (unless a write is in progress)
 * and asserts the `mtip_o` output when `MTIME >= MTIMECMP`. The `msip_o` output is asserted when
 * bit 0 of the `MSIP` register is set.
 *
 * Register accesses are handled via the Wishbone bus, supporting both word and byte accesses.
 * The module uses a state machine to ensure correct Wishbone protocol handling and safe multi-cycle writes.
 *
 * @par Ports:
 * @param[in] clk      Clock input
 * @param[in] reset    Reset input
 * @param[in] wb_stb_i Wishbone strobe
 * @param[in] wb_cyc_i Wishbone cycle
 * @param[in] wb_we_i  Wishbone write enable
 * @param[in] wb_adr_i Wishbone address input
 * @param[in] wb_dat_i Wishbone data input
 * @param[in] wb_sel_i Wishbone byte select
 * @param[out] wb_dat_o Wishbone data output
 * @param[out] wb_ack_o Wishbone acknowledge
 * @param[out] wb_err_o Wishbone error
 * @param[out] wb_rty_o Wishbone retry
 * @param[out] msip_o  Machine software interrupt output
 * @param[out] mtip_o  Machine timer interrupt output
 */

#ifndef __CLINT_H__
#define __CLINT_H__

#include <systemc.h>
#include <piconut.h>

#include "clint_defs.h"

SC_MODULE(m_clint)
{
    // Friend functions for testbench access
    friend uint32_t direct_read_msip();
    friend void direct_write_msip(uint32_t);
    friend uint64_t direct_read_mtime();
    friend uint64_t direct_read_mtimecmp();
    friend void direct_write_mtimecmp(uint64_t);

public:
    // Wishbone states for proper protocol implementation
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_READ,
        WB_WRITE1,
        WB_WRITE2
    };

    // Clock and reset
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Wishbone slave interface
    sc_in<bool> PN_NAME(wb_stb_i);         // Strobe/Request
    sc_in<bool> PN_NAME(wb_cyc_i);         // Cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);          // Write enable
    sc_in<sc_uint<32>> PN_NAME(wb_adr_i);  // Address input
    sc_in<sc_uint<32>> PN_NAME(wb_dat_i);  // Data input
    sc_in<sc_uint<4>> PN_NAME(wb_sel_i);   // Byte select
    sc_out<sc_uint<32>> PN_NAME(wb_dat_o); // Data output
    sc_out<bool> PN_NAME(wb_ack_o);        // Acknowledge
    sc_out<bool> PN_NAME(wb_err_o);        // Error output (optional)
    sc_out<bool> PN_NAME(wb_rty_o);        // Retry output (optional)

    // Interrupt outputs
    sc_out<bool> PN_NAME(msip_o); // Machine software interrupt pending
    sc_out<bool> PN_NAME(mtip_o); // Machine timer interrupt pending

protected:
    // Internal registers (use regular variables, not signals)
    sc_signal<sc_uint<32>> msip_reg;     // Software interrupt register
    sc_signal<sc_uint<64>> mtimecmp_reg; // Timer compare register
    sc_signal<sc_uint<64>> mtime_reg;    // Machine timer register

    // Private signals to avoid multiple driver issues
    sc_signal<bool> PN_NAME(in_reset); // Internal reset state

    // Wishbone state machine signals
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);
    sc_signal<bool> PN_NAME(c_wb_write_en);

    // Flag to track if MTIMECMP high word was written
    sc_signal<bool> mtimecmp_high_written;

    // Flag to track if MTIME high word was written, similar to MTIMECMP
    sc_signal<bool> mtime_high_written;

    // State tracking signals for interrupts
    sc_signal<bool> mtip_pending; // Current timer interrupt pending state
    sc_signal<bool> msip_pending; // Current software interrupt pending state

    // Flag for all MTIME write accesses (not just high word)
    sc_signal<bool> mtime_write_active; // Flag to pause incrementing during MTIME write access

public:
#if !PN_PRESYNTHESIZED_H_ONLY(CLINT)
    // Constructor
    SC_CTOR(m_clint)
    {

        // Initialize internal signals
        in_reset.write(true); // Start in reset state
        mtip_pending.write(false);
        msip_pending.write(false);
        mtimecmp_high_written.write(false);
        mtime_high_written.write(false);
        mtime_write_active.write(false);

        // Wishbone state machine
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_stb_i << wb_cyc_i << wb_we_i << wb_adr_i << wb_dat_i
                  << wb_current_state << wb_sel_i << reset
                  << mtimecmp_reg << msip_reg << mtime_reg;

        SC_CTHREAD(proc_clk_state, clk.pos());
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_wb_slave, clk.pos());
        reset_signal_is(reset, true);

        // Interrupt generation - only handles output signals based on register values
        SC_METHOD(proc_comb_process_interrupts);
        sensitive << reset << msip_pending << mtip_pending;
    }

    // Wishbone protocol methods (following template pattern)
    void proc_comb_wb_slave();
    void proc_clk_state();
    void proc_clk_wb_slave();

    // Thread for timer updates
    void timer_thread();

    // Method to update interrupt outputs
    void proc_comb_process_interrupts();

    // Helper methods for register access
    void write_register(sc_uint<32> addr, sc_uint<32> data, sc_uint<8> sel);
    sc_uint<32> read_register(sc_uint<32> addr);

#else // !PN_PRESYNTHESIZED_H_ONLY(CLINT)

    // Header-only variant: Implement all syntactically required methods by empty inliners ...
    SC_CTOR(m_clint) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(CLINT)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(CLINT)
#else // !PN_PRESYNTHESIZED_H_ONLY(CLINT)

    // Declare the module to be pre-synthesized ...
    PN_PRESYNTHESIZED;

#endif // !PN_PRESYNTHESIZED_H_ONLY(CLINT)
};

#endif // __CLINT_H__