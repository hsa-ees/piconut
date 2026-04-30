/**
 * @file i2c_wishbone.h
 * @brief
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.Fleiner1@tha.de>

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
 * @fn SC_MODULE(m_i2c_wishbone)
 *
 * This SystemC module implements the Wishbone slave interface and the
 * register file of the I2C peripheral.
 *
 * It translates Wishbone bus
 * accesses (adr/dat/we/stb/cyc/sel) into:
 *
 * - updates of I2C configuration registers (CR1, CR2, CCR, TRISE, FLTR),
 * - status readback for SR1/SR2 and DR,
 * - control signals for the I2C controller / data / clock submodules
 *
 *
 *
 * @par
 * Wishbone Ports (adr/dat/we/stb/cyc/sel) are included in pn_wishbone_slave_t wb_slave
 * Connection to I2C submodules:
 * @param[in]  clear_regs_in           : Request to clear all I2C-related registers.
 *
 * @param[out] cr1_enable_out          : Drives CR1.PE (peripheral enable).
 * @param[out] cr1_start_out           : Drives CR1.START (start generation).
 * @param[out] cr1_stop_out            : Drives CR1.STOP (stop generation).
 * @param[out] cr1_ack_out             : Drives CR1.ACK (ACK control for RX).
 * @param[in]  clear_cr1_start_in      : Clears CR1.START after controller has accepted it.
 * @param[in]  clear_cr1_stop_in       : Clears CR1.STOP after STOP is finished.
 *
 * @param[in]  sr1_start_bit_in        : SR1.SB  (start bit).
 * @param[in]  sr1_addr_bit_in         : SR1.ADDR (address sent/matched).
 * @param[in]  sr1_txe_in              : SR1.TXE (TX buffer empty).
 * @param[in]  sr1_btf_in              : SR1.BTF (byte transfer finished).
 * @param[in]  sr1_af_in               : SR1.AF  (ACK failure).
 * @param[in]  sr1_rxne_in             : SR1.RXNE (RX buffer not empty).
 * @param[out] CPU_read_SR1_out        : Pulse when CPU (via WB) reads SR1.
 * @param[out] CPU_read_SR2_out        : Pulse when CPU (via WB) reads SR2.
 *
 * @param[in]  sr2_msl_in              : SR2.MSL (master/slave).
 * @param[in]  sr2_busy_in             : SR2.BUSY (bus busy).
 * @param[in]  sr2_tra_in              : SR2.TRA (transmitter/receiver).
 *
 * @param[out] tx_data_out             : Data value to be transmitted.
 * @param[out] dr_out                  : Value written to DR from CPU.
 * @param[out] CPU_write_DR_out        : Pulse when CPU writes DR.
 * @param[out] CPU_read_DR_out         : Pulse when CPU reads DR.
 * @param[in]  enable_dr_write_in      : Data path allows DR update.
 * @param[in]  shift_data_in           : Data shifted in from SDA during RX.
 *
 * @param[out] ccr_out                 : CCR register value (clock control).
 * @param[out] trise_out               : TRISE register value (rise time).
 *
 */

#ifndef __WISHBONE_H__
#define __WISHBONE_H__

#include <systemc.h>
#include <piconut.h>
#include "../i2c_defs.h"
#include "../i2c.h"

SC_MODULE(m_i2c_wishbone)
{

public:
    // wishbone states
    enum e_wb_i2c_state
    {
        WB_IDLE = 0,
        WB_READ = 1,
        WB_WRITE1 = 2,
        WB_WRITE2 = 3,

    };

    // wishbone slave ports ...
    sc_in<bool> PN_NAME(clk);   // clock input
    sc_in<bool> PN_NAME(reset); // reset

    pn_wishbone_slave_t wb_slave;

    /*----------internal signals for submodules ------------*/
    sc_in<bool> PN_NAME(clear_regs_in);

    // CR1
    sc_out<bool> PN_NAME(cr1_enable_out);
    sc_out<bool> PN_NAME(cr1_start_out);
    sc_in<bool> PN_NAME(clear_cr1_start_in);
    sc_in<bool> PN_NAME(clear_cr1_stop_in);
    sc_out<bool> PN_NAME(cr1_stop_out);
    sc_out<bool> PN_NAME(cr1_ack_out);

    // SR1
    sc_in<bool> PN_NAME(sr1_start_bit_in);
    sc_in<bool> PN_NAME(sr1_addr_bit_in);
    sc_in<bool> PN_NAME(sr1_txe_in);
    sc_in<bool> PN_NAME(sr1_btf_in);
    sc_in<bool> PN_NAME(sr1_af_in);
    sc_in<bool> PN_NAME(sr1_rxne_in);
    sc_out<bool> PN_NAME(CPU_read_SR1_out);
    sc_out<bool> PN_NAME(CPU_read_SR2_out);

    // SR2
    sc_in<bool> PN_NAME(sr2_msl_in);
    sc_in<bool> PN_NAME(sr2_busy_in);
    sc_in<bool> PN_NAME(sr2_tra_in);

    // DR
    sc_out<sc_uint<8>> PN_NAME(tx_data_out);
    sc_out<sc_uint<8>> PN_NAME(dr_out);
    sc_out<bool> PN_NAME(CPU_write_DR_out);
    sc_out<bool> PN_NAME(CPU_read_DR_out);
    sc_in<bool> PN_NAME(enable_dr_write_in);
    sc_in<sc_uint<8>> PN_NAME(shift_data_in);

    // CCR and TRISE
    sc_out<sc_uint<16>> PN_NAME(ccr_out);
    sc_out<sc_uint<6>> PN_NAME(trise_out);

    /* Constructor... */
    SC_HAS_PROCESS(m_i2c_wishbone);
    m_i2c_wishbone(
        sc_module_name name,
        pn_wb_adr_t base_address)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = base_address,
              .size = 0x28}
    {
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_slave.stb_i
                  << wb_slave.cyc_i
                  << wb_slave.we_i
                  << wb_slave.adr_i
                  << wb_slave.dat_i
                  << wb_current_state
                  << wb_slave.sel_i
                  << CR1 << CR2 << OAR1 << OAR2 << DR << SR1 << SR2 << CCR << TRISE << FLTR
                  << sr1_start_bit_in << sr1_addr_bit_in << sr1_txe_in << sr1_btf_in << sr1_af_in << sr1_rxne_in
                  << sr2_msl_in << sr2_busy_in << sr2_tra_in
                  << clear_cr1_start_in
                  << clear_cr1_stop_in
                  << clear_regs_in
                  << enable_dr_write_in
                  << shift_data_in;

        SC_CTHREAD(proc_clk_state, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_wb_slave, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);
    }

    void pn_trace(
        sc_trace_file * tf,
        int level = 1);

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_state();

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_wb_slave();
    void proc_comb_wb_slave();

    /**
     * @brief Handles CPU_READ_SR1_SR2_seq_out signal */
    void proc_sr1_sr2_sequence();

    /**
     * @brief resets values in outgoing signals and registers */
    void reset_all_registers();

    /**
     * @brief This function is for the write byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<PN_CFG_WB_DATA_WIDTH> write_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> input_word);

    /**
     * @brief This function is for the read byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<PN_CFG_WB_DATA_WIDTH> read_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> input_word);

protected:
    /** Registers... */
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    // ---------------- I²C register file (32-bit; unused bits read as 0) ----------------
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(CR1);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(CR2);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(OAR1);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(OAR2);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(DR);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(SR1);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(SR2);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(CCR);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(TRISE);
    sc_signal<sc_uint<PN_CFG_WB_DATA_WIDTH>> PN_NAME(FLTR);

    sc_signal<bool> PN_NAME(c_wb_write_en);
};

#endif //__WISHBONE_H__
