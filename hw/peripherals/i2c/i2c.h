/**
 * @file i2c.h
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
 * @fn SC_MODULE(m_i2c)
 *
 * I2C module combining all submodules and wiring their signals.
 *
 * This module integrates:
 *   - m_i2c_wishbone  (register & Wishbone interface),
 *   - m_i2c_controller (I2C controller FSM),
 *   - m_i2c_clk_control (SCL generation, timing),
 *   - m_i2c_data_control (I2C data path, SDA handling).
 *
 * @par
 * @param[in] clk    System clock for the I2C peripheral.
 * @param[in] reset  Asynchronous reset, active high.
 *
 * @param[in]  sda_i  Input sample of the SDA line (from pad).
 * @param[out] sda_o  SDA output drive value (to pad).
 * @param[out] sda_oe SDA output enable (drive/open-drain control).
 *
 * @param[in]  scl_i  Input sample of the SCL line (from pad).
 * @param[out] scl_o  SCL output drive value (to pad).
 * @param[out] scl_oe SCL output enable (drive/open-drain control).
 *
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <systemc.h>
#include <piconut.h>

#include "i2c_defs.h"

SC_MODULE(m_i2c), pn_module_if
{

public:
    /* ----------------- External Wishbone ports ----------------- */
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    pn_wishbone_slave_t wb_slave;

    /* ----------------- I2C-Ports-------------------*/
    sc_in<bool> sda_i;   /**< @brief Sampled SDA input from the external SDA pin. */
    sc_out<bool> sda_o;  /**< @brief SDA output value driven to the external SDA pin. */
    sc_out<bool> sda_oe; /**< @brief SDA output enable (open-drain control). */

    sc_in<bool> scl_i;   /**< @brief Sampled SCL input from the external SCL pin. */
    sc_out<bool> scl_o;  /**< @brief SCL output value driven to the external SCL pin. */
    sc_out<bool> scl_oe; /**< @brief SCL output enable (open-drain control). */

    /* ----------------- Submodules ----------------- */
    class m_i2c_wishbone* i2c_wishbone;
    class m_i2c_controller* controller;
    class m_i2c_clk_control* clk_control;
    class m_i2c_data_control* data_control;

    /* Constructor...*/
    SC_HAS_PROCESS(m_i2c);
    m_i2c(
        sc_module_name name,
        pn_wb_adr_t base_address)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = base_address,
              .size = PN_I2C_REG_SIZE_BYTES}
    {
        pn_add_wishbone_slave(&wb_slave);

        init_submodules();

        SC_METHOD(update_lines);
        sensitive << sda_out_sig
                  << scl_out_sig
                  << scl_i_sync_reg[0]
                  << sda_i_sync_reg[0];

        SC_CTHREAD(proc_clk, clk.pos());
    }

    void pn_trace(sc_trace_file * tf, int level);

    /**
     * @brief Synchronizes the external inputs.
     *
     */
    void proc_clk();

protected:
    /**
     * @brief Instantiate and bind all I2C submodules.
     *
     */
    void init_submodules();

    /**
     * @brief Update the external SDA/SCL lines and output enable signals.
     */
    void update_lines();

    /* ---------------- Controller connection signals ---------------- */
    sc_signal<bool> PN_NAME(clear_regs_sig);
    // CR1
    sc_signal<bool> PN_NAME(cr1_enable_sig);
    sc_signal<bool> PN_NAME(cr1_start_sig);
    sc_signal<bool> PN_NAME(clear_cr1_start_sig);
    sc_signal<bool> PN_NAME(clear_cr1_stop_sig);
    sc_signal<bool> PN_NAME(cr1_stop_sig);
    sc_signal<bool> PN_NAME(cr1_ack_sig);
    // SR1
    sc_signal<bool> PN_NAME(sr1_start_bit_sig);
    sc_signal<bool> PN_NAME(sr1_addr_bit_sig);
    sc_signal<bool> PN_NAME(sr1_txe_sig);
    sc_signal<bool> PN_NAME(sr1_btf_sig);
    sc_signal<bool> PN_NAME(sr1_af_sig);
    sc_signal<bool> PN_NAME(CPU_read_SR2_sig);
    sc_signal<bool> PN_NAME(CPU_read_SR1_sig);
    sc_signal<bool> PN_NAME(sr1_rxne_sig);
    // SR2
    sc_signal<bool> PN_NAME(sr2_msl_sig);
    sc_signal<bool> PN_NAME(sr2_busy_sig);
    sc_signal<bool> PN_NAME(sr2_tra_sig);
    // DR
    sc_signal<sc_uint<8>> PN_NAME(dr_sig);
    // WB Master
    sc_signal<bool> PN_NAME(cpu_write_dr_sig);
    sc_signal<bool> PN_NAME(CPU_read_DR_sig);

    /* ---------------- clk_control connection signals ---------------- */

    sc_signal<sc_uint<16>> PN_NAME(ccr_sig);
    sc_signal<sc_uint<6>> PN_NAME(trise_sig);
    sc_signal<bool> PN_NAME(scl_on_sig);
    sc_signal<bool> PN_NAME(data_clk_pulse_write_sig);
    sc_signal<bool> PN_NAME(data_clk_pulse_read_sig);
    sc_signal<bool> PN_NAME(scl_ready_sig);
    sc_signal<bool> PN_NAME(init_master_sig);
    sc_signal<bool> PN_NAME(scl_out_sig);
    sc_signal<bool> PN_NAME(scl_in_sig);
    sc_signal<bool> PN_NAME(start_finished_sig);
    sc_signal<bool> PN_NAME(master_stretch_sig);

    /* ---------------- data_control connection signals ---------------- */

    sc_signal<sc_uint<8>> PN_NAME(tx_data_sig);
    sc_signal<bool> PN_NAME(start_ready_sig);
    sc_signal<bool> PN_NAME(stop_finished_sig);
    sc_signal<bool> PN_NAME(ack_received_sig);
    sc_signal<bool> PN_NAME(nack_received_sig);
    sc_signal<bool> PN_NAME(shift_loaded_sig);
    sc_signal<bool> PN_NAME(generate_start_cond_sig);
    sc_signal<bool> PN_NAME(send_addr_sig);
    sc_signal<bool> PN_NAME(start_tx_sig);
    sc_signal<bool> PN_NAME(generate_stop_cond_sig);
    sc_signal<bool> PN_NAME(stop_ready_sig);
    sc_signal<bool> PN_NAME(sda_out_sig);
    sc_signal<bool> PN_NAME(sda_in_sig);
    sc_signal<bool> PN_NAME(sending_data_sig);
    sc_signal<bool> PN_NAME(byte_loaded_in_DR_sig);
    sc_signal<bool> PN_NAME(send_ack_sig);
    sc_signal<bool> PN_NAME(send_nack_sig);
    sc_signal<bool> PN_NAME(start_rx_sig);
    sc_signal<sc_uint<8>> PN_NAME(shift_data_sig);
    sc_signal<bool> PN_NAME(enable_dr_write_sig);

    sc_vector<sc_signal<bool>> PN_NAME_VEC(scl_i_sync_reg, 3);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(sda_i_sync_reg, 3);

private:
};

#endif //__I2C_H__
