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

#include "i2c.h"

#include "i2c_wishbone/i2c_wishbone.h"
#include "i2c_controller/i2c_controller.h"
#include "i2c_clk_control/i2c_clk_control.h"
#include "i2c_data_control/i2c_data_control.h"

void m_i2c::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, sda_i);
    PN_TRACE(tf, sda_o);
    PN_TRACE(tf, sda_oe);
    PN_TRACE(tf, scl_i);
    PN_TRACE(tf, scl_o);
    PN_TRACE(tf, scl_oe);

    controller->pn_trace(tf, level);
    clk_control->pn_trace(tf, level);
    i2c_wishbone->pn_trace(tf, level);
    data_control->pn_trace(tf, level);
}

void m_i2c::init_submodules()
{
    /* ---------------- Instantiate submodules ---------------- */
    i2c_wishbone = sc_new<m_i2c_wishbone>("i2c_wishbone", wb_slave.base_address);
    controller = sc_new<m_i2c_controller>("controller");
    clk_control = sc_new<m_i2c_clk_control>("clk_control");
    data_control = sc_new<m_i2c_data_control>("data_control");
    /* ---------------- Connect Wishbone ports ---------------- */
    i2c_wishbone->clk(clk);
    i2c_wishbone->reset(reset);
    i2c_wishbone->wb_slave.stb_i(wb_slave.stb_i);
    i2c_wishbone->wb_slave.cyc_i(wb_slave.cyc_i);
    i2c_wishbone->wb_slave.we_i(wb_slave.we_i);
    i2c_wishbone->wb_slave.adr_i(wb_slave.adr_i);
    i2c_wishbone->wb_slave.dat_i(wb_slave.dat_i);
    i2c_wishbone->wb_slave.dat_o(wb_slave.dat_o);
    i2c_wishbone->wb_slave.sel_i(wb_slave.sel_i);
    i2c_wishbone->wb_slave.ack_o(wb_slave.ack_o);
    i2c_wishbone->wb_slave.err_o(wb_slave.err_o);
    i2c_wishbone->wb_slave.rty_o(wb_slave.rty_o);

    i2c_wishbone->clear_regs_in(clear_regs_sig);
    // -- CR1 (Control Register 1) --
    i2c_wishbone->cr1_enable_out(cr1_enable_sig);
    i2c_wishbone->cr1_start_out(cr1_start_sig);
    i2c_wishbone->clear_cr1_start_in(clear_cr1_start_sig);
    i2c_wishbone->clear_cr1_stop_in(clear_cr1_stop_sig);
    i2c_wishbone->cr1_stop_out(cr1_stop_sig);
    i2c_wishbone->cr1_ack_out(cr1_ack_sig);
    // -- SR1 (Status Register 1) --
    i2c_wishbone->sr1_start_bit_in(sr1_start_bit_sig);
    i2c_wishbone->sr1_addr_bit_in(sr1_addr_bit_sig);
    i2c_wishbone->sr1_txe_in(sr1_txe_sig);
    i2c_wishbone->sr1_btf_in(sr1_btf_sig);
    i2c_wishbone->sr1_af_in(sr1_af_sig);
    i2c_wishbone->CPU_read_SR1_out(CPU_read_SR1_sig);
    i2c_wishbone->sr1_rxne_in(sr1_rxne_sig);
    // -- SR2 (Status Register 2) --
    i2c_wishbone->sr2_msl_in(sr2_msl_sig);
    i2c_wishbone->sr2_busy_in(sr2_busy_sig);
    i2c_wishbone->sr2_tra_in(sr2_tra_sig);
    i2c_wishbone->CPU_read_SR2_out(CPU_read_SR2_sig);
    // -- DR (Data Register) --
    i2c_wishbone->tx_data_out(tx_data_sig);
    i2c_wishbone->dr_out(dr_sig);
    i2c_wishbone->CPU_write_DR_out(cpu_write_dr_sig);
    i2c_wishbone->CPU_read_DR_out(CPU_read_DR_sig);
    i2c_wishbone->enable_dr_write_in(enable_dr_write_sig);
    i2c_wishbone->shift_data_in(shift_data_sig);
    // -- CCR and TRISE Registers --
    i2c_wishbone->ccr_out(ccr_sig);
    i2c_wishbone->trise_out(trise_sig);

    /* ---------------- Connect controller ---------------- */
    controller->clk(clk);
    controller->rst(reset);

    controller->clear_regs_out(clear_regs_sig);
    // -- CR1 (Control) --
    controller->cr1_enable_in(cr1_enable_sig);
    controller->cr1_start_in(cr1_start_sig);
    controller->clear_cr1_start_out(clear_cr1_start_sig);
    controller->clear_cr1_stop_out(clear_cr1_stop_sig);
    controller->cr1_stop_in(cr1_stop_sig);
    controller->cr1_ack_in(cr1_ack_sig);
    // -- SR1 (Status) --
    controller->sr1_start_bit_out(sr1_start_bit_sig);
    controller->sr1_addr_bit_out(sr1_addr_bit_sig);
    controller->sr1_txe_out(sr1_txe_sig);
    controller->sr1_btf_out(sr1_btf_sig);
    controller->sr1_af_out(sr1_af_sig);
    controller->CPU_read_SR2_in(CPU_read_SR2_sig);
    controller->CPU_read_SR1_in(CPU_read_SR1_sig);
    controller->sr1_rxne_out(sr1_rxne_sig);
    // -- SR2 (Status) --
    controller->sr2_msl_out(sr2_msl_sig);
    controller->sr2_busy_out(sr2_busy_sig);
    controller->sr2_tra_out(sr2_tra_sig);
    // -- DR (Data) --
    controller->dr_in(dr_sig);
    // -- To/From clkControl --
    controller->scl_ready_in(scl_ready_sig);
    controller->init_master_out(init_master_sig);
    controller->scl_on_out(scl_on_sig);
    controller->master_stretch_out(master_stretch_sig);
    // -- To/From dataControl --
    controller->ack_received_in(ack_received_sig);
    controller->nack_received_in(nack_received_sig);
    controller->shift_loaded_in(shift_loaded_sig);
    controller->start_tx_out(start_tx_sig);
    controller->generate_start_cond_out(generate_start_cond_sig);
    controller->generate_stop_cond_out(generate_stop_cond_sig);
    controller->start_finished_in(start_finished_sig);
    controller->stop_finished_in(stop_finished_sig);
    controller->sending_data_in(sending_data_sig);
    controller->byte_loaded_in_DR_in(byte_loaded_in_DR_sig);
    controller->send_ack_out(send_ack_sig);
    controller->send_nack_out(send_nack_sig);
    controller->start_rx_out(start_rx_sig);

    // -- WB Master --
    controller->CPU_write_DR_in(cpu_write_dr_sig);
    controller->CPU_read_DR_in(CPU_read_DR_sig);

    /* ---------------- Connect clk_control ---------------- */
    clk_control->clk(clk);
    clk_control->reset(reset);
    // -- CCR Register --
    clk_control->ccr_in(ccr_sig);
    clk_control->trise_in(trise_sig);
    // -- To/From controller --
    clk_control->scl_on_in(scl_on_sig);
    clk_control->init_master_in(init_master_sig);
    clk_control->scl_ready_out(scl_ready_sig);
    clk_control->master_stretch_in(master_stretch_sig);
    // -- To data_control --
    clk_control->data_clk_pulse_write_out(data_clk_pulse_write_sig);
    clk_control->data_clk_pulse_read_out(data_clk_pulse_read_sig);

    clk_control->start_ready_in(start_ready_sig);
    clk_control->start_finished_out(start_finished_sig);
    clk_control->generate_stop_in(generate_stop_cond_sig);

    clk_control->stop_ready_out(stop_ready_sig);
    clk_control->stop_finished_out(stop_finished_sig);

    // -- SCL Pin Interface --
    clk_control->scl_in(scl_in_sig);
    clk_control->scl_out(scl_out_sig);

    /* --------------- Connect data_control --------------- */
    data_control->clk(clk);
    data_control->reset(reset);
    // -- From clk_control --
    data_control->data_clk_pulse_write_in(data_clk_pulse_write_sig);
    data_control->data_clk_pulse_read_in(data_clk_pulse_read_sig);
    // -- From WB -- --
    data_control->tx_data_in(tx_data_sig);
    // -- To/From controller --
    data_control->shift_loaded_out(shift_loaded_sig);
    data_control->ack_received_out(ack_received_sig);
    data_control->nack_received_out(nack_received_sig);
    data_control->start_tx_in(start_tx_sig);

    data_control->generate_start_cond_in(generate_start_cond_sig);
    data_control->start_ready_out(start_ready_sig);
    data_control->stop_ready_in(stop_ready_sig);
    data_control->sending_data_out(sending_data_sig);

    data_control->byte_loaded_in_DR_out(byte_loaded_in_DR_sig);
    data_control->send_ack_in(send_ack_sig);
    data_control->send_nack_in(send_nack_sig);
    data_control->start_rx_in(start_rx_sig);
    data_control->shift_data_out(shift_data_sig);
    data_control->enable_dr_write_out(enable_dr_write_sig);

    // -- SDA Pin Interface --
    data_control->sda_in(sda_in_sig);
    data_control->sda_out(sda_out_sig);
}

void m_i2c::update_lines()
{
    if(sda_out_sig.read() == false)
    {
        sda_o.write(false);
        sda_oe.write(true);
    }
    else
    {
        sda_o.write(true);
        sda_oe.write(false);
    }

    if(scl_out_sig.read() == false)
    {
        scl_o.write(false);
        scl_oe.write(true);
    }
    else
    {
        scl_o.write(true);
        scl_oe.write(false);
    }

    scl_in_sig = scl_i_sync_reg[0].read();
    sda_in_sig = sda_i_sync_reg[0].read();
}

void m_i2c::proc_clk()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            for(size_t i = 0; i < 3; ++i)
            {
                scl_i_sync_reg[i] = 0;
                sda_i_sync_reg[i] = 1;
            }
        }
        else
        {
            scl_i_sync_reg[2] = scl_i.read();
            sda_i_sync_reg[2] = sda_i.read();

            scl_i_sync_reg[1] = scl_i_sync_reg[2];
            sda_i_sync_reg[1] = sda_i_sync_reg[2];

            scl_i_sync_reg[0] = scl_i_sync_reg[1];
            sda_i_sync_reg[0] = sda_i_sync_reg[1];
        }

        wait();
    }
}