/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                     2025 Johannes Fleiner <johannes.fleiner1@tha.de>
                     2025 Sebastian Ebenhöh <sebastian.moritz.ebenhöh@tha.de>
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

#include "../i2c_controller.h"
#include <stdint.h>
#include <systemc.h>
#define PN_TRACE_ENABLE

#define PERIOD_NS 40

// Clock & Reset
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// Global clear
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
sc_signal<bool> PN_NAME(sr1_rxne_sig);
sc_signal<bool> PN_NAME(CPU_read_SR1_sig);
sc_signal<bool> PN_NAME(CPU_read_SR2_sig);

// SR2
sc_signal<bool> PN_NAME(sr2_msl_sig);
sc_signal<bool> PN_NAME(sr2_busy_sig);
sc_signal<bool> PN_NAME(sr2_tra_sig);

// DR
sc_signal<sc_uint<8>> PN_NAME(dr_in_sig);

// clk_control
sc_signal<bool> PN_NAME(scl_ready_sig);
sc_signal<bool> PN_NAME(init_master_sig);
sc_signal<bool> PN_NAME(scl_on_sig);
sc_signal<bool> PN_NAME(master_stretch_sig);

// data_control
sc_signal<bool> PN_NAME(stop_finished_sig);
sc_signal<bool> PN_NAME(start_finished_sig);
sc_signal<bool> PN_NAME(sending_data_sig);
sc_signal<bool> PN_NAME(ack_received_sig);
sc_signal<bool> PN_NAME(nack_received_sig);
sc_signal<bool> PN_NAME(send_ack_sig);
sc_signal<bool> PN_NAME(send_nack_sig);
sc_signal<bool> PN_NAME(start_rx_sig);
sc_signal<bool> PN_NAME(byte_loaded_in_DR_sig);
sc_signal<bool> PN_NAME(shift_loaded_sig);
sc_signal<bool> PN_NAME(generate_start_cond_sig);
sc_signal<bool> PN_NAME(start_tx_sig);
sc_signal<bool> PN_NAME(generate_stop_cond_sig);

// WB master
sc_signal<bool> PN_NAME(cpu_write_dr_sig);
sc_signal<bool> PN_NAME(CPU_read_DR_sig);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; ++i)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("m_i2c_controller_tb");

    m_i2c_controller PN_NAME(i_dut);

    i_dut.clk(clk);
    i_dut.rst(reset);

    i_dut.clear_regs_out(clear_regs_sig);

    // CR1
    i_dut.cr1_enable_in(cr1_enable_sig);
    i_dut.cr1_start_in(cr1_start_sig);
    i_dut.clear_cr1_start_out(clear_cr1_start_sig);
    i_dut.clear_cr1_stop_out(clear_cr1_stop_sig);
    i_dut.cr1_stop_in(cr1_stop_sig);
    i_dut.cr1_ack_in(cr1_ack_sig);

    // SR1
    i_dut.sr1_start_bit_out(sr1_start_bit_sig);
    i_dut.sr1_addr_bit_out(sr1_addr_bit_sig);
    i_dut.sr1_txe_out(sr1_txe_sig);
    i_dut.sr1_btf_out(sr1_btf_sig);
    i_dut.sr1_af_out(sr1_af_sig);
    i_dut.sr1_rxne_out(sr1_rxne_sig);
    i_dut.CPU_read_SR1_in(CPU_read_SR1_sig);
    i_dut.CPU_read_SR2_in(CPU_read_SR2_sig);

    // SR2
    i_dut.sr2_msl_out(sr2_msl_sig);
    i_dut.sr2_busy_out(sr2_busy_sig);
    i_dut.sr2_tra_out(sr2_tra_sig);

    // DR
    i_dut.dr_in(dr_in_sig);

    // clk Control
    i_dut.scl_ready_in(scl_ready_sig);
    i_dut.init_master_out(init_master_sig);
    i_dut.scl_on_out(scl_on_sig);
    i_dut.master_stretch_out(master_stretch_sig);

    // data Control
    i_dut.stop_finished_in(stop_finished_sig);
    i_dut.start_finished_in(start_finished_sig);
    i_dut.sending_data_in(sending_data_sig);
    i_dut.ack_received_in(ack_received_sig);
    i_dut.nack_received_in(nack_received_sig);
    i_dut.send_ack_out(send_ack_sig);
    i_dut.send_nack_out(send_nack_sig);
    i_dut.start_rx_out(start_rx_sig);
    i_dut.byte_loaded_in_DR_in(byte_loaded_in_DR_sig);
    i_dut.shift_loaded_in(shift_loaded_sig);
    i_dut.generate_start_cond_out(generate_start_cond_sig);
    i_dut.start_tx_out(start_tx_sig);
    i_dut.generate_stop_cond_out(generate_stop_cond_sig);

    // WB master
    i_dut.CPU_write_DR_in(cpu_write_dr_sig);
    i_dut.CPU_read_DR_in(CPU_read_DR_sig);

    // Traces
    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
   PN_INFO("***** Simulation started *****");

    reset = 1;
    cr1_enable_sig = 0;
    cr1_start_sig = 0;
    cr1_stop_sig = 0;
    cr1_ack_sig = 0;

    scl_ready_sig = 0;
    start_finished_sig = 0;
    stop_finished_sig = 0;

    cpu_write_dr_sig = 0;
    dr_in_sig = 0;

    ack_received_sig = 0;
    nack_received_sig = 0;

    shift_loaded_sig = 0;
    sending_data_sig = 0;
    byte_loaded_in_DR_sig = 0;

    CPU_read_SR1_sig = 0;
    CPU_read_SR2_sig = 0;
    CPU_read_DR_sig = 0;

    run_cycle(5);
    reset = 0;
    run_cycle(2);

    // --- CHECK RESET STATE ---
    PN_ASSERTM(init_master_sig.read() == 0, "Controller not in OFF/IDLE state after reset (Init Master is active)");
    PN_ASSERTM(sr2_busy_sig.read() == 0, "Controller indicates BUSY immediately after reset");

    // =========================================================================
    // 1) WRITE-TRANSFER: START + Address (Write) + 2 Bytes + STOP
    // =========================================================================

    cr1_enable_sig = 1;
    run_cycle(5);

    // START -> INIT_MASTER
    cr1_start_sig = 1;
    run_cycle(2);
    PN_ASSERTM(init_master_sig.read() == 1, "Controller did not enter INIT_MASTER state");

    // START_COND
    scl_ready_sig = 1;
    run_cycle(2);
    PN_ASSERTM(generate_start_cond_sig.read() == 1, "Controller did not request START condition");

    // START_FIN
    start_finished_sig = 1;
    run_cycle(2);
    cr1_start_sig = 0;
    PN_ASSERTM(sr1_start_bit_sig.read() == 1, "Flag Error: SB flag not set after START finished");

    // CPU reads SR1 (SB-Flag) -> CPU_READ_SR1_WAIT_DR
    CPU_read_SR1_sig = 1;
    run_cycle(1);
    CPU_read_SR1_sig = 0;
    start_finished_sig = 0;

    // CPU writes Slave-Adresse (Write: 0xA0, LSB=0) in DR
    dr_in_sig = 0xA0;
    cpu_write_dr_sig = 1;
    run_cycle(1);
    cpu_write_dr_sig = 0;
    PN_ASSERTM(start_tx_sig.read() == 1, "Controller did not start TX mode (ADDR write)");
    PN_ASSERTM(sr1_start_bit_sig.read() == 0, "Flag Error: SB flag not cleared after DR write");

    // LOAD_ADDR
    sending_data_sig = 1;
    run_cycle(1);
    sending_data_sig = 0;

    // Address-ACK from Slave -> ADDR_FIN
    ack_received_sig = 1;
    run_cycle(2);
    ack_received_sig = 0;
    PN_ASSERTM(sr1_addr_bit_sig.read() == 1, "Flag Error: ADDR flag not set after ACK");

    // CPU reads SR1 & SR2 -> READY_RW
    CPU_read_SR1_sig = 1;
    run_cycle(1);
    CPU_read_SR1_sig = 0;

    CPU_read_SR2_sig = 1;
    run_cycle(1);
    CPU_read_SR2_sig = 0;

    PN_ASSERTM(sr1_addr_bit_sig.read() == 0, "Flag Error: ADDR flag not cleared after SR2 read");

    // READY_RW: (dr_in & 1 == 0)
    // CPU writes byte in DR -> LOAD_SHIFT_REG
    dr_in_sig = 0x20;
    cpu_write_dr_sig = 1;
    run_cycle(1);
    cpu_write_dr_sig = 0;
    PN_ASSERTM(start_tx_sig.read() == 1, "Controller did not start TX mode (DATA write)");


    // shift register loaded -> LOAD_SHIFT_REG_2
    shift_loaded_sig = 1;
    run_cycle(1);
    shift_loaded_sig = 0;

    // SEND_DATA_1
    sending_data_sig = 1;
    run_cycle(1);
    sending_data_sig = 0;

    // SEND_DATA_2
    dr_in_sig = 0xAA;
    cpu_write_dr_sig = 1;
    run_cycle(1);
    cpu_write_dr_sig = 0;

    // LOAD_SHIFT_REG
    ack_received_sig = 1;
    run_cycle(2);
    ack_received_sig = 0;

    // shift register loaded -> LOAD_SHIFT_REG_2
    shift_loaded_sig = 1;
    run_cycle(1);
    shift_loaded_sig = 0;
    PN_ASSERTM(sr1_txe_sig.read() == 1, "Flag Error: TXE not set after shift load");

    // SEND_DATA_1
    sending_data_sig = 1;
    run_cycle(1);
    sending_data_sig = 0;

    // WRITE_FIN
    ack_received_sig = 1;
    run_cycle(2);
    ack_received_sig = 0;
    PN_ASSERTM(sr1_btf_sig.read() == 1, "Flag Error: BTF not set after last byte");

    // STOP_COND
    cr1_stop_sig = 1;
    run_cycle(2);
    PN_ASSERTM(generate_stop_cond_sig.read() == 1, "Controller did not request STOP condition");

    // STOP_FINISHED -> IDLE
    stop_finished_sig = 1;
    run_cycle(1);
    stop_finished_sig = 0;

    cr1_stop_sig = 0;
    run_cycle(3);

    PN_ASSERTM(init_master_sig.read() == 0, "Controller did not return to IDLE after STOP");

    // =========================================================================
    // 2) READ-TRANSFER: START + Address (Read) + 2 Bytes + STOP
    // =========================================================================

    ack_received_sig = 0;
    nack_received_sig = 0;
    shift_loaded_sig = 0;
    byte_loaded_in_DR_sig = 0;
    CPU_read_SR1_sig = 0;
    CPU_read_SR2_sig = 0;
    CPU_read_DR_sig = 0;
    cr1_ack_sig = 1;

    // START
    cr1_start_sig = 1;
    scl_ready_sig = 0;
    start_finished_sig = 0;
    run_cycle(2);

    // START_COND
    scl_ready_sig = 1;
    run_cycle(2);

    // START-Finish
    start_finished_sig = 1;
    run_cycle(2);
    cr1_start_sig = 0;

    // CPU reads SR1 (SB)
    CPU_read_SR1_sig = 1;
    run_cycle(1);
    CPU_read_SR1_sig = 0;

    start_finished_sig = 0;

    // CPU writes Slave-Adresse (Read: 0xA1, LSB=1) in DR
    dr_in_sig = 0xA1;
    cpu_write_dr_sig = 1;
    run_cycle(1);
    cpu_write_dr_sig = 0;

    // LOAD_ADDR -> SEND_ADDR
    sending_data_sig = 1;
    run_cycle(1);
    sending_data_sig = 0;

    // Slave ACK -> ADDR_FIN
    ack_received_sig = 1;
    run_cycle(2);
    ack_received_sig = 0;

    // CPU reads SR1 & SR2 -> READY_RW
    CPU_read_SR1_sig = 1;
    run_cycle(1);
    CPU_read_SR1_sig = 0;

    CPU_read_SR2_sig = 1;
    run_cycle(1);
    CPU_read_SR2_sig = 0;

    // READY_RW: Da dr_in & 1 == 1 -> RECV_DATA_1
    run_cycle(2);
    PN_ASSERTM(start_rx_sig.read() == 1, "Controller did not start RX mode");

    // RECV_DATA_1
    shift_loaded_sig = 1;
    run_cycle(1);
    shift_loaded_sig = 0; // -> CHECK_ACK_NACK

    // CHECK_ACK_NACK -> SEND_ACK
    run_cycle(1);
    PN_ASSERTM(send_ack_sig.read() == 1, "Controller did not send ACK");

    // SEND_ACK -> RECV_DATA_2
    byte_loaded_in_DR_sig = 1;
    run_cycle(1);
    byte_loaded_in_DR_sig = 0;
    PN_ASSERTM(sr1_rxne_sig.read() == 1, "Flag Error: RXNE not set after byte received");

    shift_loaded_sig = 1;
    run_cycle(1);
    shift_loaded_sig = 0; // -> CHECK_DR

    // CHECK_ACK_NACK
    cr1_ack_sig = 0;
    CPU_read_DR_sig = 1;
    run_cycle(1);
    CPU_read_DR_sig = 0;

    run_cycle(2);
    PN_ASSERTM(send_nack_sig.read() == 1, "Controller did not send NACK");

    // READ_FIN
    byte_loaded_in_DR_sig = 1;
    run_cycle(3);
    byte_loaded_in_DR_sig = 0;

    // STOP
    cr1_stop_sig = 1;
    run_cycle(2);

    // STOP-Finish
    stop_finished_sig = 1;
    run_cycle(1);
    stop_finished_sig = 0;

    cr1_stop_sig = 0;

    run_cycle(3);

    PN_INFO("***** Simulation done*****");
    PN_END_TRACE();
    return 0;
}