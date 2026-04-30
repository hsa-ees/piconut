/**
 * @file i2c_controller.h
 * @brief
 */

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
/**
 * @fn SC_MODULE(m_i2c_controller)
 *
 * This SystemC module implements the finite state machine (FSM) that
 * controls the PicoNut I2C master. It observes CR1/SR1/SR2/DR register
 * interfaces and coordinates submodules for Clock and Data Control.
 *
 *
 * @par
 *
 * @param[in] clk                 Clock input.
 * @param[in] rst                 Asynchronous reset (active high).
 *
 * @param[out] clear_regs_out     Clears controller-internal register flags.
 *
 * @param[in] cr1_enable_in       CR1.PE  – peripheral enable.
 * @param[in] cr1_start_in        CR1.START – CPU requests START.
 * @param[in] cr1_stop_in         CR1.STOP – CPU requests STOP.
 * @param[in] cr1_ack_in          CR1.ACK – ACK generation setting for RX.
 * @param[out] clear_cr1_start_out Clears CR1.START after START is executed.
 * @param[out] clear_cr1_stop_out  Clears CR1.STOP after STOP is finished.
 *
 * @param[out] sr1_start_bit_out  Controls SR1.SB  (start bit detected).
 * @param[out] sr1_addr_bit_out   Controls SR1.ADDR (address matched).
 * @param[out] sr1_txe_out        Controls SR1.TXE (data register empty).
 * @param[out] sr1_btf_out        Controls SR1.BTF (byte transfer finished).
 * @param[out] sr1_af_out         Controls SR1.AF (ack failure).
 * @param[out] sr1_rxne_out       Controls SR1.RXNE (byte received).
 * @param[in] CPU_read_SR2_in     CPU reads SR2 (clears ADDR sequence).
 * @param[in] CPU_read_SR1_in     CPU reads SR1 (used to advance states).
 *
 * @param[out] sr2_msl_out        Controls SR2.MSL (master/slave mode).
 * @param[out] sr2_busy_out       Controls SR2.BUSY (bus busy).
 * @param[out] sr2_tra_out        Controls SR2.TRA (TX/RX mode).
 *
 * @param[in]  dr_in              CPU-written data register value.
 *
 * @param[in]  scl_ready_in       SCL module ready for next command.
 * @param[out] init_master_out    Initialize master mode in clkControl.
 * @param[out] scl_on_out         Enable SCL generation.
 * @param[out] master_stretch_out Enable clock stretching from master.
 *
 * @param[in]  stop_finished_in   STOP condition finished.
 * @param[in]  start_finished_in  START condition finished.
 * @param[in]  sending_data_in    Data path is actively sending a byte.
 * @param[in]  ack_received_in    ACK received from slave.
 * @param[in]  nack_received_in   NACK received from slave.
 * @param[out] send_ack_out       Request data path to send ACK.
 * @param[out] send_nack_out      Request data path to send NACK.
 * @param[out] start_rx_out       Start a receive transfer.
 * @param[in]  byte_loaded_in_DR_in  Byte transferred into DR.
 * @param[in]  shift_loaded_in    Shift register is ready.
 * @param[out] generate_start_cond_out Generate a START on the bus.
 * @param[out] start_tx_out       Request start of TX transfer.
 * @param[out] generate_stop_cond_out  Generate a STOP on the bus.
 *
 * @param[in] CPU_write_DR_in     CPU writes a byte into DR.
 * @param[in]  CPU_read_DR_in     CPU read access to DR.
 *
 */

#ifndef __I2C_CONTROLLER_H__
#define __I2C_CONTROLLER_H__

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_i2c_controller)
{
public:
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(rst);

    sc_out<bool> PN_NAME(clear_regs_out);

    // CR1
    sc_in<bool> PN_NAME(cr1_enable_in);
    sc_in<bool> PN_NAME(cr1_start_in);
    sc_out<bool> PN_NAME(clear_cr1_start_out);
    sc_out<bool> PN_NAME(clear_cr1_stop_out);
    sc_in<bool> PN_NAME(cr1_stop_in);
    sc_in<bool> PN_NAME(cr1_ack_in);

    // SR1
    sc_out<bool> PN_NAME(sr1_start_bit_out);
    sc_out<bool> PN_NAME(sr1_addr_bit_out);
    sc_out<bool> PN_NAME(sr1_txe_out);
    sc_out<bool> PN_NAME(sr1_btf_out);
    sc_out<bool> PN_NAME(sr1_af_out);
    sc_out<bool> PN_NAME(sr1_rxne_out);
    sc_in<bool> PN_NAME(CPU_read_SR2_in);
    sc_in<bool> PN_NAME(CPU_read_SR1_in);

    // SR2
    sc_out<bool> PN_NAME(sr2_msl_out);
    sc_out<bool> PN_NAME(sr2_busy_out);
    sc_out<bool> PN_NAME(sr2_tra_out);

    // DR
    sc_in<sc_uint<8>> PN_NAME(dr_in);

    // clkControl
    sc_in<bool> PN_NAME(scl_ready_in);
    sc_out<bool> PN_NAME(init_master_out);
    sc_out<bool> PN_NAME(scl_on_out);
    sc_out<bool> PN_NAME(master_stretch_out);
    sc_in<bool> PN_NAME(stop_finished_in);

    // dataControl
    sc_in<bool> PN_NAME(start_finished_in);
    sc_in<bool> PN_NAME(sending_data_in);
    sc_in<bool> PN_NAME(ack_received_in);
    sc_in<bool> PN_NAME(nack_received_in);
    sc_out<bool> PN_NAME(send_ack_out);
    sc_out<bool> PN_NAME(send_nack_out);
    sc_out<bool> PN_NAME(start_rx_out);
    sc_in<bool> PN_NAME(byte_loaded_in_DR_in);
    sc_in<bool> PN_NAME(shift_loaded_in);
    sc_out<bool> PN_NAME(generate_start_cond_out);
    sc_out<bool> PN_NAME(start_tx_out);
    sc_out<bool> PN_NAME(generate_stop_cond_out);

    // WB Master
    sc_in<bool> PN_NAME(CPU_write_DR_in);
    sc_in<bool> PN_NAME(CPU_read_DR_in);

    /* Constructor... */
    SC_CTOR(m_i2c_controller)
    {
        SC_CTHREAD(proc_clk_controller, clk.pos());
        reset_signal_is(rst, true);

        SC_METHOD(proc_comb_controller);
        sensitive << state
                  << cr1_enable_in
                  << cr1_start_in
                  << scl_ready_in
                  << CPU_write_DR_in
                  << stop_finished_in
                  << dr_in
                  << CPU_read_SR1_in
                  << CPU_read_SR2_in
                  << ack_received_in
                  << nack_received_in
                  << CPU_read_SR2_in
                  << shift_loaded_in
                  << cr1_stop_in
                  << start_finished_in
                  << sending_data_in
                  << CPU_read_DR_in
                  << cr1_ack_in
                  << byte_loaded_in_DR_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /**
     * @brief Combinational part of the controller FSM.
     */
    void proc_comb_controller();

    /**
     * @brief Clocked part of the controller FSM.
     *
     */
    void proc_clk_controller();

protected:
    typedef enum
    {
        OFF,
        IDLE,
        INIT_MASTER,
        START_COND,
        START_FIN,
        CPU_READ_SR1_WAIT_DR,
        LOAD_ADDR,
        SEND_ADDR,
        ADDR_FIN,
        CPU_READ_SR1_WAIT_SR2,
        READY_RW,
        LOAD_SHIFT_REG,
        LOAD_SHIFT_REG_2,
        SEND_DATA_1,
        SEND_DATA_2,
        WRITE_FIN,
        RECV_DATA_1,
        RECV_DATA_2,
        CHECK_DR,
        CHECK_ACK_NACK,
        SEND_ACK,
        SEND_NACK,
        READ_FIN,
        STOP_COND,
        STOP_FIN,
        ACK_ERROR,
        ERROR,
        DE_INIT
    } m_controller_states;

    /// Width of the state signal in bits.
    static constexpr unsigned int state_width = 6;

    sc_signal<sc_uint<state_width>> PN_NAME(state);
    sc_signal<sc_uint<state_width>> PN_NAME(next_state);
};

#endif // __I2C_CONTROLLER_H__
