/**
 * @file i2c_data_control.h
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
 * @fn SC_MODULE(m_i2c_data_control)
 *
 * This SystemC module implements the I2C data control.
 * It handles data transfer on the SDA line, including sending and receiving bits,
 * ACK/NACK handling, and providing received bytes to the data register (DR).
 *
 * @par
 * @param[in]  clk   Clock input.
 * @param[in]  reset Reset (active high).
 *
 * @param[in]  generate_start_cond_in  Request START condition.
 * @param[in]  stop_ready_in           Request STOP condition.
 * @param[in]  start_tx_in             Start transmit of one byte.
 * @param[in]  start_rx_in             Start receive of one byte.
 * @param[in]  tx_data_in              Byte to transmit (loaded into shift register).
 * @param[in]  send_ack_in             Send ACK after receiving a byte.
 * @param[in]  send_nack_in            Send NACK after receiving a byte.
 * @param[out] shift_loaded_out        Shift register/byte phase is ready.
 * @param[out] ack_received_out        ACK detected from slave.
 * @param[out] nack_received_out       NACK detected from slave.
 * @param[out] start_ready_out         START condition finished.
 * @param[out] sending_data_out        Bits are transfered over SDA
 * @param[out] byte_loaded_in_DR_out   Received byte is available for DR.
 * @param[out] shift_data_out          Received byte forwarded to DR.
 * @param[out] enable_dr_write_out     Enable write into DR.
 *
 * @param[in]  data_clk_pulse_write_in Bit timing pulse for driving SDA (write phase).
 * @param[in]  data_clk_pulse_read_in  Bit timing pulse for sampling SDA (read phase).
 *
 * @param[in]  sda_in   SDA input.
 * @param[out] sda_out  SDA output value.
 */

#ifndef I2C_DATA_CONTROL_H
#define I2C_DATA_CONTROL_H

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_i2c_data_control)
{

public:
    // system ports
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // controller ports
    sc_in<bool> PN_NAME(generate_start_cond_in);
    sc_in<bool> PN_NAME(stop_ready_in);
    sc_in<bool> PN_NAME(start_tx_in);
    sc_in<bool> PN_NAME(start_rx_in);
    sc_in<sc_uint<8>> PN_NAME(tx_data_in);
    sc_in<bool> PN_NAME(send_ack_in);
    sc_in<bool> PN_NAME(send_nack_in);
    sc_out<bool> PN_NAME(shift_loaded_out);
    sc_out<bool> PN_NAME(ack_received_out);
    sc_out<bool> PN_NAME(nack_received_out);
    sc_out<bool> PN_NAME(start_ready_out);
    sc_out<bool> PN_NAME(sending_data_out);
    sc_out<bool> PN_NAME(byte_loaded_in_DR_out);
    sc_out<sc_uint<8>> PN_NAME(shift_data_out);
    sc_out<bool> PN_NAME(enable_dr_write_out);

    // clkControl ports
    sc_in<bool> PN_NAME(data_clk_pulse_write_in);
    sc_in<bool> PN_NAME(data_clk_pulse_read_in);

    // SDA ports
    sc_in<bool> PN_NAME(sda_in);
    sc_out<bool> PN_NAME(sda_out);

    SC_CTOR(m_i2c_data_control);

    void pn_trace(sc_trace_file * tf, int level = 1);

private:
    /**
     * @brief Enumeration of all data-control FSM states.
     *
     * The states represent:
     * - START generation and readiness.
     * - TX: loading a byte, sending bits, and reading ACK/NACK.
     * - RX: reading in bits and sending ACK/NACK.
     * - Forwarding received bytes to DR.
     * - STOP handling.
     */
    typedef enum
    {
        IDLE,
        START_COND,
        START_DONE,
        LOAD_DATA,
        LOAD_DONE,
        ACK_PHASE,
        ACK_WAIT,
        WAIT_CMD,
        LOAD_CNT,
        READ_SHIFT,
        ACK_NACK_WAIT_1,
        ACK_NACK_WAIT_2,
        ACK_SEND_1,
        NACK_SEND_1,
        FROM_SHIFT_TO_DR,
        STOP_COND,
        ACK_SEND_2,
        NACK_SEND_2,
        LOAD_BIT,
        HOLD_BIT,
        HOLD_BIT_BEFORE_ACK,
        ACK_READ,
        READ_WAIT_HIGH,
        READ_WAIT_LOW
    } m_i2c_data_control_states;

    static constexpr unsigned int state_width = 6;

    sc_signal<bool> PN_NAME(shift_enable_read);

    // ---------------- Register ----------------
    sc_signal<sc_uint<8>> PN_NAME(shift_reg);
    sc_signal<sc_uint<8>> PN_NAME(next_shift_reg);

    sc_signal<sc_uint<8>> PN_NAME(shift_reg_read);

    sc_signal<sc_uint<3>> PN_NAME(bit_cnt);
    sc_signal<sc_uint<3>> PN_NAME(next_bit_cnt);

    sc_signal<sc_uint<state_width>> PN_NAME(state);
    sc_signal<sc_uint<state_width>> PN_NAME(next_state);

    /**
     * @brief Combinational FSM: computes next_state and drives outputs.
     */
    void proc_comb_data_control();

    /**
     * @brief Clocked process updates state and registers each clock cycle.
     */
    void proc_clk_data_control();

    /**
     * @brief Clocked shift-register process for receiving bits from SDA.
     */
    void proc_clk_shift();
};

#endif // I2C_DATA_CONTROL_H