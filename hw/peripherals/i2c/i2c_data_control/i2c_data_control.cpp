/**
 * @file i2c_data_control.cpp
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

#include "i2c_data_control.h"

m_i2c_data_control::m_i2c_data_control(sc_core::sc_module_name name)
    : sc_module(name)
{
    SC_METHOD(proc_comb_data_control);
    sensitive << state
              << shift_reg
              << bit_cnt
              << generate_start_cond_in
              << stop_ready_in
              << start_tx_in
              << start_rx_in
              << tx_data_in
              << send_ack_in
              << send_nack_in
              << data_clk_pulse_write_in
              << data_clk_pulse_read_in
              << sda_in
              << shift_reg_read;

    SC_CTHREAD(proc_clk_data_control, clk.pos());
    reset_signal_is(reset, true);

    SC_CTHREAD(proc_clk_shift, clk.pos());
    reset_signal_is(reset, true);
}

void m_i2c_data_control::pn_trace(sc_trace_file* tf, int level)
{
    // System-Ports
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // controller
    PN_TRACE(tf, generate_start_cond_in);
    PN_TRACE(tf, start_tx_in);
    PN_TRACE(tf, start_rx_in);
    PN_TRACE(tf, send_ack_in);
    PN_TRACE(tf, send_nack_in);
    PN_TRACE(tf, tx_data_in);
    PN_TRACE(tf, start_ready_out);
    PN_TRACE(tf, shift_loaded_out);
    PN_TRACE(tf, ack_received_out);
    PN_TRACE(tf, nack_received_out);
    PN_TRACE(tf, byte_loaded_in_DR_out);
    PN_TRACE(tf, sending_data_out);
    PN_TRACE(tf, shift_data_out);

    // clk_control
    PN_TRACE(tf, data_clk_pulse_write_in);
    PN_TRACE(tf, data_clk_pulse_read_in);
    PN_TRACE(tf, stop_ready_in);

    // wb
    PN_TRACE(tf, enable_dr_write_out);

    // data_control
    PN_TRACE(tf, sda_in);
    PN_TRACE(tf, sda_out);

    // internal
    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);
    PN_TRACE(tf, shift_reg);
    PN_TRACE(tf, shift_reg_read);
    PN_TRACE(tf, shift_enable_read);
    PN_TRACE(tf, bit_cnt);
    PN_TRACE(tf, next_bit_cnt);
    PN_TRACE(tf, next_shift_reg);
    PN_TRACE(tf, byte_loaded_in_DR_out);

}

void m_i2c_data_control::proc_comb_data_control()
{

    next_state = state.read();
    next_shift_reg = shift_reg.read();
    next_bit_cnt = bit_cnt.read();

    ack_received_out.write(false);
    nack_received_out.write(false);
    sda_out.write(true);
    start_ready_out.write(false);
    shift_loaded_out.write(false);
    shift_enable_read.write(false);
    enable_dr_write_out.write(false);
    shift_data_out.write(false);
    sending_data_out.write(false);
    byte_loaded_in_DR_out.write(false);

    switch(state.read())
    {

        case IDLE:

            if(generate_start_cond_in.read())
            {
                next_state = START_COND;
            }
            break;

        case START_COND:

            sda_out.write(false);

            next_state = START_DONE;
            break;

        case START_DONE:

            sda_out.write(false);
            start_ready_out.write(true);

            if(start_tx_in.read())
            {
                next_state = LOAD_DATA;
            }
            break;

        case LOAD_DATA:

            sda_out.write(false);
            next_shift_reg = tx_data_in.read();
            next_bit_cnt = 7;

            next_state = LOAD_DONE;
            break;

        case LOAD_DONE:

            sda_out.write(false);
            shift_loaded_out.write(true);

            next_state = LOAD_BIT;
            break;

        case LOAD_BIT:

            sending_data_out.write(true);

            if(shift_reg.read()[7] == 0)
            {
                sda_out.write(false);
            }
            
            if(data_clk_pulse_write_in.read())
            {
                next_state = HOLD_BIT;
            }
            break;

        case HOLD_BIT:

            sending_data_out.write(true);

            if(shift_reg.read()[7] == 0)
            {
                sda_out.write(false);
            }

            if(bit_cnt.read() == 0)
            {
                next_state = HOLD_BIT_BEFORE_ACK;
            }
            else if(data_clk_pulse_write_in.read())
            {
                next_bit_cnt = bit_cnt.read() - 1;
                next_shift_reg = shift_reg.read() << 1;
                next_state = LOAD_BIT;
            }
            break;

        case HOLD_BIT_BEFORE_ACK:

            sending_data_out.write(true);

            if(shift_reg.read()[7] == 0)
            {
                sda_out.write(false);
            }

            if(data_clk_pulse_write_in.read())
            {
                next_state = ACK_PHASE;
            }
            break;

        case ACK_PHASE:

            sending_data_out.write(true);

            if(data_clk_pulse_read_in.read())
            {
                next_state = ACK_READ;
            }
            break;

        case ACK_READ:

            sending_data_out.write(true);

            if(sda_in.read() == false)
            {
                ack_received_out.write(true);
            }
            else
            {
                nack_received_out.write(true);
            }

            next_state = ACK_WAIT;
            break;

        case ACK_WAIT:

            sending_data_out.write(true);

            if(data_clk_pulse_read_in.read())
            {
                next_state = WAIT_CMD;
            }
            break;

        case WAIT_CMD:

            sda_out.write(false);

            if(start_tx_in.read())
            {
                next_state = LOAD_DATA;
            }
            else if(start_rx_in.read())
            {
                next_state = LOAD_CNT;
            }
            else if(stop_ready_in.read())
            {
                next_state = STOP_COND;
            }
            break;

        case LOAD_CNT:

            next_bit_cnt = 7;

            next_state = READ_WAIT_LOW;
            break;

        case READ_WAIT_LOW:

            if(data_clk_pulse_read_in.read())
            {
                next_state = READ_SHIFT;
            }
            break;

        case READ_SHIFT:

            shift_enable_read.write(true);

            if(bit_cnt.read() == 0)
            {
                next_state = ACK_NACK_WAIT_1;
            }
            else
            {
                next_bit_cnt = bit_cnt.read() - 1;
                next_state = READ_WAIT_HIGH;
            }
            break;

        case READ_WAIT_HIGH:

            if(data_clk_pulse_read_in.read())
            {
                next_state = READ_WAIT_LOW;
            }
            break;

        case ACK_NACK_WAIT_1:

            if(data_clk_pulse_write_in.read())
            {
                next_state = ACK_NACK_WAIT_2;
            }
            break;

        case ACK_NACK_WAIT_2:

            shift_loaded_out.write(true);

            if(send_ack_in.read())
            {
                next_state = ACK_SEND_1;
            }
            else if(send_nack_in.read())
            {
                next_state = NACK_SEND_1;
            }
            break;

        case ACK_SEND_1:

            sda_out.write(false);

            if(data_clk_pulse_write_in.read())
            {
                next_state = ACK_SEND_2;
            }
            break;

        case ACK_SEND_2:

            sda_out.write(false);

            if(data_clk_pulse_write_in.read())
            {
                next_state = FROM_SHIFT_TO_DR;
            }
            break;

        case NACK_SEND_1:

            if(data_clk_pulse_write_in.read())
            {
                next_state = NACK_SEND_2;
            }
            break;

        case NACK_SEND_2:

            if(data_clk_pulse_write_in.read())
            {
                next_state = FROM_SHIFT_TO_DR;
            }
            break;

        case FROM_SHIFT_TO_DR:

            shift_data_out.write(shift_reg_read.read());
            enable_dr_write_out.write(true);
            byte_loaded_in_DR_out.write(true);

            next_state = WAIT_CMD;
            break;

        case STOP_COND:

            next_state = IDLE;
            break;

        default:

            next_state = IDLE;
            break;
    }
}

void m_i2c_data_control::proc_clk_data_control()
{
    state.write(IDLE);
    shift_reg.write(0);
    bit_cnt.write(0);

    wait();
    while(true)
    {
        wait();
        state.write(next_state.read());
        shift_reg.write(next_shift_reg.read());
        bit_cnt.write(next_bit_cnt.read());
    }
}

void m_i2c_data_control::proc_clk_shift()
{

    shift_reg_read.write(0);
    
    wait();
    while(true)
    {
        wait();
        sc_uint<8> tmp = shift_reg_read.read();

        if(shift_enable_read.read())
        {
            tmp = tmp << 1;
            tmp[0] = sda_in.read();
        }

        shift_reg_read.write(tmp);
    }
}
