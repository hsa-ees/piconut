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

#include "i2c_controller.h"


void m_i2c_controller::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, rst);

    // CR1 / control signals
    PN_TRACE(tf, clear_regs_out);
    PN_TRACE(tf, clear_cr1_start_out);
    PN_TRACE(tf, clear_cr1_stop_out);
    PN_TRACE(tf, cr1_enable_in);
    PN_TRACE(tf, cr1_start_in);
    PN_TRACE(tf, cr1_stop_in);
    PN_TRACE(tf, cr1_ack_in);

    // Clock control signals
    PN_TRACE(tf, scl_ready_in);
    PN_TRACE(tf, scl_on_out);
    PN_TRACE(tf, init_master_out);
    PN_TRACE(tf, master_stretch_out);

    // Data register signals
    PN_TRACE(tf, CPU_write_DR_in);
    PN_TRACE(tf, CPU_read_DR_in);
    PN_TRACE(tf, dr_in);
    PN_TRACE(tf, sending_data_in);
    PN_TRACE(tf, byte_loaded_in_DR_in);

    // Status register outputs
    PN_TRACE(tf, sr1_start_bit_out);
    PN_TRACE(tf, sr1_addr_bit_out);
    PN_TRACE(tf, sr1_txe_out);
    PN_TRACE(tf, sr1_btf_out);
    PN_TRACE(tf, sr1_rxne_out);
    PN_TRACE(tf, sr1_af_out);

    PN_TRACE(tf, CPU_read_SR1_in);
    PN_TRACE(tf, CPU_read_SR2_in);

    PN_TRACE(tf, sr2_busy_out);

    // START/STOP generation
    PN_TRACE(tf, start_tx_out);
    PN_TRACE(tf, start_rx_out);
    PN_TRACE(tf, start_finished_in);
    PN_TRACE(tf, stop_finished_in);
    PN_TRACE(tf, generate_start_cond_out);
    PN_TRACE(tf, generate_stop_cond_out);

    // ACK/NACK handling
    PN_TRACE(tf, send_ack_out);
    PN_TRACE(tf, send_nack_out);
    PN_TRACE(tf, ack_received_in);
    PN_TRACE(tf, nack_received_in);

    // FSM state
    PN_TRACE(tf, state);
}

/* ========================== Combinatorial FSM ======================== */
void m_i2c_controller::proc_comb_controller()
{

    clear_regs_out.write(false);

    clear_cr1_start_out.write(false);
    clear_cr1_stop_out.write(false);

    sr1_start_bit_out.write(false);
    sr1_addr_bit_out.write(false);
    sr1_txe_out.write(false);
    sr1_btf_out.write(false);
    sr1_af_out.write(false);
    sr1_rxne_out.write(false);

    sr2_msl_out.write(false);
    sr2_busy_out.write(false);
    sr2_tra_out.write(false);

    generate_start_cond_out.write(false);
    generate_stop_cond_out.write(false);
    init_master_out.write(false);
    master_stretch_out.write(false);
    scl_on_out.write(false);
    start_tx_out.write(false);
    start_rx_out.write(false);

    send_ack_out.write(false);
    send_nack_out.write(false);

    next_state = state.read();

    switch(state.read())
    {
        case OFF:
            if(cr1_enable_in.read())
            {
                next_state = IDLE;
            }
            break;

        case IDLE:
            if(cr1_start_in.read() && !cr1_stop_in.read())
            {
                next_state = INIT_MASTER;
            }
            break;

        case INIT_MASTER:
            sr2_msl_out.write(true);
            sr2_busy_out.write(true);
            init_master_out.write(true);

            if(scl_ready_in.read())
            {
                next_state = START_COND;
            }
            break;

        case START_COND:
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            scl_on_out.write(true);
            generate_start_cond_out.write(true);

            if(start_finished_in.read())
            {
                next_state = START_FIN;
            }
            break;

        case START_FIN:
            scl_on_out.write(true);
            sr1_start_bit_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            master_stretch_out.write(true);

            if(CPU_read_SR1_in.read())
            {
                next_state = CPU_READ_SR1_WAIT_DR;
            }
            if(cr1_stop_in.read())
            {
                next_state = STOP_COND;
            }

            break;

        case CPU_READ_SR1_WAIT_DR:
            scl_on_out.write(true);
            sr1_start_bit_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            master_stretch_out.write(true);

            if(CPU_write_DR_in.read())
            {
                next_state = LOAD_ADDR;
            }
            break;

        case LOAD_ADDR:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            clear_cr1_start_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            start_tx_out.write(true);
            sr2_tra_out.write(dr_in.read() & 0x01);

            if(sending_data_in.read())
            {
                next_state = SEND_ADDR;
            }
            break;

        case SEND_ADDR:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(dr_in.read() & 0x01);

            if(ack_received_in.read())
            {
                next_state = ADDR_FIN;
            }
            else if(nack_received_in.read())
            {
                next_state = ACK_ERROR;
            }
            break;

        case ADDR_FIN:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr1_addr_bit_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(dr_in.read() & 0x01);

            if(CPU_read_SR1_in.read())
            {
                next_state = CPU_READ_SR1_WAIT_SR2;
            }
            break;

        case CPU_READ_SR1_WAIT_SR2:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr1_addr_bit_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(dr_in.read() & 0x01);

            if(CPU_read_SR2_in.read())
            {
                next_state = READY_RW;
            }
            break;

        case READY_RW:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(dr_in.read() & 0x01);

            if(dr_in.read() & 0x01)
            {
                next_state = RECV_DATA_1;
            }
            else if(!(dr_in.read() & 0x01) && CPU_write_DR_in.read())
            {
                next_state = LOAD_SHIFT_REG;
            }
            else if(cr1_stop_in.read() && !cr1_start_in.read())
            {
                next_state = STOP_COND;
            }

            break;

        case LOAD_SHIFT_REG:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            start_tx_out.write(true);

            if(shift_loaded_in.read())
            {
                next_state = LOAD_SHIFT_REG_2;
            }
            break;

        case LOAD_SHIFT_REG_2:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr1_txe_out.write(true);

            if(sending_data_in.read())
            {
                next_state = SEND_DATA_1;
            }
            break;

        case SEND_DATA_1:
            scl_on_out.write(true);
            sr1_txe_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);

            if(CPU_write_DR_in.read())
            {
                next_state = SEND_DATA_2;
            }
            else if(ack_received_in.read())
            {
                next_state = WRITE_FIN;
            }
            else if(nack_received_in.read())
            {
                next_state = ACK_ERROR;
            }
            break;

        case SEND_DATA_2:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);

            if(ack_received_in.read())
            {
                next_state = LOAD_SHIFT_REG;
            }
            else if(nack_received_in.read())
            {
                next_state = ACK_ERROR;
            }

            break;

        case WRITE_FIN:
            master_stretch_out.write(true);
            scl_on_out.write(true);
            sr1_btf_out.write(true);
            sr1_txe_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);

            if(CPU_write_DR_in.read())
            {
                next_state = LOAD_SHIFT_REG;
            }
            else if(cr1_stop_in.read() && !cr1_start_in.read())
            {
                next_state = STOP_COND;
            }
            break;

        case RECV_DATA_1:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);
            start_rx_out.write(true);

            if(shift_loaded_in.read())
            {
                next_state = CHECK_ACK_NACK;
            }

            break;

        case RECV_DATA_2:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);
            sr1_rxne_out.write(true);
            start_rx_out.write(true);

            if(CPU_read_DR_in.read())
            {
                next_state = RECV_DATA_1;
            }

            if(shift_loaded_in.read())
            {
                next_state = CHECK_DR;
            }
            break;

        case CHECK_DR:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);

            sr1_rxne_out.write(true);
            master_stretch_out.write(true);

            if(CPU_read_DR_in.read())
            {
                next_state = CHECK_ACK_NACK;
            }
            break;

        case CHECK_ACK_NACK:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);

            if(cr1_ack_in.read())
            {
                next_state = SEND_ACK;
            }
            else
            {
                next_state = SEND_NACK;
            }
            break;

        case SEND_ACK:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);
            send_ack_out.write(true);

            if(byte_loaded_in_DR_in.read())
            {
                next_state = RECV_DATA_2;
            }
            break;

        case SEND_NACK:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);
            send_nack_out.write(true);

            if(byte_loaded_in_DR_in.read())
            {
                next_state = READ_FIN;
            }
            break;

        case READ_FIN:
            scl_on_out.write(true);
            sr2_busy_out.write(true);
            sr2_msl_out.write(true);
            sr2_tra_out.write(true);
            master_stretch_out.write(true);
            sr1_rxne_out.write(true);

            if(cr1_stop_in.read() && !cr1_start_in.read())
            {
                next_state = STOP_COND;
            }

            break;

        case STOP_COND:
            scl_on_out.write(true);
            generate_stop_cond_out.write(true);

            if(stop_finished_in.read())
            {
                next_state = STOP_FIN;
            }
            break;

        case STOP_FIN:
            clear_cr1_stop_out.write(true);
            next_state = IDLE;
            break;

        case ACK_ERROR:
            scl_on_out.write(true);
            sr1_af_out.write(true);

            if(cr1_stop_in.read() && !cr1_start_in.read())
            {
                next_state = STOP_COND;
            }
            break;

        case ERROR:
            break;

        case DE_INIT:
            clear_regs_out.write(true);
            next_state = OFF;
            break;

        default:
            next_state = ERROR;
            break;
    }
}

void m_i2c_controller::proc_clk_controller()
{
    // Initialzustand
    state = OFF;

    while(true)
    {
        wait();
        if(!cr1_enable_in.read())
        {
            if(state.read() != OFF && state.read() != DE_INIT)
            {
                state = DE_INIT;
            }
            else
            {
                state = next_state.read();
            }
        }
        else
        {
            state = next_state.read();
        }
    }
}
