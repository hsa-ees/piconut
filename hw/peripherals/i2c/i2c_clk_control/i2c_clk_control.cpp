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

#include "i2c_clk_control.h"

void m_i2c_clk_control::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, ccr_in);
    PN_TRACE(tf, trise_in);
    PN_TRACE(tf, scl_on_in);
    PN_TRACE(tf, master_stretch_in);
    PN_TRACE(tf, scl_in);
    PN_TRACE(tf, scl_out);
    PN_TRACE(tf, data_clk_pulse_write_out);
    PN_TRACE(tf, data_clk_pulse_read_out);
    PN_TRACE(tf, scl_ready_out);
    PN_TRACE(tf, init_master_in);
    PN_TRACE(tf, generate_stop_in);
    PN_TRACE(tf, start_finished_out);
    PN_TRACE(tf, stop_ready_out);
    PN_TRACE(tf, start_ready_in);
    PN_TRACE(tf, stop_finished_out);

    PN_TRACE(tf, state);
    PN_TRACE(tf, counter);
}

/* ========================== Combinatorial FSM ======================== */
void m_i2c_clk_control::proc_cmb_clk_control()
{

    scl_out.write(true);
    data_clk_pulse_write_out.write(false);
    data_clk_pulse_read_out.write(false);
    scl_ready_out.write(false);
    start_finished_out.write(false);
    stop_ready_out.write(false);
    stop_finished_out.write(false);

    next_state = state.read();
    next_counter.write(counter.read());
    next_low_period.write(low_period.read());
    next_high_period.write(high_period.read());
    next_trise_delay.write(trise_delay.read());
    next_start_hold_period.write(start_hold_period.read());
    next_stop_hold_period.write(stop_hold_period.read());

    switch(state.read())
    {
        case STATE_IDLE:
            next_counter.write(0);
            if(init_master_in.read())
            {
                next_state = STATE_INIT_MASTER;
            }
            break;

        case STATE_INIT_MASTER: {
            sc_uint<12> ccr_val = ccr_in.read() & 0xFFF;
            bool fs_mode = (ccr_in.read() >> 15) & 0x1;
            bool duty_mode = (ccr_in.read() >> 14) & 0x1;

            next_trise_delay.write(static_cast<sc_uint<16>>(trise_in.read()));

            if(!fs_mode)
            {
                next_low_period.write(ccr_val);
                next_high_period.write(ccr_val);
                next_start_hold_period.write(ccr_val);
                next_stop_hold_period.write(ccr_val);
            }
            else
            {
                if(!duty_mode)
                {
                    next_low_period.write(ccr_val * 2);
                    next_high_period.write(ccr_val);
                    next_start_hold_period.write(ccr_val);
                    next_stop_hold_period.write(ccr_val);
                }
                else
                {
                    next_low_period.write(ccr_val * 16);
                    next_high_period.write(ccr_val * 9);
                    next_start_hold_period.write(ccr_val * 6);
                    next_stop_hold_period.write(ccr_val * 6);
                }
            }

            scl_ready_out.write(true);

            if(scl_on_in.read() && start_ready_in.read())
            {
                next_state = STATE_SCL_START_HOLD;
                next_counter.write(0);
            }
            break;
        }

        case STATE_SCL_START_HOLD:

            if(counter.read() >= (start_hold_period.read() - 1))
            {
                next_state = STATE_START_FINISH;
                next_counter.write(0);
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }
            break;

        case STATE_START_FINISH:

            next_state = STATE_SCL_START_LOW;
            break;

        case STATE_SCL_START_LOW:
            scl_out.write(false);
            start_finished_out.write(true);
            data_clk_pulse_write_out.write(true);
            data_clk_pulse_read_out.write(true);
            next_counter.write(0);
            next_state = STATE_SCL_LOW;
            break;

        case STATE_SCL_LOW:
            scl_out.write(false);

            if(counter.read() == (low_period.read() - 21))
            {
                next_state = STATE_CHECK_MASTER_STRETCH;
                next_counter.write(0);
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }
            break;

        case STATE_CHECK_MASTER_STRETCH:
            scl_out.write(false);

            if(!master_stretch_in.read())
            {
                next_state = STATE_SAFETY_WAIT;
            }

            break;

        case STATE_SAFETY_WAIT:
            scl_out.write(false);

            if(counter.read() >= 20)
            {
                if(generate_stop_in.read())
                {
                    next_state = STATE_SCL_WAIT_TRISE;
                    next_counter.write(0);
                }
                else
                {
                    next_state = STATE_SCL_START_HIGH;
                    next_counter.write(0);
                }
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }

            break;

        case STATE_SCL_START_HIGH:
            scl_out.write(true);
            next_counter.write(0);
            data_clk_pulse_write_out.write(true);
            next_state = STATE_SCL_WAIT_TRISE;
            break;

        case STATE_SCL_WAIT_TRISE:
            scl_out.write(true);

            if(counter.read() == (trise_delay.read() - 1))
            {
                next_state = STATE_SCL_CHECK_STRETCH;
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }
            break;

        case STATE_SCL_CHECK_STRETCH:
            scl_out.write(true);

            if(!scl_in.read())
            {
                // SCL LOW -> Slave stretchts
            }
            else
            {
                if(generate_stop_in.read())
                {
                    next_state = STATE_STOP_CON;
                    next_counter.write(0);
                }
                else
                {
                    data_clk_pulse_read_out.write(true);
                    next_counter.write(0);
                    next_state = STATE_SCL_HIGH;
                }
            }
            break;

        case STATE_SCL_HIGH:
            scl_out.write(true);

            if(scl_on_in.read() == false)
            {
                next_state = STATE_IDLE;
            }

            if(counter.read() == (high_period.read() - trise_delay.read() - 1 - 1))
            {
                next_state = STATE_SCL_START_LOW;
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }
            break;

        case STATE_STOP_CON:
            scl_out.write(true);

            if(counter.read() >= (stop_hold_period.read() - 1))
            {
                next_counter.write(0);
                next_state = STATE_STOP_READY;
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }
            break;

        case STATE_STOP_READY:
            scl_out.write(true);
            stop_ready_out.write(true);
            next_state = STATE_STOP_FINISHED_1;
            break;

        case STATE_STOP_FINISHED_1:
            scl_out.write(true);

            if(counter.read() >= (stop_hold_period.read() - 1))
            {
                next_counter.write(0);
                next_state = STATE_STOP_FINISHED_2;
            }
            else
            {
                next_counter.write(counter.read() + 1);
            }

            break;

        case STATE_STOP_FINISHED_2:
            scl_out.write(true);

            stop_finished_out.write(true);

            next_state = STATE_IDLE;
            break;
    }
}

/* ============================ Clocked FSM ============================ */
void m_i2c_clk_control::proc_clk_clk_control()
{

    state = STATE_IDLE;
    counter.write(0);
    low_period.write(0);
    high_period.write(0);
    trise_delay.write(0);
    start_hold_period.write(0);
    stop_hold_period.write(0);
    wait();

    while(true)
    {
        wait();
        state.write(next_state.read());
        counter.write(next_counter.read());
        low_period.write(next_low_period.read());
        high_period.write(next_high_period.read());
        trise_delay.write(next_trise_delay.read());
        start_hold_period.write(next_start_hold_period.read());
        stop_hold_period.write(next_stop_hold_period.read());
    }
}
