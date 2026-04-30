/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                     Johannes Hofmann <joahnnes.hofmann1@tha.de>
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

#ifndef __TOP_H__
#define __TOP_H__

#include <systemc.h>
#include <piconut.h>

// PicoNut modules used ...
#include <pn_interconnect.h>
#include <cpu.h>
#include <uart.h>
#include <gpio.h>
#include <i2c.h>

SC_MODULE(m_refdesign_demonstrator)
{
public:
    // Ports ...
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<bool> PN_NAME(rx_i);
    sc_out<bool> PN_NAME(tx_o);

    sc_in<bool> PN_NAME(gpio_up);
    sc_in<bool> PN_NAME(gpio_down);
    sc_in<bool> PN_NAME(gpio_left);
    sc_in<bool> PN_NAME(gpio_right);
    sc_in<bool> PN_NAME(gpio_fire);

    sc_in<bool> PN_NAME(scl_i);
    sc_out<bool> PN_NAME(scl_o);
    sc_out<bool> PN_NAME(scl_oe);

    sc_in<bool> PN_NAME(sda_i);
    sc_out<bool> PN_NAME(sda_o);
    sc_out<bool> PN_NAME(sda_oe);

    // Constructor/Destructors
    SC_CTOR(m_refdesign_demonstrator)
    {
        SC_METHOD(proc_cmb);
        sensitive << gpio_up << gpio_down << gpio_left << gpio_right << gpio_fire;

        init_submodules();
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_cmb();

    // Submodules
    m_pn_interconnect* pn_interconnect;
    m_cpu* cpu;
    m_uart* uart;
    m_gpio* gpio;
    m_i2c* i2c;

protected:
    // Internal Signals
    sc_signal<bool> PN_NAME(dummy_low);
    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(gpio_input);
    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(gpio_output);

    // Methods
    void init_submodules();
};

#endif // __TOP_H__
