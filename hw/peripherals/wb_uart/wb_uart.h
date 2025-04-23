/**
 * @file wb_uart.h
 */
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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
 * @fn SC_MODULE(m_wb_uart)
 * @author Lukas Bauer
 *
 * following submodules:
 * - `baudgen_x16`: Baud rate generator for the 16x baud rate
 * - `baudgen_rx`: Baud rate generator for the receive (rx) baud rate
 * - `baudgen_tx`: Baud rate generator for the transmit (tx) baud rate
 * - `uart_rx`: Receiver module
 * - `uart_tx`: Transmitter module
 * - `uart_rx_fifo`: Receiver FIFO
 * - `uart_tx_fifo`: Transmitter FIFO
 *
 * It connects the submodules and handles the Wishbone interface.
 * It provides the registers as specified for the SiFive UART.
 * The registers are:
 * - `txdata`: Transmit data register
 * - `rxdata`: Receive data register
 * - `txctrl`: Transmit control register
 * - `rxctrl`: Receive control register
 * - `ie`: Interrupt enable register
 * - `ip`: Interrupt pending register
 * - `div`: Baud rate divisor register
 *
 * The module provides control functionality for the FIFOs, receiver, and transmitter.
 * It initiates UART transmissions when data is written to the FIFOs,
 * or writes the received data to the `rx_fifo`. It also controls whether the
 * receiver or transmitter are enabled and at what baud rate they operate.
 * Interrupts are also activated and deactivated by this module.
 * A FIFO state machine handles Wishbone access to both FIFOs to prevent invalid
 * transactions of data to and from the UART module.
 *
 * There are two configuration options in the `config.mk` file:
 * - `CFG_WB_UART_DISABLE_FIFO`: This disables the FIFOs of the UART module.
 *   In this mode, only the `rx` and `tx` data registers are usable, meaning the `rxdata_empty`
 *   and `txdata_full` flags are set when the registers are in use.
 *   This also changes the behavior of the interrupts, where the `txctrl_txcnt` and `rxctrl_rxcnt`
 *   watermark levels have no effect. The watermark levels are set to 1, and the interrupts are
 *   triggered when the `txdata` and `rxdata` registers exceed the threshold as described in the
 *   SiFive UART specification.
 * - `CFG_WB_UART_BASE_ADDRESS`: This sets the base address of the UART module in the Wishbone bus
 *   address space.
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] stb_i wb strobe
 * @param[in] cyc_i wb cycle
 * @param[in] we_i wb write enable
 * @param[in] sel_i wb byte select
 * @param[out] ack_o wb acknowledge
 * @param[out] err_o wb error
 * @param[out] rty_o wb retry
 * @param[in] addr_i wb address
 * @param[in] dat_i wb data in
 * @param[out] dat_o wb data out
 * @param[in] rx rx line
 * @param[out] tx tx line
 *
 */

#ifndef __WB_UART_H__
#define __WB_UART_H__



#include <systemc.h>
#include <base.h> // contains all PN_<> Macros and is part of the PicoNut
#include <piconut-config.h>
#include <elab_alloc.h>
#include "baudgen.h"
#include "uart_rx.h"
#include "uart_tx.h"
#include "uart_fifo.h"

// #define CFG_WB_UART_DISABLE_FIFO 1
typedef enum
{
        WB_UART_REG_TXDATA = 0,
        WB_UART_REG_RXDATA = 0x4,
        WB_UART_REG_TXCTRL = 0x8,
        WB_UART_REG_RXCTRL = 0xC,
        WB_UART_REG_IE = 0x10,
        WB_UART_REG_IP = 0x14,
        WB_UART_REG_DIV = 0x18
} e_wb_uart_registers;

typedef enum
{
        WB_UART_WB_IDLE = 0,
        WB_UART_WB_READ,
        WB_UART_WB_READ_WAIT,
        WB_UART_WB_WRITE,
        WB_UART_WB_WRITE_WAIT,
} e_uart_wb_states;

typedef enum
{
        WB_UART_TOP_FIFO_IDLE = 0,
        WB_UART_TOP_FIFO_RX_ACK,
        WB_UART_TOP_FIFO_RX_READ,
        WB_UART_TOP_FIFO_TX_WRITE,
        WB_UART_TOP_FIFO_TX_ACK,
} e_uart_wb_fifo_states;

typedef enum
{
        WB_UART_TOP_TX_IDLE = 0,
        WB_UART_TOP_TX_START,
        WB_UART_TOP_TX_WAIT,
        WB_UART_TOP_TX_RUN,
} e_uart_tx_states;

typedef enum
{
        WB_UART_TOP_RX_IDLE = 0,
        WB_UART_TOP_RX_WAIT1,
        WB_UART_TOP_RX_SAVE,
        WB_UART_TOP_RX_WAIT2
} e_uart_rx_states;

SC_MODULE(m_wb_uart)
{
public:
        /** Ports ...
         * this are the two necessary signals
         * you may add your own signals here */
        sc_in_clk PN_NAME(clk);     // Clock signal of the module
        sc_in<bool> PN_NAME(reset); // Reset signal of the module

        // wishbone signals
        sc_in<bool> PN_NAME(stb_i);       // wb strobe
        sc_in<bool> PN_NAME(cyc_i);       // wb cycle
        sc_in<bool> PN_NAME(we_i);        // wb write enable
        sc_in<sc_uint<4>> PN_NAME(sel_i); // wb byte select
        sc_out<bool> PN_NAME(ack_o);      // wb acknowledge
        sc_out<bool> PN_NAME(err_o);      // wb error
        sc_out<bool> PN_NAME(rty_o);      // wb retry

        sc_in<sc_uint<32>> PN_NAME(addr_i); // wb address
        sc_in<sc_uint<32>> PN_NAME(dat_i);  // wb data in
        sc_out<sc_uint<32>> PN_NAME(dat_o); // wb data out

        // UART signals
        sc_in<bool> PN_NAME(rx);  // rx line
        sc_out<bool> PN_NAME(tx); // tx line

        /* Constructor... */
        SC_CTOR(m_wb_uart)
        {

                SC_METHOD(proc_comb_main);
                sensitive << txctrl_txen << rxctrl_rxen << div_div << txctrl_nstop << txdata_data
                          << rx_finished << rx_baudtick_disable << cyc_i << we_i;
#if !CFG_WB_UART_DISABLE_FIFO
                sensitive << tx_fifo_full << rx_fifo_empty;
#else
                sensitive << tx_reg_full << rx_reg_empty;
#endif
                SC_CTHREAD(proc_clk_wb, clk.pos());
                reset_signal_is(reset, true);
                SC_METHOD(proc_comb_wb);
                sensitive << cyc_i << stb_i << wb_ack << fifo_ack;
                SC_CTHREAD(proc_clk_tx, clk.pos());
                reset_signal_is(reset, true);
                SC_METHOD(proc_comb_tx);
                sensitive << txctrl_txen << tx_state << tx_finished << tx_data;
#if !CFG_WB_UART_DISABLE_FIFO
                sensitive << tx_fifo_empty << tx_fifo_data_out;
#else
                sensitive << tx_reg_full << txdata_data;
#endif
                SC_CTHREAD(proc_clk_rx, clk.pos());
                reset_signal_is(reset, true);
                SC_METHOD(proc_comb_rx);
                sensitive << rx_state << rxctrl_rxen << rxctrl_rxcnt << rx_finished << rx_data;
#if !CFG_WB_UART_DISABLE_FIFO
                sensitive << rx_fifo_full;
#else
                sensitive << rx_reg_empty;
#endif
                SC_METHOD(proc_comb_interupt);
                sensitive << ie_rxwm << ie_txwm << txctrl_txcnt << rxctrl_rxcnt;
#if !CFG_WB_UART_DISABLE_FIFO
                sensitive << rx_fifo_usage << tx_fifo_usage;
#else
                sensitive << rx_reg_empty << tx_reg_full;
#endif
                SC_METHOD(proc_comb_fifo);
                sensitive << cyc_i << stb_i << addr_i << we_i << fifo_state << wb_ack << sel_i;
#if !CFG_WB_UART_DISABLE_FIFO
                sensitive << tx_fifo_full << rx_fifo_empty << tx_fifo_read << rx_fifo_write;
#else
                sensitive << tx_reg_full << rx_reg_empty;
#endif
                SC_CTHREAD(proc_clk_fifo, clk.pos());
                reset_signal_is(reset, true);

                init_submodules();
        }

        /* Functions...*/
        /**
         * @brief this function is used to generate a tracefile
         * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
         * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
         * @param tf this is the tracefile object
         * @param level is used as a selector when to enable a trace*/
        void Trace(sc_trace_file * tf, int level = 1);

        // Helpers
        inline bool get_baudtick() { return baudtick_tx.read(); }

        /** Processes...
         */

        void proc_comb_main();
        void proc_clk_wb();
        void proc_comb_wb();
        void proc_comb_tx();
        void proc_clk_tx();
        void proc_comb_rx();
        void proc_clk_rx();
        void proc_comb_interupt();
        void proc_comb_fifo();
        void proc_clk_fifo();

        // Submodules

        m_baudgen *baudgen_x16;
        m_baudgen *baudgen_rx;
        m_baudgen *baudgen_tx;
        m_uart_rx *uart_rx;
        m_uart_tx *uart_tx;
#if !CFG_WB_UART_DISABLE_FIFO
        m_uart_fifo *uart_rx_fifo;
        m_uart_fifo *uart_tx_fifo;
#endif

        // Methods
        void init_submodules();

protected:
        /** Registers...
         * This are examples of Module Registers
         * you may add your own Registers here*/

        // Connection signals
        // Baudgen Signals
        sc_signal<bool> PN_NAME(baudgen_tx_en);         // enable for baudgen tx
        sc_signal<bool> PN_NAME(baudgen_tx_clear);      // clearing the baudgen tx (synchronous reset)
        sc_signal<sc_uint<16>> PN_NAME(baudgen_tx_div); // divider value for baudgen tx
        sc_signal<bool> PN_NAME(baudtick_tx);           // generated baudtick for tx

        sc_signal<bool> PN_NAME(baudgen_rx_en);         // enable for baudgen rx
        sc_signal<bool> PN_NAME(baudgen_rx_clear);      // clearing the baudgen rx (synchronous reset)
        sc_signal<sc_uint<16>> PN_NAME(baudgen_rx_div); // divider value for baudgen rx
        sc_signal<bool> PN_NAME(baudtick_rx);           // generated baudtick for rx

        sc_signal<bool> PN_NAME(baudgen_x16_en);         // enable for baudgen x16
        sc_signal<bool> PN_NAME(baudgen_x16_clear);      // clearing the baudgen x16 (synchronous reset)
        sc_signal<sc_uint<16>> PN_NAME(baudgen_x16_div); // divider value for baudgen x16
        sc_signal<bool> PN_NAME(baudtick_x16);           // generated baudtick for x16

        // RX Signals
        sc_signal<bool> PN_NAME(rx_internal);         // internal rx line signal
        sc_signal<sc_uint<8>> PN_NAME(rx_data);       // received data
        sc_signal<bool> PN_NAME(rx_finished);         // rx finished
        sc_signal<bool> PN_NAME(rx_stopbit_8_9);      // number of stop bits 9 if true, 8 if false
        sc_signal<bool> PN_NAME(rx_baudtick_disable); // disable the baudtick generation if needed

        // TX Signals
        sc_signal<bool> PN_NAME(tx_start);       // start signal for transmission
        sc_signal<sc_uint<8>> PN_NAME(tx_data);  // 8-bit word to transmit
        sc_signal<bool> PN_NAME(tx_finished);    // transmission finished signal
        sc_signal<bool> PN_NAME(tx_internal);    // internal tx line signal
        sc_signal<bool> PN_NAME(tx_stopbit_8_9); // number of stop bits 9 if true, 8 if false

#if !CFG_WB_UART_DISABLE_FIFO
        // RX FIFO Signals
        sc_signal<bool> PN_NAME(rx_fifo_clear);                           // clearing the rx fifo
        sc_signal<bool> PN_NAME(rx_fifo_write);                           // writing to rx fifo
        sc_signal<bool> PN_NAME(rx_fifo_read);                            // reading from rx fifo
        sc_signal<bool> PN_NAME(rx_fifo_full);                            // rx fifo full
        sc_signal<bool> PN_NAME(rx_fifo_empty);                           // rx fifo empty
        sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>> PN_NAME(rx_fifo_usage); // usage of the rx fifo
        sc_signal<sc_uint<8>> PN_NAME(rx_fifo_data_in);                   // data to be written to the rx fifo
        sc_signal<sc_uint<8>> PN_NAME(rx_fifo_data_out);                  // data to be read from the rx fifo

        // TX FIFO Signals
        sc_signal<bool> PN_NAME(tx_fifo_clear);                           // clearing the tx fifo
        sc_signal<bool> PN_NAME(tx_fifo_write);                           // writing to tx fifo
        sc_signal<bool> PN_NAME(tx_fifo_read);                            // reading from tx fifo
        sc_signal<bool> PN_NAME(tx_fifo_full);                            // tx fifo full
        sc_signal<bool> PN_NAME(tx_fifo_empty);                           // tx fifo empty
        sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>> PN_NAME(tx_fifo_usage); // usage of the tx fifo
        sc_signal<sc_uint<UART_FIFO_WIDTH>> PN_NAME(tx_fifo_data_in);     // data to be written to the tx fifo
        sc_signal<sc_uint<UART_FIFO_WIDTH>> PN_NAME(tx_fifo_data_out);    // data to be read from the tx fifo
#else
        sc_signal<bool> PN_NAME(rx_reg_empty);       // rx register empty
        sc_signal<bool> PN_NAME(tx_reg_full);        // tx register full
        sc_signal<bool> PN_NAME(c_set_rx_reg_empty); // set rx register empty flag
        sc_signal<bool> PN_NAME(c_set_tx_reg_full);  // set tx register full flag
        sc_signal<bool> PN_NAME(c_clr_rx_reg_empty); // clear rx register empty flag
        sc_signal<bool> PN_NAME(c_clr_tx_reg_full);  // clear tx register full flag
        sc_signal<bool> PN_NAME(c_save_rxdata_data); // save rxdata_data register
#endif

        // registers
        // wb registers
        sc_signal<bool> PN_NAME(wb_ack); // acknowledge signal

        // tx state machine registers
        sc_signal<sc_uint<2>> PN_NAME(tx_state);      // state of the tx state machine
        sc_signal<sc_uint<2>> PN_NAME(tx_next_state); // next state of the tx state machine
        sc_signal<sc_uint<8>> PN_NAME(tx_data_reg);   // data to be transmitted

        // rx state machine registers
        sc_signal<sc_uint<2>> PN_NAME(rx_state);      // state of the rx state machine
        sc_signal<sc_uint<2>> PN_NAME(rx_next_state); // next state of the rx state machine

        // fifo state machine registers
        sc_signal<sc_uint<2>> PN_NAME(fifo_state);      // state of the fifo state machine
        sc_signal<sc_uint<2>> PN_NAME(fifo_next_state); // next state of the fifo state machine
        sc_signal<bool> PN_NAME(fifo_ack);              // acknowledge signal for the fifo state machine

        // txdata register fields
        sc_signal<sc_uint<8>> PN_NAME(txdata_data); // txdata[7:0]:     contains the data to be transmitted
        sc_signal<bool> PN_NAME(txdata_full);       // txdata[31]:      indicates if the tx fifo is full

        // rxdata register fields
        sc_signal<sc_uint<8>> PN_NAME(rxdata_data); // rxdata[7:0]:     contains the received data
        sc_signal<bool> PN_NAME(rxdata_empty);      // rxdata[31]:      indicates if the rx fifo is empty

        // txctrl register fields
        sc_signal<bool> PN_NAME(txctrl_txen);        // txctrl[0]:       enables the transmitter
        sc_signal<bool> PN_NAME(txctrl_nstop);       // txctrl[1]:       number of stop bits
        sc_signal<sc_uint<3>> PN_NAME(txctrl_txcnt); // txctrl[18:16]:   transmit watermark level

        // rxctrl register fields
        sc_signal<bool> PN_NAME(rxctrl_rxen);        // rxctrl[0]:       enables the receiver
        sc_signal<sc_uint<3>> PN_NAME(rxctrl_rxcnt); // rxctrl[18:16]:   receive watermark level

        // ie register fields
        sc_signal<bool> PN_NAME(ie_txwm); // ie[0]:           tx fifo watermark interrupt enable
        sc_signal<bool> PN_NAME(ie_rxwm); // ie[1]:           rx fifo watermark interrupt enable

        // ip register fields
        sc_signal<bool> PN_NAME(ip_txwm); // ip[0]:           tx fifo watermark interrupt pending
        sc_signal<bool> PN_NAME(ip_rxwm); // ip[1]:           rx fifo watermark interrupt pending

        // div register fields
        sc_signal<sc_uint<16>> PN_NAME(div_div); // div[15:0]:       baud rate divisor
};
#endif // __WB_UART_H__