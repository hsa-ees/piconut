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

#include "uart.h"

#include "baudgen.h"
#include "uart_rx.h"
#include "uart_tx.h"
#include "uart_fifo.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_uart::pn_trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_slave.stb_i);
    PN_TRACE(tf, wb_slave.cyc_i);
    PN_TRACE(tf, wb_slave.we_i);
    PN_TRACE(tf, wb_slave.sel_i);
    PN_TRACE(tf, wb_slave.ack_o);
    PN_TRACE(tf, wb_slave.adr_i);
    PN_TRACE(tf, wb_slave.dat_i);
    PN_TRACE(tf, wb_slave.dat_o);
    PN_TRACE(tf, wb_slave.rty_o);
    PN_TRACE(tf, wb_slave.err_o);
    PN_TRACE(tf, rx);
    PN_TRACE(tf, tx);

    // internal signals
    if(level >= 2)
    {
        PN_TRACE(tf, txdata_data);
        PN_TRACE(tf, txdata_full);
        PN_TRACE(tf, rxdata_data);
        PN_TRACE(tf, rxdata_empty);
        PN_TRACE(tf, txctrl_txen);
        PN_TRACE(tf, txctrl_nstop);
        PN_TRACE(tf, txctrl_txcnt);
        PN_TRACE(tf, rxctrl_rxen);
        PN_TRACE(tf, rxctrl_rxcnt);
        PN_TRACE(tf, ie_txwm);
        PN_TRACE(tf, ie_rxwm);
        PN_TRACE(tf, ip_rxwm);
        PN_TRACE(tf, ip_txwm);
        PN_TRACE(tf, div_div);

        // Wishbone signals
        PN_TRACE(tf, wb_ack);

        // TX State
        PN_TRACE(tf, tx_state);
        PN_TRACE(tf, tx_next_state);

        // RX State
        PN_TRACE(tf, rx_state);
        PN_TRACE(tf, rx_next_state);

        // FIFO State and Signals
        PN_TRACE(tf, fifo_ack);
        PN_TRACE(tf, fifo_state);
        PN_TRACE(tf, fifo_next_state);

        // trace submodules
        baudgen_x16->pn_trace(tf, level);
        baudgen_rx->pn_trace(tf, level);
        baudgen_tx->pn_trace(tf, level);
        uart_rx->pn_trace(tf, level);
        uart_tx->pn_trace(tf, level);
#if !PN_CFG_UART_DISABLE_FIFO
        uart_rx_fifo->pn_trace(tf, level);
        uart_tx_fifo->pn_trace(tf, level);
#endif
    }
}

// **************** Helpers *********************

void m_uart::init_submodules()
{
    baudgen_x16 = sc_new<m_baudgen>("baudgen_x16");
    baudgen_x16->clk(clk);
    baudgen_x16->reset(reset);
    baudgen_x16->en_i(baudgen_x16_en);
    baudgen_x16->clear_i(baudgen_x16_clear);
    baudgen_x16->div_i(baudgen_x16_div);
    baudgen_x16->baudtick_o(baudtick_x16);

    baudgen_tx = sc_new<m_baudgen>("baudgen_tx");
    baudgen_tx->clk(clk);
    baudgen_tx->reset(reset);
    baudgen_tx->en_i(baudgen_tx_en);
    baudgen_tx->clear_i(baudgen_tx_clear);
    baudgen_tx->div_i(baudgen_tx_div);
    baudgen_tx->baudtick_o(baudtick_tx);

    baudgen_rx = sc_new<m_baudgen>("baudgen_rx");
    baudgen_rx->clk(clk);
    baudgen_rx->reset(reset);
    baudgen_rx->en_i(baudgen_rx_en);
    baudgen_rx->clear_i(baudgen_rx_clear);
    baudgen_rx->div_i(baudgen_rx_div);
    baudgen_rx->baudtick_o(baudtick_rx);

    uart_rx = sc_new<m_uart_rx>("uart_rx");
    uart_rx->clk(clk);
    uart_rx->reset(reset);
    uart_rx->baudtick_i(baudtick_rx);
    uart_rx->baudtick_x16_i(baudtick_x16);
    uart_rx->stopbit_8_9_i(rx_stopbit_8_9);
    uart_rx->rx_i(rx);
    uart_rx->data_o(rx_data);
    uart_rx->rx_finished_o(rx_finished);
    uart_rx->baudtick_disable_o(rx_baudtick_disable);

    uart_tx = sc_new<m_uart_tx>("uart_tx");
    uart_tx->clk(clk);
    uart_tx->reset(reset);
    uart_tx->baudtick_i(baudtick_tx);
    uart_tx->tx_start_i(tx_start);
    uart_tx->stopbit_cnt_i(tx_stopbit_8_9);
    uart_tx->tx_o(tx);
    uart_tx->tx_data_i(tx_data);
    uart_tx->tx_finish_o(tx_finished);

#if !PN_CFG_UART_DISABLE_FIFO
    uart_rx_fifo = sc_new<m_uart_fifo>("uart_rx_fifo");
    uart_rx_fifo->clk(clk);
    uart_rx_fifo->reset(reset);
    uart_rx_fifo->clear_i(rx_fifo_clear);
    uart_rx_fifo->write_i(rx_fifo_write);
    uart_rx_fifo->read_i(rx_fifo_read);
    uart_rx_fifo->full_o(rx_fifo_full);
    uart_rx_fifo->empty_o(rx_fifo_empty);
    uart_rx_fifo->usage_o(rx_fifo_usage);
    uart_rx_fifo->data_i(rx_fifo_data_in);
    uart_rx_fifo->data_o(rx_fifo_data_out);

    uart_tx_fifo = sc_new<m_uart_fifo>("uart_tx_fifo");
    uart_tx_fifo->clk(clk);
    uart_tx_fifo->reset(reset);
    uart_tx_fifo->clear_i(tx_fifo_clear);
    uart_tx_fifo->write_i(tx_fifo_write);
    uart_tx_fifo->read_i(tx_fifo_read);
    uart_tx_fifo->full_o(tx_fifo_full);
    uart_tx_fifo->empty_o(tx_fifo_empty);
    uart_tx_fifo->usage_o(tx_fifo_usage);
    uart_tx_fifo->data_i(tx_fifo_data_in);
    uart_tx_fifo->data_o(tx_fifo_data_out);
#endif
}

// **************** m_uart ******************

void m_uart::proc_comb_main()
{

    // if tx or rx is enabled, enable the baudgen
    if(txctrl_txen.read() || rxctrl_rxen.read())
    {
        baudgen_tx_en = 1;
    }
    else
    {
        baudgen_tx_en = 0;
    }

    // if rx is enabled enable the baudgen_x16
    if(rxctrl_rxen.read())
    {
        baudgen_rx_en = 1;
        baudgen_x16_en = 1;
    }
    else
    {
        baudgen_rx_en = 0;
        baudgen_x16_en = 0;
    }

    // calculate the divider for the baudgen_x16 from the divider register value
    baudgen_x16_div = div_div.read() >> 4;

    // connect the divider value for the baudgen to the divider register value
    baudgen_tx_div = div_div.read();

    baudgen_rx_div = div_div.read();

    // connect the stopbit value to the transmitter
    tx_stopbit_8_9 = txctrl_nstop.read();

    // TODO: Talk about it if it should be configurable or not (has no entry in the rxctrl register)
    rx_stopbit_8_9 = 0;

    // disable baudtick generators for rx if there is no transmission detected
    baudgen_x16_clear = rx_baudtick_disable;
    baudgen_rx_clear = rx_baudtick_disable;

#if !PN_CFG_UART_DISABLE_FIFO
    // connect dat_i to the input of the tx_fifo
    tx_fifo_data_in = txdata_data.read();

    // connect the tx_fifo_full to the register value txdata_full
    txdata_full = tx_fifo_full.read();

    // connect the rx_fifo_empty to the register value rxdata_empty
    rxdata_empty = rx_fifo_empty.read();

    // disable fifo clear
    rx_fifo_clear = 0;
    tx_fifo_clear = 0;
#else
    // if the fifos are connect the register signals that indicate if the register is full or empty
    txdata_full = tx_reg_full.read();
    rxdata_empty = rx_reg_empty.read();
#endif
}

void m_uart::proc_comb_wb()
{

    // generate ack if the cyc and stb are active and the
    // module wants to acknowledge the transaction
    // for this no fifo transaction that was initiated by the wb_slave
    // should be pending
    wb_slave.ack_o = wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_ack.read() && fifo_ack.read();

    wb_slave.rty_o = 0;
    wb_slave.err_o = 0;
}

void m_uart::proc_clk_wb()
{

    // Internal Variables
    sc_uint<16> div_div_var;
    sc_uint<32> wb_data_out_var;

    // Reset values
    wb_ack = 0;

    // register default values
    txdata_data = 0;

    txctrl_txen = 0;
    txctrl_nstop = 0;
    txctrl_txcnt = 0;

    rxctrl_rxen = 0;
    rxctrl_rxcnt = 0;

    ie_txwm = 0;
    ie_rxwm = 0;

    div_div = 0;

    while(true)
    {
        wait();

        wb_ack = 0;
        // check if the module is addressed
        if(wb_slave.adr_i.read().range(31, 5) == PN_CFG_UART_BASE_ADDRESS >> 5)
        {
            // check if a write transaction is in progress
            // and that no data is pending to be written to the
            // tx fifo
            if(wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_slave.we_i.read())
            {

                // only save data when the byteselect signal is correct
                switch(wb_slave.adr_i.read().range(4, 0))
                {

                    case WB_UART_REG_TXDATA:

#if !PN_CFG_UART_DISABLE_FIFO
                        if(wb_slave.sel_i.read()[0])
                        {
                            txdata_data = wb_slave.dat_i.read().range(7, 0);
                        }
#else
                        // enshure that the tx data register is not full
                        // if the fifo is disabled
                        if(wb_slave.sel_i.read()[0] && tx_reg_full.read() == 0)
                        {
                            txdata_data = wb_slave.dat_i.read().range(7, 0);
                        }
#endif

                        break;

                    case WB_UART_REG_RXDATA:
                        // register is read only

                        break;

                    case WB_UART_REG_TXCTRL:
                        if(wb_slave.sel_i.read()[0])
                        {

                            txctrl_txen = wb_slave.dat_i.read()[0];
                            txctrl_nstop = wb_slave.dat_i.read()[1];
                        }
                        if(wb_slave.sel_i.read()[2])
                        {
                            txctrl_txcnt = wb_slave.dat_i.read().range(18, 16);
                        }

                        break;

                    case WB_UART_REG_RXCTRL:
                        if(wb_slave.sel_i.read()[0])
                        {
                            rxctrl_rxen = wb_slave.dat_i.read()[0];
                        }
                        if(wb_slave.sel_i.read()[2])
                        {
                            rxctrl_rxcnt = wb_slave.dat_i.read().range(18, 16);
                        }

                        break;

                    case WB_UART_REG_IE:
                        if(wb_slave.sel_i.read()[0])
                        {
                            ie_txwm = wb_slave.dat_i.read()[0];
                            ie_rxwm = wb_slave.dat_i.read()[1];
                        }

                        break;

                    case WB_UART_REG_IP:
                        // register is read only

                        break;

                    case WB_UART_REG_DIV:

                        div_div_var = div_div.read();

                        if(wb_slave.sel_i.read()[0])
                        {
                            div_div_var(7, 0) = wb_slave.dat_i.read().range(7, 0);
                        }
                        if(wb_slave.sel_i.read()[1])
                        {
                            div_div_var(15, 8) = wb_slave.dat_i.read().range(15, 8);
                        }

                        div_div = div_div_var;

                        break;

                    default:
                        break;
                }

                // when data is written to the selected register
                // acknowledge the transaction
                wb_ack = 1;
            }

            // check if a read transaction is in progress
            // and that no data is pending to be read from the
            // rx fifo otherwise wait until the data is read
            if(wb_slave.cyc_i.read() && wb_slave.stb_i.read() && !wb_slave.we_i.read())
            {
                switch(wb_slave.adr_i.read().range(4, 0))
                {

                    case WB_UART_REG_TXDATA:

                        wb_data_out_var = (txdata_full, sc_uint<23>(0), txdata_data);
                        break;

                    case WB_UART_REG_RXDATA:

                        // if the rx fifo is disabled use the rx_data register
#if !PN_CFG_UART_DISABLE_FIFO
                        wb_data_out_var = (rxdata_empty, sc_uint<23>(0), rx_fifo_data_out);
#else
                        wb_data_out_var = (rxdata_empty, sc_uint<23>(0), rxdata_data);
#endif
                        break;

                    case WB_UART_REG_TXCTRL:

                        wb_data_out_var = (sc_uint<13>(0), txctrl_txcnt, sc_uint<14>(0), txctrl_nstop, txctrl_txen);
                        break;

                    case WB_UART_REG_RXCTRL:

                        wb_data_out_var = (sc_uint<13>(0), rxctrl_rxcnt, sc_uint<15>(0), rxctrl_rxen);
                        break;

                    case WB_UART_REG_IE:

                        wb_data_out_var = (sc_uint<30>(0), ie_rxwm, ie_txwm);
                        break;

                    case WB_UART_REG_IP:

                        wb_data_out_var = (sc_uint<30>(0), ip_rxwm, ip_txwm);
                        break;

                    case WB_UART_REG_DIV:

                        wb_data_out_var = (sc_uint<16>(0), div_div);
                        break;

                    default:

                        wb_data_out_var = 0;
                        break;
                }

                // mask the data according to the byteselect signal
                for(uint8_t i = 0; i < 4; i++)
                {
                    if(wb_slave.sel_i.read()[i] == 0)
                    {

                        wb_data_out_var(7 + i * 8, i * 8) = wb_data_out_var(7 + i * 8, i * 8) & 0x00;
                    }
                }

                wb_slave.dat_o = (sc_uint<32>(), wb_data_out_var);

                // acknowledge the read transaction
                wb_ack = 1;
            }
        }
    }
}

void m_uart::proc_comb_tx()
{

    // Default values
    tx_next_state = tx_state;
    tx_start = 0;
    baudgen_tx_clear = 0;
    tx_data_reg = tx_data.read();
#if !PN_CFG_UART_DISABLE_FIFO
    tx_fifo_read = 0;
#else
    c_clr_tx_reg_full = 0;
#endif

    switch(tx_state.read())
    {

        case WB_UART_TOP_TX_IDLE:

#if !PN_CFG_UART_DISABLE_FIFO
            // check if the tx fifo is not empty and the tx is enabled
            // to start the transmission of a byte
            if(!tx_fifo_empty.read() && txctrl_txen.read())
            {
                tx_next_state = WB_UART_TX_START;
            }
#else

            // check if there is a byte to send in the tx register if the
            // fifo is disabled also check if the tx is enabled then start
            // the transmission of the byte
            if(tx_reg_full.read() && txctrl_txen.read())
            {
                tx_next_state = WB_UART_TOP_TX_START;
            }
#endif

            break;

        case WB_UART_TOP_TX_START:

#if !PN_CFG_UART_DISABLE_FIFO
            // read a byte from the fifo and start the transmission of the byte
            tx_start = 1;
            tx_fifo_read = 1;
            tx_data_reg = tx_fifo_data_out.read();

            tx_next_state = WB_UART_TOP_TX_WAIT;
#else
            // read a byte from the register and start the transmission of the byte
            tx_start = 1;
            tx_data_reg = txdata_data.read();
            tx_next_state = WB_UART_TOP_TX_WAIT;
#endif

            break;

        case WB_UART_TOP_TX_WAIT:

            // wait until finish is not set to prevent missing sending a byte
            tx_start = 1;

#if PN_CFG_UART_DISABLE_FIFO
            // reset the tx register full flag to 0
            c_clr_tx_reg_full = 1;
#endif

            if(!tx_finished.read())
            {
                tx_next_state = WB_UART_TOP_TX_RUN;
            }

            break;

        case WB_UART_TOP_TX_RUN:

            // wait until the transmission of the byte is finished
            tx_start = 1;
            if(tx_finished.read())
            {
                tx_next_state = WB_UART_TOP_TX_IDLE;
            }
    }
}

void m_uart::proc_clk_tx()
{

    // default state
    tx_state = WB_UART_TOP_TX_IDLE;

    while(true)
    {
        wait();

        // read the tx data to create a register
        tx_data = tx_data_reg.read();

        // state transition
        tx_state = tx_next_state;
#ifndef __SYNTHESIS__
        /**
         * Debug output for simulator
         */
        if(tx_state.read() == WB_UART_TOP_TX_RUN &&
            tx_next_state.read() == WB_UART_TOP_TX_IDLE &&
            tx_finished.read())
        {
            cout << (char)tx_data.read().to_uint() << flush;
        }
#endif // __SYNTHESIS__
    }
}

void m_uart::proc_comb_rx()
{

    // default values
    rx_next_state = rx_state;
#if !PN_CFG_UART_DISABLE_FIFO
    rx_fifo_data_in = 0;
    rx_fifo_write = 0;
#else
    c_clr_rx_reg_empty = 0;
    c_save_rxdata_data = 0;
#endif

    switch(rx_state.read())
    {

        case WB_UART_TOP_RX_IDLE:

#if !PN_CFG_UART_DISABLE_FIFO
            // check if the rx fifo is not full, the rx is enabled and the rx finished receiving a byte

            if(!rx_fifo_full.read() && rxctrl_rxen.read() && rx_finished.read())
            {

                rx_next_state = WB_UART_TOP_RX_SAVE;
            }
#else
            // check if the rx register is not full, if the fifo is disabled, the rx is enabled and the rx finished receiving a byte
            if(rx_reg_empty.read() && rxctrl_rxen.read() && rx_finished.read())
            {
                rx_next_state = WB_UART_TOP_RX_SAVE;
            }
#endif

            break;

        case WB_UART_TOP_RX_WAIT1:

            // wait at least one clock cycle to prevent interpreting
            // the rx_finished from the last byte as a new byte finishing
            // the reception

            rx_next_state = WB_UART_TOP_RX_SAVE;

            break;

        case WB_UART_TOP_RX_SAVE:

#if !PN_CFG_UART_DISABLE_FIFO
            // write the received byte to the rx fifo
            rx_fifo_write = 1;
            rx_fifo_data_in = rx_data.read();

            rx_next_state = WB_UART_TOP_RX_WAIT2;
#else
            // set the rx register empty flag to 0
            // and save data to the rx register
            c_clr_rx_reg_empty = 1;
            c_save_rxdata_data = 1;
            rx_next_state = WB_UART_TOP_RX_WAIT2;
#endif

            break;

        case WB_UART_TOP_RX_WAIT2:

            // write the received byte to the rx fifo

            if(!rx_finished.read())
            {
                rx_next_state = WB_UART_TOP_RX_IDLE;
            }

            break;
    }
}

void m_uart::proc_clk_rx()
{
    // reset
    rxdata_data = 0;

    // default state
    rx_state = WB_UART_TOP_RX_IDLE;

    while(true)
    {
        wait();

        // state transition
        rx_state = rx_next_state;

#if PN_CFG_UART_DISABLE_FIFO
        // save the received data to the rx register
        if(c_save_rxdata_data.read())
        {
            rxdata_data = rx_data.read();
        }
#endif
    }
}

void m_uart::proc_comb_interupt()
{

#if !PN_CFG_UART_DISABLE_FIFO
    // check if the tx fifo usage is less than the tx counter
    // and if the tx watermark interrupt is enabled
    // then set the tx watermark interrupt pending flag
    if((tx_fifo_usage.read() < txctrl_txcnt.read()) && ie_txwm.read())
    {
        ip_txwm = 1;
    }
    else
    {
        ip_txwm = 0;
    }

    // check if the rx fifo usage is greater than the rx counter
    // and if the rx watermark interrupt is enabled
    // then set the rx watermark interrupt pending flag
    if((rx_fifo_usage.read() > rxctrl_rxcnt.read()) && ie_rxwm.read())
    {
        ip_rxwm = 1;
    }
    else
    {
        ip_rxwm = 0;
    }
#else
    // if the fifo is disabled set the interupt flag regardles of the watermark
    // value when the tx register is empty and the tx watermark interrupt is enabled
    if(tx_reg_full.read() && ie_txwm.read())
    {
        ip_txwm = 1;
    }
    else
    {
        ip_txwm = 0;
    }

    // if the fifo is disabled set the interupt flag regardles of the watermark
    // value when the rx register is empty and the rx watermark interrupt is enable
    if(rx_reg_empty.read() && ie_rxwm.read())
    {
        ip_rxwm = 1;
    }
    else
    {
        ip_rxwm = 0;
    }
#endif
}

void m_uart::proc_comb_fifo()
{

    // defaults
    fifo_next_state = fifo_state;
    fifo_ack = 1;
#if !PN_CFG_UART_DISABLE_FIFO
    rx_fifo_read = 0;
    tx_fifo_write = 0;
#else
    c_set_rx_reg_empty = 0;
    c_set_tx_reg_full = 0;
#endif

    switch(fifo_state.read())
    {
        case WB_UART_TOP_FIFO_IDLE:

#if !PN_CFG_UART_DISABLE_FIFO
            // if a write transaction is started to WB_UART_REG_TXDATA while the fifo
            // is not full then change state. it is checked that ack is not set
            // to enshure that the transaction has just started also make shure that
            // the fifo is not beeing read from this is to prevent invalid data transactions
            // to prevent writing old data to the fifo prevent a fifo write when byteselect is not correct
            if(wb_slave.adr_i.read() == PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA && wb_slave.we_i.read() && !tx_fifo_full.read() && !tx_fifo_read.read() && !wb_ack.read() && wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_slave.sel_i.read()[0])
            {
                fifo_next_state = WB_UART_TOP_FIFO_TX_WRITE;
            }

            // if a read transaction is started to WB_UART_REG_RXDATA while the fifo
            // is not empty then change state. it is checked that ack is not set
            // to enshure that the transaction has just started also make shure that
            // the fifo is not beeing written to this is to prevent invalid data transactions
            // to prevent reading old data from the fifo prevent a fifo read when byteselect is not correct
            if(wb_slave.adr_i.read() == PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA && !wb_slave.we_i.read() && !rx_fifo_empty.read() && !rx_fifo_write.read() && !wb_ack.read() && wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_slave.sel_i.read()[0])
            {
                fifo_next_state = WB_UART_TOP_FIFO_RX_ACK;
            }

#else
            // if a write transaction is started to WB_UART_REG_TXDATA while the fifo is disabled and the txdata register
            // is not in use then change state. it is checked that ack is not set to enshure that the transaction has just started
            // also make shure that the fifo is not beeing read from this is to prevent invalid data transactions
            // to prevent writing old data to the fifo prevent a fifo write when byteselect is not correct
            if(wb_slave.adr_i.read() == PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA && wb_slave.we_i.read() && !tx_reg_full.read() && !wb_ack.read() && wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_slave.sel_i.read()[0])
            {
                fifo_next_state = WB_UART_TOP_FIFO_TX_WRITE;
            }

            // if a read transaction is started to WB_UART_REG_RXDATA while the fifo is disabled and the rxdata register
            // is not in use then change state. it is checked that ack is not set to enshure that the transaction has just started
            // also make shure that the fifo is not beeing written to this is to prevent invalid data transactions
            // to prevent reading old data from the fifo prevent a fifo read when byteselect is not correct
            if(wb_slave.adr_i.read() == PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA && !wb_slave.we_i.read() && !rx_reg_empty.read() && !wb_slave.wb_ack.read() && wb_slave.cyc_i.read() && wb_slave.stb_i.read() && wb_slave.sel_i.read()[0])
            {
                fifo_next_state = WB_UART_TOP_FIFO_RX_ACK;
            }

#endif

            break;

        case WB_UART_TOP_FIFO_RX_ACK:

            // wait till the current transaction is finished
            // meaning that the word is read completely from the fifo
            if(!wb_slave.cyc_i.read() && !wb_slave.stb_i.read())
            {
                fifo_next_state = WB_UART_TOP_FIFO_RX_READ;
            }

            break;

        case WB_UART_TOP_FIFO_RX_READ:

#if !PN_CFG_UART_DISABLE_FIFO
            // read the next word out of the fifo into
            // the WB_UART_REG_RXDATA register while
            // the transaction is actiive prevent new reads
            // from the register to prevent invalid data transactions
            rx_fifo_read = 1;
            fifo_ack = 0;
            fifo_next_state = WB_UART_TOP_FIFO_IDLE;
#else
            // set the rx register empty flag to 1
            // to indicate that the register is empty
            // and the next read transaction can be started
            // also prevent new reads from the register to prevent
            c_set_rx_reg_empty = 1;
            fifo_ack = 0;
            fifo_next_state = WB_UART_TOP_FIFO_IDLE;
#endif

            break;

        case WB_UART_TOP_FIFO_TX_WRITE:

#if !PN_CFG_UART_DISABLE_FIFO
            // wait till the data is written to the fifo
            // and prevent acks durring the time
            // to prevent invalid data transactions
            fifo_ack = 0;
            tx_fifo_write = 1;

            fifo_next_state = WB_UART_TOP_FIFO_TX_ACK;
#else
            // set the tx register full flag to 1
            // to indicate that the register is full
            // and the next write transaction can be started
            // also prevent new writes to the register to prevent
            c_set_tx_reg_full = 1;
            fifo_ack = 0;
            fifo_next_state = WB_UART_TOP_FIFO_IDLE;
#endif

            break;

        case WB_UART_TOP_FIFO_TX_ACK:

            // wait till the wb transaction is finished to reset the
            // fifo state to idle
            if(!wb_slave.cyc_i.read() && !wb_slave.stb_i.read())
            {
                fifo_next_state = WB_UART_TOP_FIFO_IDLE;
            }

            break;

        default:
            break;
    }
}

void m_uart::proc_clk_fifo()
{

    // reset
    fifo_state = WB_UART_TOP_FIFO_IDLE;

#if PN_CFG_UART_DISABLE_FIFO
    rx_reg_empty = 1;
    tx_reg_full = 0;
#endif

    while(true)
    {
        wait();

        // state transition

        fifo_state = fifo_next_state;

#if PN_CFG_UART_DISABLE_FIFO
        // set the rx register empty flag to 0
        // to indicate that the register is not empty
        // and the next read transaction has to wait
        if(c_clr_rx_reg_empty.read())
        {
            rx_reg_empty = 0;
        }

        // set the tx register full flag to 1
        // to indicate that the register is full
        // and the next write transaction can be started
        if(c_set_rx_reg_empty.read())
        {
            rx_reg_empty = 1;
        }

        // set the tx register full flag to 0
        // to indicate that the register is empty
        // and the next write transaction can be started
        if(c_clr_tx_reg_full.read())
        {
            tx_reg_full = 0;
        }

        // set the tx register full flag to 1
        // to indicate that the register is full
        // and the next write transaction has to wait
        if(c_set_tx_reg_full.read())
        {
            tx_reg_full = 1;
        }

#endif
    }
}
