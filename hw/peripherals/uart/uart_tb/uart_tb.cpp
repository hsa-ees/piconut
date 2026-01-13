/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Main testbench of the UART peripheral module.

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

#include "../uart.h"

#include <stdint.h>

#define PERIOD_NS 40
#define SYS_FREQ 1 / 40E-9
#define BAUDRATE 115200
#define BAUDDIV (int)((SYS_FREQ / BAUDRATE) - 1)
#define BAUDTICKS (int)(((float)1 / BAUDRATE) / 40E-9)

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// wishbone signals
sc_signal<bool> PN_NAME(stb_i);        // Strobe
sc_signal<bool> PN_NAME(cyc_i);        // Cycle
sc_signal<bool> PN_NAME(we_i);         // Write Enable
sc_signal<pn_wb_sel_t> PN_NAME(sel_i); // Select

sc_signal<bool> PN_NAME(ack_o); // Acknowledge
sc_signal<bool> PN_NAME(err_o); // Error
sc_signal<bool> PN_NAME(rty_o); // Retry

sc_signal<pn_wb_adr_t> PN_NAME(addr_i); // Address
sc_signal<pn_wb_dat_t> PN_NAME(dat_i);  // Data in
sc_signal<pn_wb_dat_t> PN_NAME(dat_o);  // Data out

// UART signals
sc_signal<bool> PN_NAME(rx); // RX
sc_signal<bool> PN_NAME(tx); // TX

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void wishbone_write(uint32_t addr, uint32_t data, uint8_t sel = 0xF)
{

    int count = 0;

    // set the signals for wb write setup
    stb_i = 1;
    cyc_i = 1;
    we_i = 1;
    sel_i = sel;
    addr_i = addr;
    dat_i = data;

    // wait for acknoledgement of the write
    while(ack_o.read() == 0)
    {
        run_cycle();
        count++;

        if(count > 100)
        {
            PN_ERROR("WB Write Timeout");
        }
    }

    // clear the signals at the wb
    stb_i = 0;
    cyc_i = 0;
    we_i = 0;
    sel_i = 0;
    addr_i = 0;
    dat_i = 0;

    run_cycle();
}

uint32_t wishbone_read(uint32_t addr, uint8_t sel = 0xF)
{

    int cycles = 0;

    // set the signals for wb read setup
    stb_i = 1;
    cyc_i = 1;
    we_i = 0;
    sel_i = sel;
    addr_i = addr;
    dat_i = 0;

    // wait for acknoledgement of the read
    while(ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles > 100)
        {
            PN_ERROR("WB Read Timeout");
        }
    }

    // getting the data from the wb
    uint32_t data = (uint32_t)dat_o.read();

    // clear the signals at the wb
    stb_i = 0;
    cyc_i = 0;
    we_i = 0;
    sel_i = 0;
    addr_i = 0;

    run_cycle();

    return data;
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("uart");

    // Initialliaze the Design under Testing (DUT)
    m_uart i_dut{"i_dut", PN_CFG_UART_BASE_ADDRESS};

    // connects signals from TOP to TB
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.wb_slave.stb_i(stb_i);
    i_dut.wb_slave.cyc_i(cyc_i);
    i_dut.wb_slave.we_i(we_i);
    i_dut.wb_slave.sel_i(sel_i);
    i_dut.wb_slave.ack_o(ack_o);
    i_dut.wb_slave.adr_i(addr_i);
    i_dut.wb_slave.dat_i(dat_i);
    i_dut.wb_slave.dat_o(dat_o);
    i_dut.wb_slave.rty_o(rty_o);
    i_dut.wb_slave.err_o(err_o);
    i_dut.rx(rx);
    i_dut.tx(tx);

    // Traces of signals here

    i_dut.pn_trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // set rx to 1 (default)
    rx = 1;

    // Reset
    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);

    // set the baudrate divider register for 9600 baud (2604)

    PN_INFO("Test setting baudrate divider register");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_DIV, BAUDDIV, 0x3);
    uint32_t data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_DIV, 0x3);
    PN_INFOF(("DIV: %d", data));
    PN_ASSERTM(data == BAUDDIV, "DIV not set correctly");

    // Enable Transmitter
    PN_INFO("Test enabeling transmitter");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: %d", data));
    PN_ASSERTM(data == 0x1, "TXCTRL not set correctly");

    // Send 0x55
    PN_INFO("Test sending 0x55");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x55, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x1);
    PN_INFOF(("TXDATA: 0x%02x", data));

    // checking for startbit
    PN_INFO("Checking for startbit");
    // wait until the baudtick + 1 cycle for the tx line to change
    while(!i_dut.get_baudtick())
    {
        run_cycle();
    }
    run_cycle();
    PN_INFOF(("TX: %d", tx.read()));
    PN_ASSERTM(tx == 0, "TX startbit not set correctly");

    // Checking for the bits of the data
    PN_INFO("Checking for the bits of the data");
    // wait until the baudtick + 1 cycle for the tx line to change
    for(int i = 0; i < 8; i++)
    {

        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();
        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTF(tx.read() == ((0x55 >> i) & 0x1), ("TX[%d] not set correctly", i));
    }

    // Checking for the stopbit
    PN_INFO("Checking for the stopbit");
    // wait until the baudtick + 1 cycle for the tx line to change
    while(!i_dut.get_baudtick())
    {
        run_cycle();
    }
    run_cycle();
    PN_INFOF(("TX: %d", tx.read()));
    PN_ASSERTM(tx == 1, "TX stopbit not set correctly");

    // Chaning tx to use two stopbits
    PN_INFO("Changing tx to use two stopbits");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x3, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: %d", data));
    PN_ASSERTM(data == 0x3, "TXCTRL not set correctly");

    // Send 0x55
    PN_INFO("Test sending 0x55");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x55, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x1);
    PN_INFOF(("TXDATA: 0x%02x", data));
    PN_ASSERTM(data == 0x55, "TXDATA not set correctly");

    // checking for startbit
    PN_INFO("Checking for startbit");
    // wait until the baudtick + 1 cycle for the tx line to change
    while(!i_dut.get_baudtick())
    {
        run_cycle();
    }
    run_cycle(1);
    PN_INFOF(("TX: %d", tx.read()));
    PN_ASSERTM(tx == 0, "TX startbit not set correctly");

    // Checking for the bits of the data
    PN_INFO("Checking for the bits of the data");
    // wait until the baudtick + 1 cycle for the tx line to change
    for(int i = 0; i < 8; i++)
    {

        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();
        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTF(tx.read() == ((0x55 >> i) & 0x1), ("TX[%d] not set correctly", i));
    }

    // Checking for the stopbit
    PN_INFO("Checking for the stopbit");
    // wait until the baudtick + 1 cycle for the tx line to change
    for(int i = 0; i < 2; i++)
    {
        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();
        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTM(tx == 1, "TX stopbit not set correctly");
    }

    // Checking for the full flag to be set when tx fifo is full
    PN_INFO("Checking for the full flag to be set when tx fifo is full");

    // Disable Transmitter
    PN_INFO("disabeling transmitter");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x0, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: %d", data));
    PN_ASSERTM(data == 0x0, "TXCTRL not set correctly");

    // Enable the transmitter interrupt
    PN_INFO("Enabeling the transmitter interrupt");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IE, 0x1, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IE, 0x1);
    PN_INFOF(("IE: %d", data));
    PN_ASSERTM(data == 0x1, "IE not set correctly");

// these tests are only used when the fifo is enabled
#if !PN_CFG_UART_DISABLE_FIFO
    // Set the tx interupt watermark to 4
    PN_INFO("Setting the tx interrupt watermark to 4");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x4);
    data &= ~0x00070000;
    data |= 0x4 << 16;
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, data, 0x4);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x4);
    PN_INFOF(("TXCTRL: 0x%08x", data));
    PN_ASSERTM(data == 0x00040000, "TXCTRL not set correctly");

    // Check if the watermark flag is set correctly when 4 bytes are written
    for(int i = 1; i <= 8; i++)
    {

        wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, i, 0x1);
        data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x1);
        data &= 0xFF;
        PN_INFOF(("TXDATA: 0x%02x", data));
        PN_ASSERTM(data == i, "TXDATA not set correctly");

        if(i < 4)
        {
            data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
            data &= 0x1;
            PN_INFOF(("IP: %d", data));
            PN_ASSERTM(data == 1, "IP not set correctly");
        }
        else
        {
            data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
            data &= 0x1;
            PN_INFOF(("IP: %d", data));
            PN_ASSERTM(data == 0, "IP not set correctly");
        }
    }

    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x8);
    data = (data >> 31) & 0x1;
    PN_INFOF(("TXFULL: %d", data));
    PN_ASSERTM(data, "TXFULL not set correctly");

    // Sending the 8 bytes

    // Enable Transmitter
    PN_INFO("Test enabeling transmitter");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    data &= ~(0x1);
    data |= 0x1;
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, data, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: 0x%08x", data));
    PN_ASSERTM(data == 0x1, "TXCTRL not set correctly");

    for(int i = 1; i <= 8; i++)
    {

        PN_INFOF(("Sending: 0x%02x", i));

        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();

        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTM(tx.read() == 0, "Startbit not set correctly");

        for(int j = 0; j < 8; j++)
        {

            while(!i_dut.get_baudtick())
            {
                run_cycle();
            }
            run_cycle();
            PN_INFOF(("TX: %d", tx.read()));
            PN_ASSERTF(tx.read() == ((i >> j) & 0x1), ("TX[%d] not set correctly", j));
        }

        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();
        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTM(tx == 1, "TX stopbit not set correctly");
    }

#else
    // checking if the full flag and the interrupt flag are set when only one byte is written
    // when the fifo is disabled

    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x1, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x1);
    data &= 0xFF;
    PN_INFOF(("TXDATA: 0x%02x", data));
    PN_ASSERTM(data == 0x1, "TXDATA not set correctly");

    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
    data &= 0x1;
    PN_INFOF(("IP: %d", data));
    PN_ASSERTM(data == 1, "IP not set correctly");

    // checking the full flag
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXDATA, 0x8);
    data = (data >> 31) & 0x1;
    PN_INFOF(("TXFULL: %d", data));
    PN_ASSERTM(data, "TXFULL not set correctly");

    // Enable Transmitter
    PN_INFO("Test enabeling transmitter");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    data &= ~(0x1);
    data |= 0x1;
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, data, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: 0x%08x", data));
    PN_ASSERTM(data == 0x1, "TXCTRL not set correctly");

    PN_INFOF(("Sending: 0x%02x", 0x1));

    while(!i_dut.get_baudtick())
    {
        run_cycle();
    }
    run_cycle();

    PN_INFOF(("TX: %d", tx.read()));
    PN_ASSERTM(tx.read() == 0, "Startbit not set correctly");

    for(int j = 0; j < 8; j++)
    {

        while(!i_dut.get_baudtick())
        {
            run_cycle();
        }
        run_cycle();
        PN_INFOF(("TX: %d", tx.read()));
        PN_ASSERTF(tx.read() == ((0x1 >> j) & 0x1), ("TX[%d] not set correctly", j));
    }

    while(!i_dut.get_baudtick())
    {
        run_cycle();
    }
    run_cycle();
    PN_INFOF(("TX: %d", tx.read()));
    PN_ASSERTM(tx == 1, "TX stopbit not set correctly");

#endif

    // Disable Transmitter

    PN_INFO("disabeling transmitter");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x0, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL, 0x1);
    PN_INFOF(("TXCTRL: %d", data));
    PN_ASSERTM(data == 0x0, "TXCTRL not set correctly");

    // Enable Receiver
    PN_INFO("Test enabeling receiver");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL, 0x1, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL, 0x1);
    PN_INFOF(("RXCTRL: %d", data));
    PN_ASSERTM(data == 0x1, "RXCTRL not set correctly");

    // Send 0x55

    PN_INFOF(("Sending: 0x55"));

    // Sending start bit
    rx = 0;
    run_cycle(BAUDTICKS);

    for(int i = 0; i < 8; i++)
    {

        rx = (0x55 >> i) & 0x1;
        run_cycle(BAUDTICKS);
    }

    // Sending stop bit
    rx = 1;
    run_cycle(BAUDTICKS);

    // Checking the rxdata empty flag
    PN_INFO("Checking the rxdata empty flag");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x8);
    PN_INFOF(("RXDATA_EMPTY: %d", data));
    PN_ASSERTM(data == 0, "RXDATA_EMPTY should be unset because the fifo is not empty");

    // Checking for the data
    PN_INFO("Checking for the data");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x1);
    data &= 0xFF;
    PN_INFOF(("RXDATA: 0x%02x", data));
    PN_ASSERTM(data == 0x55, "RXDATA not set correctly");

    // Enable the receiver interrupt
    PN_INFO("Enabeling the receiver interrupt");
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IE, 0x2, 0x1);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IE, 0x1);
    PN_INFOF(("IE: %d", data));
    PN_ASSERTM(data == 0x2, "IE not set correctly");

#if !PN_CFG_UART_DISABLE_FIFO
    // Set the rx interupt watermark to 4
    PN_INFO("Setting the rx interrupt watermark to 4");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL, 0x4);
    data &= ~0x00070000;
    data |= 0x4 << 16;
    wishbone_write(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL, data, 0x4);
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL);
    PN_INFOF(("RXCTRL: 0x%08x", data));
    PN_ASSERTM(data == 0x00040001, "RXCTRL not set correctly");

    // Sending multiple bytes
    PN_INFO("Sending multiple bytes");
    for(int i = 1; i <= 8; i++)
    {

        PN_INFOF(("Sending: 0x%02x", i));

        // Sending start bit
        rx = 0;
        run_cycle(BAUDTICKS);

        for(int j = 0; j < 8; j++)
        {

            rx = (i >> j) & 0x1;
            run_cycle(BAUDTICKS);
            PN_INFOF(("RX: %d", (i >> j) & 0x1));
        }

        // Sending stop bit
        rx = 1;
        run_cycle(BAUDTICKS);
    }

    // Checking for the data
    PN_INFO("Checking the Data received");
    for(int i = 1; i <= 8; i++)
    {

        // Checking if rxdata empty is unset
        data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x8);
        PN_INFOF(("RXDATA_EMPTY: %d", data));
        PN_ASSERTM(data == 0, "RXDATA_EMPTY should be unset because the fifo is not empty");

        data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x1);
        data &= 0xFF;
        PN_INFOF(("RXDATA: 0x%02x", data));
        PN_ASSERTF(data == i, ("RXDATA not set correctly. Should be 0x%02x is 0x%02x", i, data));

        run_cycle();

        if(i < 4)
        {
            data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
            data &= 0x2;
            PN_INFOF(("IP: %d", data));
            PN_ASSERTM(data == 2, "IP not set correctly");
        }
        else
        {
            data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
            data &= 0x2;
            PN_INFOF(("IP: %d", data));
            PN_ASSERTM(data == 0, "IP not set correctly");
        }
    }

    data = ((wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x8) >> 31) & 1);
    PN_INFOF(("RXDATA_EMPTY: 0x%01x", data));
    PN_ASSERTM(data == 1, "RXDATA_EMPTY should be set because the fifo is empty");

#else

    // Sending one byte to check if the full flag is set when the fifo is disabled
    // and the interrupt flag is set when the fifo is disabled
    PN_INFO("Sending one byte");

    PN_INFOF(("Sending: 0x%02x", 0x1));

    // Sending start bit
    rx = 0;
    run_cycle(BAUDTICKS);

    for(int j = 0; j < 8; j++)
    {

        rx = (0x1 >> j) & 0x1;
        run_cycle(BAUDTICKS);
        PN_INFOF(("RX: %d", (0x1 >> j) & 0x1));
    }

    // Sending stop bit
    rx = 1;
    run_cycle(BAUDTICKS);

    // Checking for the data
    PN_INFO("Checking the Data received");
    // Checking if rxdata empty is unset
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x8);
    PN_INFOF(("RXDATA_EMPTY: %d", data));
    PN_ASSERTM(data == 0, "RXDATA_EMPTY should be unset because the fifo is not empty");

    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x1);
    data &= 0xFF;
    PN_INFOF(("RXDATA: 0x%02x", data));
    PN_ASSERTF(data == 0x1, ("RXDATA not set correctly. Should be 0x%02x is 0x%02x", 0x1, data));

    PN_INFO("Checking the interrupt flag");
    data = wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_IP, 0x1);
    data &= 0x2;
    PN_INFOF(("IP: %d", data));
    PN_ASSERTM(data == 0, "IP not set correctly");
    run_cycle();

#endif

    // Send 0x55 and reset

    PN_INFOF(("Sending: 0x55"));

    // Sending start bit
    rx = 0;
    run_cycle(BAUDTICKS);

    for(int i = 0; i < 8; i++)
    {

        rx = (0x55 >> i) & 0x1;
        run_cycle(BAUDTICKS);
    }

    // Sending stop bit
    rx = 1;
    run_cycle(BAUDTICKS);

    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);

    data = ((wishbone_read(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA, 0x8) >> 31) & 1);
    PN_INFOF(("RXDATA_EMPTY: 0x%01x", data));
    PN_ASSERTM(data == 1, "RXDATA_EMPTY should be set because the fifo is empty");

    run_cycle();
    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
