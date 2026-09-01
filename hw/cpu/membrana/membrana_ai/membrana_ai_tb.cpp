/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_memory Unit (MemU) for simulation ONLY

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


#include "membrana_ai.h"

#include <pn_uart.h>





// ********************** Ports ************************************************


// General ...
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// IPorts ...
sc_vector<sc_signal<bool>>          PN_NAME_VEC(iport_stb, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(iport_adr, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<4>>>    PN_NAME_VEC(iport_bsel, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(iport_rdata, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(iport_ack, PN_CFG_CPU_CORES);

// DPorts ...
sc_vector<sc_signal<bool>>          PN_NAME_VEC(dport_stb, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(dport_we, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(dport_adr, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(dport_wdata, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<4>>>    PN_NAME_VEC(dport_bsel, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(dport_amo, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(dport_lrsc, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(dport_rdata, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(dport_ack, PN_CFG_CPU_CORES);

// VPorts ...
sc_vector<sc_signal<bool>>          PN_NAME_VEC(vport_stb, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(vport_we, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(vport_adr, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(vport_wdata, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<4>>>    PN_NAME_VEC(vport_bsel, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(vport_amo, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(vport_lrsc, PN_CFG_CPU_CORES);
sc_vector<sc_signal<sc_uint<32>>>   PN_NAME_VEC(vport_rdata, PN_CFG_CPU_CORES);
sc_vector<sc_signal<bool>>          PN_NAME_VEC(vport_ack, PN_CFG_CPU_CORES);





// ********************** Helpers **********************************************


#define PERIOD_NS 10.0


static int cycles = 0;


void run_cycle (int n = 1) {
  for (int i = 0; i < n; i++) {
    clk = 1;
    sc_start (PERIOD_NS / 2, SC_NS);
    clk = 0;
    sc_start (PERIOD_NS / 2, SC_NS);
  }
  clk = 1;
  sc_start (0, SC_NS);    // Trigger DUT events to let them happen before new inputs are set by testbench
  cycles += n;
}


static inline int get_cycles () { return cycles; }





// ********************** Read/Write functions *********************************


// ***** IPort *****


void iport_read32_start (int port, uint32_t adr, int bsel = 0xf) {
  iport_adr[port] = adr;    // set address
  iport_bsel[port] = bsel;  // set byte select
  iport_stb[port] = 1;      // set strobe
  run_cycle ();
  iport_stb[port] = 0;      // reset strobe
}


uint32_t iport_read32_wait (int port) {
  while (iport_ack[port] == 0) run_cycle ();
  return iport_rdata[port].read ();
}


uint32_t iport_read32 (int port, uint32_t adr, int bsel = 0xf) {
  iport_read32_start (port, adr, bsel);
  return iport_read32_wait (port);
}


// ***** DPort: Read *****


void dport_read32_start (int port, uint32_t adr, int bsel = 0xf) {
  dport_adr[port] = adr;    // set address
  dport_bsel[port] = bsel;  // set byte select
  dport_we[port] = 0;       // set write enable = 0
  dport_stb[port] = 1;      // set strobe
  run_cycle ();
  dport_stb[port] = 0;      // reset strobe
}


uint32_t dport_read32_wait (int port) {
  while (dport_ack[port] == 0) run_cycle ();
  return dport_rdata[port].read ();
}


uint32_t dport_read32 (int port, uint32_t adr, int bsel = 0xf) {
  dport_read32_start (port, adr, bsel);
  return dport_read32_wait (port);
}


// ***** DPort: Write *****


void dport_write32_start (int port, uint32_t adr, int bsel, uint32_t wdata) {
  dport_adr[port] = adr;      // set address
  dport_bsel[port] = bsel;    // set byte select
  dport_wdata[port] = wdata;  // set data
  dport_we[port] = 1;         // set write enable
  dport_stb[port] = 1;        // set strobe
  run_cycle ();
  dport_we[port] = 0;         // reset write enable
  dport_stb[port] = 0;        // reset strobe
}


void dport_write32_wait (int port) {
  while (dport_ack[port] == 0) run_cycle ();
}


void dport_write32 (int port, uint32_t adr, int bsel, uint32_t wdata) {
  dport_write32_start (port, adr, bsel, wdata);
  dport_write32_wait (port);
}


// ***** VPort: Read *****


void vport_read32_start (int port, uint32_t adr, int bsel = 0xf) {
  vport_adr[port] = adr;    // set address
  vport_bsel[port] = bsel;  // set byte select
  vport_we[port] = 0;       // set write enable = 0
  vport_stb[port] = 1;      // set strobe
  run_cycle ();
  vport_stb[port] = 0;      // reset strobe
}


uint32_t vport_read32_wait (int port) {
  while (vport_ack[port] == 0) run_cycle ();
  return vport_rdata[port].read ();
}


uint32_t vport_read32 (int port, uint32_t adr, int bsel = 0xf) {
  vport_read32_start (port, adr, bsel);
  return vport_read32_wait (port);
}


// ***** VPort: Write *****


void vport_write32_start (int port, uint32_t adr, int bsel, uint32_t wdata) {
  vport_adr[port] = adr;      // set address
  vport_bsel[port] = bsel;    // set byte select
  vport_wdata[port] = wdata;  // set data
  vport_we[port] = 1;         // set write enable
  vport_stb[port] = 1;        // set strobe
  run_cycle ();
  vport_we[port] = 0;         // reset write enable
  vport_stb[port] = 0;        // reset strobe
}


void vport_write32_wait (int port) {
  while (vport_ack[port] == 0) run_cycle ();
}


void vport_write32 (int port, uint32_t adr, int bsel, uint32_t wdata) {
  vport_write32_start (port, adr, bsel, wdata);
  vport_write32_wait (port);
}





// ********************** Main *************************************************


int sc_main (int argc, char** argv) {
  int c0, c1;

  // Parse args ...
  pn_parse_enable_trace_core = 1;   // enable '-c' option
  pn_tb_parse_cmd_args(argc, argv);

  // Instantiate and connect DUT ...
  m_membrana_ai i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

  i_dut.clk (clk);
  i_dut.reset (reset);

  i_dut.iport_adr (iport_adr);
  i_dut.iport_bsel (iport_bsel);
  i_dut.iport_rdata (iport_rdata);
  i_dut.iport_stb (iport_stb);
  i_dut.iport_ack (iport_ack);

  i_dut.dport_adr (dport_adr);
  i_dut.dport_bsel (dport_bsel);
  i_dut.dport_rdata (dport_rdata);
  i_dut.dport_wdata (dport_wdata);
  i_dut.dport_stb (dport_stb);
  i_dut.dport_we (dport_we);
  i_dut.dport_ack (dport_ack);
  i_dut.dport_lrsc (dport_lrsc);
  i_dut.dport_amo (dport_amo);

  i_dut.vport_adr (vport_adr);
  i_dut.vport_bsel (vport_bsel);
  i_dut.vport_rdata (vport_rdata);
  i_dut.vport_wdata (vport_wdata);
  i_dut.vport_stb (vport_stb);
  i_dut.vport_we (vport_we);
  i_dut.vport_ack (vport_ack);
  i_dut.vport_lrsc (vport_lrsc);
  i_dut.vport_amo (vport_amo);

  // Open trace file ...
  sc_trace_file *tf = pn_tb_start_trace ();
  i_dut.pn_trace (tf, pn_cfg_vcd_level);

  // Start simulation ...
  sc_start ();
  dport_lrsc[0] = 0;   // set unused signal
  dport_amo[0] = 0;    // set unused signal
  reset = 0;
  run_cycle (4);
  reset = 1;
  run_cycle ();
  reset = 0;
  run_cycle ();

  // ***** Tests (begin) *****

  PN_INFO("DPort: Sequential write & read ...");
  dport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0xf, 0x01234567);
  dport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0xf, 0x89abcdef);
  PN_ASSERT (dport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE    ) == 0x01234567);
  PN_ASSERT (dport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4) == 0x89abcdef);

  PN_INFO("DPort: Byte selection ...");
  dport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0x0, 0x00000000);
  dport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0x9, 0x76543210);
  PN_ASSERT (dport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE    ) == 0x01234567);
  PN_ASSERT (dport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4) == 0x76abcd10);

  PN_INFO("IPort: Sequential read ...");
  PN_ASSERT (iport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE    ) == 0x01234567);
  PN_ASSERT (iport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4) == 0x76abcd10);

  PN_INFO("DPort: Overlapping write & read ...");
  c0 = get_cycles ();
  dport_write32_start (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0xf, 0x11223344);
  dport_write32_start (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0xf, 0x55667788);
  dport_write32_wait (0);
  dport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE    );
  dport_write32_wait (0);
  dport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE + 4);
  PN_ASSERT (dport_read32_wait (0) == 0x11223344);
  run_cycle ();
  PN_ASSERT (dport_read32_wait (0) == 0x55667788);
  c1 = get_cycles ();
  PN_ASSERTM (c1 - c0 == 5, "DPort: Overlapping write + read too slow");

  PN_INFO("IPort: Overlapping read ...");
  c0 = get_cycles ();
  iport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE    );
  iport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE + 4);
  PN_ASSERT (iport_read32_wait (0) == 0x11223344);
  run_cycle ();
  PN_ASSERT (iport_read32_wait (0) == 0x55667788);
  c1 = get_cycles ();
  PN_ASSERTM (c1 - c0 == 3, "IPort: Overlapping read too slow");

  PN_INFO("VPort: Sequential write & read ...");
  vport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0xf, 0x01234567);
  vport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0xf, 0x89abcdef);
  PN_ASSERT (vport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE    ) == 0x01234567);
  PN_ASSERT (vport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4) == 0x89abcdef);

  PN_INFO("VPort: Byte selection ...");
  vport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0x0, 0x00000000);
  vport_write32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0x9, 0x76543210);
  PN_ASSERT (vport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE    ) == 0x01234567);
  PN_ASSERT (vport_read32 (0, PN_CFG_MEMBRANA_EMEM_BASE + 4) == 0x76abcd10);

  PN_INFO("VPort: Overlapping write & read ...");
  c0 = get_cycles ();
  vport_write32_start (0, PN_CFG_MEMBRANA_EMEM_BASE    , 0xf, 0x11223344);
  vport_write32_start (0, PN_CFG_MEMBRANA_EMEM_BASE + 4, 0xf, 0x55667788);
  vport_write32_wait (0);
  vport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE    );
  vport_write32_wait (0);
  vport_read32_start (0, PN_CFG_MEMBRANA_EMEM_BASE + 4);
  PN_ASSERT (vport_read32_wait (0) == 0x11223344);
  run_cycle ();
  PN_ASSERT (vport_read32_wait (0) == 0x55667788);
  c1 = get_cycles ();
  PN_ASSERTM (c1 - c0 == 5, "VPort: Overlapping write + read too slow");

  PN_INFO("UART: Print 'Hello' ...");
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, 'H');
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, 'e');
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, 'l');
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, 'l');
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, 'o');
  dport_write32 (0, PN_CFG_UART_BASE + PN_UART_OFS(txdata), 0x1, '\n');

  // ***** Tests (end) *****

  // Done ...
  PN_INFO("Simulation completed.");
  pn_tb_end_trace ();
  return 0;
}
