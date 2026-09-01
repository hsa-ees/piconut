/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <christian.zellinger1@tha.de>
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

// TBD: Move pn_csr.* to hw/common/cpu and adapt other Nuclei to use it.
// TBD: Improve the simulation performance: fewer processes, fewer signals?

// Note: Presently, Nucleus AI is not using the module, only some defs.


/**
 *
 * @file csr.h
 * @brief This file contains the definition of the CSR module.
 * @author Johannes Hofmann, Gundolf Kiefer
 */


#ifndef __PN_CSR_H__
#define __PN_CSR_H__

#include <piconut.h>





// ********************** Constants & Types ************************************


#define PN_CSR_BUS_ADR_WIDTH 12U
  ///< @brief Width of the address in the CSR bus.

#define PN_CSR_BUS_DATA_WIDTH 32U
  ///< @brief Width of the data in the CSR bus.





// ********************** CSR Module *******************************************


/**
 * @brief CSR module for Nucleus implementations.
 *
 * This module implements the Control and Status Registers (CSR) which have effect
 * in the Nucleus itself, like for debug purposes, processor status etc.
 * The registers are connected to the CSR bus for basic read/write operations.
 * The read/write protection, if it exists, is implemented separately for each
 * register in its own thread.
 *
 * Present CSRs in this module:
 *
 * | Address | Name      |
 * |---------|-----------|
 * | 0x300   | MSTATUS   |
 * | 0x301   | MISA      |
 * | 0x304   | MIE       |
 * | 0x305   | MTVEC     |
 * | 0x341   | MEPC      |
 * | 0x342   | MCAUSE    |
 * | 0x343   | MTVAL     |
 * | 0x344   | MIP       |
 * | 0x7b0   | DCSR      |
 * | 0x7b1   | DPC       |
 * | 0x7b2   | DSCRATCH0 |
 * | 0x7b3   | DSCRATCH1 |
 *
 * @par Interrupt Handling:
 * This module implements RISC-V machine-mode interrupt handling according to the specification.
 * - MIE (0x304): Machine Interrupt Enable register controls which interrupt types are enabled
 * - MTVEC (0x305): Machine Trap Vector register contains the base address of the interrupt handler
 * - MEPC (0x341): Machine Exception Program Counter stores the return address for exceptions/interrupts
 * - MCAUSE (0x342): Machine Cause register identifies the exception/interrupt type
 * - MTVAL (0x343): Machine Trap Value register provides additional trap information
 * - MIP (0x344): Machine Interrupt Pending register shows pending interrupt sources
 *
 * Supported interrupt sources:
 * - MSIP: Machine Software Interrupt (via CLINT)
 * - MTIP: Machine Timer Interrupt (via CLINT)
 * - MEIP: Machine External Interrupt (platform-specific)
 *
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] csr_bus_read_en_in CSR-bus read enable.
 * @param[in] csr_bus_write_en_in CSR-bus write enable.
 * @param[in] csr_bus_adr_in <`PN_CSR_BUS_ADR_WIDTH`> CSR-bus address.
 * @param[in] csr_bus_wdata_in <`PN_CSR_BUS_DATA_WIDTH`> CSR-bus write data.
 * @param[out] csr_bus_rdata_out <`PN_CSR_BUS_DATA_WIDTH`> CSR-bus read data (must be 0, if the slave module is not addressed.
 * @param[in] pc_in <32> Program Counter.
 * @param[in] debug_level_enter_ebreak_in Debug level enter request caused by ebreak.
 * @param[in] debug_level_enter_haltrequest_in Debug level enter request caused by halt request.
 * @param[in] debug_level_enter_step_in Debug level enter request caused by step.
 * @param[in] debug_level_leave_in Debug level leave request.
 * @param[out] dpc_out <32> Full CSR Debug Program Counter.
 * @param[out] debug_level_enter_out Debug level enter request.
 * @param[out] debug_step_out Debug step signal.
 * @param[in] msip_in Machine Software Interrupt Pending from CLINT.
 * @param[in] mtip_in Machine Timer Interrupt Pending from CLINT.
 * @param[in] meip_in Machine External Interrupt Pending (platform-specific).
 * @param[in] mret_in Machine Return instruction execution signal.
 * @param[in] interrupt_in General interrupt signal.
 * @param[out] interrupt_pending_out Combined interrupt pending output signal.
 * @param[out] mip_msip_out Machine Software Interrupt Pending output.
 * @param[out] mie_msie_out Machine Software Interrupt Enable output.
 * @param[out] mstatus_mie_out Machine Status Global Interrupt Enable output.
 * @param[out] mtvec_trap_address_out Machine Trap Vector address output.
 * @param[out] mip_mtip_out Machine Timer Interrupt Pending output.
 * @param[out] mip_meip_out Machine External Interrupt Pending output.
 * @param[out] mie_mtie_out Machine Timer Interrupt Enable output.
 * @param[out] mie_meie_out Machine External Interrupt Enable output.
 * @param[out] mepc_out Machine Exception Program Counter output.
 */
SC_MODULE(m_pn_csr) {
  public:

    // Ports: General ...
    sc_in_clk     PN_NAME(clk);
    sc_in<bool>   PN_NAME(reset);

    // Ports: CSR bus...
    sc_in<bool>                             PN_NAME(csr_bus_read_en_in);
    sc_in<bool>                             PN_NAME(csr_bus_write_en_in);
    sc_in<sc_uint<2>>                       PN_NAME(csr_bus_write_mode_in);
    sc_in<sc_uint<PN_CSR_BUS_ADR_WIDTH>>    PN_NAME(csr_bus_adr_in);
    sc_in<sc_uint<PN_CSR_BUS_DATA_WIDTH>>   PN_NAME(csr_bus_wdata_in);
    sc_out<sc_uint<PN_CSR_BUS_DATA_WIDTH>>  PN_NAME(csr_bus_rdata_out);

    // Ports: Inputs from CPU ...
    //   The following signals must be provided by the CPU/Nucleus.

    // ... program counter (PC) ...
    sc_in<sc_uint<32>> PN_NAME(pc_in);

    // ... external interrupts ...
    sc_in<bool> PN_NAME(msip_in);       // Machine Software Interrupt from CLINT
    sc_in<bool> PN_NAME(mtip_in);       // Machine Timer Interrupt from CLINT
    sc_in<bool> PN_NAME(meip_in);       // Machine External Interrupt (platform-specific)

    sc_in<bool> PN_NAME(mret_in);       // Machine Return instruction executed

    sc_in<bool> PN_NAME(interrupt_in);  // General interrupt signal;
      // TBD: Is interrupt_in necessary? What is the difference between interrupt_in and (msip_in | mtip_in | meip_in)?

    // ... debugging ...
    sc_in<bool> PN_NAME(debug_level_enter_ebreak_in);
    sc_in<bool> PN_NAME(debug_level_enter_haltrequest_in);
    sc_in<bool> PN_NAME(debug_level_enter_step_in);
    sc_in<bool> PN_NAME(debug_level_leave_in);

    // Ports: Outputs to CPU ...
    //   The following signals are provided for the CPU/Nucleus.

    // ... external interrupts ...
    sc_out<bool> PN_NAME(mstatus_mie_out);  // Machine Status Global Interrupt Enable output.
    sc_out<bool> PN_NAME(mie_msie_out);     // Machine Software Interrupt Enable output
    sc_out<bool> PN_NAME(mie_mtie_out);     // Machine Timer Interrupt Enable output
    sc_out<bool> PN_NAME(mie_meie_out);     // Machine External Interrupt Enable (MEIE)

    sc_out<bool> PN_NAME(mip_msip_out);     // Machine Software Interrupt Pending (MSIP)
    sc_out<bool> PN_NAME(mip_mtip_out);     // Machine Timer Interrupt Pending (MSIP)
    sc_out<bool> PN_NAME(mip_meip_out);     // Machine External Interrupt Pending (MSIP)

    sc_out<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mtvec_trap_address_out);  // Machine Trap Vector address
    sc_out<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mepc_out);  // Machine Exception Program Counter (MEPC)

    sc_out<bool> PN_NAME(interrupt_pending_out);
      // TBD: Is this necessary? What is the difference between this and (msip_out | mtip_out | meip_out)?

    // ... debugging ...
    sc_out<bool> PN_NAME(debug_level_enter_out);
    sc_out<bool> PN_NAME(debug_step_out);

    sc_out<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(dpc_out);   // Debug Program Counter (DPC)

    // Constructor...
    SC_CTOR(m_pn_csr) {

      // CSR bus ...
      SC_CTHREAD(proc_clk_bus_read, clk.pos());
      reset_signal_is(reset, true);

      // Registers ...
      SC_CTHREAD(proc_clk_mstatus, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_misa, clk.pos());
      reset_signal_is(reset, true);

      // Interrupt ...
      SC_CTHREAD(proc_clk_mie, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_mtvec, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_mepc, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_mcause, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_mtval, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_mip, clk.pos());
      reset_signal_is(reset, true);

      SC_METHOD(proc_cmb_update_interrupt_pending);
      sensitive <<mip_msip_out
                <<mie_msie_out
                <<mip_mtip_out
                <<mie_mtie_out
                <<mip_meip_out
                <<mie_meie_out
                <<mstatus_mie_out
                <<mret_in
                <<mcause_reg;

      SC_CTHREAD(proc_clk_dcsr, clk.pos());
      reset_signal_is(reset, true);
      SC_METHOD(proc_cmb_dcsr);
      sensitive << dcsr_reg
                << debug_level_reg;
      SC_CTHREAD(proc_clk_dpc, clk.pos());
      reset_signal_is(reset, true);
      SC_METHOD(proc_cmb_dpc);
      sensitive << dpc_reg;
      SC_CTHREAD(proc_clk_dscratch0, clk.pos());
      reset_signal_is(reset, true);
      SC_CTHREAD(proc_clk_dscratch1, clk.pos());
      reset_signal_is(reset, true);

      // Control signals
      SC_CTHREAD(proc_clk_debug_level, clk.pos());
      reset_signal_is(reset, true);

      SC_METHOD(proc_cmb_debug_level_enter);
      sensitive << s_debug_level_enter;

      // Internal signals
      SC_METHOD(proc_cmb_debug_mode_enter);
      sensitive << debug_level_enter_ebreak_in
                << debug_level_enter_haltrequest_in
                << debug_level_enter_step_in;

      SC_METHOD(proc_cmb_mtvec_address);
      sensitive << mtvec_reg << mcause_reg << interrupt_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    // CSR bus
    void proc_clk_bus_read();

    // Registers
    void proc_clk_mstatus();
    void proc_clk_misa();

    void proc_clk_dcsr();
    void proc_cmb_dcsr();
    void proc_clk_dpc();
    void proc_cmb_dpc();
    void proc_clk_dscratch0();
    void proc_clk_dscratch1();

    // Interrupts
    void proc_clk_mie();
    void proc_clk_mtvec();
    void proc_clk_mepc();
    void proc_clk_mcause();
    void proc_clk_mtval();
    void proc_clk_mip();
    void proc_cmb_mtvec_address();

    // MRET handling
    void proc_cmb_update_interrupt_pending();

    // Privilege level
    void proc_cmb_privilege_level();

    // Debug level
    void proc_clk_debug_level();
    void proc_cmb_debug_level();

    void proc_cmb_debug_level_enter();

    // Internal status signals
    void proc_cmb_debug_mode_enter();

protected:

    // Registers
    enum class e_mstatus_fieldpos : int {
      WPRI = 0,
      SIE = 1,
      WPRI1 = 2,
      MIE = 3,
      WPRI2 = 4,
      SPIE = 5,
      UBE = 6,
      MPIE = 7,
      SPP = 8,
      VS = 9,
      MPP = 11,
      FS = 13,
      XS = 15,
      MPRV = 17,
      SUM = 18,
      MXR = 19,
      TVM = 20,
      TW = 21,
      TSR = 22,
      WPRI3 = 23,
      SD = 31,
    };

    enum class e_misa_mxl : int {
      BITS_32 = 1,
      BITS_64 = 2,
      BITS_128 = 3,
    };

    enum class e_misa_extensions : int {
      A = 0,
      B = 1,
      C = 2,
      D = 3,
      E = 4,
      F = 5,
      H = 7,
      I = 8,
      M = 12,
      N = 13,
      P = 15,
      Q = 16,
      S = 18,
      U = 20,
      V = 21,
      X = 23,
    };

    enum class e_dcsr_fieldpos : int {
      PRV = 0,
      STEP = 2,
      NMIP = 3,
      MPRVEN = 4,
      CAUSE = 6,
      STOPTIME = 9,
      STOPCOUNT = 10,
      STEPIE = 11,
      EBREAKU = 12,
      EBREAKS = 13,
      EBREAKM = 15,
      XDEBUGVER = 28,
    };

    enum class e_dcsr_prv : int {
      USER = 0,
      SUPERVISOR = 1,
      MACHINE = 3,
    };

    enum class e_dcsr_cause : int {
      NONE = 0,
      EBREAK = 1,
      TRIGGER = 2, // Not implemented
      HALTREQUEST = 3,
      SINGLESTEP = 4,       // Not implemented
      RESETHALTREQUEST = 5, // Not implemented
    };

    enum class e_dcsr_xdebugver : int {
      NOT_SUPPORTED = 0,
      AS_DESCRIBED = 4,
      CUSTOM = 15,
    };

  protected:

    // Registers
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mstatus_reg);
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(misa_reg);

    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(dcsr_reg);
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(dpc_reg);
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(dscratch0_reg);
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(dscratch1_reg);

    // Interrupt control registers
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mie_reg);      // Machine Interrupt Enable
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mtvec_reg);    // Machine Trap Vector
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mepc_reg);     // Machine Exception Program Counter
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mcause_reg);   // Machine Cause Register
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mtval_reg);    // Machine Trap Value
    sc_signal<sc_uint<PN_CSR_BUS_DATA_WIDTH>> PN_NAME(mip_reg);      // Machine Interrupt Pending

    // Internal registers
    sc_signal<bool> PN_NAME(debug_level_reg);

    // Internal status signals
    sc_signal<bool> PN_NAME(s_debug_level_enter);
};


#endif // __PN_CSR_H__
