/**
 * @file csr_master.h
 * @brief This file contains the definition of the csr_master module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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
 * @fn SC_MODULE(m_csr_master)
 *
 * The CSR master module is an interface between the controller and the csr bus.
 * It helps generating the correct write data on the CSR-bus (`csr_bus_wdata_out`).
 * The `csr_bus_wdata_out` signal generation can either be based on a
 * general-purpose-register (GPR) or an immediate value. By default the input
 * register is used as the source. To use immediate value `imm_en_in` must be
 * set to `1` and the desired immediate value must be set at `imm_in`. The
 * immediate value gets zero extended to match `PN_CFG_CSR_BUS_DATA_WIDTH`.
 *
 * There are three write modes selected by the write_mode signal:
 *  1. Write: This mode sets the `csr_bus_wdata_out` to the source value.
 *  2. Set:   In this mode, `csr_bus_wdata_out` is set to the source value and
 *            OR-masked with the value of `csr_bus_rdata_in`.
 *  3. Clear: In this mode, `csr_bus_wdata_out` is set to the source value and
 *            AND-masked with the value of `csr_bus_rdata_in`.
 *
 * Write mode signal decoding:
 * | Signal | Meaning          |
 * |--------|------------------|
 * | 00     | Write            |
 * | 01     | Set (or-mask)    |
 * | 10     | Clear (and-mask) |
 * | 11     | Reserved         |
 *
 * @par Ports:
 * @param[in] csr_bus_rdata_in <`PN_CFG_CSR_BUS_DATA_WIDTH`> Csr bus read data.
 * @param[in] source_reg_in <32> Source register, that is written to the csr bus.
 * @param[in] imm_en_in Enable signal for immediate value generation.
 * @param[in] imm_in <5> Immediate value.
 * @param[in] write_mode_in <2> Write mode of the next write operation to the csr bus.
 * @param[out] csr_bus_wdata_out <`PN_CFG_CSR_BUS_DATA_WIDTH`> Csr bus write data.
 *
 */

#ifndef __CSR_MASTER_H__
#define __CSR_MASTER_H__

#include <systemc.h>
#include <piconut.h>

#include "typedef.h"

#include <cstdint>

#include "csr.h"


SC_MODULE(m_csr_master)
{
public:
    sc_in<sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_rdata_in);
    sc_in<sc_uint<32>> PN_NAME(source_reg_in);

    sc_in<bool> PN_NAME(imm_en_in);
    sc_in<sc_uint<5>> PN_NAME(imm_in);
    sc_in<sc_uint<2>> PN_NAME(write_mode_in);

    sc_out<sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_wdata_out);

    /* Constructor... */
    SC_CTOR(m_csr_master)
    {
        SC_METHOD(proc_cmb);
        sensitive << csr_bus_rdata_in
                  << source_reg_in
                  << write_mode_in
                  << imm_en_in
                  << imm_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    void proc_cmb();
};

#endif //__PC_H__
