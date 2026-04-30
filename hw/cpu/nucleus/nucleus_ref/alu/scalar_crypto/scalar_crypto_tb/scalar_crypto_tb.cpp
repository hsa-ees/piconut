/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Niklas Sirch <niklas.sirch1@tha.de>
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
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR a_in PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

/**
 * @brief Testbench for the scalar_crypto ALU module.
 *
 * This should not test the AES algorithm itself, but only the correct integration
 * into the scalar_crypto module. Math is tested in riscv-arch-tests with ~500 tests.
 */

#include "../scalar_crypto.h"
#include "../../../nucleus_ref_defs.h"
#include "../aes_helpers.h"

#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(rs1_in);
sc_signal<sc_uint<32>> PN_NAME(rs2_in);
sc_signal<sc_uint<7>> PN_NAME(funct7_in);
sc_signal<sc_uint<3>> PN_NAME(funct3_in);
sc_signal<bool> PN_NAME(start_in);
sc_signal<bool> PN_NAME(valid_out);
sc_signal<sc_uint<32>> PN_NAME(res_out);

/**
 * @brief Assert that a signal has the expected value.
 *
 * Reading signal before printf to read in same cycle.
 */
#define ASSERT_SIGNAL_EQ_M(signal, expected, msg)                                   \
    do                                                                              \
    {                                                                               \
        int32_t val = (signal).read();                                              \
        PN_ASSERTF(val == (expected), ("%s wrong, got %#X;%s", #signal, val, msg)); \
    } while(0)

#define ASSERT_SIGNAL_EQ(signal, expected) \
    ASSERT_SIGNAL_EQ_M(signal, expected, "")

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void test_reset()
{
    rs1_in.write(0);
    rs2_in.write(0);
    funct7_in.write(0);
    funct3_in.write(0);
    start_in.write(0);
    reset.write(1);
    run_cycle(1);
    reset.write(0);
    PN_ASSERT(!valid_out.read());
    PN_ASSERT(res_out.read() == 0);
}

void drive_crypt_instr(sc_uint<32> rs1, sc_uint<32> rs2, e_funct7_lower op, uint8_t bs)
{
    rs1_in.write(rs1);
    rs2_in.write(rs2);
    // Construct funct7: [bs (2-bits) | reserved (2-bits) | op (5-bits)]
    sc_uint<7> f7 = 0;
    f7.range(6, 5) = bs;
    f7.range(4, 0) = op;
    funct7_in.write(f7);

    start_in.write(1);
    run_cycle(1); // Transition to S_BUSY
    run_cycle(1); // Transition to S_BUSY
    PN_ASSERT(valid_out.read());
    run_cycle(1); // Transition to S_DONE (valid_out should be high)
    start_in.write(0);
}

void test_aes32_dsi_esi()
{
    drive_crypt_instr(0xDEADBEEF, 0x12345678, FUNCT7L_AES32DSI, 0);
    ASSERT_SIGNAL_EQ(res_out, 0XDEADBE2E);

    test_reset();

    drive_crypt_instr(0xDEADBEEF, 0x12345678, FUNCT7L_AES32ESI, 2);
    ASSERT_SIGNAL_EQ(res_out, 0XDEB5BEEF);
}

void test_byte_selection()
{
    sc_uint<32> test_rs2 = 0x0D0C0B0A;

    uint32_t expected_res[] = {0x67, 0x2B00, 0xFE0000, 0XD7000000};

    for(int bs = 0; bs < 4; bs++)
    {
        drive_crypt_instr(0x00000000, test_rs2, FUNCT7L_AES32ESI, bs);

        ASSERT_SIGNAL_EQ_M(res_out, expected_res[bs], (" (bs=" + std::to_string(bs) + ")").c_str());
        test_reset();
    }
}

void test_aes_middle_rounds()
{
    drive_crypt_instr(0x00000000, 0x00000000, FUNCT7L_AES32ESMI, 0);
    ASSERT_SIGNAL_EQ(res_out, 0xA56363C6);

    drive_crypt_instr(0x00000000, 0x00000000, FUNCT7L_AES32DSMI, 0);
    ASSERT_SIGNAL_EQ(res_out, 0X50A7F451);
}

void test_state_machine()
{
    reset.write(0);
    start_in.write(0);
    run_cycle(1);
    PN_ASSERT(!valid_out.read());

    start_in.write(1);
    run_cycle(1);
    PN_ASSERT(!valid_out.read());

    run_cycle(1);
    PN_ASSERT(valid_out.read());

    run_cycle(1); // DONE and already overwritten by regfile
    PN_ASSERT(!valid_out.read());

    start_in.write(0);
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("scalar_crypto_tb");

    m_scalar_crypto i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.rs1_in(rs1_in);
    i_dut.rs2_in(rs2_in);
    i_dut.funct7_in(funct7_in);
    i_dut.funct3_in(funct3_in);

    i_dut.start_in(start_in);
    i_dut.valid_out(valid_out);
    i_dut.res_out(res_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    const std::vector<std::function<void()>> tests = {
        [] { test_state_machine(); },
        [] { test_aes32_dsi_esi(); },
        [] { test_byte_selection(); },
        [] { test_aes_middle_rounds(); },
    };

    for(const auto& test : tests)
    {
        test();
        test_reset();
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}