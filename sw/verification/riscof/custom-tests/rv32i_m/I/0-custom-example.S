// Copied from riscv-arch-test/riscv-test-suite/rv32i_m/I/add-01.S:
// Copyright (c) 2020. RISC-V International. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#include "model_test.h"
#include "arch_test.h"
RVTEST_ISA("RV32I")

.section .text.init
.globl rvtest_entry_point
rvtest_entry_point:
RVMODEL_BOOT
RVTEST_CODE_BEGIN

#ifdef TEST_CASE_1

RVTEST_CASE(0,"//check ISA:=regex(.*32.*);check ISA:=regex(.*I.*);def TEST_CASE_1=True;",custom-example)

RVTEST_SIGBASE( x3,signature_x3_1)

inst_0:
// rs2 == rd != rs1, rs1==x4, rs2==x24, rd==x24, rs1_val > 0 and rs2_val > 0, rs2_val == 1, rs1_val == (2**(xlen-1)-1), rs1_val != rs2_val, rs1_val == 2147483647
// opcode: add ; op1:x4; op2:x24; dest:x24; op1val:0x7fffffff;  op2val:0x1
TEST_RR_OP(add, x24, x4, x24, 0x80000000, 0x7fffffff, 0x1, x3, 0, x18)
#endif


RVTEST_CODE_END
RVMODEL_HALT

RVTEST_DATA_BEGIN
.align 4

rvtest_data:
.word 0xbabecafe
.word 0xbabecafe
.word 0xbabecafe
.word 0xbabecafe
RVTEST_DATA_END


RVMODEL_DATA_BEGIN
rvtest_sig_begin:
sig_begin_canary:
CANARY;


signature_x3_0:
    .fill 0*(XLEN/32),4,0xdeadbeef


signature_x3_1:
    .fill 17*(XLEN/32),4,0xdeadbeef


signature_x8_0:
    .fill 16*(XLEN/32),4,0xdeadbeef


signature_x1_0:
    .fill 512*(XLEN/32),4,0xdeadbeef


signature_x1_1:
    .fill 43*(XLEN/32),4,0xdeadbeef

#ifdef rvtest_mtrap_routine

tsig_begin_canary:
CANARY;
mtrap_sigptr:
    .fill 64*(XLEN/32),4,0xdeadbeef
tsig_end_canary:
CANARY;

#endif

#ifdef rvtest_gpr_save

gpr_save:
    .fill 32*(XLEN/32),4,0xdeadbeef

#endif

sig_end_canary:
CANARY;
rvtest_sig_end:
RVMODEL_DATA_END
