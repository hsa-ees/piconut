/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                     2024 Lukas Bauer <lukas.bauer1@tha.de>
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

#include "base.h"

#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <systemc.h>

// *********** Dynamic Configuration ************

int pn_cfg_vcd_level = 0;
int pn_cfg_insn_trace = 0;
bool pn_cfg_disable_cache = 0;
bool pn_cfg_debug_mode = 0;

bool pn_disable_assert_abort = 0;
bool pn_assert_use_abort = 0;
bool pn_cfg_trace_core = 0; // enable to trace core requests
bool pn_parse_enable_trace_core = 0; // enable to trace core requests

bool pn_cfg_enable_application_path = 0;
std::string pn_cfg_application_path = "";

// **************** Tracing *********************

bool pn_trace_verbose = false;

std::string pn_get_trace_name(sc_object *obj, const char *name, int dim, int arg1, int arg2)
{
    std::string ret;

    if (dim < 0 || dim > 2)
        PN_ERRORF(("pn_get_trace_name: Parameter dim outside of range (0-2): %d", dim));

    // Read full object name and get the base module name
    ret = obj->name();
    ret = ret.substr(0, ret.find_last_of('.') + 1);

    // Add name...
    ret += name;

    // Add first dimension...
    if (dim > 0)
        ret += "(" + std::to_string(arg1) + ")";

    // Add second dimension...
    if (dim == 2)
        ret += "(" + std::to_string(arg2) + ")";

    return ret;
}

// **************** Testbench helpers ***********

sc_trace_file *pn_trace_file = NULL;

sc_trace_file *pn_tb_start_trace(const char *filename)
{
    if (pn_cfg_vcd_level > 0)
    {
        pn_trace_file = sc_create_vcd_trace_file(filename);
    }
    else
    {
        pn_trace_file = NULL;
    }
    return pn_trace_file;
}

void pn_tb_parse_cmd_args(const int argc, char **argv)
{
    int arg, cfg_help = 0;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-')
    {
        switch (argv[arg][1])
        {
        case 't':
            pn_cfg_vcd_level = MAX(0, MIN(9, argv[arg][2] - '0'));
            break;
        case 'h':
            cfg_help = 1;
            break;
        case 'a':
            pn_disable_assert_abort = 1;
            break;
        case 'A':
            pn_assert_use_abort = 1;
            break;
        case 'c':
            if (pn_parse_enable_trace_core)
            {
                pn_cfg_trace_core = 1;
            }
            else
            {
                cfg_help = 1;
                printf("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
                printf("PN_ERROR: Core trace option is not enabled.\n");
            }
            break;
        default:
            printf("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
            arg = argc;
        }
        arg++;
    }

    // parse application path
    if(pn_cfg_enable_application_path &&
        argc > 1)
    {
        char* last_arg = argv[argc - 1];
        if(last_arg[0] != '-')
        {
            pn_cfg_application_path = std::string(last_arg);
        }
    }

    if(cfg_help)
    {
        if(pn_cfg_enable_application_path)
        {
            printf("Usage: %s [<options>] <path-to-executable>\n", argv[0]);
        }
        else
        {
            printf("Usage: %s [<options>]\n", argv[0]);
        }
        printf("\n"
               "Options:\n"
               "  -t<n>: set VCD trace level (0 = no trace file; >0 == trace file; default = 0)\n"
               "  -a: disable abort on assertion failure\n"
               "  -A: uses abort() instead of exit() on assertion failure\n"
               "  -h: show this help\n");
        if(pn_parse_enable_trace_core)
        {
            puts("  -c: enable core trace option\n");
        }
            puts("  -t<n>: set VCD trace level (0 = no trace file; >0 == trace file; default = 0)\n");
        exit(3);
    }

    // Printing informa1tion about the configuration set

    fprintf(stdout, "\n");

    // Prints information about set configuration
    if(pn_cfg_vcd_level == 0)
    {
        fprintf(stdout, "Tracing disabled\n");
    }
    else
    {
        fprintf(stdout, "Tracing enabled Level: %d\n", pn_cfg_vcd_level);
    }

    // Prints information about Assertion handling
    if(pn_disable_assert_abort)
    {
        fprintf(stdout, "Abort on Assertion Failure disabled\n");
    }

    // Prints information about Assertion using abort

    if(pn_assert_use_abort)
    {
        fprintf(stdout, "Using abort() on Assertion Failure\n");
    }

    fprintf(stdout, "\n");
}

void pn_tb_end_trace()
{
    if(pn_trace_file)
        sc_close_vcd_trace_file(pn_trace_file);
}

char* pn_tb_printf(const char* format, ...)
{
    static char buf[200];

    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    return buf;
}

void pn_tb_assert(bool cond, const char* msg, const char* filename, const int line)
{
    if(!cond)
    {
        fprintf(stderr, "ASSERTION FAILURE: %s, %s:%i", sc_time_stamp().to_string().c_str(), filename, line);
        if(msg)
            fprintf(stderr, ": %s\n", msg);
        else
            fprintf(stderr, "\n");
        if(!pn_disable_assert_abort)
        {

            if(pn_trace_file)
                sc_close_vcd_trace_file(pn_trace_file);

            if(pn_assert_use_abort)
            {
                abort();
            }
            else
            {
                exit(1);
            }
        }
    }
}

void pn_tb_info(const char* msg, const char* filename, const int line)
{
    int time_size = sc_time_stamp().to_double() == 0.0 ? 1 : 15;
    fprintf(stdout, "(INFO): %*s, %s:%i:   %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
}

void pn_tb_warning(const char* msg, const char* filename, const int line)
{
    int time_size = sc_time_stamp().to_double() == 0.0 ? 1 : 12;
    fprintf(stdout, "(WARNING): %*s, %s:%i: %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
}

void pn_tb_error(const char* msg, const char* filename, const int line)
{
    int time_size = sc_time_stamp().to_double() == 0.0 ? 1 : 14;
    fprintf(stderr, "(ERROR): %*s, %s:%i: %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
    if(pn_trace_file)
        sc_close_vcd_trace_file(pn_trace_file);
    exit(3);
}

void pn_info_file(const char* msg, const char* filename, const int line, FILE* core_dump_file)
{
    int time_size = sc_time_stamp().to_double() == 0.0 ? 1 : 15;
    fprintf(core_dump_file, "(INFO): %*s, %s:%i:   %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
}
// **************** DisAss **********************

#ifndef __SYNTHESIS__
char* pn_dis_ass(sc_uint<32> insn)
{
    static char ret[80] = "";
    sc_uint<32> opcode, funct3, funct7, rs1, rs2, rd, bit30, bit20, bit21, bit25, bit28, bit29, itype, utype, btype, jtype, stype;
    sc_uint<32> inst = insn;

    opcode = insn & 0x7f;
    funct3 = (insn >> 12) & 0x7;
    funct7 = (insn >> 25);

    rd = (insn >> 7) & 0x1f;
    rs1 = (insn >> 15) & 0x1f;
    rs2 = (insn >> 20) & 0x1f;

    bit20 = (insn >> 20) & 0x1;
    bit21 = (insn >> 21) & 0x1;
    bit25 = (insn >> 25) & 0x1;
    bit28 = (insn >> 28) & 0x1;
    bit29 = (insn >> 28) & 0x1;
    bit30 = (insn >> 30) & 0x1;

    itype = ((insn >> 20) ^ 0x800) - 0x800;
    utype = insn & 0xFFFFF000;
    btype = ((sc_uint<32>)(inst[31], inst[7], inst(30, 25), inst(11, 8), 0) ^ 0x1000) - 0x1000;
    jtype = ((sc_uint<32>)(inst[31], inst(19, 12), inst[20], inst(30, 25), inst(24, 21), 0) ^ 0x100000) - 0x100000;
    stype = ((((sc_uint<32>)(inst(31, 25), 0, 0, 0, 0, 0)).value() + rd) ^ 0x800) - 0x800;

    strcpy(ret, "  ");

    // ALU instructions...
    if(opcode == 0x13)
    { // OP_IMM
        const char* table[] = {"addi", "slli", "slti", "sltiu", "xori", "srli", "ori", "andi"};
        if(funct3 == 5 || funct3 == 1)
        {
            if(bit30)
                table[funct3] = "srai";
            itype &= 0x1F; // shamt
        }
        sprintf(ret + 2, "%s r%u, r%u, 0x%x", table[funct3], (uint)rd, (uint)rs1, (uint)itype);
    }
    else if(opcode == 0x33 && !bit25)
    { // OP (Bit 25 is on for M-Extension (DIV/MUL...)
        const char* table[] = {"add", "sll", "slt", "sltu", "xor", "srl", "or", "and"};
        if(funct3 == 0)
        {
            if(bit30)
                table[funct3] = "sub";
        }
        else if(funct3 == 5)
        {
            if(bit30)
                table[funct3] = "sra";
        }
        sprintf(ret + 2, "%s r%u, r%u, r%u", table[funct3], (uint)rd, (uint)rs1, (uint)rs2);
    }
    else if((opcode == 0x33 && bit25))
    { // OP - M-Extension
        static const char* table[] = {"mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"};
        sprintf(ret + 2, "%s r%u, r%u, r%u", table[funct3], (uint)rd, (uint)rs1, (uint)rs2);
    }
    else if(opcode == 0x17)
    { // AUIPC
        sprintf(ret + 2, "auipc r%u, 0x%x", (uint)rd, (uint)utype);
    }
    else if(opcode == 0x37)
    { // LUI
        sprintf(ret + 2, "lui r%u, 0x%x", (uint)rd, (uint)utype);
    }
    else if(opcode == 0x63)
    { // BRANCH
        const char* table[] = {"beq", "bneq", "INV_SUB", "INV_SUB", "blt", "bge", "bltu", "bgeu"};
        sprintf(ret + 2, "%s r%u, r%u, 0x%x", table[funct3], (uint)rs1, (uint)rs2, (uint)btype);
    }
    else if(opcode == 0x6F)
    { // JAL
        sprintf(ret + 2, "jal r%u, 0x%x", (uint)rd, (uint)jtype);
    }
    else if(opcode == 0x67)
    { // JALR
        sprintf(ret + 2, "jalr r%u, r%u, 0x%x", (uint)rd, (uint)rs1, (uint)itype);
    }
    else if(opcode == 0x03)
    { // LOAD
        static const char* table[] = {"lb", "lh", "lw", "INV_SUB", "lbu", "lhu"};
        sprintf(ret + 2, "%s r%u, 0x%x(r%u)", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
    }
    else if(opcode == 0x23)
    { // STORE
        static const char* table[] = {"sb", "sh", "sw"};
        sprintf(ret + 2, "%s r%u, 0x%x(r%u)", table[funct3], (uint)rs2, (uint)stype, (uint)rs1);
    }
    else if(opcode == 0x73)
    { // SYSTEM
        const char* table[] = {"ecall", "csrrw", "csrrs", "csrrc", "INV_SUB", "csrrwi", "csrrsi", "csrrci"};
        if(bit25 && bit28)
            table[0] = "sfence.vma";
        if(bit21)
            table[0] = bit28 ? bit29 ? "mret" : "sret" : "uret";
        if(bit20)
            table[0] = "ebreak";
        if(funct3 == 0)
            sprintf(ret + 2, "%s", table[funct3]);
        else if(funct3 < 4)
            sprintf(ret + 2, "%s r%u, 0x%x, r%u", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
        else
            sprintf(ret + 2, "%s r%u, 0x%x, 0x%x", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
    }
    else if(opcode == 0x0F)
    {
        sprintf(ret + 2, "fence");
    }
    else if(opcode == 0xB)
    { // PARA
        static const char* table[] = {"halt", "cinvalidate", "cwriteback", "cflush"};
        if(funct3 == 0)
            sprintf(ret + 2, "%s", table[funct3]);
        else
            sprintf(ret + 2, "%s 0x%x(r%u) ", table[funct3], (uint)itype, (uint)rs1);
    }
    else if(opcode == 0x2F)
    {                                         // AMO
        if(funct3 == 2 && (funct7 >> 2) == 2) // LR.W
            sprintf(ret + 2, "lr.w r%u, (r%u)", (uint)rd, (uint)rs1);
        else if(funct3 == 2 && (funct7 >> 2) == 3) // SC.W
            sprintf(ret + 2, "sc.w r%u, r%u, (r%u) ", (uint)rd, (uint)rs2, (uint)rs1);
        else
            sprintf(ret + 2, "AMO_INVALID r%u, r%u, (r%u) ", (uint)rd, (uint)rs2, (uint)rs1);
    }
    else
        sprintf(ret, "? 0x%08x ?", (uint)insn);

    return ret;
}
#endif //__SYNTHESIS__