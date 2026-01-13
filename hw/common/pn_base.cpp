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

#include "pn_base.h"

#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>

#include <systemc.h>

/// @brief Minimum of A and B.
#define MIN(A, B) ((A) < (B) ? (A) : (B))
/// @brief Maximum of A and B.
#define MAX(A, B) ((A) > (B) ? (A) : (B))

// *********** Dynamic Configuration ************

int pn_cfg_vcd_level = 0;
int pn_cfg_itrace_level = 0;
bool pn_cfg_disable_cache = 0;
bool pn_cfg_debug_mode = 0;
bool pn_cfg_dump_memory = 0;

bool pn_disable_assert_abort = 0;
bool pn_assert_use_abort = 0;
bool pn_cfg_trace_core = 0;          // enable to trace core requests
bool pn_parse_enable_trace_core = 0; // enable to trace core requests

bool pn_cfg_enable_application_path = 0;
std::string pn_cfg_application_path = "";
std::string pn_cfg_signature_path = "";
uint64_t pn_cfg_signature_begin_addr = 0;
uint64_t pn_cfg_signature_end_addr = 0;

// **************** Tracing *********************

bool pn_trace_verbose = false;

std::string pn_get_trace_name(sc_object* obj, const char* name, int dim, int arg1, int arg2)
{
    std::string ret;

    if(dim < 0 || dim > 2)
        PN_ERRORF(("pn_get_trace_name: Parameter dim outside of range (0-2): %d", dim));

    // Read full object name and get the base module name
    ret = obj->name();
    ret = ret.substr(0, ret.find_last_of('.') + 1);

    // Add name...
    ret += name;

    // Add first dimension...
    if(dim > 0)
        ret += "(" + std::to_string(arg1) + ")";

    // Add second dimension...
    if(dim == 2)
        ret += "(" + std::to_string(arg2) + ")";

    return ret;
}

// **************** Testbench helpers ***********

sc_trace_file* pn_trace_file = NULL;

sc_trace_file* pn_tb_start_trace(const char* filename)
{
    if(pn_cfg_vcd_level > 0)
    {
        pn_trace_file = sc_create_vcd_trace_file(filename);
    }
    else
    {
        pn_trace_file = NULL;
    }
    return pn_trace_file;
}

enum ARGS_OPTIONS
{
    ARGS_OPTION_DUMP = 256,
    ARGS_OPTION_TRACE_CORE,
    ARGS_OPTION_NO_ASSERT_ABORT,
    ARGS_OPTION_ASSERT_USE_ABORT,
    ARGS_OPTION_SIGNATURE_PATH,
    ARGS_OPTION_SIGNATURE_BEGIN,
    ARGS_OPTION_SIGNATURE_END,
};

void pn_tb_parse_cmd_args(const int argc, char** argv)
{
    int cfg_help = 0;

    // Use environment variables as a base that can be overridden by command line
    const char* env_trace_level = getenv("SIM_TRACE_LEVEL");
    if(env_trace_level)
    {
        pn_cfg_vcd_level = MAX(0, MIN(9, atoi(env_trace_level)));
    }

    const char* env_dump_memory = getenv("SIM_DUMP");
    if(env_dump_memory)
    {
        pn_cfg_dump_memory = 1;
    }

    // Define long options
    static struct option long_options[] = {
        {"vcd-level", required_argument, 0, 't'},
        {"itrace-level", required_argument, 0, 'i'},
        {"help", no_argument, 0, 'h'},
        {"dump", no_argument, 0, ARGS_OPTION_DUMP},
        {"trace-core", no_argument, 0, ARGS_OPTION_TRACE_CORE},
        {"signature-path", required_argument, 0, ARGS_OPTION_SIGNATURE_PATH},
        {"signature-begin", required_argument, 0, ARGS_OPTION_SIGNATURE_BEGIN},
        {"signature-end", required_argument, 0, ARGS_OPTION_SIGNATURE_END},
        {"no-assert-abort", no_argument, 0, ARGS_OPTION_NO_ASSERT_ABORT},
        {"assert-use-abort", no_argument, 0, ARGS_OPTION_ASSERT_USE_ABORT},
        {0, 0, 0, 0} // End of options
    };

    int c;
    int option_index = 0;
    // Parse command line...
    while((c = getopt_long(argc, argv, "t:i:haAcd", long_options, &option_index)) != -1)
    {
        switch(c)
        {
            case 't': // trace
                pn_cfg_vcd_level = MAX(0, MIN(9, optarg[0] - '0'));
                if(strlen(optarg) > 1 && (optarg[1] < '0' || optarg[1] > '9'))
                {
                    // check if the argument is longer than 1 char and not a number
                    printf("PN_ERROR: Invalid trace level '%s'. Must be a single digit 0-9.\n", optarg);
                    cfg_help = 1;
                }
                else if(strlen(optarg) > 1)
                {
                    pn_cfg_vcd_level = MAX(0, MIN(9, atoi(optarg)));
                }
                break;
            case 'i': // itrace
                pn_cfg_itrace_level = atoi(optarg);
                break;
            case 'h': // help
                cfg_help = 1;
                break;
            case ARGS_OPTION_DUMP:
                pn_cfg_dump_memory = 1;
                break;
            case ARGS_OPTION_TRACE_CORE:
                if(pn_parse_enable_trace_core)
                {
                    pn_cfg_trace_core = 1;
                }
                else
                {
                    cfg_help = 1;
                    printf("PN_ERROR: Core trace option is not enabled in this build.\n");
                }
                break;
            case ARGS_OPTION_NO_ASSERT_ABORT:
                pn_disable_assert_abort = 1;
                break;
            case ARGS_OPTION_ASSERT_USE_ABORT:
                pn_assert_use_abort = 1;
                break;
            case ARGS_OPTION_SIGNATURE_PATH:
                pn_cfg_signature_path = optarg;
                break;
            case ARGS_OPTION_SIGNATURE_BEGIN:
                pn_cfg_signature_begin_addr = strtoull(optarg, NULL, 16);
                break;
            case ARGS_OPTION_SIGNATURE_END:
                pn_cfg_signature_end_addr = strtoull(optarg, NULL, 16);
                break;
            case '?':
                // getopt_long already printed an error message.
                cfg_help = 1;
                break;
            default:
                printf("PN_ERROR: Unknown option character `\\x%x'.\n", optopt);
                cfg_help = 1;
        }
    }

    // parse application path (remaining non-option argument)
    if(pn_cfg_enable_application_path && !cfg_help)
    {
        if(optind < argc)
        {
            pn_cfg_application_path = std::string(argv[optind]);
            optind++;
        }
        else
        {
            printf("PN_ERROR: Application path parsing enabled, but no argument left for application path.\n");
            cfg_help = 1;
        }
    }

    if(optind < argc)
    {
        printf("PN_ERROR: Unknown non-option ARGV-elements: ");
        while(optind < argc)
            printf("%s ", argv[optind++]);
        printf("\n");
        cfg_help = 1;
    }

    bool sig_path_set = !pn_cfg_signature_path.empty();
    bool sig_begin_addr_set = (pn_cfg_signature_begin_addr != 0);
    bool sig_end_addr_set = (pn_cfg_signature_end_addr != 0);

    if((sig_path_set || sig_begin_addr_set || sig_end_addr_set) && !(sig_path_set && sig_begin_addr_set && sig_end_addr_set))
    {
        printf("PN_ERROR: For signature dumping, path, begin and end address must be set.\n");
        cfg_help = 1;
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
               "  -t<n>, --vcd-level=<n>     Set VCD trace level (0 = no trace file; >0 == trace file; default = 0)\n"
               "  -i<n>, --itrace-level=<n>  Set instruction trace level (1 = simple itrace; 2 = with all regs; default = 0)\n"
               "  -d, --dump                 Enable memory dump at the beginning and end of simulation\n"
               "  --no-assert-abort          Disable abort on assertion failure\n"
               "  --assert-use-abort         Uses abort() instead of exit() on assertion failure\n"
               "  --signature-path=<path>    Path to the signature file\n"
               "  --signature-begin=<addr>   Begin address of the signature (hex)\n"
               "  --signature-end=<addr>     End address of the signature (hex) aligned to 16 bytes\n"
               "  -h, --help                 Show this help\n");
        if(pn_parse_enable_trace_core)
        {
            printf("  -c, --trace-core        Enable core trace option\n"
                   "                          Outputs core_trace.log\n");
        }
        printf("\n"
               "Environment Variables:\n"
               "  SIM_TRACE_LEVEL          Set VCD trace level (0 = no trace file; >0 == trace file; default = 0)\n"
               "  SIM_DUMP                 Enable memory dump at the beginning and end of simulation\n"
               "\n");
        exit(3);
    }

    // Printing information about the configuration set

    // Prints information about set configuration
    if(pn_cfg_vcd_level == 0)
    {
        PN_INFO("Tracing disabled");
    }
    else
    {
        PN_INFOF(("Tracing enabled Level: %d\n", pn_cfg_vcd_level));
    }

    // Prints information about Assertion handling
    if(pn_disable_assert_abort)
    {
        PN_INFO("Abort on Assertion Failure disabled");
    }

    // Prints information about Assertion using abort

    if(pn_assert_use_abort)
    {
        PN_INFO("Using abort() on Assertion Failure");
    }
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

void pn_tb_assert(bool cond, bool warn_only, const char* msg, const char* filename, const int line)
{
    if(!cond)
    {
        fprintf(stderr, "ASSERTION FAILURE: %s, %s:%i", sc_time_stamp().to_string().c_str(), filename, line);
        if(msg)
            fprintf(stderr, ": %s\n", msg);
        else
            fprintf(stderr, "\n");
        if(!warn_only && !pn_disable_assert_abort)
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
    fprintf(stderr, "(INFO): %*s, %s:%i:   %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
}

void pn_tb_warning(const char* msg, const char* filename, const int line)
{
    int time_size = sc_time_stamp().to_double() == 0.0 ? 1 : 12;
    fprintf(stderr, "(WARNING): %*s, %s:%i: %s\n", time_size, sc_time_stamp().to_string().c_str(), filename, line, msg);
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

// ********************** Disassembler *****************************************

#ifndef __SYNTHESIS__

const char* pn_disassemble_legacy(uint32_t insn)
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
        sprintf(ret + 2, "%s x%u, x%u, 0x%x", table[funct3], (uint)rd, (uint)rs1, (uint)itype);
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
        sprintf(ret + 2, "%s x%u, x%u, x%u", table[funct3], (uint)rd, (uint)rs1, (uint)rs2);
    }
    else if((opcode == 0x33 && bit25))
    { // OP - M-Extension
        static const char* table[] = {"mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"};
        sprintf(ret + 2, "%s x%u, x%u, x%u", table[funct3], (uint)rd, (uint)rs1, (uint)rs2);
    }
    else if(opcode == 0x17)
    { // AUIPC
        sprintf(ret + 2, "auipc x%u, 0x%x", (uint)rd, (uint)utype);
    }
    else if(opcode == 0x37)
    { // LUI
        sprintf(ret + 2, "lui x%u, 0x%x", (uint)rd, (uint)utype);
    }
    else if(opcode == 0x63)
    { // BRANCH
        const char* table[] = {"beq", "bneq", "INV_SUB", "INV_SUB", "blt", "bge", "bltu", "bgeu"};
        sprintf(ret + 2, "%s x%u, x%u, 0x%x", table[funct3], (uint)rs1, (uint)rs2, (uint)btype);
    }
    else if(opcode == 0x6F)
    { // JAL
        sprintf(ret + 2, "jal x%u, 0x%x", (uint)rd, (uint)jtype);
    }
    else if(opcode == 0x67)
    { // JALR
        sprintf(ret + 2, "jalr x%u, x%u, 0x%x", (uint)rd, (uint)rs1, (uint)itype);
    }
    else if(opcode == 0x03)
    { // LOAD
        static const char* table[] = {"lb", "lh", "lw", "INV_SUB", "lbu", "lhu"};
        sprintf(ret + 2, "%s x%u, 0x%x(x%u)", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
    }
    else if(opcode == 0x23)
    { // STORE
        static const char* table[] = {"sb", "sh", "sw"};
        sprintf(ret + 2, "%s x%u, 0x%x(x%u)", table[funct3], (uint)rs2, (uint)stype, (uint)rs1);
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
            sprintf(ret + 2, "%s x%u, 0x%x, x%u", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
        else
            sprintf(ret + 2, "%s x%u, 0x%x, 0x%x", table[funct3], (uint)rd, (uint)itype, (uint)rs1);
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
            sprintf(ret + 2, "%s 0x%x(x%u) ", table[funct3], (uint)itype, (uint)rs1);
    }
    else if(opcode == 0x2F)
    {                                         // AMO
        if(funct3 == 2 && (funct7 >> 2) == 2) // LR.W
            sprintf(ret + 2, "lr.w x%u, (x%u)", (uint)rd, (uint)rs1);
        else if(funct3 == 2 && (funct7 >> 2) == 3) // SC.W
            sprintf(ret + 2, "sc.w x%u, x%u, (x%u) ", (uint)rd, (uint)rs2, (uint)rs1);
        else
            sprintf(ret + 2, "AMO_INVALID x%u, x%u, (x%u) ", (uint)rd, (uint)rs2, (uint)rs1);
    }
    else
        sprintf(ret, "(unknown) 0x%08x", (uint)insn);

    return ret;
}

#endif //__SYNTHESIS__
