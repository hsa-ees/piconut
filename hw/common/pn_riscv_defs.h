/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                     Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Common RISC-V constants and definitions to be used both in software and hardware.

    NOTE: CSRs, instruction codes and exception are included from `riscv_encoding.h`,
          which is auto-generated from https://github.com/riscv/riscv-opcodes .
          Since the latter does no stick to PicoNut naming conventions, this file not
          included automatically, but may be included by any PicoNut module on
          a case-by-case basis (#include <pn_riscv_defs.h>).

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


#ifndef __PN_RISCV_DEFS_H__
#define __PN_RISCV_DEFS_H__


// Include general RISC-V constants ...
#include <riscv_encoding.h>


// PicoNut-specific identification fields ...
#define PN_RISCV_VENDORID 0     // non-commercial
#define PN_RISCV_MARCHID 0      // architecture ID; TBD: Allocate a PicoNut ID with RISC-V International

#define PN_RISCV_MIMPID_PICONUT 0x0100   // M Implementation ID for a standard PicoNut (reference Nucleus + reference Membrana)
#define PN_RISCV_MIMPID_NUCLEUS_AI 0x01



/** @brief RISC-V General purpose register ABI names.
 */
typedef enum {
  zero = 0,
  ra,
  sp,
  gp,
  tp,
  t0, t1, t2,
  s0, s1,
  a0, a1, a2, a3, a4, a5, a6, a7,
  s2, s3, s4, s5, s6, s7, s8, s9, s10, s11,
  t3, t4, t5, t6
} pn_reg_e;


/** @brief System calls (according to PK syscall ABI).
 *
 * For details see: https://github.com/riscv-software-src/riscv-pk
 */
typedef enum {
  sys_exit = 93,
  sys_exit_group = 94,
  sys_getpid = 172,
  sys_kill = 129,
  sys_tgkill = 131,
  sys_read = 63,
  sys_write = 64,
  sys_openat = 56,
  sys_close = 57,
  sys_lseek = 62,
  sys_brk = 214,
  sys_linkat = 37,
  sys_unlinkat = 35,
  sys_mkdirat = 34,
  sys_renameat = 38,
  sys_chdir = 49,
  sys_getcwd = 17,
  sys_fstat = 80,
  sys_fstatat = 79,
  sys_faccessat = 48,
  sys_pread = 67,
  sys_pwrite = 68,
  sys_uname = 160,
  sys_getuid = 174,
  sys_geteuid = 175,
  sys_getgid = 176,
  sys_getegid = 177,
  sys_gettid = 178,
  sys_sysinfo = 179,
  sys_mmap = 222,
  sys_munmap = 215,
  sys_mremap = 216,
  sys_mprotect = 226,
  sys_prlimit64 = 261,
  sys_getmainvars = 2011,
  sys_rt_sigaction = 134,
  sys_writev = 66,
  sys_gettimeofday = 169,
  sys_times = 153,
  sys_fcntl = 25,
  sys_ftruncate = 46,
  sys_getdents = 61,
  sys_dup = 23,
  sys_dup3 = 24,
  sys_readlinkat = 78,
  sys_rt_sigprocmask = 135,
  sys_ioctl = 29,
  sys_getrlimit = 163,
  sys_setrlimit = 164,
  sys_getrusage = 165,
  sys_clock_gettime = 113,
  sys_set_tid_address = 96,
  sys_set_robust_list = 99,
  sys_madvise = 233,
  sys_statx = 291,
  sys_readv = 65,
  sys_riscv_hwprobe = 258,
  sys_futex = 98,
  sys_getrandom = 278,

  old_syscall_threshold = 1024,
  sys_open = 1024,
  sys_link = 1025,
  sys_unlink = 1026,
  sys_mkdir = 1030,
  sys_access = 1033,
  sys_stat = 1038,
  sys_lstat = 1039,
  sys_time = 1062,
} pn_pk_syscall_e;


/** @brief CSR write modes
 */
typedef enum {
  WRITE_ALL   = 0x0,
  WRITE_SET   = 0x1,
  WRITE_CLEAR = 0x2
} pn_csr_write_mode_e;


#endif // __PN_RISCV_DEFS_H__
