/*Header***********************************************************************/
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2019-2022 Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
                     2023 Lukas Bauer <lukas.bauer@hs-augsburg.de>
                     2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the startup code (bootrom) for a PicoNut
    processor. It is usable with software compiled with or without
    the libparanut (see example applications).
    Gets the PicoNut processor into a known state, invalidates
    the cache, sets up a default trap handler and afterwards jumps into
    _init function.

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

/*
 * Assign some labels to memory areas to make them more easily accessible.
 */

.section ".tdata.begin"
.align   2                             /* align _tdata_begin to 2^2 bytes     */
_tdata_begin:

.section ".tdata.end"
.align   2                             /* align _tdata_end to 2^2 bytes       */
_tdata_end:

.section ".tbss.end"
.align   2                             /* align _tbss_end to 2^2 bytes        */
_tbss_end:


.align   4                             /* align _thread_data to 2^4 bytes     */
_thread_data:
   .zero 32*4*2                        /* thread space for 32 CPUs (256 Byte) */


/* declare labels in here to be global */
.globl   _tdata_begin                  /* label for area .tdata.begin         */
.globl   _tdata_end                    /* label for area .tdata.end           */
.globl   _tbss_end                     /* label for area .tbss.end            */
.globl   _thread_data                  /* label for area .tdata.begin         */


.section ".text.vector_table"
.align   2                            
__vector_table:
    j _start          
    j debug_handler
    /* Add new handlers etc here */ 

/*
 * Put the startup code in the correct section
 */
.section ".text.init"                  /* enter .text.init section            */
.balign 4

/* declare the startup code as global */
.globl   _start                        /* startup routine                     */

/*Functions********************************************************************/

_start:

  /*
   * Set all registers to 0 (except x0, of course, which is hardwired 0).
   */

   li    x1,      0
   li    x2,      0
   li    x3,      0
   li    x4,      0
   li    x5,      0
   li    x6,      0
   li    x7,      0
   li    x8,      0
   li    x9,      0
   li    x10,     0
   li    x11,     0
   li    x12,     0
   li    x13,     0
   li    x14,     0
   li    x15,     0
   li    x16,     0
   li    x17,     0
   li    x18,     0
   li    x19,     0
   li    x20,     0
   li    x21,     0
   li    x22,     0
   li    x23,     0
   li    x24,     0
   li    x25,     0
   li    x26,     0
   li    x27,     0
   li    x28,     0
   li    x29,     0
   li    x30,     0
   li    x31,     0


init_gp:

  /*
   * The global pointer is supposed to be an absolute location for all cores
   * to orient themselves on. We cannot let the linker throw it around at will
   * for the sake of optimizing away an auipc+jalr!
   * This means we are using the option .norelax whenever we get the gp from
   * __global_pointer$.
   */

   .option push
   .option norelax
   la gp, __global_pointer$            /* set global pointer                  */
   .option pop


init_sp:

   la    tp,      _end + 63
   and   tp,      tp,      -64
   mv    sp,      tp                   /* set sp to tp                        */

   mv    t0,      a0                   /* copy core id to t0                  */
   la    t1,      __usr_stack_heap_size/* load the offset/size into t1        */
   add   sp,      sp,      t1          /* add offset to sp                    */


reloc_data:
   la    t2,      __data_start         /* load target address                */
  /*
   * Function to realocate the data from rom to ram
   */
  la    x7,  __start_copy_data_src_begin // .data: start of copy-source (in .rodata)
  la    x8,  __start_copy_data_dst_begin // .data: start of actual data region
  la    x9,  __start_copy_data_dst_end   // .data: end of actual data region

  beq   x7, x8, __start_data_copy_end // __start_copy_data_src_begin = __start_copy_data_dst_begin

   __start_data_copy:
      bge   x8, x9,  __start_data_copy_end
      lw    x15, 0(x7)
      sw    x15, 0(x8)
      addi  x7, x7, 4          // word-wise operations; section begins and ends on word boundary
      addi  x8, x8, 4
      j     __start_data_copy

__start_data_copy_end:

clear_bss:
   /*
    * Function to clear the bss section.
    * The bss section is not loaded from the executable, but cleared by the startup code.
    * This is done to save space in the executable and to speed up loading.
    */
   la    x10, __bss_start            // .bss: start of bss section
   la    x11, __bss_end              // .bss: end of bss section

   __start_bss_clear:
      bge   x10, x11, __start_bss_clear_end
      sw    zero, 0(x10)
      addi  x10, x10, 4        // word-wise operations; section begins and ends on word boundary
      j     __start_bss_clear

__start_bss_clear_end:


init_uart:

   li    t0,      0x30000000           /* load UART base address into t0      */
   li    t1,      0x00000001           /* enable the UART RX/TX and 1 stop bit*/

   sb    t1,      0x08(t0)             /* enable the UART TX                   */
   sb    t1,      0x0C(t0)             /* enable the UART RX                   */
   li    t1,       216                 /* Set UART baudrate to 115200 @ 25 MHz */
   sw    t1,      0x18(t0)

jump_c:
  /*
   * Jump into C code.
   */

   call __libc_init_array              /* Call __libc_init_array              */
   j     _init                         /* jump into _init (syscalls.c)        */
   call __libc_fini_array              /* Call __libc_fini_array - never reached
                                          but should be executed by exit()    */

startup_error:

   # la    a0,      error_string         /* load address of error_string      */
   # call  printf                        /* print error_string                */
   #.word 0x0000000B                    /* halt execution                     */

/*----------------------------------------------------------------------------*/


error_string:                          /* prepare error message               */
   # .string "There was an error in the startup code. Contact maintainers!\n\n"


/*Handlers*********************************************************************/

/*Debug************************************************************************/
.section ".text.debug_handler"
.align   2                            
debug_handler:
   csrrw x0,      dscratch0, x8
   addi  x8,      x0,        1
   sw    x8,      0x38(x0)       /* set hartstatus halted flag */

loop:
   lw    x8,      0x34(x0)
   andi  x8,      x8,        3
   beq   x0,      x8,        loop

run_commands:
   andi  x8,      x8,        2
   beq   x8,      x0,        resume
   addi  x8,      x0,        9
   sw    x8,      0x38(x0)      /* set hartstatus run_commands flag */
   csrrw x8,      dscratch0, x0
   jalr  x0,      0x14(x0)

resume:
   addi  x8,      x0,        2
   sw    x8,      0x38(x0)       /* set hartstatus running flag */
   csrrw x8,      dscratch0, x0
   .word 0x7b200073           /* dret */
/******************************************************************************/

/*EOF**************************************************************************/
