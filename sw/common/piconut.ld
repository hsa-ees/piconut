/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2019-2022 Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
                2023      Lukas Bauer <lukas.bauer@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
     This is the linker script used when building software for the PicoNut.

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

/*======================================================================*/
/* PicoNut linker script                                           */
/*======================================================================*/

/*
PROVIDE_HIDDEN (CFG_START_ADDRESS = 0x10000000);
PROVIDE_HIDDEN (CFG_CODE_SIZE = 0x0001FFFF);
PROVIDE_HIDDEN (CFG_RAM_SIZE = 0x0001FFFF);
PROVIDE_HIDDEN (CFG_STACK_SIZE = 0x00001000);
PROVIDE_HIDDEN (CFG_HEAP_SIZE = 0x00001000);
*/

/*----------------------------------------------------------------------*/
/* Setup                                                                */
/*----------------------------------------------------------------------*/

/* The OUTPUT_ARCH command specifies the machine architecture where the
   argument is one of the names used in the BFD library. More
   specifically one of the entires in bfd/cpu-mips.c */
OUTPUT_ARCH( "riscv" )

/* The ENTRY command specifies the entry function in the ELF header.
   Does not necessarily mean that this is the first code to be executed
   upon reset (depends on the SECTIONS configured below). */
ENTRY(_start)

/* Memory Spaces Definitions */
/* On the ZYBO boards and the SystemC simulation the memory for code, data, stack
   and heap is all the same, but they are defined seperately here to enable some
   checks (e.g. read only) and the relocation of the data to the ram memory */
MEMORY
{
    code (rx)  : ORIGIN = CFG_START_ADDRESS, LENGTH = CFG_CODE_SIZE /* sometimes also called rom */
    ram  (rwx) : ORIGIN = CFG_START_ADDRESS + CFG_CODE_SIZE + 1, LENGTH = CFG_RAM_SIZE
}

/* This constant is used to check that at least STACK_SIZE of RAM
  is available for stack. If not, linker will issue an error. */
STACK_SIZE = CFG_STACK_SIZE; 

/* This constant is used to check that at least HEAP_SIZE of RAM
   is available for heap. If not, linker will issue an error. */
HEAP_SIZE = CFG_HEAP_SIZE;


/*----------------------------------------------------------------------*/
/* Sections                                                             */
/*----------------------------------------------------------------------*/

SECTIONS
{
  /* text section - Program code and read only data section */
  .text :
  {
   /* Vector table storing the jump instructions to the start or handler sections */
    . = ALIGN(0x4);
    *(.text.vector_table)

    /* Startup and handler code */
    . = ALIGN(0x4);
    *(.text.init)
    *(.text.debug_handler)
    /* Add new handlers here */

    /* libc/newlib stuff - aligned to 0x100 */
    . = ALIGN(0x100);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array ))
    PROVIDE_HIDDEN (__init_array_end = .);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    /* The rest of the program code */
    . = ALIGN(0x100);
    *(.text)
    . = ALIGN(0x4);
    *(.text.*)
    *(.gnu.linkonce.t.*)
    /* Read only data */
    *(.rodata*)
    *(.srodata*)
    /* C++ exception stuff */
    *(.eh_frame_hdr)
    *(.eh_frame)

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__text_end = .);
  } > code

  /** **************************************************************************
  * .data section - pre-initialized variables
  * This section is physically located in the code memory, but virtually
  * mapped to the ram memory. It will be reloacted by startup.S to ram.
  * This ensures that the pre-initialized variables are not modified by runs
  * before reset.
  * ***************************************************************************/
  .data : ALIGN(4)
  {
    PROVIDE(__data_start = .);
    __global_pointer$ = . + 0x800;

    *(.data .data.* .data* .gnu.linkonce.d.*)
    *(.srodata .srodata.*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)
    *(.tdata .tdata.* .gnu.linkonce.td.*)

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__data_end = .);
  } > ram AT > code

   /** *************************************************************************
   * .bss section - uninitialized data
   * Uninitialized data section that gets zeroed out by the startup code.
   * NOLOAD means that it does not actually take any space in the ELF/Binary
   * and will not be transferred to the hardware during load.
   * **************************************************************************/
  .bss (NOLOAD):
  {
    . = ALIGN(4);
    /* Start of .bss label in the ram memory (for startup) */
    __bss_start = .;
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    /* End of .bss label in the ram memory (for startup) */
    __bss_end = .;                 /* end of .bss label (for startup) */
  } >ram

  /* End of initialized/uninitialized data segment in the ram memory. */
  . = ALIGN(0x4);
  _end = .;

  /** **************************************************************************
  * ._usr_stack_heap section - User stack and heap
  * This is just to check that there is enough ram left for the user stack
  * and heap of ONE core. It should generate an error if it's full.
  * _stack_start is used in syscalls.c to ensure no more heap is used than expected
  * **************************************************************************/
  ._usr_stack_heap (NOLOAD): {
    . = ALIGN(4);
    __heap_start = .;
    . = . + HEAP_SIZE ;
    __heap_end = .;
    _stack_start = .;
    . = . + STACK_SIZE ;
    . = ALIGN(4);
    /* __freertos_irq_stack_top = .; */  /* PicoNut uses configISR_STACK_SIZE_WORDS for freertos */
  } >ram

}

/* Exports for startup.S */
PROVIDE(__start_copy_data_src_begin = LOADADDR(.data));
PROVIDE(__start_copy_data_dst_begin = ADDR(.data));
PROVIDE(__start_copy_data_dst_end   = ADDR(.data) + SIZEOF(.data));
PROVIDE(__start_ram_last            = (ORIGIN(ram) + LENGTH(ram)) - 1);
PROVIDE(__usr_stack_heap_size = SIZEOF(._usr_stack_heap));

/* size helpers */
__text_size = SIZEOF(.text);
__data_size = SIZEOF(.data);
__bss_size = SIZEOF(.bss);
