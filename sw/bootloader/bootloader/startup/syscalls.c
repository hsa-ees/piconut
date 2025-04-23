/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2019-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
						  Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
						  Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
                2023      Lukas Bauer <lukas.bauer@hs-augsburg.de>
	  Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
	This file contains the PicoNut system call implementations for
	full newlib functionality

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

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/times.h>
#include "encoding.h"

/* Defines */
#define FLUSH_TOHOST() ({                  \
	asm volatile(                          \
		"lui x6, %hi(pn_tohost)\n"         \
		"addi x6, x6, %lo(pn_tohost)\n"    \
		".word (0x300B | ((6) << 15))\n"); \
})

#define FLUSH_FROMHOST() ({                \
	asm volatile(                          \
		"lui x6, %hi(pn_fromhost)\n"       \
		"addi x6, x6, %lo(pn_fromhost)\n"  \
		".word (0x300B | ((6) << 15))\n"); \
})

/* Variables */
extern int errno;

register char *stack_ptr asm("sp");

volatile extern uint8_t pn_tohost;
volatile extern uint8_t pn_fromhost;
volatile extern uint8_t uart_begin;
int _write(int file, char *ptr, int len);

/* Functions */
int main(int argc, char** argv);

void _init(int cid, int nc)
{
	// Only CePU should ever get here.
	int ret = main(0, 0);
	exit(ret);
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit(int status)
{
	kill(status, -1);
	// If we are being debugged this triggers the debugger to get control
	// before we hang/halt the execution
	asm volatile("ebreak");
	while (1)
	{
		asm(".word 0x0000000B;");
	} /* Make sure we hang here */
}

int _read(int file, char *ptr, int len)
{

	return -1;
}

int _putc(int ch)
{
#ifdef UART_EXTERNAL
	volatile unsigned char* lsr = (unsigned char*) 0x60000014;
	// Wait till pn_tohost is empty
	while((*lsr & 0x20) == 1);

	uart_begin = ch & 0xFF;
#else
	volatile unsigned char *uart_sr = (unsigned char *)0xE000102C;
	volatile unsigned char *uart_fifo = (unsigned char *)0xE0001030;
	// Check if UART is not full els wait
	while (*uart_sr & 0x10 == 0x10)
		;

	*uart_fifo = ch & 0xFF;
#endif

	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		_putc(*ptr++);
	}

	return DataIdx;
}

caddr_t _sbrk(int incr)
{
	extern char _tdata_begin, _tdata_end, _tbss_end, __heap_end, __heap_start;
	static char *heap_brk;
	char *prev_heap_brk;

	if (heap_brk == 0)
		heap_brk = &__heap_start;

	prev_heap_brk = heap_brk;
	if (heap_brk + incr > &__heap_end)
	{
		_write(1, "Heap and stack collision\n", 25);
		abort();
	}

	heap_brk += incr;

	return (caddr_t)prev_heap_brk;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 0;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _gettimeofday(struct timeval *tv, struct timezone *tz)
{

	return -1;
}

int _times(struct tms *buf)
{

	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

int _feof(int *file)
{
	return 0;
}

int usleep(unsigned long useconds)
{
}
