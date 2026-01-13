/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1.tha.de>
                     Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Test application for verifying the RISC-V A-Extension (atomic instructions) using assertions.
    Exercises various atomic operations and checks their correctness.
    Executed in CI pipeline.

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

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdatomic.h>
#include <assert.h>

#define KRES "\x1B[0m"
#define KGRN "\x1B[32m"

/**
 * atomic_lr_sc_success - Performs a successful LR/SC sequence.
 * This function simulates a successful atomic update using LR/SC by not interfering with the reservation.
 */
int atomic_lr_sc_success(_Atomic uint32_t* addr)
{
    uint32_t old_val;
    int sc_status;
    volatile uint32_t dummy_storage = 0;

    __asm__ volatile(
        "lr.w   %[val], (%[addr])\n"            // Load-reserved from *addr into old_val
        "addi   %[val], %[val], 1\n"            // Increment the value
        "sc.w   %[status], %[val], (%[addr])\n" // Store-conditional back to *addr
        : [val] "=&r"(old_val), [status] "=&r"(sc_status)
        : [addr] "r"(addr)
        : "memory");

    return sc_status;
}

/**
 * atomic_lr_sc_success_dummy_sw - Performs a successful LR/SC sequence.
 * This function simulates a successful atomic update using LR/SC and also stores a dummy value in between.
 */
int atomic_lr_sc_success_dummy_sw(_Atomic uint32_t* addr)
{
    uint32_t old_val;
    int sc_status;
    volatile uint32_t dummy_storage = 0;

    __asm__ volatile(
        "lr.w   %[val], (%[addr])\n"            // Load-reserved from *addr into old_val
        "addi   %[val], %[val], 1\n"            // Increment the value
        "sw     %[val], 0(%[dummy])\n"          // Store to dummy (does not break reservation)
        "sc.w   %[status], %[val], (%[addr])\n" // Store-conditional back to *addr
        : [val] "=&r"(old_val), [status] "=&r"(sc_status)
        : [addr] "r"(addr), [dummy] "r"(&dummy_storage)
        : "memory");

    return sc_status;
}

/**
 * atomic_lr_sc_fail_dummy_sc - Forces SC failure by performing SC on a dummy address first.
 * The dummy SC invalidates the reservation before the real SC.
 */
int atomic_lr_sc_fail_dummy_sc(_Atomic uint32_t* addr)
{
    uint32_t old_val;
    int sc_status;
    int dummy_status;
    volatile uint32_t dummy_storage = 0;

    __asm__ volatile(
        "lr.w   %[val], (%[addr])\n"            // Load-reserved from *addr
        "addi   %[val], %[val], 1\n"            // Increment value
        "sc.w   %[dstat], %[val], (%[dummy])\n" // SC to dummy address - breaks reservation
        "sc.w   %[stat], %[val], (%[addr])\n"   // SC to original address - expected to fail
        : [val] "=&r"(old_val), [stat] "=&r"(sc_status), [dstat] "=&r"(dummy_status)
        : [addr] "r"(addr), [dummy] "r"(&dummy_storage)
        : "memory");

    return sc_status;
}

/**
 * atomic_lr_sc_fail - Forces SC failure by using an AMO (which breaks reservation).
 * AMO instruction invalidates the reservation, so SC fails.
 */
int atomic_lr_sc_fail(_Atomic uint32_t* addr)
{
    uint32_t old_val;
    int sc_status;
    volatile uint32_t dummy_storage = 0;

    __asm__ volatile(
        "lr.w       %[val], (%[addr])\n"          // Load-reserved from *addr
        "amoadd.w   %[val], %[val], (%[addr])\n"  // AMO breaks the reservation
        "sw         %[val], 0(%[dummy])\n"        // Store to dummy (unrelated to reservation)
        "sc.w       %[stat], %[val], (%[addr])\n" // Store-conditional expected to fail
        : [val] "=&r"(old_val), [stat] "=&r"(sc_status)
        : [addr] "r"(addr), [dummy] "r"(&dummy_storage)
        : "memory");

    return sc_status;
}

void atomic_swap_zero(_Atomic uint32_t* addr)
{
    __asm__ volatile(
        "amoswap.w.rl x0, x0, (%0)"
        :
        : "r"(addr)
        : "memory");
}

int atomic_min(_Atomic uint32_t* addr, int val)
{
    int result;
    __asm__ volatile(
        "amomin.w %0, %2, %1"
        : "=r"(result), "+A"(*addr)
        : "r"(val)
        : "memory");
    return result;
}

int atomic_max(_Atomic uint32_t* addr, int val)
{
    int result;
    __asm__ volatile(
        "amomax.w %0, %2, %1"
        : "=r"(result), "+A"(*addr)
        : "r"(val)
        : "memory");
    return result;
}

int atomic_minu(_Atomic uint32_t* addr, uint32_t val)
{
    uint32_t result;
    __asm__ volatile(
        "amominu.w %0, %2, %1"
        : "=r"(result), "+A"(*addr)
        : "r"(val)
        : "memory");
    return result;
}

int atomic_maxu(_Atomic uint32_t* addr, uint32_t val)
{
    uint32_t result;
    __asm__ volatile(
        "amomaxu.w %0, %2, %1"
        : "=r"(result), "+A"(*addr)
        : "r"(val)
        : "memory");
    return result;
}

int atomic_amoadd_rs2_eq_rd(_Atomic uint32_t* addr)
{
    int result, tmp;
    tmp = *addr;
    __asm__ volatile(
        "amoadd.w %0, %2, %1"
        : "=r"(result), "+A"(*addr)
        : "r"(tmp)
        : "memory");
    return result;
}

_Atomic uint32_t atomic_global = 0x1001;

int main()
{
    printf(KGRN);

    uint32_t old;

    old = atomic_global;
    printf("lr/sc success\n");
    int success = atomic_lr_sc_success(&atomic_global);
    printf("lr/sc: (old=%#X, new=%#X, status=%#X)\n", old, atomic_global, success);
    assert(atomic_global == old + 1);
    assert(success == 0);

    old = atomic_global;
    printf("lr/sc success with dummy store\n");
    success = atomic_lr_sc_success_dummy_sw(&atomic_global);
    printf("lr/sc: (old=%#X, new=%#X, status=%#X)\n", old, atomic_global, success);
    assert(atomic_global == old + 1);
    assert(success == 0);

    old = atomic_global;
    printf("lr/sc fail because of dummy sc\n");
    success = atomic_lr_sc_fail_dummy_sc(&atomic_global);
    printf("lr/sc: (old=%#X, new=%#X, status=%#X)\n", old, atomic_global, success);
    assert(atomic_global == old); // Should not change because of dummy sc
    assert(success == 1);

    old = atomic_global;
    printf("lr/sc fail\n");
    success = atomic_lr_sc_fail(&atomic_global);
    printf("lr/sc: (old=%#X, new=%#X, status=%#X)\n", old, atomic_global, success);
    assert(atomic_global == old + old); // amoadd inside simulated interrupt handler
    assert(success == 1);

    old = atomic_exchange(&atomic_global, 0xA00A);
    printf("amoswap.w : old value=%#X, swapped with=0xA00A -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == 0xA00A);

    old = atomic_fetch_add(&atomic_global, 0x0BB0);
    printf("amoadd.w  : old value=%#X, added=0x0BB0 -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == old + 0x0BB0);

    old = atomic_global;
    atomic_swap_zero(&atomic_global);
    printf("amoswap.w : old value=%#X, swapped with=0x0 -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == 0);

    old = atomic_fetch_and(&atomic_global, 0x10);
    printf("amoand.w  : old value=%#X, ANDed with=0x0010 -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == (old & 0x10));

    old = atomic_fetch_or(&atomic_global, 0x10);
    printf("amoor.w   : old value=%#X, ORed with=0x0010 -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == (old | 0x10));

    old = atomic_fetch_xor(&atomic_global, 0x10);
    printf("amoxor.w  : old value=%#X, XORed with=0x0010 -> new value=%#X\n", old, atomic_global);
    assert(atomic_global == (old ^ 0x10));

    atomic_global = -1;
    uint32_t initial_val_signed_test = atomic_global; // Store before atomic_min
    old = atomic_min(&atomic_global, 0x1);
    printf("amomin.w  : old value=%#X, min with=0x0001 -> new value=%#X\n", old, atomic_global);
    assert(old == initial_val_signed_test);
    assert(atomic_global == ((int)initial_val_signed_test < (int)0x1 ? initial_val_signed_test : (uint32_t)0x1));

    atomic_global = -1;
    initial_val_signed_test = atomic_global; // Store before atomic_max
    old = atomic_max(&atomic_global, 0x1);
    printf("amomax.w  : old value=%#X, max with=0x0001 -> new value=%#X\n", old, atomic_global);
    assert(old == initial_val_signed_test);
    assert(atomic_global == ((int)initial_val_signed_test > (int)0x1 ? initial_val_signed_test : (uint32_t)0x1));

    atomic_global = -1;
    uint32_t initial_val_unsigned_test = atomic_global; // Store before atomic_minu
    old = atomic_minu(&atomic_global, 0x1);
    printf("amominu.w : old value=%#X, unsigned min with=0x0001 -> new value=%#X\n", old, atomic_global);
    assert(old == initial_val_unsigned_test);
    assert(atomic_global == (initial_val_unsigned_test < 0x1 ? initial_val_unsigned_test : 0x1));

    atomic_global = -1;
    initial_val_unsigned_test = atomic_global; // Store before atomic_maxu
    old = atomic_maxu(&atomic_global, 0x1);
    printf("amomaxu.w : old value=%#X, unsigned max with=0x0001 -> new value=%#X\n", old, atomic_global);
    assert(old == initial_val_unsigned_test);
    assert(atomic_global == (initial_val_unsigned_test > 0x1 ? initial_val_unsigned_test : 0x1));

    atomic_global = 0x5;
    old = atomic_amoadd_rs2_eq_rd(&atomic_global);
    printf("amoadd.w (rs2==rd): old value=%#X, new value=%#X\n", old, atomic_global);
    assert(atomic_global == old + old); // Should double the value

    printf("Successfully executed all atomic operations.\n");

    printf("%s\n", KRES);

    return 0;
}
