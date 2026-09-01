#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025-2026 Gundolf Kiefer <gundolf.kiefer@tha.de>
#     Technische Hochschule Augsburg / Technical University of Applied Sciences
#
#  Description:
#    Vector implementations of LeNet-5 layers.
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------


        .file   "lenet5_asm.s"
        .option nopic
        .attribute arch, "rv32imv"     # NOTE: This overrides an eventual '-march' command line option
        .attribute unaligned_access, 0
        .attribute stack_align, 16

        .data
        .align  2

        .text
        .align  2


#include "lenet5.h"


#define PN_XLEN 32      // integer register length (32 for RV32I/RV32E)

#define PN_VLEN 1024    // number of bits in a single vector register
#define PN_ELEN 32      // maximum size in bits of a vector element that any operation can produce or consume

#define PN_VLENB ((PN_VLEN) / 8)





################################################################################
#                                                                              #
#                        Activation                                            #
#                                                                              #
################################################################################



######################### run_relu_vec_32f8 ####################################


# void run_relu_vec_32f8 (DTYPE *data, int count);
#   a0 = data, a1 = count
#
        .global run_relu_vec_32f8
        .type   run_relu_vec_32f8, @function
run_relu_vec_32f8:
        vsetvli   t0, zero, e32, m8, ta, ma   # 32 bit elements, LMUL = 8
        vmv.v.x   v16, zero                   # set v16 := all-0

.relu_vec_32f8_l010:
        vsetvli   t0, a1, e32, m8, ta, ma     # setup VL, returned t0 is the chunk length
        vle32.v   v8, (a0)
        vmslt.vx  v0, v8, zero                # set a mask for all elements <0
        vse32.v   v16, (a0), v0.t             # write zeros for all those elements
#        vmax.vx   v8, v8, zero                # set all negative elements to 0
#        vse32.v   v8, (a0)                    # write back whole vector

        sub       a1, a1, t0                  # decrement loop counter
        slli      t0, t0, 2
        add       a0, a0, t0                  # increment adress (+= 4 * chunk size)
        bne       a1, zero, .relu_vec_32f8_l010

        # Done ...
        ret
        .size   run_relu_vec_32f8, .-run_relu_vec_32f8



######################### run_relu_vec_16f4 ####################################


# void run_relu_vec_16f4 (DTYPE *data, int count);
#   a0 = data, a1 = count
#
        .global run_relu_vec_16f4
        .type   run_relu_vec_16f4, @function
run_relu_vec_16f4:
        vsetvli   t0, zero, e16, m8, ta, ma   # 16-Bit elements + LMUL = 8
        vmv.v.x   v16, zero                   # set v12 := all-0

.relu_vec_16f4_l010:
        vsetvli   t0, a1, e16, m8, ta, ma     # setup VL, t0 is the chunk length to handle in this iteration
        vle16.v   v8, (a0)
        vmslt.vx  v0, v8, zero                # set a mask for all elements <0
        vse16.v   v16, (a0), v0.t             # write zeros for all those elements

.relu_vec_16f4_c010:
        sub       a1, a1, t0                  # decrement loop counter
        slli      t0, t0, 1
        add       a0, a0, t0                  # increment adress (+= 2 * chunk size)
        bne       a1, zero, .relu_vec_16f4_l010

        # Done ...
.relu_vec_16f4_c011:
        ret
        .size   run_relu_vec_16f4, .-run_relu_vec_16f4





################################################################################
#                                                                              #
#                        Pooling                                               #
#                                                                              #
################################################################################



######################### run_maxpool_vec_32f8 #################################


# void run_maxpool (DTYPE *out, DTYPE *in, int channels, int dim);
#
#   Arguments:
#     a0 = out[channels][dim][dim]
#     a1 = in[channels][2*dim][2*dim]
#     a2 = channels
#     a3 = dim
#
#   Registers:
#     t1: input stride (single line)
#     t2: output stride
#     t3: = 2 * dim (for input)
#     t4: loop counter
#
#     v0: vector mask selecting elements 0, 2, ...
#     v1: vector mask selecting elements 1, 3, ...
#
        .global run_maxpool_vec_32f8
        .type   run_maxpool_vec_32f8, @function
run_maxpool_vec_32f8:
        slli      t1, a3, 3     # input stride: 2*dim * 4 bytes/word
        slli      t2, a3, 2     # output stride: dim * 4 bytes/word
        slli      t3, a3, 1     # input vector length: 2*dim
        mul       t4, a2, a3    # Iterations

        vsetvli   x0, t3, e8, m1, ta, ma
        li        t0, 0x55
        vmv.v.x   v0, t0        # v0 := 0x55..55 - bitmask for even elements
        li        t0, 0xaa
        vmv.v.x   v1, t0        # v1 := 0xaa..aa - bitmask for odd elements

.maxpool_32f8_l00:
        vsetvli       x0, t3, e32, m1, ta, ma
        vle32.v       v2, (a1)      # load upper line
        add           a1, a1, t1
        vcompress.vm  v4, v2, v0    # upper-left elements
        vcompress.vm  v5, v2, v1    # upper-right elements
        vle32.v       v3, (a1)      # load lower line
        add           a1, a1, t1
        vcompress.vm  v6, v3, v0    # lower-left elements
        vcompress.vm  v7, v3, v1    # lower-right elements

        vsetvli       x0, a3, e32, m1, ta, ma
        vmax.vv       v8, v4, v5    # compute maximum ...
        vmax.vv       v9, v6, v7
        vmax.vv       v10, v8, v9

        vse32.v       v10, (a0)     # store maximum row
        add           a0, a0, t2

        addi          t4, t4, -1
        bne           t4, zero, .maxpool_32f8_l00
        ret
        .size   run_maxpool_vec_32f8, .-run_maxpool_vec_32f8



######################### run_maxpool_vec_16f4 #################################


# void run_maxpool (DTYPE *out, DTYPE *in, int channels, int dim);
#
#   Arguments:
#     a0 = out[channels][dim][dim]
#     a1 = in[channels][2*dim][2*dim]
#     a2 = channels
#     a3 = dim
#
#   Registers:
#     t1: input stride (single line)
#     t2: output stride
#     t3: = 2 * dim (for input)
#     t4: loop counter
#
#     v0: vector mask selecting elements 0, 2, ...
#     v1: vector mask selecting elements 1, 3, ...
#
        .global run_maxpool_vec_16f4
        .type   run_maxpool_vec_16f4, @function
run_maxpool_vec_16f4:
        slli      t1, a3, 2     # input stride: 2*dim * 2 bytes/word
        slli      t2, a3, 1     # output stride: dim * 2 bytes/word
        slli      t3, a3, 1     # input vector length: 2*dim
        mul       t4, a2, a3    # Iterations

        vsetvli   x0, t3, e8, m1, ta, ma
        li        t0, 0x55
        vmv.v.x   v0, t0        # v0 := 0x55..55 - bitmask for even elements
        li        t0, 0xaa
        vmv.v.x   v1, t0        # v1 := 0xaa..aa - bitmask for odd elements

.maxpool_16f4_l00:
        vsetvli       x0, t3, e16, m1, ta, ma
        vle16.v       v2, (a1)      # load upper line
        add           a1, a1, t1
        vcompress.vm  v4, v2, v0    # upper-left elements
        vcompress.vm  v5, v2, v1    # upper-right elements
        vle16.v       v3, (a1)      # load lower line
        add           a1, a1, t1
        vcompress.vm  v6, v3, v0    # lower-left elements
        vcompress.vm  v7, v3, v1    # lower-right elements

        vsetvli       x0, a3, e16, m1, ta, ma
        vmax.vv       v8, v4, v5    # compute maximum ...
        vmax.vv       v9, v6, v7
        vmax.vv       v10, v8, v9

        vse16.v       v10, (a0)     # store maximum row
        add           a0, a0, t2

        addi          t4, t4, -1
        bne           t4, zero, .maxpool_16f4_l00
        ret
        .size   run_maxpool_vec_16f4, .-run_maxpool_vec_16f4





################################################################################
#                                                                              #
#                        Convolutional Layers                                  #
#                                                                              #
################################################################################



######################### Common Data ##########################################


# Index arrays to collect from a sliding window buffer ...
#   Windows are stored in the vector as an array [25][<n>], where the lower dimension
#   represents the output channels, and the higher dimension represents the input
#   pixels. This way, vector operations on 8-element vectors (or multiples) can be
#   used for the accumulating additions.

        .data

        .macro CONV_IDX_WINDOW_5_5_8_VEC stride, row, col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
        .endm
        .macro CONV_IDX_WINDOW_5_5_16_VEC stride, row, col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
            .half \row * \stride + \col, \row * \stride + \col, \row * \stride + \col, \row * \stride + \col
        .endm
        .macro CONV_IDX_WINDOW_ROW func, stride, row
            \func \stride, \row, 0
            \func \stride, \row, 1
            \func \stride, \row, 2
            \func \stride, \row, 3
            \func \stride, \row, 4
        .endm
        .macro CONV_IDX_WINDOW func, stride
            CONV_IDX_WINDOW_ROW \func, \stride, 0
            CONV_IDX_WINDOW_ROW \func, \stride, 1
            CONV_IDX_WINDOW_ROW \func, \stride, 2
            CONV_IDX_WINDOW_ROW \func, \stride, 3
            CONV_IDX_WINDOW_ROW \func, \stride, 4
        .endm

.conv1_window_idx:
        # Index array to collect a window in the Conv-1 step for output vectors of size 8.
        CONV_IDX_WINDOW CONV_IDX_WINDOW_5_5_8_VEC INPUT_DIM

.conv2_window_idx_8:
        # Index array to collect a window in the Conv-2 step for output vectors of size 8.
        CONV_IDX_WINDOW CONV_IDX_WINDOW_5_5_8_VEC CONV1_POOL_DIM

.conv2_window_idx_16:
        # Index array to collect a window in the Conv-2 step for output vectors of size 16.
        CONV_IDX_WINDOW CONV_IDX_WINDOW_5_5_16_VEC CONV1_POOL_DIM


# Index array to reformat weights ...
#   Weights originally contain vectors with 6 elements for output channels.
#   These vectors are expanded to be aligned to 8 vector elements.
#   This faciltates the parallel accumulation of the vector.

        .macro CONV_IDX_WEIGHTS_VEC_8 row, col
            .half (\row * 5 + \col) * 6 + 0, (\row * 5 + \col) * 6 + 1
            .half (\row * 5 + \col) * 6 + 2, (\row * 5 + \col) * 6 + 3
            .half (\row * 5 + \col) * 6 + 4, (\row * 5 + \col) * 6 + 5
            .half (\row * 5 + \col) * 6 + 5, (\row * 5 + \col) * 6 + 5    // dummy copy of the last two columns
        .endm
        .macro CONV_IDX_WEIGHTS_ROW func, row
            \func \row, 0
            \func \row, 1
            \func \row, 2
            \func \row, 3
            \func \row, 4
        .endm
        .macro CONV_IDX_WEIGHTS func
            CONV_IDX_WEIGHTS_ROW \func, 0
            CONV_IDX_WEIGHTS_ROW \func, 1
            CONV_IDX_WEIGHTS_ROW \func, 2
            CONV_IDX_WEIGHTS_ROW \func, 3
            CONV_IDX_WEIGHTS_ROW \func, 4
        .endm

.conv1_weights_idx_8:
        CONV_IDX_WEIGHTS CONV_IDX_WEIGHTS_VEC_8





######################### run_conv1_vec_32f8 ###################################


##### Macros to set common vector formats #####

# Note: These macros may clobber 't0'!

.macro VSETVL_CONV1_32F8_WEIGHTS
    # Weights: 5*5*6 = 150 elements, LMUL=8, (optionally in a different format than products)
    li t0, CONV1_CHS * KERNEL_DIM * KERNEL_DIM
    vsetvli x0, t0, e32, m8, ta, ma
.endm

.macro VSETVL_CONV1_32F8_PRODUCTS
    # Partial products: 5*5*8 = 200 elements, LMUL=8
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e32, m8, ta, ma
.endm

.macro VSETVL_CONV1_32F8_PRODUCTS_MASK
    # Mask for partial products: 5*5*8 = 200 bits = 200/8 bytes = 25 bytes, LMUL=1
    li t0, KERNEL_DIM * KERNEL_DIM
    vsetvli x0, t0, e8, m1, ta, ma
.endm

.macro VSETVL_CONV1_32F8_PRODUCTS_INDEX
    # Products/weights: 16-bit index elements for 'vrgatherei16': 5*5*8 = 200 elements, LMUL=4, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV1_32F8_SUMVECS vecs lmul
    # Partial products: <vecs>*8 elements, LMUL=<lmul>
    .if \vecs * CONV2_CHS/2 < 32
    vsetivli x0, \vecs * CONV2_CHS/2, e32, \lmul, ta, ma
    .else
    li t0, \vecs * CONV2_CHS/2
    vsetvli x0, t0, e32, \lmul, ta, ma
    .endif
.endm

.macro VSETVL_CONV1_32F8_SUMVEC
    # Accumulator: 6 elements
    vsetivli x0, CONV1_CHS, e32, m1, ta, ma
.endm

.macro VSETVL_CONV1_32F8_SWB
    # Sliding window buffer: 4*32+5 = 133 elements, LMUL=8
    li t0, (KERNEL_DIM - 1) * INPUT_DIM + KERNEL_DIM
  vsetvli x0, t0, e32, m8, ta, ma
.endm

.macro VSETVL_CONV1_32F8_SWB_INDEX
    # Sliding window buffer, 16-bit index elements for 'vrgatherei16': 5*5*8 = 200 elements, LMUL=4, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm


##### Implementation #####

# This function leverages the sliding window approach to minimize the number of
# memory accesses. All weights and biases are kept in vector registers.
# Altogether, each element of the input, output and model is accessed exactly
# once, which represents the theoretical minimum of data accesses.
#
# Pseudo-Code:
#
#   Preload ALL weights and biases into vector registers
#   initialize the sliding window buffer
#   for each output row:
#     for each output column:
#       multiply the input window with its weights (all output channels in parallel)
#       calculate the sum of the products
#       add biases and perform rounding (vectorized over channels)
#       write the ouputs
#       progress sliding window buffer
#

# void run_conv1_vec_32f8 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
#
#   Arguments:
#     a0 = in[INPUT_DIM][INPUT_DIM]
#     a1 = out[CONV1_CHS][CONV1_DIM][CONV1_DIM]
#     a2 = weight[KERNEL_DIM][KERNEL_DIM][CONV1_CHS]
#     a3 = bias[CONV1_CHS]
#
#   Register use:
#     a0 .. a3: Pointers to the current elements (incremented inside the routine).
#     t0: temporary, potentially clobbered by macros
#     t1: end value for output address a1, stop condition for row loop
#     t2: end value for output address a1, stop condition for column loop
#     t3: loop counter (inner channel loop)
#
#     v24 .. v31: Weights (LMUL=8)
#     v16 .. v23: Sliding window buffer (LMUL=8)
#     v8  .. v15: Working register (LMUL=8) for products
#     v4  .. v7: Index vector for gathering kernel from sliding window buffer (LMUL=4, E16, VLEN=same as weights)
#     v3: Bias
#     v1 .. v2: Temporary (LMUL=1),
#     v0 : Temporary (sequential variant)
#     v0 : Mask for products (parallel variant)
#
        .text
        .global run_conv1_vec_32f8
        .type   run_conv1_vec_32f8, @function
run_conv1_vec_32f8:

        # Init product mask (-> v0) ...
        VSETVL_CONV1_32F8_PRODUCTS_MASK
        li              t0, 0x3f
        vmv.v.x         v0, t0

        # Load weights (-> v24..v31) ...
        VSETVL_CONV1_32F8_WEIGHTS
        vle32.v         v8, (a2)
        VSETVL_CONV1_32F8_PRODUCTS_INDEX
        la              t0, .conv1_weights_idx_8
        vle16.v         v4, (t0), v0.t
        VSETVL_CONV1_32F8_PRODUCTS
        vrgatherei16.vv v24, v8, v4, v0.t

        # Load bias (-> v3) ...
        VSETVL_CONV1_32F8_SUMVEC
        vle32.v         v3, (a3)

        # Index vector to select kernel elements from sliding window buffer (-> v4..v7) ...
        VSETVL_CONV1_32F8_SWB_INDEX
        la              t0, .conv1_window_idx
        vle16.v         v4, (t0)

        # Init sliding window buffer for input channel (v16..v23) ...
        VSETVL_CONV1_32F8_SWB
        vle32.v         v16, (a0)
        addi            a0, a0, ((KERNEL_DIM - 1) * INPUT_DIM + KERNEL_DIM) * 4

        # Loop over output rows ...
        li              t0, 4 * CONV1_DIM * CONV1_DIM
        add             t1, a1, t0              # end value for output address a1
.conv1_32f8_l00:

        # Loop over output columns ...
        addi            t2, a1, 4 * CONV1_DIM   # end value for output address a1
.conv1_32f8_l01:

        # Get window (-> v8..v15) ...
        VSETVL_CONV1_32F8_PRODUCTS
        vrgatherei16.vv v8, v16, v4, v0.t
          # load window from v16..v23 into v8..v15, v4..v7 are index values

        # Multiply with weights ...
        vmul.vv         v8, v8, v24, v0.t # multiply with weights

        # Accumulate (in parallel for all output channels) ...
        #   v8..v15[25][8] contains 25 vectors P[0] ... P[24] to be added.
        #   Each single vector register contains (up to) 4 vectors.
        # P[0..7] += P[8..15] ...
        VSETVL_CONV1_32F8_SUMVECS 8 m2
        vadd.vv         v8, v8, v10, v0.t
        # P[0..7] += P[16..23] ...
        vadd.vv         v8, v8, v12, v0.t
        # P[0..3] += P[4..7] ...
        VSETVL_CONV1_32F8_SUMVECS 4 m1
        vadd.vv         v8, v8, v9, v0.t
        # P[0..1] += P[2..3] ...
        vslidedown.vi   v1, v8, 2*8, v0.t
        VSETVL_CONV1_32F8_SUMVECS 2 m1
        vadd.vv         v8, v8, v1, v0.t
        # P[0] += P[1] ...
        vslidedown.vi   v1, v8, 1*8, v0.t
        VSETVL_CONV1_32F8_SUMVEC
        vadd.vv         v8, v8, v1, v0.t
        # v1 = P[0] + P[24] ...
        vadd.vv         v1, v8, v14, v0.t

        # Round and shift ...
        VSETVL_CONV1_32F8_SUMVEC
        li              t0, 0x80          # DTYPE_POST_MUL(): round and ...
        vadd.vx         v1, v1, t0
        vsra.vi         v1, v1, 8         # ... shift-right

        # Add bias ...
        vadd.vv         v1, v1, v3

        # Write result ...
        li              t0, 4 * CONV1_DIM * CONV1_DIM       # write result ...
        vsse32.v        v1, (a1), t0    # strided store
        addi            a1, a1, 4       # increment address

        # End of last row? ...
        bge             a1, t1, .conv1_32f8_c00

        # Load next element into sliding window buffer ...
        VSETVL_CONV1_32F8_SWB
        lw              t0, (a0)
        addi            a0, a0, 4
        vslide1down.vx  v16, v16, t0

        # End of row? If not: back to process next column ...
        blt             a1, t2, .conv1_32f8_l01

        # End of row: Read borders into sliding window buffer ...
        .if KERNEL_DIM != 5
        .err  # "The following code asserts that KERNEL_DIM == 5"
        .endif
        VSETVL_CONV1_32F8_SWB
        lw              t0,  0(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0,  4(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0,  8(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0, 12(a0)
        vslide1down.vx  v16, v16, t0
        addi            a0, a0, 16

        # Back to process next row ...
        j               .conv1_32f8_l00

        # Done ...
.conv1_32f8_c00:
        ret
        .size   run_conv1_vec_32f8, .-run_conv1_vec_32f8





######################### run_conv1_vec_16f4 ###################################


##### Macros to set common vector formats #####

# Note: These macros may clobber 't0'!

.macro VSETVL_CONV1_16F4_WEIGHTS
    # Weights: 5*5*6 = 150 elements, LMUL=4, (optionally in a different format than products)
    li t0, KERNEL_DIM * KERNEL_DIM * CONV1_CHS
    vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV1_16F4_PRODUCTS
    # Partial products: 5*5*8 = 200 elements, LMUL=4
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV1_16F4_PRODUCTS_MASK
    # Mask for partial products: 5*5*8 = 200 bits = 200/8 bytes = 25 bytes, LMUL=1
    li t0, KERNEL_DIM * KERNEL_DIM
    vsetvli x0, t0, e8, m1, ta, ma
.endm

.macro VSETVL_CONV1_16F4_PRODUCTS_INDEX
    # Products/weights: 16-bit index elements for 'vrgatherei16': 5*5*8 = 200 elements, LMUL=4, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV1_16F4_SUMVECS vecs lmul
    # Partial products: <vecs>*8 elements, LMUL=<lmul>
    .if \vecs * CONV2_CHS/2 < 32
    vsetivli x0, \vecs * CONV2_CHS/2, e16, \lmul, ta, ma
    .else
    li t0, \vecs * CONV2_CHS/2
    vsetvli x0, t0, e16, \lmul, ta, ma
    .endif
.endm

.macro VSETVL_CONV1_16F4_SUMVEC
    # Accumulator: 6 elements
    vsetivli x0, CONV1_CHS, e16, m1, ta, ma
.endm

.macro VSETVL_CONV1_16F4_SWB
    # Sliding window buffer: 4*32+5 = 133 elements, LMUL=4
    li t0, (KERNEL_DIM - 1) * INPUT_DIM + KERNEL_DIM
  vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV1_16F4_SWB_INDEX
    # Sliding window buffer, 16-bit index elements for 'vrgatherei16': 5*5*8 = 200 elements, LMUL=4, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm



##### Implementation #####

# This function leverages the sliding window approach to minimize the number of
# memory accesses. All weights and biases are kept in vector registers.
# Altogether, each element of the input, output and model is accessed exactly
# once, which represents the theoretical minimum of data accesses.
#
# Pseudo-Code:
#
#   Preload ALL weights and biases into vector registers
#   initialize the sliding window buffer
#   for each output row:
#     for each output column:
#       multiply the input window with its weights (all output channels in parallel)
#       calculate the sum of the products
#       add biases and perform rounding (vectorized over channels)
#       write the ouputs
#       progress sliding window buffer
#

# void run_conv1_vec_16f4 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
#
#   Arguments:
#     a0 = in[INPUT_DIM][INPUT_DIM]
#     a1 = out[CONV1_CHS][CONV1_DIM][CONV1_DIM]
#     a2 = weight[KERNEL_DIM][KERNEL_DIM][CONV1_CHS]
#     a3 = bias[CONV1_CHS]
#
#   Register use:
#     a0 .. a3: Pointers to the current elements (incremented inside the routine).
#     t0: Temporary, potentially clobbered by macros
#     t1: end value for output address a1, stop condition for row loop
#     t2: end value for output address a1, stop condition for column loop
#     t3: loop counter (inner channel loop)
#
#     v24 .. v27: Weights (LMUL=4)
#     v16 .. v19: Sliding window buffer (LMUL=4)
#     v8  .. v11: Working register (LMUL=4) for products
#     v4  .. v7: Index vector for gathering kernel from sliding window buffer (LMUL=4, E16, VLEN=same as weights)
#     v3: Bias
#     v1 .. v2: Temporary (LMUL=1),
#     v0 : Temporary (sequential variant)
#     v0 : Mask for products (parallel variant)
#
        .text
        .global run_conv1_vec_16f4
        .type   run_conv1_vec_16f4, @function
run_conv1_vec_16f4:

        # Init product mask (-> v0) ...
        VSETVL_CONV1_16F4_PRODUCTS_MASK
        li              t0, 0x3f
        vmv.v.x         v0, t0

        # Load weights (-> v24..v31) ...
        VSETVL_CONV1_16F4_WEIGHTS
        vle16.v         v8, (a2)
        VSETVL_CONV1_16F4_PRODUCTS_INDEX
        la              t0, .conv1_weights_idx_8
        vle16.v         v4, (t0), v0.t
        VSETVL_CONV1_16F4_PRODUCTS
        vrgatherei16.vv v24, v8, v4, v0.t

        # Load bias (-> v3) ...
        VSETVL_CONV1_16F4_SUMVEC
        vle16.v         v3, (a3)

        # Index vector to select kernel elements from sliding window buffer (-> v4..v7) ...
        VSETVL_CONV1_16F4_SWB_INDEX
        la              t0, .conv1_window_idx
        vle16.v         v4, (t0)

        # Init sliding window buffer for input channel (v16..v19) ...
        VSETVL_CONV1_16F4_SWB
        vle16.v         v16, (a0)
        addi            a0, a0, ((KERNEL_DIM - 1) * INPUT_DIM + KERNEL_DIM) * 2

        # Loop over output rows ...
        li              t0, 2 * CONV1_DIM * CONV1_DIM
        add             t1, a1, t0              # end value for output address a1
.conv1_16f4_l00:

        # Loop over output columns ...
        addi            t2, a1, 2 * CONV1_DIM   # end value for output address a1
.conv1_16f4_l01:

        # Get window (-> v8..v11) ...
        VSETVL_CONV1_16F4_PRODUCTS
        vrgatherei16.vv v8, v16, v4, v0.t
          # load window from v16..v19 into v8..v11, v4..v7 are index values

        # Multiply with weights ...
        vmul.vv         v8, v8, v24, v0.t # multiply with weights

        # Accumulate (in parallel for all output channels) ...
        #   v8..v11[25][8] contains 25 vectors P[0] ... P[24] to be added.
        #   Each single vector register contains (up to) 8 vectors.
        # P[0..7] += P[8..15] ...
        VSETVL_CONV1_16F4_SUMVECS 8 m1
        vadd.vv         v8, v8, v9, v0.t
        # P[0..7] += P[16..23] ...
        vadd.vv         v8, v8, v10, v0.t
        # P[0..3] += P[4..7] ...
        li              t0, 4*8
        vslidedown.vx   v1, v8, t0, v0.t
        VSETVL_CONV1_16F4_SUMVECS 4 m1
        vadd.vv         v8, v8, v1, v0.t
        # P[0..1] += P[2..3] ...
        vslidedown.vi   v1, v8, 2*8, v0.t
        VSETVL_CONV1_16F4_SUMVECS 2 m1
        vadd.vv         v8, v8, v1, v0.t
        # P[0] += P[1] ...
        vslidedown.vi   v1, v8, 1*8, v0.t
        VSETVL_CONV1_16F4_SUMVEC
        vadd.vv         v8, v8, v1, v0.t
        # v1 = P[0] + P[24] ...
        vadd.vv         v1, v8, v11, v0.t

        # Round and shift ...
        VSETVL_CONV1_16F4_SUMVEC
        vadd.vi         v1, v1, 8         # DTYPE_POST_MUL(): round and ...
        vsra.vi         v1, v1, 4         # ... shift-right

        # Add bias ...
        vadd.vv         v1, v1, v3

        # Write result ...
        li              t0, 2 * CONV1_DIM * CONV1_DIM       # write result ...
        vsse16.v        v1, (a1), t0    # strided store
        addi            a1, a1, 2       # increment address

        # End of last row? ...
        bge             a1, t1, .conv1_16f4_c00

        # Load next element into sliding window buffer ...
        VSETVL_CONV1_16F4_SWB
        lh              t0, (a0)
        addi            a0, a0, 2
        vslide1down.vx  v16, v16, t0

        # End of row? If not: back to process next column ...
        blt             a1, t2, .conv1_16f4_l01

        # End of row: Read borders into sliding window buffer ...
        .if KERNEL_DIM != 5
        .err  # "The following code asserts that KERNEL_DIM == 5"
        .endif
        VSETVL_CONV1_16F4_SWB
        lh              t0,  0(a0)
        vslide1down.vx  v16, v16, t0
        lh              t0,  4(a0)
        vslide1down.vx  v16, v16, t0
        lh              t0,  8(a0)
        vslide1down.vx  v16, v16, t0
        lh              t0, 12(a0)
        vslide1down.vx  v16, v16, t0
        addi            a0, a0, 4*2

        # Back to process next row ...
        j               .conv1_16f4_l00

        # Done ...
.conv1_16f4_c00:
        ret
        .size   run_conv1_vec_16f4, .-run_conv1_vec_16f4





######################### run_conv2_vec_32f8 ###################################


##### Macros to set common vector formats #####

# Note: These macros may clobber 't0'!

.macro VSETVL_CONV2_32F8_WEIGHTS
    # Weights: 8*5*5 = 200 elements, LMUL=8, (optionally in a different format than products)
    li t0, CONV2_CHS/2 * KERNEL_DIM * KERNEL_DIM
    vsetvli x0, t0, e32, m8, ta, ma
.endm

.macro VSETVL_CONV2_32F8_PRODUCTS
    # Partial products: 5*5*8 = 200 elements, LMUL=8
    li t0, KERNEL_DIM * KERNEL_DIM * CONV2_CHS/2
    vsetvli x0, t0, e32, m8, ta, ma
.endm

.macro VSETVL_CONV2_32F8_SUMVECS vecs lmul
    # Partial products: <vecs>*8 elements, LMUL=<lmul>
    .if \vecs * CONV2_CHS/2 < 32
    vsetivli x0, \vecs * CONV2_CHS/2, e32, \lmul, ta, ma
    .else
    li t0, \vecs * CONV2_CHS/2
    vsetvli x0, t0, e32, \lmul, ta, ma
    .endif
.endm

.macro VSETVL_CONV2_32F8_SUMVEC
    # Accumulator: CONV2_CHS/2 = 8 elements
    VSETVL_CONV2_32F8_SUMVECS 1 m1
.endm

.macro VSETVL_CONV2_32F8_SWB
    # Sliding window buffer: (5-1)*14+5 = 61 elements, LMUL=2
    li t0, (KERNEL_DIM - 1) * CONV1_POOL_DIM + KERNEL_DIM
    vsetvli x0, t0, e32, m2, ta, ma
.endm

.macro VSETVL_CONV2_32F8_SWB_INDEX
    # Sliding window buffer, 16-bit index elements for 'vrgatherei16': 5*5*8 = 200 elements, LMUL=4, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 8
    vsetvli x0, t0, e16, m4, ta, ma
.endm


##### Implementation #####

# void run_conv2_vec_32f8 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
#
#   Arguments:
#     a0 = in[CONV1_CHS][CONV1_POOL_DIM][CONV1_POOL_DIM]        ;  6 x 14 x 14
#     a1 = out[CONV2_CHS][CONV2_DIM][CONV2_DIM]                 ; 16 x 10 x 10
#     a2 = weight[CONV1_CHS][KERNEL_DIM][KERNEL_DIM][CONV2_CHS] ;  6 x  5 x  5 x 16
#     a3 = bias[CONV2_CHS]                                      ; 16
#
#   Stack:
#     SP - (CONV2_DIM * CONV2_DIM * CONV2_CHS/2 * 4) .. SP: Accumulator array (t5)
#
#   Register use:
#     a0 .. a3: pointers to the current elements (incremented inside the routine).
#     t0: temporary, potentially clobbered by macros
#     t1: end value for output address a1, stop condition for row loop
#     t2: end value for output address a1, stop condition for column loop
#     t3: input channel loop counter (counts from CONV1_CHS - 1 down to 0)
#     t4: pass counter (counts from 1 down to 0)
#     t5: accumulator base address on the stack (_out[CONV2_DIM][CONV2_DIM][CONV2_CHS/2])
#     t6, a4, ... a7: temporaries
#
#     v24 .. v31: weights (LMUL=8)
#     v16 .. v17: sliding window buffer (LMUL=2)
#     v8  .. v15: working register (LMUL=8) for products
#     v4  ..  v7: index vector for gathering kernel from sliding window buffer (LMUL=4, E16, VLEN=same as weights)
#     v3: bias
#     v0 .. v2: temporaries (LMUL=1), v0 may be required for masks
#
#  The implementation works similar to the implementation of 'run_conv1_vec_32f8',
#  with the following exceptions:
#
#  - Masking and weight vector expansion are not necessary (we have 2 x 8 outputs instead of 6).
#
#  - Two new outermost loops iterate over the weight chunks and input channels.
#
#  - Due to a lack of registers, each input layer is read twice, and a
#    weight chunk covers the weights of an input channel and half (8) of the output
#    channels. For 16-bit or smaller data types or larger VLEN values, this is not necessary,
#    a single weight chunk can potentially cover all of an input channel.
#
#  - Accumulator values need to be stored and read back between weight chunks.
#    For fast, non-strided access, the intermediate outputs are stored in a
#    temporary array on the stack.
#
#  - The sliding window buffer fits into 2 Vregs.
#
        .text
        .global run_conv2_vec_32f8
        .type   run_conv2_vec_32f8, @function
run_conv2_vec_32f8:

        # Reserve stack space ...
        li              t0, CONV2_DIM * CONV2_DIM * CONV2_CHS/2 * 4
        sub             sp, sp, t0

        # Preload index vector to select kernel elements in sliding window buffer (-> v4..v7) ...
        VSETVL_CONV2_32F8_SWB_INDEX
        la              t0, .conv2_window_idx_8
        vle16.v         v4, (t0)

        # Loop over passes ...
        li              t4, 2
.conv2_32f8_next_pass:

        # Load bias (-> v3) ...
        VSETVL_CONV2_32F8_SUMVEC
        vle32.v         v3, (a3)

        # Loop over input channels ...
        li              t3, CONV1_CHS
.conv2_32f8_next_input:
        addi            t3, t3, -1    # decrement loop counter in the beginning to simplify the detection of the last iteration

        # Reset accumulator base address ...
        mv              t5, sp

        # Load weights (-> v24..v31) ...
        VSETVL_CONV2_32F8_WEIGHTS
        li              t0, (KERNEL_DIM * KERNEL_DIM - 1) * CONV2_CHS * 4
        add             t6, a2, t0              # final value for loop
.conv2_32f8_l01:
        vslideup.vi     v8, v24, CONV2_CHS/2    # shift everything up
        VSETVL_CONV2_32F8_SUMVEC                # set VL to 8 (= 1 half vector)
        vle32.v         v8, (t6)                # load half vector of 8 elements
        VSETVL_CONV2_32F8_WEIGHTS
        vor.vx          v24, v8, zero           # move v8..v15 back to v24..v31
        addi            t6, t6, -CONV2_CHS * 4  # next vector
        bge             t6, a2, .conv2_32f8_l01

        # Init sliding window buffer for input channel (v16..v17) ...
        VSETVL_CONV2_32F8_SWB
        vle32.v         v16, (a0)
        addi            a0, a0, ((KERNEL_DIM - 1) * CONV1_POOL_DIM + KERNEL_DIM) * 4

        # Loop over output rows ...
        addi            t1, a1, CONV2_DIM * CONV2_DIM * 4   # end value for output address a1
.conv2_32f8_next_row:

        # Loop over output columns ...
        addi            t2, a1, CONV2_DIM * 4     # end value for output address a1
.conv2_32f8_next_column:

        # Get window (-> v8..v15) ...
        VSETVL_CONV2_32F8_PRODUCTS
        vrgatherei16.vv  v8, v16, v4
          # load window from v16..v17 into v8..v15, v4..v7 are index values

        # Multiply with weights ...
        vmul.vv         v8, v8, v24               # multiply with weights

        # Add value from accumulator to P[24] (unless in first input channel iteration) ...
        li              t0, CONV1_CHS - 1
        bge             t3, t0, .conv2_32f8_c02   # do not read and add in the first iteration
        VSETVL_CONV2_32F8_SUMVEC
        vle32.v         v2, (t5)
        vadd.vv         v14, v14, v2    # add to P[24]
.conv2_32f8_c02:

        # Accumulate (in parallel for all output channels) ...
        #   v8..v15[25][8] contains 25 vectors P[0] ... P[24] to be added.
        #   Each single vector register contains (up to) 4 vectors, e.g. P[0..3].
        # P[0..7] += P[8..15] ...
        VSETVL_CONV2_32F8_SUMVECS 8 m2
        vadd.vv         v8, v8, v10
        # P[0..7] += P[16..23] ...
        vadd.vv         v8, v8, v12
        # P[0..3] += P[4..7] ...
        VSETVL_CONV2_32F8_SUMVECS 4 m1
        vadd.vv         v8, v8, v9
        # P[0..1] += P[2..3] ...
        vslidedown.vi   v1, v8, 2*8
        VSETVL_CONV2_32F8_SUMVECS 2 m1
        vadd.vv         v8, v8, v1
        # P[0] += P[1] ...
        vslidedown.vi   v1, v8, 1*8
        VSETVL_CONV2_32F8_SUMVEC
        vadd.vv         v8, v8, v1
        # v1 = P[0] + P[24] ...
        vadd.vv         v1, v8, v14

        # In the last input channel iteration: Round, shift, add bias and write to output array ...
        bne             t3, zero, .conv2_32f8_c03

        VSETVL_CONV2_32F8_SUMVEC
        li              t0, 0x80          # DTYPE_POST_MUL(): round and ...
        vadd.vx         v1, v1, t0
        vsra.vi         v1, v1, 8         # ... shift-right

        vadd.vv         v1, v1, v3        # add bias

        li              t0, CONV2_DIM * CONV2_DIM * 4     # write to output array ...
        vsse32.v        v1, (a1), t0      # strided store
        j               .conv2_32f8_c04

.conv2_32f8_c03:

        # Not last input channel iteration: Write to accumulator array on stack ...
        vse32.v         v1, (t5)

.conv2_32f8_c04:
        addi            t5, t5, CONV2_CHS/2 * 4   # increment accumulator address
        addi            a1, a1, 4                 # increment output address

        # End of last row? ...
        bge             a1, t1, .conv2_32f8_c00

        # Load next element into sliding window buffer ...
        VSETVL_CONV2_32F8_SWB
        lw              t0, (a0)
        addi            a0, a0, 4
        vslide1down.vx  v16, v16, t0

        # End of row? If not: back to process next column ...
        blt             a1, t2, .conv2_32f8_next_column

        # End of row: Read borders into sliding window buffer ...
        .if KERNEL_DIM != 5
        .err  # "The following code asserts that KERNEL_DIM == 5"
        .endif
        VSETVL_CONV2_32F8_SWB
        lw              t0,  0(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0,  4(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0,  8(a0)
        vslide1down.vx  v16, v16, t0
        lw              t0, 12(a0)
        vslide1down.vx  v16, v16, t0
        addi            a0, a0, 16

        # Back to process next row ...
        j               .conv2_32f8_next_row

        # Next input channel ...
.conv2_32f8_c00:
        addi            a1, a1, -1 * CONV2_DIM * CONV2_DIM * 4            # reset output base address
        addi            a2, a2, KERNEL_DIM * KERNEL_DIM * CONV2_CHS * 4   # advance weight address
        bne             t3, zero, .conv2_32f8_next_input

        # Next pass ...
        addi            t4, t4, -1
        beq             t4, zero, .conv2_32f8_c01

        li              t0, -CONV1_CHS * CONV1_POOL_DIM * CONV1_POOL_DIM * 4
        add             a0, a0, t0                  # reset input base
        li              t0, CONV2_CHS/2 * CONV2_DIM * CONV2_DIM * 4
        add             a1, a1, t0                  # reset output base address to start of second half
        li              t0, - (CONV1_CHS * CONV2_CHS * KERNEL_DIM * KERNEL_DIM - CONV2_CHS/2) * 4
        add             a2, a2, t0                  # reset weight address to first unprocessed ouput channel
        addi            a3, a3, CONV2_CHS/2 * 4     # set bias base to second half
        j               .conv2_32f8_next_pass

        # Done ...
.conv2_32f8_c01:

        # Cleanup stack ...
        li              t0, CONV2_DIM * CONV2_DIM * CONV2_CHS/2 * 4
        add             sp, sp, t0
        ret
        .size   run_conv2_vec_32f8, .-run_conv2_vec_32f8





######################### run_conv2_vec_16f4 ###################################


##### Macros to set common vector formats #####

# Note: These macros may clobber 't0'!

.macro VSETVL_CONV2_16F4_WEIGHTS
    # Weights: 5*5*16 = 400 elements, LMUL=8 (optionally in a different format than products)
    li t0, KERNEL_DIM * KERNEL_DIM * CONV2_CHS
    vsetvli x0, t0, e16, m8, ta, ma
.endm

.macro VSETVL_CONV2_16F4_PRODUCTS
    # Partial products: 5*5*16 = 400 elements, LMUL=8
    li t0, KERNEL_DIM * KERNEL_DIM * CONV2_CHS
    vsetvli x0, t0, e16, m8, ta, ma
.endm

.macro VSETVL_CONV2_16F4_PRODUCTS_HALF
    # Partial products: 5*5*16/2 = 200 elements, LMUL=4
    li t0, KERNEL_DIM * KERNEL_DIM * CONV2_CHS/2
    vsetvli x0, t0, e16, m4, ta, ma
.endm

.macro VSETVL_CONV2_16F4_SUMVECS vecs lmul
    # Partial products: <vecs>*16 elements, LMUL=<lmul>
    .if \vecs * CONV2_CHS < 32
    vsetivli x0, \vecs * CONV2_CHS, e16, \lmul, ta, ma
    .else
    li t0, \vecs * CONV2_CHS
    vsetvli x0, t0, e16, \lmul, ta, ma
    .endif
.endm

.macro VSETVL_CONV2_16F4_SUMVEC
    # Accumulator: CONV2_CHS = 16 elements
    VSETVL_CONV2_16F4_SUMVECS 1 m1
.endm

.macro VSETVL_CONV2_16F4_SWB
    # Sliding window buffer: (5-1)*14+5 = 61 elements, LMUL=1
    li t0, (KERNEL_DIM - 1) * CONV1_POOL_DIM + KERNEL_DIM
    vsetvli x0, t0, e16, m1, ta, ma
.endm

.macro VSETVL_CONV2_16F4_SWB_INDEX
    # Sliding window buffer, 16-bit index elements for 'vrgatherei16': 5*5*16 = 400 elements, LMUL=8, E16
    li t0, KERNEL_DIM * KERNEL_DIM * 16
    vsetvli x0, t0, e16, m8, ta, ma
.endm


##### Implementation #####

# void run_conv2_vec_16f4 (DTYPE *in, DTYPE *out, DTYPE *weight, DTYPE *bias);
#
#   Arguments:
#     a0 = in[CONV1_CHS][CONV1_POOL_DIM][CONV1_POOL_DIM]        ;  6 x 14 x 14
#     a1 = out[CONV2_CHS][CONV2_DIM][CONV2_DIM]                 ; 16 x 10 x 10
#     a2 = weight[CONV1_CHS][CONV2_CHS][KERNEL_DIM][KERNEL_DIM] ;  6 x 16 x  5 x  5
#     a3 = bias[CONV2_CHS]                                      ; 16
#
#   Stack:
#     SP - (CONV2_DIM * CONV2_DIM * CONV2_CHS/2 * 4) .. SP: Accumulator array (t5)
#
#   Register use:
#     a0 .. a3: pointers to the current elements (incremented inside the routine).
#     t0: temporary, potentially clobbered by macros
#     t1: end value for output address a1, stop condition for row loop
#     t2: end value for output address a1, stop condition for column loop
#     t3: input channel loop counter (counts from CONV1_CHS - 1 down to 0)
#     t5: accumulator base address on the stack (_out[CONV2_DIM][CONV2_DIM][CONV2_CHS])
#     t4, t6: temporaries
#
#     v24 .. v31: weights (for both chunks) (LMUL=8)
#     v16 .. v23: index vector for gathering kernel from sliding window buffer (LMUL=8, E16, VLEN=same as weights)
#     v10 .. v11: temporaries (LMUL=1)
#     v9: bias
#     v8: sliding window buffer (LMUL=1), must be 8-reg-aligned, since it is used by gather operation
#     v0  ..  v7: working register(s) (LMUL=8) for products
#
#  The implementation is similar to the implementation of 'run_conv2_vec_32f8',
#  with the following exceptions:
#
#  - There are sufficient registers to keep the weights and accumulated data of
#    all outputs at once. Hence, the outer loop of passes can be eleminated.
#    Image data only needs to be read once, and the accumulator only needs to
#    be written once.
#
#  - The sliding window buffer fits into 1 Vreg (instead of 2).
#
        .text
        .global run_conv2_vec_16f4
        .type   run_conv2_vec_16f4, @function
run_conv2_vec_16f4:

        # Reserve stack space ...
        li              t0, CONV2_DIM * CONV2_DIM * CONV2_CHS * 2
        sub             sp, sp, t0

        # Preload index vector to select kernel elements in sliding window buffer (-> v16..v23) ...
        VSETVL_CONV2_16F4_SWB_INDEX
        la              t0, .conv2_window_idx_16
        vle16.v         v16, (t0)

        # Load bias (-> v3) ...
        VSETVL_CONV2_16F4_SUMVEC
        vle16.v         v9, (a3)

        # Loop over input channels ...
        li              t3, CONV1_CHS
.conv2_16f4_next_input:
        addi            t3, t3, -1    # decrement loop counter in the beginning to simplify the detection of the last iteration

        # Reset accumulator base address ...
        mv              t5, sp

        # Load weights (-> v24..v31) ...
        VSETVL_CONV2_16F4_WEIGHTS
        vle16.v         v24, (a2)

        # Init sliding window buffer for input channel (v4) ...
        VSETVL_CONV2_16F4_SWB
        vle16.v         v8, (a0)
        addi            a0, a0, ((KERNEL_DIM - 1) * CONV1_POOL_DIM + KERNEL_DIM) * 2

        # Loop over output rows ...
        addi            t1, a1, CONV2_DIM * CONV2_DIM * 2   # end value for output address a1
.conv2_16f4_next_row:

        # Loop over output columns ...
        addi            t2, a1, CONV2_DIM * 2     # end value for output address a1
.conv2_16f4_next_column:

        # Get window (-> v0..v7) ...
        VSETVL_CONV2_16F4_PRODUCTS
        vrgatherei16.vv v0, v8, v16
          # load window from v8 into v0..v7, v16..v23 are index values

        # Multiply with weights ...
        vmul.vv         v0, v0, v24               # multiply with weights

        # Add value from stored accumulator to P[24] (unless in first input channel iteration) ...
        li              t0, CONV1_CHS - 1
        bge             t3, t0, .conv2_16f4_c02   # do not read and add in the first iteration
        VSETVL_CONV2_16F4_SUMVEC
        vle16.v         v10, (t5)
        vadd.vv         v3, v3, v10               # add to P[24]
.conv2_16f4_c02:

        # Accumulate (in parallel for all output channels) ...
        #   v0..v7[25][16] contains 25 vectors P[0] ... P[24] to be added.
        #   Each single vector register contains (up to) 4 vectors, e.g. P[0..3].

        # P[0..7] += P[8..15] ...
        VSETVL_CONV2_16F4_SUMVECS 8 m2
        vadd.vv         v0, v0, v2
        # P[0..7] += P[16..23] ...
        vadd.vv         v0, v0, v4
        # P[0..3] += P[4..7] ...
        VSETVL_CONV2_16F4_SUMVECS 4 m1
        vadd.vv         v0, v0, v1
        # P[0..1] += P[2..3] ...
        li              t0, 2*16
        vslidedown.vx   v1, v0, t0
        VSETVL_CONV2_16F4_SUMVECS 2 m1
        vadd.vv         v0, v0, v1
        # P[0] += P[1] ...
        vslidedown.vi   v1, v0, 1*16
        VSETVL_CONV2_16F4_SUMVEC
        vadd.vv         v0, v0, v1
        # v1 = P[0] + P[24] ...
        vadd.vv         v1, v0, v6

        # In the last input channel iteration: Round, shift, add bias and write to output array ...
        bne             t3, zero, .conv2_16f4_c03

        VSETVL_CONV2_16F4_SUMVEC
        vadd.vi         v1, v1, 8         # DTYPE_POST_MUL(): round and ...
        vsra.vi         v1, v1, 4         # ... shift-right

        vadd.vv         v1, v1, v9        # add bias

        li              t0, CONV2_DIM * CONV2_DIM * 2     # write to output array ...
        vsse16.v        v1, (a1), t0      # strided store
        j               .conv2_16f4_c04

.conv2_16f4_c03:

        # Not last input channel iteration: Write to accumulator array on stack ...
        vse16.v         v1, (t5)

.conv2_16f4_c04:
        addi            t5, t5, CONV2_CHS * 2   # increment accumulator address
        addi            a1, a1, 2               # increment output address

        # End of last row? ...
        bge             a1, t1, .conv2_16f4_c00

        # Load next element into sliding window buffer ...
        VSETVL_CONV2_16F4_SWB
        lh              t0, (a0)
        addi            a0, a0, 2
        vslide1down.vx  v8, v8, t0

        # End of row? If not: back to process next column ...
        blt             a1, t2, .conv2_16f4_next_column

        # End of row: Read borders into sliding window buffer ...
        .if KERNEL_DIM != 5
        .err  # "The following code asserts that KERNEL_DIM == 5"
        .endif
        VSETVL_CONV2_16F4_SWB
        lh              t0, 0(a0)
        vslide1down.vx  v8, v8, t0
        lh              t0, 2(a0)
        vslide1down.vx  v8, v8, t0
        lh              t0, 4(a0)
        vslide1down.vx  v8, v8, t0
        lh              t0, 6(a0)
        vslide1down.vx  v8, v8, t0
        addi            a0, a0, 4*2

        # Back to process next row ...
        j               .conv2_16f4_next_row

        # Next input channel ...
.conv2_16f4_c00:
        addi            a1, a1, -1 * CONV2_DIM * CONV2_DIM * 2            # reset output base address
        addi            a2, a2, KERNEL_DIM * KERNEL_DIM * CONV2_CHS * 2   # advance weight address to next input channel
        bne             t3, zero, .conv2_16f4_next_input

        # Done ...
.conv2_16f4_c01:

        # Cleanup stack ...
        li              t0, CONV2_DIM * CONV2_DIM * CONV2_CHS * 2
        add             sp, sp, t0
        ret
        .size   run_conv2_vec_16f4, .-run_conv2_vec_16f4





################################################################################
#                                                                              #
#                        Fully-Connected Layers                                #
#                                                                              #
################################################################################



######################### run_fc_vec_32f8 ######################################


# void run_fc_vec_32f8 (DTYPE *out, int out_dim, DTYPE *in, int in_dim, DTYPE *weight, DTYPE *bias);
#   a0 = in[in_dim],   a1 = in_dim (-> loop counter)
#   a2 = out[out_dim], a3 = out_dim
#   a4 = weight[in_dim][out_dim]
#   a5 = bias[out_dim]
#
#   v4 .. v7 = accumulator
#
        .global run_fc_vec_32f8
        .type   run_fc_vec_32f8, @function
run_fc_vec_32f8:

        # Init accumulator ...
        vsetvli   zero, a3, e32, m4, ta, ma
          # This works for up to 128 outputs (VLEN=1024, VEW=32, LMUL=4);
          # no need for an inner loop.
        vle32.v   v4, (a5)      # preload accumulator with bias
        vsll.vi   v4, v4, 8     # shift left by number of fractional bits
        slli      t1, a3, 2     # t1 := stride for weight loading

.fc_vec_32f8_l020:
        lw        t2, (a0)      # load input
        vle32.v   v8, (a4)      # load weight row
        vmul.vx   v8, v8, t2    # multiply
        vadd.vv   v4, v4, v8    # accumulate

        # end of iteration ...
        addi      a1, a1, -1    # decrement loop counter
        addi      a0, a0, 4     # next input element
        add       a4, a4, t1    # next row in weight array
        bne       a1, zero, .fc_vec_32f8_l020

.fc_vec_32f8_c020:
        li        t3, 128       # DTYPE_POST_MUL(): round and ...
        vadd.vx   v4, v4, t3
        vsra.vi   v4, v4, 8     # ... shift-right
        vse32.v   v4, (a2)      # write results
        ret
        .size   run_fc_vec_32f8, .-run_fc_vec_32f8



######################### run_fc_vec_16f4 ######################################


# void run_fc_vec_16f4 (DTYPE *out, int out_dim, DTYPE *in, int in_dim, DTYPE *weight, DTYPE *bias);
#   a0 = in,  a1 = in_dim (-> loop counter)
#   a2 = out, a3 = out_dim
#   a4 = weight[in_dim][out_dim]
#   a5 = bias[out_dim]
#
#   v4 .. v5 = accumulator
#
        .global run_fc_vec_16f4
        .type   run_fc_vec_16f4, @function
run_fc_vec_16f4:

        # Init accumulator ...
        vsetvli   zero, a3, e16, m2, ta, ma
          # This works for up to 128 outputs (VLEN=1024, VEW=32, LMUL=2);
          # no need for an inner loop.
        vle16.v   v4, (a5)      # preload accumulator with bias
        vsll.vi   v4, v4, 4     # shift left by number of fractional bits
        slli      t1, a3, 1     # t1 := stride for weight loading

.fc_vec_16f4_l020:
        lh        t2, (a0)      # load input
        vle16.v   v8, (a4)      # load weight row
        vmul.vx   v8, v8, t2    # multiply
        vadd.vv   v4, v4, v8    # accumulate

        # end of iteration ...
        addi      a1, a1, -1    # decrement loop counter
        addi      a0, a0, 2     # next input element
        add       a4, a4, t1    # next row in weight array
        bne       a1, zero, .fc_vec_16f4_l020

.fc_vec_16f4_c020:
        vadd.vi   v4, v4, 8     # DTYPE_POST_MUL(): round and ...
        vsra.vi   v4, v4, 4     # ... shift-right
        vse16.v   v4, (a2)      # write results
        ret
        .size   run_fc_vec_16f4, .-run_fc_vec_16f4
