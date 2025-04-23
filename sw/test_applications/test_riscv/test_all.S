/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
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


.section ".text.init"
.globl _start

_start:
				# make sure only CePU proceeds

				beqz x1, loadstore


#### TEST Load and Store ####
loadstore:	    la  x1, result
			    li  x2, 0xCAFEBABE
			   	sw  x2, 0(x1)

			    lw  x3, 0(x1)
			    bne x2, x3, end

			    la  x15, result_end
				li  x14, 0xDEADBEEF
				lw  x13, 0(x15)
				bne x13, x14, end

				# Test LB
			    li  x2, 0xffffffBE
			    lb  x3, 0(x1)  # x3 = 0xBE
			    li  x4, 0xffffffBA
			    lb  x5, 1(x1)  # x5 = 0xBA
			    li  x6, 0xffffffFE
			    lb  x7, 2(x1)  # x5 = 0xFE
			    li  x8, 0xffffffCA
			    lb  x9, 3(x1)  # x5 = 0xCA
			    bne x2, x3, end
			    bne x4, x5, end
			    bne x6, x7, end
			    bne x8, x9, end

			    # Test LH
			    li  x2, 0xffffBABE
			    lh  x3, 0(x1)  # x3 = 0xBABE
			    li  x4, 0xffffCAFE
			    lh  x5, 2(x1)  # x5 = 0xCAFE
			    bne x2, x3, end
			    bne x4, x5, end

				# Test LHU
			    li  x2, 0xBABE
			    lhu x3, 0(x1)  # x3 = 0xBABE
			    li  x4, 0xCAFE
			    lhu x5, 2(x1)  # x5 = 0xCAFE
			    bne x2, x3, end
			    bne x4, x5, end


			    # Test LBU
			  	li  x2, 0xBE
			    lbu x3, 0(x1)  # x3 = 0xBE
			    li  x4, 0xBA
			    lbu x5, 1(x1)  # x5 = 0xBA
			    li  x6, 0xFE
			    lbu x7, 2(x1)  # x5 = 0xFE
			    li  x8, 0xCA
			    lbu x9, 3(x1)  # x5 = 0xCA
			    bne x2, x3, end
			    bne x4, x5, end
			    bne x6, x7, end
			    bne x8, x9, end

			    # Test SH
			    la  x1, temp
			    li  x2, 0xCAFE
			    li  x3, 0xBABE
			    li  x5, 0xCAFEBABE
			    sh  x2, 2(x1)

			    sh  x3, 0(x1)

			    lhu  x4, 2(x1)
			    bne x2, x4, end
			    lhu  x4, 0(x1)
			    bne x3, x4, end
			    lw  x4, 0(x1) # x4 = 0xCAFEBABE
			    bne x5, x4, end


			    # Test SB
			    li  x2, 0xA5
			    li  x5, 0xA2A3A4A5
			    sb  x2, 0(x1)
			    lbu  x4, 0(x1)
			    bne x2, x4, end
			    addi x2, x2, -1
			    sb  x2, 1(x1)
			    lbu  x4, 1(x1)
			    bne x2, x4, end
			    addi x2, x2, -1
			    sb  x2, 2(x1)
			    lbu  x4, 2(x1)
			    bne x2, x4, end
			    addi x2, x2, -1
			    sb  x2, 3(x1)
			    lbu  x4, 3(x1)
			    bne x2, x4, end

			    lw  x3, 0(x1) # x3 = 0xA2A3A4A5
			    bne x3, x5, end

			    la  x1, load_ok
			    la  x2, store_ok
			    sw  x0, 0(x1)
			    sw  x0, 0(x2)

#### Load and Store passed ####



#### ALU immediate instructions  ####
    			.global alu_imm
alu_imm:		nop
			    la    x31, alu_imm_ok

			    # addi
			    addi  x1, x0, 0x123
			    addi  x1, x1, -0x223
			    addi  x1, x1, 0x100
			    sb    x1, 0(x31)      # add ok

			    # ori
			    ori   x0, x0, 0x7ff  # must be 0!
			    ori   x1, x1, 0x123
			    ori   x1, x1, 0x321  # x1 = 0x323
			    addi  x2, x1, -0x323 # x1 = 0
			    sb    x2, 1(x31)      # or ok

			    # and
			    andi  x1, x1, 0x7f0  # x1 = 0x320
			    andi  x1, x1, 0x3ff  # x1 = 0x320
			    addi  x1, x1, -0x320 # x1 = 0
			    sb    x1, 2(x31)      # and ok

			    # xori
			    xori  x1, x2, 0x7ff  # x1 = 0x7ff
			    xori  x1, x1, 0x7ff  # x1 = 0
			    sb    x1, 3(x31)      # xor ok

			    # sltiu
			    li    x1, 0x12345678
			    sltiu x2, x1, 0xFFFFFFFF # x2 = 1
			    sltiu x2, x2, 1 # x2 = 0
			    sb    x2, 4(x31)      # sltiu ok

			    # slti
			    slti x2, x1, -1 # x2 = 1
			    slti x2, x2, 1 # x2 = 0
			    sb    x2, 5(x31)      # slti ok

			    # slli
			    li    x1, 0x0f0f0f0f
			    slli  x1, x1, 4
			    li    x2, 0xf0f0f0f0
			    bne   x1, x2, end
			    sb    x2, 6(x31)      # slli ok

			    # srli
			    srli  x2, x2, 4
			    li    x1, 0x0f0f0f0f
			    bne   x1, x2, end
			    sb    x2, 7(x31)      # srli ok

			    # srai
			    li    x2, 0x80000000
			    srai  x2, x2, 16
			    li    x1, 0xffff8000
			    bne   x1, x2, end
			    sb    x2, 8(x31)      # srai ok

#### ALU immediate passed ####



#### ALU register instructions  ####
    			.global alu
alu:			nop
			    la    x31, alu_ok

			    # add
			    li    x1, 0x12345678
			    li    x2, 0xCAFEBABE
			    add   x3, x1, x2
			    li    x4, 0xDD331136
			    bne   x3, x4, end
			    sb    x3, 0(x31)      # add ok

			    # sub
			    sub   x3, x3, x1
			    bne   x2, x3, end
			    sb    x3, 1(x31)      # sub ok

			    # or
			    ori   x1, x1, 0x123
			    ori   x2, x2, 0x432
			    or    x3, x1, x2
			    li 	  x4, 0xDAFEFFFF
			    bne   x3, x4, end
			    sb    x3, 2(x31)      # or ok

			    # and
				li x1, 0xaaaaaaaa
				li x2, 0x55555555
				li x4, 0x00000000
			    and   x3, x1, x2
			    bne   x3, x4, end
			    sb    x3, 3(x31)      # and ok

			    # xor
			    li    x1, 0xF0F0F0F0
			    li    x2, 0x5A5A5A5A
			    xor   x3, x1, x2
			    li    x4, 0xAAAAAAAA
			    bne   x3, x4, end
			    sb    x3, 4(x31)      # xor ok

			    # sltu
			    li    x1, 0x12345678
			    li    x2, 0xFFFFFFFF
			    sltu  x3, x1, x2 # x3 = 1
			    beq   x3, x0, end
			    sb    x3, 5(x31)      # sltu ok

			    # slt
			    li    x1, -1
			    li    x2, 0x12345678
			    slt   x3, x1, x2 # x3 = 1
			    beq   x3, x0, end
			    sb    x3, 6(x31)      # slt ok

			    # sll
			    li    x1, 0x0f0f0f0f
			    li    x2, 4
			    sll   x1, x1, x2
			    li    x3, 0xf0f0f0f0
			    bne   x1, x3, end
			    sb    x1, 7(x31)      # sll ok

			    # srl
			    srl   x1, x1, x2
			    li    x3, 0x0f0f0f0f
			    bne   x1, x3, end
			    sb    x1, 8(x31)      # srl ok

			    # sra
			    li    x1, 0x80000000
			    li    x2, 16
			    sra   x1, x1, x2
			    li    x3, 0xffff8000
			    bne   x1, x3, end
			    sb    x1, 9(x31)      # sra ok

#### ALU register passed ####

#### Control flow ####

control_flow:	la    x31, cf_ok
		li    x1, 2
		li    x2, 1
		beq   x1, x2, end # if 1 == 2
		sb    x0, 0(x31)   # beq ok
		bne   x1, x1, end # if 2 != 2
		sb    x0, 1(x31)   # bne ok
		blt   x1, x2, end # if 2 < 1
		sb    x0, 2(x31)   # blt ok
		bltu  x1, x2, end # if 2 < 1
		sb    x0, 3(x31)   # bltu ok
		ble   x1, x2, end # if 2 <= 1
		sb    x0, 4(x31)   # ble ok
		bleu  x1, x2, end # if 2 <= 1
		sb    x0, 5(x31)   # bleu ok
		bge   x2, x1, end # if 2 >= 1
		sb    x0, 6(x31)   # bge ok
		bgeu  x2, x1, end # if 2 >= 1
		sb    x0, 7(x31)   # bgeu ok
		jal   x1, 2f
		sb    x0, 8(x31)   # jal ok

2:
# Initialize the registers
   		li t0, 0x123          # t0 = 0
    	la t1, target     # t1 = address of target label
    	li t2, 0xDEADBEEF # t2 = some value to check after jump

   		# Store return address in ra and jump to target
   		jalr ra, t1, 0

    # This instruction should not execute if jump was successful
    li t0, 0xBAD

    # Check if t0 is not changed
    beq t0, zero, end
    li t0, 0xBAD

end:
    # Halt (pseudo-instruction for ebreak)
    ebreak

target:
    # After jump, set t0 to t2
    mv t0, t2

    # Return to the address in ra
    jr ra


#### Data ####
.section ".data"

.global result
result:        .word 0x12345678
result_end:    .word 0xDEADBEEF
temp:	         .word 0xFFFFFFFF
load_ok:       .byte 0xFF
store_ok:      .byte 0xFF
alu_imm_ok:    .byte 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
alu_ok:        .byte 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
cf_ok:         .byte 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF

.data_end: .word 0xFFFFFFFF
