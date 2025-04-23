
# The Minimal Nucleus
**Authors: Lorenz Sommer 2024, Johannes Hofmann 2025**

```{contents} nucleus submodules
    :local:
    :depth: 2
```


This chapter gives an overview of the minimal nucleus' capabilities and details its submodules.

## Overview

The main purpose of the minimal nucleus is to provide a well documented and minimal implementation
of the [RV32I RISC-V subset](https://drive.google.com/file/d/1uviu1nH-tScFfgrovvFCrj7Omv8tFtkp/view).


### Capabilities
The minimal nucleus largely supports running of programs compiled by the RISC-V GCC toolchain. CSR instructuions
are unsupported with this initial implementation. The nucleus is capable of running programs that do not rely on external
peripherals or interrupts. It does not contain any pipelines and does not support multi-nucleus architectures.



### Memory
The minimal nucleus is intended for use with byte-addressable memory.
It interfaces with a so called "memory unit" to access memory. It does not  access memory directly by itself.
Communication with the memory unit is split into two ports, the instruction port for instruction fetching, and the data
port for `load` and `store` operations. For these ports, a [custom communication protocol](sec:idef:wishbone) has been created.
The memory unit is connected to the internal system bus and acts as an interface for the nucleus' requests to the system bus.

### Working principle
The minimal nucleus utilizes a simple fetch-decode-execute workflow.
The `decode` step is always executed within one clock cycle. All instructions that are not `load` or `store`
instructions, also execute within one clock cycle. The `fetch` step is a IPort memory transaction and takes
3 clock cycles to complete. `load` operations require 4 clock cycles, store operations require 3.
For the time being, the ECALL and EBREAK operations halt the processor. The FENCE operation is implemented as a
NOP operation.



```{figure} figures/minimalnucleus/nucleus.drawio.svg
:align: center
Minimal nucleus block diagram
```



(sec:nucleus:alu)=
## Arithmetic Logic Unit (ALU)

The ALU (Arithmetic Logic Unit) performs arithmetic and logical operations on the two 32-bit operands A and B.
It is implemented using combinatorics and is not clock synchronous.


```{list-table} ALU input ports
:name: "tab:nucleus:alu_input_ports"
:header-rows: 1

*   - Port name
    - Width in bits
    - Description
*   - A
    - 32
    - Operand A.
*   - B
    - 32
    - Operand B.
*   - select_op
    - 3
    - 3-bit control signal that selects the operation to be performed. \
      Derived from IR[14:12].
*   - funct7_flag
    - 1
    - Decides between ADD/SUB, SRA/SRL operations. \
      Derived from IR[30].
*   - force_add
    - 1
    - If set, forces the ALU to add the operands.
```



```{list-table} ALU output ports
:name: "tab:nucleus:alu_output_ports"
:header-rows: 1

*   - Port name
    - Width in bits
    - Description
*   - Y
    - 32
    - Result of the logical operation performed on operands A and B.
*   - equal
    - 1
    - Status signal, 1 if a == b, else 0.
*   - less
    - 1
    - Status signal, 1 if a < b, else 0.
*   - lessu
    - 1
    - Status signal, 1 if a < b, else 0 (unsigned).
```

Given the input signals `funct3` and `funct7_flag`, the ALU performs the following operations on the operands A and B.

```{list-table} ALU operations
:name: "tab:nucleus:alu_operations"
:header-rows: 1

*   - Operation
    - Shorthand
    - funct3
    - funct7
    - Description
*   - ADD
    - y = a + b
    - 0x0
    - 0
    - Regular addition
*   - SUB
    - y = a - b
    - 0x0
    - 1
    - Two's complement of B is added to A. R-Type only.
*   - AND
    - y = a & b
    - 0x7
    - 0
    - Bitwise AND
*   - OR
    - y = a | b
    - 0x6
    - 0
    - Bitwise OR
*   - XOR
    - y = a ^ b
    - 0x4
    - 0
    - Bitwise XOR
*   - SLL
    - y = a << b
    - 0x1
    - 0
    - Shift left logical
*   - SRL
    - y = a >> b
    - 0x5
    - 0
    - Shift right logical
*   - SRA
    - y = a >> b
    - 0x5
    - 1
    - Shift right arithmetic (sign extends)
*   - SLT
    - y = a < b ? 1,0
    - 0x2
    - 0
    - Set less than
*   - SLTU
    - y = a < b ? 1,0
    - 0x3
    - 0
    - Set less than unsigned (zero extends)
```

```{figure} figures/minimalnucleus/alu.drawio.svg
:align: center
ALU module symbol
```


The ALU can handle R- and I-Type instructions. In order to correctly implement the few differences the two instruction
types have, a few distinctions have to be made. This concerns the fact, that there is no I-Type `subi` instruction.
As seen in [Table 3](tab:nucleus:alu_operations), `funct7` decides whether subtraction is performed instead of
addition and whether to shift logically or arithmetically. According to the RISC-V
specification the only relevant values of the `funct7` block are `0x0` and `0x2`. This allows for the whole block
to be reduced to its 6th bit, which equals the 30th bit of the whole instruction and thus `IR[30]`. Additionally,
even though I-Type instructions get decoded into their internal immediate value, the position of the `funct7` block
remains the same.

Because there is no specified `subi` instruction in the specification, a simple forwarding of `IR[30]` to the ALU input
`funct7_flag` would lead to erroneous results. This leads to a necessary separation of the I-Type instructions into the
ones that care about the `funct7` block and the ones that do not. More specifically, the I-Type shift operations must be separated.
For this purpose the controller outputs a control signal `c_funct7_flag`. It decides whether the `funct7_flag` input of
the ALU is set to `0x0` or the bit `IR[30]`, depending on the `funct3` block.

```{list-table} funct7_flag truth table
:name: "nucleus:tab:instruction_types"
:header-rows: 1

*   - Instruction type
    - funct3
    - funct7_flag (ALU input)
    - c_funct7_flag (Control signal)
*   - R-Type
    - 0x0
    - IR[30]
    - 0x1
*   - R-Type
    - 0x5
    - IR[30]
    - 0x1
*   - I-Type
    - 0x0
    - 0x0
    - 0x0
*   - I-Type
    - 0x5
    - IR[30]
    - 0x1
```

````{note}

This decision logic is implemented in the top level module `nucleus.cpp`

```{sourcecode} cpp
if (signal_c_funct7_flag == 0x1)
{
    signal_funct7 = signal_ir_out.read().range(30, 30);
}
else
{
    signal_funct7 = 0x0;
}
```

`signal_funct7` is then forwarded to the ALU input `funct7_flag`.

````

Next to R- and I-Type instructions, the ALU is also indirectly used to perform address calculations for the load, store,
branch and jump instructions. It also performs the additions necessary for the LUI and AUIPC instructions. All of
the previously named instructions use the `force_add` flag to force the ALU to perform addition. Depending on the
instruction, the operands are selected accordingly by the control unit (controller). Possible sources for operand A
are `rs1` and `pc_out` (current program counter). Possible operands for operand B are `rs2` and `imm_out` (immediate value).

(sec:nucleus:regfile)=
## Regfile

The regfile component of the nucleus is used to store temporary values that are needed for calculations
and program flow. It contains 32 32-bit registers.

```{list-table} Regfile input ports
:name: "nucleus:tab:regfile_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - data_in
    - 32
    - Data input
*   - select_in
    - 5
    - Selects the register in which input data is stored
*   - rs1_select_in
    - 5
    - Selects which registers value gets output to output port `rs1_out`
*   - rs2_select_in
    - 5
    - Selects which registers value gets output to output port `rs2_out`
*   - en_load_in
    - 1
    - Control signal to enable/disable storing of input data with the next rising edge.
```

```{list-table} Regfile output ports
:name: "nucleus:tab:regfile_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - rs1_out
    - 32
    - Output of the register selected by `rs1_select_in`
*   - rs2_out
    - 32
    - Output of the register selected by `rs2_select_in`
```

```{figure} figures/minimalnucleus/regfile.drawio.svg
:align: center
Regfile module symbol
```

The register `x0` holds the permanent value `0x00000000` according to the RISC-V specification.
The reset value of all registers is `0x00000000`.

(sec:nucleus:pc)=
## Program Counter (PC)

The program counter module is used to store the address of the current instruction at any given time. Its content
can be thought of as the current position within the program. The program counter is incremented by `0x4` with the next
rising edge, once the controller sets the `s_pc_inc4` control signal. Its internal value is be overwritten with the value
at its input port with the next rising edge if `s_pc_ld_en` is set. This is used to perform jump and branch instrucitons.


```{list-table} Program counter input ports
:name: "nucleus:tab:pc_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - pc_in
    - 32
    - Data input for when the PC is to be manipulated by jump and branch instructions.
*   - inc_in
    - 1
    - Control signal. When set, the PC is incremented by `0x4` at the next rising edge.
*   - en_load_in
    - 1
    - Control signal. Enables loading of a new value with the next rising edge.
```

```{note} Program counter internal register
The internal register of the program counter is **30 bits wide**. The lowest two bits of any given address pertaining
to the main program in memory can be omitted because every instruction is naturally word aligned and 4 bytes in size.
To maintain a level of consistency for the program counters interactions with other modules and the instruction port
interface, the output and input ports kept at a width of 32 bits. The lowest two bits are simply set to a constant `0` at the output and dismissed at the input. Instead of
every connected module having to append these two zeroes at their inputs, it is done once at the PCs output. The same goes for the PC input port.
```

```{list-table} Program counter output ports
:name: "nucleus:tab:pc_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - pc_out
    - 32
    - Constant output of the internal register.
```

```{figure} figures/minimalnucleus/pc.drawio.svg
:align: center
Program counter module symbol
```

(sec:nucleus:ir)=
## Instruction Register (IR)

The instruction register module stores the current instruction in its internal register and provides its content
to the rest of the nucleus via a constant output.

```{list-table} Instruction register input ports
:name: "nucleus:tab:ir_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - ir_in
    - 32
    - Input data. Internal register takes on this value at the next rising edge.
*   - en_load_in
    - 1
    - Control signal. Enables loading of a new value with the next rising edge.
```

```{list-table} Instruction register output ports
:name: "nucleus:tab:ir_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - ir_out
    - 32
    - Constant output of the internal register.
```

The IR input port `data_in` is connected to the IPort `rdata` signal.


```{figure} figures/minimalnucleus/ir.drawio.svg
:align: center
Instruction register module symbol
```

(sec:nucleus:immgen)=
## Immediate Generator (immgen)

The immediate generator (immgen) module decodes the current instruction and generates an immediate value from it.
All but the R-Type instruction type carry an immediate value. This immediate value is a simple constant built into
an instruction.

```{list-table} Immediate generator input ports
:name: "nucleus:tab:immgen_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - data_in
    - 32
    - Instruction word from which an immediate value is to be generated from.
```

```{list-table} Immediate generator output ports
:name: "nucleus:tab:immgen_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - imm_out
    - 32
    - Immediate value generated from an instruction word.
```

Immediate values are decoded differently and serve a different purpose depending on the instruction and instruction type.
Decoding and generation are executed in accordance with the [RISC-V specification](https://drive.google.com/file/d/1uviu1nH-tScFfgrovvFCrj7Omv8tFtkp/edit)
page 44, section "Immediate Encoding Variants.

```{list-table} Immediate encoding variants
:name: "nucleus:tab:immgen_decoding"
:header-rows: 1

*   - Instruction Type
    - Resulting immediate value
*   - I-Type
    - `(inst[31])[31:11] + inst[30:25] + inst[24:21] + inst[20]`
*   - S-Type
    - `(inst[31])[31:11] + inst[30:25] + inst[11:8]  + inst[7]`
*   - B-Type
    - `(inst[31])[31:12] + inst[7]     + inst[30:25] + inst[11:8]  + 0`
*   - U-Type
    - `inst[31]          + inst[30:20] + inst[19:12] + 0[11:0]`
*   - J-Type
    - `(inst[31])[31:20] + inst[19:12] + inst[20]    + inst[30:25] + inst[24:21] + 0`

```

```{admonition} How to read the immediate encoding table
:class: tip

In the table above, `inst` stands for the current instruction word from which an immediate value is to be generated from.
The operator `+` in this case stands for concatenation of bits/bit-ranges.
Additionally, syntax like `(inst[31])[31:12]` resolves as: "fill the range `[31:12]` in the output value with `inst[31]`".
```

The immediate generator module detects the correct decoding variant by evaluating the `opcode` field of the current
instruction. Groups of instructions in the RV32I subset (load/store/immediate/branch/jump/upper immediate) do not
always use the same instruction type format across the board.

```{list-table} Instructions by instruction type
:name: "nucleus:tab:instruction_type_per_instruction"
:header-rows: 1

*   - Instruction type
    - Instructions
*   - I-Type
    - All "immediate arithmetic" instructions, all `load` instructions, **`jalr`**
*   - S-Type
    - All `store` instructions
*   - B-Type
    - All `branch` instructions
*   - U-Type
    - `lui`, `auipc`
*   - J-Type
    - `jal`
```

```{note}
The R-Type instruction format is not listed, since it does not contain an immediate value.
```

```{figure} figures/minimalnucleus/immgen.drawio.svg
:align: center
Immediate generator module symbol
```


(sec:nucleus:byteselector)=
## Byteselector

The byteselector module generates the `bsel` signal. This signal is forwarded to the data port interface as
well as modules that handle memory access. It is necessary for the implementation of byte and halfword load/
store commands.

```{list-table} Byteselector input ports
:name: "nucleus:tab:bsel_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - adr_in
    - 2
    - Lowest two bits of the `alu_out` signal which represents the alignment of a calculated address.
*   - funct3_in
    - 3
    - `funct3` block of the current instruction.
```

```{list-table} Byteselector output ports
:name: "nucleus:tab:bsel_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - byteselect_out
    - 4
    - Outgoing byteselect signal.
*   - invalid_out
    - 1
    - Set if adress is misaligned.

```

Because memory accesses always use a full 32-bit word aligned address (lowest two bits are `0`) it is necessary to
generate an additional signal to allow for loading/storing of individual bytes and halfwords. This `bsel` signal
signifies which of the 4 bytes of a 32-bit word at a given address are to be loaded from or stored to. The `bsel` signal
is 4 bits wide and each bit represents a single byte of a 32-bit word.

When an address is calculated by the ALU, only the upper 30 bits are forwarded to the data port interface.
Memory accesses must always be word aligned and thus the lowest two bits of any address must always be `00`.
This is because the main memory is byte-addressable and therefore 32-bit words sit in memory in intervals of `0x4`,
leading to the lowest to bits always being `00`.For the purpose of generating the `bsel` signal, they - along with the
`funct3`  block of the current  instruction - lead to the following `bsel` signal assignments.


```{list-table} Byteselect generation table
:name: "nucleus:tab:bsel_generation_table"
:header-rows: 1

*   - adr_in
    - funct3_in
    - bsel_out
*   - `00`
    - lb/lbu/sb
    - `0001`
*   - `01`
    - lb/lbu/sb
    - `0010`
*   - `10`
    - lb/lbu/sb
    - `0100`
*   - `11`
    - lb/lbu/sb
    - `1000`
*   - `00`
    - lh/lhu/sh
    - `0011`
*   - `10`
    - lh/lhu/sh
    - `1100`
*   - `01`
    - lh/lhu/sh
    - invalid, `invalid_out` is set
*   - `11`
    - lh/lhu/sh
    - invalid, `invalid_out` is set
*   - `00`
    - lw
    - `1111`
*   - `01`
    - lw
    - invalid, `invalid_out` is set
*   - `10`
    - lw
    - invalid, `invalid_out` is set
*   - `11`
    - lw
    - invalid, `invalid_out` is set
```

```{note}
The nucleus currently only supports fully 32-bit aligned loads and stores. It is not possible to load/store data
beyond the 32-bit boundary of any address. Additionally, disjointed loads are also not supported. (for example `bsel` `0101`).
```
### Examples

To further illustrate the interaction between the effective address and the `bsel` signal, consider the following examples:

Assume the following memory layout.

| Address      | Value                 |
| ------------ | --------------------- |
| `0x00000000` | `0x12 0x34 0x56 0x78` |
| `0x00000004` | `0xCA 0xFE 0xBA 0xBE` |

#### Example 1

1. Let the registers `x1` and `x2` contain the value `0`.
2. Let the current instruction be `lb x1, 1(x2)`.
3. This reads as "load the value of the byte at address `0x00000000` offset by `0x1` into `x1`.
4. This leads to an effective target address of `0x00000001`.
5. The targeted byte is therefore `0x34`.
6. The address at the dport interface wil be `0x00000000`. (Lowest two bits always `00`!)
7. The byteselector will evaluate the `funct3` block (here: `0x0`) and the lowest two bits of the effective address
   (here: `0x1`), resulting in a `bsel` value of `0010`.
8. After the memory access transaction is complete, the register `x1` will contain the value `0x00000034`.

#### Example 2

1. Let the register `x1` contain the value `0x0` and register `x2` contain the value `0x00000004`.
2. Let the current instruction be `lh x1, 2(x2)`.
3. This reads as "load the value of the halfword at address `0x00000004` offset by `0x2` into `x1`.
4. This leads to an effective address of `0x00000006`.
5. The targeted halfword is therefore `0xBABE`.
6. The address at the dport interface wil be `0x00000004`. (Lowest two bits always `00`!)
7. The byteselector will evaluate the `funct3` block (here: `0x1`) and the lowest two bits of the effective address
   (here: `0x2`), resulting in a `bsel` value of `1100`.
8. After the memory transaction is complete. the register `x1` will contain the value `0x0000BABE`.

```{note}
The examples above only demonstrate the generation and function of the `bsel` signal. A full memory transaction involves
additional modules.
```

```{figure} figures/minimalnucleus/byteselector.drawio.svg
:align: center

Byteselector module symbol
```


(sec:nucleus:extender)=
## Extender

The extender module prepares **incoming** data requested by the nucleus via load instructions. It ensures
relevant data is in the correct position within the data word and sign extends it, if needed.

```{list-table} Extender input ports
:name: "nucleus:tab:extender_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - data_in
    - 32
    - Incoming data word to be processed
*   - funct3_in
    - 3
    - funct3 block of the current instruction
*   - bsel_in
    - 4
    - Byteselect signal generated by the byteselector.
```

```{list-table} Extender output ports
:name: "nucleus:tab:extender_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - extend_out
    - 32
    - Processed data word
```

When the nucleus requests data from memory via a load instruction, the memory unit will always provide a 32-bit
data word. If the load instruction requests a full 32-bit word (lw instruction) no further steps have to be taken and
the full data word is stored in the target internal register. This changes, when the nucleus requests to load
a byte or halfword via the lb or lh instructions respectively. The memory unit will still provide a 32-bit word
with the relevant data occupying either any of the single 4 bytes within the full word (lb) or two consecutive bytes
(lh). The purpose of this module is to resolve this arrangement and to reorient it to form a new 32-bit word
that represents this data. For this, the extender module uses the `bsel` signal already provided by the
[Byteselector](sec:nucleus:byteselector). The `bsel` signal represents a mask where each of its bits corresponds to a byte
in the incoming data word.

```{list-table} Extender output table
:name: "nucleus:tab:extender_table"
:header-rows: 1

*   - Incoming data
    - `bsel`
    - Output
*   - `0x12345678`
    - `0001`
    - `0x00000078`
*   - `0x12345678`
    - `0010`
    - `0x00000056`
*   - `0x12345678`
    - `0100`
    - `0x00000034`
*   - `0x12345678`
    - `1000`
    - `0x00000012`
*   - `0x12345678`
    - `0011`
    - `0x00005678`
*   - `0x12345678`
    - `1100`
    - `0x00001234`
```

Additionally, the `lb` and `lh` instructions require sign extensions, if need be. For this purpose, the extender module
evaluates the `funct3` block of the current instruction, to determine whether to zero extend (`lbu/lhu`) or to sign
extend. In the `lb/lh` case, the most significant bit of the already previously rearranged data is evaluated.
Should it read `1`, the remaining bits of the whole word are filled with `1`. Should this not apply or the instruction
is `lbu` or `lhu`, the word will simply be zero extended.



```{figure} figures/minimalnucleus/extender.drawio.svg
:align: center
Extender module symbol
```


(sec:nucleus:datahandler)=
## Datahandler

The datahandler module is prepares **outgoing** data to the Dport interface. It ensures data within the outgoing 32-bit
word is in the correct position.

```{list-table} Datahandler input ports
:name: "nucleus:tab:datahandler_input_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - data_in
    - 32
    - Incoming data word to be processed
*   - bsel_in
    - 4
    - Byteselect signal generated by the byteselector
```

```{list-table} Datahandler output ports
:name: "nucleus:tab:datahandler_output_ports"
:header-rows: 1

*   - Port name
    - Signal width
    - Description
*   - data_out
    - 32
    - Processed data word
```

Data stored in any internal register (the regfile) occupies the low bits of the register according to the RISC-V
specification. However, since `sb` and `sh` instructions enable the storing of individual bytes and halfwords to a
specific, byte or halfword aligned position within any given memory address, it is necessary to rearrange outgoing
data accordingly. Essentially, the datahandler module performs the inverse operation of the
[extender module](sec:nucleus:extender). It uses the `bsel` signal to determine which position within the outgoing
data word the incoming data should occupy. Unlike the [extender module](sec:nucleus:extender), it does not perform
any sign or zero extension.

```{list-table} Datahandler output table
:name: "nucleus:tab:datahandler_table"
:header-rows: 1

*   - Incoming data
    - `bsel`
    - Output
*   - `0x00000012`
    - `1000`
    - `0x12000000`
*   - `0x00000012`
    - `0100`
    - `0x00120000`
*   - `0x00000012`
    - `0010`
    - `0x00001200`
*   - `0x00000012`
    - `0001`
    - `0x00000012`
*   - `0x00001234`
    - `1100`
    - `0x12340000`
*   - `0x00001234`
    - `0011`
    - `0x00001234`
*   - `0x12345678`
    - `1111`
    - `0x12345678`
```

```{figure} figures/minimalnucleus/datahandler.drawio.svg
:align: center
Datahandler module symbol
```

(sec:nucleus:controller)=
## Controller

The controller (also known as the control unit) is a finite state machine that sets a number of control signals which control
signal flow and toggle functions of other submodules. It reads status signals from various parts of the nucleus and evaluates
them at specific decision points to facilitate the overall function of executing instructions by setting control signals accordingly.

```{list-table} Controller status signals (inputs)
:name: "nucleus:tab:controller_status_signals"
:header-rows: 1

*   - Signal name
    - Signal width
    - Description
*   - s_opcode
    - 5
    - Opcode of the current instruction
*   - s_funct3
    - 3
    - funct3 block of the current instruction
*   - s_alu_less
    - 1
    - Status signal from the ALU. 1 if A < B, else 0.
*   - s_alu_lessu
    - 1
    - Status signal from the ALU. 1 if A < B (unsigned), else 0.
*   - s_alu_equal
    - 1
    - Status signal from the ALU. 1 if A == B, else 0.
*   - s_dport_ack
    - 1
    - Acknowledge signal from the data port interface.
*   - s_iport_ack
    - 1
    - Acknowledge signal from the instruction port interface.
```


```{list-table} Controller control signals (outputs)
:name: "nucleus:tab:controller_control_signals"
:header-rows: 1

*   - Signal name
    - Description
*   - c_iport_stb
    - Instruction port strobe signal. Begins a transaction.
*   - c_dport_stb
    - Data port strobe signal. Begins a transaction.
*   - c_dport_we
    - Data port write enable signal.
*   - c_reg_ld_en
    - Regfile load enable signal.
*   - c_reg_ldpc
    - If set, regfile input is program counter.
*   - c_reg_ldmem
    - If set, regfile input is a value from memory (via Dport).
*   - c_reg_ldimm
    - If set, regfile input is the current immediate value.
*   - c_reg_ldalu
    - If set, regfile input is the current ALU output.
*   - c_alu_pc
    - If set, ALU operand A is the program counter.
*   - c_alu_imm
    - If set, ALU operand B is the current immediate value.
*   - c_force_add
    - If set, forces the ALU to perform an addition operation.
*   - c_funct7_flag
    - Control signal for the ALU. Decides between ADD/SUB, SRA/SRL operations.
*   - c_pc_inc4
    - If set, the program counter is incremented by `0x4` at the next rising edge.
*   - c_pc_ld_en
    - If set, the program counter takes on the value present at the `pc_in` port at the next rising edge.
*   - c_ir_ld
    - If set, the instruction register takes on the value present at the `ir_in` port at the next rising edge.
```

The controller has 22 internal states which it enters/leaves depending on the current status signals.


```{list-table} Controller logical states
:name: "nucleus:tab:controller_states"
:header-rows: 1

*   - State name
    - Description
*   - `RESET`
    - Initial state. Resets all control signals.
*   - `IPORT_STB`
    - Instruction port strobe is set high.
*   - `AWAIT_IPORT_ACK`
    - Await the instruction port acknowledge signal.
*   - `DECODE`
    - Instruction fetched. Decode the current instruction.
*   - `ALU`
    - Execute ALU instructions.
*   - `ALU_IMM`
    - Execute immediate ALU operations (I-Type).
*   - `ALU_IMM_SHIFT`
    - Execute immediate ALU shift instructions.
*   - `LUI`
    - Execute upper immediate instructions.
*   - `AUIPC`
    - Execute add upper immediate to PC instructions.
*   - `NOP`
    - No operation, PC advances by `0x4`.
*   - `BRANCH`
    - Calculate branch target address and set PC if condition is met.
*  - `DONT_BRANCH`
    - Do not branch, PC advances by `0x4`.
*   - `JAL`
    - Jump and link instruction. Calculate address and set PC.
*   - `JALR`
    - Jump and link register instruction. Calculate address and set PC.
```

The load and store procedures (and related states) are subject to optimization and currently focus on on eliminating
potential edge cases to ensure proper function.

```{list-table} Controller load/store states
:name: "nucleus:tab:controller_load_store_states"
:header-rows: 1

*   - State name
    - Description
*   - `LOAD1`
    - Await data port acknowledge signal is low to ensure no conflict with an ongoing transaction. Then go to `LOAD2` else remain.
*   - `LOAD2`
    - Set data port strobe signal high to begin transaction. Calculate the target address and increment PC. Go to `LOAD3`.
*   - `LOAD3`
    - Await data port acknowledge signal. If high, go to `LOAD4`, else remain.
*   - `LOAD4`
    - Data port acknowledge signal is high. Enable regfile to load data and go to `IPORT_STB`.
*   -
    -
*   - `STORE1`
    - Await data port acknowledge signal is low to ensure no conflict with an ongoing transaction. Then go to `STORE2` else remain.
*   - `STORE2`
    - Set data port strobe signal high to begin transaction. Calculate the target address and increment PC. Set write enable signal high. Go to `STORE3`.
*   - `STORE3`
    - Await data port acknowledge signal. If high, go to `IPORT_STB`, else remain.
```

```{note}
While load and store procedures are essentially the same, the load procedure required an additional state to
avoid erroneous behavior when the target and source register (that holds the target address) are the same.
The specific solution was to delay the  signals that allow the regfile to load a new value (`c_reg_ldmem`, `c_reg_ld_en`)
and enable them only after the bus transaction is complete and when the data present at the regfile input signal is valid.

There is potential for skipping over states when certain conditions are met, this however has not
 been explored with this implementation and is subject to future optimization.
```

See the following diagram to get an overview of the state machine.

```{figure} figures/minimalnucleus/controller.drawio.svg
:align: center
Controller state machine diagram
```

Because the `BRANCH` and `DONT_BRANCH` transition edges would clutter the diagram unnecessarily, their conditions
are detailed in the following table

```{note}
The table is meant to be read such that if the conditions in the "Conditions" column are met, the controller transitions
into the `BRANCH` state, else it transitions into the `DONT_BRANCH` state.
```


```{list-table} Branch conditions
:name: "nucleus:tab:branch_conditions"
:header-rows: 1

*   - Branch Type and opcode
    - Condition
*   - `BEQ`, `0x0`
    - `s_alu_equal == 1`
*   - `BNE`, `0x1`
    - `s_alu_equal == 0`
*   - `BLT`, `0x4`
    - `s_alu_less == 1`
*   - `BGE`, `0x5`
    - `s_alu_less == 0`
*   - `BLTU`, `0x6`
    - `s_alu_lessu == 1`
*   - `BGEU`, `0x7`
    - `s_alu_lessu == 0`
```

## CSR Master

```{figure} figures/minimalnucleus/csr-master.drawio.svg
:align: center
CSR-Master module symbol
```

```{doxygenfunction} SC_MODULE(m_csr_master)
:project: minimalnucleus
```

## CSR

```{figure} figures/minimalnucleus/csr.drawio.svg
:align: center
CSR module symbol
```

```{doxygenfunction} SC_MODULE(m_csr)
:project: minimalnucleus
```