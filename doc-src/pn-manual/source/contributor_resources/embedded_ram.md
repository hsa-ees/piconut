
# Embedded RAM Templates
**Author: Lukas Bauer 2024**


This section provides an explanation of the Block RAM templates that
are available in the PicoNut project. The templates are used to
generate the Block RAMs that can be uses in the PicoNut project to
e.g. store program code/data, caches or other data structures.

Firstly, the different types of Block RAMs that are available in the
PicoNut project are explained. Secondly, a description of how to use
one of the templates in a design is given.

The Types of Block RAMs that are available in the PicoNut project are:
* [Single Port Block RAM](sec:bram:single_port_bram)
* [Dual Port Block RAM](sec:bram:dual_port_bram)
* [Dual Port Byte Enable Block RAM](sec:bram:dual_port_byte_enable_port_bram)

All the Block RAMs are synchronous RAMs that are write first.

They consist of two parts:
* The SystemC part that is used to simulate the RAM
* The Verilog part that is used to synthesize the RAM

This is done to ensure that the Block RAMs are not changed during the
ICSC synthesis process. So that the synthesis tools for the boards e.g. Vivado
or yosys can detect the RAMs and synthesize them correctly.

(sec:bram:single_port_bram)=
## Single Port Block RAM

### Description
The single port block RAM is a simple block RAM that has one port for
reading and writing data. The RAM is a synchronous RAM that is write first.

### Ports
```{figure} figures/bram/SinglePortBRAM_ports.drawio.svg
:scale: 80%
:alt: Single Port Block RAM Ports
:align: center

single port block RAM ports
```

```{table} Single Port Block RAM Ports
:widths: 20, 10, 10, 60
|    Port Name    | Direction | Width |                               Description                               |
|:---------------:|:---------:|:-----:|:------------------------------------------------------------------------:|
|       clk       |   input   |   1   |                      clock signal for the block RAM                       |
|       wea       |   input   |   1   | write enable signal for the block RAM **Important:** The `ena` signal must be high to write to the block RAM |
|       ena       |   input   |   1   |     enable signal for the block RAM. Needs to be high for read and write operations    |
|      addra      |   input   |  32   |                 address of the word to read or write                  |
|       dia       |   input   |  32   |                      data input for write operations                     |
|       doa       |  output   |  32   |                     data output for read operations                     |

```


(sec:bram:dual_port_bram)=
## Dual Port Block RAM

### Description
The dual port block RAM is a block RAM that has two ports for reading
and writing data simultaneously. The RAM is a synchronous RAM that is write first.

### Ports
```{figure} figures/bram/DualPortBRAM_ports.drawio.svg
:scale: 80%
:alt: Dual Port Block RAM Ports
:align: center

dual port block RAM ports
```

```{table} Dual Port Block RAM Ports
:widths: 20, 10, 10, 60
|    Port Name    | Direction | Width |                               Description                               |
|:---------------:|:---------:|:-----:|:------------------------------------------------------------------------:|
|     **Port A**      |           |       |                                                                            |
|      clka       |   input   |   1   |                 clock signal for the first port of the block RAM                 |
|       wea       |   input   |   1   | write enable signal for the first port of the block RAM **Important:** The `ena` signal must be high to write to the block RAM |
|       ena       |   input   |   1   |    enable signal for the first port of the block RAM. Needs to be high for read and write operations   |
|      addra      |   input   |  32   |           address of the word to read or write for the first port           |
|       dia       |   input   |  32   |              data input for write operations for the first port             |
|       doa       |  output   |  32   |              data output for read operations for the first port             |
|     **Port B**                                                                                 |
|      clkb       |   input   |   1   |                 clock signal for the second port of the block RAM                 |
|       web       |   input   |   1   | write enable signal for the second port of the block RAM **Important:** The `enb` signal must be high to write to the block RAM |
|       enb       |   input   |   1   |    enable signal for the second port of the block RAM. Needs to be high for read and write operations   |
|      addrb      |   input   |  32   |          address of the word to read or write for the second port          |
|       dib       |   input   |  32   |             data input for write operations for the second port             |
|       dob       |  output   |  32   |             data output for read operations for the second port             |
```

(sec:bram:dual_port_byte_enable_port_bram)=
## Dual Port Byte Enable Block RAM

### Description
The dual port byte enable block RAM is a block RAM that has two ports
for reading and writing data simultaneously. The RAM is a synchronous
RAM that is write first. Additionally, the RAM can write to individual
bytes of a word, reading is always done on a word basis.

### Ports

```{figure} figures/bram/DualPortByteEnableBRAM_ports.drawio.svg
:scale: 80%
:alt: Dual Port Byte Enable Block RAM Ports
:align: center

dual port byte enable block RAM ports
```

```{table} Dual Port Byte Enable Block RAM Ports
:widths: 20, 10, 10, 60
|    Port Name    | Direction | Width |                               Description                               |
|:---------------:|:---------:|:-----:|:------------------------------------------------------------------------:|
|     **Port A**      |           |       |                                                                            |
|      clka       |   input   |   1   |                 clock signal for the first port of the block RAM                 |
|       wea       |   input   |   4   | write enable signal for the first port of the block RAM the write enable signal also works as a byte enable each bit represents a byte to write to **Important:** The `ena` signal must be high to write to the block RAM |
|       ena       |   input   |   1   |    enable signal for the first port of the block RAM. Needs to be high for read and write operations   |
|      addra      |   input   |  32   |           address of the word to read or write for the first port           |
|       dia       |   input   |  32   |              data input for write operations for the first port             |
|       doa       |  output   |  32   |              data output for read operations for the first port             |
|     **Port B**      |           |       |                                                                            |
|      clkb       |   input   |   1   |                 clock signal for the second port of the block RAM                 |
|       web       |   input   |   4   | write enable signal for the second port of the block RAM the write enable signal also works as a byte enable each bit represents a byte to write to **Important:** The `enb` signal must be high to write to the block RAM |
|       enb       |   input   |   1   |    enable signal for the second port of the block RAM. Needs to be high for read and write operations   |
|      addrb      |   input   |  32   |          address of the word to read or write for the second port          |
|       dib       |   input   |  32   |             data input for write operations for the second port             |
|       dob       |  output   |  32   |             data output for read operations for the second port             |
```


## Using a template in a design

An implemented version of a BRAM can be found in the `hw/peripherals/bootram` directory.

To use a template in a design, the following steps need to be taken:

1. Copy the desired template from the `boards/bram_templates` directory
   to the cores, memus or peripherals directory of the design.
   The .h and .cpp files need to be copied into the top level directory
   of the module and the .v file needs to be copied into a `verilog` directory.

2. The blockram needs to be instantiated in the the SystemC Module in which it is used.

3. Add the .cpp file to the `MODULE_SRC` variable in the `Makefile` of the design.

4. Add the custom make targets needed for synthesis of the blockram to the `Makefile`

    ```{code} makefile
    ################ Variables ####################################################
    VERILOG_TEMPLATE_PATH = $(SYSC_MODULE_DIR)/verilog
    VERILOG_OUTPUT_PATH = $(SYNTH_DIR)/sc_build/verilog

    VERILOG_TEMPLATES = $(wildcard $(VERILOG_TEMPLATE_PATH)/*_template.v)
    VERILOG_TARGET_FILES = $(addprefix $(VERILOG_OUTPUT_PATH)/,$(subst _template.v,.v,$(notdir $(VERILOG_TEMPLATES))))

    ################ Special Targets ###############################################


    # extend the synthesis target from sysc-base
    custom-copy-verilog:: $(VERILOG_TARGET_FILES)

    $(VERILOG_OUTPUT_PATH)/%.v: $(VERILOG_TEMPLATE_PATH)/%_template.v $(PNS_CONFIG_MK) $(CONFIG_MK)
        @mkdir --parents $(VERILOG_OUTPUT_PATH)
        @echo "### Generating $(notdir $@) ..."
        @sed \
        -e 's#{RAM_SIZE}#<variable>#g' \
        $< > $@

    ```

    **Note:** The sed command needs to be adjusted to the specific template that is used.
    And the Variables that need to be replaced.

