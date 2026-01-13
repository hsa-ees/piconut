(sec:idef:csoft)=

# Software-Simulated Hardware

**Author: Marco Milenkovic 2024**

## Overview

This documentation provides an overview and usage guide for the software library designed to manage peripherals and handle ELF file parsing for embedded system simulations. The library is structured to support easy integration and extension, with components dedicated to different aspects of system emulation, such as memory management, peripheral interfacing, and file handling.

## Features

- **Peripheral Management**: Manages a variety of simulated peripherals.
- **Memory Simulation**: Simulates different memory scenarios and behaviors.
- **File Parsing**: Supports reading and interpreting ELF files.

## Getting Started

### Prerequisites

- A modern C++ compiler supporting C++17 or later.
- Basic understanding of computer architecture and peripheral management.
- Familiarity with the PicoNut project and its components.

---

## Library Components

The library is divided into several key components, each handling a specific aspect of system simulation. Detailed documentation for each component is provided in the following sections.

---

## `c_soft_peripheral`

The `c_soft_peripheral` class is the base class for all peripherals in the software library. It provides a common interface for peripheral operations and is designed to be extended for specific peripheral implementations.

### Usage Example

```cpp
class my_peripheral : public c_soft_peripheral {
public:
    my_peripheral(uint32_t base_address) : c_soft_peripheral(base_address) {}

    uint8_t read(uint32_t address) override {
        // Read from the peripheral
    }

    void write(uint32_t address, uint8_t data) override {
        // Write to the peripheral
    }
};
```

Refer to existing peripherals such as `c_soft_memory` and `c_soft_uart` for further examples.

---

## `c_soft_memory`

The `c_soft_memory` class simulates memory within the software library. It supports basic read and write operations to access memory locations.

### Usage Example

```cpp
c_soft_memory memory(1024, 0x40000000);
memory.write(0x40000000, 0x12);
uint8_t data = memory.read(0x40000000);
```

The constructor takes the memory size and base address as parameters. It also provides methods to dump the memory contents to a file.

---

## `c_soft_uart`

The `c_soft_uart` class simulates a UART interface. It allows data transmission and reception over a simulated UART connection and supports setting the baud rate, checking for data availability, and reading/writing data.

### Usage Example

```cpp
c_soft_uart uart(0x40001000);
uart.set_baud_rate(9600);
uart.write(0x40001000, 'A');
uint8_t data = uart.read(0x40001000);
```

---

## Peripheral Interface

The `c_soft_peripheral_container` class manages the peripherals in a simulation-only environment. It allows adding, interacting with, and managing peripherals, typically used within a `m_membrana_soft` or similar module.

### Class: c_soft_peripheral_container

#### Public Methods

### `void add_peripheral(uint64_t base_address, std::unique_ptr<c_soft_peripheral> peripheral)`

**Description**:
Adds a new peripheral to the system at the specified base address.

**Parameters**:

- `base_address` (_uint64_t_): The base address of the peripheral.
- `peripheral` (_std::unique_ptr<c_soft_peripheral>_): A unique pointer to the peripheral.

**Usage Example**:

```cpp
auto my_peripheral = std::make_unique<my_peripheral>();
peripheral_interface.add_peripheral(0x1000, std::move(my_peripheral));
```

---

### `c_soft_peripheral* find_peripheral(uint64_t address)`

**Description**:
Finds a peripheral by its memory address.

**Parameters**:

- `address` (_uint64_t_): The memory address to search for.

**Returns**:

- A pointer to the peripheral if found, otherwise `nullptr`.

**Usage Example**:

```cpp
c_soft_peripheral* peripheral = peripheral_interface.find_peripheral(0x1000);
if (peripheral != nullptr) {
    // Peripheral found
}
```

---

### `void write_peripheral(uint64_t address, uint32_t data, uint8_t bsel)`

**Description**:
Writes data to a peripheral at the specified address, using the byte-select value.

**Parameters**:

- `address` (_uint64_t_): The peripheral address.
- `data` (_uint32_t_): Data to write.
- `bsel` (_uint8_t_): Byte-select value.

**Usage Example**:

```cpp
peripheral_interface.write_peripheral(0x1000, 0xFF00FF00, 0x0F);
```

---

### `uint32_t read_peripheral(uint64_t address, uint8_t bsel)`

**Description**:
Reads data from a peripheral at the specified address, using the byte-select value.

**Parameters**:

- `address` (_uint64_t_): The address of the peripheral.
- `bsel` (_uint8_t_): Byte-select value.

**Returns**:

- The data read from the peripheral.

**Usage Example**:

```cpp
uint32_t data = peripheral_interface.read_peripheral(0x1000, 0x01);
```

---

## ELF Reader

The `elf_reader` class provides methods for reading and parsing ELF files in the software library. It reads the contents of an ELF file, extracts important information such as entry points and program headers, and supports symbol parsing for debugging purposes.

### Example Usage

```cpp
elf_reader elf("my_program.rv32");
elf.initialize_from_file();
char* memory = elf.get_memory();
```

The ELF reader loads ELF files into memory and provides access to symbols for debugging and memory dumping.

---

## Troubleshooting

Common issues and solutions:

- **Peripheral not found**: Ensure the peripheral's base address is correctly added to the system.

  - Is also intended to show up whenever a new peripheral is added to declare that there is not already a existing one at that address range.

- **Invalid memory access**: Check that the memory address falls within the bounds of the initialized memory.
