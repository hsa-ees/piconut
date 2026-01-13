# Code Style Guidelines

**Author: Johannes Hofmann, Niklas Sirch, Gundolf Kiefer 2025**

## Naming Conventions

This section summarizes all naming and coding conventions. The list may be incomplete an be extended as the project grows.

### Overview

```{list-table} Naming Conventions
:header-rows: 1

*   - Category
    - Convention
    - Example(s)
    - Notes
*   - Classes
    - `snake_case`<br>with `c_` prefix
    - `c_peripheral_interface`
    - Use `c` for pure software classes.<br>Use `c_soft_` prefix for SW version of HW modules.
*   - Variables
    - `snake_case`
    - `memory_size`<br>`base_address`
    - Start with lowercase, use snake_case.
*   - Methods/Functions
    - `snake_case`
    - `load_elf`<br>`write_peripheral`
    - Start with lowercase, use snake_case.
*   - Constants (Macros)
    - `UPPER_SNAKE_CASE`
    - `PN_MEMORY_SIZE_128K`<br>`CORE_TRACE_FILE`
    - All caps with underscores.
*   - SystemC modules,<br>VHDL/Verilog entities
    - `snake_case`<br>with `m_` prefix
    - `m_nucleus_ref`<br>`m_uart`
    - Prefix `m` for hardware modules.
*   - SystemC processes
    - `snake_case`<br>with `proc_` prefix
    - `proc_behavior`<br>`proc_cmb_iport`
    - Use `proc_` + `clk`/`cmb` for synthesizable processes
*   - SystemC processes<br>(synthesisable)
    - `snake_case`<br>with prefix<br>`proc_cmb_` or `proc_clk_`
    - `proc_cmb_output`<br>`proc_clk_transition`
    - `cmb` indicates a combinatorial circuit,<br>`clk` indicates a synchronous sequential circuit.
*   - Enumerations (enum)
    - Type: `snake_case`<br>Values: `UPPER_SNAKE_CASE`<br>with type prefix
    - `e_membrana_state`<br>`MEMBRANA_STATE_IDLE`<br>`MEMBRANA_STATE_BUSY`
    - Enum values must have a prefix to avoid name collisions.
*   - Config Options
    - `UPPER_SNAKE_CASE`<br>with `PN_CFG_` prefix
    - `PN_CFG_CPU_CORES`<br>`PN_CFG_UART_ADDR`
    - See section on [configuration parameters](sec:makesys:config) for details.
```


### Additional Rules


1. All symbols that may go into `PN_INSTALLED_DIR` must be prefixed by `PN_` or `pn_`. Legal exceptions are:
	* purely internal symbols (e.g. internal modules of a Nucleus)
	* Makefile symbols that directly affect the build process and nothing else, e.g.: PREFIX, VERBOSE, DEBUG

2. All hardware modules must follow these rules, where *<modname>* is the module name (`PN_MODULE_NAME`):
    * The entity name is `m_<modname>`.
    * Any sub-entities defined in the same module must be prefixed by `m_<modname>_`. For example, the embedded memory inside the *Membrana* variant `m_membrana_hw` is named `m_membrana_emem`.
    * The containing directory is named `<modname>`.


## Formatting

The project uses `clang-format` to format the code. The configuration is in
`.clang-format`. It is highly recommended to follow these guidelines to ensure a
consistent code style.

You can use the VS-Code Extension `xaver.clang-format` or format with these
commands.

```console
$ # For single files use
$ clang-format -i file.cpp
$ # For all project files use
$ git ls-files '*.[ch]' '*.[ch]pp' | xargs clang-format -i
```
