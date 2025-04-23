# Debugging
**Author: Johannes Hofmann 2025**

For developing larger and more complex software, a debugger is an essential tool.
It allows developers to analyze and control program execution with features such as:
 - Starting and stopping execution
 - Setting breakpoints
 - Reading and writing memory
 - Reading and writing registers

One widely used debugger is the `GNU Debugger (GDB)`. When debugging embedded
hardware, `OpenOCD` is required to facilitate communication between the hardware
and GDB.

The debug hardware follows the `RISC-V External Debug Support` specification, ensuring compatibility with GDB for seamless debugging.

In figure Overview of the debug hardware you can see the debug infrastructure
architecture.

```{figure} ./figures/debugger/Overview.drawio.svg
:name: debugger_overview
:width: 100%
:align: center
Overview of the debug hardware
```

## Soft Debugger

### Remote BitBang

```{doxygengroup} c_remote_bitbang
:project: c_remote_bitbang
```

#### Functions

```{doxygenfunction} c_remote_bitbang
:project: c_remote_bitbang
```

```{doxygenfunction} ~c_remote_bitbang
:project: c_remote_bitbang
```

```{doxygenfunction} process
:project: c_remote_bitbang
```

```{doxygenfunction} connected
:project: c_remote_bitbang
```

### Debug Module Interface (DMI)

The `DMI` is the interface between the `Debug Transport Module (DTM)` and the
`Debug Module (DM)`. For the software debugger its three functions that the `DM` (DMI slave)
has to implement which are called by the `DTM` (DMI master):
- `void dmi_write(uint8_t adr, uint32_t data)`: Called when a register write operation is performed.
- `uint32_t dmi_read(uint8_t data)`: Called when a register read operation is performed.
- `void dmi_reset()`: Called when a reset operation is performed.

### Debug Transport Module (DTM)

```{figure} ./figures/debugger/jtag_tap_controller_state_diagram.svg
:name: debugger_dtm_tap
:width: 100%
:align: center
Overview TAP-Machine.
```

```{doxygengroup} c_dtm
:project: c_soft_debugger
```

#### Functions

```{doxygenfunction} c_dtm
:project: c_soft_debugger
```

```{doxygenfunction} ~c_dtm
:project: c_soft_debugger
```

```{doxygenfunction} jtag_set_input_pins
:project: c_soft_debugger
```

```{doxygenfunction} jtag_get_output_pin
:project: c_soft_debugger
```

```{doxygenfunction} get_instruction_reg_width
:project: c_soft_debugger
```

```{doxygenfunction} get_data_reg_width
:project: c_soft_debugger
```

### Debug Module (DM)

```{doxygengroup} c_soft_dm
:project: c_soft_debugger
```

#### Debug registers

```{doxygengroup} c_debug_regs
:project: c_soft_debugger
```

```{doxygengroup} c_debug_reg_command
:project: c_soft_debugger
```

```{doxygengroup} c_debug_reg_abstractcs
:project: c_soft_debugger
```

#### Functions

```{doxygenfunction} c_soft_dm
:project: c_soft_debugger
```


```{doxygenfunction} ~c_soft_dm
:project: c_soft_debugger
```

```{doxygenfunction} get_info
:project: c_soft_debugger
```

```{doxygenfunction} is_addressed
:project: c_soft_debugger
```

```{doxygenfunction} read32
:project: c_soft_debugger
```

```{doxygenfunction} write32
:project: c_soft_debugger
```

```{doxygenfunction} dmi_write
:project: c_soft_debugger
```

```{doxygenfunction} dmi_read
:project: c_soft_debugger
```


```{doxygenfunction} dmi_reset
:project: c_soft_debugger
```

### Soft Debugger Wrapper

This module serves as a wrapper for all c_soft_debugger modules.
It instantiates a `c_remote_bitbang`, a `c_dtm` and a `c_soft_dm`.
The `c_soft_peripheral` interface is forwarded to the `c_soft_dm`.

Note: The module is not synthesizable and is only used for simulation purposes.

### Usage

An example refdesign can be found at `systems/refdesign_c_soft_debuggger`.
The example application for the `refdesign_c_soft_debugger` is the `animation`
application.

1. Compile the application and start with a refdesign that instantiates a debugger. For example the refdesign_c_soft_debugger

```console
$ cd sw/applications/animation
$ PN_SYSTEM=refdesign_c_soft_debugger make sim
```

To start a debug session you can either use `GDB` in terminal mode or use the
VSCode interface.

#### Terminal

To attach GDB to the current runing application type in to same application folder:
```console
$ make attach-gdb
```

Alternatively, you can start OpenOCD and GDB seperate in two terminals.

1. Start OpenOCD with simulation configuration file.
The configuration for OpenOCD is found at `tools/etc/openocd-sim.cfg`.
From the animation directory invoke:

```console
$ make start-openocd-sim
```

2. Start GDB with configuration file.
The configuration for GDB is found at `tools/etc/gdb_start`.
From the animation directory invoke:

```console
$ make start-gdb
```

This attaches `GDB` to the running program. If you want to debug the program from the first assembler command you need to restart the application. In the `GDB` console type:

```bash
b _start
j _start
```

#### VSCode

Add the necessary configurations to the VSCodes configuration files.

The debugger configuration in the `launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(GDB) PicoNut Software Debugger",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/sw/application/animation/animation.elf",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "preLaunchTask": "start-openocd-sim",
            "postDebugTask": "stop-openocd",
            "serverStarted": "Listening on port 3333",
            "environment": [],
            "externalConsole": true,
            "MIMode": "gdb",
            "miDebuggerPath": "riscv64-unknown-elf-gdb",
            "miDebuggerServerAddress": "localhost:3333",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set hex format for gdb",
                    "text": "set output-radix 16"
                },
                {
                    "description": "Set available registers",
                    "text": "set tdesc filename ${workspaceFolder}/tools/etc/gdb_piconut.xml"
                }
            ]
        }
    ]
}
```

The tasks to start or stop OpenOCD when a debug session is started or stopped
in the `tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "start-openocd-sim",
            "type": "shell",
            "command": "openocd -f ${workspaceFolder}/tools/etc/openocd-sim.cfg",
            "problemMatcher": [],
            "isBackground": true,
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "presentation": {
                "echo": true,
                "reveal": "silent",
                "focus": false,
                "panel": "shared"
            }
        },
        {
            "label": "stop-openocd",
            "type": "shell",
            "command": "pkill -f openocd",
            "problemMatcher": [],
            "presentation": {
                "reveal": "never",
                "panel": "shared"
            }
        }
    ]
}
```

To start the debug session go to the `Run and Debug` menu an select the
`(GDB) PicoNut Software Debugger`. After clicking run ignore the warning popup
saying that OpenOCD hasn't terminated yet and click `Debug Anyway`.

## Debug Handler

```{doxygengroup} debug_handler
:project: c_soft_debugger
```

The `Debug Handler` program behaves like described in the diagram below.


```{figure} ./figures/debugger/debug_handler_flow_diagram.drawio.svg
:name: debug_handler_flow_diagram
:width: 30%
:align: center
Debug Handler program flow.
```

## Hardware Debugger

TODO
