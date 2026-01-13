# Simulator

To facilitate testing, debugging and  rapid prototyping, the project includes a comprehensive
simulator. It allows the user to provide the system with a gcc compiled program which is then run.
Beyond allowing for standard output print statements, a `.vcd` trace file containing all nucleus
signal states, a core trace and a memory dump are generated alongside. 

To enable rapid integration of new peripherals in software only a simulation only software
peripheral interface class is provided. The peripheral interface class `c_soft_peripheral` allows
software models to interface with hardware models in simulation. This approach unlocks the speed,
resources and features of the C++ language for the verification of hardware modules during the
prototyping process, especially when potentially complex modules are required for the DUT to
function.

## Simulator Usage

Simulators are SystemC test-benches that are compiled by the build system for each
PicoNut system. Currently they are located in the build directory for example under
`/systems/refdesign/pub/hw/bin/refdesign_sim`.

CLI parameters and environment variables can be used to control simulator behavior. But most often a
`make sim` command is sufficient to run the simulator with default parameters.

The most important CLI parameters and environment variables are:

| Option/Parameter            | Description                    |
|-----------------------------|--------------------------------|
| `<path-to-executable>`      | Path to the RISC-V executable to be loaded into memory and executed |
| `SIM_TRACE_LEVEL`           | |
| `-t<n>`, `--tracelevel=<n>` | Set VCD trace level (`0` = no trace file; `>0` = trace file; default = `0`) |
| `SIM_DUMP`                  | |
| `-d`, `--dump`              | Enable memory dump at the beginning and end of simulation |
| `-h`, `--help`              | Show help |

```{note}
Normal test-benches do also accept some arguments for simulators.
```

## Implementation Details

From `base.h` a test-bench gets methods to initialize the common piconut simulator and the
runtime configuration variables.