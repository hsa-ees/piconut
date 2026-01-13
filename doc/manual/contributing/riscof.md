# RISC-V Test Framework (RISCOF)
**Author: Niklas Sirch 2025**

Piconut utilizes RISCOF, the RISC-V Architectural Test Framework, to verify
compliance with the RISC-V ISA using community-developed test suites.

## Overview

RISCOF uses a plugin system where each plugin defines how tests are built and
run for a particular Core. For each test, RISCOF runs the same code on both the
target core (e.g., Piconut) and a reference implementation - the
RISC-V Sail Model, a simulator used as the golden reference.

Each executed test produces a signature file, which contains a snapshot of
working memory area. These signature files are then compared between the DUT and
the reference model to detect discrepancies.

The architectural tests themselves are sourced from the
[riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) repository,
maintained by the RISC-V community. These tests are structured to target
specific parts of the ISA.

## Running RISCOF

RISCOF needs the riscof binary and sailc which are inside the provided docker
image with the script. You can run the tests on a simulated piconut with

```console
$ cd sw/verification/riscof
$ make run
```

```{note}
To list all make options use:
```console
$ make
```

## Configuration

### Enabling Extensions

The `piconut_isa.yaml` file controls which tests are executed by configuring
the ISA and MISA fields. To enable or disable specific extensions, edit this
file accordingly. Refer to the table in section
"3.1.1. Machine ISA Register misa" of the privileged RISC-V specification for
the appropriate bit settings.

After making changes, update the badge in the project root `README.md`
to reflect the newly tested extensions.

### Updating Tests

To update the tests update the `RISCV_ARCH_TEST_VERSION` variable in the
`run-riscof.sh` script. This variable specifies the version git tag of the
riscv-arch-test repository to be used.

### Custom Tests

For regression testing or piconut specific coverage, you may want to add custom
tests. RISCOF does not natively support custom tests, as it only sources tests
from the riscv-arch-test repository.

However, within the Piconut project, you can create custom tests by adding them
to the `sw/verification/riscof/custom-tests` directory. These tests are
temporarily copied and committed to the riscv-arch-test repository during the
RISCOF run, and are removed after the run completes.

Guidance on writing tests can be found in the riscv-arch-test repository
[here](https://github.com/riscv-non-isa/riscv-arch-test/blob/dev/spec/TestFormatSpec.adoc).
