(sec:sw:crypto)=
# Crypto Library
**Author: Niklas Sirch 2026**

This library provides cryptographic functions optimized for RISC-V architectures.

It should include implementations of various algorithms such as AES, SHA, and RSA,
designed to leverage the capabilities of RISC-V processors for enhanced performance and security.

Currently only AES is implemented.

## Implementation

### AES

The AES implementation in this library supports 128, 192, and 256-bit keys and operates in ECB and
CTR modes.  Basis for the implementation are the Zkne and Zknd RV32 AES instructions and the
reference implementation from the [RISC-V Crypto Project](https://github.com/riscv/riscv-crypto).

On the lower level the AES is implemented naively using the `aes32esmi`/`aes32dsmi` instructions for
the rounds with `aes32esi`/`aes32dsi` for the final round.
The [key schedule](https://en.wikipedia.org/wiki/AES_key_schedule#The_key_schedule) is also generated using
`aes32esi`/`aes32dsi` instructions it also needs the SBox by using the SBox in hardware power traces
can be reduced.

## Interface


```{doxygenfile} rv_crypto.h
:project: crypto_lib
```
