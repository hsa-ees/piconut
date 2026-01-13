# Uart Test System

This system is **not** a piconut system but a test system for the UART
Wishbone interface.

- When simulating it executes a few tests on the UART module by sending data back and forth.
- When synthesizing it will repeat received data back to the sender.
  
  Note for picocom: You can test this by enabling and disabling local echo with `Ctrl-A` followed by `C`.
