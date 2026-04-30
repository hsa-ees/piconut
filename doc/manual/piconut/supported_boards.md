(chp:supported_boards)=
# Supported Boards
**Author: Johannes Hofmann 2026**

The PicoNut aims to support a variaty of FPGA boards from diffrent vendors.

Currently the following boards are supported:

- ULX3S: <https://radiona.org/ulx3s/>
- OrangeCrab: <https://orangecrab-fpga.github.io/orangecrab-hardware/>

```{note}
The udev rules for the supported boards are in the [Udev Rules](cha:apx:udev) chapter.
```

By default the ULX3S FPGA board is used when building a system. To specify the
target board set the `PICONUT_BOARD` variable accordingly.

The following chapters show how to run the `refdesign` system with the
`hello_piconut` app.

## ULX3S

```console
$ make hello-ulx3s
```

or

```console
$ cd systems/refdesign
$ make TECHS=syn program
```


## OrangeCrab

The OrangeCrab board has no built in serial-to-USB converter so an external module
has to be used.

```{note}
When plugging in the board, press the btn0 button so the FPGA can be programmed
later.
```

```console
$ cd systems/refdesign
$ PICONUT_BOARD=orangecrab make TECHS=syn program
```

The programming tool throws an error after a successful upload (see below).
Despite this message, the design actually uploads perfectly.

```
Download        [=========================] 100%
Download done.
DFU state(7) = dfuMANIFEST, status(0) = No error condition is present
dfu-util: unable to read DFU status after completion (LIBUSB_ERROR_PIPE)
```

