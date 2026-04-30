(cha:apx:udev)=
# Udev Rules for Supported Boards
**Author: Johannes Hofmann 2025**

(sec:apx:udev:setup)=
## Setup

```{note}
If you are using a docker container. Please remind, that udev rules have 
to be added to the `host system` not to the container.
```

1. Create a New udev Rule:

Create a new file called \<xx-device-name\>.rules and place it in `/etc/udev/rules.d/\<xx-device-name\>.rules`.
Paste the udev rule of your board into the file.

2. Reload udev Rules:
   
```console
$ sudo udevadm control --reload
$ sudo udevadm trigger
```

(sec:apx:udev:ulx3s)=
## ULX3S

```
# /etc/udev/rules.d/80-fpga-ulx3s.rules
# this is for usb-serial tty device
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
  MODE="664", GROUP="dialout"
# this is for ujprog libusb access
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
  GROUP="dialout", MODE="666"
```

(sec:apx:udev:orangecrab)=
## OrangeCrab

```
# /etc/udev/rules.d/50-fpga-orangecrab.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5af0", GROUP="plugdev", MODE="0666"
```
