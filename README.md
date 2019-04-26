USB DFU Bootloader for SAMD11 / SAMD21 / SAME54
===============================================

This is just another bootloader for Atmel SAMx Devices. It is based upon  [SAMDx1-USB-DFU-Bootloader](https://github.com/majbthrd/SAMDx1-USB-DFU-Bootloader). 
__This version adds more features and thus increases the size beyond the original 1k requirement.__
Some of the new features were inspired by the [DAFU](https://github.com/opendime/DAFU/) Bootloader.

This USB bootloader is ~2kBytes and implements the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/) and [webdfu](https://github.com/devanlai/webdfu).

## New/changed Features over the original SAMDx1-USB-DFU-Bootloader
- changed into a AtmelStudio Project
- Add support for Atmel SAMD5x/E5x devices
- remove double-tap-magic
- add full blown USB descriptors including WinUSB descriptors (no .inf file needed for recent Windows versions)
- blink some LEDs so the user can see that the device is in DFU-mode
- add simple i2c and spi routines
- version information is added during compile and linked to a fixed address (this can be read by the application)
- new/changed bootloader entry condition
  - removed crc check via DSU
  - Stack- and Instruction-Pointer verification
  - Use watchdog as bootloader-entry-condition
  - Detection of a USB-Dongle (short D+ and C- with 100R) to force bootloader entry
- powershell build-script

### Unfinished features
- dual-stage bootloading via external SPI Flash

## Usage

Downloading can be accomplished with any software that supports DFU, which includes [dfu-util](http://dfu-util.sourceforge.net/) and [webdfu](https://github.com/devanlai/webdfu).

With [dfu-util](http://dfu-util.sourceforge.net/), downloading is like so:

```
dfu-util -D application.dfu
```

## Requirements for compiling

This repository includes an AtmelStudio project file. It does not need AtmelStart, the DeviceSupportPackage is sufficient.
Since AtmelStudio uses a regular arm-gcc any other arm-gcc toolchain should be able to build this code as well.
The original project used a Clang based ARM toolchain, but Clang has not been tested for this version.

## Programming targets with bootloader

Since this is an AtmelStudio project now the easies way is to use the build-in device programming.
[openocd](http://openocd.org/) also works just as well (This [openocd script](https://bitbucket.org/snippets/carstenpresser/ye59b8) was used for the bootloader). Newer versions of openocd also include support for Atmel SAME devices.
