bin2dfu
======
This tool is for users of the Dx1bootloader to generate a DFU image directly from a already reloacted .bin file.

This DFU image output includes a CRC32 calculation that is stored inside the user application; this is checked by the bootloader to verify the user application's integrity.

## Sample Usage
```
bin2dfu myapp.elf myapp.dfu 0x1234 0x4567
```

## Theory of Operation
The Cortex-M0 vector table has nine unused 32-bit entries marked 'Reserved' that only serve as wasted flash space.  By using two of these entries to store the user application length and its CRC32, the bin2dfu utility can communicate this information to the bootloader without consuming additional space.

By storing the user application length, the bootloader will only compute the CRC32 over a prescribed portion of the flash.  This frees the user application, if it wishes, to store and re-write data in upper portions of the flash without impacting the CRC32 protection.

## Compiling
Just put it into `gcc`
```
gcc -o bin2dfu bin2dfu.c
```
A Windows binary can be build with `i686-w64-mingw32-gcc-win32`
```
i686-w64-mingw32-gcc-win32-o bin2dfu.exe bin2dfu.c
```
