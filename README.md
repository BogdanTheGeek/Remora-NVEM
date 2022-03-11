# Remora-NVEM

Remora firmware for NVEM Ethernet CNC board. The NVEM board is intended for use with Mach3. This version of the Remora firmware allows the NVEM board to be used with LinuxCNC.

Precompile bin file available in the Firmware directory. Use an ST-Link to flash to the board. Development thread <https://forum.linuxcnc.org/18-computer/44828-remora-ethernet-nvem-cnc-board>

There is also a new LinuxCNC component in the LinuxCNC directory.

The firmware has been developed in STM32CubeIDE. Please note that binary files produced under the Debug profile will crash the board due to the heavy additional debug code included. Use a release profile, or disable / remove Modules from the Base thread when doing testing or development requiring debug capabilities.
