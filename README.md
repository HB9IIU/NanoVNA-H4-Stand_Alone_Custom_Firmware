# NanoVNA-H4 Custom Firmware for Tuning Magnetic Loop Antenna

Download latest bin from here (https://github.com/HB9IIU/NanoVNA-H4-Stand_Alone_Custom_Firmware/blob/main/build/H4.bin)

It will not brick your nanovna, but use at your own risk



Note to developpers
---------------------------------------------------------
  KEYBOARD SHORTCUTS (Custom)
---------------------------------------------------------

  Ctrl + Alt + B    → Build H4
      • Calls 00_HB9IIUscripts/buildonly.sh
      • Cleans + compiles firmware for STM32F303

  Ctrl + Alt + F    → Flash H4
      • Calls 00_HB9IIUscripts/flashonly.sh
      • Flashes build/H4.bin via dfu-util

  Ctrl + Alt + Q    → Build + Flash H4
      • Calls 00_HB9IIUscripts/build_and_flash_h4.sh
      • Performs full build and then flashes automatically

  Ctrl + Alt + C    → Clean H4
      • Calls 00_HB9IIUscripts/makeclean.sh
      • Removes build artifacts (build/ folder)

---------------------------------------------------------
  LOCATION OF SCRIPTS
---------------------------------------------------------

All automation scripts are stored in:

    00_HB9IIUscripts/

Each script begins by returning to project root:
    cd "$(dirname "$0")/.."

This ensures the Makefile is found correctly.

---------------------------------------------------------
  NOTES
---------------------------------------------------------
Build firmware can be found inside 'build' folder (H4.bin)
