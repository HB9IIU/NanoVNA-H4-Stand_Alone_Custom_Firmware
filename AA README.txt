==============================================================================
 NanoVNA-H4 – Development Shortcuts (VS Code)
==============================================================================

This file is a quick reminder of the custom keyboard shortcuts I configured in
VS Code for building and flashing the NanoVNA-H4 firmware.

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

• The Git error ("fatal: not a git repository") has been removed by updating
  the VERSION definition in the Makefile.

• Only the STM32F303 platform is used. The F072 directory can be safely deleted.

• IntelliSense has been fully configured for STM32F303 + ChibiOS.

• Scripts run through the MSYS2 bash environment on Windows.

==============================================================================
 End of AA README.txt
==============================================================================


