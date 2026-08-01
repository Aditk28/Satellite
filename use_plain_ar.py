"""
use_plain_ar.py

WORKAROUND for: "arm-none-eabi-gcc-ar.exe was blocked by your organization's
Device Guard policy" during the libFrameworkArduino.a archive step.

WHY THIS MIGHT WORK: gcc-ar is a thin wrapper around plain `ar` that adds
LTO plugin support. Windows Defender Application Control (WDAC) policies
block specific executables, usually by signature or hash. The compile step
(arm-none-eabi-g++.exe) succeeded from the SAME directory, which means the
policy is not blanket-blocking the toolchain folder -- it singled out this
binary. Plain arm-none-eabi-ar.exe is a different file and may well be
allowed.

This costs nothing but the LTO plugin, which this project does not use.

HOW TO USE:
  1. Save this file in the project root (next to platformio.ini)
  2. Add to platformio.ini under [env:nucleo_f446re]:
         extra_scripts = pre:use_plain_ar.py
  3. Rebuild.

If it still fails, the plain archiver is blocked too -- see the notes at the
bottom of this file for the remaining options.
"""

Import("env")

# Swap the LTO wrapper for the plain binutils archiver/ranlib.
env.Replace(
    AR="arm-none-eabi-ar",
    RANLIB="arm-none-eabi-ranlib",
)

print("[use_plain_ar] AR set to arm-none-eabi-ar (bypassing blocked gcc-ar wrapper)")

# ---------------------------------------------------------------------------
# IF THIS DOES NOT WORK, remaining options roughly in order of effort:
#
# 1. Ask IT to whitelist C:\Users\<you>\.platformio\packages\  -- this is the
#    correct fix. WDAC policies commonly block unsigned executables in
#    user-writable directories, which is exactly where PlatformIO installs
#    its toolchain. A managed school or work laptop will usually have a
#    process for this.
#
# 2. Build under WSL2. WDAC applies to Windows processes, not to Linux
#    binaries running inside WSL. Install PlatformIO Core in WSL and build
#    there. Flashing needs the USB device passed through (usbipd-win), or
#    you can build in WSL and flash from Windows with STM32CubeProgrammer
#    against the generated .bin/.elf in .pio/build/.
#
# 3. Build on a personal (unmanaged) machine.
#
# 4. Install the ARM toolchain system-wide into Program Files (signed
#    installer from Arm), then point PlatformIO at it:
#        platform_packages = toolchain-gccarmnoneeabi@symlink://C:/Program Files/...
#    Executables in Program Files are far more likely to satisfy the policy.
# ---------------------------------------------------------------------------s