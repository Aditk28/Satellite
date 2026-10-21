"""
PlatformIO extra script: archive with plain arm-none-eabi-ar instead of gcc-ar.

This machine's Device Guard (WDAC) policy blocks arm-none-eabi-gcc-ar.exe, which
breaks the libFrameworkArduino.a step. Plain ar is allowed and only lacks the LTO
plugin, which this project does not use. Loaded via extra_scripts in platformio.ini.
"""

Import("env")

env.Replace(
    AR="arm-none-eabi-ar",
    RANLIB="arm-none-eabi-ranlib",
)

print("[use_plain_ar] AR set to arm-none-eabi-ar (gcc-ar is blocked)")

# If plain ar is ever blocked too: get the ~/.platformio/packages folder allowed,
# or build under WSL2, where WDAC does not apply.
