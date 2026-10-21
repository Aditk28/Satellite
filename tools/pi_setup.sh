#!/usr/bin/env bash
#
# One-shot setup for the vision Pi. Run on the Pi as the normal user:
#
#     bash pi_setup.sh
#
# Safe to re-run. It:
#  - raises swap to 1024 MB (needed only to build pupil-apriltags)
#  - sets gpu_mem=16 (headless)
#  - enables the UART and disables Bluetooth, so GPIO14/15 get the PL011
#    instead of the mini-UART, whose baud rate drifts with CPU load
#  - turns off the serial console so boot text does not reach the STM32
#  - installs OpenCV/NumPy/pyserial from apt (building them takes hours)
#  - creates ~/vision venv with system site packages and builds
#    pupil-apriltags single-threaded (parallel builds run out of memory)

set -euo pipefail

say() { printf '\n=== %s\n' "$*"; }

# Bookworm moved the firmware config; support both.
if   [ -f /boot/firmware/config.txt ]; then BOOTDIR=/boot/firmware
elif [ -f /boot/config.txt          ]; then BOOTDIR=/boot
else echo "cannot find config.txt, is this a Raspberry Pi?" >&2; exit 1
fi
CONFIG="$BOOTDIR/config.txt"
CMDLINE="$BOOTDIR/cmdline.txt"
say "using $BOOTDIR"

# Swap
say "swap -> 1024 MB (needed to build pupil-apriltags on 512 MB)"
if [ -f /etc/dphys-swapfile ]; then
  sudo dphys-swapfile swapoff || true
  sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
  sudo dphys-swapfile setup
  sudo dphys-swapfile swapon
else
  echo "  dphys-swapfile not present, skipping (build may run out of memory)"
fi

# Firmware config. add_once appends a setting only if it is not already there.
add_once() {
  if ! grep -qE "^\s*$1" "$CONFIG"; then
    echo "$1" | sudo tee -a "$CONFIG" >/dev/null
    echo "  added: $1"
  else
    echo "  already set: $1"
  fi
}
say "firmware config"
add_once "enable_uart=1"
add_once "dtoverlay=disable-bt"
add_once "gpu_mem=16"

# Serial console off
say "serial console off (it would inject boot text into the pose protocol)"
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable --now serial-getty@ttyS0.service   2>/dev/null || true
sudo systemctl disable --now hciuart.service              2>/dev/null || true
# cmdline.txt must stay a single line, so edit in place.
if grep -q 'console=serial0' "$CMDLINE"; then
  sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
  echo "  removed console=serial0 from cmdline.txt"
else
  echo "  cmdline.txt already clean"
fi

# System packages
say "apt packages (prebuilt, compiling OpenCV here would take hours)"
sudo apt-get update -qq
sudo apt-get install -y python3-opencv python3-numpy python3-serial \
                        python3-venv cmake build-essential

# venv and AprilTag detector
say "venv at ~/vision (inherits the apt packages)"
[ -d "$HOME/vision" ] || python3 -m venv --system-site-packages "$HOME/vision"

say "building pupil-apriltags single-threaded (parallel builds run out of memory)"
if CMAKE_BUILD_PARALLEL_LEVEL=1 "$HOME/vision/bin/pip" install pupil-apriltags; then
  DETECTOR="pupil_apriltags"
else
  echo "  Build failed, falling back to OpenCV's aruco AprilTag support."
  echo "  The scripts report which backend they use. Not fatal, but"
  echo "  pupil_apriltags has better range."
  DETECTOR="cv2.aruco (fallback)"
fi

say "done"
cat <<EOF

  detector : $DETECTOR
  venv     : ~/vision/bin/python
  boot cfg : $CONFIG

  Reboot for disable-bt and the console change to take effect:

      sudo reboot

  After reboot, check:

      ls -l /dev/serial0            # must point to ttyAMA0, not ttyS0
      vcgencmd get_throttled        # want 0x0 under load, see below

      for i in 1 2 3 4; do (while :; do :; done) & done; \\
        for i in 1 2 3; do sleep 2; vcgencmd measure_clock arm; \\
        vcgencmd get_throttled; done; kill %1 %2 %3 %4

  Want frequency(48)=1400000000 and throttled=0x0 on every line.
  Anything else means the supply is sagging and vision frame rates on this
  machine will not be representative.

EOF
