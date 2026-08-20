#!/usr/bin/env bash
#
# One-shot setup for the vision Pi. Run ON THE PI, as the normal user:
#
#     bash pi_setup.sh
#
# Idempotent -- safe to re-run. Everything it does was worked out the hard way
# during the first Pi bring-up; this exists so a future rebuild is one command
# instead of an afternoon.
#
# WHAT IT DOES AND WHY:
#
#  1. SWAP -> 1024 MB. The pupil-apriltags build OOM-killed cc1 on a 1 GB Pi with
#     parallel jobs; on 512 MB it will not build at all without swap. Needed for
#     the BUILD only -- if the running detector ever swaps, that is a fault to
#     fix, not a condition to tolerate (multi-hundred-ms stalls in the
#     measurement path of a control loop).
#
#  2. gpu_mem=16. Headless, so reserving 64 MB for video wastes 48 MB of a
#     512 MB board.
#
#  3. disable-bt + enable_uart. On a Pi 3 the PL011 is wired to the Bluetooth
#     modem by default and GPIO14/15 get the MINI-UART, whose baud generator is
#     clocked from the VPU core clock and therefore drifts with CPU load. The
#     link would be perfect at idle and start throwing framing errors exactly
#     when AprilTag loads the CPU. See TRANSLATION_DOCKING.md trap T28.
#
#  4. Serial console OFF. It owns the UART and would inject boot text into the
#     pose protocol.
#
#  5. apt packages, not pip, for OpenCV/NumPy/pyserial -- building those from
#     source on this hardware takes hours. The venv uses --system-site-packages
#     so it inherits them.
#
#  6. pupil-apriltags with CMAKE_BUILD_PARALLEL_LEVEL=1, because ninja's default
#     parallelism is what triggers the OOM kill.

set -euo pipefail

say() { printf '\n=== %s\n' "$*"; }

# Bookworm moved the firmware config; support both.
if   [ -f /boot/firmware/config.txt ]; then BOOTDIR=/boot/firmware
elif [ -f /boot/config.txt          ]; then BOOTDIR=/boot
else echo "cannot find config.txt -- is this a Raspberry Pi?" >&2; exit 1
fi
CONFIG="$BOOTDIR/config.txt"
CMDLINE="$BOOTDIR/cmdline.txt"
say "using $BOOTDIR"

# --- 1. swap ---------------------------------------------------------------
say "swap -> 1024 MB (needed to build pupil-apriltags on 512 MB)"
if [ -f /etc/dphys-swapfile ]; then
  sudo dphys-swapfile swapoff || true
  sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
  sudo dphys-swapfile setup
  sudo dphys-swapfile swapon
else
  echo "  dphys-swapfile not present -- skipping (build may OOM)"
fi

# --- 2/3. firmware config --------------------------------------------------
# add_once: append only if the setting is not already there, so re-running does
# not accumulate duplicate lines.
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

# --- 4. serial console off -------------------------------------------------
say "serial console off (it would inject boot text into the pose protocol)"
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable --now serial-getty@ttyS0.service   2>/dev/null || true
sudo systemctl disable --now hciuart.service              2>/dev/null || true
# cmdline.txt is ONE line -- edit in place, never append a newline.
if grep -q 'console=serial0' "$CMDLINE"; then
  sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
  echo "  removed console=serial0 from cmdline.txt"
else
  echo "  cmdline.txt already clean"
fi

# --- 5. system packages ----------------------------------------------------
say "apt packages (prebuilt -- compiling OpenCV here would take hours)"
sudo apt-get update -qq
sudo apt-get install -y python3-opencv python3-numpy python3-serial \
                        python3-venv cmake build-essential

# --- 6. venv + apriltag detector ------------------------------------------
say "venv at ~/vision (inherits the apt packages)"
[ -d "$HOME/vision" ] || python3 -m venv --system-site-packages "$HOME/vision"

say "building pupil-apriltags single-threaded (parallel builds OOM here)"
if CMAKE_BUILD_PARALLEL_LEVEL=1 "$HOME/vision/bin/pip" install pupil-apriltags; then
  DETECTOR="pupil_apriltags"
else
  echo "  BUILD FAILED -- falling back to OpenCV's aruco AprilTag support."
  echo "  The scripts detect this automatically and will report which backend"
  echo "  they used. Not fatal, but pupil_apriltags has better range."
  DETECTOR="cv2.aruco (fallback)"
fi

# --- report ----------------------------------------------------------------
say "done"
cat <<EOF

  detector : $DETECTOR
  venv     : ~/vision/bin/python
  boot cfg : $CONFIG

  REBOOT REQUIRED for disable-bt and the console change to take effect:

      sudo reboot

  After reboot, verify -- BOTH of these matter:

      ls -l /dev/serial0            # MUST say -> ttyAMA0, not ttyS0
      vcgencmd get_throttled        # want 0x0 under load, see below

      for i in 1 2 3 4; do (while :; do :; done) & done; \\
        for i in 1 2 3; do sleep 2; vcgencmd measure_clock arm; \\
        vcgencmd get_throttled; done; kill %1 %2 %3 %4

  Want frequency(48)=1400000000 and throttled=0x0 on every line.
  Anything else means the supply is sagging and NO benchmark taken on this
  machine will mean anything.

EOF
