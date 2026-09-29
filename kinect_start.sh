#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
KINECT2PIPE="${BUILD_DIR}/kinect2pipe"
DEVICE="/dev/video10"
export LD_LIBRARY_PATH="/run/media/john/ssd_external/git/libfreenect2/build/lib:${LD_LIBRARY_PATH:-}"

if [[ ! -x "${KINECT2PIPE}" ]]; then
  echo "kinect2pipe not found at ${KINECT2PIPE}, building..."
  make -C "${SCRIPT_DIR}"
fi

cleanup() {
  if [[ -n "${PIPE_PID:-}" ]]; then
    kill "${PIPE_PID}" 2>/dev/null || true
    wait "${PIPE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Check if exclusive_caps is active ('Y')
is_exclusive_caps_active() {
  if [[ -f /sys/module/v4l2loopback/parameters/exclusive_caps ]]; then
    local val
    val="$(cat /sys/module/v4l2loopback/parameters/exclusive_caps 2>/dev/null || true)"
    if [[ "${val}" == *"Y"* ]]; then
      return 0
    fi
  fi
  return 1
}

# Reload v4l2loopback if not loaded, missing device, or loaded with exclusive_caps=0
if [[ ! -e "${DEVICE}" ]] || ! is_exclusive_caps_active; then
  echo "Configuring v4l2loopback (exclusive_caps=1) for Discord / Chromium..."
  if [[ -d /sys/module/v4l2loopback ]]; then
    echo "Unloading existing v4l2loopback module..."
    pkill -x kinect2pipe 2>/dev/null || true
    while pgrep -x kinect2pipe >/dev/null 2>&1; do
      sleep 0.2
    done
    if command -v fuser >/dev/null 2>&1 && fuser "${DEVICE}" >/dev/null 2>&1; then
      echo "Releasing device handles on ${DEVICE}..."
      fuser -k -9 "${DEVICE}" 2>/dev/null || true
      sleep 0.2
    fi
    sudo modprobe -r v4l2loopback
  fi
  echo "Loading v4l2loopback (video_nr=10 exclusive_caps=1)..."
  sudo modprobe v4l2loopback video_nr=10 card_label="KinectCam" exclusive_caps=1 max_buffers=4

  if ! is_exclusive_caps_active; then
    echo "Warning: exclusive_caps is not reporting 'Y' in /sys/module/v4l2loopback/parameters/exclusive_caps."
  fi
fi

if [[ ! -e "${DEVICE}" ]]; then
  echo "Error: ${DEVICE} not found. Is v4l2loopback loaded with video_nr=10?"
  exit 1
fi

echo "Kinect2pipe running on ${DEVICE} at 1080p Full HD (1920x1080)."
echo "You can now connect Discord, OBS, browser, mpv, etc."
echo "Press Ctrl+C or 'q' to stop."

# Pin process to the last hardware core (CPU 14,15) for zero contention with games
exec taskset -c 14,15 "${KINECT2PIPE}" "${DEVICE}"
