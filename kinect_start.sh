#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
KINECT2PIPE="${BUILD_DIR}/kinect2pipe"
DEVICE="/dev/video10"
OBS_DEVICE="/dev/video11"
for libpath in "${SCRIPT_DIR}/lib" "${SCRIPT_DIR}/../libfreenect2/build/lib" "/home/john/git/libfreenect2/build/lib" "/run/media/john/ssd_external/git/libfreenect2/build/lib"; do
  if [[ -d "${libpath}" ]]; then
    export LD_LIBRARY_PATH="${libpath}:${LD_LIBRARY_PATH:-}"
    break
  fi
done

ENABLE_OBS=false
for arg in "$@"; do
  case "${arg}" in
    --obs|-o)
      ENABLE_OBS=true
      ;;
    -h|--help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --obs, -o     Initialize secondary loopback device (/dev/video11) for OBS Virtual Camera"
      echo "  -h, --help    Show this help message"
      exit 0
      ;;
  esac
done

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

# Determine if module reload is needed
needs_reload=false
if [[ ! -e "${DEVICE}" ]] || ! is_exclusive_caps_active; then
  needs_reload=true
elif [[ "${ENABLE_OBS}" == "true" && ! -e "${OBS_DEVICE}" ]]; then
  needs_reload=true
fi

if [[ "${needs_reload}" == "true" ]]; then
  if [[ -d /sys/module/v4l2loopback ]]; then
    echo "Unloading existing v4l2loopback module..."
    pkill -x kinect2pipe 2>/dev/null || true
    while pgrep -x kinect2pipe >/dev/null 2>&1; do
      sleep 0.2
    done
    if command -v fuser >/dev/null 2>&1; then
      echo "Releasing device handles on loopback devices..."
      fuser -k -9 "${DEVICE}" 2>/dev/null || true
      fuser -k -9 "${OBS_DEVICE}" 2>/dev/null || true
      sleep 0.2
    fi
    sudo modprobe -r v4l2loopback
  fi

  if [[ "${ENABLE_OBS}" == "true" ]]; then
    echo "Loading v4l2loopback with 2 devices (KinectCam on ${DEVICE}, OBS-Virtual-Cam on ${OBS_DEVICE})..."
    sudo modprobe v4l2loopback devices=2 video_nr=10,11 card_label="KinectCam","OBS-Virtual-Cam" exclusive_caps=1,1 max_buffers=4
  else
    echo "Loading v4l2loopback (video_nr=10 exclusive_caps=1)..."
    sudo modprobe v4l2loopback video_nr=10 card_label="KinectCam" exclusive_caps=1 max_buffers=4
  fi

  if ! is_exclusive_caps_active; then
    echo "Warning: exclusive_caps is not reporting 'Y' in /sys/module/v4l2loopback/parameters/exclusive_caps."
  fi
fi

if [[ ! -e "${DEVICE}" ]]; then
  echo "Error: ${DEVICE} not found. Is v4l2loopback loaded with video_nr=10?"
  exit 1
fi

if [[ "${ENABLE_OBS}" == "true" && ! -e "${OBS_DEVICE}" ]]; then
  echo "Error: ${OBS_DEVICE} not found for OBS Virtual Camera."
  exit 1
fi

echo "Kinect2pipe running on ${DEVICE} at 1080p Full HD (1920x1080)."
if [[ "${ENABLE_OBS}" == "true" ]]; then
  echo "OBS Virtual Camera loopback ready on ${OBS_DEVICE} (OBS-Virtual-Cam)."
  echo "Workflow:"
  echo "  1. In OBS: Add Video Capture Device (V4L2) -> ${DEVICE} (KinectCam)"
  echo "  2. In OBS: Click 'Start Virtual Camera'"
  echo "  3. In Discord/Browser: Select 'OBS-Virtual-Cam' (${OBS_DEVICE})"
else
  echo "Tip: Run with '--obs' if you want a dedicated virtual camera node for OBS + Discord."
fi
echo "Press Ctrl+C or 'q' to stop."

# Pin process to the last hardware core (CPU 14,15) for zero contention with games
exec taskset -c 14,15 "${KINECT2PIPE}" "${DEVICE}"
