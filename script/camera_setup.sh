#!/bin/bash
# Downward camera settings for mapping at ~20 m — run BEFORE vision_video / vision_geo.
# Works for the Logitech C922 and simpler UVC cameras (e.g. Yahboom H62): controls a camera
# does not have are skipped, gain is set as a % of the camera's own range.
# Values can be overridden for tuning, e.g.:   EXPOSURE=40 GAIN_PCT=40 ./script/camera_setup.sh
# Check sharpness on the ground with:          python3 script/sharpness_test.py
#
# Why: autofocus hunts over water/grass (blur), auto exposure picks long exposures (motion blur),
#      auto white balance drifts the colours (colour = buoy class), dynamic framerate drops to 15 fps.

CAM="${CAM:-/dev/video0}"
EXPOSURE="${EXPOSURE:-150}"        # x 100 us -> 60 = 6 ms (daylight). Darker: 100-150. Shorter = less motion blur
GAIN_PCT="${GAIN_PCT:-6}"        # % of the camera's gain range. Raise when too dark with a short exposure
WB_TEMP="${WB_TEMP:-5500}"        # K, fixed white balance (daylight ~5000-5500)
FOCUS="${FOCUS:-0}"               # C922: 0 = infinity (far). Fixed-focus cameras: skipped
POWER_LINE="${POWER_LINE:-1}"     # 0 = off, 1 = 50 Hz (Indonesia / Singapore), 2 = 60 Hz

CTRLS="$(v4l2-ctl -d "$CAM" --list-ctrls 2>/dev/null)"
[ -z "$CTRLS" ] && { echo "cannot read controls of $CAM"; exit 1; }
echo "camera: $(v4l2-ctl -d "$CAM" --info 2>/dev/null | awk -F': ' '/Card type/ {print $2}') ($CAM)"

# set_ctrl NAME VALUE — skip if the camera has no such control, clamp to its min/max
set_ctrl() {
  local line min max v="$2"
  line="$(echo "$CTRLS" | grep -E "^\s*$1 ")"
  if [ -z "$line" ]; then echo "  - $1: not on this camera, skipped"; return; fi
  min="$(echo "$line" | grep -oP 'min=\K-?[0-9]+')"; max="$(echo "$line" | grep -oP 'max=\K-?[0-9]+')"
  if [ -n "$min" ] && [ -n "$max" ]; then
    [ "$v" -lt "$min" ] && v="$min"; [ "$v" -gt "$max" ] && v="$max"
  fi
  v4l2-ctl -d "$CAM" --set-ctrl="$1=$v" && echo "  $1=$v" || echo "  ! could not set $1=$v"
}

# gain as % of this camera's range (C922: 0-255, Yahboom: 1-8)
GLINE="$(echo "$CTRLS" | grep -E '^\s*gain ')"
GMIN="$(echo "$GLINE" | grep -oP 'min=\K-?[0-9]+')"; GMAX="$(echo "$GLINE" | grep -oP 'max=\K-?[0-9]+')"
GAIN="$(( ${GMIN:-0} + (${GMAX:-0} - ${GMIN:-0}) * GAIN_PCT / 100 ))"

# order matters: switch an AUTO mode off BEFORE setting its manual value
set_ctrl focus_automatic_continuous 0
set_ctrl focus_absolute "$FOCUS"
set_ctrl auto_exposure 3                      # 1 = manual, 3 = aperture priority (auto)
set_ctrl exposure_dynamic_framerate 0         # never drop to 15 fps
set_ctrl exposure_time_absolute "$EXPOSURE"
set_ctrl gain "$GAIN"
set_ctrl white_balance_automatic 0
set_ctrl white_balance_temperature "$WB_TEMP"
set_ctrl power_line_frequency "$POWER_LINE"
set_ctrl backlight_compensation 0
