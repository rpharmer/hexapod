#!/usr/bin/env bash
# Locate the Xbox pad evdev node after the wireless adapter is attached via usbipd.
# Run from WSL once the controller is powered on.
set -euo pipefail

if [[ ! -d /dev/input ]]; then
  echo "No /dev/input yet. Attach the dongle (scripts/attach_xbox_wsl.ps1) and power the pad ON." >&2
  exit 1
fi

echo "=== /dev/input ==="
ls -l /dev/input

echo
echo "=== Xbox-like devices ==="

mapfile -t matches < <(
  awk '
    BEGIN { RS = ""; FS = "\n" }
    {
      name = ""; handlers = ""
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^N: Name=/) {
          name = $i
          sub(/^N: Name="/, "", name)
          sub(/"$/, "", name)
        }
        if ($i ~ /^H: Handlers=/) {
          handlers = $i
          sub(/^H: Handlers=/, "", handlers)
        }
      }
      if (name ~ /[Xx]box|[Gg]amepad|[Cc]ontroller/) {
        event = ""
        n = split(handlers, h, /[ \t]+/)
        for (j = 1; j <= n; j++) if (h[j] ~ /^event[0-9]+$/) { event = h[j]; break }
        printf "Name=%s\nHandlers=%s\nEvent=%s\n", name, handlers, event
      }
    }
  ' /proc/bus/input/devices
)

if [[ ${#matches[@]} -eq 0 ]]; then
  echo "No Xbox pad in /proc/bus/input/devices yet." >&2
  echo "If the dongle is up but the pad never appears, pair under Windows first," >&2
  echo "then re-attach with the pad powered OFF (see docs/WSL_XBOX_CONTROLLER.md)." >&2
  exit 2
fi

found=0
name="" handlers="" event=""
for line in "${matches[@]}"; do
  case "$line" in
    Name=*) name=${line#Name=} ;;
    Handlers=*) handlers=${line#Handlers=} ;;
    Event=*)
      event=${line#Event=}
      echo "Name=$name"
      echo "Handlers=$handlers"
      if [[ -n "$event" ]]; then
        echo "Use: --controller-device /dev/input/$event"
        found=1
      fi
      echo "---"
      name="" handlers="" event=""
      ;;
  esac
done

if [[ "$found" -eq 0 ]]; then
  echo "Found Xbox-like device(s) but no event node." >&2
  exit 3
fi
