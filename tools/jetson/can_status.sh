#!/usr/bin/env bash
# Every CAN adapter on the Jetson: its name, which physical adapter it is, its bus state, and the
# motor IDs that answer on it. Brings each USB adapter up at 1 Mbit/s first (needs sudo) unless
# --no-up. Read-only on the bus: only the "Get Device ID" request of find_motor_id.sh.
#
#   tools/jetson/can_status.sh            # bring up and scan every adapter
#   tools/jetson/can_status.sh --no-up    # don't touch interfaces that are down
#
# Stop robot.launch.py and other motor tools first: a scan while nodes run is harmless but slow.
set -uo pipefail

bring_up=1
[[ "${1:-}" == "--no-up" ]] && bring_up=0
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scan="$here/../../src/robstride_motor/scripts/find_motor_id.sh"
declare -A expect=([can_left]="11 12 13 14 15" [can_right]="21 22 23 24 25")

ifaces=$(ip -br link show type can | awk '{print $1}')
if [[ -z "$ifaces" ]]; then
  echo "No CAN interfaces. Is a USB-CAN adapter plugged in? (sudo dmesg -T | tail)"
  exit 1
fi

usb_adapters=0
for i in $ifaces; do
  driver=$(basename "$(readlink -f "/sys/class/net/$i/device/driver")" 2>/dev/null)
  if [[ "$driver" != "gs_usb" ]]; then
    printf '%-10s built-in (%s), not used by the robot\n\n' "$i" "${driver:-?}"
    continue
  fi
  usb_adapters=$((usb_adapters + 1))
  usbdev=$(readlink -f "/sys/class/net/$i/device/..")
  serial=$(cat "$usbdev/serial" 2>/dev/null || echo "?")
  port=$(basename "$usbdev")
  if [[ $bring_up == 1 ]]; then
    sudo ip link set "$i" down 2>/dev/null
    sudo ip link set "$i" up type can bitrate 1000000 || echo "  (could not bring $i up)"
  fi
  state=$(ip -details link show "$i" | sed -n 's/.*can state \([A-Z-]*\).*/\1/p')
  updown=$(ip -br link show "$i" | awk '{print $2}')
  ids=""
  if [[ "$updown" == "UP" ]]; then
    ids=$("$scan" "$i" 2>/dev/null | awk '/^[0-9]+ \(uuid/{printf "%s ", $1}')
  fi
  printf '%-10s adapter %s  USB port %s  %s %s\n' "$i" "$serial" "$port" "$updown" "${state:-}"
  if [[ "$updown" != "UP" ]]; then
    echo "           down: sudo ip link set $i up type can bitrate 1000000"
  elif [[ -n "$ids" ]]; then
    echo "           motors: $ids"
  else
    echo "           motors: NONE answer (power, CAN_H/CAN_L/GND wiring, termination)"
  fi
  want="${expect[$i]:-}"
  if [[ -n "$want" && "$updown" == "UP" ]]; then
    missing=$(comm -23 <(tr ' ' '\n' <<<"$want" | sort) <(tr ' ' '\n' <<<"$ids" | sed '/^$/d' | sort) | tr '\n' ' ')
    extra=$(comm -13 <(tr ' ' '\n' <<<"$want" | sort) <(tr ' ' '\n' <<<"$ids" | sed '/^$/d' | sort) | tr '\n' ' ')
    if [[ -z "$missing$extra" ]]; then
      echo "           OK: all of $want"
    else
      [[ -n "$missing" ]] && echo "           MISSING (expected on $i): $missing"
      [[ -n "$extra" ]] && echo "           UNEXPECTED on $i: $extra (other leg's bus, or a new motor at a factory ID)"
    fi
  elif [[ "$i" == can_usb ]]; then
    echo "           (unnamed adapter: not one of the two in tools/jetson/05-can-*.link)"
  fi
  echo
done
[[ $usb_adapters -lt 2 ]] && echo "Only $usb_adapters USB-CAN adapter(s) found; the robot uses two (can_left, can_right)."
exit 0
