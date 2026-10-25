#!/usr/bin/env bash
# Discover RobStride motor IDs on a CAN bus. Read-only: sends only the
# "Get Device ID" request (comm type 0) to every ID 0-127. It never enables
# or configures anything. Run it, then pass the ID you want explicitly as
# `-p motor_id:=<id>` to the node.
#
# Usage: find_motor_id.sh [can_interface]   (default: can0)
set -euo pipefail

iface="${1:-can0}"
log="$(mktemp)"
trap 'kill "${dump_pid:-}" 2>/dev/null || true; rm -f "$log"' EXIT

candump -L "$iface" > "$log" &
dump_pid=$!
sleep 0.2

for id in $(seq 0 127); do
  printf -v hexid '%08X' $((0xFF00 | id))
  cansend "$iface" "${hexid}#0000000000000000"
  sleep 0.02
done
sleep 0.3

# Replies look like `(ts) can0 000001FE#6E453B4E20713 00B`: comm type 0,
# responding motor ID in bits 8-15, destination 0xFE, non-zero UUID payload.
found=$(awk '{
  split($3, f, "#")
  if (length(f[1]) == 8 && substr(f[1], 1, 2) == "00" && substr(f[1], 7, 2) == "FE" \
      && f[2] !~ /^0+$/)
    print strtonum("0x" substr(f[1], 5, 2)) " (uuid " f[2] ")"
}' "$log" | sort -un)

if [ -z "$found" ]; then
  echo "No motors answered on $iface. Check power, ground, bitrate (1 Mbps)." >&2
  exit 1
fi
echo "Motors responding on $iface:"
echo "$found"
