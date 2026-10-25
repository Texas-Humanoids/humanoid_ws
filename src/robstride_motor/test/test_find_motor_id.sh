#!/usr/bin/env bash
# Exercise discovery with fake CAN utilities and the system's default awk.
set -euo pipefail

script="$(cd "$(dirname "$0")/../scripts" && pwd)/find_motor_id.sh"
mocks="$(mktemp -d)"
trap 'rm -rf "$mocks"' EXIT

cat > "$mocks/candump" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "$TEST_FRAMES"
MOCK
cat > "$mocks/cansend" <<'MOCK'
#!/usr/bin/env bash
exit 0
MOCK
cat > "$mocks/sleep" <<'MOCK'
#!/usr/bin/env bash
if [ "$1" != "0.02" ]; then
  /bin/sleep "$1"
fi
MOCK
chmod +x "$mocks/candump" "$mocks/cansend" "$mocks/sleep"
export PATH="$mocks:$PATH"

export TEST_FRAMES="(1.0) can0 000001FE#6E453B4E2071300B
(1.1) can0 00007FFE#1122334455667788
(1.2) can0 000001FE#6E453B4E2071300B
(1.3) can0 000002FE#0000000000000000
(1.4) can0 020003FE#1122334455667788"
actual="$(bash "$script" can0)"
expected="Motors responding on can0:
1 (uuid 6E453B4E2071300B)
127 (uuid 1122334455667788)"
if [ "$actual" != "$expected" ]; then
  printf 'Unexpected discovery result:\n%s\n' "$actual" >&2
  exit 1
fi

export TEST_FRAMES="(1.0) can0 000001FE#0000000000000000"
if bash "$script" can0 > "$mocks/output" 2>&1; then
  echo "Discovery unexpectedly accepted an all-zero UUID" >&2
  exit 1
fi
if ! grep -q 'No motors answered on can0' "$mocks/output"; then
  cat "$mocks/output" >&2
  exit 1
fi
echo "Motor discovery tests passed"
