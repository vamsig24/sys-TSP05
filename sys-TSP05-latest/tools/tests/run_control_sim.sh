#!/usr/bin/env bash
# Deterministic host simulation (not a substitute for AVR hardware testing).
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
g++ -std=c++11 -Wall -Wextra -I"$root/tools/tests/sim_arduino" -I"$root" \
    "$root/tools/tests/sim_arduino/test_motor_and_sharp.cpp" \
    "$root/DcMotor.cpp" "$root/SharpIRSensor.cpp" -o "$tmp/verify"
"$tmp/verify"
g++ -std=c++11 -Wall -Wextra -I"$root" \
    "$root/tools/tests/sim_arduino/test_sensor_mapping.cpp" \
    -o "$tmp/verify_mapping"
"$tmp/verify_mapping"
