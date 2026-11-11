#!/usr/bin/env bash
# Host C++ type/syntax check ONLY. Always run arduino-cli compile on the real toolchain.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
command -v g++ >/dev/null || { echo 'Install g++ to run host syntax checks'; exit 2; }
g++ -std=c++11 -fsyntax-only -x c++ -Itools/mock_arduino -I. sys-TSP05-fixed.ino
for cpp in Button.cpp DcMotor.cpp DistanceSensor.cpp FlexSensor.cpp Potentiometer.cpp SharpIRSensor.cpp StepperMotor.cpp; do
  g++ -std=c++11 -fsyntax-only -Itools/mock_arduino -I. "$cpp"
done
echo 'PASS host Arduino C++ syntax (NOT real AVR compile/upload)'
