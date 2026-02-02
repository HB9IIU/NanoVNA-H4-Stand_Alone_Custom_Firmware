#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."

export TARGET=F303

BIN_FILE="build/H4.bin"

echo "=== Checking if firmware is built ==="
if [ ! -f "$BIN_FILE" ]; then
  echo "No firmware found at $BIN_FILE"
  echo "=== Running make clean + make ==="
  make clean
  make
else
  echo "Firmware found → skipping build"
fi

echo "=== Checking DFU device ==="
dfu-util -l

echo "=== Flashing $BIN_FILE to 0x08000000 (alt=0) ==="
dfu-util -a 0 -s 0x08000000:leave -D "$BIN_FILE"
