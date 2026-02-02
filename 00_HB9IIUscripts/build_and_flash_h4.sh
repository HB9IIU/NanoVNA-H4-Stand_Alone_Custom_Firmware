#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."

export TARGET=F303

echo "=== Cleaning build ==="
make clean

echo "=== Building firmware (H4) ==="
make

echo "=== Checking DFU device ==="
dfu-util -l

echo "=== Flashing build/H4.bin to 0x08000000 (alt=0) ==="
dfu-util -a 0 -s 0x08000000:leave -D build/H4.bin
