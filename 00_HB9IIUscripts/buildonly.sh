#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."

export TARGET=F303

echo "=== Cleaning build ==="
make clean

echo "=== Building firmware (H4) ==="
make
