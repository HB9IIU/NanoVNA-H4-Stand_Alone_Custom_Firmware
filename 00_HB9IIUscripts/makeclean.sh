#!/usr/bin/env bash
set -e

# Go to the project root directory
cd "$(dirname "$0")/.."

export TARGET=F303

echo "=== Cleaning build ==="
make clean
