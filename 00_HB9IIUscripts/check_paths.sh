#!/usr/bin/env bash

echo "Checking IntelliSense Include Paths..."

paths=(
    "ChibiOS/os/common"
    "ChibiOS/os/common/ext/CMSIS"
    "ChibiOS/os/common/ext/CMSIS/ST/STM32F3xx"
    "ChibiOS/os/hal/include"
    "ChibiOS/os/hal/src"
    "ChibiOS/os/hal/ports/STM32"
    "ChibiOS/os/hal/ports/common/ARMCMx"
    "ChibiOS/os/common/startup/ARMCMx"
)

ok=true

for p in "${paths[@]}"; do
    if [ -d "$p" ]; then
        echo "✔ FOUND: $p"
    else
        echo "❌ MISSING: $p"
        ok=false
    fi
done

if $ok; then
    echo "All paths OK ✔ (IntelliSense config is valid)."
else
    echo "Some paths were not found ❗"
    echo "Please verify folder structure."
fi
