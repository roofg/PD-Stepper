#!/bin/bash
# Run native planner and PD controller tests using clang++.
# PlatformIO's native platform requires GCC, which isn't available on this Windows
# machine. This script compiles and runs tests directly with clang++ (MSVC target).
#
# Prerequisites:
#   - clang++ on PATH (LLVM installed)
#   - Unity test framework installed via: pio pkg install -e native_test
#
# Usage: bash run_tests.sh

set -e
cd "$(dirname "$0")"

UNITY_DIR=".pio/libdeps/native_test/Unity/src"
OUT_DIR=".pio/build/native_manual"
FLAGS="-std=c++17 -DUNITY_INCLUDE_FLOAT -I src -I $UNITY_DIR"

mkdir -p "$OUT_DIR"

echo "=== Building tests ==="
clang++ $FLAGS -o "$OUT_DIR/test_planner_chain.exe" \
    test/test_planner/test_planner_chain.cpp "$UNITY_DIR/unity.c" \
    -Wno-deprecated 2>&1

clang++ $FLAGS -o "$OUT_DIR/test_pd_controller.exe" \
    test/test_planner/test_pd_controller.cpp "$UNITY_DIR/unity.c" \
    -Wno-deprecated 2>&1

echo ""
echo "=== Planner & Trajectory Tests ==="
"$OUT_DIR/test_planner_chain.exe"

echo ""
echo "=== PD Controller Tests ==="
"$OUT_DIR/test_pd_controller.exe"
