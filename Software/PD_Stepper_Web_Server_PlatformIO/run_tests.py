#!/usr/bin/env python3
"""Build and run native C++ unit tests for the planner core and PD controller.

Works on any platform (Windows, macOS, Linux) with either GCC or Clang installed.
PlatformIO is NOT required — only a C++ compiler.

Usage:
    python run_tests.py          # auto-detect compiler
    python run_tests.py --cxx clang++   # force a specific compiler

Exit code: 0 if all tests pass, 1 otherwise.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
TEST_DIR = ROOT / "test" / "test_planner"
OUT_DIR = ROOT / ".pio" / "build" / "native_manual"

# Unity can come from PlatformIO's package cache or a local checkout
UNITY_CANDIDATES = [
    ROOT / ".pio" / "libdeps" / "native_test" / "Unity" / "src",
    ROOT / "lib" / "Unity" / "src",
]

TEST_FILES = [
    ("test_planner_chain", TEST_DIR / "test_planner_chain.cpp"),
    ("test_pd_controller", TEST_DIR / "test_pd_controller.cpp"),
    ("test_ring_buffer",   TEST_DIR / "test_ring_buffer.cpp"),
    ("test_loop_planner",  TEST_DIR / "test_loop_planner.cpp"),
]


def find_compiler() -> str:
    """Return the first available C++ compiler."""
    for cxx in ("g++", "clang++"):
        if shutil.which(cxx):
            return cxx
    print("ERROR: No C++ compiler found. Install GCC or Clang.", file=sys.stderr)
    sys.exit(1)


def find_unity() -> Path:
    """Return the path to the Unity test framework source directory."""
    for p in UNITY_CANDIDATES:
        if (p / "unity.h").exists():
            return p
    # Try to install via PlatformIO if available
    if shutil.which("pio"):
        print("Unity not found — installing via PlatformIO...")
        subprocess.run(
            ["pio", "pkg", "install", "-e", "native_test"],
            cwd=str(ROOT), check=False, capture_output=True,
        )
        for p in UNITY_CANDIDATES:
            if (p / "unity.h").exists():
                return p
    print("ERROR: Unity test framework not found.", file=sys.stderr)
    print("  Run: pio pkg install -e native_test", file=sys.stderr)
    sys.exit(1)


def build_and_run(cxx: str, unity: Path) -> bool:
    """Compile and run all test files. Returns True if all pass."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    unity_c = str(unity / "unity.c")
    flags = [
        "-std=c++17",
        "-DUNITY_INCLUDE_FLOAT",
        f"-I{SRC}",
        f"-I{unity}",
        "-Wno-deprecated",  # suppress clang 'treating .c as C++' warning
    ]

    all_passed = True
    for name, src in TEST_FILES:
        ext = ".exe" if sys.platform == "win32" else ""
        out = str(OUT_DIR / f"{name}{ext}")

        print(f"\n{'='*60}")
        print(f"Building {name}...")
        cmd = [cxx] + flags + ["-o", out, str(src), unity_c]
        result = subprocess.run(cmd, cwd=str(ROOT))
        if result.returncode != 0:
            print(f"FAILED to compile {name}")
            all_passed = False
            continue

        print(f"Running {name}...")
        result = subprocess.run([out], cwd=str(ROOT))
        if result.returncode != 0:
            all_passed = False

    return all_passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", help="C++ compiler to use (default: auto-detect)")
    args = parser.parse_args()

    cxx = args.cxx or find_compiler()
    print(f"Compiler: {cxx}")

    unity = find_unity()
    print(f"Unity:    {unity}")

    ok = build_and_run(cxx, unity)
    print(f"\n{'='*60}")
    print("ALL TESTS PASSED" if ok else "SOME TESTS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
