"""PlatformIO extra script: fall back to Clang when GCC is not available.

Creates shim scripts (gcc.bat / g++.bat / ar.bat) in a temporary directory
and prepends it to the PATH so that all SCons environments — including
cloned ones used by the test framework — invoke Clang transparently.
"""
import os
import shutil
import sys
import tempfile

Import("env")

if shutil.which("gcc"):
    # GCC available — no need for shims
    pass
else:
    clang = shutil.which("clang")
    clangpp = shutil.which("clang++")
    llvm_ar = shutil.which("llvm-ar")
    llvm_ranlib = shutil.which("llvm-ranlib")

    if not clang:
        sys.exit("use_clang.py: neither gcc nor clang found on PATH")

    # Create a temp directory with shim .bat files
    shim_dir = os.path.join(env.subst("$BUILD_DIR"), "_clang_shims")
    os.makedirs(shim_dir, exist_ok=True)

    def write_shim(name, target):
        path = os.path.join(shim_dir, name + ".bat")
        with open(path, "w") as f:
            f.write('@"%s" %%*\n' % target)

    write_shim("gcc", clang)
    write_shim("g++", clangpp or clang)
    write_shim("ar", llvm_ar or "ar")
    write_shim("ranlib", llvm_ranlib or "ranlib")

    # Prepend shim dir to PATH for all SCons environments
    cur_path = env["ENV"].get("PATH", os.environ.get("PATH", ""))
    env["ENV"]["PATH"] = shim_dir + os.pathsep + cur_path

    print("use_clang.py: shims created in %s" % shim_dir, file=sys.stderr)
