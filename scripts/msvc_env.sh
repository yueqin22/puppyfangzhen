#!/bin/bash
# msvc_env.sh — Set up MSVC 2022 x64 toolchain without invoking `cmd /c`
# Source this file before any cl.exe / cmake --build invocation:
#   source scripts/msvc_env.sh
#   cd /e/puppyfangzhen/cpp_src/sim/build && cmake --build . --config Release
export VS="C:/Program Files/Microsoft Visual Studio/2022/Community"
export VCT="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.44.35207"
export SDK="C:/Program Files (x86)/Windows Kits/10"
export SDKVER="10.0.26100.0"
export NINJA="$VS/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja"

# PATH: Git Bash/MSYS needs POSIX drive paths for native tools.  Keeping the
# old ``C:/Program Files/...`` spelling here makes command lookup split at the
# space and causes `cl.exe` to be reported as not found.
MSYS_VCT="/c/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.44.35207"
MSYS_SDK="/c/Program Files (x86)/Windows Kits/10"
MSYS_NINJA="/c/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja"
export PATH="$MSYS_VCT/bin/Hostx64/x64:$MSYS_SDK/bin/$SDKVER/x64:$MSYS_NINJA:$PATH"

# INCLUDE / LIB: cl.exe uses ';' as separator (Windows tool semantics)
export INCLUDE="$VCT/include;$SDK/Include/$SDKVER/ucrt;$SDK/Include/$SDKVER/um;$SDK/Include/$SDKVER/shared;$SDK/Include/$SDKVER/winrt"
export LIB="$VCT/lib/x64;$SDK/Lib/$SDKVER/ucrt/x64;$SDK/Lib/$SDKVER/um/x64"

# MSYS otherwise rewrites compiler switches such as `/O2` and `/I...` into
# drive paths before invoking a native Windows executable.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL='*'
