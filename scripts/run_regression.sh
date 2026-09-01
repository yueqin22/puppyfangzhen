#!/usr/bin/env bash
# run_regression.sh — 构建并运行 Puppy 导航仿真回归套件 (jihua20260818 §14/§15)
# =====================================================================
# 一键完成: 编译 sim_test + nav_ue_bridge, 运行 C++/Python 冒烟、
# 配置一致性、Bridge 诊断, 输出统一 PASS/FAIL 汇总与退出码。
#
# 用法 (Git Bash / WSL):
#   bash scripts/run_regression.sh
# 退出码: 0 = 全部通过, 非 0 = 存在失败项
#
# 依赖: scripts/msvc_env.sh (MSVC 2022 x64 工具链)
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd -W)"
cd "$ROOT"
source "$ROOT/scripts/msvc_env.sh"

CL="$VCT/bin/Hostx64/x64/cl.exe"
SIM_DIR="$ROOT/cpp_src/sim"
BRIDGE_DIR="$ROOT/cpp_src/bridge"
ART="$ROOT/artifacts"
mkdir -p "$ART"

PASS=0
FAIL=0
BUILD_FAIL=0

report() {  # name expected_exit actual_exit
    if [ "$2" = "$3" ]; then
        echo "[PASS] $1 (exit=$3)"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] $1 (expected=$2 actual=$3)"
        FAIL=$((FAIL + 1))
    fi
}

PY="${PYTHON:-python3}"

echo "================================================================"
echo " Puppy 导航仿真回归套件"
echo " root : $ROOT"
echo " clang: $CL"
echo "================================================================"

# ---- 1) 编译 sim_test ----
echo "== [build] sim_test =="
( cd "$SIM_DIR" && "$CL" /O2 /std:c++17 /EHsc /utf-8 /MT \
    /I../puppy_nav_core/include /I. /I../common \
    main.cpp \
    ../puppy_nav_core/src/occupancy_grid.cpp \
    ../puppy_nav_core/src/costmap.cpp \
    ../puppy_nav_core/src/astar_planner.cpp \
    ../puppy_nav_core/src/amcl.cpp \
    ../puppy_nav_core/src/path_validator.cpp \
    /Fe:build/Release/sim_test.exe \
    /link /OUT:build/Release/sim_test.exe ) >/dev/null 2>&1
if [ $? -ne 0 ]; then echo "[FAIL] sim_test build"; BUILD_FAIL=$((BUILD_FAIL+1)); fi

# ---- 2) 编译 nav_ue_bridge ----
echo "== [build] nav_ue_bridge =="
( cd "$BRIDGE_DIR" && "$CL" /O2 /std:c++17 /EHsc /utf-8 /MT /DNOMINMAX \
    /I. /I../sim /I../puppy_nav_core/include /I../common \
    nav_ue_bridge.cpp \
    ../puppy_nav_core/src/occupancy_grid.cpp \
    ../puppy_nav_core/src/costmap.cpp \
    ../puppy_nav_core/src/astar_planner.cpp \
    ../puppy_nav_core/src/amcl.cpp \
    ../puppy_nav_core/src/path_validator.cpp \
    /Fe:build/Release/nav_ue_bridge.exe \
    /link /OUT:build/Release/nav_ue_bridge.exe ws2_32.lib ) >/dev/null 2>&1
if [ $? -ne 0 ]; then echo "[FAIL] nav_ue_bridge build"; BUILD_FAIL=$((BUILD_FAIL+1)); fi

SIM="$SIM_DIR/build/Release/sim_test.exe"
BR="$BRIDGE_DIR/build/Release/nav_ue_bridge.exe"
SC="$ROOT/config/scene_home.json"

# ---- 3) C++ 冒烟 (300 帧, 安全门控, exit 0) ----
echo "== [test] cpp_smoke =="
if [ -x "$SIM" ]; then
    "$SIM" --frames 300 --seed 1 --timeout 60 --report "$ART/cpp_smoke.json" >/dev/null 2>&1
    report "cpp_smoke (exit 0)" 0 $?
else
    echo "[SKIP] cpp_smoke (sim_test not built)"; FAIL=$((FAIL+1))
fi

# ---- 4) Bridge 诊断: --check-scene ----
echo "== [test] bridge --check-scene =="
if [ -x "$BR" ]; then
    "$BR" --check-scene "$SC" >/dev/null 2>&1
    report "bridge --check-scene (exit 0)" 0 $?
else
    echo "[SKIP] bridge --check-scene (bridge not built)"; FAIL=$((FAIL+1))
fi

# ---- 5) Bridge 诊断: --dry-run ----
echo "== [test] bridge --dry-run =="
if [ -x "$BR" ]; then
    "$BR" --dry-run --scene "$SC" >/dev/null 2>&1
    report "bridge --dry-run (exit 0)" 0 $?
else
    echo "[SKIP] bridge --dry-run (bridge not built)"; FAIL=$((FAIL+1))
fi

# ---- 6) Bridge 诊断: --connect-timeout 无客户端必须超时退出 (exit 4) ----
echo "== [test] bridge --connect-timeout (no UE, expect 4) =="
if [ -x "$BR" ]; then
    "$BR" --connect-timeout 2 >/dev/null 2>&1
    report "bridge --connect-timeout (exit 4)" 4 $?
else
    echo "[SKIP] bridge --connect-timeout (bridge not built)"; FAIL=$((FAIL+1))
fi

# ---- 7) Python 无头冒烟 (exit 0) ----
echo "== [test] python_smoke =="
"$PY" "$ROOT/scripts/run_python_smoke.py" --frames 300 --seed 1 --report "$ART/python_smoke.json" >/dev/null 2>&1
report "python_smoke (exit 0)" 0 $?

# ---- 8) 配置一致性 (exit 0) ----
echo "== [test] config_consistency =="
"$PY" "$ROOT/scripts/check_config_consistency.py" --report "$ART/config_consistency.json" >/dev/null 2>&1
report "config_consistency (exit 0)" 0 $?

echo "================================================================"
echo " 回归结果: PASS=$PASS  FAIL=$FAIL  BUILD_FAIL=$BUILD_FAIL"
echo "================================================================"
if [ "$FAIL" -eq 0 ] && [ "$BUILD_FAIL" -eq 0 ]; then
    echo "REGRESSION: ALL GREEN"
    exit 0
else
    echo "REGRESSION: HAS FAILURES"
    exit 1
fi
