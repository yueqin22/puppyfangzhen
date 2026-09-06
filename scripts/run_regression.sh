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

# 解释器选择: 本机存在两个 python —— 3.13 (默认) 没有 PyYAML/pytest,
# C:\Program Files\Python312 才有。直接用默认解释器会让"配置/几何/单测"门禁
# 静默降级成 SKIP 或误报, 所以显式挑一个**同时具备 pytest 与 PyYAML**的解释器。
pick_py() {
    for cand in "${PYTHON:-}" "python3" "python" \
        "C:/Program Files/Python312/python.exe" \
        "C:/Users/Administrator/.workbuddy/binaries/python/versions/3.13.12/python.exe"; do
        [ -n "$cand" ] || continue
        command -v "$cand" >/dev/null 2>&1 || [ -x "$cand" ] || continue
        if "$cand" -c "import pytest, yaml" >/dev/null 2>&1; then
            echo "$cand"; return 0
        fi
    done
    echo "${PYTHON:-python3}"
}
PY="$(pick_py)"

# 激活 profile (用于报告标签; 默认 dev)
PROFILE="$("$PY" -c "import sys;sys.path.insert(0,'$ROOT');from config.config_loader import parse_yaml;print((parse_yaml(open('$ROOT/config/unified_params.yaml',encoding='utf-8').read()) or {}).get('profile','dev'))" 2>/dev/null || echo dev)"

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
# 同一份源码产出两个可执行文件:
#   nav_ue_bridge.exe       生产入口 (接真实 UE)
#   nav_ue_bridge_test.exe  §5.1/§5.2 复验用的同源码副本
#     为什么单独再编一份: verify_bridge_handshake.py 会主动连 7777 端口并断言
#     无客户端超时/静默急停, 与人工启动的 UE 会话抢端口。给它独立文件名, 是为了
#     让"门禁跑的 bridge"与"演示跑的 bridge"可区分, 而不是靠约定不冲突。
echo "== [build] nav_ue_bridge =="
BRIDGE_SRC="nav_ue_bridge.cpp \
    ../puppy_nav_core/src/occupancy_grid.cpp \
    ../puppy_nav_core/src/costmap.cpp \
    ../puppy_nav_core/src/astar_planner.cpp \
    ../puppy_nav_core/src/amcl.cpp \
    ../puppy_nav_core/src/path_validator.cpp"
( cd "$BRIDGE_DIR" && "$CL" /O2 /std:c++17 /EHsc /utf-8 /MT /DNOMINMAX \
    /I. /I../sim /I../puppy_nav_core/include /I../common \
    $BRIDGE_SRC \
    /Fe:build/Release/nav_ue_bridge.exe \
    /link /OUT:build/Release/nav_ue_bridge.exe ws2_32.lib ) >/dev/null 2>&1
if [ $? -ne 0 ]; then echo "[FAIL] nav_ue_bridge build"; BUILD_FAIL=$((BUILD_FAIL+1)); fi

( cd "$BRIDGE_DIR" && "$CL" /O2 /std:c++17 /EHsc /utf-8 /MT /DNOMINMAX \
    /I. /I../sim /I../puppy_nav_core/include /I../common \
    $BRIDGE_SRC \
    /Fe:build/Release/nav_ue_bridge_test.exe \
    /link /OUT:build/Release/nav_ue_bridge_test.exe ws2_32.lib ) >/dev/null 2>&1
if [ $? -ne 0 ]; then echo "[FAIL] nav_ue_bridge_test build"; BUILD_FAIL=$((BUILD_FAIL+1)); fi

SIM="$SIM_DIR/build/Release/sim_test.exe"
BR="$BRIDGE_DIR/build/Release/nav_ue_bridge.exe"
BRT="$BRIDGE_DIR/build/Release/nav_ue_bridge_test.exe"
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

# ---- 9) 场景校验 (exit 0) ----
# 场景是单一真源: 房间/障碍/目标/初始位姿/可达性/UE 镜像全在这里把关。
echo "== [test] scene_validation =="
"$PY" "$ROOT/scripts/validate_scene.py" >/dev/null 2>&1
report "scene_validation (exit 0)" 0 $?

# ---- 10) 几何尺寸一致性 (jihua20260905 §4 P0-3 出口标准) ----
# 防止"契约说 0.35、Nav2 实际按 0.25 规划"这类偷安全的隐性漂移。
echo "== [test] geometry_consistency =="
"$PY" "$ROOT/scripts/check_geometry_consistency.py" --report "$ART/geometry_consistency.json" >/dev/null 2>&1
report "geometry_consistency (exit 0)" 0 $?

# ---- 10b) vx/vy/wz 运动能力表 (§4 P0-3 第 4 条 / §7.4) ----
# 防止"导航指望某个轴、限速却把它夹成 0"的静默失效 —— 这正是 max_linear_y=0.0
# 那个历史坑的形状。
echo "== [test] motion_capability =="
"$PY" "$ROOT/scripts/check_motion_capability.py" --report "$ART/motion_capability.json" >/dev/null 2>&1
report "motion_capability (exit 0)" 0 $?

# ---- 10c) 运行 Profile 有效性 (§11 第 1 周出口) ----
# 激活 profile 必须定义在 unified_params.yaml 的 profiles: 中, 且 nav_core 不得为 mock。
echo "== [test] profile_validity =="
"$PY" "$ROOT/scripts/check_profile.py" --report "$ART/profile_check.json" >/dev/null 2>&1
report "profile_validity (exit 0)" 0 $?

# ---- 10d) 文档一致性 (§13 风险登记第 8 项) ----
# 发布前必须无"历史文档与当前场景矛盾": 旧房间数 / 旧半径 / Jazzy 混写。
# 历史计划/交接文档降为 WARNING, 不阻断; 活动文档矛盾阻断门禁。
echo "== [test] docs_consistency =="
"$PY" "$ROOT/scripts/check_docs_consistency.py" --report "$ART/docs_consistency.json" >/dev/null 2>&1
report "docs_consistency (exit 0)" 0 $?

# ---- 11) §5.1 bridge 握手复验 ----
# 注意用 nav_ue_bridge_test 副本: 它会主动占用端口并断言"无客户端超时",
# 不能与人工演示会话抢 7777。
echo "== [test] bridge_handshake (§5.1) =="
if [ -x "$BRT" ]; then
    "$PY" "$ROOT/scripts/verify_bridge_handshake.py" --bridge "$BRT" \
        --report "$ART/bridge_handshake.json" >/dev/null 2>&1
    report "bridge_handshake (exit 0)" 0 $?
else
    echo "[SKIP] bridge_handshake (bridge not built)"; FAIL=$((FAIL+1))
fi

# ---- 12) §5.2/§5.3 bridge 首帧一致性与逐帧指标 ----
echo "== [test] bridge_loopback (§5.2/§5.3) =="
if [ -x "$BRT" ]; then
    "$PY" "$ROOT/scripts/verify_bridge_loopback.py" --bridge "$BRT" \
        --report "$ART/bridge_loopback.json" >/dev/null 2>&1
    report "bridge_loopback (exit 0)" 0 $?
else
    echo "[SKIP] bridge_loopback (bridge not built)"; FAIL=$((FAIL+1))
fi

# ---- 13) §5.2 bridge 单位换算单测 (UE cm <-> bridge m) ----
echo "== [test] bridge_units =="
"$PY" -m pytest -q --no-header -p no:cacheprovider \
    "$ROOT/cpp_src/bridge/test_bridge_units.py" >/dev/null 2>&1
report "bridge_units (exit 0)" 0 $?

# ---- 14) Python 单测 (§12 验收: 关键测试 100% 通过) ----
# 刻意只跑**受维护套件**, 不跑仓库根:
#   根级散落 17 个 test_*.py / 十余个研究脚本 (test_v4_all/test_zmq/test_conn 等),
#   直接 `pytest -q` 根目录会全仓收集 —— 实测 8 分钟仍未结束 (还会去连 socket),
#   数字既不可复现也不可解释。受维护套件见 docs/test_inventory.md。
echo "== [test] python_pytest =="
if [ "${SKIP_PYTEST:-0}" = "1" ]; then
    echo "[SKIP] python_pytest (SKIP_PYTEST=1)"
else
    ( cd "$ROOT/src/puppy_minicpm_robot" && \
      PYTHONPATH=".$PYTHONPATH" \
      "$PY" -m pytest test -q --no-header -p no:cacheprovider ) \
        >"$ART/python_pytest.txt" 2>&1
    report "python_pytest (exit 0)" 0 $?
fi

echo "================================================================"
echo " 回归结果: PASS=$PASS  FAIL=$FAIL  BUILD_FAIL=$BUILD_FAIL"
echo "================================================================"

# ---- 15) §9.3 一键发布报告 (信息性, 不阻断) ----
# 聚合 artifacts/ 下已存在的 cpp_*_seedN_planB.json; 若本环境跑过 36000/108000
# 长稳, 则直接生成 artifacts/<run_id>/ 完整报告, 否则仅聚合已有 smoke 证据。
if ls "$ART"/cpp_*_seed1_planB.json >/dev/null 2>&1; then
    echo "== [report] gen_release_report (§9.3) =="
    RID="release-$(date +%Y%m%d)"
    "$PY" "$ROOT/scripts/gen_release_report.py" --run-id "$RID" --profile "$PROFILE" \
        >"$ART/release_report.log" 2>&1 || echo "[WARN] gen_release_report 未全绿, 见 $ART/release_report.log"
    echo "  -> artifacts/$RID/ (summary.md + plots/)"
else
    echo "[SKIP] gen_release_report: 未发现 cpp_*_seedN_planB.json (本回归仅 smoke)"
fi

if [ "$FAIL" -eq 0 ] && [ "$BUILD_FAIL" -eq 0 ]; then
    echo "REGRESSION: ALL GREEN"
    exit 0
else
    echo "REGRESSION: HAS FAILURES"
    exit 1
fi
