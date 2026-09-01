#!/usr/bin/env python3
"""
ue_regression_test.py — UE vs C++ 等价性回归测试 (UE-PLAN M4.1)
===============================================================
对比 UE 仿真 (nav_ue_bridge) 与 C++ sim_test 在相同种子下的指标差异，
验证 UE 场景几何/传感器/导航栈与 C++ 仿真等价。

三种运行模式:
  1. --mode cpp:        仅运行 C++ sim_test 基线
  2. --mode standalone:  运行 nav_ue_bridge --standalone (无需 UE, 验证 bridge 导航逻辑)
  3. --mode ue:          运行 nav_ue_bridge + UE (需要 UE 工程运行中)
  4. --mode compare:     同时运行 C++ + standalone, 对比差异 (默认)

用法:
  python ue_regression_test.py --mode compare --seeds 1,6,8 --frames 36000
  python ue_regression_test.py --mode standalone --seeds 1 --frames 3600
  python ue_regression_test.py --mode ue --seeds 1 --frames 36000 --ue-running

等价性判据 (standalone vs cpp):
  - astar_rate:  |差值| < 5%   (bridge 简化安全栈, 容差较大)
  - avg_err:     |差值| < 0.15m
  - max_err:     |差值| < 2.0m
  - rooms:       |差值| <= 2
  - collisions:  仅记录, 不做硬性判据 (bridge 无完整安全栈)

等价性判据 (ue vs cpp, UE 就绪后):
  - collision:   UE == CPP (必须 0)
  - astar_rate:  |UE - CPP| < 2%
  - avg_err:     |UE - CPP| < 0.10m
  - max_err:     |UE - CPP| < 1.0m
  - rooms:       |UE - CPP| <= 1
"""
import argparse
import json
import subprocess
import sys
import os
import re

# ===== 路径常量 =====
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(SCRIPT_DIR, "..", "sim")
BRIDGE_DIR = SCRIPT_DIR
SCENE_PATH = os.path.join(SCRIPT_DIR, "..", "..", "config", "scene_home.json")

def parse_summary_line(line):
    """解析 SUMMARY: 行"""
    result = {}
    for kv in line.split():
        if '=' in kv:
            k, v = kv.split('=', 1)
            try:
                result[k] = float(v)
            except ValueError:
                result[k] = v
    return result

def run_cpp_sim(seed, frames, sim_dir):
    """运行 C++ sim_test 仿真"""
    exe = os.path.join(sim_dir, "sim_test.exe")
    if not os.path.exists(exe):
        print(f"  [CPP] sim_test.exe 不存在: {exe}")
        return None

    # sim_test 参数: frames half_frames seed
    cmd = [exe, str(frames), str(frames // 2), str(seed)]
    print(f"  [CPP] seed={seed} frames={frames}")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                cwd=sim_dir, timeout=frames // 30 + 300)
    except subprocess.TimeoutExpired:
        print(f"  [CPP] 超时!")
        return None

    for line in result.stdout.split('\n'):
        if line.startswith('SUMMARY:'):
            return parse_summary_line(line)
    print(f"  [CPP] 未找到 SUMMARY 行")
    return None

def run_bridge_standalone(seed, frames, bridge_dir):
    """运行 nav_ue_bridge --standalone (无需 UE, 自测 bridge 导航逻辑)"""
    exe = os.path.join(bridge_dir, "build", "Release", "Release", "nav_ue_bridge.exe")
    if not os.path.exists(exe):
        # 尝试备用路径
        exe = os.path.join(bridge_dir, "build", "Release", "nav_ue_bridge.exe")
    if not os.path.exists(exe):
        print(f"  [BRIDGE] nav_ue_bridge.exe 不存在, 请先运行 build_bridge.bat")
        return None

    scene = os.path.abspath(SCENE_PATH)
    cmd = [exe, "--standalone", "--seed", str(seed), "--frames", str(frames),
           "--scene", scene]
    print(f"  [BRIDGE] seed={seed} frames={frames}")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                cwd=bridge_dir, timeout=frames // 30 + 300)
    except subprocess.TimeoutExpired:
        print(f"  [BRIDGE] 超时!")
        return None

    for line in result.stdout.split('\n'):
        if line.startswith('SUMMARY:'):
            return parse_summary_line(line)

    # 打印最后几行输出帮助调试
    lines = result.stdout.strip().split('\n')
    for line in lines[-5:]:
        print(f"    {line}")
    print(f"  [BRIDGE] 未找到 SUMMARY 行")
    return None

def run_ue_sim(seed, frames, bridge_dir, port=7777):
    """运行 UE 仿真 (需要 UE 和 bridge 都在运行)

    流程:
      1. 启动 nav_ue_bridge.exe (TCP server, --frames N)
      2. 等待 UE 连接 (UE 作为 TCP client)
      3. Bridge 处理 N 帧后输出 SUMMARY 并退出
      4. 解析 SUMMARY

    前提: UE5 编辑器需已加载场景并运行 (PIE),
          PuppyRobotPawn 的 TcpComp 会自动连接到 localhost:port
    """
    exe = os.path.join(bridge_dir, "build", "Release", "Release", "nav_ue_bridge.exe")
    if not os.path.exists(exe):
        exe = os.path.join(bridge_dir, "build", "Release", "nav_ue_bridge.exe")
    if not os.path.exists(exe):
        print(f"  [UE] nav_ue_bridge.exe 不存在, 请先运行 build_bridge.bat")
        return None

    scene = os.path.abspath(SCENE_PATH)
    cmd = [exe, "--port", str(port), "--frames", str(frames), "--scene", scene]
    print(f"  [UE] seed={seed} frames={frames} port={port}")
    print(f"  [UE] 请确保 UE5 已运行场景 (PIE), 等待连接...")

    # 超时: frames/30 秒 + 60秒连接等待
    timeout = frames // 30 + 120
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                cwd=bridge_dir, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"  [UE] 超时 ({timeout}s), UE 可能未连接")
        return None

    for line in result.stdout.split('\n'):
        if line.startswith('SUMMARY:'):
            return parse_summary_line(line)

    lines = result.stdout.strip().split('\n')
    for line in lines[-5:]:
        print(f"    {line}")
    print(f"  [UE] 未找到 SUMMARY 行")
    return None

def compare_metrics(cpp_metrics, other_metrics, label="UE", strict=True):
    """对比 C++ 和其他 (standalone/UE) 指标

    strict=True:  UE vs CPP (严格判据)
    strict=False: standalone vs CPP (宽松判据, bridge 无完整安全栈)
    """
    if not other_metrics:
        return False, f"{label} 结果不可用"

    if strict:
        checks = [
            ("collision",   lambda c, u: u == c,                    f"{label}碰撞 != CPP碰撞"),
            ("astar_rate",  lambda c, u: abs(u - c) < 2.0,          "A*成功率差异 >= 2%"),
            ("avg_err",     lambda c, u: abs(u - c) < 0.10,         "avg_err差异 >= 0.10m"),
            ("max_err",     lambda c, u: abs(u - c) < 1.0,          "max_err差异 >= 1.0m"),
            ("rooms",       lambda c, u: abs(u - c) <= 1,           "房间数差异 > 1"),
        ]
    else:
        checks = [
            ("astar_rate",  lambda c, u: abs(u - c) < 5.0,          "A*成功率差异 >= 5%"),
            ("avg_err",     lambda c, u: abs(u - c) < 0.15,         "avg_err差异 >= 0.15m"),
            ("max_err",     lambda c, u: abs(u - c) < 2.0,          "max_err差异 >= 2.0m"),
            ("rooms",       lambda c, u: abs(u - c) <= 2,           "房间数差异 > 2"),
            # collision 仅记录不做判据 (bridge 无完整安全栈)
        ]

    all_pass = True
    for key, check, fail_msg in checks:
        c = cpp_metrics.get(key)
        u = other_metrics.get(key)
        if c is None or u is None:
            print(f"    {key}: 数据缺失 (cpp={c}, {label.lower()}={u})")
            all_pass = False
            continue

        passed = check(c, u)
        status = "PASS" if passed else "FAIL"
        print(f"    {key}: cpp={c} {label.lower()}={u} [{status}]")
        if not passed:
            print(f"      -> {fail_msg}")
            all_pass = False

    # collision 在非严格模式下仅记录
    if not strict:
        c = cpp_metrics.get("collision", 0)
        u = other_metrics.get("collision", 0)
        print(f"    collision: cpp={c} {label.lower()}={u} [INFO] (bridge 无完整安全栈, 仅记录)")

    return all_pass, "全部通过" if all_pass else "存在差异"

def main():
    parser = argparse.ArgumentParser(description='UE vs C++ 等价性回归测试')
    parser.add_argument('--mode', default='compare',
                        choices=['cpp', 'standalone', 'ue', 'compare'],
                        help='运行模式: cpp(仅C++) / standalone(仅bridge) / ue(需UE运行) / compare(C++ vs standalone)')
    parser.add_argument('--seeds', default='1,6,8', help='种子列表 (逗号分隔)')
    parser.add_argument('--frames', type=int, default=36000, help='帧数 (默认36000=10分钟)')
    parser.add_argument('--sim-dir', default=os.path.abspath(SIM_DIR), help='C++仿真目录')
    parser.add_argument('--bridge-dir', default=os.path.abspath(BRIDGE_DIR), help='Bridge目录')
    parser.add_argument('--port', type=int, default=7777, help='UE bridge TCP端口')
    args = parser.parse_args()

    seeds = [int(s.strip()) for s in args.seeds.split(',')]
    print(f"=== 等价性回归测试 ===")
    print(f"  模式: {args.mode}")
    print(f"  种子: {seeds}")
    print(f"  帧数: {args.frames}")
    print()

    results = []
    for seed in seeds:
        print(f"--- Seed {seed} ---")

        cpp_metrics = None
        other_metrics = None
        label = "BRIDGE"
        strict = False

        if args.mode in ('cpp', 'compare'):
            cpp_metrics = run_cpp_sim(seed, args.frames, args.sim_dir)
            if cpp_metrics:
                print(f"  [CPP] collision={int(cpp_metrics.get('collisions',0))} "
                      f"astar={cpp_metrics.get('astar_rate',0):.1f}% "
                      f"avg_err={cpp_metrics.get('avg_err',0):.3f}m "
                      f"max_err={cpp_metrics.get('max_err',0):.3f}m "
                      f"rooms={int(cpp_metrics.get('rooms',0))}")

        if args.mode == 'standalone':
            other_metrics = run_bridge_standalone(seed, args.frames, args.bridge_dir)
            label = "BRIDGE"
            strict = False
        elif args.mode == 'ue':
            other_metrics = run_ue_sim(seed, args.frames, args.bridge_dir, args.port)
            label = "UE"
            strict = True
        elif args.mode == 'compare':
            other_metrics = run_bridge_standalone(seed, args.frames, args.bridge_dir)
            label = "BRIDGE"
            strict = False

        if other_metrics:
            print(f"  [{label}] collision={int(other_metrics.get('collisions',0))} "
                  f"astar={other_metrics.get('astar_rate',0):.1f}% "
                  f"avg_err={other_metrics.get('avg_err',0):.3f}m "
                  f"max_err={other_metrics.get('max_err',0):.3f}m "
                  f"rooms={int(other_metrics.get('rooms',0))}")

        # 对比
        if args.mode == 'compare' and cpp_metrics and other_metrics:
            passed, msg = compare_metrics(cpp_metrics, other_metrics, label, strict)
            results.append((seed, passed, msg))
        elif args.mode == 'ue' and cpp_metrics and other_metrics:
            passed, msg = compare_metrics(cpp_metrics, other_metrics, label, strict)
            results.append((seed, passed, msg))
        elif args.mode in ('standalone', 'ue') and other_metrics:
            results.append((seed, True, "单侧运行完成"))
        elif args.mode == 'cpp' and cpp_metrics:
            results.append((seed, True, "C++基线完成"))
        elif cpp_metrics and not other_metrics:
            print(f"  [SKIP] {label} 结果不可用")
            results.append((seed, None, f"{label}未就绪"))
        else:
            print(f"  [ERROR] 仿真失败")
            results.append((seed, False, "失败"))

        print()

    # 汇总
    print("=== 汇总 ===")
    for seed, passed, msg in results:
        if passed is None:
            status = "SKIP"
        elif passed:
            status = "PASS"
        else:
            status = "FAIL"
        print(f"  Seed {seed}: {status} - {msg}")

    all_pass = all(p is True for _, p, _ in results) if results else False
    sys.exit(0 if all_pass else 1)

if __name__ == '__main__':
    main()
