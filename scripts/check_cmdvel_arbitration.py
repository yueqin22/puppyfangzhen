#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_cmdvel_arbitration.py — /cmd_vel 单一仲裁者门禁
================================================================================
依据 jihua20260905 §4 P0-4 ("明确 /cmd_vel 的唯一仲裁者, Nav2、MiniCPM、巡逻和
恢复节点不得同时直连硬件") 与 §13 风险登记 ("多套脚本同时发速度 -> 运动抢占和
不可复现 -> 单一 safety/mission 仲裁层"), 以及 INTERFACES.md §1.1 的话题契约。

强制的分层:

    意图层 (可多个)   Nav2 / teleop / MiniCPM / 巡逻 / 恢复  -->  /cmd_vel
    仲裁层 (唯一)     safety_manager                        -->  /cmd_vel_safe
    执行层            motion_adapter / fake_motion_server    -->  硬件/仿真

核心规则:
  R1 恰好一个仲裁者: 同时订阅 /cmd_vel 且发布 /cmd_vel_safe
  R2 只有仲裁者可以发布 /cmd_vel_safe  (发布即"绕过安全仲裁", 直接 FAIL)
  R3 执行层只订阅 /cmd_vel_safe, 绝不发布它
  R4 警告: 仲裁者若未订阅看门狗急停, 急停无法作用到执行器

为什么 R2 是硬门禁:
  /cmd_vel_safe 是 safety_manager 的输出, 直连执行器。任何其它节点发它,
  就绕过了 电机使能 / 限速 / 低电量 / 跌倒 / 命令超时急停 五道安全闸门。
  历史真实缺陷: track_cmd_adapter 曾把"未抑制的指令速度"发到 /cmd_vel_safe,
  而 dry-run 只把 /cmd_vel 置零 —— 表面安全, 实际执行器收到了满速指令。

用法:
    python scripts/check_cmdvel_arbitration.py
    python scripts/check_cmdvel_arbitration.py --report artifacts/cmdvel.json
"""
import os
import re
import sys
import json
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

INTENT_TOPIC = "/cmd_vel"
SAFE_TOPIC = "/cmd_vel_safe"

# 扫描目录 (ROS2 Python 包 + C++ 适配器镜像)
SCAN_DIRS = ["src", "cpp_src"]

# Python: create_publisher(Twist, "/topic", 10)
RE_PY_PUB = re.compile(
    r"create_publisher\s*\(\s*[^,()]+,\s*[\"'](/?[A-Za-z0-9_/]+)[\"']", re.M)
RE_PY_SUB = re.compile(
    r"create_subscription\s*\(\s*[^,()]+,\s*[\"'](/?[A-Za-z0-9_/]+)[\"']", re.M)

# C++: create_publisher<Twist>("/topic", 10)
RE_CPP_PUB = re.compile(
    r"create_publisher\s*<[^>]*>\s*\(\s*[\"'](/?[A-Za-z0-9_/]+)[\"']", re.M)
RE_CPP_SUB = re.compile(
    r"create_subscription\s*<[^>]*>\s*\(\s*[\"'](/?[A-Za-z0-9_/]+)[\"']", re.M)


def scan(root):
    """返回 {relpath: {"pub": set(topics), "sub": set(topics)}}"""
    graph = {}
    for d in SCAN_DIRS:
        base = os.path.join(root, d)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames
                           if x not in ("build", "install", "log",
                                        "__pycache__", ".git")]
            for fn in filenames:
                if not fn.endswith((".py", ".cpp", ".h", ".hpp")):
                    continue
                p = os.path.join(dirpath, fn)
                try:
                    text = open(p, encoding="utf-8", errors="ignore").read()
                except OSError:
                    continue
                pubs, subs = set(), set()
                for rx in (RE_PY_PUB, RE_CPP_PUB):
                    pubs.update(rx.findall(text))
                for rx in (RE_PY_SUB, RE_CPP_SUB):
                    subs.update(rx.findall(text))
                if not (pubs or subs):
                    continue
                rel = os.path.relpath(p, root).replace("\\", "/")
                graph[rel] = {"pub": pubs, "sub": subs}
    return graph


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=_ROOT)
    ap.add_argument("--report", default=None, help="写出 JSON 报告")
    args = ap.parse_args()

    graph = scan(args.root)

    # src/ 与 cpp_src/ 是同一批节点的 Python / C++ 双实现 (如 safety_manager.py
    # 与 safety_manager.cpp)。仲裁者是"逻辑节点", 必须按逻辑名去重后再判数量,
    # 否则双实现会被误判成"两个仲裁者互相抢占"。
    def logical(rel):
        return os.path.splitext(os.path.basename(rel))[0]

    lg = {}
    for f, g in graph.items():
        n = logical(f)
        e = lg.setdefault(n, {"pub": set(), "sub": set(), "files": []})
        e["pub"] |= g["pub"]
        e["sub"] |= g["sub"]
        e["files"].append(f)

    def primary(n):
        """同一逻辑节点里优先取 .py 作为判据文件。"""
        fs = sorted(lg[n]["files"])
        for f in fs:
            if f.endswith(".py"):
                return f
        return fs[0]

    intent_pubs = sorted(n for n, e in lg.items() if INTENT_TOPIC in e["pub"])
    safe_pubs = sorted(n for n, e in lg.items() if SAFE_TOPIC in e["pub"])
    safe_subs = sorted(n for n, e in lg.items() if SAFE_TOPIC in e["sub"])
    arbiters = sorted(n for n in safe_pubs
                      if INTENT_TOPIC in lg[n]["sub"])
    graph = lg  # 后续按逻辑节点查表

    checks = []

    # ---- R1 恰好一个仲裁者 ----
    ok = (len(arbiters) == 1)
    checks.append(("single_arbiter", ok,
                   "仲裁者(订阅 %s 且发布 %s) 数量 = %d%s"
                   % (INTENT_TOPIC, SAFE_TOPIC, len(arbiters),
                      "" if ok else " (期望 1): " + ", ".join(arbiters or ["无"]))))

    # ---- R2 只有仲裁者可以发布 /cmd_vel_safe ----
    bypass = [f for f in safe_pubs if f not in arbiters]
    ok2 = not bypass
    checks.append(("no_safe_topic_bypass", ok2,
                   "非仲裁者发布 %s 的节点: %s"
                   % (SAFE_TOPIC, ", ".join(bypass) if bypass else "无")))

    # ---- R3 执行层不发布 /cmd_vel_safe ----
    bad_act = [f for f in safe_subs if SAFE_TOPIC in graph[f]["pub"] and f not in arbiters]
    ok3 = not bad_act
    checks.append(("actuator_publish_free", ok3,
                   "执行层(订阅 %s)反发该话题: %s"
                   % (SAFE_TOPIC, ", ".join(bad_act) if bad_act else "无")))

    # ---- R4 警告: 仲裁者是否消费看门狗急停 ----
    warn = []
    if arbiters:
        arb = primary(arbiters[0])
        text = open(os.path.join(args.root, arb), encoding="utf-8",
                    errors="ignore").read()
        if "emergency_stop" not in text:
            warn.append("仲裁者 %s 未订阅看门狗急停 (/watchdog/emergency_stop); "
                        "看门狗触发的急停无法作用到执行器。" % arb)
        if "motors_enabled" not in text and "motor" not in text.lower():
            warn.append("仲裁者 %s 似乎未做电机使能判定。" % arb)

    print("=" * 68)
    print("  /cmd_vel 单一仲裁者门禁 (jihua20260905 §4 P0-4 / §13)")
    print("=" * 68)
    def impl_tag(n):
        exts = sorted({os.path.splitext(f)[1] for f in lg[n]["files"]})
        return "+".join(e.lstrip(".") for e in exts)

    print("  意图话题 %s 发布者 (%d 个逻辑节点):" % (INTENT_TOPIC, len(intent_pubs)))
    for n in intent_pubs:
        print("      - %-24s [%s]" % (n, impl_tag(n)))
    print("  安全话题 %s 发布者 (%d 个逻辑节点):" % (SAFE_TOPIC, len(safe_pubs)))
    for n in safe_pubs:
        tag = "  <- 仲裁者" if n in arbiters else "  !! 绕过仲裁"
        print("      - %-24s [%s]%s" % (n, impl_tag(n), tag))
    print("  安全话题 %s 订阅者 (%d 个逻辑节点):" % (SAFE_TOPIC, len(safe_subs)))
    for n in safe_subs:
        print("      - %-24s [%s]" % (n, impl_tag(n)))
    print("-" * 68)
    for name, okx, msg in checks:
        print("  %-26s [%s]  %s" % (name, "PASS" if okx else "FAIL", msg))
    for w in warn:
        print("  %-26s [%s]  %s" % ("warning", "WARN", w))
    print("-" * 68)

    passed = all(c[1] for c in checks)
    print("RESULT: %s" % ("PASS" if passed else "FAIL"))
    if arbiters:
        print("  (仲裁者: %s)" % ", ".join(arbiters))

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump({
                "intent_topic": INTENT_TOPIC, "safe_topic": SAFE_TOPIC,
                "intent_publishers": intent_pubs,
                "safe_publishers": safe_pubs,
                "safe_subscribers": safe_subs,
                "arbiters": arbiters,
                "bypass_nodes": bypass,
                "checks": [{"rule": n, "pass": o, "detail": m}
                           for n, o, m in checks],
                "warnings": warn,
                "result": "PASS" if passed else "FAIL",
            }, f, indent=2, ensure_ascii=False)

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
