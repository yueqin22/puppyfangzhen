#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_scene.py — 场景配置单一事实源校验 (规划 guihua20260812.md §5 / P0-03)
================================================================================
白盒测试：在启动任何正式仿真之前，校验 config/scene_home.json 的合法性与
一致性，确保"几何、目标、初始位姿"只有一份权威来源（规划 §3.1）。

检查项（任何硬检查失败 -> 退出码 1，阻止正式仿真）：
  [schema]         顶层必填字段与类型
  [grid]           网格 尺寸/分辨率/原点 自洽
  [obstacle-geom]  每个障碍 xmin<xmax, ymin<ymax, 落在网格范围内
  [target-safety]  每个巡航目标不在任何障碍膨胀区(radius_planning)内
  [init-safety]    初始位姿不在障碍膨胀区 / 网格边界外
  [reachability]   BFS(8连通, 障碍膨胀 radius_planning) 确认
                    初始点可达所有目标, 且目标两两可达(巡逻闭环)
  [ue-mirror]      UE 硬编码 HOME_OBSTACLES 与 JSON 数量/坐标一致
                    (不一致 = P0-02 单源违规, 必须失败)
  [hash]           输出 sha256, 用于产物追溯 (规划 §11.1)

用法:
  python scripts/validate_scene.py [--scene config/scene_home.json]
                                  [--ue D:/puppy_ue/Source/PuppyNav/PuppyRobotPawn.cpp]
"""
import argparse
import hashlib
import json
import os
import re
import sys

DEFAULT_SCENE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "config", "scene_home.json")
DEFAULT_UE = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
if not os.path.exists(DEFAULT_UE):
    wsl_candidate = "/mnt/d/puppy_ue/Source/PuppyNav/PuppyRobotPawn.cpp"
    if os.path.exists(wsl_candidate):
        DEFAULT_UE = wsl_candidate

REQUIRED_TOP = ["version", "grid", "robot", "lidar", "amcl", "obstacles",
                "patrol_targets", "pedestrians", "rooms"]


def log(name, ok, detail=""):
    mark = "[PASS]" if ok else "[FAIL]"
    line = "  %-14s %s" % (name, mark)
    if detail:
        line += "  " + detail
    print(line)
    return ok


def cell_of(x, y, g):
    cx = int(round((x - g["origin_x"]) / g["resolution"]))
    cy = int(round((y - g["origin_y"]) / g["resolution"]))
    return cx, cy


def build_blocked_grid(scene):
    g = scene["grid"]
    W, H = g["width"], g["height"]
    res = g["resolution"]
    r = scene["robot"]["radius_planning"]
    blocked = [[False] * H for _ in range(W)]
    for o in scene["obstacles"]:
        x0 = o["xmin"] - r
        x1 = o["xmax"] + r
        y0 = o["ymin"] - r
        y1 = o["ymax"] + r
        cx0 = int((x0 - g["origin_x"]) / res)
        cx1 = int((x1 - g["origin_x"]) / res)
        cy0 = int((y0 - g["origin_y"]) / res)
        cy1 = int((y1 - g["origin_y"]) / res)
        for cx in range(max(0, cx0), min(W, cx1 + 1)):
            for cy in range(max(0, cy0), min(H, cy1 + 1)):
                blocked[cx][cy] = True
    return blocked, W, H


def build_blocked_grid_r(scene, r):
    """build_blocked_grid variant with an explicit inflation radius (margin checks)."""
    g = scene["grid"]
    W, H = g["width"], g["height"]
    res = g["resolution"]
    blocked = [[False] * H for _ in range(W)]
    for o in scene["obstacles"]:
        x0, x1 = o["xmin"] - r, o["xmax"] + r
        y0, y1 = o["ymin"] - r, o["ymax"] + r
        cx0 = int((x0 - g["origin_x"]) / res)
        cx1 = int((x1 - g["origin_x"]) / res)
        cy0 = int((y0 - g["origin_y"]) / res)
        cy1 = int((y1 - g["origin_y"]) / res)
        for cx in range(max(0, cx0), min(W, cx1 + 1)):
            for cy in range(max(0, cy0), min(H, cy1 + 1)):
                blocked[cx][cy] = True
    return blocked, W, H


def bfs_reachable(blocked, W, H, src, dst):
    from collections import deque
    sx, sy = src
    dx, dy = dst
    if not (0 <= sx < W and 0 <= sy < H) or not (0 <= dx < W and 0 <= dy < H):
        return False
    if blocked[sx][sy] or blocked[dx][dy]:
        return False
    seen = [[False] * H for _ in range(W)]
    q = deque([(sx, sy)])
    seen[sx][sy] = True
    while q:
        x, y = q.popleft()
        if (x, y) == (dx, dy):
            return True
        for nx, ny in ((x+1, y), (x-1, y), (x, y+1), (x, y-1),
                       (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)):
            if 0 <= nx < W and 0 <= ny < H and not seen[nx][ny] and not blocked[nx][ny]:
                seen[nx][ny] = True
                q.append((nx, ny))
    return False


def parse_ue_home_obstacles(ue_path):
    """Extract the hardcoded HOME_OBSTACLES[] table from the UE pawn source (cm)."""
    if not os.path.exists(ue_path):
        return None
    txt = open(ue_path, encoding="utf-8", errors="ignore").read()
    # only the block between HOME_OBSTACLES[] = { ... };
    m = re.search(r"HOME_OBSTACLES\[\]\s*=\s*\{(.*?)\};", txt, re.S)
    if not m:
        return []
    body = m.group(1)
    out = []
    for mm in re.finditer(
            r'TEXT\("([^"]+)"\)\s*,\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*,\s*'
            r'(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?', body):
        name = mm.group(1).strip()
        xmin, ymin, xmax, ymax = (float(mm.group(i)) / 100.0 for i in (2, 3, 4, 5))
        out.append((name, xmin, ymin, xmax, ymax))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--ue", default=DEFAULT_UE)
    args = ap.parse_args()

    fails = 0
    print("SCENE VALIDATION  (%s)" % os.path.basename(args.scene))
    print("-" * 60)

    # ---- [schema] ----
    try:
        scene = json.load(open(args.scene, encoding="utf-8"))
        ok = all(k in scene for k in REQUIRED_TOP)
        if ok:
            log("schema", True)
        else:
            missing = [k for k in REQUIRED_TOP if k not in scene]
            log("schema", False, "missing: %s" % missing)
            fails += 1
    except Exception as e:
        log("schema", False, str(e))
        print("\nABORT: cannot read scene json")
        return 1

    # ---- [grid] ----
    g = scene.get("grid", {})
    gw, gh = g.get("width", 0), g.get("height", 0)
    res = g.get("resolution", 0)
    grid_ok = (gw > 0 and gh > 0 and res > 0 and
               abs(gw * res - 16.0) < 1e-6 and abs(gh * res - 12.0) < 1e-6)
    if not grid_ok:
        log("grid", False, "width*res=%g height*res=%g (expect 16x12)"
            % (gw * res, gh * res))
        fails += 1
    else:
        log("grid", True, "%dx%d @%.2fm (%.1fx%.1fm)" % (gw, gh, res, gw*res, gh*res))

    # ---- [obstacle-geom] ----
    x0w = g.get("origin_x", 0)
    y0w = g.get("origin_y", 0)
    x1w = x0w + gw * res
    y1w = y0w + gh * res
    geom_ok = True
    bad = []
    for o in scene["obstacles"]:
        if not (o["xmin"] < o["xmax"] and o["ymin"] < o["ymax"]):
            geom_ok = False
            bad.append(o["name"] + ":degenerate")
            continue
        if not (x0w - 1e-6 <= o["xmin"] and o["xmax"] <= x1w + 1e-6 and
                y0w - 1e-6 <= o["ymin"] and o["ymax"] <= y1w + 1e-6):
            geom_ok = False
            bad.append(o["name"] + ":out-of-grid")
    if geom_ok:
        log("obstacle-geom", True, "%d obstacles in-bounds" % len(scene["obstacles"]))
    else:
        log("obstacle-geom", False, ", ".join(bad))
        fails += 1

    # ---- [target-safety] & [init-safety] ----
    r = scene["robot"]["radius_planning"]
    def in_inflated(x, y):
        for o in scene["obstacles"]:
            if (o["xmin"] - r <= x <= o["xmax"] + r and
                    o["ymin"] - r <= y <= o["ymax"] + r):
                return o["name"]
        return None

    tgt_bad = []
    for t in scene["patrol_targets"]:
        hit = in_inflated(t["x"], t["y"])
        if hit:
            tgt_bad.append("%s(in %s)" % (t["name"], hit))
    if tgt_bad:
        log("target-safety", False, ", ".join(tgt_bad))
        fails += 1
    else:
        log("target-safety", True, "%d targets clear of %gm inflation"
            % (len(scene["patrol_targets"]), r))

    ip = scene["robot"]["initial_pose"]
    init_hit = in_inflated(ip["x"], ip["y"])
    if init_hit:
        log("init-safety", False, "initial_pose inside %s inflation" % init_hit)
        fails += 1
    elif not (x0w <= ip["x"] <= x1w and y0w <= ip["y"] <= y1w):
        log("init-safety", False, "initial_pose outside grid bounds")
        fails += 1
    else:
        log("init-safety", True, "initial_pose (%.2f,%.2f) safe" % (ip["x"], ip["y"]))

    # ---- [reachability] ----
    blocked, W, H = build_blocked_grid(scene)
    init_c = cell_of(ip["x"], ip["y"], g)
    targets_c = [(t["name"], cell_of(t["x"], t["y"], g)) for t in scene["patrol_targets"]]
    reach_ok = True
    unreach = []
    # initial -> each target
    for name, c in targets_c:
        if not bfs_reachable(blocked, W, H, init_c, c):
            reach_ok = False
            unreach.append("init->%s" % name)
    # consecutive target loop (closed patrol)
    for i in range(len(targets_c)):
        a = targets_c[i][1]
        b = targets_c[(i + 1) % len(targets_c)][1]
        if not bfs_reachable(blocked, W, H, a, b):
            reach_ok = False
            unreach.append("%s->%s" % (targets_c[i][0], targets_c[(i+1) % len(targets_c)][0]))
    if reach_ok:
        log("reachability", True, "%d targets + loop reachable (BFS, inflate %gm)"
            % (len(targets_c), r))
    else:
        log("reachability", False, ", ".join(unreach))
        fails += 1

    # ---- [ue-mirror] (P0-02 single-source-of-truth) ----
    ue_obs = parse_ue_home_obstacles(args.ue)
    if ue_obs is None:
        log("ue-mirror", False, "UE source not found: %s" % args.ue)
        fails += 1
    elif len(ue_obs) == 0:
        log("ue-mirror", True, "UE loads obstacles from JSON (no hardcoded table) -- P0-02 OK")
    else:
        json_obs = {(o["name"], round(o["xmin"], 3), round(o["ymin"], 3),
                     round(o["xmax"], 3), round(o["ymax"], 3))
                    for o in scene["obstacles"]}
        ue_set = {(n, round(a, 3), round(b, 3), round(c, 3), round(d, 3))
                  for (n, a, b, c, d) in ue_obs}
        if ue_set == json_obs:
            log("ue-mirror", True,
                "UE hardcoded %d obstacles MATCH json (note: P0-02 risk if edited independently)"
                % len(ue_obs))
        else:
            only_ue = ue_set - json_obs
            only_json = json_obs - ue_set
            log("ue-mirror", False,
                "UE<->JSON DIVERGE (%d ue / %d json). ue-only=%s json-only=%s"
                % (len(ue_obs), len(scene["obstacles"]),
                   list(only_ue)[:3], list(only_json)[:3]))
            fails += 1

    # ---- [nav-margin] (left-shift catch for §8 / §16.4) ----
    # The real UE capsule radius is 0.30 m (PuppyRobotPawn.cpp: SetCapsuleRadius(30.0f)).
    # A JSON-declared planning/CBF radius < 0.30 understates the body and yields a
    # false "safe" guarantee; and every target must be reachable with a +0.10 m
    # clearance margin so the planner can absorb the allowed localization error.
    TRUE_BODY = 0.30
    margin_issues = []
    jr = scene["robot"].get("radius_planning", 0.0)
    if jr < TRUE_BODY - 1e-6:
        margin_issues.append("radius_planning=%.2f < UE body %.2f" % (jr, TRUE_BODY))
    jc = scene["robot"].get("radius_cbf", 0.0)
    if jc < TRUE_BODY - 1e-6:
        margin_issues.append("radius_cbf=%.2f < UE body %.2f" % (jc, TRUE_BODY))
    blocked_m, Wm, Hm = build_blocked_grid_r(scene, TRUE_BODY + 0.10)
    init_cm = cell_of(ip["x"], ip["y"], g)
    tight = [name for name, c in targets_c
             if not bfs_reachable(blocked_m, Wm, Hm, init_cm, c)]
    if tight:
        margin_issues.append("targets only via sub-%.2fm corridor: %s"
                              % (TRUE_BODY + 0.10, ", ".join(tight)))
    if margin_issues:
        # Informational (§8): surfaces the risk but does not block the build.
        log("nav-margin", False, " | ".join(margin_issues))
    else:
        log("nav-margin", True,
            "body=%.2fm, all targets reachable with +0.10m clearance" % TRUE_BODY)

    # ---- [hash] ----
    raw = open(args.scene, "rb").read()
    h = hashlib.sha256(raw).hexdigest()
    log("hash", True, "sha256:%s" % h[:16])

    print("-" * 60)
    if fails == 0:
        print("RESULT: PASS  (scene is a valid single source of truth)")
        return 0
    print("RESULT: FAIL  (%d blocking issue(s))" % fails)
    return 1


if __name__ == "__main__":
    sys.exit(main())
