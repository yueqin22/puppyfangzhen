#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bundle one validation run into the artifact layout required by
guihua20260812.md section 10.3.

Produces:

    artifacts/<run_id>/
    |-- manifest.json        (provenance + conclusion, section 10.3 field list)
    |-- metrics.json         (delegated to analyze_acceptance.py)
    |-- summary.md           (human-readable verdict)
    |-- bridge.log
    |-- ue.log
    |-- trajectory.csv       (pose columns split out of the raw trace)
    |-- diagnostics.csv      (planner / recovery columns)
    |-- screenshots/
    |-- config_snapshot/     (scene + tuning yaml actually used)
    +-- environment.json     (OS / compiler / UE / python versions)

Usage:
    python scripts/bundle_artifacts.py --run-id r29-stuckfix
"""
import argparse
import csv
import datetime
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Pose columns go to trajectory.csv; everything else (plus `frame`) to
# diagnostics.csv.  Keeping `frame` in both makes each file joinable alone.
TRAJ_COLS = ["frame", "t", "true_x", "true_y", "true_yaw",
             "est_x", "est_y", "est_yaw", "err", "conf"]

CONFIG_FILES = [
    "config/scene_home.json",
    "config/sim_cpp.yaml",
    "config/planner_global.yaml",
    "config/planner_local.yaml",
    "config/runtime.yaml",
]


def sha256_file(path):
    if not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit():
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                             capture_output=True, text=True, timeout=15)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return None


def git_dirty():
    """A dirty tree means the artifact is NOT reproducible from the commit."""
    try:
        out = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                             capture_output=True, text=True, timeout=20)
        if out.returncode == 0:
            return bool(out.stdout.strip())
    except Exception:
        pass
    return None


def msvc_version():
    """Read the compiler stamp CMake recorded, so we don't need a VS shell."""
    cache = os.path.join(ROOT, "cpp_src", "bridge", "build", "CMakeCache.txt")
    if not os.path.exists(cache):
        return None
    want = ("CMAKE_CXX_COMPILER:", "CMAKE_CXX_COMPILER_VERSION:")
    found = {}
    with open(cache, encoding="utf-8", errors="replace") as f:
        for line in f:
            for w in want:
                if line.startswith(w):
                    found[w.rstrip(":")] = line.split("=", 1)[1].strip()
    return found or None


def ue_version(ue_exe):
    """Infer the engine version from the install path (UE_5.3 -> 5.3)."""
    for part in os.path.normpath(ue_exe or "").split(os.sep):
        if part.startswith("UE_"):
            return part[3:]
    return None


def split_trace(trace_path, traj_out, diag_out):
    """Split the raw per-frame trace into trajectory + diagnostics CSVs."""
    if not os.path.exists(trace_path) or os.path.getsize(trace_path) == 0:
        return 0
    with open(trace_path, newline="", encoding="utf-8", errors="replace") as f:
        rdr = csv.reader(f)
        try:
            header = next(rdr)
        except StopIteration:
            return 0
        traj_idx = [i for i, c in enumerate(header) if c in TRAJ_COLS]
        # `frame` is duplicated into diagnostics so the file stands alone.
        diag_idx = [i for i, c in enumerate(header)
                    if c not in TRAJ_COLS or c == "frame"]
        n = 0
        with open(traj_out, "w", newline="", encoding="utf-8") as tf, \
             open(diag_out, "w", newline="", encoding="utf-8") as df:
            tw, dw = csv.writer(tf), csv.writer(df)
            tw.writerow([header[i] for i in traj_idx])
            dw.writerow([header[i] for i in diag_idx])
            for row in rdr:
                if len(row) < len(header):
                    continue          # truncated final line on a hard kill
                tw.writerow([row[i] for i in traj_idx])
                dw.writerow([row[i] for i in diag_idx])
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--bridge-log", default=os.path.join(ROOT, "bridge_val.log"))
    ap.add_argument("--ue-log", default=os.path.join(ROOT, "ue_val.log"))
    ap.add_argument("--trace", default=os.path.join(ROOT, "trace_val.csv"))
    ap.add_argument("--scene", default=os.path.join(ROOT, "config", "scene_home.json"))
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--profile", default="default")
    ap.add_argument("--launch-cmd", default="python run_ue_validation.py --frames 3600")
    ap.add_argument("--ue-exe",
                    default=r"C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe")
    args = ap.parse_args()

    out_dir = os.path.join(ROOT, "artifacts", args.run_id)
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(os.path.join(out_dir, "screenshots"), exist_ok=True)
    cfg_dir = os.path.join(out_dir, "config_snapshot")
    os.makedirs(cfg_dir, exist_ok=True)

    # ---- logs -------------------------------------------------------------
    for src, dst in ((args.bridge_log, "bridge.log"), (args.ue_log, "ue.log")):
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(out_dir, dst))

    # ---- trace split ------------------------------------------------------
    rows = split_trace(args.trace,
                       os.path.join(out_dir, "trajectory.csv"),
                       os.path.join(out_dir, "diagnostics.csv"))

    # ---- config snapshot --------------------------------------------------
    snap = {}
    for rel in CONFIG_FILES:
        src = os.path.join(ROOT, rel)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(cfg_dir, os.path.basename(rel)))
            snap[rel] = sha256_file(src)

    # ---- metrics (delegate to the acceptance gate) -------------------------
    py = sys.executable
    gate = subprocess.run(
        [py, os.path.join(ROOT, "scripts", "analyze_acceptance.py"),
         "--bridge-log", args.bridge_log, "--trace", args.trace,
         "--run-id", args.run_id, "--out", out_dir],
        capture_output=True, text=True)
    gate_txt = gate.stdout + gate.stderr
    metrics = {}
    mpath = os.path.join(out_dir, "metrics.json")
    if os.path.exists(mpath):
        with open(mpath, encoding="utf-8") as f:
            metrics = json.load(f)

    overall = metrics.get("overall_pass")
    checks = metrics.get("checks", [])
    failed = [c for c in checks if not c.get("pass")]

    # ---- environment.json -------------------------------------------------
    env = {
        "os": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "python": platform.python_version(),
        "compiler": msvc_version(),
        "ue_version": ue_version(args.ue_exe),
        "ros2_version": None,   # not used on the Windows/UE validation path
        "cmake_generator": "Ninja",
        "captured_at": datetime.datetime.now().astimezone().isoformat(),
    }
    with open(os.path.join(out_dir, "environment.json"), "w",
              encoding="utf-8") as f:
        json.dump(env, f, indent=2, ensure_ascii=False)

    # ---- manifest.json (section 10.3 required fields) ---------------------
    manifest = {
        "run_id": args.run_id,
        "git_commit": git_commit(),
        "git_dirty": git_dirty(),
        "scene": {
            "path": os.path.relpath(args.scene, ROOT).replace("\\", "/"),
            "sha256": sha256_file(args.scene),
            "version": None,
        },
        "robot_config_version": None,
        "algorithm_profile": args.profile,
        "random_seed": args.seed,
        "environment": env,
        "launch_command": args.launch_cmd,
        "end_reason": None,
        "conclusion": {
            "overall_pass": overall,
            "failed_checks": [c["metric"] for c in failed],
        },
        "config_snapshot_sha256": snap,
        "trace_rows": rows,
        "created_at": datetime.datetime.now().astimezone().isoformat(),
    }

    # scene version + end reason are best-effort reads from real data
    try:
        with open(args.scene, encoding="utf-8") as f:
            manifest["scene"]["version"] = json.load(f).get("version")
    except Exception:
        pass
    bl = os.path.join(out_dir, "bridge.log")
    if os.path.exists(bl):
        with open(bl, encoding="utf-8", errors="replace") as f:
            tail = f.read()[-4000:]
        for token in ("SIM_END", "frame budget reached", "client disconnected",
                      "SUMMARY:"):
            if token in tail:
                manifest["end_reason"] = token
                break

    with open(os.path.join(out_dir, "manifest.json"), "w",
              encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    # ---- summary.md -------------------------------------------------------
    s = metrics.get("summary", {})
    lines = [
        "# Validation run `%s`" % args.run_id,
        "",
        "- verdict: **%s**" % ("ACCEPT" if overall else "NEEDS WORK"),
        "- commit: `%s`%s" % (manifest["git_commit"],
                              " (dirty tree)" if manifest["git_dirty"] else ""),
        "- scene: `%s` sha256 `%s`" % (manifest["scene"]["path"],
                                       (manifest["scene"]["sha256"] or "")[:16]),
        "- frames: %s | end reason: `%s`" % (s.get("frames"),
                                             manifest["end_reason"]),
        "- trace rows: %d" % rows,
        "",
        "## Acceptance checks (plan section 16)",
        "",
        "| metric | value | need | result |",
        "|---|---|---|---|",
    ]
    for c in checks:
        lines.append("| %s | %s | %s %s | %s |"
                     % (c["metric"], c["value"], c["op"], c["limit"],
                        "PASS" if c["pass"] else "**FAIL**"))
    if failed:
        lines += ["", "## Open defects", ""]
        for c in failed:
            lines.append("- `%s` = %s (need %s %s) — %s"
                         % (c["metric"], c["value"], c["op"], c["limit"],
                            c["desc"]))
    lines += ["", "## Raw gate output", "", "```", gate_txt.strip(), "```", ""]
    with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print("[bundle] artifacts -> %s" % out_dir)
    for name in sorted(os.listdir(out_dir)):
        p = os.path.join(out_dir, name)
        mark = "/" if os.path.isdir(p) else ""
        size = "" if os.path.isdir(p) else " (%d B)" % os.path.getsize(p)
        print("   %s%s%s" % (name, mark, size))
    print("[bundle] verdict: %s" % ("ACCEPT" if overall else "NEEDS WORK"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
