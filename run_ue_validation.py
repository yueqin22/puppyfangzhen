#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UE GPU validation orchestrator for the P0 navigation fixes (2026-08-13), v2.

CORRECT launch order (discovered during v1 diagnosis):
  - The BRIDGE is the TCP SERVER (binds/listens/accepts on 127.0.0.1:7777).
  - The UE pawn (PuppyTcpServer) is the TCP CLIENT; it calls Connect() at
    BeginPlay. If no server is listening at that moment, the non-blocking
    connect gets stuck pending/error and the pawn never cleanly connects,
    so it never sends LIDAR -> bridge recv_frame() blocks -> deadlock.

Therefore we MUST start the bridge (server) FIRST, then UE (client). The
pawn's first Connect() then succeeds immediately and lockstep proceeds.

Flow:
  1. Kill any leftover UnrealEditor / nav_ue_bridge (free port 7777).
  2. Launch the bridge (server) with --frames so it self-exits; it blocks
     on accept() until UE connects.
  3. Launch UE in -game mode (client).
  4. Wait for the bridge process to exit (frame budget reached).
  5. Tear down UE.
  6. Full logs go to *_val.log; analysis is done afterwards.
"""
import subprocess
import sys
import time
import os
import argparse

UE_EXE = r"C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe"
UE_PROJECT = r"D:\puppy_ue\puppy_ue.uproject"
BRIDGE_EXE = r"E:\puppyfangzhen\cpp_src\bridge\build\Release\nav_ue_bridge.exe"
SCENE = r"E:\puppyfangzhen\config\scene_home.json"
PORT = 7777
FRAMES = 3600          # 3600 lockstep frames @30Hz ~= 120 s of simulation
BRIDGE_TIMEOUT = 420   # max seconds to wait for the bridge to finish
UE_LOG = r"E:\puppyfangzhen\ue_val.log"
BRIDGE_LOG = r"E:\puppyfangzhen\bridge_val.log"
# R27: per-frame CSV trace. Aggregate health numbers (avg_err etc.) cannot tell
# us *where* the estimate drifts; this file can.
TRACE_CSV = r"E:\puppyfangzhen\trace_val.csv"


def kill_leftovers():
    for name in ("UnrealEditor.exe", "nav_ue_bridge.exe"):
        try:
            subprocess.run(["taskkill", "/F", "/IM", name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=30)
        except Exception:
            pass
    time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inject-divergence-at", type=int, default=-1,
                    help="Fault-injection: simulate a +3m localization jump at "
                         "this frame to verify divergence recovery (test only).")
    ap.add_argument("--frames", type=int, default=FRAMES)
    args = ap.parse_args()

    inject = ["--inject-divergence-at", str(args.inject_divergence_at)] \
        if args.inject_divergence_at > 0 else []
    kill_leftovers()
    print("[ORCH v2] launching BRIDGE (TCP server) first ...")
    with open(BRIDGE_LOG, "w") as bf:
        br = subprocess.Popen(
            [BRIDGE_EXE, "--port", str(PORT), "--frames", str(args.frames),
             "--scene", SCENE, "--trace", TRACE_CSV] + inject,
            stdout=bf, stderr=subprocess.STDOUT)

    # Give the bridge a moment to bind+listen before UE (client) starts.
    time.sleep(3)
    print("[ORCH v2] launching UE (-game, TCP client) ...")
    ue = subprocess.Popen(
        [UE_EXE, UE_PROJECT, "-game", "-map=/Game/Maps/HomeMap",
         "-SceneConfig=" + SCENE,
         "-windowed", "-resx=1280", "-resy=720", "-log"],
        stdout=open(UE_LOG, "w"), stderr=subprocess.STDOUT,
        close_fds=True)

    print("[ORCH v2] waiting for bridge to finish (<=%.0fs) ..." % BRIDGE_TIMEOUT)
    try:
        rc = br.wait(timeout=BRIDGE_TIMEOUT)
        print("[ORCH v2] bridge exited code=%s" % rc)
    except subprocess.TimeoutExpired:
        print("[ORCH v2] bridge did not finish in %.0fs; terminating." % BRIDGE_TIMEOUT)
        br.terminate()
        try:
            br.wait(timeout=30)
        except subprocess.TimeoutExpired:
            br.kill()

    # Tear down UE
    print("[ORCH v2] terminating UE ...")
    try:
        ue.terminate()
        ue.wait(timeout=30)
    except Exception:
        try:
            ue.kill()
        except Exception:
            pass
    print("[ORCH v2] done. UE_LOG=%s BRIDGE_LOG=%s" % (UE_LOG, BRIDGE_LOG))
    if os.path.exists(TRACE_CSV):
        with open(TRACE_CSV) as f:
            n = sum(1 for _ in f) - 1
        print("[ORCH v2] TRACE_CSV=%s (%d frame rows)" % (TRACE_CSV, n))
    else:
        print("[ORCH v2] WARNING: no trace file produced at %s" % TRACE_CSV)


if __name__ == "__main__":
    main()
