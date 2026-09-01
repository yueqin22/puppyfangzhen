#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulation Screenshot & Diagnostic Runner
==========================================
Runs visual_sim.py in headless mode across multiple obstacle avoidance algorithms,
captures screenshots at critical moments, records quantitative metrics,
and analyzes bugs/issues in the system.
"""
import os
import sys
import time
import json
import numpy as np

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'

import pygame
from visual_sim import VisualSimulator, ALGORITHMS

def run_diagnostics():
    print("Starting visual simulation diagnostic run...")
    sim = VisualSimulator()

    results = {}
    screenshots = []

    # Run tests on key algorithms: Baseline, VO, STVOC, RVO, CBF, A*+CBF
    test_algos = [0, 1, 4, 5, 8, 9] # Baseline, STVOC, VO, RVO, CBF, A*+CBF

    for algo_idx in test_algos:
        algo_name = ALGORITHMS[algo_idx]
        print(f"\n--- Testing Algorithm: {algo_name} (idx={algo_idx}) ---")
        sim.reset()
        sim.current_algo_idx = algo_idx
        sim.current_algo = algo_name

        # Run for 600 frames (20 seconds simulated time)
        for frame in range(600):
            sim.step()
            sim.draw()

            # Capture screenshot at frame 300 (midway) and 600 (end)
            if frame == 300 or frame == 599:
                safe_algo_name = algo_name.replace('*', '_star_')
                img_filename = f"sim_snap_algo_{algo_idx}_{safe_algo_name}_f{frame}.png"
                img_path = os.path.abspath(img_filename)
                pygame.image.save(sim.screen, img_path)
                screenshots.append(img_path)
                print(f"Captured screenshot: {img_filename}")

        metrics = {
            'algorithm': algo_name,
            'rounds_completed': sim.rounds_completed,
            'total_collisions': sim.total_collisions,
            'near_miss': sim.near_miss,
            'skip_count': sim.skip_count,
            'stall_events': sim.stall_events,
            'robot_position': [round(sim.sim.robot_x, 3), round(sim.sim.robot_y, 3)],
            'target_idx': sim.target_idx,
        }
        results[algo_name] = metrics
        print(f"Metrics for {algo_name}: {json.dumps(metrics, ensure_ascii=False)}")

    # Save diagnostic summary
    summary_path = os.path.abspath("visual_sim_diagnostic_report.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump({
            'timestamp': time.time(),
            'results': results,
            'screenshots': screenshots
        }, f, indent=2, ensure_ascii=False)

    print(f"\nDiagnostic run complete. Report saved to {summary_path}")
    return results, screenshots

if __name__ == '__main__':
    run_diagnostics()
