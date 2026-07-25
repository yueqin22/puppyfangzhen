#!/usr/bin/env python3
"""
Ablation Study Framework (v3.0)
================================
Systematic ablation experiments with statistical significance testing.

Tests the contribution of each v3.0 improvement:
  1. KLD-Sampling AMCL (Fox 2003) — USE_KLD=1/0
  2. Information-Theoretic Frontier (Bourgault 2002) — USE_INFO_THEORY=1/0
  3. TEB Time-Optimal + Jerk (Rösmann 2012) — USE_TEB=1/0
  4. AMCL vs Ground Truth — USE_AMCL=1/0

Each configuration is run N_TRIALS times with different random seeds.
Results are analyzed with paired t-tests and Cohen's d effect size.

Usage:
  python ablation_study.py run     # Run all experiments
  python ablation_study.py analyze # Analyze existing results
  python ablation_study.py all     # Run + analyze
"""
import os
import sys
import csv
import json
import math
import subprocess
import numpy as np
from pathlib import Path

# === Experiment Configuration ===
N_TRIALS = 3           # Number of repeated runs per config (for statistics)
MAX_FRAMES = 4000      # Frames per trial (~11 min wall-clock, enough for convergence)
RESULTS_DIR = Path("e:/puppyfangzhen/ablation_results")
NAV_SCRIPT = Path("e:/puppyfangzhen/autonomous_nav.py")

# Ablation configurations
# Each config is (name, description, env_dict)
CONFIGS = [
    ("proposed_v3", "Full v3.0: KLD-AMCL + TEB(time+jerk) + Info-theory frontier", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
    }),
    ("no_kld", "Ablation: Fixed-particle AMCL (no KLD-sampling)", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "0", "USE_INFO_THEORY": "1",
    }),
    ("no_info_theory", "Ablation: Count-based frontier (no Shannon entropy)", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "0",
    }),
    ("dwa_baseline", "Ablation: DWA planner (no TEB time-optimal + jerk)", {
        "USE_AMCL": "1", "USE_TEB": "0", "USE_KLD": "1", "USE_INFO_THEORY": "1",
    }),
    ("gt_baseline", "Ablation: Ground truth localization (no AMCL)", {
        "USE_AMCL": "0", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
    }),
]


def run_experiment(config_name, env_vars, trial_num):
    """Run one experiment trial.

    Args:
        config_name: name of the configuration
        env_vars: dict of environment variables
        trial_num: trial number (1-indexed)

    Returns:
        Path to the CSV data file for this trial
    """
    trial_dir = RESULTS_DIR / config_name / f"trial_{trial_num}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    # autonomous_nav.py evaluator saves to EVAL_DIR/{METHOD_NAME}_data.csv
    csv_path = trial_dir / f"{config_name}_data.csv"

    # Set environment variables
    env = os.environ.copy()
    env.update(env_vars)
    env["METHOD_NAME"] = config_name
    env["MAX_FRAMES"] = str(MAX_FRAMES)
    env["EVAL_DIR"] = str(trial_dir)
    env["PYTHONUNBUFFERED"] = "1"  # Disable stdout buffering for real-time logs

    # Set numpy random seed for reproducibility
    seed = hash(f"{config_name}_{trial_num}") % (2**32)
    env["PYTHONHASHSEED"] = str(seed)
    env["NPY_SEED"] = str(seed)

    print(f"\n{'='*60}")
    print(f"  Running: {config_name} (trial {trial_num}/{N_TRIALS})")
    print(f"  Seed: {seed}, Frames: {MAX_FRAMES}")
    print(f"  Output: {csv_path}")
    print(f"{'='*60}")

    # Run the simulation
    cmd = [sys.executable, str(NAV_SCRIPT)]
    proc = subprocess.Popen(
        cmd, env=env, cwd=str(NAV_SCRIPT.parent),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True
    )

    # Stream output
    log_path = trial_dir / "run.log"
    with open(log_path, "w") as log_f:
        for line in proc.stdout:
            log_f.write(line)
            # Print key lines only
            if any(tag in line for tag in ["[NAV]", "[STATE]", "[TELEMETRY]",
                                            "Error", "error", "Exception"]):
                print(f"  {line.rstrip()}")

    proc.wait()
    print(f"  Exit code: {proc.returncode}")

    # Evaluator saves CSV directly to EVAL_DIR/{METHOD_NAME}_data.csv
    if csv_path.exists():
        print(f"  CSV saved: {csv_path} ({csv_path.stat().st_size} bytes)")
    else:
        # Fallback: try copying from temp data_file
        temp_csv = Path(os.path.join(os.environ.get("TEMP", "/tmp"), "puppy_nav_data.csv"))
        if temp_csv.exists():
            import shutil
            shutil.copy2(temp_csv, csv_path)
            print(f"  CSV copied from temp: {csv_path}")

    # Wait between trials to let CoppeliaSim reset
    import time
    time.sleep(3)

    return csv_path


def load_trial_metrics(csv_path):
    """Load metrics from a single trial's CSV file.

    Returns a dict of summary metrics.
    """
    if not csv_path.exists():
        return None

    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                # Handle both evaluator CSV (loc_error) and data_file CSV (loc_err)
                loc_err_val = r.get("loc_err", r.get("loc_error", "0.0"))
                rows.append({
                    "frame": int(r["frame"]),
                    "coverage": float(r["coverage"]),
                    "state": int(r["state"]),
                    "loc_err": float(loc_err_val),
                    "dwa_v": float(r.get("dwa_v", 0.0)),
                })
            except (ValueError, KeyError):
                continue

    if not rows:
        return None

    # Compute summary metrics
    states = [r["state"] for r in rows]
    loc_errs = [r["loc_err"] for r in rows if r["loc_err"] > 0]
    coverages = [r["coverage"] for r in rows]

    # Time to 80% coverage
    time_to_80 = -1.0
    for r in rows:
        if r["coverage"] >= 80:
            time_to_80 = r["frame"] / 10.0  # assume 10Hz
            break

    # Time to 90% coverage
    time_to_90 = -1.0
    for r in rows:
        if r["coverage"] >= 90:
            time_to_90 = r["frame"] / 10.0
            break

    # RECOVER count (state transitions to state=2)
    recover_count = sum(1 for i in range(1, len(states))
                       if states[i] == 2 and states[i-1] != 2)

    # State distribution
    total = len(states)
    follow_pct = sum(1 for s in states if s == 1) / total * 100
    recover_pct = sum(1 for s in states if s == 2) / total * 100
    done_pct = sum(1 for s in states if s == 3) / total * 100

    # Average speed (FOLLOW state only)
    follow_speeds = [r["dwa_v"] for r in rows if r["state"] == 1]
    avg_speed = np.mean(follow_speeds) if follow_speeds else 0.0

    return {
        "final_coverage": coverages[-1],
        "max_coverage": max(coverages),
        "time_to_80": time_to_80,
        "time_to_90": time_to_90,
        "mean_loc_err": float(np.mean(loc_errs)) if loc_errs else -1.0,
        "max_loc_err": float(np.max(loc_errs)) if loc_errs else -1.0,
        "p95_loc_err": float(np.percentile(loc_errs, 95)) if loc_errs else -1.0,
        "recover_count": recover_count,
        "follow_pct": follow_pct,
        "recover_pct": recover_pct,
        "done_pct": done_pct,
        "avg_speed": avg_speed,
        "total_frames": total,
    }


def paired_t_test(a, b):
    """Compute paired t-test statistic and p-value.

    Uses numpy only (no scipy dependency).
    Returns (t_stat, p_value, degrees_of_freedom).
    """
    a, b = np.array(a), np.array(b)
    if len(a) != len(b) or len(a) < 2:
        return 0.0, 1.0, 0, False

    diff = a - b
    mean_diff = np.mean(diff)
    std_diff = np.std(diff, ddof=1)
    if std_diff < 1e-12:
        return float('inf') if mean_diff > 0 else float('-inf'), 0.0, len(a) - 1, True

    t_stat = mean_diff / (std_diff / math.sqrt(len(a)))
    df = len(a) - 1

    # Approximate p-value using normal approximation for df >= 30
    # For small df, use a simple t-distribution approximation
    # |t| > critical value at alpha=0.05 (two-tailed)
    t_crit_05 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
                 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228}
    if df in t_crit_05:
        sig = abs(t_stat) > t_crit_05[df]
    else:
        sig = abs(t_stat) > 1.96  # normal approximation

    # Simple p-value approximation
    if abs(t_stat) > 3.0:
        p_val = 0.01
    elif abs(t_stat) > 2.0:
        p_val = 0.05
    elif abs(t_stat) > 1.5:
        p_val = 0.15
    else:
        p_val = 0.30

    return float(t_stat), p_val, df, sig


def cohens_d(a, b):
    """Compute Cohen's d effect size."""
    a, b = np.array(a), np.array(b)
    pooled_std = math.sqrt((np.var(a, ddof=1) + np.var(b, ddof=1)) / 2)
    if pooled_std < 1e-12:
        return 0.0
    return float((np.mean(a) - np.mean(b)) / pooled_std)


def confidence_interval(data, confidence=0.95):
    """Compute 95% confidence interval using t-distribution."""
    data = np.array(data)
    n = len(data)
    if n < 2:
        return float(np.mean(data)), (float(np.mean(data)), float(np.mean(data)))

    mean = np.mean(data)
    sem = np.std(data, ddof=1) / math.sqrt(n)

    # t-critical for 95% CI (simplified)
    t_crit = {2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
              7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228}
    t_val = t_crit.get(n - 1, 1.96)

    margin = t_val * sem
    return float(mean), (float(mean - margin), float(mean + margin))


def analyze_results():
    """Analyze all ablation results and generate statistical comparison."""
    print("\n" + "=" * 80)
    print("  ABLATION STUDY STATISTICAL ANALYSIS")
    print("=" * 80)

    # Load all trial results
    all_results = {}
    for config_name, desc, _ in CONFIGS:
        config_dir = RESULTS_DIR / config_name
        if not config_dir.exists():
            print(f"  [SKIP] {config_name}: no results directory")
            continue

        trials = []
        for trial_dir in sorted(config_dir.iterdir()):
            if trial_dir.is_dir() and trial_dir.name.startswith("trial_"):
                # Try evaluator CSV format first, then data_file format
                csv_path = trial_dir / f"{config_name}_data.csv"
                if not csv_path.exists():
                    csv_path = trial_dir / "nav_data.csv"
                metrics = load_trial_metrics(csv_path)
                if metrics:
                    trials.append(metrics)

        if trials:
            all_results[config_name] = trials
            print(f"  [OK] {config_name}: {len(trials)} trials loaded")

    if not all_results:
        print("\n  No results found. Run 'python ablation_study.py run' first.")
        return

    # === Generate comparison table ===
    print("\n" + "-" * 80)
    print("  METRICS SUMMARY (mean ± 95% CI across trials)")
    print("-" * 80)

    metrics_keys = [
        ("final_coverage", "Coverage(%)", True),
        ("time_to_80", "T80%(s)", False),
        ("time_to_90", "T90%(s)", False),
        ("mean_loc_err", "LocErr(m)", False),
        ("max_loc_err", "MaxErr(m)", False),
        ("recover_count", "RECOVER", False),
        ("follow_pct", "Follow(%)", True),
        ("avg_speed", "Speed(m/s)", True),
    ]

    # Print table header
    config_names = list(all_results.keys())
    header = f"{'Metric':<15s}"
    for name in config_names:
        header += f" {name:>16s}"
    print(header)
    print("-" * (15 + 17 * len(config_names)))

    # Print each metric row
    metric_values = {}  # metric_name -> {config -> [values]}
    for key, label, higher_better in metrics_keys:
        row = f"{label:<15s}"
        metric_values[key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config] if t[key] >= 0]
            if not values:
                row += f" {'N/A':>16s}"
                continue
            mean, ci = confidence_interval(values)
            metric_values[key][config] = values
            row += f" {mean:>7.2f}±{((ci[1]-ci[0])/2):>5.2f}"
        print(row)

    # === Statistical significance tests ===
    print("\n" + "-" * 80)
    print("  STATISTICAL SIGNIFICANCE (proposed_v3 vs each ablation)")
    print("-" * 80)

    if "proposed_v3" not in all_results:
        print("  [ERROR] proposed_v3 results not found")
        return

    proposed = all_results["proposed_v3"]
    print(f"\n  {'Metric':<15s} {'Ablation':<20s} {'t-stat':>8s} {'p-val':>8s} "
          f"{'Cohen d':>8s} {'Sig?':>6s}")
    print("  " + "-" * 70)

    for key, label, higher_better in metrics_keys:
        if key not in metric_values or "proposed_v3" not in metric_values[key]:
            continue
        prop_vals = metric_values[key]["proposed_v3"]
        if len(prop_vals) < 2:
            continue

        for config in config_names:
            if config == "proposed_v3":
                continue
            if config not in metric_values.get(key, {}):
                continue
            abl_vals = metric_values[key][config]
            if len(abl_vals) < 2:
                continue

            # Align trial counts (use min length)
            min_len = min(len(prop_vals), len(abl_vals))
            if min_len < 2:
                continue
            p_vals = prop_vals[:min_len]
            a_vals = abl_vals[:min_len]

            t_stat, p_val, df, sig = paired_t_test(p_vals, a_vals)
            d = cohens_d(p_vals, a_vals)

            sig_str = "***" if sig and abs(d) > 0.8 else ("**" if sig else "ns")
            print(f"  {label:<15s} {config:<20s} {t_stat:>8.3f} {p_val:>8.3f} "
                  f"{d:>8.3f} {sig_str:>6s}")

    print("\n  Legend: *** = significant (p<0.05, large effect), "
          "** = significant, ns = not significant")
    print("  Cohen's d: >0.8 large, >0.5 medium, >0.2 small effect")

    # === Save results to JSON ===
    summary_path = RESULTS_DIR / "ablation_summary.json"
    summary = {}
    for config, trials in all_results.items():
        summary[config] = {
            "n_trials": len(trials),
            "metrics": {}
        }
        for key, label, _ in metrics_keys:
            values = [t[key] for t in trials if t[key] >= 0]
            if values:
                mean, ci = confidence_interval(values)
                summary[config]["metrics"][key] = {
                    "mean": mean,
                    "ci_low": ci[0],
                    "ci_high": ci[1],
                    "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                    "values": values,
                }

    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n  Summary saved to: {summary_path}")


def run_all():
    """Run all ablation experiments."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    total_runs = len(CONFIGS) * N_TRIALS
    current = 0

    for config_name, desc, env_vars in CONFIGS:
        for trial in range(1, N_TRIALS + 1):
            current += 1
            print(f"\n[{current}/{total_runs}] {desc}")
            run_experiment(config_name, env_vars, trial)

    print(f"\n{'='*60}")
    print(f"  All {total_runs} experiments completed!")
    print(f"{'='*60}")

    analyze_results()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python ablation_study.py [run|analyze|all]")
        print("  run     - Run all ablation experiments")
        print("  analyze - Analyze existing results")
        print("  all     - Run + analyze")
        sys.exit(1)

    cmd = sys.argv[1]
    if cmd == "run":
        run_all()
    elif cmd == "analyze":
        analyze_results()
    elif cmd == "all":
        run_all()
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)
