#!/usr/bin/env python3
# summarize_planB.py — 汇总 Plan B 长稳验收 (10 种子) 的 JSON 报告与每帧耗时 P99
# 用法:
#   python scripts/summarize_planB.py [artifacts_dir]
# 默认 artifacts_dir = 脚本所在目录的上一级的 artifacts (E:/puppyfangzhen/artifacts)
import json, csv, sys, os, glob

def p99(vals):
    if not vals:
        return 0.0
    s = sorted(vals)
    n = len(s)
    # 线性插值分位数
    idx = 0.99 * (n - 1)
    lo = int(idx)
    hi = min(lo + 1, n - 1)
    frac = idx - lo
    return s[lo] + (s[hi] - s[lo]) * frac

def analyze_timing(csv_path):
    ms = []
    try:
        with open(csv_path, newline="") as f:
            r = csv.reader(f)
            next(r, None)  # header
            for row in r:
                if len(row) >= 2:
                    try:
                        ms.append(float(row[1]))
                    except ValueError:
                        pass
    except FileNotFoundError:
        return None
    if not ms:
        return None
    ms_sorted = sorted(ms)
    n = len(ms_sorted)
    return {
        "n": n,
        "mean": sum(ms) / n,
        "p99": p99(ms),
        "over200": sum(1 for v in ms if v > 200),
    }

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    art = os.path.join(os.path.dirname(here), "artifacts")
    # argv[1] = 可选 artifacts 目录; argv[2] = 可选帧前缀 (默认 cpp_36000)
    if len(sys.argv) > 1 and os.path.isdir(sys.argv[1]):
        art = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else "cpp_36000"
    pat = os.path.join(art, f"{prefix}_seed*_planB.json")
    files = sorted(glob.glob(pat))
    if not files:
        print(f"未找到匹配 {pat} 的报告")
        sys.exit(2)
    print(f"# 汇总前缀: {prefix}  (共 {len(files)} 份报告)")
    print(f"{'seed':>4} {'status':>5} {'exit':>4} {'rawPF':>6} {'persPF':>6} {'planOK':>6} "
          f"{'coll':>4} {'rooms':>5} {'stuck%':>6} {'amclMax':>7} {'P99ms':>6} {'>200':>4} {'elaps':>5}")
    print("-" * 88)
    pass_n = 0
    total = 0
    for fp in files:
        with open(fp, encoding="utf-8") as f:
            d = json.load(f)
        total += 1
        seed = d.get("seed", "?")
        status = d.get("status", "?")
        exit_code = d.get("exit_code", -1)
        m = d.get("metrics", {})
        raw_pf = m.get("planning_failures", -1)
        pers_pf = m.get("planning_persistent_failures", -1)
        acc = d.get("acceptance", {})
        plan_ok = acc.get("planning_ok", None)
        coll = m.get("collisions", -1)
        rooms = m.get("rooms_visited", -1)
        stuck = m.get("stuck_ratio_pct", -1)
        amcl_max = m.get("amcl_max_err_m", -1)
        elaps = d.get("elapsed_sec", -1)
        # timing
        csvp = fp.replace("_planB.json", "_planB.timing.csv")
        t = analyze_timing(csvp)
        p99ms = f"{t['p99']:.1f}" if t else "  -"
        over200 = f"{t['over200']}" if t else " -"
        if status == "PASS":
            pass_n += 1
        print(f"{seed:>4} {status:>5} {exit_code:>4} {raw_pf:>6} {pers_pf:>6} "
              f"{str(plan_ok):>6} {coll:>4} {rooms:>5} {stuck:>6.2f} {amcl_max:>7.3f} "
              f"{p99ms:>6} {over200:>4} {elaps:>5}")
    print("-" * 88)
    print(f"总计: {total} 种子, 通过 {pass_n}, 失败 {total - pass_n}")
    print(f"P2 单帧 P99<=50ms 目标: {'ALL MET' if pass_n == total else 'CHECK'}")

if __name__ == "__main__":
    main()
