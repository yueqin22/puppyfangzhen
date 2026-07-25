#!/usr/bin/env python3
"""长时间仿真数据分析 (v9.0)

分析项目:
  1. 穿墙检测 — 检查真实位置是否进入障碍物bounding box
  2. 传送检测 — 帧间位置突变 > 1m
  3. AMCL漂移 — loc_err时间序列分析
  4. 状态机转换 — 统计PLAN/FOLLOW/RECOVER/DONE分布
  5. 恢复事件 — 统计RECOVER次数和原因
  6. 性能指标 — 帧率/速度/覆盖率变化
  7. 窄通道穿越 — 检测y=0交叉事件
  8. 异常速度 — 速度过大或震荡
  9. 路径效率 — 直线距离/实际距离
  10. 障碍物交互 — 接近障碍物的频率

用法:
  python analyze_long_run.py [csv_path] [log_path]

不指定路径时使用默认路径。
"""
import csv
import math
import os
import sys
import re
import json
from collections import defaultdict, Counter


# 障碍物bounding box (从CoppeliaSim场景中硬编码)
OBSTACLES_BBOX = [
    ('wall_south',    -5.00, -4.05, 5.00, -3.95),
    ('wall_north',    -5.00,  3.95, 5.00,  4.05),
    ('wall_west',     -5.05, -4.00, -4.95, 4.00),
    ('wall_east',      4.95, -4.00,  5.05, 4.00),
    ('wall_divide_1', -5.00, -0.05, -1.00, 0.05),
    ('wall_divide_2',  1.00, -0.05,  5.00, 0.05),
    ('wall_bedroom_1',-5.05,  0.00, -4.95, 4.00),
    ('wall_kitchen_1', 4.95,  0.00,  5.05, 4.00),
    ('sofa',           2.75, -3.30, 4.25, -2.70),
    ('bed',           -4.25,  2.00, -2.75, 4.00),
    ('dining_table',   1.60,  2.47, 2.40,  2.53),
]

ROBOT_RADIUS = 0.35  # 机器人碰撞半径


def load_csv(csv_path):
    """加载CSV数据"""
    rows = []
    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for r in reader:
                for k in ['frame', 'time', 'rx', 'ry', 'ryaw', 'coverage',
                          'visited', 'cost_at_robot', 'dwa_v', 'dwa_w',
                          'no_progress', 'goal_x', 'goal_y', 'dist_to_goal',
                          'path_len', 'true_x', 'true_y', 'loc_err']:
                    if k in r and r[k]:
                        try:
                            r[k] = float(r[k])
                        except ValueError:
                            r[k] = 0.0
                if 'frame' in r:
                    r['frame'] = int(r['frame'])
                rows.append(r)
    except Exception as e:
        print(f"加载CSV失败: {e}")
    return rows


def parse_log(log_path):
    """从日志文件中提取关键事件"""
    events = {
        'state_transitions': [],
        'recover_events': [],
        'doorway_crossings': [],
        'narrow_passages': [],
        'amcl_recover': [],
        'plan_frontiers': [],
        'errors': [],
    }
    if not os.path.exists(log_path):
        return events

    try:
        with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                # 状态转换
                m = re.search(r'\[STATE\] (\w+)\s*->\s*(\w+)\s+at\s+\(([-\d.]+),([-\d.]+)\)\s+cov=([\d.]+)%', line)
                if m:
                    events['state_transitions'].append({
                        'from': m.group(1), 'to': m.group(2),
                        'x': float(m.group(3)), 'y': float(m.group(4)),
                        'cov': float(m.group(5)),
                    })
                # RECOVER事件
                m = re.search(r'FOLLOW.*RECOVER|PLAN.*RECOVER', line)
                if m and '-> RECOVER' in line:
                    frame_m = re.search(r'frame\s*(\d+)', line, re.IGNORECASE) or re.search(r'frames?=\s*(\d+)', line, re.IGNORECASE)
                    reason = 'unknown'
                    if 'no_progress' in line:
                        reason = 'no_progress'
                    elif 'no_frontier' in line or 'No reachable' in line:
                        reason = 'no_frontier'
                    events['recover_events'].append({
                        'reason': reason,
                        'line': line.strip()[:200],
                    })
                # AMCL恢复
                m = re.search(r'\[AMCL-RECOVER\].*?loc_err=([\d.]+)', line)
                if m:
                    events['amcl_recover'].append({
                        'loc_err': float(m.group(1)),
                        'line': line.strip()[:200],
                    })
                # 门道穿越
                if '[DOORWAY]' in line:
                    events['doorway_crossings'].append(line.strip()[:200])
                # 窄通道
                if '[NARROW]' in line:
                    events['narrow_passages'].append(line.strip()[:200])
                # PLAN frontier
                m = re.search(r'\[PLAN\] Frontier\s+\(([-\d.]+),([-\d.]+)\)\s+info_gain=([\d.]+).*?dist=([\d.]+)m.*?path_q=([\d.]+)', line)
                if m:
                    events['plan_frontiers'].append({
                        'fx': float(m.group(1)), 'fy': float(m.group(2)),
                        'info_gain': float(m.group(3)),
                        'distance': float(m.group(4)),
                        'path_quality': float(m.group(5)),
                    })
                # 错误
                if '[WARN]' in line and 'error' in line.lower():
                    events['errors'].append(line.strip()[:200])
    except Exception as e:
        print(f"解析日志失败: {e}")
    return events


def check_wall_clipping(rows):
    """1. 穿墙检测 — 真实位置是否在障碍物内"""
    print("\n" + "="*70)
    print("1. 穿墙检测 (Wall Clipping)")
    print("="*70)
    print(f"   检查标准: 真实位置进入障碍物bbox + 机器人半径({ROBOT_RADIUS}m)")

    clips = []
    near_misses = []  # 接近但未穿入
    real_clips = []  # 真正穿入障碍物中心
    edge_contacts = []  # 仅边缘接触（机器人半径与bbox边缘）
    for r in rows:
        tx = r.get('true_x', 0)
        ty = r.get('true_y', 0)
        frame = r.get('frame', 0)
        for name, xmin, ymin, xmax, ymax in OBSTACLES_BBOX:
            # 完全穿入 (中心点在bbox内)
            if xmin < tx < xmax and ymin < ty < ymax:
                real_clips.append({
                    'frame': frame, 'x': tx, 'y': ty,
                    'obstacle': name, 'type': 'center_inside',
                    'bbox': (xmin, ymin, xmax, ymax),
                })
            # 碰撞检测 (机器人半径与bbox相交)
            elif (tx > xmin - ROBOT_RADIUS and tx < xmax + ROBOT_RADIUS and
                  ty > ymin - ROBOT_RADIUS and ty < ymax + ROBOT_RADIUS):
                edge_contacts.append({
                    'frame': frame, 'x': tx, 'y': ty,
                    'obstacle': name, 'type': 'radius_collision',
                    'bbox': (xmin, ymin, xmax, ymax),
                })
            # 近距离 (0.5m内)
            elif (tx > xmin - 0.5 and tx < xmax + 0.5 and
                  ty > ymin - 0.5 and ty < ymax + 0.5):
                near_misses.append({
                    'frame': frame, 'x': tx, 'y': ty,
                    'obstacle': name,
                })

    print(f"   真穿墙 (中心在障碍物内): {len(real_clips)}")
    print(f"   边缘接触 (半径碰撞): {len(edge_contacts)}")
    print(f"   近距离接触 (<0.5m): {len(near_misses)}")

    if real_clips:
        print("\n   *** 真穿墙详情 (前20个) ***")
        for c in real_clips[:20]:
            print(f"   帧{c['frame']:5d}: ({c['x']:.2f},{c['y']:.2f}) "
                  f"穿入 {c['obstacle']} bbox={c['bbox']}")
        by_obs = Counter(c['obstacle'] for c in real_clips)
        print("\n   按障碍物统计:")
        for obs, cnt in by_obs.most_common():
            print(f"     {obs}: {cnt}次")

    if edge_contacts:
        print(f"\n   *** 边缘接触详情 (前10个) ***")
        for c in edge_contacts[:10]:
            print(f"   帧{c['frame']:5d}: ({c['x']:.2f},{c['y']:.2f}) "
                  f"接触 {c['obstacle']} bbox={c['bbox']}")
        by_obs = Counter(c['obstacle'] for c in edge_contacts)
        print("\n   按障碍物统计:")
        for obs, cnt in by_obs.most_common():
            print(f"     {obs}: {cnt}次")

    return real_clips, edge_contacts, near_misses


def check_teleportation(rows):
    """2. 传送检测 — 帧间位置突变"""
    print("\n" + "="*70)
    print("2. 传送检测 (Teleportation)")
    print("="*70)
    print("   检查标准: 连续帧间位置变化 > 1.0m (正常步长<0.3m)")

    teleports = []
    for i in range(1, len(rows)):
        dx = rows[i].get('true_x', 0) - rows[i-1].get('true_x', 0)
        dy = rows[i].get('true_y', 0) - rows[i-1].get('true_y', 0)
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 1.0:
            teleports.append({
                'frame': rows[i].get('frame', i),
                'distance': dist,
                'from': (rows[i-1].get('true_x', 0), rows[i-1].get('true_y', 0)),
                'to': (rows[i].get('true_x', 0), rows[i].get('true_y', 0)),
            })

    print(f"   传送事件: {len(teleports)}")
    if teleports:
        for t in teleports[:10]:
            print(f"   帧{t['frame']:5d}: 跳变 {t['distance']:.2f}m "
                  f"({t['from'][0]:.1f},{t['from'][1]:.1f}) -> "
                  f"({t['to'][0]:.1f},{t['to'][1]:.1f})")
        # 统计最大跳变
        max_t = max(teleports, key=lambda x: x['distance'])
        print(f"\n   最大跳变: {max_t['distance']:.2f}m (帧{max_t['frame']})")
    else:
        print("   ✓ 未检测到异常传送")

    return teleports


def analyze_amcl_drift(rows):
    """3. AMCL漂移分析"""
    print("\n" + "="*70)
    print("3. AMCL漂移分析")
    print("="*70)

    if not rows or 'loc_err' not in rows[0]:
        print("   数据无loc_err字段")
        return

    loc_errs = [r.get('loc_err', 0) for r in rows]
    n = len(loc_errs)

    avg = sum(loc_errs) / n
    mx = max(loc_errs)
    p95 = sorted(loc_errs)[int(n * 0.95)]
    p99 = sorted(loc_errs)[int(n * 0.99)]

    print(f"   总帧数: {n}")
    print(f"   平均: {avg:.4f}m")
    print(f"   P95:  {p95:.4f}m")
    print(f"   P99:  {p99:.4f}m")
    print(f"   最大: {mx:.4f}m")

    # 按时间段分析
    chunk_size = max(1000, n // 10)
    print(f"\n   时间分段分析 (每{chunk_size}帧):")
    print(f"   {'帧范围':<20} {'平均':<10} {'P95':<10} {'最大':<10}")
    for start in range(0, n, chunk_size):
        chunk = loc_errs[start:start+chunk_size]
        if not chunk:
            break
        c_avg = sum(chunk) / len(chunk)
        c_p95 = sorted(chunk)[int(len(chunk) * 0.95)]
        c_max = max(chunk)
        f0 = rows[start].get('frame', start)
        f1 = rows[min(start+chunk_size-1, n-1)].get('frame', start+chunk_size-1)
        print(f"   {f0:6d}-{f1:6d}    {c_avg:<10.4f} {c_p95:<10.4f} {c_max:<10.4f}")

    # 漂移分类
    drift_05 = sum(1 for e in loc_errs if e > 0.5)
    drift_1 = sum(1 for e in loc_errs if e > 1.0)
    drift_2 = sum(1 for e in loc_errs if e > 2.0)
    print(f"\n   漂移分布:")
    print(f"     loc_err > 0.5m: {drift_05}帧 ({drift_05/n*100:.1f}%)")
    print(f"     loc_err > 1.0m: {drift_1}帧 ({drift_1/n*100:.1f}%)")
    print(f"     loc_err > 2.0m: {drift_2}帧 ({drift_2/n*100:.1f}%)")

    # 持续高漂移检测 (>1m持续超过30帧)
    persistent_drift = []
    drift_start = None
    for i, e in enumerate(loc_errs):
        if e > 1.0:
            if drift_start is None:
                drift_start = i
        else:
            if drift_start is not None and (i - drift_start) >= 30:
                persistent_drift.append({
                    'start': rows[drift_start].get('frame', drift_start),
                    'end': rows[i-1].get('frame', i-1),
                    'duration': i - drift_start,
                    'max_err': max(loc_errs[drift_start:i]),
                })
            drift_start = None
    if drift_start is not None and (n - drift_start) >= 30:
        persistent_drift.append({
            'start': rows[drift_start].get('frame', drift_start),
            'end': rows[n-1].get('frame', n-1),
            'duration': n - drift_start,
            'max_err': max(loc_errs[drift_start:]),
        })

    print(f"\n   持续高漂移事件 (loc_err>1m持续≥30帧): {len(persistent_drift)}")
    for p in persistent_drift[:5]:
        print(f"     帧{p['start']:5d}-{p['end']:5d} "
              f"(持续{p['duration']}帧, 最大err={p['max_err']:.3f}m)")


def analyze_state_machine(rows, events):
    """4. 状态机分析"""
    print("\n" + "="*70)
    print("4. 状态机分析")
    print("="*70)

    transitions = events['state_transitions']
    print(f"   状态转换总数: {len(transitions)}")

    # 统计状态分布 (从日志转换推算)
    state_counts = Counter()
    for t in transitions:
        state_counts[t['to']] += 1
    print(f"   转换目标分布:")
    for s, cnt in state_counts.most_common():
        print(f"     {s}: {cnt}次")

    # RECOVER分析
    recovers = events['recover_events']
    print(f"\n   RECOVER事件: {len(recovers)}")
    recover_reasons = Counter(r['reason'] for r in recovers)
    for reason, cnt in recover_reasons.most_common():
        print(f"     {reason}: {cnt}次")

    # AMCL恢复
    amcl_recovers = events['amcl_recover']
    print(f"\n   AMCL恢复事件: {len(amcl_recovers)}")
    if amcl_recovers:
        amcl_errs = [r['loc_err'] for r in amcl_recovers]
        print(f"     恢复前loc_err: 平均={sum(amcl_errs)/len(amcl_errs):.3f}m "
              f"最大={max(amcl_errs):.3f}m")


def analyze_doorway(rows, events):
    """5. 门道穿越分析"""
    print("\n" + "="*70)
    print("5. 门道穿越分析")
    print("="*70)

    crossings = events['doorway_crossings']
    print(f"   门道穿越次数: {len(crossings)}")

    # 从CSV数据检测y=0穿越
    y_crossings = []
    for i in range(1, len(rows)):
        prev_y = rows[i-1].get('true_y', 0)
        curr_y = rows[i].get('true_y', 0)
        if (prev_y < 0 and curr_y >= 0) or (prev_y >= 0 and curr_y < 0):
            y_crossings.append({
                'frame': rows[i].get('frame', i),
                'prev_y': prev_y,
                'curr_y': curr_y,
            })
    print(f"   y=0穿越次数 (CSV数据): {len(y_crossings)}")

    # 检测震荡 (短时间内多次穿越)
    oscillations = []
    if len(y_crossings) > 2:
        for i in range(2, len(y_crossings)):
            frame_diff = y_crossings[i]['frame'] - y_crossings[i-2]['frame']
            if frame_diff < 40:  # 40帧内3次穿越
                oscillations.append({
                    'start_frame': y_crossings[i-2]['frame'],
                    'end_frame': y_crossings[i]['frame'],
                    'count': 3,
                })
    print(f"   门道震荡 (40帧内≥3次穿越): {len(oscillations)}")
    for o in oscillations[:5]:
        print(f"     帧{o['start_frame']:5d}-{o['end_frame']:5d}")

    # 窄通道事件
    narrow = events['narrow_passages']
    print(f"\n   窄通道事件: {len(narrow)}")
    for n in narrow[:5]:
        print(f"     {n}")


def analyze_velocity(rows):
    """6. 速度分析"""
    print("\n" + "="*70)
    print("6. 速度分析")
    print("="*70)

    if not rows or 'dwa_v' not in rows[0]:
        print("   数据无速度字段")
        return

    velocities = [r.get('dwa_v', 0) for r in rows]
    omegas = [r.get('dwa_w', 0) for r in rows]

    print(f"   线速度: 平均={sum(velocities)/len(velocities):.3f} "
          f"最大={max(velocities):.3f} "
          f"最小={min(velocities):.3f}")
    print(f"   角速度: 平均={sum(omegas)/len(omegas):.3f} "
          f"最大={max(omegas):.3f} "
          f"最小={min(omegas):.3f}")

    # 速度突变检测
    v_jumps = []
    for i in range(1, len(velocities)):
        dv = abs(velocities[i] - velocities[i-1])
        if dv > 0.5:  # 线速度突变>0.5
            v_jumps.append({
                'frame': rows[i].get('frame', i),
                'delta': dv,
                'from': velocities[i-1],
                'to': velocities[i],
            })
    print(f"\n   速度突变 (>0.5m/s): {len(v_jumps)}")
    for j in v_jumps[:5]:
        print(f"     帧{j['frame']:5d}: Δv={j['delta']:.2f} "
              f"({j['from']:.2f}->{j['to']:.2f})")

    # 零速停滞
    zero_frames = sum(1 for v in velocities if abs(v) < 0.01)
    print(f"\n   零速帧: {zero_frames} ({zero_frames/len(velocities)*100:.1f}%)")

    # 震荡检测 (速度符号频繁变化)
    sign_changes = 0
    for i in range(1, len(velocities)):
        if velocities[i-1] * velocities[i] < 0:  # 异号
            sign_changes += 1
    print(f"   速度符号变化: {sign_changes}次")


def analyze_coverage(rows):
    """7. 覆盖率分析"""
    print("\n" + "="*70)
    print("7. 覆盖率分析")
    print("="*70)

    if not rows or 'coverage' not in rows[0]:
        print("   数据无覆盖率字段")
        return

    coverages = [r.get('coverage', 0) for r in rows]
    visited = [r.get('visited', 0) for r in rows]

    print(f"   初始覆盖率: {coverages[0]:.1f}%")
    print(f"   最终覆盖率: {coverages[-1]:.1f}%")
    print(f"   最大覆盖率: {max(coverages):.1f}%")
    print(f"   覆盖率增量: {coverages[-1] - coverages[0]:.1f}%")
    print(f"   访问单元数: {visited[-1]}")

    # 覆盖率增长曲线
    print(f"\n   覆盖率增长曲线 (每{max(1, len(coverages)//10)}帧):")
    step = max(1, len(coverages) // 10)
    for i in range(0, len(coverages), step):
        f = rows[i].get('frame', i)
        bar = '█' * int(coverages[i] / 2)
        print(f"     帧{f:6d}: {coverages[i]:5.1f}% {bar}")


def analyze_path_efficiency(rows):
    """8. 路径效率分析"""
    print("\n" + "="*70)
    print("8. 路径效率分析")
    print("="*70)

    if len(rows) < 2:
        return

    total_dist = 0
    for i in range(1, len(rows)):
        dx = rows[i].get('true_x', 0) - rows[i-1].get('true_x', 0)
        dy = rows[i].get('true_y', 0) - rows[i-1].get('true_y', 0)
        total_dist += math.sqrt(dx*dx + dy*dy)

    start_x = rows[0].get('true_x', 0)
    start_y = rows[0].get('true_y', 0)
    end_x = rows[-1].get('true_x', 0)
    end_y = rows[-1].get('true_y', 0)
    direct_dist = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)

    efficiency = direct_dist / total_dist if total_dist > 0 else 0
    print(f"   总移动距离: {total_dist:.2f}m")
    print(f"   起止直线距离: {direct_dist:.2f}m")
    print(f"   路径效率 (直线/实际): {efficiency:.2%}")
    print(f"   每帧平均移动: {total_dist/len(rows):.4f}m")


def analyze_obstacle_proximity(rows):
    """9. 障碍物接近分析"""
    print("\n" + "="*70)
    print("9. 障碍物接近分析")
    print("="*70)

    proximity_counts = defaultdict(int)
    min_distances = {}
    for r in rows:
        tx = r.get('true_x', 0)
        ty = r.get('true_y', 0)
        for name, xmin, ymin, xmax, ymax in OBSTACLES_BBOX:
            # 到bbox最近距离
            dx = max(xmin - tx, 0, tx - xmax)
            dy = max(ymin - ty, 0, ty - ymax)
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 1.0:  # 1m内
                proximity_counts[name] += 1
                if name not in min_distances or dist < min_distances[name]:
                    min_distances[name] = dist

    print(f"   障碍物接近统计 (1m内):")
    for name, cnt in sorted(proximity_counts.items(), key=lambda x: -x[1]):
        min_d = min_distances.get(name, 0)
        print(f"     {name:<20s}: {cnt:5d}帧, 最小距离={min_d:.3f}m")


def analyze_errors(rows, events):
    """10. 错误分析"""
    print("\n" + "="*70)
    print("10. 错误分析")
    print("="*70)

    errors = events['errors']
    print(f"   错误总数: {len(errors)}")

    if errors:
        # 按错误类型分类
        error_types = defaultdict(int)
        for e in errors:
            if 'setObjectPosition' in e:
                error_types['setObjectPosition'] += 1
            elif 'object does not exist' in e:
                error_types['object_does_not_exist'] += 1
            elif 'timeout' in e.lower():
                error_types['timeout'] += 1
            elif 'connection' in e.lower():
                error_types['connection'] += 1
            else:
                error_types['other'] += 1

        print(f"   错误分类:")
        for etype, cnt in sorted(error_types.items(), key=lambda x: -x[1]):
            print(f"     {etype}: {cnt}")

        print(f"\n   前10个错误:")
        for e in errors[:10]:
            print(f"     {e[:150]}")


def main():
    print("=" * 70)
    print("长时间仿真综合数据分析 (v9.0)")
    print("=" * 70)

    # 默认路径
    default_csv = os.path.join(os.environ.get('TEMP', '/tmp'), 'puppy_nav_data.csv')
    default_log = 'sim_run_v5c.log'

    csv_path = sys.argv[1] if len(sys.argv) > 1 else default_csv
    log_path = sys.argv[2] if len(sys.argv) > 2 else default_log

    print(f"\nCSV路径: {csv_path}")
    print(f"日志路径: {log_path}")

    # 加载数据
    rows = load_csv(csv_path)
    events = parse_log(log_path)

    if not rows:
        print("错误: 无数据")
        return 1

    print(f"\n加载数据: {len(rows)}帧")
    print(f"日志事件: 状态转换={len(events['state_transitions'])}, "
          f"RECOVER={len(events['recover_events'])}, "
          f"门道={len(events['doorway_crossings'])}")

    # 运行所有分析
    real_clips, edge_contacts, near_misses = check_wall_clipping(rows)
    teleports = check_teleportation(rows)
    analyze_amcl_drift(rows)
    analyze_state_machine(rows, events)
    analyze_doorway(rows, events)
    analyze_velocity(rows)
    analyze_coverage(rows)
    analyze_path_efficiency(rows)
    analyze_obstacle_proximity(rows)
    analyze_errors(rows, events)

    # 综合评估
    print("\n" + "="*70)
    print("综合评估")
    print("="*70)

    issues = []
    if real_clips:
        issues.append(f"真穿墙事件: {len(real_clips)}次 (严重!)")
    if edge_contacts:
        issues.append(f"边缘接触: {len(edge_contacts)}次 (需关注)")
    if teleports:
        issues.append(f"传送事件: {len(teleports)}次")
    loc_errs = [r.get('loc_err', 0) for r in rows]
    if loc_errs:
        avg_err = sum(loc_errs) / len(loc_errs)
        if avg_err > 0.5:
            issues.append(f"AMCL平均漂移: {avg_err:.3f}m (偏高)")
        drift_1 = sum(1 for e in loc_errs if e > 1.0)
        if drift_1 > len(loc_errs) * 0.1:
            issues.append(f"AMCL漂移>1m: {drift_1}帧 ({drift_1/len(loc_errs)*100:.1f}%)")
    if events['errors']:
        issues.append(f"错误事件: {len(events['errors'])}次")

    if issues:
        print("\n   ⚠ 发现问题:")
        for issue in issues:
            print(f"     - {issue}")
    else:
        print("\n   ✓ 仿真运行正常，未发现重大问题")

    # 保存报告
    report_path = 'long_run_analysis_report.txt'
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(f"长时间仿真分析报告\n")
        f.write(f"数据: {len(rows)}帧\n")
        f.write(f"真穿墙事件: {len(real_clips)}\n")
        f.write(f"边缘接触: {len(edge_contacts)}\n")
        f.write(f"传送事件: {len(teleports)}\n")
        if loc_errs:
            f.write(f"AMCL平均误差: {sum(loc_errs)/len(loc_errs):.4f}m\n")
        f.write(f"问题列表: {issues}\n")
    print(f"\n报告已保存: {report_path}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
