#!/usr/bin/env python3
"""
长时间巡航仿真 + 数据分析
==========================
运行多轮巡航循环（每轮访问8个房间），累计2小时仿真时长，
分析穿墙、碰撞、定位漂移、覆盖率、性能衰减等问题。

用法：
  python long_patrol_simulation.py
"""
import os
import sys
import math
import json
import csv
import time
import argparse
import tempfile
import numpy as np
from datetime import datetime

# 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from auto_patrol_simulation import (
    AutoPatrolSimulator, PATROL_POINTS, OBSTACLES_BBOX,
    is_position_safe, check_wall_penetration,
    DynamicObstacle, _ray_circle_intersect
)


class LongPatrolSimulator(AutoPatrolSimulator):
    """长时间巡航仿真器：多轮循环 + 详细数据记录"""

    def __init__(self, use_coppelia=False, target_frames=72000):
        super().__init__(use_coppelia=use_coppelia)
        self.target_frames = target_frames
        self.patrol_rounds = 0  # 完成的巡航轮数
        self.room_visits = {}   # 房间访问统计
        self.per_round_stats = []  # 每轮统计
        self.frame_times = []  # 每帧计算时间（性能监控）
        self.battery_history = []  # 电量历史
        self.position_samples = []  # 位置采样（用于覆盖率分析）
        self.collision_hotspots = {}  # 碰撞热点位置
        self.stuck_events = []  # 卡死事件
        self.recovery_events = []  # 恢复事件详细记录

        # AMCL 定位监控（如果使用）
        self.loc_errors = []  # 定位误差历史
        self.particle_counts = []  # 粒子数历史
        self.kidnapping_events = []  # kidnapping事件

        print(f"[LONG] 长时间巡航仿真器初始化")
        print(f"[LONG] 目标帧数: {target_frames} (~{target_frames/3600:.1f}小时)")
        print(f"[LONG] 预计巡航轮数: ~{target_frames // 500}")

    def run_long_patrol(self):
        """运行长时间巡航"""
        print("\n" + "=" * 60)
        print("  长时间多房间巡航仿真开始")
        print("=" * 60)
        start_time = time.time()

        while self.frame < self.target_frames:
            self.patrol_rounds += 1
            round_start = self.frame
            round_time_start = time.time()

            print(f"\n{'#' * 60}")
            print(f"#  巡航第 {self.patrol_rounds} 轮  "
                  f"(帧 {self.frame}/{self.target_frames})")
            print(f"{'#' * 60}")

            # 运行一轮巡航
            round_results = []
            for i, pt in enumerate(PATROL_POINTS):
                if self.frame >= self.target_frames:
                    break

                # 导航到巡航点
                success, reason, steps = self.navigate_to(
                    pt['x'], pt['y'], pt['yaw'], max_steps=500)

                result = {
                    'round': self.patrol_rounds,
                    'index': i + 1,
                    'name': pt['name'],
                    'room': pt['room'],
                    'success': success,
                    'reason': reason,
                    'steps': steps,
                    'battery': self.battery,
                    'frame': self.frame,
                }
                round_results.append(result)

                # 房间访问统计
                room = pt['room']
                if room not in self.room_visits:
                    self.room_visits[room] = {'total': 0, 'success': 0, 'fail': 0}
                self.room_visits[room]['total'] += 1
                if success:
                    self.room_visits[room]['success'] += 1
                else:
                    self.room_visits[room]['fail'] += 1

                # 低电量回充
                if not success and reason == 'low_battery':
                    print(f"[LONG] 低电量回充...")
                    self.navigate_to(
                        PATROL_POINTS[0]['x'], PATROL_POINTS[0]['y'],
                        max_steps=300)
                    self.battery = 100.0
                    # 重试
                    success, reason, steps = self.navigate_to(
                        pt['x'], pt['y'], pt['yaw'], max_steps=400)
                    result['success'] = success
                    result['reason'] = reason
                    result['steps'] = steps

            # 每轮统计
            round_time = time.time() - round_time_start
            round_frames = self.frame - round_start
            room_results = [r for r in round_results
                           if not PATROL_POINTS[r['index']-1].get('waypoint', False)]
            arrived = sum(1 for r in room_results if r['success'])

            self.per_round_stats.append({
                'round': self.patrol_rounds,
                'arrived': arrived,
                'total_rooms': len(room_results),
                'arrival_rate': arrived / len(room_results) if room_results else 0,
                'frames': round_frames,
                'time_sec': round_time,
                'start_frame': round_start,
                'end_frame': self.frame,
                'battery_end': self.battery,
                'wall_penetrations': len(self.wall_penetrations),
                'dynamic_collisions': len(self.dynamic_collisions),
            })

            elapsed = time.time() - start_time
            print(f"\n[LONG] 第{self.patrol_rounds}轮完成: "
                  f"{arrived}/{len(room_results)}到达, "
                  f"{round_frames}帧, {round_time:.1f}s, "
                  f"总进度{self.frame}/{self.target_frames} "
                  f"({self.frame/self.target_frames*100:.1f}%)")

            # 每10轮打印一次汇总
            if self.patrol_rounds % 10 == 0:
                self._print_progress(elapsed)

            # 每50轮保存检查点
            if self.patrol_rounds % 50 == 0:
                self._save_checkpoint()

        total_time = time.time() - start_time
        print(f"\n[LONG] 仿真完成! 总时长: {total_time:.1f}s "
              f"({total_time/3600:.2f}小时)")
        self._save_final_results()
        self._analyze_data()
        return self.per_round_stats

    def _print_progress(self, elapsed):
        """打印进度"""
        print(f"\n{'='*50}")
        print(f"  进度报告 (第{self.patrol_rounds}轮)")
        print(f"{'='*50}")
        print(f"  帧数: {self.frame}/{self.target_frames} "
              f"({self.frame/self.target_frames*100:.1f}%)")
        print(f"  耗时: {elapsed:.0f}s ({elapsed/3600:.2f}h)")
        print(f"  穿墙: {len(self.wall_penetrations)}")
        print(f"  动态碰撞: {len(self.dynamic_collisions)}")
        print(f"  近距离规避: {len(self.near_misses)}")
        print(f"  恢复次数: {self.recovery_count}")

        # 房间到达率
        print(f"\n  房间访问统计:")
        for room, stats in self.room_visits.items():
            if stats['total'] > 0:
                rate = stats['success'] / stats['total'] * 100
                print(f"    {room}: {stats['success']}/{stats['total']} "
                      f"({rate:.0f}%)")

        # 近5轮到达率趋势
        if len(self.per_round_stats) >= 5:
            recent = self.per_round_stats[-5:]
            rates = [r['arrival_rate'] for r in recent]
            print(f"\n  近5轮到达率: {[f'{r:.0%}' for r in rates]}")
            print(f"  平均: {np.mean(rates):.1%}")
        print(f"{'='*50}")

    def _save_checkpoint(self):
        """保存检查点"""
        output_dir = os.path.join(tempfile.gettempdir(), 'long_patrol')
        os.makedirs(output_dir, exist_ok=True)
        ckpt = {
            'frame': self.frame,
            'rounds': self.patrol_rounds,
            'battery': self.battery,
            'wall_penetrations': len(self.wall_penetrations),
            'dynamic_collisions': len(self.dynamic_collisions),
            'near_misses': len(self.near_misses),
            'recovery_count': self.recovery_count,
            'per_round_stats': self.per_round_stats,
            'room_visits': self.room_visits,
            'timestamp': datetime.now().isoformat(),
        }
        path = os.path.join(output_dir, 'checkpoint.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(ckpt, f, indent=2, ensure_ascii=False)
        print(f"[LONG] 检查点已保存: {path}")

    def _save_final_results(self):
        """保存最终结果"""
        output_dir = os.path.join(tempfile.gettempdir(), 'long_patrol')
        os.makedirs(output_dir, exist_ok=True)

        # 汇总JSON
        summary = {
            'timestamp': datetime.now().isoformat(),
            'total_frames': self.frame,
            'total_rounds': self.patrol_rounds,
            'wall_penetrations': len(self.wall_penetrations),
            'dynamic_collisions': len(self.dynamic_collisions),
            'near_misses': len(self.near_misses),
            'recovery_count': self.recovery_count,
            'battery_final': self.battery,
            'per_round_stats': self.per_round_stats,
            'room_visits': self.room_visits,
        }
        json_path = os.path.join(output_dir, 'long_summary.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"[LONG] 汇总已保存: {json_path}")

        # 轨迹CSV
        csv_path = os.path.join(output_dir, 'long_trajectory.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['frame', 'x', 'y', 'yaw', 'target_x',
                            'target_y', 'battery'])
            for t in self.trajectory:
                writer.writerow([t['frame'], f"{t['x']:.4f}",
                                f"{t['y']:.4f}", f"{t['yaw']:.4f}",
                                f"{t['target_x']:.4f}", f"{t['target_y']:.4f}",
                                f"{t['battery']:.1f}"])
        print(f"[LONG] 轨迹已保存: {csv_path}")

        # 穿墙事件CSV
        if self.wall_penetrations:
            wp_path = os.path.join(output_dir, 'wall_penetrations.csv')
            with open(wp_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['frame', 'from_x', 'from_y',
                                'to_x', 'to_y', 'dist'])
                for wp in self.wall_penetrations:
                    fx, fy = wp['from']
                    tx, ty = wp['to']
                    writer.writerow([wp['frame'], f"{fx:.4f}", f"{fy:.4f}",
                                    f"{tx:.4f}", f"{ty:.4f}",
                                    f"{wp['dist']:.4f}"])
            print(f"[LONG] 穿墙事件: {wp_path}")

        # 动态碰撞CSV
        if self.dynamic_collisions:
            dc_path = os.path.join(output_dir, 'dynamic_collisions.csv')
            with open(dc_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['frame', 'obstacle', 'distance',
                                'robot_x', 'robot_y', 'obs_x', 'obs_y'])
                for dc in self.dynamic_collisions:
                    rx, ry = dc['robot_pos']
                    writer.writerow([dc['frame'], dc['obstacle'],
                                    f"{dc['distance']:.4f}",
                                    f"{rx:.4f}", f"{ry:.4f}"])
            print(f"[LONG] 碰撞事件: {dc_path}")

    def _analyze_data(self):
        """分析仿真数据"""
        print("\n" + "=" * 60)
        print("  长时间仿真数据分析报告")
        print("=" * 60)

        # 1. 总体统计
        print(f"\n--- 1. 总体统计 ---")
        print(f"  总帧数: {self.frame}")
        print(f"  巡航轮数: {self.patrol_rounds}")
        print(f"  穿墙事件: {len(self.wall_penetrations)}")
        print(f"  动态碰撞: {len(self.dynamic_collisions)}")
        print(f"  近距离规避: {len(self.near_misses)}")
        print(f"  恢复次数: {self.recovery_count}")

        # 2. 到达率分析
        print(f"\n--- 2. 到达率分析 ---")
        if self.per_round_stats:
            rates = [r['arrival_rate'] for r in self.per_round_stats]
            print(f"  平均到达率: {np.mean(rates):.1%}")
            print(f"  最高: {max(rates):.1%}")
            print(f"  最低: {min(rates):.1%}")
            print(f"  标准差: {np.std(rates):.3f}")

            # 到达率趋势（前1/3 vs 后1/3）
            n = len(rates)
            if n >= 6:
                early = np.mean(rates[:n//3])
                late = np.mean(rates[2*n//3:])
                trend = "上升" if late > early + 0.05 else \
                        "下降" if early > late + 0.05 else "稳定"
                print(f"  趋势: 前1/3={early:.1%} → 后1/3={late:.1%} [{trend}]")

        # 3. 房间访问统计
        print(f"\n--- 3. 房间访问统计 ---")
        for room, stats in sorted(self.room_visits.items()):
            if stats['total'] > 0:
                rate = stats['success'] / stats['total'] * 100
                tag = " [问题]" if rate < 80 else ""
                print(f"  {room:8s}: {stats['success']:3d}/{stats['total']:3d} "
                      f"({rate:5.1f}%){tag}")

        # 4. 穿墙分析
        print(f"\n--- 4. 穿墙分析 ---")
        if self.wall_penetrations:
            # 按位置聚类
            wp_positions = []
            for wp in self.wall_penetrations:
                fx, fy = wp['from']
                wp_positions.append((fx, fy))

            # 简单聚类：找重复位置
            from collections import Counter
            wp_cells = Counter()
            for x, y in wp_positions:
                cell = (round(x, 1), round(y, 1))
                wp_cells[cell] += 1

            print(f"  穿墙总数: {len(self.wall_penetrations)}")
            print(f"  穿墙热点 (前10):")
            for (x, y), count in wp_cells.most_common(10):
                print(f"    ({x:.1f}, {y:.1f}): {count}次")
        else:
            print(f"  [OK] 无穿墙事件")

        # 5. 动态碰撞分析
        print(f"\n--- 5. 动态碰撞分析 ---")
        if self.dynamic_collisions:
            from collections import Counter
            obs_counts = Counter(dc['obstacle'] for dc in self.dynamic_collisions)
            print(f"  碰撞总数: {len(self.dynamic_collisions)}")
            for obs, count in obs_counts.most_common():
                dists = [dc['distance'] for dc in self.dynamic_collisions
                         if dc['obstacle'] == obs]
                print(f"    {obs}: {count}次, "
                      f"最近={min(dists):.2f}m, 平均={np.mean(dists):.2f}m")

            # 碰撞热点
            dc_cells = Counter()
            for dc in self.dynamic_collisions:
                rx, ry = dc['robot_pos']
                dc_cells[(round(rx, 1), round(ry, 1))] += 1
            print(f"  碰撞热点 (前5):")
            for (x, y), count in dc_cells.most_common(5):
                print(f"    ({x:.1f}, {y:.1f}): {count}次")
        else:
            print(f"  [OK] 无动态碰撞")

        # 6. 性能分析
        print(f"\n--- 6. 性能分析 ---")
        if self.per_round_stats:
            times = [r['time_sec'] for r in self.per_round_stats]
            frames = [r['frames'] for r in self.per_round_stats]
            fps = [f/t if t > 0 else 0 for f, t in zip(frames, times)]
            print(f"  平均FPS: {np.mean(fps):.1f}")
            print(f"  最高FPS: {max(fps):.1f}")
            print(f"  最低FPS: {min(fps):.1f}")

            # 性能衰减检测
            n = len(fps)
            if n >= 6:
                early_fps = np.mean(fps[:n//3])
                late_fps = np.mean(fps[2*n//3:])
                if late_fps < early_fps * 0.8:
                    print(f"  [!] 性能衰减: {early_fps:.1f}→{late_fps:.1f} FPS")
                else:
                    print(f"  [OK] 性能稳定: {early_fps:.1f}→{late_fps:.1f} FPS")

        # 7. 电量分析
        print(f"\n--- 7. 电量分析 ---")
        battery_vals = [t['battery'] for t in self.trajectory]
        if battery_vals:
            print(f"  初始电量: {battery_vals[0]:.1f}%")
            print(f"  最低电量: {min(battery_vals):.1f}%")
            print(f"  最高电量: {max(battery_vals):.1f}%")
            print(f"  末尾电量: {battery_vals[-1]:.1f}%")
            low_battery_count = sum(1 for b in battery_vals if b < 20)
            print(f"  低电量(<20%)帧数: {low_battery_count}")

        # 8. 覆盖率分析
        print(f"\n--- 8. 覆盖率分析 ---")
        if self.trajectory:
            # 栅格化覆盖率
            visited_cells = set()
            for t in self.trajectory:
                gx = int(t['x'] / 0.5)
                gy = int(t['y'] / 0.5)
                visited_cells.add((gx, gy))

            # 可达区域估算（房间区域）
            total_cells = 0
            for x in np.arange(-5, 5, 0.5):
                for y in np.arange(-4, 4, 0.5):
                    if is_position_safe(x, y, OBSTACLES_BBOX, radius=0.3):
                        total_cells += 1

            coverage = len(visited_cells) / total_cells * 100 if total_cells > 0 else 0
            print(f"  访问栅格数: {len(visited_cells)}")
            print(f"  可达栅格数: {total_cells}")
            print(f"  覆盖率: {coverage:.1f}%")

            # 各区域覆盖
            regions = {
                '客厅(y<0)': lambda x, y: y < 0,
                '卧室1(x<-3.5,0<y<2)': lambda x, y: x < -3.5 and 0 < y < 2,
                '书房(x<-3.5,y>2)': lambda x, y: x < -3.5 and y > 2,
                '卧室2(-3.5<x<-1,y>2)': lambda x, y: -3.5 < x < -1 and y > 2,
                '卫生间(-1<x<1,y>2)': lambda x, y: -1 < x < 1 and y > 2,
                '储物间(1<x<3.5,y>2)': lambda x, y: 1 < x < 3.5 and y > 2,
                '餐厅(1<x<2.5,0<y<2)': lambda x, y: 1 < x < 2.5 and 0 < y < 2,
                '厨房(x>2.5,0<y<2)': lambda x, y: x > 2.5 and 0 < y < 2,
            }
            for name, pred in regions.items():
                visited = set()
                for t in self.trajectory:
                    if pred(t['x'], t['y']):
                        gx = int(t['x'] / 0.5)
                        gy = int(t['y'] / 0.5)
                        visited.add((gx, gy))
                print(f"    {name}: {len(visited)}格")

        # 9. 问题总结
        print(f"\n--- 9. 问题总结 ---")
        issues = []
        if self.wall_penetrations:
            issues.append(f"穿墙{len(self.wall_penetrations)}次")
        if self.dynamic_collisions:
            issues.append(f"动态碰撞{len(self.dynamic_collisions)}次")
        if self.recovery_count > self.patrol_rounds * 2:
            issues.append(f"恢复过多({self.recovery_count}次)")

        # 检查到达率下降
        if self.per_round_stats and len(self.per_round_stats) >= 6:
            rates = [r['arrival_rate'] for r in self.per_round_stats]
            n = len(rates)
            if np.mean(rates[2*n//3:]) < np.mean(rates[:n//3]) - 0.1:
                issues.append("到达率下降趋势")

        if issues:
            print(f"  [!] 发现问题: {', '.join(issues)}")
        else:
            print(f"  [OK] 未发现显著问题")

        print(f"\n{'=' * 60}")
        print(f"  分析完成")
        print(f"{'=' * 60}")


def main():
    parser = argparse.ArgumentParser(description='长时间巡航仿真')
    parser.add_argument('--frames', type=int, default=72000,
                        help='目标帧数 (默认72000≈2小时)')
    parser.add_argument('--no-coppelia', action='store_true',
                        help='纯模拟模式')
    args = parser.parse_args()

    sim = LongPatrolSimulator(use_coppelia=not args.no_coppelia,
                              target_frames=args.frames)
    sim.run_long_patrol()


if __name__ == '__main__':
    main()
