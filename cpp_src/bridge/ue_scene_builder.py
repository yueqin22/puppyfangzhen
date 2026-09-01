#!/usr/bin/env python3
"""
ue_scene_builder.py — 从 scene_home.json 在 UE5 中自动生成场景 (UE-PLAN M1.3)
===============================================================
在 UE5 编辑器内通过 Python 控制台执行:
  > py ue_scene_builder.py --json "E:/puppyfangzhen/config/scene_home.json"

功能:
  1. 读取 scene_home.json
  2. 为每个 obstacle 创建 BSP Brush（长方体，高度2.5m）
  3. 为每个 patrol_target 创建一个蓝图标记点
  4. 为每个 pedestrian 创建一个圆柱体 Actor
  5. 设置地板（10m×8m）
  6. 设置光照（DirectionalLight + SkyLight）

依赖: UE5 >= 5.3, Python 3.9+, 启用 "Python Editor Script Plugin"

坐标系映射:
  JSON 世界坐标 (x, y) → UE 坐标 (x, y, z=0)
  UE y 轴 = JSON y 轴（1:1 映射，不翻转）
  障碍物高度: z=0 到 z=2.5（墙面高度）
  地板: z=-0.05 到 z=0（地板厚度0.1m）
"""

import argparse
import json
import math
import os
import sys

def load_scene(json_path):
    """加载场景 JSON"""
    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def ue_coord(x, y, z=0.0):
    """JSON 世界坐标 → UE 坐标 (cm)"""
    # JSON 坐标单位: 米, UE 坐标单位: 厘米
    # 坐标系映射: x→x, y→y, z→z
    return (x * 100.0, y * 100.0, z * 100.0)

def build_floor(scene):
    """创建地板"""
    import unreal
    editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    
    grid = scene['grid']
    width_m = grid['width'] * grid['resolution']  # 10m
    height_m = grid['height'] * grid['resolution']  # 8m
    
    # 创建地板 (BSP Brush)
    floor_location = ue_coord(0, 0, -0.05)
    # 使用 AddCube BSP
    unreal.EditorLevelUtils.add_level_geometry_brush(
        unreal.EngineStreamableTypes.CUBE,
        floor_location,
        (width_m * 100, height_m * 100, 10.0)  # 厚度 10cm
    )
    unreal.log(f"[SceneBuilder] 地板: {width_m}m × {height_m}m")

def build_obstacles(scene):
    """创建障碍物（墙体/家具）"""
    import unreal
    
    obstacles = scene.get('obstacles', [])
    created = 0
    
    for obs in obstacles:
        xmin, ymin = obs['xmin'], obs['ymin']
        xmax, ymax = obs['xmax'], obs['ymax']
        name = obs.get('name', 'obstacle')
        
        # 计算尺寸和中心
        width = (xmax - xmin) * 100.0   # cm
        depth = (ymax - ymin) * 100.0   # cm
        height = 250.0                   # 2.5m = 250cm
        cx = (xmin + xmax) / 2.0 * 100.0
        cy = (ymin + ymax) / 2.0 * 100.0
        cz = height / 2.0  # 底部在 z=0
        
        # 创建 BSP Brush 立方体
        location = unreal.Vector(cx, cy, cz)
        extent = unreal.Vector(width, depth, height)
        
        # 使用 BSP Brush
        brush = unreal.EditorLevelUtils.add_level_geometry_brush(
            unreal.EngineStreamableTypes.CUBE,
            (cx, cy, cz),
            (width, depth, height)
        )
        
        # 设置标签
        if brush:
            brush.set_actor_label(f"OBS_{name}")
            created += 1
    
    unreal.log(f"[SceneBuilder] 障碍物: {created}/{len(obstacles)}")

def build_patrol_targets(scene):
    """创建巡逻点标记"""
    import unreal
    
    targets = scene.get('patrol_targets', [])
    created = 0
    
    for i, target in enumerate(targets):
        x = target['x']
        y = target['y']
        name = target.get('name', f'target_{i}')
        
        # 创建一个空 Actor 作为标记点
        location = unreal.Vector(x * 100.0, y * 100.0, 0.0)
        actor = unreal.EditorLevelUtils.spawn_actor_from_class(
            unreal.Actor,
            location,
            unreal.Rotator(0, 0, 0)
        )
        
        if actor:
            actor.set_actor_label(f"PATROL_{name}")
            # 添加一个球体组件作为可视化
            sphere = unreal.NewObject(unreal.SphereComponent, actor, f"Sphere_{name}")
            sphere.set_sphere_radius(15.0)  # 15cm
            sphere.attach_to_component(
                actor.get_root_component(),
                unreal.AttachLocation.SNAP_TO_TARGET
            )
            sphere.register_component()
            created += 1
    
    unreal.log(f"[SceneBuilder] 巡逻点: {created}/{len(targets)}")

def build_pedestrians(scene):
    """创建行人 Actor (使用 PuppyPedestrianActor, 支持自动巡逻+墙反弹)"""
    import unreal

    peds = scene.get('pedestrians', [])
    created = 0

    # 尝试加载 PuppyPedestrianActor 蓝图类
    # 优先从 PuppyNav 模块加载 C++ 类
    ped_class = None
    try:
        # 尝试通过 class path 加载
        ped_class = unreal.load_class(None, "/Script/PuppyNav.PuppyPedestrianActor")
    except Exception:
        pass

    for i, ped in enumerate(peds):
        x = ped['x']
        y = ped['y']
        vx = ped.get('vx', 0.0)
        vy = ped.get('vy', 0.0)
        name = ped.get('name', f'ped_{i}')
        radius = ped.get('radius', 0.3)  # 默认 30cm

        location = unreal.Vector(x * 100.0, y * 100.0, 90.0)

        if ped_class:
            # 使用 PuppyPedestrianActor (有运动+反弹行为)
            actor = unreal.EditorLevelUtils.spawn_actor_from_class(
                ped_class,
                location,
                unreal.Rotator(0, 0, 0)
            )
            if actor:
                actor.set_actor_label(f"PED_{name}")
                # 设置速度 (m/s -> cm/s)
                actor.set_editor_property("velocity", unreal.Vector2D(vx * 100.0, vy * 100.0))
                actor.set_editor_property("radius", radius * 100.0)
                created += 1
        else:
            # Fallback: 普通 Actor + 胶囊体 (无运动行为, 仅占位)
            print(f"  [WARN] PuppyPedestrianActor 未加载, 使用 fallback Actor for {name}")
            actor = unreal.EditorLevelUtils.spawn_actor_from_class(
                unreal.Actor,
                location,
                unreal.Rotator(0, 0, 0)
            )
            if actor:
                actor.set_actor_label(f"PED_{name}")
                capsule = unreal.NewObject(unreal.CapsuleComponent, actor, f"Capsule_{name}")
                capsule.set_capsule_radius(radius * 100.0)
                capsule.set_capsule_half_height(90.0)
                if not actor.get_root_component():
                    actor.set_root_component(capsule)
                else:
                    capsule.attach_to_component(
                        actor.get_root_component(),
                        unreal.AttachLocation.SNAP_TO_TARGET
                    )
                capsule.register_component()
                created += 1

    unreal.log(f"[SceneBuilder] 行人: {created}/{len(peds)}")

def build_lighting(scene):
    """设置场景光照"""
    import unreal
    
    grid = scene['grid']
    width_m = grid['width'] * grid['resolution']
    height_m = grid['height'] * grid['resolution']
    
    # Directional Light (太阳光)
    sun_location = unreal.Vector(0, 0, 500.0)  # 5m高
    sun = unreal.EditorLevelUtils.spawn_actor_from_class(
        unreal.DirectionalLight,
        sun_location,
        unreal.Rotator(-45.0, -30.0, 0.0)  # 45°俯角
    )
    if sun:
        sun.set_actor_label("Sun_DirectionalLight")
    
    # Sky Light (环境光)
    sky_location = unreal.Vector(0, 0, 400.0)
    sky = unreal.EditorLevelUtils.spawn_actor_from_class(
        unreal.SkyLight,
        sky_location,
        unreal.Rotator(0, 0, 0)
    )
    if sky:
        sky.set_actor_label("Sky_SkyLight")
    
    # Player Start
    robot = scene.get('robot', {})
    init_pose = robot.get('initial_pose', {})
    start_location = unreal.Vector(
        init_pose.get('x', -1.0) * 100.0,
        init_pose.get('y', -3.0) * 100.0,
        50.0
    )
    start = unreal.EditorLevelUtils.spawn_actor_from_class(
        unreal.PlayerStart,
        start_location,
        unreal.Rotator(0, 0, 0)
    )
    if start:
        start.set_actor_label("PlayerStart_Robot")
    
    unreal.log("[SceneBuilder] 光照+PlayerStart 完成")

def main():
    parser = argparse.ArgumentParser(description='UE5 Scene Builder from JSON')
    parser.add_argument('--json', required=True, help='Path to scene_home.json')
    parser.add_argument('--floor-only', action='store_true', help='Only build floor')
    parser.add_argument('--obstacles-only', action='store_true', help='Only build obstacles')
    args = parser.parse_args()
    
    # 加载场景
    scene = load_scene(args.json)
    print(f"[SceneBuilder] 加载场景: {args.json}")
    print(f"  障碍物: {len(scene.get('obstacles', []))}")
    print(f"  巡逻点: {len(scene.get('patrol_targets', []))}")
    print(f"  行人: {len(scene.get('pedestrians', []))}")
    
    # 检查是否在 UE 环境中
    try:
        import unreal
    except ImportError:
        print("[ERROR] 此脚本必须在 UE5 编辑器 Python 控制台中运行!")
        print("  1. 启用 Edit > Plugins > Python Editor Script Plugin")
        print("  2. 打开 Python 控制台 (Output Log 下拉选 Python)")
        print(f"  3. 执行: py {os.path.abspath(__file__)} --json {args.json}")
        sys.exit(1)
    
    # 构建
    if not args.obstacles_only:
        build_floor(scene)
        build_lighting(scene)
        build_patrol_targets(scene)
        build_pedestrians(scene)
    
    if not args.floor_only:
        build_obstacles(scene)
    
    # 保存关卡
    unreal.EditorLevelUtils.save_current_level()
    unreal.log("[SceneBuilder] 场景构建完成! 关卡已保存。")

if __name__ == '__main__':
    main()
