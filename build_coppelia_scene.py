#!/usr/bin/env python3
"""
Puppy 机械狗 - CoppeliaSim 场景构建脚本
创建 home 环境 + 简化机器人 + ROS2 接口

环境布局（与 home.world 一致，10m x 8m）：
  - 外墙：x=[-5,5], y=[-4,4]
  - 门口：x=[-1,1], y=0（4m 宽通道）
  - 客厅：y<0 区域
  - 卧室/厨房/走廊：y>0 区域
"""
import argparse
import os
import sys
import math
import time


def import_remote_api_client():
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        return RemoteAPIClient
    except ImportError:
        coppelia_root = os.environ.get('COPPELIASIM_ROOT')
        if coppelia_root:
            candidate = os.path.join(
                coppelia_root, 'programming', 'zmqRemoteApi', 'clients', 'python')
            if os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.insert(0, candidate)
                from coppeliasim_zmqremoteapi_client import RemoteAPIClient
                return RemoteAPIClient
        raise


RemoteAPIClient = import_remote_api_client()


def default_demo_scene():
    coppelia_root = os.environ.get('COPPELIASIM_ROOT')
    if coppelia_root:
        candidate = os.path.join(coppelia_root, 'scenes', 'fabricationBlocks.ttt')
        if os.path.exists(candidate):
            return candidate
    return None


def default_output_scene():
    repo_worlds = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'src', 'puppy_worlds', 'worlds')
    return os.path.join(repo_worlds, 'home_coppelia.ttt')


def create_scene(demo_scene=None, output_scene=None):
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    demo_scene = demo_scene or default_demo_scene()
    if demo_scene and os.path.exists(demo_scene):
        print("=== 加载基础场景（保留相机和灯光）===")
        sim.loadScene(demo_scene)
        time.sleep(0.5)
    else:
        print("=== 未提供可用 demo 场景，使用当前已打开场景继续构建 ===")

    print("=== 清理demo物体（保留相机和灯光）===")
    # 只删除shape、dummy、joint等对象，保留camera(3)和light(9)
    scene_objects = sim.getObjectsInTree(sim.handle_scene)
    for obj in scene_objects:
        try:
            obj_type = sim.getObjectType(obj)
            # 0=shape, 1=joint, 4=dial, 5=graph, 6=proxsensor, 8=visionsensor, 10=dummy
            # 保留: 3=camera, 9=light
            if obj_type in [0, 1, 4, 5, 6, 8, 10]:
                sim.removeObject(obj)
        except:
            pass
    time.sleep(1)
    
    print("=== 创建地板 ===")
    # 地板 10m x 8m (primitiveshape_cuboid=3)
    floor = sim.createPrimitiveShape(3, [10.0, 8.0, 0.02])
    sim.setObjectPosition(floor, -1, [0, 0, -0.01])
    sim.setObjectAlias(floor, 'Floor')
    sim.setShapeColor(floor, None, sim.colorcomponent_ambient, [0.3, 0.3, 0.3])
    
    print("=== 创建墙壁 ===")
    wall_height = 2.0
    wall_thickness = 0.1
    wall_color = [0.7, 0.7, 0.7]
    
    def create_wall(x, y, length, angle_deg=0, name='wall'):
        """创建一面墙"""
        wall = sim.createPrimitiveShape(3, 
                                         [length, wall_thickness, wall_height])
        sim.setObjectPosition(wall, -1, [x, y, wall_height/2])
        sim.setObjectOrientation(wall, -1, [0, 0, math.radians(angle_deg)])
        sim.setObjectAlias(wall, name)
        sim.setShapeColor(wall, None, sim.colorcomponent_ambient, wall_color)
        # 设为静态
        sim.setObjectInt32Param(wall, sim.objintparam_visibility_layer, 1)
        return wall
    
    # 外墙（4面）
    create_wall(0, -4, 10, 0, 'wall_south')   # 南墙 y=-4
    create_wall(0, 4, 10, 0, 'wall_north')    # 北墙 y=4
    create_wall(-5, 0, 8, 90, 'wall_west')    # 西墙 x=-5
    create_wall(5, 0, 8, 90, 'wall_east')     # 东墙 x=5
    
    # 内墙（分隔客厅和卧室区域，y=0 处，留 4m 门口）
    # 西侧内墙 x=[-5, -1]，长度 4m
    create_wall(-3, 0, 4, 0, 'wall_divide_1')
    # 东侧内墙 x=[1, 5]，长度 4m
    create_wall(3, 0, 4, 0, 'wall_divide_2')
    
    # 卧室隔墙（x=-5 处，分隔卧室和走廊）
    create_wall(-5, 2, 4, 90, 'wall_bedroom_1')
    
    # 厨房隔墙（x=5 处，分隔厨房和走廊）
    create_wall(5, 2, 4, 90, 'wall_kitchen_1')
    
    print("=== 创建家具 ===")
    # 沙发 (3.5, -3)
    sofa = sim.createPrimitiveShape(3, [1.5, 0.6, 0.4])
    sim.setObjectPosition(sofa, -1, [3.5, -3, 0.2])
    sim.setObjectAlias(sofa, 'sofa')
    sim.setShapeColor(sofa, None, sim.colorcomponent_ambient, [0.5, 0.3, 0.2])
    
    # 床 (-3.5, 3)
    bed = sim.createPrimitiveShape(3, [1.5, 2.0, 0.3])
    sim.setObjectPosition(bed, -1, [-3.5, 3, 0.15])
    sim.setObjectAlias(bed, 'bed')
    sim.setShapeColor(bed, None, sim.colorcomponent_ambient, [0.8, 0.8, 0.9])
    
    # 餐桌 (2, 2.5) - 圆柱体 (值=1)
    table = sim.createPrimitiveShape(1, [0.8, 0.05, 0.05])
    sim.setObjectPosition(table, -1, [2, 2.5, 0.4])
    sim.setObjectAlias(table, 'dining_table')
    sim.setShapeColor(table, None, sim.colorcomponent_ambient, [0.6, 0.4, 0.2])
    
    # 充电桩 (-1, -3)
    dock = sim.createPrimitiveShape(3, [0.3, 0.3, 0.1])
    sim.setObjectPosition(dock, -1, [-1, -3, 0.05])
    sim.setObjectAlias(dock, 'charging_dock')
    sim.setShapeColor(dock, None, sim.colorcomponent_ambient, [0.0, 0.8, 0.0])
    
    print("=== 创建机器狗模型 ===")
    # base_footprint（根节点）
    base_footprint = sim.createDummy(0.01)
    sim.setObjectAlias(base_footprint, 'base_footprint')
    sim.setObjectPosition(base_footprint, -1, [1.0, -2.0, 0.0])

    # ---- 机器狗身体参数 ----
    body_color = [0.2, 0.4, 0.8]      # 蓝色主体
    body_dark = [0.15, 0.25, 0.5]     # 深色关节
    leg_color = [0.3, 0.3, 0.35]      # 灰色腿
    head_color = [0.25, 0.45, 0.85]   # 头部稍亮
    ear_color = [0.1, 0.15, 0.3]      # 深色耳朵
    accent_color = [0.9, 0.7, 0.0]    # 金色装饰（眼睛/项圈）

    # ---- 主躯干 (base_link) ----
    base_link = sim.createPrimitiveShape(3, [0.35, 0.18, 0.10])
    sim.setObjectPosition(base_link, base_footprint, [0, 0, 0.30])
    sim.setObjectAlias(base_link, 'base_link')
    sim.setShapeColor(base_link, None, sim.colorcomponent_ambient, body_color)
    sim.setObjectParent(base_link, base_footprint, True)

    # ---- 前胸（前部稍微变窄的形状） ----
    chest = sim.createPrimitiveShape(3, [0.12, 0.14, 0.09])
    sim.setObjectPosition(chest, base_link, [0.20, 0, -0.005])
    sim.setObjectAlias(chest, 'chest')
    sim.setShapeColor(chest, None, sim.colorcomponent_ambient, body_dark)
    sim.setObjectParent(chest, base_link, True)

    # ---- 臀部（后部） ----
    hip = sim.createPrimitiveShape(3, [0.10, 0.16, 0.09])
    sim.setObjectPosition(hip, base_link, [-0.18, 0, -0.005])
    sim.setObjectAlias(hip, 'hip')
    sim.setShapeColor(hip, None, sim.colorcomponent_ambient, body_dark)
    sim.setObjectParent(hip, base_link, True)

    # ---- 头部 ----
    head = sim.createPrimitiveShape(3, [0.10, 0.12, 0.10])
    sim.setObjectPosition(head, base_link, [0.28, 0, 0.06])
    sim.setObjectAlias(head, 'head')
    sim.setShapeColor(head, None, sim.colorcomponent_ambient, head_color)
    sim.setObjectParent(head, base_link, True)

    # ---- 口鼻部 ----
    snout = sim.createPrimitiveShape(3, [0.06, 0.07, 0.05])
    sim.setObjectPosition(snout, head, [0.07, 0, -0.02])
    sim.setObjectAlias(snout, 'snout')
    sim.setShapeColor(snout, None, sim.colorcomponent_ambient, body_dark)
    sim.setObjectParent(snout, head, True)

    # ---- 眼睛（两个小球） ----
    eye_l = sim.createPrimitiveShape(1, [0.015, 0.015, 0.015])
    sim.setObjectPosition(eye_l, head, [0.05, 0.035, 0.02])
    sim.setObjectAlias(eye_l, 'eye_left')
    sim.setShapeColor(eye_l, None, sim.colorcomponent_ambient, accent_color)
    sim.setObjectParent(eye_l, head, True)

    eye_r = sim.createPrimitiveShape(1, [0.015, 0.015, 0.015])
    sim.setObjectPosition(eye_r, head, [0.05, -0.035, 0.02])
    sim.setObjectAlias(eye_r, 'eye_right')
    sim.setShapeColor(eye_r, None, sim.colorcomponent_ambient, accent_color)
    sim.setObjectParent(eye_r, head, True)

    # ---- 耳朵（两个小三角/薄片） ----
    ear_l = sim.createPrimitiveShape(3, [0.03, 0.015, 0.05])
    sim.setObjectPosition(ear_l, head, [-0.01, 0.06, 0.05])
    sim.setObjectOrientation(ear_l, head, [0, 0.3, 0])
    sim.setObjectAlias(ear_l, 'ear_left')
    sim.setShapeColor(ear_l, None, sim.colorcomponent_ambient, ear_color)
    sim.setObjectParent(ear_l, head, True)

    ear_r = sim.createPrimitiveShape(3, [0.03, 0.015, 0.05])
    sim.setObjectPosition(ear_r, head, [-0.01, -0.06, 0.05])
    sim.setObjectOrientation(ear_r, head, [0, -0.3, 0])
    sim.setObjectAlias(ear_r, 'ear_right')
    sim.setShapeColor(ear_r, None, sim.colorcomponent_ambient, ear_color)
    sim.setObjectParent(ear_r, head, True)

    # ---- 尾巴 ----
    tail = sim.createPrimitiveShape(1, [0.012, 0.012, 0.10])
    sim.setObjectPosition(tail, base_link, [-0.23, 0, 0.03])
    sim.setObjectOrientation(tail, base_link, [0.6, 0, 0])
    sim.setObjectAlias(tail, 'tail')
    sim.setShapeColor(tail, None, sim.colorcomponent_ambient, body_dark)
    sim.setObjectParent(tail, base_link, True)

    # ---- 项圈（装饰条） ----
    collar = sim.createPrimitiveShape(3, [0.03, 0.15, 0.02])
    sim.setObjectPosition(collar, base_link, [0.22, 0, 0.05])
    sim.setObjectAlias(collar, 'collar')
    sim.setShapeColor(collar, None, sim.colorcomponent_ambient, accent_color)
    sim.setObjectParent(collar, base_link, True)

    # ---- 4条腿 ----
    # 腿参数
    upper_leg_len = 0.08
    upper_leg_thick = 0.025
    lower_leg_len = 0.10
    lower_leg_thick = 0.020
    leg_positions = [
        # (x, y, name)  前左、前右、后左、后右
        (0.14, 0.10, 'fl'),
        (0.14, -0.10, 'fr'),
        (-0.14, 0.10, 'rl'),
        (-0.14, -0.10, 'rr'),
    ]

    for lx, ly, lname in leg_positions:
        # 上腿（髋关节/肩关节）
        upper = sim.createPrimitiveShape(3, [upper_leg_thick, upper_leg_thick, upper_leg_len])
        sim.setObjectPosition(upper, base_link, [lx, ly, -0.08])
        sim.setObjectAlias(upper, f'leg_{lname}_upper')
        sim.setShapeColor(upper, None, sim.colorcomponent_ambient, leg_color)
        sim.setObjectParent(upper, base_link, True)

        # 下腿（膝关节以下）
        lower = sim.createPrimitiveShape(3, [lower_leg_thick, lower_leg_thick, lower_leg_len])
        sim.setObjectPosition(lower, upper, [0, 0, -0.08])
        sim.setObjectAlias(lower, f'leg_{lname}_lower')
        sim.setShapeColor(lower, None, sim.colorcomponent_ambient, body_dark)
        sim.setObjectParent(lower, upper, True)

        # 脚掌
        foot = sim.createPrimitiveShape(3, [0.035, 0.025, 0.01])
        sim.setObjectPosition(foot, lower, [0, 0, -0.055])
        sim.setObjectAlias(foot, f'leg_{lname}_foot')
        sim.setShapeColor(foot, None, sim.colorcomponent_ambient, [0.05, 0.05, 0.05])
        sim.setObjectParent(foot, lower, True)

    # ---- 激光雷达 ----
    laser_link = sim.createPrimitiveShape(1, [0.06, 0.04, 0.04])
    sim.setObjectPosition(laser_link, base_link, [0, 0, 0.08])
    sim.setObjectAlias(laser_link, 'laser_link')
    sim.setShapeColor(laser_link, None, sim.colorcomponent_ambient, [0.1, 0.1, 0.1])
    sim.setObjectParent(laser_link, base_link, True)

    print("=== 创建激光雷达传感器 ===")
    laser_sensor = sim.createDummy(0.01)
    sim.setObjectAlias(laser_sensor, 'laser_sensor')
    sim.setObjectPosition(laser_sensor, laser_link, [0, 0, 0])
    sim.setObjectParent(laser_sensor, laser_link, True)
    
    print("=== 场景创建完成 ===")
    print(f"base_footprint handle: {base_footprint}")
    print(f"base_link handle: {base_link}")
    print(f"laser_sensor handle: {laser_sensor}")

    # 恢复demo相机视角
    try:
        cam = sim.getObject('/DefaultCamera')
        sim.setObjectPosition(cam, -1, [2.43, -5.91, 5.09])
        sim.setObjectOrientation(cam, -1, [-2.35, -0.375, 2.794])
        print("相机视角已设置")
    except Exception as e:
        print(f"相机设置跳过: {e}")

    # 保存场景
    scene_path = output_scene or default_output_scene()
    os.makedirs(os.path.dirname(scene_path), exist_ok=True)
    sim.saveScene(scene_path)
    print(f"场景已保存到: {scene_path}")

    return base_footprint, base_link, laser_sensor


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--demo-scene',
        default=default_demo_scene(),
        help='可选：用于保留相机和灯光的基础场景路径')
    parser.add_argument(
        '--output',
        default=default_output_scene(),
        help='输出 .ttt 场景文件路径')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    create_scene(args.demo_scene, args.output)
