#!/usr/bin/env python3
"""
多房间创新场景构建脚本
======================
在 CoppeliaSim 中构建一个包含 8 个不同房间的复杂室内环境，
用于验证自主导航系统的探索、定位和巡航能力。

房间布局（10m × 8m）：
  ┌──────────────────────────────────────────────┐
  │  书房      │  卧室2   │  卫生间   │  储物间  │
  │  (study)  │(bedroom2)│(bathroom) │(storage) │
  │           │          │          │          │
  ├──────┬────┤    ┌─────┴────┐    ├──────────┤
  │      │走廊 │    │  餐厅    │    │          │
  │ 卧室1│hall │    │(dining)  │    │  厨房    │
  │(bed) │     │    │          │    │(kitchen) │
  │      ├─────┘    └──────────┘    │          │
  ├──────┴──────────────────────────┴──────────┤
  │              客厅 (living room)            │
  │              [充电桩] [沙发] [茶几]          │
  └──────────────────────────────────────────────┘

创新点：
  - 8个语义不同房间（客厅/卧室1/卧室2/厨房/卫生间/书房/餐厅/储物间）
  - 6个门道（y=0 通道 + 各房间入口）
  - 不同宽度的通道（窄通道测试）
  - 家具布局差异化（每个房间不同家具类型）
  - 多个语义地标（用于语义SLAM验证）
"""
import math
import sys
import os
import time

# 导入 CoppeliaSim Remote API
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


def build_multi_room_scene():
    """构建多房间场景"""
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # 清理场景
    print("=== 清理场景 ===")
    scene_objects = sim.getObjectsInTree(sim.handle_scene)
    for obj in scene_objects:
        try:
            obj_type = sim.getObjectType(obj)
            if obj_type in [0, 1, 4, 5, 6, 8, 10]:  # shape, joint, etc
                sim.removeObject(obj)
        except:
            pass
    time.sleep(1)

    # 场景参数
    wall_height = 2.0
    wall_thickness = 0.1
    wall_color = [0.7, 0.7, 0.7]
    accent_color = [0.6, 0.6, 0.8]

    def create_wall(x, y, length, angle_deg=0, name='wall', color=None):
        """创建一面墙"""
        wall = sim.createPrimitiveShape(3, [length, wall_thickness, wall_height])
        sim.setObjectPosition(wall, -1, [x, y, wall_height/2])
        sim.setObjectOrientation(wall, -1, [0, 0, math.radians(angle_deg)])
        sim.setObjectAlias(wall, name)
        sim.setShapeColor(wall, None, sim.colorcomponent_ambient, color or wall_color)
        sim.setObjectInt32Param(wall, sim.objintparam_visibility_layer, 1)
        return wall

    def create_box(x, y, sx, sy, sz, name, color, z_offset=0):
        """创建一个盒子家具"""
        box = sim.createPrimitiveShape(3, [sx, sy, sz])
        sim.setObjectPosition(box, -1, [x, y, z_offset + sz/2])
        sim.setObjectAlias(box, name)
        sim.setShapeColor(box, None, sim.colorcomponent_ambient, color)
        return box

    print("=== 创建地板 ===")
    floor = sim.createPrimitiveShape(3, [10.0, 8.0, 0.02])
    sim.setObjectPosition(floor, -1, [0, 0, -0.01])
    sim.setObjectAlias(floor, 'Floor')
    sim.setShapeColor(floor, None, sim.colorcomponent_ambient, [0.3, 0.3, 0.3])

    # ================================================================
    # 外墙
    # ================================================================
    print("=== 创建外墙 ===")
    create_wall(0, -4, 10, 0, 'wall_south')
    create_wall(0, 4, 10, 0, 'wall_north')
    create_wall(-5, 0, 8, 90, 'wall_west')
    create_wall(5, 0, 8, 90, 'wall_east')

    # ================================================================
    # 内墙布局 - 8个房间
    # ================================================================
    print("=== 创建内墙（8房间布局）===")

    # --- y=0 主分隔墙（客厅与上方房间）---
    # 西段：x=[-5, -3.5]，留1.5m门道
    create_wall(-4.25, 0, 1.5, 0, 'wall_div_w')
    # 中西段：x=[-2, 0]，留2m门道（主通道）
    create_wall(-1, 0, 2, 0, 'wall_div_mw')
    # 中东段：x=[2, 0]，留2m门道
    create_wall(1, 0, 2, 0, 'wall_div_me')
    # 东段：x=[3.5, 5]，留1.5m门道
    create_wall(4.25, 0, 1.5, 0, 'wall_div_e')

    # --- 上层水平分隔（y=2）---
    # 书房/卧室1 vs 卧室2/餐厅
    create_wall(-3.5, 2, 1.5, 0, 'wall_up_h1')  # x=[-5,-3.5]西段
    # 留门道 x=[-3.5, -2]
    create_wall(-1, 2, 2, 0, 'wall_up_h2')  # x=[-2, 0]
    # 留门道 x=[0, 1]
    create_wall(2.5, 2, 3, 0, 'wall_up_h3')  # x=[1, 4]
    # 留门道 x=[4, 5]
    # 东段
    create_wall(4.5, 2, 1, 0, 'wall_up_h4')

    # --- 垂直分隔墙 ---
    # 书房/卧室1 分隔（x=-3.5, y=[0,2]）
    create_wall(-3.5, 1, 2, 90, 'wall_v_study_bed')
    # 留1m门道 y=[0.5, 1.5]

    # 卧室2/卫生间 分隔（x=-1, y=[2,4]）
    create_wall(-1, 3, 2, 90, 'wall_v_bed2_bath')
    # 留1m门道 y=[2.5, 3.5]

    # 餐厅/厨房 分隔（x=2.5, y=[0,2]）— 窄通道测试
    create_wall(2.5, 1.5, 1, 90, 'wall_v_dining_kitchen')  # 窄门道0.8m

    # 卫生间/储物间 分隔（x=1, y=[2,4]）
    create_wall(1, 3, 2, 90, 'wall_v_bath_storage')
    # 留1m门道

    # 储物间/厨房 分隔（x=3.5, y=[0,2]）
    create_wall(3.5, 1, 2, 90, 'wall_v_storage_kitchen')

    # ================================================================
    # 家具布局 - 8个房间
    # ================================================================
    print("=== 创建家具（8个房间）===")

    # 1. 客厅 (living room) - y<0 区域
    create_box(2.5, -3, 1.8, 0.6, 0.4, 'sofa', [0.5, 0.3, 0.2])  # 沙发
    create_box(2.5, -2, 1.0, 0.5, 0.35, 'coffee_table', [0.4, 0.25, 0.15])  # 茶几
    create_box(-1, -3, 0.4, 0.4, 0.1, 'charging_dock', [0.0, 0.8, 0.0])  # 充电桩
    create_box(-3, -3, 1.5, 0.3, 0.5, 'tv_stand', [0.2, 0.2, 0.3])  # 电视柜
    create_box(-3, -2.5, 0.8, 0.1, 0.6, 'tv', [0.1, 0.1, 0.15])  # 电视

    # 2. 卧室1 (bedroom1) - x=[-5,-3.5], y=[0,2]
    create_box(-4.5, 1, 1.5, 2.0, 0.3, 'bed', [0.8, 0.8, 0.9])  # 床
    create_box(-4.2, 0.2, 0.4, 0.4, 0.4, 'nightstand', [0.6, 0.5, 0.4])  # 床头柜
    create_box(-3.8, 1.8, 0.8, 0.3, 1.8, 'wardrobe_bed1', [0.5, 0.4, 0.3])  # 衣柜

    # 3. 书房 (study) - x=[-5,-3.5], y=[2,4]
    create_box(-4.2, 3, 1.2, 0.6, 0.75, 'desk', [0.4, 0.3, 0.2])  # 书桌
    create_box(-4.2, 3.5, 0.3, 0.3, 1.8, 'bookshelf', [0.3, 0.2, 0.15])  # 书架
    create_box(-4.5, 2.5, 0.4, 0.4, 0.4, 'chair', [0.2, 0.2, 0.3])  # 椅子

    # 4. 卧室2 (bedroom2) - x=[-3.5,-1], y=[2,4]
    create_box(-2.5, 3, 1.2, 1.8, 0.3, 'bed2', [0.7, 0.7, 0.85])  # 床2
    create_box(-3.2, 2.3, 0.5, 0.5, 0.4, 'dresser', [0.6, 0.5, 0.4])  # 梳妆台
    create_box(-1.3, 3.5, 0.8, 0.3, 1.8, 'wardrobe_bed2', [0.5, 0.4, 0.3])  # 衣柜2

    # 5. 卫生间 (bathroom) - x=[-1,1], y=[2,4]
    create_box(-0.5, 3, 0.6, 0.6, 0.4, 'toilet', [0.9, 0.9, 0.9])  # 马桶
    create_box(0.5, 3, 0.8, 0.6, 0.5, 'sink', [0.8, 0.8, 0.85])  # 洗手台
    create_box(0, 3.7, 1.0, 0.5, 0.3, 'bathtub', [0.7, 0.8, 0.9])  # 浴缸

    # 6. 储物间 (storage) - x=[1,3.5], y=[2,4]（小房间）
    create_box(2, 3, 1.5, 0.4, 1.8, 'shelf_storage', [0.4, 0.35, 0.25])  # 储物架
    create_box(3.2, 2.5, 0.3, 0.3, 0.5, 'box_storage', [0.5, 0.4, 0.2])  # 收纳箱

    # 7. 餐厅 (dining) - x=[1,2.5], y=[0,2]
    create_box(1.75, 1, 0.8, 0.05, 0.05, 'dining_table', [0.6, 0.4, 0.2])  # 餐桌(圆柱)
    table = sim.createPrimitiveShape(1, [0.8, 0.05, 0.05])  # 圆柱餐桌
    sim.setObjectPosition(table, -1, [1.75, 1, 0.4])
    sim.setObjectAlias(table, 'dining_table_round')
    sim.setShapeColor(table, None, sim.colorcomponent_ambient, [0.6, 0.4, 0.2])
    # 餐椅
    for i, (cx, cy) in enumerate([(1.3, 0.5), (2.2, 0.5), (1.3, 1.5), (2.2, 1.5)]):
        create_box(cx, cy, 0.35, 0.35, 0.45, f'chair_dining_{i}', [0.5, 0.4, 0.3])

    # 8. 厨房 (kitchen) - x=[2.5,5], y=[0,2]
    create_box(3.5, 1.8, 1.5, 0.6, 0.85, 'counter', [0.7, 0.7, 0.7])  # 橱柜
    create_box(4.5, 1.8, 0.6, 0.6, 1.8, 'fridge', [0.8, 0.8, 0.85])  # 冰箱
    create_box(3, 1.8, 0.5, 0.5, 0.9, 'stove', [0.5, 0.3, 0.2])  # 灶台

    # ================================================================
    # 创建机器狗模型（简化版）
    # ================================================================
    print("=== 创建机器狗 ===")
    base_footprint = sim.createDummy(0.01)
    sim.setObjectAlias(base_footprint, 'base_footprint')
    sim.setObjectPosition(base_footprint, -1, [0.0, -3.0, 0.0])

    body_color = [0.2, 0.4, 0.8]
    base_link = sim.createPrimitiveShape(3, [0.35, 0.18, 0.10])
    sim.setObjectPosition(base_link, base_footprint, [0, 0, 0.30])
    sim.setObjectAlias(base_link, 'base_link')
    sim.setShapeColor(base_link, None, sim.colorcomponent_ambient, body_color)
    sim.setObjectParent(base_link, base_footprint, True)

    # 四条腿
    for name, lx, ly in [("fl", 0.15, 0.08), ("fr", 0.15, -0.08),
                          ("rl", -0.15, 0.08), ("rr", -0.15, -0.08)]:
        upper = sim.createPrimitiveShape(3, [0.04, 0.04, 0.15])
        sim.setObjectPosition(upper, base_link, [lx, ly, -0.075])
        sim.setObjectAlias(upper, f'leg_{name}_upper')
        sim.setShapeColor(upper, None, sim.colorcomponent_ambient, [0.3, 0.3, 0.35])
        sim.setObjectParent(upper, base_link, True)

        lower = sim.createPrimitiveShape(3, [0.03, 0.03, 0.12])
        sim.setObjectPosition(lower, upper, [0, 0, -0.135])
        sim.setObjectAlias(lower, f'leg_{name}_lower')
        sim.setShapeColor(lower, None, sim.colorcomponent_ambient, [0.25, 0.25, 0.3])
        sim.setObjectParent(lower, upper, True)

    # 头部
    head = sim.createPrimitiveShape(3, [0.12, 0.10, 0.08])
    sim.setObjectPosition(head, base_link, [0.22, 0, 0.04])
    sim.setObjectAlias(head, 'head')
    sim.setShapeColor(head, None, sim.colorcomponent_ambient, [0.25, 0.45, 0.85])
    sim.setObjectParent(head, base_link, True)

    # ================================================================
    # 创建 LiDAR 传感器
    # ================================================================
    print("=== 创建 LiDAR ===")
    lidar = sim.createDummy(0.01)
    sim.setObjectAlias(lidar, 'lidar_mount')
    sim.setObjectPosition(lidar, base_link, [0, 0, 0.15])
    sim.setObjectParent(lidar, base_link, True)

    # ================================================================
    # 保存场景
    # ================================================================
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'multi_room_scene.ttt')
    sim.saveScene(output)
    print(f"=== 场景已保存: {output} ===")

    # 打印场景统计
    all_shapes = [o for o in sim.getObjectsInTree(sim.handle_scene)
                  if sim.getObjectType(o) == 0]
    print(f"\n场景统计:")
    print(f"  总对象数: {len(all_shapes)}")
    walls = [o for o in all_shapes if 'wall' in sim.getObjectAlias(o).lower()]
    furniture = [o for o in all_shapes
                  if 'wall' not in sim.getObjectAlias(o).lower()
                  and o != floor]
    print(f"  墙壁数: {len(walls)}")
    print(f"  家具数: {len(furniture)}")
    print(f"  房间数: 8 (客厅/卧室1/卧室2/书房/餐厅/厨房/卫生间/储物间)")

    return output


if __name__ == '__main__':
    scene_path = build_multi_room_scene()
    print(f"\n多房间场景构建完成: {scene_path}")
    print("可使用以下命令运行自动巡航:")
    print("  python auto_patrol_simulation.py")
