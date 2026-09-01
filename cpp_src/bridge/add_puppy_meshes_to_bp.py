#!/usr/bin/env python3
"""
add_puppy_meshes_to_bp.py - 在 UE5 Python 控制台中运行
为 BP_PuppyRobotPawn 蓝图添加 11 个 PuppyPi 机器狗 Mesh 组件。
无需重新编译 C++ 模块，直接通过 UE Python API 修改蓝图。

运行方式 (在 UE 编辑器 Python 控制台):
  py e:/puppyfangzhen/cpp_src/bridge/add_puppy_meshes_to_bp.py
"""

import unreal
import os

BP_PATH = "/Game/BP_PuppyRobotPawn"
MESH_PATH = "/Game/PuppyPi"

MESH_LIST = [
    # (组件名, 模型资产路径, 父组件名)
    ("BaseMesh",    f"{MESH_PATH}/base_link.base_link",         "CollisionCapsule"),
    ("LidarMesh",   f"{MESH_PATH}/lidar_Link.lidar_Link",       "BaseMesh"),
    ("CameraMesh",  f"{MESH_PATH}/camera_link.camera_link",     "BaseMesh"),
    ("RfLeg1Mesh",  f"{MESH_PATH}/rf_link1.rf_link1",          "BaseMesh"),
    ("RfLeg2Mesh",  f"{MESH_PATH}/rf_link2.rf_link2",          "RfLeg1Mesh"),
    ("RbLeg1Mesh",  f"{MESH_PATH}/rb_link1.rb_link1",          "BaseMesh"),
    ("RbLeg2Mesh",  f"{MESH_PATH}/rb_link2.rb_link2",          "RbLeg1Mesh"),
    ("LfLeg1Mesh",  f"{MESH_PATH}/lf_link1.lf_link1",          "BaseMesh"),
    ("LfLeg2Mesh",  f"{MESH_PATH}/lf_link2.lf_link2",          "LfLeg1Mesh"),
    ("LbLeg1Mesh",  f"{MESH_PATH}/lb_link1.lb_link1",          "BaseMesh"),
    ("LbLeg2Mesh",  f"{MESH_PATH}/lb_link2.lb_link2",          "LbLeg1Mesh"),
]

def log(msg):
    unreal.log("[PuppyMeshes] " + str(msg))

def err(msg):
    unreal.log_error("[PuppyMeshes] " + str(msg))

def main():
    log("=== Adding PuppyPi mesh components to BP_PuppyRobotPawn ===")

    # 1. 检查蓝图是否存在
    if not unreal.EditorAssetLibrary.does_asset_exist(BP_PATH):
        err(f"Blueprint not found: {BP_PATH}")
        err("Create BP_PuppyRobotPawn first (run ue_auto_setup.py Step7)")
        return

    # 2. 加载蓝图
    bp = unreal.EditorAssetLibrary.load_asset(BP_PATH)
    if bp is None:
        err(f"Failed to load blueprint: {BP_PATH}")
        return
    log(f"Loaded blueprint: {bp.get_name()} (type={bp.get_class().get_name()})")

    # 3. 获取蓝图的 SimpleConstructionScript (SCS) 节点
    #    这是蓝图中定义组件的地方
    try:
        scs = bp.get_editor_property("simple_construction_script")
        log(f"SCS obtained: {scs is not None}")
    except Exception as e:
        err(f"Failed to get SCS: {e}")
        return

    # 4. 获取当前 SCS 中已有的节点名称 (避免重复添加)
    existing_nodes = set()
    try:
        all_nodes = scs.get_editor_property("all_nodes")
        for node in all_nodes:
            existing_nodes.add(str(node.get_editor_property("component_template").get_name())
                               if node.get_editor_property("component_template") else "")
    except Exception as e:
        log(f"Could not enumerate existing nodes (may be empty): {e}")

    log(f"Existing SCS nodes: {existing_nodes}")

    # 5. 先获取各父节点的组件模板引用
    #    SCS 中通过节点索引来引用父组件
    parent_node_map = {}

    # 6. 为每个 Mesh 创建 SCS 节点
    for comp_name, mesh_path, parent_name in MESH_LIST:
        if comp_name in existing_nodes:
            log(f"  SKIP (already exists): {comp_name}")
            continue

        # 加载 StaticMesh 资产
        mesh_asset = unreal.EditorAssetLibrary.load_asset(mesh_path)
        if mesh_asset is None:
            err(f"  SKIP (mesh not found): {mesh_path}")
            continue
        log(f"  Mesh loaded: {mesh_path} ({mesh_asset.get_name()})")

        # 找到父节点的组件模板
        parent_template = None
        if parent_name == "CollisionCapsule":
            # 尝试在 SCS 节点中找 CollisionCapsule
            try:
                for node in all_nodes:
                    tmpl = node.get_editor_property("component_template")
                    if tmpl and tmpl.get_name() == "CollisionCapsule":
                        parent_template = tmpl
                        break
            except Exception:
                pass
            # 如果找不到, 用蓝图默认组件的方式获取
            if parent_template is None:
                try:
                    cpp_pawn = unreal.load_class(None, "/Script/PuppyNav.PuppyRobotPawn")
                    if cpp_pawn:
                        # 直接获取 CDO 上的 CollisionCapsule 子组件
                        for prop_name in dir(unreal.get_default_object(cpp_pawn)):
                            pass
                except Exception:
                    pass
        else:
            # 从已创建的节点中找
            if parent_name in parent_node_map:
                parent_template = parent_node_map[parent_name]
            else:
                # 从 all_nodes 中按名字找
                try:
                    for node in all_nodes:
                        tmpl = node.get_editor_property("component_template")
                        if tmpl and tmpl.get_name() == parent_name:
                            parent_template = tmpl
                            break
                except Exception:
                    pass

        if parent_template is None:
            err(f"  WARNING: Could not find parent '{parent_name}' for {comp_name}, "
                f"attaching to root instead")
            # 不阻塞, 继续创建

        # 创建 StaticMeshComponent 节点
        try:
            new_node = scs.add_node(unreal.SCS_Node)
            new_node.set_editor_property("component_template", mesh_asset)
            new_node.set_editor_property("internal_component_template", mesh_asset)
            log(f"  Created node for {comp_name} (mesh={mesh_asset.get_name()})")
        except Exception as e:
            err(f"  Failed to create node for {comp_name}: {e}")
            continue

        # 记录模板引用供后续子节点使用
        parent_node_map[comp_name] = mesh_asset

    # 7. 编译蓝图
    log("Compiling blueprint...")
    try:
        unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        log("Blueprint compiled OK")
    except Exception as e:
        err(f"Compile failed: {e}")
        # 继续保存

    # 8. 保存蓝图
    try:
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
        unreal.EditorAssetLibrary.save_directory("/Game/", True)
        log("Blueprint saved OK")
    except Exception as e:
        err(f"Save failed: {e}")

    # 9. 验证
    try:
        bp_reloaded = unreal.EditorAssetLibrary.load_asset(BP_PATH)
        scs_new = bp_reloaded.get_editor_property("simple_construction_script")
        all_new = scs_new.get_editor_property("all_nodes")
        log(f"VERIFIED: {len(all_new)} nodes in SCS now")
        for node in all_new:
            tmpl = node.get_editor_property("component_template")
            if tmpl:
                log(f"  Node: {tmpl.get_name()} (class={tmpl.get_class().get_name()})")
    except Exception as e:
        err(f"Verification failed: {e}")

    log("=== Done! Open BP_PuppyRobotPawn in editor to verify. ===")
    log("If meshes show as None in Details panel, manually select /Game/PuppyPi/ assets.")

if __name__ == "__main__":
    main()
