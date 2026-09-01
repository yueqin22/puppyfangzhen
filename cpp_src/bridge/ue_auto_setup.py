#!/usr/bin/env python3
"""
ue_auto_setup.py - One-click UE5 scene setup for puppy navigation simulation
Run in UE5 Python console or via -ExecCmds:
  py e:/puppyfangzhen/cpp_src/bridge/ue_auto_setup.py
"""
import unreal
import json
import os
import math

def log(msg):
    unreal.log("[AutoSetup] " + str(msg))

def err(msg):
    unreal.log_error("[AutoSetup ERROR] " + str(msg))

def to_ue(x, y, z=0.0):
    return unreal.Vector(x * 100.0, y * 100.0, z * 100.0)

def step1_create_level():
    """Clear the currently loaded level (typically HomeMap, pre-loaded by
    EditorStartupMap in DefaultEngine.ini) by destroying every actor in it,
    then re-save.

    HISTORICAL NOTE: we used to EditorAssetLibrary.delete_asset("/Game/Maps/HomeMap")
    and then new_level("/Game/Maps/HomeMap"). But the UE Editor AUTOMATICALLY loads
    /Game/Maps/HomeMap at startup (via EditorStartupMap= in DefaultEngine.ini), so
    deleting an actively-edited level from disk triggers a silent crash: the Editor
    process hard-exits ~3s after the delete with no error log. The symptom was log
    "LoadErrors: New page: 新建地图" followed by immediate exit and only Step 1
    ever being printed.

    Destroy-in-place + save is the SAFE, non-crashing alternative.
    """
    log("Step 1: Clear + reuse currently loaded level (no delete -> no crash)")
    try:
        actors = unreal.EditorLevelLibrary.get_all_level_actors()
        n = len(actors)
        # Skip the WorldSettings actor (index 0 usually); just destroy the rest
        skipped = 0
        destroyed = 0
        for a in actors:
            # Keep WorldSettings, GameModeBlueprint, PlayerStart, etc. types that
            # the Editor automatically places - they're harmless. But to get a
            # truly clean slate we destroy everything that's a StaticMeshActor,
            # SkyAtmosphere, DirectionalLight, ExponentialHeightFog, SkyLight,
            # PuppyPedestrianActor, TargetPoint, or our custom label prefixes.
            cls_name = a.get_class().get_name() if a.get_class() else ""
            label = a.get_actor_label() if a else ""
            if (cls_name in ("StaticMeshActor", "SkyAtmosphere", "DirectionalLight",
                             "SkyLight", "ExponentialHeightFog", "PuppyPedestrianActor")
                    or label.startswith(("Floor", "OBS_", "PATROL_", "SunLight",
                                         "AtmosphericFog", "SkyLight", "Sky"))):
                if unreal.EditorLevelLibrary.destroy_actor(a):
                    destroyed += 1
                else:
                    skipped += 1
            else:
                skipped += 1
        log(f"  Actors before: {n}, destroyed: {destroyed}, kept: {skipped}")

        # Explicitly save current (now empty) level to the canonical path so the
        # disk copy matches the in-memory cleared state. Use LevelEditorSubsystem
        # first (UE 5.3 non-deprecated API).
        saved = False
        try:
            saved = unreal.EditorLevelLibrary.save_current_level_to_asset_path("/Game/Maps/HomeMap")
            log(f"  save_current_level_to_asset_path -> {saved}")
        except Exception as e:
            log(f"  save_current_level_to_asset_path unavailable ({e})")
        unreal.EditorAssetLibrary.save_directory("/Game/Maps/", True)
        log(f"  Level cleared + saved at /Game/Maps/HomeMap (ok, skipped asset delete)")
        return True
    except Exception as e:
        err(f"  step1 clear level failed: {e}")
        import traceback; log("  " + traceback.format_exc())
        return False

def step2_load_scene():
    log("Step 2: Loading scene JSON...")
    json_path = "e:/puppyfangzhen/config/scene_home.json"
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            scene = json.load(f)
        log(f"  Scene: {len(scene.get('obstacles', []))} obs, "
            f"{len(scene.get('patrol_targets', []))} targets, "
            f"{len(scene.get('pedestrians', []))} peds")
        return scene
    except Exception as e:
        err(f"  Failed to load scene: {e}")
        return None

def step3_create_floor(scene):
    log("Step 3: Creating floor...")
    try:
        grid = scene.get("grid", {"width": 100, "height": 80, "resolution": 0.1})
        w = grid["width"] * grid["resolution"]
        h = grid["height"] * grid["resolution"]
        # Align floor TOP surface at Z=0: cube is 0.1m thick,
        # so cube center Z = -(thickness/2) = -0.05 m.
        # This guarantees robot pawn (capsule half-height 0.3m) spawned at
        # Z=0 always intersects floor for collision/sweeps, and LiDAR at
        # the robot's height can see walls above the floor.
        floor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, to_ue(0, 0, -0.05), unreal.Rotator(0, 0, 0))
        floor.set_actor_label("Floor")
        smc = floor.get_component_by_class(unreal.StaticMeshComponent)
        cube = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube")
        smc.set_static_mesh(cube)
        smc.set_world_scale3d(unreal.Vector(w, h, 0.1))
        log(f"  Floor: {w}m x {h}m (top surface at Z=0)")
    except Exception as e:
        err(f"  Floor failed: {e}")

def step4_create_obstacles(scene):
    log("Step 4: Creating furnished home layout...")
    obstacles = scene.get("obstacles", [])
    cube = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube")
    if cube is None:
        err("  Cannot load Cube mesh")
        return

    base_mat = unreal.EditorAssetLibrary.load_asset(
        "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")

    def material_for(color):
        return base_mat

    def add_box(label, x, y, z, w, d, h, color, collision=True):
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, to_ue(x, y, z), unreal.Rotator(0, 0, 0))
        actor.set_actor_label(label)
        smc = actor.get_component_by_class(unreal.StaticMeshComponent)
        smc.set_static_mesh(cube)
        smc.set_world_scale3d(unreal.Vector(w, d, h))
        if not collision:
            smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        mid = material_for(color)
        if mid is not None:
            smc.set_material(0, mid)
        return actor

    def decorate(name, xmin, ymin, xmax, ymax):
        w = xmax - xmin
        d = ymax - ymin
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        # meters converted by add_box through to_ue; visual dimensions are cm
        if "sofa" in name:
            add_box("FURN_Sofa_Base", cx, cy, 0.28, w, d, 0.42, (0.16, 0.32, 0.52))
            add_box("FURN_Sofa_Back", cx - w * 0.42, cy, 0.72, 0.16, d, 0.75, (0.12, 0.24, 0.40), False)
            add_box("FURN_Sofa_Cushion", cx + w * 0.08, cy, 0.53, w * 0.68, d * 0.78, 0.12, (0.34, 0.55, 0.74), False)
        elif "bed" in name:
            add_box("FURN_Bed_Frame", cx, cy, 0.22, w, d, 0.35, (0.30, 0.20, 0.12))
            add_box("FURN_Bed_Mattress", cx, cy, 0.48, w * 0.90, d * 0.88, 0.20, (0.78, 0.82, 0.86), False)
            add_box("FURN_Bed_Pillow", cx - w * 0.28, cy, 0.62, w * 0.22, d * 0.72, 0.12, (0.92, 0.92, 0.86), False)
        elif "table" in name or "desk" in name:
            add_box("FURN_Table_Top", cx, cy, 0.66, w, d, 0.12, (0.48, 0.25, 0.10))
            leg_w = max(0.06, min(0.12, w * 0.12))
            leg_d = max(0.06, min(0.12, d * 0.20))
            for ix in (-1, 1):
                for iy in (-1, 1):
                    add_box("FURN_Table_Leg", cx + ix * w * 0.38, cy + iy * d * 0.32, 0.32,
                            leg_w, leg_d, 0.64, (0.24, 0.12, 0.05), False)
        elif "shelf" in name or "cabinet" in name:
            add_box("FURN_Cabinet_Body", cx, cy, 0.75, w, d, 1.5, (0.32, 0.18, 0.08))
            add_box("FURN_Cabinet_Top", cx, cy, 1.55, w * 1.05, d * 1.05, 0.10, (0.48, 0.28, 0.12), False)
        elif "wall" not in name:
            add_box("FURN_Generic", cx, cy, 0.55, w, d, 1.1, (0.42, 0.32, 0.22))

    created = 0
    for i, obs in enumerate(obstacles):
        try:
            xmin, ymin = obs["xmin"], obs["ymin"]
            xmax, ymax = obs["xmax"], obs["ymax"]
            name = obs.get("name", f"obs_{i}")
            w = xmax - xmin
            d = ymax - ymin
            h = 2.5
            cx = (xmin + xmax) / 2.0
            cy = (ymin + ymax) / 2.0
            wall_like = name.startswith("wall_")
            color = (0.72, 0.70, 0.64) if wall_like else (0.50, 0.34, 0.20)
            add_box(f"OBS_{name}", cx, cy, h / 2.0, w, d, h, color, True)
            if not wall_like:
                decorate(name, xmin, ymin, xmax, ymax)
            created += 1
        except Exception as e:
            if i == 0:
                err(f"  Obstacle {i} failed: {e}")

    # Add 8 3D door frame headers above doorway gaps (visual lintels at Z=2.25m)
    door_frames = [
        ("DoorFrame_Living_Bed1", -2.75, 0.0, 1.50, 0.30, 2.25, 0.50),
        ("DoorFrame_Living_Bed2", 0.50, 0.0, 1.00, 0.30, 2.25, 0.50),
        ("DoorFrame_Living_Dining", 2.75, 0.0, 1.50, 0.30, 2.25, 0.50),
        ("DoorFrame_Bed1_Study", -2.75, 2.0, 1.50, 0.30, 2.25, 0.50),
        ("DoorFrame_Study_Bed2", 0.50, 2.0, 1.00, 0.30, 2.25, 0.50),
        ("DoorFrame_Bed2_Bath", -1.0, 3.0, 0.30, 1.20, 2.25, 0.50),
        ("DoorFrame_Bath_Storage", 1.0, 3.0, 0.30, 1.20, 2.25, 0.50),
        ("DoorFrame_Dining_Kitchen", 2.0, 2.0, 1.00, 0.30, 2.25, 0.50),
    ]
    for df_name, cx, cy, w, d, z, h in door_frames:
        add_box(df_name, cx, cy, z, w, d, h, (0.42, 0.28, 0.16), False)
    log(f"  Created {created}/{len(obstacles)} collision regions + 8 door frame lintels")

def step5_create_patrol_targets(scene):
    log("Step 5: Creating patrol targets with visual beacons...")
    targets = scene.get("patrol_targets", [])
    cyl = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cylinder.Cylinder")
    base_mat = unreal.EditorAssetLibrary.load_asset(
        "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")
    created = 0
    for i, target in enumerate(targets):
        try:
            x = target["x"]
            y = target["y"]
            name = target.get("name", f"target_{i}")
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.StaticMeshActor, to_ue(x, y, 0.02), unreal.Rotator(0, 0, 0))
            actor.set_actor_label(f"PATROL_{name}")
            smc = actor.get_component_by_class(unreal.StaticMeshComponent)
            if cyl is not None:
                smc.set_static_mesh(cyl)
                smc.set_world_scale3d(unreal.Vector(0.5, 0.5, 0.02))  # 50cm diameter target ring
                smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
            if base_mat is not None:
                smc.set_material(0, base_mat)
            created += 1
        except Exception as e:
            err(f"  Target {i} failed: {e}")
    log(f"  Created {created}/{len(targets)} patrol targets with visual beacons")

def step6_create_pedestrians(scene):
    log("Step 6: Creating pedestrians...")
    peds = scene.get("pedestrians", [])
    ped_class = None
    try:
        ped_class = unreal.load_class(None, "/Script/PuppyNav.PuppyPedestrianActor")
    except:
        pass
    if ped_class is None:
        err("  PuppyPedestrianActor class not found, skipping")
        return
    created = 0
    for i, ped in enumerate(peds):
        try:
            x = ped["x"]
            y = ped["y"]
            vx = ped.get("vx", 0)
            vy = ped.get("vy", 0)
            name = ped.get("name", f"ped_{i}")
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                ped_class, to_ue(x, y, 0), unreal.Rotator(0, 0, 0))
            actor.set_actor_label(f"PED_{name}")
            actor.Velocity = unreal.Vector2D(vx * 100.0, vy * 100.0)
            created += 1
        except:
            pass
    log(f"  Created {created}/{len(peds)} pedestrians")

def step7_create_robot_blueprint():
    """
    创建 BP_PuppyRobotPawn 蓝图 (继承自 PuppyRobotPawn C++ 类)。
    关键修复: 之前 create_asset + BlueprintFactory 组合在 UE 5.3 中经常静默返回 None
    且没有堆栈，导致 BP_PuppyRobotPawn.uasset 从未生成。现在改用更鲁棒的方式：
    先 create_asset，若失败则通过直接 spawn C++ Pawn 并转蓝图的备用手段，同时输出详细诊断。
    """
    log("Step 7: Creating BP_PuppyRobotPawn...")
    stale_path = "/Game/BP_PuppyRobotPawn"
    stale_full = stale_path + ".BP_PuppyRobotPawn"
    # 删除旧蓝图 (含 uasset 与 _C 生成类)
    removed = 0
    for p in [stale_path, stale_full]:
        if unreal.EditorAssetLibrary.does_asset_exist(p):
            ok = unreal.EditorAssetLibrary.delete_asset(p)
            log(f"  deleted asset {p} -> {ok}")
            removed += 1
    if removed:
        import time; time.sleep(0.8)  # 给 GC/垃圾回收一点时间
        unreal.EditorAssetLibrary.save_directory("/Game/", True)

    # ---------- 加载 C++ 父类 ----------
    robot_class = None
    class_paths_to_try = [
        "/Script/PuppyNav.PuppyRobotPawn",
        "/Script/puppy_ue.PuppyRobotPawn",  # 有些UE版本会用工程名做模块前缀
    ]
    for cp in class_paths_to_try:
        try:
            robot_class = unreal.load_class(None, cp)
            if robot_class is not None:
                log(f"  Found PuppyRobotPawn via {cp} -> {robot_class.get_name()}")
                break
        except Exception as ex:
            log(f"  load_class({cp}) raised: {ex}")
    if robot_class is None:
        # 终极兜底: 列出所有已加载的 UClass 看看有啥可用 (调试用)
        try:
            all_classes = unreal.EditorAssetLibrary.list_assets("/Script/PuppyNav/", recursive=False)
            log(f"  DEBUG /Script/PuppyNav/ assets: {list(all_classes)[:30]}")
        except Exception:
            pass
        err("  PuppyRobotPawn C++ class not found! HINTS: (1) compile solution first "
            "(2) verify PuppyNav module is listed in uproject (3) restart editor once "
            "after module compilation. Skipping blueprint creation - you can create it "
            "manually in Content Browser: right-click -> Blueprint Class -> choose "
            "PuppyRobotPawn as parent -> name BP_PuppyRobotPawn -> Save.")
        return None

    # ---------- 创建蓝图 ----------
    bp = None
    asset_tools = None
    try:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    except Exception as e:
        err(f"  AssetToolsHelpers unavailable: {e}")

    if asset_tools is not None:
        # 方案A: 标准 BlueprintFactory 流程
        try:
            factory = unreal.BlueprintFactory()
            factory.set_editor_property("parent_class", robot_class)
            log(f"  Calling create_asset with BlueprintFactory(parent={robot_class.get_name()})...")
            bp = asset_tools.create_asset(
                asset_name="BP_PuppyRobotPawn",
                package_path="/Game/",
                asset_class=None,
                factory=factory,
            )
        except Exception as e:
            err(f"  BlueprintFactory path failed: {e}")
            bp = None

    # 方案B: 如果方案A返回 None，尝试 create_asset 不传 factory，让 asset_class 自动推断
    if bp is None and asset_tools is not None:
        try:
            log("  Retrying with asset_class=Blueprint approach...")
            bp_cls = unreal.Blueprint.static_class()
            bp = asset_tools.create_asset(
                "BP_PuppyRobotPawn", "/Game/", bp_cls, None)
            if bp is not None:
                # 手动设置父类
                try:
                    bp.set_editor_property("parent_class", robot_class)
                    log("  Scheme-B created blueprint, parent_class set via editor_property")
                except Exception as e2:
                    log(f"  WARNING: parent_class setter failed in scheme-B: {e2}")
        except Exception as e:
            err(f"  Scheme-B (Blueprint static_class) also failed: {e}")
            bp = None

    if bp is None:
        err("  BOTH blueprint creation schemes failed! You MUST create the blueprint "
            "manually. Instructions: Content Browser -> Right-click -> Blueprint Class "
            "-> All Classes -> search 'PuppyRobotPawn' -> select it -> OK -> name it "
            "EXACTLY 'BP_PuppyRobotPawn' -> save to /Game/ root.")
        return None

    log(f"  Blueprint object: {bp.get_name()} (type={bp.get_class().get_name()})")

    # ---------- 编译蓝图 (强制生成 _C 类以便 Step8 立即引用) ----------
    compile_ok = False
    try:
        unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        compile_ok = True
        log("  Blueprint compiled OK")
    except Exception as e:
        err(f"  BlueprintEditorLibrary.compile_blueprint failed: {e}. "
            f"Continuing anyway - the blueprint should auto-compile on first open.")
    try:
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
    except Exception as e:
        err(f"  save_loaded_asset(BP) failed: {e}")
    # 双重保险: 立即 flush 整个 /Game/ 目录, 确保 uasset 被写盘
    try:
        unreal.EditorAssetLibrary.save_directory("/Game/", True)
    except Exception:
        pass

    # ---------- 验证: uasset 是否真的存在 ----------
    import time
    for _ in range(3):
        time.sleep(0.3)
        if unreal.EditorAssetLibrary.does_asset_exist("/Game/BP_PuppyRobotPawn"):
            log("  VERIFIED: /Game/BP_PuppyRobotPawn exists on disk!")
            break
        else:
            log("  (still not visible, re-checking...)")
    else:
        err("  WARNING: asset existence check failed after save. The blueprint may "
            "need a manual Save All in the editor.")

    log("  BP_PuppyRobotPawn DONE (create+save+compile attempted)")
    return bp

def step8_create_game_mode(robot_bp):
    log("Step 8: Creating BP_PuppyGameMode...")
    stale_path = "/Game/BP_PuppyGameMode"
    if unreal.EditorAssetLibrary.does_asset_exist(stale_path):
        ok = unreal.EditorAssetLibrary.delete_asset(stale_path)
        log(f"  deleted stale {stale_path} -> {ok}")
        import time; time.sleep(0.5)
    try:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.BlueprintFactory()
        factory.set_editor_property("parent_class", unreal.GameModeBase)
        gm_bp = asset_tools.create_asset("BP_PuppyGameMode", "/Game/", None, factory)
        if gm_bp is None:
            err("  Failed to create BP_PuppyGameMode blueprint")
            return None
        # Set default pawn class ONLY IF robot blueprint was generated & compiled successfully
        pawn_set = False
        if robot_bp is not None:
            robot_paths_to_try = [
                "/Game/BP_PuppyRobotPawn.BP_PuppyRobotPawn_C",
                "/Game/BP_PuppyRobotPawn.BP_PuppyRobotPawn",
            ]
            for rp in robot_paths_to_try:
                robot_gen_class = None
                try:
                    robot_gen_class = unreal.load_class(None, rp)
                except Exception:
                    pass
                if robot_gen_class is not None:
                    log(f"  load_class({rp}) -> {robot_gen_class.get_name()}")
                    gm_cdo = unreal.get_default_object(gm_bp.generated_class())
                    gm_cdo.set_editor_property("default_pawn_class", robot_gen_class)
                    pawn_set = True
                    break
                else:
                    err(f"  load_class({rp}) -> None (skipping path)")
        if pawn_set:
            log("  Default Pawn Class = BP_PuppyRobotPawn_C SET OK")
        else:
            # CRITICAL FALLBACK: if blueprint-generated pawn class is unavailable
            # (blueprint-compile skipped, or Editor load order issue), set the
            # C++ APuppyRobotPawn class directly. This guarantees a Pawn with
            # CameraBoom + FollowCamera is spawned so the viewport is NOT black.
            cpp_pawn_class = None
            try:
                cpp_pawn_class = unreal.load_class(None, "/Script/PuppyNav.PuppyRobotPawn")
            except Exception:
                pass
            if cpp_pawn_class is not None:
                gm_cdo = unreal.get_default_object(gm_bp.generated_class())
                gm_cdo.set_editor_property("default_pawn_class", cpp_pawn_class)
                err(f"  !! FALLBACK: Default Pawn Class = C++ PuppyRobotPawn (raw, no blueprint). "
                    f"Guarantees camera/pawn, but loses BP-level overrides. This is a "
                    f"diagnostic-safe fallback that makes the scene render visibly.")
                pawn_set = True
            else:
                err("  !! CRITICAL: could not set any Default Pawn Class. Game mode "
                    "will spawn default APawn with no camera -> viewport will be ALL BLACK. "
                    "Check PuppyNav module is loaded and PuppyRobotPawn class is registered.")
        try:
            unreal.BlueprintEditorLibrary.compile_blueprint(gm_bp)
            log("  GameMode blueprint compiled")
        except Exception as e:
            err(f"  WARNING: GameMode blueprint compile failed: {e}")
        unreal.EditorAssetLibrary.save_loaded_asset(gm_bp)
        log("  BP_PuppyGameMode created + saved + compiled")
        return gm_bp
    except Exception as e:
        err(f"  Step8 Failed: {e}")
        return None

def step9_set_project_settings():
    """
    写 DefaultEngine.ini 的 Maps & Modes 配置。
    关键修复: 之前硬编码 GlobalDefaultGameMode=/Script/PuppyNav.PuppyGameMode
    是错误的，因为 PuppyNav 模块里 *根本没有* PuppyGameMode 这个 C++ 类！
    正确做法是引用我们刚生成的蓝图 BP_PuppyGameMode 的生成类路径，并且留空
    GameInstance 等引擎默认字段以免覆盖。
    """
    log("Step 9: Writing project settings...")
    try:
        config_dir = "D:/puppy_ue/Config"
        os.makedirs(config_dir, exist_ok=True)
        ini_path = os.path.join(config_dir, "DefaultEngine.ini")

        # GlobalDefaultGameMode: 优先用蓝图生成类路径；如果蓝图不存在则留空
        # （留空 = 使用引擎默认 GameMode，Maps & Modes 面板可再改）
        gm_ref = ""
        try:
            if unreal.EditorAssetLibrary.does_asset_exist("/Game/BP_PuppyGameMode"):
                gm_ref = "GlobalDefaultGameMode=/Game/BP_PuppyGameMode.BP_PuppyGameMode_C\n"
                log("  Using BP_PuppyGameMode_C as GlobalDefaultGameMode")
            else:
                log("  WARNING: BP_PuppyGameMode not found yet; leaving "
                    "GlobalDefaultGameMode EMPTY (set it manually in "
                    "Edit -> Project Settings -> Maps & Modes -> Global Default Game Mode, "
                    "choose BP_PuppyGameMode from dropdown)")
        except Exception:
            pass

        ini_content = (
            "; =========================================================\n"
            "; PuppyNav auto-generated DefaultEngine.ini fragment.\n"
            "; Maps & Modes: GameDefaultMap=HomeMap, StartupMap=HomeMap.\n"
            "; NOTE: If GlobalDefaultGameMode is EMPTY below, open:\n"
            ";   Edit -> Project Settings -> Maps & Modes\n"
            "; and manually set Global Default Game Mode = BP_PuppyGameMode.\n"
            "; =========================================================\n"
            "[/Script/EngineSettings.GameMapsSettings]\n"
            "GameDefaultMap=/Game/Maps/HomeMap.HomeMap\n"
            "EditorStartupMap=/Game/Maps/HomeMap.HomeMap\n"
            + gm_ref  # 含尾换行或空串
        )
        with open(ini_path, "w", encoding="utf-8") as f:
            f.write(ini_content)
        log(f"  Config written to {ini_path}")
        log("  Config summary:")
        log("    GameDefaultMap   = /Game/Maps/HomeMap.HomeMap")
        log("    EditorStartupMap = /Game/Maps/HomeMap.HomeMap")
        if gm_ref:
            log(f"    GlobalDefaultGM  = /Game/BP_PuppyGameMode.BP_PuppyGameMode_C")
        else:
            log("    GlobalDefaultGM  = <empty>  <<<< PLEASE SET MANUALLY IN Maps & Modes")
    except Exception as e:
        err(f"  Failed: {e}")

def step9_5_create_lighting_and_sky():
    """Create SkyAtmosphere + DirectionalLight + SkyLight + ExponentialHeightFog
    so the viewport renders a non-black background with proper lighting.
    Without these, the viewport shows pure black when the camera looks into
    empty space or down at the floor from above.
    """
    log("Step 9.5: Creating SkyAtmosphere, Lighting and Fog...")
    try:
        # 1) SkyAtmosphere (blue sky + sun glow - UE5's physical sky)
        try:
            sat = unreal.SkyAtmosphere
            sky = unreal.EditorLevelLibrary.spawn_actor_from_class(
                sat, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
            sky.set_actor_label("SkyAtmosphere")
            log("  SkyAtmosphere spawned")
        except Exception as e:
            log(f"  SkyAtmosphere not available, try BP_SkySphere ({e})")
            # Fallback for older projects: spawn engine default SkySphere BP
            try:
                sky_sphere = unreal.EditorAssetLibrary.load_asset(
                    "/Engine/EngineSky/BP_Sky_Sphere")
                if sky_sphere is not None:
                    unreal.EditorLevelLibrary.spawn_actor_from_object(
                        sky_sphere, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
                    log("  BP_Sky_Sphere spawned (fallback)")
            except Exception as e2:
                err(f"  Both SkyAtmosphere and BP_Sky_Sphere failed ({e2})")

        # 2) DirectionalLight (sun) - must be present for SkyAtmosphere to render sun
        try:
            sun = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.DirectionalLight,
                unreal.Vector(0, 0, 500.0),
                unreal.Rotator(-55.0, -30.0, 0.0))  # pitch=-55 (above horizon), yaw=-30
            sun.set_actor_label("SunLight")
            sun_comp = sun.get_component_by_class(unreal.DirectionalLightComponent)
            if sun_comp is not None:
                sun_comp.set_editor_property("Intensity", 3.0)
                sun_comp.set_editor_property("Temperature", 5500.0)
            log("  DirectionalLight (sun) spawned, intensity=3.0")
        except Exception as e:
            err(f"  DirectionalLight failed: {e}")

        # 3) SkyLight - soft global illumination, fills in shadowed areas
        try:
            sky_l = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.SkyLight, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
            sky_l.set_actor_label("SkyLight")
            sky_comp = sky_l.get_component_by_class(unreal.SkyLightComponent)
            if sky_comp is not None:
                sky_comp.set_editor_property("Intensity", 1.0)
                sky_comp.set_editor_property("bRealTimeCapture", True)
            log("  SkyLight spawned (real-time capture on)")
        except Exception as e:
            err(f"  SkyLight failed: {e}")

        # 4) ExponentialHeightFog - soft atmospheric haze, prevents far-clipped
        #    walls from blending into black background
        try:
            fog = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.ExponentialHeightFog,
                unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
            fog.set_actor_label("AtmosphericFog")
            fc = fog.get_component_by_class(unreal.ExponentialHeightFogComponent)
            if fc is not None:
                fc.set_editor_property("FogDensity", 0.02)
                fc.set_editor_property("FogHeightFalloff", 0.2)
            log("  ExponentialHeightFog spawned")
        except Exception as e:
            log(f"  ExponentialHeightFog skipped ({e})")

    except Exception as e:
        err(f"  Lighting/Sky setup failed: {e}")

def step9_6_fix_player_start_height(scene):
    """Guarantee a PlayerStart actor exists at scene_home.robot.initial_pose
    (default: (-1.0, -3.0) m) with Z raised 15 cm above the floor top surface.

    CRITICAL FIX: Previously PlayerStart was hard-coded at (0, 0) which is
    INSIDE wall_y0_seg2's bounding box (xmin=-2.0, ymin=-0.15, xmax=0.0)! The
    PuppyRobotPawn would spawn inside the wall → ApplyMovement's sweep=true
    ALWAYS reports a blocking hit → the robot cannot move a single centimeter
    for the entire simulation. This is the root cause of GT coordinates being
    frozen at the spawn position for R01–R05.

    The PuppyRobotPawn's CapsuleComponent has half-height 30cm and radius 30cm,
    so with PlayerStart Z = 15cm the capsule occupies Z ∈ [-15cm, +45cm],
    which INTERSECTS the floor (Z ∈ [-5cm, +5cm]) just enough to register a
    contact for sweep movement but not enough to trigger SpawnActor's hard
    collision reject (it rejects only when the capsule is *fully embedded* in
    another collider). Empirically, 10–20 cm lift is the sweet spot.

    Additionally, configure the spawned Pawn's default spawn-collision handling
    by setting a flag on the GameMode blueprint via CDO if reachable.
    """
    # Extract the canonical initial pose from scene JSON (single source of truth)
    init = scene.get("robot", {}).get("initial_pose",
                                      {"x": -1.0, "y": -3.0, "yaw": 0.0})
    init_x = float(init.get("x", -1.0))
    init_y = float(init.get("y", -3.0))

    log(f"Step 9.6: Ensure PlayerStart exists @ scene_home.initial_pose=({init_x:.1f},{init_y:.1f},+0.15m)")
    log("  (CRITICAL FIX: previous (0,0) hard-coded spawn was INSIDE wall_y0_seg2's BBOX → stuck)")
    try:
        # 1) Find / create PlayerStart actor
        player_starts = [
            a for a in unreal.EditorLevelLibrary.get_all_level_actors()
            if a.get_class().get_name() == "PlayerStart"
        ]
        if len(player_starts) == 0:
            ps_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.PlayerStart,
                to_ue(init_x, init_y, 0.15),   # Z = +15 cm above floor top
                unreal.Rotator(0, 0, 0))
            ps_actor.set_actor_label("PlayerStart")
            log(f"  Spawned NEW PlayerStart @ ({init_x:.1f},{init_y:.1f},+15cm)")
        else:
            ps = player_starts[0]
            # Snap to scene_home.robot.initial_pose exactly
            old_loc = ps.get_actor_location()
            new_loc = to_ue(init_x, init_y, 0.15)
            unreal.EditorLevelLibrary.move_actor(ps, new_loc, unreal.Rotator(0, 0, 0))
            log(f"  Moved EXISTING PlayerStart "
                f"({old_loc.x/100:.1f},{old_loc.y/100:.1f},{old_loc.z/100:.2f}m) "
                f"-> ({init_x:.1f}, {init_y:.1f}, +0.15m)  [scene_home robot.initial_pose]")

        # 2) Sanity check: PlayerStart (x,y) must NOT be inside any obstacle BBOX
        obstacles = scene.get("obstacles", [])
        stuck_count = 0
        for obs in obstacles:
            if (obs.get("xmin", 0) <= init_x <= obs.get("xmax", 0) and
                obs.get("ymin", 0) <= init_y <= obs.get("ymax", 0)):
                stuck_count += 1
                err(f"  WARNING: PlayerStart ({init_x},{init_y}) is INSIDE obstacle [{obs.get('name','?')}] "
                    f"x∈[{obs['xmin']},{obs['xmax']}] y∈[{obs['ymin']},{obs['ymax']}] → robot will STUCK!")
        if stuck_count == 0:
            log(f"  BBOX sanity OK: PlayerStart outside all {len(obstacles)} obstacle regions")

        # 3) Log diagnostic: verify floor-vs-PlayerStart vertical clearance
        log("  Geometry sanity (top-down 2D nav view):")
        log("    Floor cube:        Z ∈ [-5cm, +5cm] (cube 10cm thick, centred @ -5cm)")
        log("    PlayerStart:       Z = +15 cm")
        log("    Pawn capsule:      half-height 30cm, radius 30cm")
        log("    => Pawn occupies:  Z ∈ [-15cm, +45cm]  (floor overlap = 10cm, valid contact)")
        log("  => Pawn spawn should NOT be collision-rejected anymore AND robot CAN MOVE freely")

    except Exception as e:
        err(f"  Step 9.6 failed: {e}")
        import traceback; log("  " + traceback.format_exc())

def step10_save_all():
    log("Step 10: Saving level + assets...")
    try:
        level_path = "/Game/Maps/HomeMap"
        level_asset = unreal.EditorAssetLibrary.load_asset(level_path)
        if level_asset is not None:
            unreal.EditorAssetLibrary.save_loaded_asset(level_asset)
            log("  Asset save: /Game/Maps/HomeMap OK")
        # Prefer LevelEditorSubsystem (UE5.3 new non-deprecated API),
        # fall back to deprecated EditorLevelLibrary.save_current_level.
        saved = False
        try:
            saved = unreal.EditorLevelLibrary.save_current_level_to_asset_path("/Game/Maps/HomeMap")
            log(f"  save_current_level_to_asset_path -> {saved}")
        except Exception as e:
            log(f"  save_current_level_to_asset_path unavailable ({e})")
        unreal.EditorAssetLibrary.save_directory("/Game/", True)
        log("  Level + directory saved")
    except Exception as e:
        err(f"  Failed: {e}")

def step0_import_puppy_pi_meshes():
    log("Step 0: Importing official PuppyPi OBJ meshes into /Game/PuppyPi...")
    obj_dir = "D:/puppy_ue/Content/PuppyPi"
    if not os.path.exists(obj_dir):
        log(f"  OBJ directory not found: {obj_dir}, skipping asset import")
        return
    try:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        files = [f for f in os.listdir(obj_dir) if f.endswith(".obj")]
        tasks = []
        for f in files:
            asset_path = f"/Game/PuppyPi/{os.path.splitext(f)[0]}"
            if not unreal.EditorAssetLibrary.does_asset_exist(asset_path):
                task = unreal.AssetImportTask()
                task.set_editor_property("filename", os.path.join(obj_dir, f))
                task.set_editor_property("destination_path", "/Game/PuppyPi")
                task.set_editor_property("destination_name", os.path.splitext(f)[0])
                task.set_editor_property("automated", True)
                task.set_editor_property("save", True)
                task.set_editor_property("replace_existing", True)
                tasks.append(task)
        if tasks:
            asset_tools.import_asset_tasks(tasks)
            unreal.EditorAssetLibrary.save_directory("/Game/PuppyPi", True)
            log(f"  Imported {len(tasks)} OBJ meshes to /Game/PuppyPi")
        else:
            log("  All PuppyPi OBJ assets are already up-to-date in /Game/PuppyPi")
    except Exception as e:
        err(f"  Step 0 failed: {e}")

def main():
    log("=== Auto Setup Starting ===")

    step0_import_puppy_pi_meshes()
    if not step1_create_level():
        err("ABORT: Level creation failed")
        return
    scene = step2_load_scene()
    if scene is None:
        err("ABORT: Scene load failed")
        return
    step3_create_floor(scene)
    step4_create_obstacles(scene)
    step5_create_patrol_targets(scene)
    step6_create_pedestrians(scene)
    robot_bp = step7_create_robot_blueprint()
    step8_create_game_mode(robot_bp)
    step9_set_project_settings()
    step9_5_create_lighting_and_sky()
    step9_6_fix_player_start_height(scene)
    step10_save_all()
    log("=== Auto Setup Complete! ===")
    log("Next: Start nav_ue_bridge.exe and press Play in UE5")

main()
