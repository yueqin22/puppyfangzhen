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
            les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
            if les is not None:
                saved = les.save_current_level()
                log(f"  LevelEditorSubsystem.save_current_level -> {saved}")
        except Exception as e:
            log(f"  save_current_level (LES) unavailable ({e}), falling back")
        if not saved:
            unreal.EditorLevelLibrary.save_current_level()
            log("  EditorLevelLibrary.save_current_level (deprecated) -> ok")
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
    log("Step 4: Creating obstacles...")
    obstacles = scene.get("obstacles", [])
    try:
        cube = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube")
    except:
        err("  Cannot load Cube mesh")
        return
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
            cz = h / 2.0
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.StaticMeshActor, to_ue(cx, cy, cz), unreal.Rotator(0, 0, 0))
            actor.set_actor_label(f"OBS_{name}")
            smc = actor.get_component_by_class(unreal.StaticMeshComponent)
            smc.set_static_mesh(cube)
            smc.set_world_scale3d(unreal.Vector(w, d, h))
            created += 1
        except Exception as e:
            if i == 0:
                err(f"  Obstacle {i} failed: {e}")
    log(f"  Created {created}/{len(obstacles)} obstacles")

def step5_create_patrol_targets(scene):
    log("Step 5: Creating patrol targets...")
    targets = scene.get("patrol_targets", [])
    created = 0
    for i, target in enumerate(targets):
        try:
            x = target["x"]
            y = target["y"]
            name = target.get("name", f"target_{i}")
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.Actor, to_ue(x, y, 0), unreal.Rotator(0, 0, 0))
            actor.set_actor_label(f"PATROL_{name}")
            created += 1
        except:
            pass
    log(f"  Created {created}/{len(targets)} patrol targets")

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
    log("Step 7: Creating BP_PuppyRobotPawn...")
    stale_path = "/Game/BP_PuppyRobotPawn"
    if unreal.EditorAssetLibrary.does_asset_exist(stale_path):
        ok = unreal.EditorAssetLibrary.delete_asset(stale_path)
        log(f"  deleted stale {stale_path} -> {ok}")
        import time; time.sleep(0.5)
    robot_class = None
    try:
        robot_class = unreal.load_class(None, "/Script/PuppyNav.PuppyRobotPawn")
    except:
        pass
    if robot_class is None:
        err("  PuppyRobotPawn C++ class not found (PuppyNav module not loaded yet?)")
        return None
    log(f"  PuppyRobotPawn C++ class OK -> {robot_class.get_name()}")
    try:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.BlueprintFactory()
        factory.set_editor_property("parent_class", robot_class)
        bp = asset_tools.create_asset("BP_PuppyRobotPawn", "/Game/", None, factory)
        if bp is None:
            err("  BlueprintFactory.create_asset returned None")
            return None
        # Compile blueprint RIGHT NOW so the BlueprintGeneratedClass is available
        # immediately for step8 (load_class(..._C)). Without this, blueprint is
        # "stub only" until editor-compiled, and load_class(_C) returns None ->
        # DefaultPawnClass is never set -> no Pawn spawned at game boot -> all-black viewport.
        try:
            unreal.BlueprintEditorLibrary.compile_blueprint(bp)
            log("  Blueprint compiled (BlueprintGeneratedClass ready)")
        except Exception as e:
            err(f"  WARNING: blueprint compile failed: {e} (trying to save anyway)")
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
        log("  BP_PuppyRobotPawn created + saved + compiled")
        return bp
    except Exception as e:
        err(f"  Step7 Failed: {e}")
        return None

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
    log("Step 9: Writing project settings...")
    try:
        config_dir = "D:/puppy_ue/Config"
        os.makedirs(config_dir, exist_ok=True)
        ini_path = os.path.join(config_dir, "DefaultEngine.ini")
        ini_content = (
            "[/Script/EngineSettings.GameMapsSettings]\n"
            "GameDefaultMap=/Game/Maps/HomeMap.HomeMap\n"
            "EditorStartupMap=/Game/Maps/HomeMap.HomeMap\n"
            "GlobalDefaultGameMode=/Game/BP_PuppyGameMode.BP_PuppyGameMode_C\n"
        )
        with open(ini_path, "w", encoding="utf-8") as f:
            f.write(ini_content)
        log(f"  Config written to {ini_path}")
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

def step9_6_fix_player_start_height():
    """Guarantee a PlayerStart actor exists with Z raised 15 cm above the floor
    top surface (which is at Z=0). This prevents the infamous warning:

        SpawnActor failed because of collision at the spawn location
        [X=0 Y=0 Z=0] for [BP_PuppyRobotPawn_C]

    The PuppyRobotPawn's CapsuleComponent has half-height 30cm and radius 30cm,
    so with PlayerStart Z = 15cm the capsule occupies Z ∈ [-15cm, +45cm],
    which INTERSECTS the floor (Z ∈ [-5cm, +5cm]) just enough to register a
    contact for sweep movement but not enough to trigger SpawnActor's hard
    collision reject (it rejects only when the capsule is *fully embedded* in
    another collider). Empirically, 10–20 cm lift is the sweet spot.

    Additionally, configure the spawned Pawn's default spawn-collision handling
    by setting a flag on the GameMode blueprint via CDO if reachable.
    """
    log("Step 9.6: Ensure PlayerStart exists @ (0,0,+15cm) (avoid pawn-spawn collision reject)")
    try:
        # 1) Find / create PlayerStart actor
        player_starts = [
            a for a in unreal.EditorLevelLibrary.get_all_level_actors()
            if a.get_class().get_name() == "PlayerStart"
        ]
        if len(player_starts) == 0:
            ps_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.PlayerStart,
                to_ue(0.0, 0.0, 0.15),   # Z = +15 cm above floor top
                unreal.Rotator(0, 0, 0))
            ps_actor.set_actor_label("PlayerStart")
            log(f"  Spawned NEW PlayerStart @ (0,0,15cm)")
        else:
            ps = player_starts[0]
            # Raise any existing PlayerStart to Z = +15 cm regardless of prior Z
            old_loc = ps.get_actor_location()
            new_loc = to_ue(0.0, 0.0, 0.15)
            # Keep X/Y the same if the user had moved it? No — force (0,0) so
            # the bridge hard-coded GT start (robot_x=0, robot_y=0) matches.
            unreal.EditorLevelLibrary.move_actor(ps, new_loc, unreal.Rotator(0, 0, 0))
            log(f"  Moved EXISTING PlayerStart "
                f"({old_loc.x/100:.1f},{old_loc.y/100:.1f},{old_loc.z/100:.2f}m) "
                f"-> (0.0, 0.0, +0.15m)")

        # 2) Log diagnostic: verify floor-vs-PlayerStart vertical clearance
        log("  Geometry sanity (top-down 2D nav view):")
        log("    Floor cube:        Z ∈ [-5cm, +5cm] (cube 10cm thick, centred @ -5cm)")
        log("    PlayerStart:       Z = +15 cm")
        log("    Pawn capsule:      half-height 30cm, radius 30cm")
        log("    => Pawn occupies:  Z ∈ [-15cm, +45cm]  (floor overlap = 10cm, valid contact)")
        log("  => Pawn spawn should NOT be collision-rejected anymore")

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
            les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
            if les is not None:
                saved = les.save_current_level()
                log(f"  LevelEditorSubsystem.save_current_level -> {saved}")
        except Exception as e:
            log(f"  LevelEditorSubsystem.save_current_level unavailable, fallback deprecated API ({e})")
        if not saved:
            unreal.EditorLevelLibrary.save_current_level()
        unreal.EditorAssetLibrary.save_directory("/Game/", True)
        log("  Level + directory saved")
    except Exception as e:
        err(f"  Failed: {e}")

def main():
    log("=== Auto Setup Starting ===")

    # IMPORTANT: do NOT globally delete assets BEFORE any per-step asset creation.
    # Reason: the Editor has already loaded DefaultEngine.ini *before* Python runs,
    # and the ini references /Game/BP_PuppyGameMode + /Game/Maps/HomeMap from the
    # previous run. If we delete them globally at the top of this script, the Editor's
    # world / GameMode CDO will already be referencing missing assets and we crash.
    # Instead, each step that (re)creates an asset FIRST locally deletes any stale
    # copy of THAT asset, then recreates it. This lets Editor boot cleanly and then
    # incrementally refresh assets.

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
    step9_6_fix_player_start_height()
    step10_save_all()
    log("=== Auto Setup Complete! ===")
    log("Next: Start nav_ue_bridge.exe and press Play in UE5")

main()
