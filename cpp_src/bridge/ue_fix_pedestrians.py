#!/usr/bin/env python3
"""ue_fix_pedestrians.py - Create pedestrians using direct class access"""
import unreal
import json

def log(msg):
    unreal.log("[FixPed] " + str(msg))

log("Creating pedestrians...")

# Load scene JSON
with open("e:/puppyfangzhen/config/scene_home.json", "r", encoding="utf-8") as f:
    scene = json.load(f)

peds = scene.get("pedestrians", [])

# Try multiple ways to get PuppyPedestrianActor class
ped_class = None

# Method 1: Direct attribute access
try:
    ped_class = unreal.PuppyPedestrianActor
    log(f"Method 1 (direct): {ped_class}")
except:
    pass

# Method 2: load_class
if ped_class is None:
    try:
        ped_class = unreal.load_class(None, "/Script/PuppyNav.PuppyPedestrianActor")
        log(f"Method 2 (load_class): {ped_class}")
    except:
        pass

# Method 3: find_object
if ped_class is None:
    try:
        ped_class = unreal.find_object(None, "/Script/PuppyNav.PuppyPedestrianActor")
        log(f"Method 3 (find_object): {ped_class}")
    except:
        pass

if ped_class is None:
    log("ERROR: Cannot find PuppyPedestrianActor class by any method")
    # List available PuppyNav classes
    try:
        for attr in dir(unreal):
            if "Puppy" in attr or "puppy" in attr:
                log(f"  Found: unreal.{attr}")
    except:
        pass
else:
    created = 0
    for i, ped in enumerate(peds):
        try:
            x = ped["x"]
            y = ped["y"]
            vx = ped.get("vx", 0)
            vy = ped.get("vy", 0)
            name = ped.get("name", f"ped_{i}")
            loc = unreal.Vector(x * 100.0, y * 100.0, 0.0)
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                ped_class, loc, unreal.Rotator(0, 0, 0))
            actor.set_actor_label(f"PED_{name}")
            actor.Velocity = unreal.Vector2D(vx * 100.0, vy * 100.0)
            created += 1
            log(f"  Created {name} at ({x}, {y})")
        except Exception as e:
            log(f"  Failed ped {i}: {e}")
    log(f"Created {created}/{len(peds)} pedestrians")

    # Save level
    try:
        unreal.EditorLevelLibrary.save_current_level()
        log("Level saved")
    except:
        pass

log("Done")
