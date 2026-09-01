"""
capture_ue_screenshot.py -- Capture viewport screenshot of UE5 HomeMap level
"""
import os
import unreal

def capture():
    level_path = "/Game/Maps/HomeMap"
    world = unreal.EditorLevelLibrary.get_editor_world()
    
    # Spawn a camera actor above the scene looking down at 45 degrees
    cam_loc = unreal.Vector(0.0, -100.0, 800.0)  # 8m above ground
    cam_rot = unreal.Rotator(-60.0, 90.0, 0.0)   # Looking down
    
    cam = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.CineCameraActor if hasattr(unreal, "CineCameraActor") else unreal.CameraActor,
        cam_loc, cam_rot
    )
    if cam:
        cam.set_actor_label("DIAG_OverviewCamera")
        print("[Capture] Overview camera spawned at", cam_loc)
    
    # Take high res screenshot
    out_dir = r"C:\Users\Administrator\.gemini\antigravity\brain\9f43f43f-3ac5-4496-9072-844d8264c252"
    filename = os.path.join(out_dir, "ue_home_scene_overview.png")
    
    unreal.AutomationLibrary.take_high_res_screenshot(1920, 1080, filename)
    print(f"[Capture] High-res screenshot saved to {filename}")

if __name__ == "__main__":
    capture()
