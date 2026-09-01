"""
ue_setup_meshes.py - UE5 Python Editor Script
在编辑器模式下运行，为 BP_PuppyRobotPawn 蓝图添加 11 个 PuppyPi 机器狗 Mesh 组件。

运行方式: UnrealEditor.exe project.uproject -run=PythonScript -script="ue_setup_meshes.py"
或在 Python 控制台: py e:/puppyfangzhen/cpp_src/bridge/ue_setup_meshes.py
"""
import unreal

BP_PATH = "/Game/BP_PuppyRobotPawn"
PAWN_CLASS_PATH = "/Script/PuppyNav.PuppyRobotPawn"
MESH_DIR = "/Game/PuppyPi"

MESH_DEFS = [
    ("BaseMesh",   f"{MESH_DIR}/base_link.base_link",         "Root"),
    ("LidarMesh",  f"{MESH_DIR}/lidar_Link.lidar_Link",       "BaseMesh"),
    ("CameraMesh", f"{MESH_DIR}/camera_link.camera_link",     "BaseMesh"),
    ("RfLeg1Mesh", f"{MESH_DIR}/rf_link1.rf_link1",          "BaseMesh"),
    ("RfLeg2Mesh", f"{MESH_DIR}/rf_link2.rf_link2",          "RfLeg1Mesh"),
    ("RbLeg1Mesh", f"{MESH_DIR}/rb_link1.rb_link1",          "BaseMesh"),
    ("RbLeg2Mesh", f"{MESH_DIR}/rb_link2.rb_link2",          "RbLeg1Mesh"),
    ("LfLeg1Mesh", f"{MESH_DIR}/lf_link1.lf_link1",          "BaseMesh"),
    ("LfLeg2Mesh", f"{MESH_DIR}/lf_link2.lf_link2",          "LfLeg1Mesh"),
    ("LbLeg1Mesh", f"{MESH_DIR}/lb_link1.lb_link1",          "BaseMesh"),
    ("LbLeg2Mesh", f"{MESH_DIR}/lb_link2.lb_link2",          "LbLeg1Mesh"),
]

def log(msg):
    unreal.log("[PuppySetup] " + str(msg))

def err(msg):
    unreal.log_error("[PuppySetup] " + str(msg))

def find_or_create_blueprint():
    """Find existing BP or create a new one from PuppyRobotPawn C++ class."""
    if unreal.EditorAssetLibrary.does_asset_exist(BP_PATH):
        log(f"Blueprint exists: {BP_PATH}")
        return True

    log(f"Creating new BP from C++ class: {PAWN_CLASS_PATH}")
    pawn_class = unreal.load_class(None, PAWN_CLASS_PATH)
    if pawn_class is None:
        err(f"Cannot find C++ class: {PAWN_CLASS_PATH}")
        return False

    try:
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.BlueprintFactory()
        factory.set_editor_property("parent_class", pawn_class)
        bp = asset_tools.create_asset("BP_PuppyRobotPawn", "/Game/", None, factory)
        if bp is None:
            err("create_asset returned None")
            return False
        unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
        log("Blueprint created + compiled + saved")
        return True
    except Exception as e:
        err(f"Failed to create blueprint: {e}")
        return False

def add_mesh_components_to_bp():
    """Add 11 StaticMeshComponent nodes to the blueprint's SCS."""
    bp = unreal.EditorAssetLibrary.load_asset(BP_PATH)
    if bp is None:
        err(f"Cannot load {BP_PATH}")
        return False

    log(f"Loaded BP: {bp.get_name()}")

    # Get SCS
    try:
        scs = bp.get_editor_property("simple_construction_script")
    except Exception as e:
        err(f"Cannot get SCS: {e}")
        return False

    # Collect existing node template names to avoid duplicates
    existing_names = set()
    try:
        for node in scs.get_editor_property("all_nodes"):
            tmpl = node.get_editor_property("component_template")
            if tmpl:
                existing_names.add(tmpl.get_name())
    except Exception:
        pass

    log(f"Existing component templates: {existing_names}")

    # Track created templates for parent resolution
    created_templates = {}

    for comp_name, mesh_path, parent_name in MESH_DEFS:
        if comp_name in existing_names:
            log(f"  SKIP exists: {comp_name}")
            created_templates[comp_name] = None
            continue

        # Load mesh
        mesh = unreal.EditorAssetLibrary.load_asset(mesh_path)
        if mesh is None:
            err(f"  SKIP mesh not found: {mesh_path}")
            continue

        log(f"  Adding: {comp_name} <- {mesh_path} (parent: {parent_name})")

        # Create SCS node
        node = scs.add_node(unreal.SCS_Node)
        node.set_editor_property("component_template", mesh)
        # Also set internal_component_template for runtime use
        try:
            node.set_editor_property("internal_component_template", mesh)
        except Exception:
            pass

        # Set parent attachment
        if parent_name != "Root":
            parent_tmpl = None
            # Find parent in existing or created templates
            for n in scs.get_editor_property("all_nodes"):
                t = n.get_editor_property("component_template")
                if t and t.get_name() == parent_name:
                    parent_tmpl = t
                    break

            if parent_tmpl:
                node.set_editor_property("parent_component_template", parent_tmpl)
            else:
                log(f"    WARNING: parent '{parent_name}' not found, attaching to root")

        created_templates[comp_name] = mesh

    # Compile
    log("Compiling blueprint...")
    try:
        unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        log("Compiled OK")
    except Exception as e:
        err(f"Compile error (non-fatal): {e}")

    # Save
    try:
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
        unreal.EditorAssetLibrary.save_directory("/Game/", True)
        log("Saved OK")
    except Exception as e:
        err(f"Save error: {e}")

    # Verify
    try:
        bp2 = unreal.EditorAssetLibrary.load_asset(BP_PATH)
        scs2 = bp2.get_editor_property("simple_construction_script")
        count = len(list(scs2.get_editor_property("all_nodes")))
        log(f"VERIFIED: {count} SCS nodes total")
        for n in scs2.get_editor_property("all_nodes"):
            t = n.get_editor_property("component_template")
            if t:
                log(f"  -> {t.get_name()}")
    except Exception as e:
        err(f"Verify error: {e}")

    return True

def main():
    log("=" * 60)
    log("PuppyPi Mesh Setup for BP_PuppyRobotPawn")
    log("=" * 60)

    # Step 1: Create or find blueprint
    if not find_or_create_blueprint():
        err("Blueprint setup failed, aborting")
        return

    # Step 2: Add mesh components
    if not add_mesh_components_to_bp():
        err("Mesh component setup failed")
        return

    log("=" * 60)
    log("Setup complete!")
    log("Next: Open BP_PuppyRobotPawn in editor, verify meshes, then Play.")
    log("=" * 60)

if __name__ == "__main__":
    main()
