import unreal
import os

def import_obj_files():
    obj_dir = r"D:\puppy_ue\Content\PuppyPi"
    destination_path = "/Game/PuppyPi"

    if not os.path.exists(obj_dir):
        unreal.log_error(f"[ImportPuppy] Directory not found: {obj_dir}")
        return

    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    files = [f for f in os.listdir(obj_dir) if f.endswith('.obj')]
    
    tasks = []
    for f in files:
        file_path = os.path.join(obj_dir, f)
        task = unreal.AssetImportTask()
        task.set_editor_property("filename", file_path)
        task.set_editor_property("destination_path", destination_path)
        task.set_editor_property("destination_name", os.path.splitext(f)[0])
        task.set_editor_property("automated", True)
        task.set_editor_property("save", True)
        task.set_editor_property("replace_existing", True)
        tasks.append(task)

    if tasks:
        asset_tools.import_asset_tasks(tasks)
        unreal.log(f"[ImportPuppy] Successfully queued {len(tasks)} OBJ import tasks to {destination_path}")
        unreal.EditorAssetLibrary.save_directory(destination_path, True)

if __name__ == "__main__":
    import_obj_files()
