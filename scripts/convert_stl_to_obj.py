#!/usr/bin/env python3
"""
convert_stl_to_obj.py - Convert STL mesh files to Wavefront OBJ files.
Handles binary and ASCII STL formats.
"""
import os
import struct

def convert_stl_to_obj(stl_path, obj_path):
    with open(stl_path, 'rb') as f:
        header = f.read(80)
        num_triangles_bytes = f.read(4)
        if len(num_triangles_bytes) < 4:
            print(f"Error: {stl_path} is empty or corrupted")
            return False
        
        num_triangles = struct.unpack('<I', num_triangles_bytes)[0]
        
        # Simple heuristic to verify binary vs ASCII STL
        expected_size = 80 + 4 + num_triangles * 50
        file_size = os.path.getsize(stl_path)
        
        vertices = []
        faces = []

        if file_size == expected_size:
            # Binary STL
            for i in range(num_triangles):
                data = f.read(50)
                if len(data) < 50:
                    break
                # Normal (3 floats), Vert 1 (3 floats), Vert 2 (3 floats), Vert 3 (3 floats), attr (uint16)
                floats = struct.unpack('<12fH', data)
                v1 = floats[3:6]
                v2 = floats[6:9]
                v3 = floats[9:12]
                
                idx = len(vertices) + 1
                vertices.extend([v1, v2, v3])
                faces.append((idx, idx + 1, idx + 2))
        else:
            # ASCII STL
            f.seek(0)
            text = f.read().decode('utf-8', errors='ignore')
            current_verts = []
            for line in text.splitlines():
                line = line.strip()
                if line.startswith('vertex'):
                    parts = line.split()
                    if len(parts) >= 4:
                        current_verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                        if len(current_verts) == 3:
                            idx = len(vertices) + 1
                            vertices.extend(current_verts)
                            faces.append((idx, idx + 1, idx + 2))
                            current_verts = []

    os.makedirs(os.path.dirname(obj_path), exist_ok=True)
    with open(obj_path, 'w', encoding='utf-8') as f:
        f.write(f"# Converted from {os.path.basename(stl_path)}\n")
        f.write(f"g {os.path.splitext(os.path.basename(stl_path))[0]}\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")

    print(f"Converted: {os.path.basename(stl_path)} ({len(vertices)} verts, {len(faces)} faces) -> {obj_path}")
    return True

def main():
    src_dir = r"C:\Users\Administrator\.gemini\antigravity\brain\6b6f1636-d5ec-4d75-96ae-5554f8d8f5b7\scratch\PuppyPi\src\simulations\puppypi_description\meshes"
    out_dirs = [
        r"E:\puppyfangzhen\models\puppy_pi",
        r"D:\puppy_ue\Content\PuppyPi"
    ]
    
    if not os.path.exists(src_dir):
        print(f"Source dir does not exist: {src_dir}")
        return

    stl_files = [f for f in os.listdir(src_dir) if f.lower().endswith('.stl')]
    print(f"Found {len(stl_files)} STL files in {src_dir}")

    for out_dir in out_dirs:
        for stl in stl_files:
            stl_p = os.path.join(src_dir, stl)
            obj_name = os.path.splitext(stl)[0] + ".obj"
            obj_p = os.path.join(out_dir, obj_name)
            convert_stl_to_obj(stl_p, obj_p)

if __name__ == '__main__':
    main()
