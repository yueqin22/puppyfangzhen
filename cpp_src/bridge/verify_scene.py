# verify_scene.py — 验证 scene_home.json 与 simulation.h 场景定义等价性
# (UE-PLAN-20260803 M1.5: 几何等价性自动校验)
import json, re, sys

# 1. 加载 JSON
with open("../../config/scene_home.json", "r", encoding="utf-8") as f:
    scene = json.load(f)

json_obstacles = [(o["xmin"], o["ymin"], o["xmax"], o["ymax"]) for o in scene["obstacles"]]
json_patrols   = [(p["x"], p["y"]) for p in scene["patrol_targets"]]
json_peds      = [(p["x"], p["y"], p["vx"], p["vy"]) for p in scene["pedestrians"]]

print(f"JSON: {len(json_obstacles)} obstacles, {len(json_patrols)} patrols, {len(json_peds)} peds")

# 2. 从 simulation.h 提取 BBox (正则匹配 build_obstacles)
with open("../sim/simulation.h", "r", encoding="utf-8") as f:
    sim_lines = f.readlines()

# 提取 build_obstacles 中的 BBox 列表 (跳过注释行)
in_obstacles = False
cpp_obstacles = []
for line in sim_lines:
    if "build_obstacles" in line:
        in_obstacles = True
        continue
    if in_obstacles and "// ===== 行人" in line:
        break
    if in_obstacles and line.strip().startswith("//"):
        continue  # 跳过注释行
    if in_obstacles:
        for m in re.finditer(r'\{(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)\}', line):
            cpp_obstacles.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))

print(f"CPP:  {len(cpp_obstacles)} obstacles")

# 3. 逐项对比
mismatches = 0
for i, (j_obs, c_obs) in enumerate(zip(json_obstacles, cpp_obstacles)):
    if j_obs != c_obs:
        print(f"  MISMATCH #{i}: JSON={j_obs} CPP={c_obs}")
        mismatches += 1

if len(json_obstacles) != len(cpp_obstacles):
    print(f"  COUNT MISMATCH: JSON={len(json_obstacles)} CPP={len(cpp_obstacles)}")
    mismatches += 1

# 4. 巡逻点对比
in_patrols = False
cpp_patrols = []
for line in sim_lines:
    if "build_patrol_targets" in line:
        in_patrols = True
        continue
    if in_patrols and "// ===== 障碍物" in line:
        break
    if in_patrols and line.strip().startswith("//"):
        continue
    if in_patrols:
        m = re.search(r'PatrolTarget\{(-?[\d.]+),\s*(-?[\d.]+),\s*([^,]+),\s*"(\w+)"\}', line)
        if m:
            cpp_patrols.append((float(m.group(1)), float(m.group(2))))

print(f"CPP:  {len(cpp_patrols)} patrols")
if len(json_patrols) != len(cpp_patrols):
    print(f"  PATROL COUNT: JSON={len(json_patrols)} CPP={len(cpp_patrols)}")
    mismatches += 1
else:
    for i, (j_p, c_p) in enumerate(zip(json_patrols, cpp_patrols)):
        if abs(j_p[0] - c_p[0]) > 0.01 or abs(j_p[1] - c_p[1]) > 0.01:
            print(f"  PATROL MISMATCH #{i}: JSON={j_p} CPP={c_p}")
            mismatches += 1

# 5. 行人对比
in_peds = False
cpp_peds = []
for line in sim_lines:
    if "create_pedestrians" in line:
        in_peds = True
        continue
    if in_peds and "// 检查点是否在墙内" in line:
        break
    if in_peds and line.strip().startswith("//"):
        continue
    if in_peds:
        m = re.search(r'"(\w+)",\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)', line)
        if m:
            cpp_peds.append((float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))))

print(f"CPP:  {len(cpp_peds)} peds")
if len(json_peds) != len(cpp_peds):
    print(f"  PED COUNT: JSON={len(json_peds)} CPP={len(cpp_peds)}")
    mismatches += 1
else:
    for i, (j_p, c_p) in enumerate(zip(json_peds, cpp_peds)):
        if abs(j_p[0] - c_p[0]) > 0.01 or abs(j_p[1] - c_p[1]) > 0.01:
            print(f"  PED MISMATCH #{i}: JSON={j_p} CPP={c_p}")
            mismatches += 1

# 6. 结论
print(f"\n{'='*50}")
if mismatches == 0:
    print("PASS: scene_home.json 与 simulation.h 完全等价")
    sys.exit(0)
else:
    print(f"FAIL: {mismatches} 处不一致")
    sys.exit(1)
