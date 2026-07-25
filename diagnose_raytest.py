#!/usr/bin/env python3
"""测试 proximity sensor 和 simGeom rayTest 方案"""
import sys

sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

print("=== 测试射线检测方案 ===")

# 方案1: 创建 proximity sensor 并测试
print("\n--- 方案1: 创建 ray-type proximity sensor ---")
lua_create_prox = """
-- 尝试创建 ray-type proximity sensor
-- sim.createProximitySensor(sensorType, subType, options, volumes, params, thresholds)
local sensorType = sim.proximitysensor_ray_type
local subType = sim.proximitysensor_ray_subtype

-- 查找常量值
local rayType = sim.proximitysensor_ray_type or -1
local raySubType = sim.proximitysensor_ray_subtype or -1

-- 尝试不同的参数格式
local ok, sensor = pcall(sim.createProximitySensor, rayType, subType, 0, 
    {0.1, 12.0, 0.0, 0.0, 0.0},  -- volumes
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  -- params  
    {0.0, 0.0, 0.0}  -- thresholds
)

return tostring(ok), tostring(sensor), tostring(rayType), tostring(raySubType)
"""
try:
    result = sim.executeLuaCode(lua_create_prox)
    print(f"创建结果: {result}")
except Exception as e:
    print(f"ERROR: {e}")

# 方案2: 使用 simGeom.createMesh + getMeshSegmentDistance
print("\n--- 方案2: simGeom mesh segment distance ---")
lua_simgeom_test = """
local ok, simGeom = pcall(require, 'simGeom')
if not ok or type(simGeom) ~= 'table' then return 'simGeom not available' end

-- 获取所有 shape 对象
local shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
if not shapes or #shapes == 0 then return 'no shapes' end

-- 获取第一个 shape 的位置和 mesh
local shape = shapes[1]
local shape_pos = sim.getObjectPosition(shape, -1)
local shape_ori = sim.getObjectOrientation(shape, -1)

-- 获取 mesh 数据
local vertices, indices = sim.getShapeMesh(shape)
if not vertices then return 'no mesh data' end

return 'shapes=' .. #shapes .. ', vertices=' .. #vertices/3 .. ', indices=' .. #indices/3
"""
try:
    result = sim.executeLuaCode(lua_simgeom_test)
    print(f"simGeom mesh 数据: {result}")
except Exception as e:
    print(f"ERROR: {e}")

# 方案3: 测试 sim.checkProximitySensorEx2
print("\n--- 方案3: sim.checkProximitySensorEx2 ---")
lua_check_prox = """
return tostring(type(sim.checkProximitySensorEx2)), tostring(type(sim.checkProximitySensorEx)), tostring(type(sim.checkProximitySensor))
"""
try:
    result = sim.executeLuaCode(lua_check_prox)
    print(f"函数类型: {result}")
except Exception as e:
    print(f"ERROR: {e}")

# 方案4: 测试 sim.checkDistance
print("\n--- 方案4: sim.checkDistance ---")
lua_check_dist = """
local base = sim.getObject('/base_footprint')
-- 获取场景中所有 shape
local shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
if not shapes or #shapes == 0 then return 'no shapes' end

-- 检查 base_footprint 到第一个 shape 的距离
local ok, dist, pt1, pt2 = pcall(sim.checkDistance, base, shapes[1], 100.0)
return tostring(ok), tostring(dist), tostring(pt1), tostring(pt2)
"""
try:
    result = sim.executeLuaCode(lua_check_dist)
    print(f"checkDistance 结果: {result}")
except Exception as e:
    print(f"ERROR: {e}")

# 方案5: 完整的 simGeom rayTest 实现
print("\n--- 方案5: simGeom 完整射线测试 ---")
lua_full_raytest = """
local ok, simGeom = pcall(require, 'simGeom')
if not ok then return 'require failed' end

-- 获取所有 shape
local shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
if not shapes then return 'no shapes' end

-- 创建 simGeom meshes
local meshes = {}
local mesh_count = 0

for i = 1, #shapes do
    local shape = shapes[i]
    -- 获取 shape 的世界变换矩阵
    local matrix = sim.getObjectMatrix(shape, -1)
    -- 获取 mesh 数据
    local vertices, indices = sim.getShapeMesh(shape)
    if vertices and indices and #vertices >= 3 then
        -- 转换顶点到世界坐标
        local world_verts = {}
        for j = 1, #vertices, 3 do
            local v = {vertices[j], vertices[j+1], vertices[j+2]}
            local wv = sim.multiplyVector(matrix, v)
            world_verts[#world_verts+1] = wv[1]
            world_verts[#world_verts+1] = wv[2]
            world_verts[#world_verts+1] = wv[3]
        end
        
        -- 创建 simGeom mesh
        local ok2, mesh = pcall(simGeom.createMesh, world_verts, indices)
        if ok2 and mesh then
            meshes[#meshes+1] = mesh
            mesh_count = mesh_count + 1
        end
    end
end

-- 测试一条射线
local laser = sim.getObject('/base_footprint/base_link/laser_link')
local laser_pos = sim.getObjectPosition(laser, -1)
local ray_start = laser_pos
local ray_end = {laser_pos[1] + 5.0, laser_pos[2], laser_pos[3]}

local min_dist = 12.0
for i = 1, #meshes do
    local ok3, dist = pcall(simGeom.getMeshSegmentDistance, meshes[i], ray_start, ray_end)
    if ok3 and dist and dist < min_dist then
        min_dist = dist
    end
end

return 'meshes=' .. mesh_count .. ', min_dist=' .. min_dist
"""
try:
    result = sim.executeLuaCode(lua_full_raytest)
    print(f"完整射线测试: {result}")
except Exception as e:
    print(f"ERROR: {e}")

print("\n=== 完成 ===")
