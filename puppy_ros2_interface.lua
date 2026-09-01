-- Puppy 机械狗 - CoppeliaSim 最小脚本
-- 仅设置初始位置，所有 ROS2 通信由 Python ZMQ 桥接节点处理
-- （simROS2 插件在多订阅者下发布不可靠，改用 ZMQ Remote API）

PUPPY = {}

function sysCall_init()
    sim = require('sim')

    PUPPY.base_footprint = sim.getObject('/base_footprint')

    -- 机器人初始状态
    PUPPY.x = 1.0
    PUPPY.y = -2.0
    PUPPY.theta = 0.0

    -- 设置初始位置
    sim.setObjectPosition(PUPPY.base_footprint, -1, {PUPPY.x, PUPPY.y, 0.0})
    sim.setObjectOrientation(PUPPY.base_footprint, -1, {0, 0, PUPPY.theta})

    sim.addLog(sim.verbosity_msgs, "Puppy 最小脚本已初始化 (ROS2 由 Python ZMQ 桥接处理)")
end
