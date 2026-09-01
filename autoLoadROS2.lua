sim = require 'sim'

function sysCall_init()
    -- 自动加载 simROS2 插件
    local ok, err = pcall(function()
        simROS2 = require('simROS2')
    end)
    if ok then
        sim.addLog(sim.verbosity_msgs, "simROS2 plugin loaded successfully via autoLoadROS2 add-on!")
    else
        sim.addLog(sim.verbosity_errors, "Failed to load simROS2 plugin: " .. tostring(err))
    end
end

function sysCall_cleanup()
end
