@echo off
chcp 65001 >nul
echo ============================================================
echo   MiniCPM-RobotTrack 视觉跟踪与任务编排节点启动器
echo ============================================================
echo.
echo 正在启动 MiniCPM 视觉大模型与任务编排节点...
echo.

wsl.exe -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source ~/puppy_ws/install/setup.bash && ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py mode:=sim camera_source:=sim"

pause
