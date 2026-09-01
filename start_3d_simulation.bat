@echo off
chcp 65001 >nul
echo ============================================================
echo   Puppy Robot + MiniCPM-RobotTrack 3D 物理仿真启动器
echo ============================================================
echo.
echo 正在启动 WSL2 (Ubuntu-22.04) 并运行 3D 物理仿真环境...
echo 提示:
echo   - Gazebo 窗口将显示为 [Gazebo (Ubuntu-22.04)]
echo   - RViz2 窗口将显示机器人 3D 模型、激光雷达点云与前置相机画面
echo.

wsl.exe -d Ubuntu-22.04 bash -c "chmod +x /mnt/e/puppyfangzhen/run_simulation_wsl.sh && /mnt/e/puppyfangzhen/run_simulation_wsl.sh small_room"

pause
