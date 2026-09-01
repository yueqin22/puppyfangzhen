@echo off
chcp 65001 >nul
title UE5 三维联合仿真启动器
echo ============================================================
echo  正在启动 Unreal Engine 5 高保真 3D 联合仿真...
echo  (支持 360° 激光雷达、3D 动态行人与 A* + CBF 实时导航)
echo ============================================================
cd /d e:\puppyfangzhen\cpp_src\bridge
call launch_ue_sim.bat --ue
pause
