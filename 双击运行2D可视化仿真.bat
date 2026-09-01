@echo off
chcp 65001 >nul
title 2D 多房间可视化仿真器
echo ============================================================
echo  正在启动 2D 多房间可视化仿真器...
echo  快捷键:
echo    1-0 键 : 切换 10 种导航避障算法 (1:STVOC, 4:VO, 5:RVO, 8:CBF, 0:A*+CBF)
echo    Space  : 暂停/继续
echo    F 键   : 加速 (x2 / x4 / x8)
echo    ESC    : 退出
echo ============================================================
cd /d e:\puppyfangzhen
python visual_sim.py
pause
