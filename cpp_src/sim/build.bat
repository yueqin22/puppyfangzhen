@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cl.exe /O2 /std:c++17 /EHsc /utf-8 /MT /D_CRT_SECURE_NO_WARNINGS /Fe:sim_test.exe main.cpp ..\puppy_nav_core\src\occupancy_grid.cpp ..\puppy_nav_core\src\costmap.cpp ..\puppy_nav_core\src\astar_planner.cpp ..\puppy_nav_core\src\amcl.cpp /I..\puppy_nav_core\include /I.
