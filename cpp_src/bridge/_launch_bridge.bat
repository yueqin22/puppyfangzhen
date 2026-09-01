@echo off
setlocal enabledelayedexpansion
set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "!VCVARS!" set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
call "!VCVARS!" >nul 2>&1
set "SRC=e:\puppyfangzhen\cpp_src"
cl.exe /nologo /EHsc /std:c++17 /O2 /W3 /I"%SRC%\bridge" /I"%SRC%\puppy_nav_core\include" /I"%SRC%\sim" /D_CRT_SECURE_NO_WARNINGS "%SRC%\bridge\nav_ue_bridge.cpp" "%SRC%\puppy_nav_core\src\occupancy_grid.cpp" "%SRC%\puppy_nav_core\src\costmap.cpp" "%SRC%\puppy_nav_core\src\astar_planner.cpp" "%SRC%\puppy_nav_core\src\amcl.cpp" /Fe:"%SRC%\bridge\nav_ue_bridge.exe" /link ws2_32.lib
