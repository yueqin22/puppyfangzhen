@echo off
REM apply_collision_fix.bat - 一键应用碰撞检测修复并重新编译
REM 用法: 双击运行 或 在PowerShell中执行 .\apply_collision_fix.bat
setlocal

echo ============================================================
echo  R22 Collision Fix - Applying patched PuppyRobotPawn.cpp
echo ============================================================

set "SRC=e:\puppyfangzhen\cpp_src\bridge\PuppyRobotPawn_patched.cpp"
set "DST=D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"

echo [1/4] Copying patched file...
copy /Y "%SRC%" "%DST%" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Copy failed! Try manually:
    echo   copy "%SRC%" "%DST%"
    pause
    exit /b 1
)
echo   OK - Copied to %DST%

echo [2/4] Cleaning conflicting files in Source\puppy_ue...
if exist "D:\puppy_ue\Source\puppy_ue\PuppyRobotPawn.cpp" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyRobotPawn.cpp" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyRobotPawn.h" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyRobotPawn.h" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyLiDARComponent.cpp" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyLiDARComponent.cpp" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyLiDARComponent.h" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyLiDARComponent.h" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyTcpServer.cpp" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyTcpServer.cpp" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyTcpServer.h" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyTcpServer.h" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyPedestrianActor.cpp" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyPedestrianActor.cpp" 2>nul
if exist "D:\puppy_ue\Source\puppy_ue\PuppyPedestrianActor.h" del /Q "D:\puppy_ue\Source\puppy_ue\PuppyPedestrianActor.h" 2>nul
echo   OK - Conflicting files removed

echo [3/4] Building UE project (this may take 2-5 minutes)...
"C:\Program Files\Epic Games\UE_5.3\Engine\Build\BatchFiles\Build.bat" puppy_ue Win64 Development -Project="D:\puppy_ue\puppy_ue.uproject" -WaitMutex
if errorlevel 1 (
    echo [ERROR] Build failed! Check errors above.
    pause
    exit /b 1
)
echo   OK - Build successful

echo [4/4] Done!
echo.
echo ============================================================
echo  Fix applied! Now run the simulation:
echo    1. Start bridge: e:\puppyfangzhen\cpp_src\bridge\build\Release\Release\nav_ue_bridge.exe --port 7777 --scene e:\puppyfangzhen\config\scene_home.json
echo    2. Start UE:      "C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe" "D:\puppy_ue\puppy_ue.uproject" -game -map=/Game/Maps/HomeMap -windowed -log
echo ============================================================
pause
endlocal
