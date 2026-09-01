@echo off
chcp 65001 >nul
echo ========================================
echo  Starting PuppyPi UE Home Simulation
echo ========================================

echo [1/2] Launching UE5 game...
start "PuppyUE" "C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe" "D:\puppy_ue\puppy_ue.uproject" -game -map=/Game/Maps/HomeMap -windowed -resx=1280 -resy=720 -log

echo [2/2] Waiting 15 seconds for UE to load...
ping 127.0.0.1 -n 16 >nul

echo [3/3] Starting navigation bridge...
e:\puppyfangzhen\cpp_src\bridge\build\Release\Release\nav_ue_bridge.exe --port 7777 --scene e:\puppyfangzhen\config\scene_home.json

echo.
echo Simulation ended.
pause
