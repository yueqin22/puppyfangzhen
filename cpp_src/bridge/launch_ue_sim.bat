@echo off
setlocal EnableExtensions EnableDelayedExpansion
set "BRIDGE_DIR=%~dp0"
set "BRIDGE_EXE=%BRIDGE_DIR%build\Release\Release\nav_ue_bridge.exe"
set "PORT=7777"
set "FRAMES=0"
set "SEED=1"
set "SCENE=%BRIDGE_DIR%..\..\config\scene_home.json"
set "MODE=bridge"
set "UE_PROJECT=D:\puppy_ue\puppy_ue.uproject"
set "UE_EDITOR=C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe"

:parse
if "%~1"=="" goto start
if /I "%~1"=="--standalone" set "MODE=standalone"& shift& goto parse
if /I "%~1"=="--ue" set "MODE=ue"& shift& goto parse
if /I "%~1"=="--frames" set "FRAMES=%~2"& shift& shift& goto parse
if /I "%~1"=="--seed" set "SEED=%~2"& shift& shift& goto parse
if /I "%~1"=="--port" set "PORT=%~2"& shift& shift& goto parse
if /I "%~1"=="--scene" set "SCENE=%~2"& shift& shift& goto parse
shift
goto parse

:start
if not exist "%BRIDGE_EXE%" echo [ERROR] bridge executable missing& exit /b 1
if /I "%MODE%"=="standalone" goto standalone
if /I "%MODE%"=="ue" goto ue

echo [INFO] Waiting for UE client on port %PORT%
if "%FRAMES%"=="0" ("%BRIDGE_EXE%" --port %PORT% --scene "%SCENE%") else ("%BRIDGE_EXE%" --port %PORT% --frames %FRAMES% --scene "%SCENE%")
exit /b %ERRORLEVEL%

:standalone
if "%FRAMES%"=="0" set "FRAMES=36000"
"%BRIDGE_EXE%" --standalone --seed %SEED% --frames %FRAMES% --scene "%SCENE%"
exit /b %ERRORLEVEL%

:ue
if not exist "%UE_EDITOR%" echo [ERROR] UnrealEditor.exe missing& exit /b 1
if not exist "%UE_PROJECT%" echo [ERROR] UE project missing& exit /b 1
start "PuppyUE" "%UE_EDITOR%" "%UE_PROJECT%" -game -map=/Game/Maps/HomeMap -windowed -log
ping 127.0.0.1 -n 13 >nul
echo [INFO] Starting bridge against UE game
if "%FRAMES%"=="0" ("%BRIDGE_EXE%" --port %PORT% --scene "%SCENE%") else ("%BRIDGE_EXE%" --port %PORT% --frames %FRAMES% --scene "%SCENE%")
exit /b %ERRORLEVEL%