@echo off
REM build_bridge.bat - Build nav_ue_bridge via CMake + MSVC (Ninja)
setlocal enabledelayedexpansion
set "VCVARS="
for %%Y in (Community Professional Enterprise BuildTools) do (
  if exist "C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
  )
)
if "!VCVARS!"=="" ( echo [ERROR] VS2022 not found & exit /b 1 )
call "!VCVARS!" >nul 2>&1
set "NINJA_DIR=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
if exist "%NINJA_DIR%\ninja.exe" ( set "PATH=%NINJA_DIR%;%PATH%" )
set "SRC=%~dp0"
set "BUILD=%SRC%build"
if /i "%1"=="clean" ( if exist "%BUILD%" rmdir /s /q "%BUILD%" )
cmake -S "%SRC%." -B "%BUILD%" -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 ( echo [ERROR] CMake configure failed & exit /b 1 )
cmake --build "%BUILD%" --config Release
if errorlevel 1 ( echo [ERROR] Build failed & exit /b 1 )
echo.
echo [OK] nav_ue_bridge.exe built successfully
echo      Run: nav_ue_bridge.exe --port 7777 --scene ..\..\config\scene_home.json
endlocal
