@echo off
REM Standalone build runner — only builds sim_test.exe (no tests)
setlocal enabledelayedexpansion
set "SIM_DIR=%~dp0"
for %%I in ("%SIM_DIR%..") do set "SRC_DIR=%%~fI"
set "BUILD_DIR=%SIM_DIR%build"

set "VCVARS="
for %%Y in (Community Professional Enterprise BuildTools) do (
    if exist "C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
)
if "!VCVARS!"=="" ( echo [ERROR] VS2022 not found & exit /b 1 )

REM clean option: first arg = "clean"
if /i "%1"=="clean" ( if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%" )

echo [1/2] Init vcvars & cmake configure ...
call "!VCVARS!" >nul 2>&1
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    cmake -S "%SRC_DIR%." -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release
    if errorlevel 1 exit /b 1
)

echo [2/2] Build sim_test ...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b 1

REM Copy sim_test.exe to sim dir
set "SIM_TEST_EXE="
for %%P in (
  "%BUILD_DIR%\sim\Release\sim_test.exe"
  "%BUILD_DIR%\Release\sim_test.exe"
  "%BUILD_DIR%\sim_test.exe"
) do ( if exist "%%~P" set "SIM_TEST_EXE=%%~P" )
if defined SIM_TEST_EXE (
    copy /y "%SIM_TEST_EXE%" "%SIM_DIR%sim_test.exe" >nul
    echo [OK] sim_test.exe copied to sim dir
    exit /b 0
) else (
    echo [ERROR] sim_test.exe not found after build
    exit /b 1
)
