@echo off
REM Standalone unittest runner — mirrors build.bat unittest logic without relying on build.bat encoding
setlocal enabledelayedexpansion

set "SIM_DIR=%~dp0"
for %%I in ("%SIM_DIR%..") do set "SRC_DIR=%%~fI"
set "BUILD_DIR=%SIM_DIR%build"

REM Locate vcvars64
set "VCVARS="
for %%Y in (Community Professional Enterprise BuildTools) do (
    if exist "C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
)
if "!VCVARS!"=="" ( echo [ERROR] VS2022 not found & exit /b 1 )

echo [1/3] vcvars64 ...
call "!VCVARS!" >nul 2>&1

echo [2/3] cmake configure/build (BUILD_TESTING=ON) ...
cmake -S "%SRC_DIR%." -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
if errorlevel 1 exit /b 1
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b 1

echo [3/3] ctest ...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
set "CTEST_EXIT=%ERRORLEVEL%"
if %CTEST_EXIT%==0 ( echo [OK] All unit tests passed ) else ( echo [FAIL] ctest exit=%CTEST_EXIT% )
exit /b %CTEST_EXIT%
