@echo off
REM build.bat - C++ simulator build + test + regression validation (CMake-based)
REM Replaces hardcoded cl.exe command, supports any VS2022 SKU
REM Usage: build.bat [clean|test|unittest|validate|validate_full]
REM   no arg        - incremental build (sim_test.exe only)
REM   clean         - remove build dir then rebuild
REM   test          - build and run integration tests (astar_corridor / nav_integration)
REM   unittest      - build with BUILD_TESTING=ON, then run ctest (AMCL + OccupancyGrid)
REM   validate      - FULL PIPELINE QUICK: build → unittest → 3-seed(1,6,8) regression → baseline diff (~15min)
REM   validate_full - FULL PIPELINE:       build → unittest → 10-seed regression → baseline diff (~1.5h)
REM   clean_validate / clean_validate_full - same as above but start with a clean rebuild

setlocal enabledelayedexpansion

REM ---- Locate VS2022 vcvars64.bat ----
set "VCVARS="
for %%Y in (Community Professional Enterprise BuildTools) do (
    if exist "C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" (
        set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
    )
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" (
        set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
    )
)

if "!VCVARS!"=="" (
    echo [ERROR] Visual Studio 2022 not found. Install any VS2022 SKU.
    echo         Checked: C:\Program Files\Microsoft Visual Studio\2022\{Community,Professional,Enterprise,BuildTools}
    exit /b 1
)

echo [1/3] Initializing MSVC environment...
call "!VCVARS!" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] vcvars64.bat failed
    exit /b 1
)

REM ---- Check CMake ----
where cmake >nul 2>&1
if errorlevel 1 (
    echo [ERROR] cmake not found. Install CMake 3.8+ and add to PATH.
    exit /b 1
)

REM ---- Source dir is parent (cpp_src/, contains top-level CMakeLists.txt) ----
set "SIM_DIR=%~dp0"
for %%I in ("%SIM_DIR%..") do set "SRC_DIR=%%~fI"
set "BUILD_DIR=%SIM_DIR%build"

REM ---- clean option ----
if /i "%1"=="clean" (
    echo [2/3] Cleaning build directory...
    if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
)

REM ---- CMake configure (first time or after clean) ----
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [2/3] CMake configure...
    cmake -S "%SRC_DIR%." -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release
    if errorlevel 1 (
        echo [ERROR] CMake configure failed
        exit /b 1
    )
) else (
    echo [2/3] Using existing CMake cache
)

REM ---- Build ----
echo [3/3] Building sim_test...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 (
    echo [ERROR] Build failed
    exit /b 1
)

REM ---- Copy artifact to sim dir (for existing scripts using .\sim_test.exe) ----
REM  顶层 CMakeLists 做 add_subdirectory(sim)，产物在 build/sim/Release/ 下
set "SIM_TEST_EXE="
for %%P in (
    "%BUILD_DIR%\sim\Release\sim_test.exe"
    "%BUILD_DIR%\Release\sim_test.exe"
    "%BUILD_DIR%\sim_test.exe"
    "%BUILD_DIR%\sim\sim_test.exe"
) do (
    if exist "%%~P" set "SIM_TEST_EXE=%%~P"
)

if defined SIM_TEST_EXE (
    copy /y "%SIM_TEST_EXE%" "%SIM_DIR%sim_test.exe" >nul
    echo.
    echo [OK] Build succeeded: %SIM_DIR%sim_test.exe
    echo      Quick test: sim_test.exe 54000 27000 8
) else (
    echo [ERROR] sim_test.exe not found in build output
    exit /b 1
)

REM ---- test option: build and run integration tests ----
if /i "%1"=="test" (
    echo.
    echo ==== Running integration tests ====
    cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release -DBUILD_INTEGRATION_TESTS=ON
    cmake --build "%BUILD_DIR%" --config Release
    if exist "%BUILD_DIR%\Release\astar_corridor_test.exe" (
        "%BUILD_DIR%\Release\astar_corridor_test.exe"
    )
    if exist "%BUILD_DIR%\Release\nav_integration_test.exe" (
        "%BUILD_DIR%\Release\nav_integration_test.exe"
    )
    if exist "%BUILD_DIR%\astar_corridor_test.exe" (
        "%BUILD_DIR%\astar_corridor_test.exe"
    )
    if exist "%BUILD_DIR%\nav_integration_test.exe" (
        "%BUILD_DIR%\nav_integration_test.exe"
    )
)

REM ---- unittest option: build and run unit tests via ctest ----
if /i "%1"=="unittest" (
    echo.
    echo ==== Building unit tests ====
    cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
    if errorlevel 1 (
        echo [ERROR] CMake configure with BUILD_TESTING=ON failed
        exit /b 1
    )
    cmake --build "%BUILD_DIR%" --config Release
    if errorlevel 1 (
        echo [ERROR] Build failed
        exit /b 1
    )
    echo.
    echo ==== Running unit tests (ctest) ====
    ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
    if errorlevel 1 (
        echo [ERROR] Unit tests FAILED
        exit /b 1
    )
    echo [OK] All unit tests passed
)

REM ---- validate / validate_full options: dispatch to regression_pipeline.ps1 ----
set _PSARGS=
if /i "%1"=="validate"            ( set "_PSARGS=-Quick"         & goto :RUN_PIPELINE )
if /i "%1"=="clean_validate"      ( set "_PSARGS=-Quick -Clean"  & goto :RUN_PIPELINE )
if /i "%1"=="validate_full"       ( set "_PSARGS="               & goto :RUN_PIPELINE )
if /i "%1"=="clean_validate_full" ( set "_PSARGS=-Clean"         & goto :RUN_PIPELINE )
goto :SKIP_PIPELINE

:RUN_PIPELINE
echo.
echo ======================================================================
echo  Running regression pipeline: build.bat %1
echo  PowerShell args: %_PSARGS%
echo ======================================================================
REM Ensure PowerShell can run scripts (-ExecutionPolicy Bypass) and use 64-bit PS
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SIM_DIR%regression_pipeline.ps1" %_PSARGS%
set _PSEXIT=%ERRORLEVEL%
echo.
if %_PSEXIT%==0 (
    echo [VALIDATE OK] Pipeline finished successfully (exit 0)
) else (
    echo [VALIDATE FAIL] Pipeline exited with code %_PSEXIT%
)
exit /b %_PSEXIT%

:SKIP_PIPELINE

endlocal
