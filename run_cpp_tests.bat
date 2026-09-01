@echo off
setlocal enabledelayedexpansion

set "ROOT_DIR=%~dp0"
set "CPP_SRC_DIR=%ROOT_DIR%cpp_src"
set "TESTS_DIR=%CPP_SRC_DIR%\tests"
set "BUILD_DIR=%TESTS_DIR%\build"
set "MODE=build_and_run"

if /i "%1"=="--build-only" set MODE=build_only
if /i "%1"=="--run-only" set MODE=run_only

echo ============================================================
echo  C++ ctest runner
echo  Mode: %MODE%
echo  Dir:  %TESTS_DIR%
echo  Build:%BUILD_DIR%
echo ============================================================

set "VCVARS="
for %%Y in (Community Professional Enterprise BuildTools) do (
    if exist "C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" (
        set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
    )
)
if "!VCVARS!"=="" (
    for %%Y in (Community Professional Enterprise BuildTools) do (
        if exist "C:\Program Files ^(x86^)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat" (
            set "VCVARS=C:\Program Files ^(x86^)\Microsoft Visual Studio\2022\%%Y\VC\Auxiliary\Build\vcvars64.bat"
        )
    )
)
if "!VCVARS!"=="" (
    echo [ERROR] Visual Studio 2022 vcvars64.bat not found
    exit /b 1
)

call "!VCVARS!" >nul 2>&1

set "NINJA_DIR=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
if exist "%NINJA_DIR%\ninja.exe" (
    set "PATH=%NINJA_DIR%;%PATH%"
)

if not "%MODE%"=="run_only" (
    echo [1/3] CMake configure Ninja...
    cmake -S "%TESTS_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release
    if errorlevel 1 (
        echo [ERROR] CMake configure failed
        exit /b 1
    )

    echo [2/3] Building test targets...
    cmake --build "%BUILD_DIR%" --config Release
    if errorlevel 1 (
        echo [ERROR] Build failed
        exit /b 1
    )
    echo [OK] Build succeeded
)

if "%MODE%"=="build_only" (
    echo [OK] Build-only mode complete.
    exit /b 0
)

echo [3/3] Running ctest...
cd /d "%BUILD_DIR%"
ctest -C Release --output-on-failure --timeout 60
set "CTEST_EXIT=%ERRORLEVEL%"

echo ============================================================
if %CTEST_EXIT% equ 0 (
    echo [PASS] All C++ unit tests passed
) else (
    echo [FAIL] Test failure occurred (exit code: %CTEST_EXIT%)
)
echo ============================================================

exit /b %CTEST_EXIT%
