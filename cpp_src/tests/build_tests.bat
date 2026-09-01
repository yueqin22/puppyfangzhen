@echo off
REM build_tests.bat - Build C++ white-box unit tests (tests/) using Ninja
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
cmake -S "%SRC%." -B "%BUILD%" -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 ( echo [ERROR] CMake configure failed & exit /b 1 )
cmake --build "%BUILD%" --config Release
if errorlevel 1 ( echo [ERROR] Build failed & exit /b 1 )
echo [OK] C++ unit tests built (Ninja)
endlocal
