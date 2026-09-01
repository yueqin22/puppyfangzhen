@echo off
REM Standalone quick pipeline runner — mirrors build.bat validate Quick mode
REM Runs: 1 seed (4) x 32400 frames, SkipUnitTest (already passed)
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass ^
  -File "%~dp0regression_pipeline.ps1" ^
  -SkipUnitTest -Seeds 4 -Frames 32400
exit /b %ERRORLEVEL%
