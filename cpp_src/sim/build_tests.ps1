# build_tests.ps1 — 构建并运行单元测试
# 用法: powershell -ExecutionPolicy Bypass -File build_tests.ps1 [clean]

param([string]$Mode = "")

$simDir = "e:\puppyfangzhen\cpp_src\sim"
$srcDir = "e:\puppyfangzhen\cpp_src"
$buildDir = "$simDir\build"

# ---- 定位 VS2022 vcvars64.bat ----
$vcvars = $null
foreach ($sku in @("Community", "Professional", "Enterprise", "BuildTools")) {
    $p = "C:\Program Files\Microsoft Visual Studio\2022\$sku\VC\Auxiliary\Build\vcvars64.bat"
    if (Test-Path $p) { $vcvars = $p; break }
    $p = "C:\Program Files (x86)\Microsoft Visual Studio\2022\$sku\VC\Auxiliary\Build\vcvars64.bat"
    if (Test-Path $p) { $vcvars = $p; break }
}
if (-not $vcvars) {
    Write-Host "[ERROR] VS2022 not found" -ForegroundColor Red
    exit 1
}

# ---- 清理 ----
if ($Mode -eq "clean" -and (Test-Path $buildDir)) {
    Write-Host "[1/4] Cleaning build directory..."
    Remove-Item -Recurse -Force $buildDir
}

# ---- 生成临时批处理（桥接 vcvars64.bat）----
$tempBat = "$simDir\_build_tests_temp.bat"
$batContent = @"
@echo off
call "$vcvars" >nul 2>&1
cmake -S "$srcDir" -B "$buildDir" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
if errorlevel 1 exit /b 1
cmake --build "$buildDir" --config Release
if errorlevel 1 exit /b 1
ctest --test-dir "$buildDir" -C Release --output-on-failure
"@
Set-Content -Path $tempBat -Value $batContent -Encoding ASCII

Write-Host "[2/4] CMake configure + build (BUILD_TESTING=ON)..."
Write-Host "[3/4] Running ctest..."
& $tempBat
$exitCode = $LASTEXITCODE
Remove-Item $tempBat -ErrorAction SilentlyContinue

if ($exitCode -ne 0) {
    Write-Host "[ERROR] Build or tests failed (exit $exitCode)" -ForegroundColor Red
    exit $exitCode
}

Write-Host "[4/4] All unit tests passed" -ForegroundColor Green
