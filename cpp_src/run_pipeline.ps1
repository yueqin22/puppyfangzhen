# run_pipeline.ps1 - M4.3: One-click pipeline
# Build (sim+bridge) -> unittest -> geometry check -> regression
#
# Usage:
#   .\run_pipeline.ps1              # Quick (3 seeds 36k frames)
#   .\run_pipeline.ps1 -Full        # Full (10 seeds, ~1h)
#   .\run_pipeline.ps1 -SkipReg     # Skip regression
#   .\run_pipeline.ps1 -SkipBuild   # Skip build steps
param(
    [switch]$Full,
    [switch]$SkipReg,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$simDir = Join-Path $root "sim"
$bridgeDir = Join-Path $root "bridge"
$results = @()

function Write-Step($n, $total, $msg) {
    Write-Host ""
    Write-Host "[$n/$total] $msg" -ForegroundColor Cyan
    Write-Host ("=" * 60) -ForegroundColor DarkGray
}

function Write-Result($name, $passed, $detail) {
    $color = if ($passed) { "Green" } else { "Red" }
    $status = if ($passed) { "PASS" } else { "FAIL" }
    Write-Host "  [$status] $name - $detail" -ForegroundColor $color
    $script:results += [PSCustomObject]@{
        Name = $name; Status = $status; Detail = $detail
    }
}

$totalSteps = if ($SkipReg) { 4 } else { 5 }
$step = 0

# ===== 1. Build sim_test =====
if (-not $SkipBuild) {
    Write-Step (++$step) $totalSteps "Build sim_test"
    $launch = Join-Path $simDir "_launch_build.bat"
    if (Test-Path $launch) {
        & $launch 2>&1 | Select-Object -Last 3
    } else {
        & (Join-Path $simDir "build.bat") 2>&1 | Select-Object -Last 3
    }
    $simExe = Join-Path $simDir "sim_test.exe"
    if (Test-Path $simExe) {
        Write-Result "Build sim_test" $true "sim_test.exe OK"
    } else {
        Write-Result "Build sim_test" $false "sim_test.exe not found"
    }
}

# ===== 2. Build bridge + geometry_check =====
if (-not $SkipBuild) {
    Write-Step (++$step) $totalSteps "Build bridge + geometry_check"
    & (Join-Path $bridgeDir "build_bridge.bat") 2>&1 | Select-Object -Last 3
    $bridgeExe = Join-Path $bridgeDir "build\Release\Release\nav_ue_bridge.exe"
    $geomExe = Join-Path $bridgeDir "build\Release\Release\geometry_check.exe"
    $bp = Test-Path $bridgeExe
    $gp = Test-Path $geomExe
    if ($bp -and $gp) {
        Write-Result "Build bridge" $true "nav_ue_bridge + geometry_check OK"
    } else {
        Write-Result "Build bridge" $false "bridge=$bp geom=$gp"
    }
}

# ===== 3. Unit tests =====
Write-Step (++$step) $totalSteps "Unit tests (AMCL + OccupancyGrid)"
$launchUt = Join-Path $simDir "_launch_unittest.bat"
if (Test-Path $launchUt) {
    $utOut = & $launchUt 2>&1
} else {
    $utOut = & (Join-Path $simDir "build.bat") unittest 2>&1
}
$utPass = $utOut | Select-String "100% tests passed"
if ($utPass) {
    Write-Result "Unit tests" $true "All tests passed"
} else {
    Write-Result "Unit tests" $false "See output"
    $utOut | Select-Object -Last 5
}

# ===== 4. Geometry check =====
Write-Step (++$step) $totalSteps "Geometry check (500 samples)"
$geomExe = Join-Path $bridgeDir "build\Release\Release\geometry_check.exe"
$scenePath = Join-Path $root "..\config\scene_home.json"
$geomOut = & $geomExe --n 500 --seed 42 --scene $scenePath --json (Join-Path $bridgeDir "geometry_baseline.json") 2>&1
$geomSummary = $geomOut | Select-String "^SUMMARY:"
if ($geomSummary) {
    Write-Result "Geometry check" $true $geomSummary.ToString().Trim()
} else {
    Write-Result "Geometry check" $false "No SUMMARY"
}

# ===== 5. Regression (3-seed compare) =====
if (-not $SkipReg) {
    $seeds = if ($Full) { "1,2,3,4,5,6,7,8,9,10" } else { "1,6,8" }
    $label = if ($Full) { "10-seed FULL" } else { "3-seed QUICK" }
    Write-Step (++$step) $totalSteps "Regression: $label compare"
    $regOut = & python (Join-Path $bridgeDir "ue_regression_test.py") --mode compare --seeds $seeds --frames 36000 2>&1
    $regOut | ForEach-Object { Write-Host "  $_" }

    $regFail = $regOut | Where-Object { $_ -match "FAIL" -and $_ -notmatch "collision" }
    $regPass = -not $regFail
    $passCount = ($regOut | Where-Object { $_ -match "PASS" }).Count
    $failCount = ($regOut | Where-Object { $_ -match "FAIL" -and $_ -notmatch "collision" }).Count
    if ($regPass) {
        Write-Result "Regression" $true "${label}: $passCount PASS"
    } else {
        Write-Result "Regression" $false "${label}: $passCount PASS, $failCount FAIL"
    }
}

# ===== Summary =====
Write-Host ""
Write-Host ("=" * 60) -ForegroundColor Cyan
Write-Host "PIPELINE SUMMARY" -ForegroundColor Cyan
Write-Host ("=" * 60) -ForegroundColor Cyan
$results | Format-Table -AutoSize

$allPass = ($results | Where-Object { $_.Status -eq "FAIL" }).Count -eq 0
if ($allPass) {
    Write-Host ""
    Write-Host "[ALL PASS] Pipeline completed successfully" -ForegroundColor Green
    exit 0
} else {
    $failCount = ($results | Where-Object { $_.Status -eq "FAIL" }).Count
    Write-Host ""
    Write-Host "[FAIL] $failCount step(s) failed" -ForegroundColor Red
    exit 1
}
