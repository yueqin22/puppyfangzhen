# regression_pipeline.ps1 — One-click regression validation pipeline
# Flow: (clean?) → build → unittest → N-seed regression → save JSON → compare with baseline
#
# Usage:
#   .\regression_pipeline.ps1 [-Quick] [-Clean] [-SkipUnitTest] [-Frames N] [-Seeds 1,6,8]
#     -Quick:       seeds=1,6,8 (default: 1..10)
#     -Clean:       rm build dir before building (default: incremental)
#     -SkipUnitTest: skip ctest step (e.g. already ran)
#     -Frames:      override sim frames (default: 108000 = 30fps × 1h)
#     -Seeds:       explicit seed list, overrides -Quick
param(
    [switch]$Quick,
    [switch]$Clean,
    [switch]$SkipUnitTest,
    [int]$Frames = 108000,
    [int[]]$Seeds = @()
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# ---- Params ----
$REPORT_INTERVAL = [Math]::Max(540, [int]($Frames / 2))   # report at mid-point + end
$BASELINE_FILE  = ".\baseline\v3218i.json"
$RESULTS_DIR    = ".\results"
if (-not (Test-Path $RESULTS_DIR)) { New-Item -ItemType Directory -Path $RESULTS_DIR | Out-Null }
$TIMESTAMP = Get-Date -Format "yyyyMMdd_HHmmss"
$RUN_JSON  = Join-Path $RESULTS_DIR "run_results_$TIMESTAMP.json"
$LATEST    = Join-Path $RESULTS_DIR "latest_run.json"

# Seed list priority: explicit -Seeds > -Quick > 1..10
if ($Seeds.Count -eq 0) {
    $Seeds = if ($Quick) { @(1, 6, 8) } else { 1..10 }
}
$mode = if ($Quick) { "QUICK ($($Seeds.Count) seeds)" } else { "FULL ($($Seeds.Count) seeds)" }
$env:USE_AMCL = "1"
$env:USE_IMU_FUSION = "1"
$env:USE_ICP = "0"
Remove-Item Env:SEED -ErrorAction SilentlyContinue

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " Regression Pipeline (v3.2.18i baseline)  [$mode]"
Write-Host " Frames=$Frames  Seeds=$($Seeds -join ',')"
Write-Host "========================================" -ForegroundColor Cyan

# -----------------------------------------------------------
# Step 1: Build (+ optionally clean) — use standalone launcher to avoid bat BOM issues
# -----------------------------------------------------------
Write-Host "`n[1/4] Building $($(if($Clean){'(CLEAN) '}else{''}))..." -ForegroundColor Yellow
$buildLaunchArgs = if ($Clean) { "clean" } else { "" }
& cmd /c "`"$PSScriptRoot\_launch_build.bat`" $buildLaunchArgs"
if ($LASTEXITCODE -ne 0) {
    Write-Host "[FAIL] Build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] Build succeeded" -ForegroundColor Green

# -----------------------------------------------------------
# Step 2: Unit tests (ctest) — use standalone unittest launcher
# -----------------------------------------------------------
if (-not $SkipUnitTest) {
    Write-Host "`n[2/4] Running unit tests..." -ForegroundColor Yellow
    & cmd /c "`"$PSScriptRoot\_launch_unittest.bat`""
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] Unit tests FAILED" -ForegroundColor Red
        exit 1
    }
    Write-Host "[OK] All unit tests passed" -ForegroundColor Green
} else {
    Write-Host "`n[2/4] Unit tests skipped (-SkipUnitTest)" -ForegroundColor Gray
}

# -----------------------------------------------------------
# Step 3: Run sim regression for each seed
# -----------------------------------------------------------
Write-Host "`n[3/4] Running $($Seeds.Count)-seed regression (frames=$Frames)..." -ForegroundColor Yellow
$results = @()
$seedResults = @()
$passPerSeed = $true

foreach ($seed in $Seeds) {
    Write-Host "  seed $seed ... " -NoNewline
    $start = Get-Date

    $output = & .\sim_test.exe $Frames $REPORT_INTERVAL $seed 2>&1
    $elapsed = [math]::Round(((Get-Date) - $start).TotalSeconds, 1)
    $outStr = $output -join "`n"

    # Parse metrics — primary: "SUMMARY: collisions=0 astar_rate=100.0 avg_err=0.053 ..." line
    $coll=-1; $astar=-1; $avgErr=-1; $maxErr=-1; $conf=-1; $rooms=-1; $rounds=-1; $distance=-1; $stuckPct=-1
    if ($outStr -match 'SUMMARY\s*:\s*(.+)') {
        $sumLine = $matches[1]
        if ($sumLine -match 'collisions=(\d+)')        { $coll     = [int]$matches[1] }
        if ($sumLine -match 'astar_rate=([\d.]+)')     { $astar    = [double]$matches[1] }
        if ($sumLine -match 'avg_err=([\d.]+)')        { $avgErr   = [double]$matches[1] }
        if ($sumLine -match 'max_err=([\d.]+)')        { $maxErr   = [double]$matches[1] }
        if ($sumLine -match 'confidence=([\d.]+)')     { $conf     = [double]$matches[1] }
        if ($sumLine -match 'rooms=(\d+)')             { $rooms    = [int]$matches[1] }
        if ($sumLine -match 'rounds=(\d+)')            { $rounds   = [int]$matches[1] }
        if ($sumLine -match 'distance=([\d.]+)')       { $distance = [double]$matches[1] }
        if ($sumLine -match 'stuck_ratio=([\d.]+)')    { $stuckPct = [double]$matches[1] }
    }
    # Fallback: legacy Chinese report regex (kept for older builds)
    if ($coll -lt 0   -and $outStr -match '碰撞次数:\s*(\d+)')          { $coll   = [int]$matches[1]    }
    if ($astar -lt 0  -and $outStr -match 'A\* 成功率:\s*([\d.]+)%')   { $astar  = [double]$matches[1] }
    if ($avgErr -lt 0 -and $outStr -match '定位误差\(平均\):\s*([\d.]+)') { $avgErr = [double]$matches[1] }
    if ($maxErr -lt 0 -and $outStr -match '最大\s*([\d.]+)\s*m')        { $maxErr = [double]$matches[1] }
    if ($conf -lt 0   -and $outStr -match '置信度:\s*([\d.]+)')         { $conf   = [double]$matches[1] }
    if ($rooms -lt 0  -and $outStr -match '访问房间数:\s*(\d+)')        { $rooms  = [int]$matches[1]    }
    if ($rounds -lt 0 -and $outStr -match '完成轮次:\s*(\d+)')         { $rounds = [int]$matches[1]    }

    # Hard-coded per-seed pass (for terminal print; overall compare uses compare_baseline.ps1)
    $seedOk = ($coll -eq 0 -and $astar -ge 90.0 -and $avgErr -lt 0.5 -and $rooms -ge 3)
    if (-not $seedOk) { $passPerSeed = $false }
    $color  = if ($seedOk) { "Green" } else { "Red" }
    $status = if ($seedOk) { "OK"    } else { "FAIL" }
    Write-Host ("$status ({0}s coll={1} A*={2}% err={3}m conf={4})" -f $elapsed,$coll,$astar,$avgErr,$conf) -ForegroundColor $color

    $results += [PSCustomObject]@{
        Seed     = $seed; Coll=$coll; AStar=$astar; AvgErr=$avgErr;
        MaxErr=$maxErr; Conf=$conf; Rooms=$rooms; Rounds=$rounds;
        Distance=$distance; StuckPct=$stuckPct; Time=$elapsed
    }
    $seedResults += [ordered]@{
        seed     = [int]$seed
        collision= [int]$coll
        astar_rate = [double]$astar
        avg_err  = [double]$avgErr
        max_err  = [double]$maxErr
        conf     = [double]$conf
        rooms    = [int]$rooms
        rounds   = [int]$rounds
        distance_m = [double]$distance
        stuck_pct  = [double]$stuckPct
        elapsed_s  = [double]$elapsed
    }
}

# Build and save run_results JSON (same schema as baseline/v3218i.json)
$mColl   = $results | Measure-Object -Property Coll   -Sum
$mAStar  = $results | Measure-Object -Property AStar  -Average
$mAvgErr = $results | Measure-Object -Property AvgErr -Average
$mConf   = $results | Measure-Object -Property Conf   -Average
$mAsMin  = $results | Measure-Object -Property AStar  -Minimum
$mEmMax  = $results | Measure-Object -Property AvgErr -Maximum
$runObj = [ordered]@{
    version     = "run-$TIMESTAMP"
    date        = (Get-Date -Format "yyyy-MM-dd HH:mm:ss")
    description = "Regression run: seeds=$($Seeds -join ',') frames=$Frames"
    test_config = [ordered]@{
        frames          = [int]$Frames
        report_interval = [int]$REPORT_INTERVAL
        env = [ordered]@{
            USE_AMCL = $env:USE_AMCL
            USE_IMU_FUSION = $env:USE_IMU_FUSION
            USE_ICP  = $env:USE_ICP
        }
    }
    seeds   = $seedResults
    summary = [ordered]@{
        total_collisions  = [int]$mColl.Sum
        avg_astar_rate    = [math]::Round([double]$mAStar.Average, 2)
        avg_avg_err       = [math]::Round([double]$mAvgErr.Average, 3)
        avg_conf          = [math]::Round([double]$mConf.Average, 3)
        min_astar_rate    = [math]::Round([double]$mAsMin.Minimum, 2)
        max_avg_err       = [math]::Round([double]$mEmMax.Maximum, 3)
    }
}
$runJsonStr = $runObj | ConvertTo-Json -Depth 5
Set-Content -Path $RUN_JSON -Value $runJsonStr -Encoding utf8
Copy-Item -Path $RUN_JSON -Destination $LATEST -Force
Write-Host ("  Saved: {0}" -f $RUN_JSON)

# -----------------------------------------------------------
# Step 4: Compare with baseline
# -----------------------------------------------------------
Write-Host "`n[4/4] Comparing with baseline ($BASELINE_FILE)..." -ForegroundColor Yellow
& .\compare_baseline.ps1 -RunJson $RUN_JSON -BaselineJson $BASELINE_FILE
$cmpExit = $LASTEXITCODE

# -----------------------------------------------------------
# Final summary
# -----------------------------------------------------------
Write-Host "`n========================================" -ForegroundColor Cyan
$totalColl = [int]$mColl.Sum
$avgAstar  = [math]::Round([double]$mAStar.Average, 1)
$avgErr2   = [math]::Round([double]$mAvgErr.Average, 3)
Write-Host (" Summary: {0} seeds | collisions={1} | A*avg={2}% | err_avg={3}m" -f $Seeds.Count,$totalColl,$avgAstar,$avgErr2)
Write-Host (" Result JSON: {0}" -f $RUN_JSON)
Write-Host "========================================" -ForegroundColor Cyan

$overallOk = ($passPerSeed -and $cmpExit -eq 0 -and $totalColl -eq 0)
if ($overallOk) {
    Write-Host "[PASS] Regression clean — no collisions, baseline matched" -ForegroundColor Green
    exit 0
} else {
    Write-Host "[FAIL] Pipeline failed (seeds-fail=$(-not $passPerSeed), cmp-exit=$cmpExit, coll=$totalColl)" -ForegroundColor Red
    exit 1
}
