# compare_baseline.ps1 - Regression baseline comparator
# Compares run results (JSON) against v3218i baseline, exits non-zero on degradation
#
# Usage:
#   .\compare_baseline.ps1 -RunJson <path>  [-BaselineJson <path>]
#   -RunJson:       result file to compare (same schema as baseline/v3218i.json)
#   -BaselineJson:  optional, defaults to .\baseline\v3218i.json
#   -ToleranceAstar: allow A* to drop this many percentage points  (default 2.0)
#   -ToleranceErr:   allow avg_err to rise by this many meters      (default 0.10)
param(
    [Parameter(Mandatory=$true)] [string]$RunJson,
    [string]$BaselineJson = ".\baseline\v3218i.json",
    [double]$ToleranceAstar = 2.0,
    [double]$ToleranceErr  = 0.10
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path $RunJson)) {
    Write-Host "[ERROR] Run result file not found: $RunJson" -ForegroundColor Red
    exit 2
}
if (-not (Test-Path $BaselineJson)) {
    Write-Host "[ERROR] Baseline file not found: $BaselineJson" -ForegroundColor Red
    exit 2
}

$baseObj = Get-Content $BaselineJson -Raw | ConvertFrom-Json
$runObj  = Get-Content $RunJson     -Raw | ConvertFrom-Json

$baseBySeed = @{}
foreach ($s in $baseObj.seeds) { $baseBySeed[[int]$s.seed] = $s }
$runBySeed  = @{}
foreach ($s in $runObj.seeds)  { $runBySeed[[int]$s.seed]  = $s }

$commonSeeds = $baseBySeed.Keys | Where-Object { $runBySeed.ContainsKey($_) } | Sort-Object
if ($commonSeeds.Count -eq 0) {
    Write-Host "[ERROR] No seeds in common between baseline and run result" -ForegroundColor Red
    exit 2
}

Write-Host "============================================" -ForegroundColor Cyan
$runVer = if ($runObj.version) { [string]$runObj.version } else { 'current-run' }
Write-Host (" Baseline comparison ({0} vs {1})" -f $baseObj.version, $runVer)
Write-Host " Tolerance: A* -$ToleranceAstar%,  avg_err +$ToleranceErr m"
Write-Host "============================================" -ForegroundColor Cyan

Write-Host ""
Write-Host ("{0,5} | {1,14} | {2,17} | {3,17} | {4,11} | {5}" -f
    "Seed", "Coll (base/now)", "A* % (base/now)", "Err m (base/now)", "Status", "Notes")
Write-Host "------+----------------+-------------------+-------------------+-------------+-------------------------" -ForegroundColor Gray

$allPass = $true
$degradeSummaries = @()

foreach ($seed in $commonSeeds) {
    $b = $baseBySeed[$seed]
    $r = $runBySeed[$seed]

    $notes = @()
    $collDeg  = $false
    $astarDeg = $false
    $errDeg   = $false

    # 1. Collision — strict zero tolerance: > base means regression
    if ($r.collision -gt $b.collision) {
        $collDeg = $true
        $notes += ("coll +" + ($r.collision - $b.collision))
    } elseif ($r.collision -lt $b.collision) {
        $notes += ("coll -" + ($b.collision - $r.collision) + " (improved)")
    }

    # 2. A* success rate — allow drop of ToleranceAstar points
    $astarDelta = [double]$r.astar_rate - [double]$b.astar_rate
    if ($astarDelta -lt -$ToleranceAstar) {
        $astarDeg = $true
        $notes += ("A* {0:F1}pp worse" -f (-$astarDelta))
    } elseif ($astarDelta -gt 0.5) {
        $notes += ("A* +{0:F1}pp" -f $astarDelta)
    }

    # 3. avg_err — allow rise of ToleranceErr meters (also enforce hard cap 0.5m per pass_criteria)
    $errDelta = [double]$r.avg_err - [double]$b.avg_err
    if ($errDelta -gt $ToleranceErr) {
        $errDeg = $true
        $notes += ("err +{0:F3}m" -f $errDelta)
    } elseif ([double]$r.avg_err -gt 0.5) {
        $errDeg = $true
        $notes += ("err >0.5m cap")
    } elseif ($errDelta -lt -0.02) {
        $notes += ("err {0:F3}m better" -f $errDelta)
    }

    # 4. Hard pass-criteria floor (same as baseline.pass_criteria)
    if ([double]$r.astar_rate -lt 90.0)  { $astarDeg = $true; $notes += "A* <90% floor" }
    if ([double]$r.conf  -lt 0.1)        { $errDeg   = $true; $notes += "conf <0.1" }
    if ([int]$r.rooms -lt 3)             { $collDeg  = $true; $notes += "rooms <3" }

    $degraded = $collDeg -or $astarDeg -or $errDeg
    if ($degraded) {
        $allPass = $false
        $status = "DEGRADED"
        $color  = "Red"
        $degradeSummaries += "seed $seed : $($notes -join ', ')"
    } else {
        $status = "OK"
        $color  = "Green"
    }

    $collStr  = "{0}/{1}"   -f $b.collision, $r.collision
    $astarStr = "{0:F1}%/{1:F1}%" -f [double]$b.astar_rate, [double]$r.astar_rate
    $errStr   = "{0:F3}/{1:F3}"  -f [double]$b.avg_err,   [double]$r.avg_err

    Write-Host ("{0,5} | {1,14} | {2,17} | {3,17} | {4,11} | {5}" -f
        $seed, $collStr, $astarStr, $errStr, $status, ($notes -join '; ')) -ForegroundColor $color
}

Write-Host ""

# Aggregate diff
$runColl  = ($runObj.seeds  | Measure-Object -Property collision -Sum).Sum
$baseColl = ($baseObj.seeds | Where-Object { $commonSeeds -contains $_.seed } | Measure-Object -Property collision -Sum).Sum
$runAstar  = [math]::Round(($runObj.seeds  | Measure-Object -Property astar_rate -Average).Average, 2)
$baseAstar = [math]::Round(($baseObj.seeds | Where-Object { $commonSeeds -contains $_.seed } | Measure-Object -Property astar_rate -Average).Average, 2)
$runAvgErr  = [math]::Round(($runObj.seeds  | Measure-Object -Property avg_err -Average).Average, 3)
$baseAvgErr = [math]::Round(($baseObj.seeds | Where-Object { $commonSeeds -contains $_.seed } | Measure-Object -Property avg_err -Average).Average, 3)

Write-Host ("Aggregate ({0} seeds): coll {1}/{2} | A* avg {3:F2}/{4:F2} | err_avg {5:F3}/{6:F3}" -f
    $commonSeeds.Count, $runColl, $baseColl, $runAstar, $baseAstar, $runAvgErr, $baseAvgErr) -ForegroundColor Gray

Write-Host ""
if ($allPass) {
    Write-Host "[PASS] Baseline matched within tolerance" -ForegroundColor Green
    exit 0
} else {
    Write-Host "[FAIL] Regression detected in $($degradeSummaries.Count) seed(s):" -ForegroundColor Red
    foreach ($d in $degradeSummaries) {
        Write-Host "  - $d" -ForegroundColor Red
    }
    exit 1
}
