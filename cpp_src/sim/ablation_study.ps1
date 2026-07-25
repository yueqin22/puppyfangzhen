param(
    [int]$Frames = 54000,
    [int]$Seed = 1
)

$configs = @(
    @{ Name = "Baseline";   USE_AMCL = "1"; USE_KLD = "1"; USE_RVO = "1" }
    @{ Name = "No-AMCL";    USE_AMCL = "0"; USE_KLD = "1"; USE_RVO = "1" }
    @{ Name = "Fixed-500";  USE_AMCL = "1"; USE_KLD = "0"; USE_RVO = "1" }
    @{ Name = "CBF-vs-RVO"; USE_AMCL = "1"; USE_KLD = "1"; USE_RVO = "0" }
)

$results = @()

Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "  Ablation Study: 4 configs x 30min (seed=$Seed)" -ForegroundColor Cyan
Write-Host "======================================================================" -ForegroundColor Cyan

foreach ($cfg in $configs) {
    Write-Host ">>> Running: $($cfg.Name) ..." -ForegroundColor Yellow

    $env:USE_AMCL = $cfg.USE_AMCL
    $env:USE_KLD = $cfg.USE_KLD
    $env:USE_RVO = $cfg.USE_RVO

    $output = & .\sim_test.exe $Frames 54000 $Seed 2>&1 | Out-String

    $collision = 0; $astar_rate = 0.0; $loc_error = 0.0; $conf = 0.0
    $stuck = 0.0; $distance = 0.0; $rooms = 0; $rounds = 0
    $amcl_ms = 0.0; $astar_ms = 0.0; $elapsed = 0.0

    if ($output -match "Collision:\s*(\d+)") { $collision = [int]$Matches[1] }
    if ($output -match "A\* success.*?(\d+\.?\d*)") { $astar_rate = [double]$Matches[1] }
    if ($output -match "loc_error:\s*(\d+\.?\d*)") { $loc_error = [double]$Matches[1] }
    if ($output -match "confidence:\s*(\d+\.?\d*)") { $conf = [double]$Matches[1] }
    if ($output -match "stuck_ratio:\s*(\d+\.?\d*)") { $stuck = [double]$Matches[1] }
    if ($output -match "total_distance:\s*(\d+\.?\d*)") { $distance = [double]$Matches[1] }
    if ($output -match "rooms_visited:\s*(\d+)") { $rooms = [int]$Matches[1] }
    if ($output -match "rounds:\s*(\d+)") { $rounds = [int]$Matches[1] }

    # Fallback: parse Chinese output
    if ($collision -eq 0 -and $output -match "\d+") {
        if ($output -match "(\d+)\s+(?:near|near_miss)") { $collision = 0 }
    }
    # Parse from final report section
    $lines = $output -split "`n"
    foreach ($line in $lines) {
        if ($line -match "A\*.*?(\d+\.?\d*)%") { $astar_rate = [double]$Matches[1] }
        if ($line -match "(\d+\.?\d*)\s*m\s+") { $distance = [double]$Matches[1] }
    }

    $result = [PSCustomObject]@{
        Config     = $cfg.Name
        Collision  = $collision
        AStarRate  = $astar_rate
        LocError   = $loc_error
        Confidence = $conf
        StuckRatio = $stuck
        Distance   = $distance
        Rooms      = $rooms
        Rounds     = $rounds
    }

    $results += $result
    Write-Host "    Done: Coll=$collision A*=$astar_rate Err=$loc_error Conf=$conf Stuck=$stuck Dist=$distance Rooms=$rooms Rounds=$rounds" -ForegroundColor Green
}

# CSV export
$results | Export-Csv -Path "ablation_results.csv" -NoTypeInformation -Encoding UTF8

# Summary table
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "  Ablation Summary (seed=$Seed)" -ForegroundColor Cyan
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ("{0,-12} {1,6} {2,8} {3,8} {4,8} {5,8} {6,8} {7,6} {8,6}" -f "Config","Coll","A*Pct","Err(m)","Conf","Stuck%","Dist(m)","Rooms","Rounds")
Write-Host ("{0,-12} {1,6} {2,8} {3,8} {4,8} {5,8} {6,8} {7,6} {8,6}" -f "------","----","------","------","----","------","-------","-----","------")
foreach ($r in $results) {
    Write-Host ("{0,-12} {1,6} {2,8.1} {3,8.3} {4,8.3} {5,8.1} {6,8.1} {7,6} {8,6}" -f $r.Config,$r.Collision,$r.AStarRate,$r.LocError,$r.Confidence,$r.StuckRatio,$r.Distance,$r.Rooms,$r.Rounds)
}

Write-Host ""
Write-Host "CSV saved: ablation_results.csv" -ForegroundColor Green
