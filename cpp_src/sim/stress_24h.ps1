# stress_24h.ps1 - 24h stress test + memory leak detection
# Task P0-1.2: verify long-run stability and memory safety
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File stress_24h.ps1            # full 10 seeds 24h
#   powershell -ExecutionPolicy Bypass -File stress_24h.ps1 -Quick     # quick: 1 seed 10min
#   powershell -ExecutionPolicy Bypass -File stress_24h.ps1 -Seeds 1,2 -Hours 2
#
# Output:
#   results/stress_24h_summary.csv        per-seed summary
#   results/stress_24h_<seed>_hourly.csv  per-hour metrics
#   results/stress_24h_<seed>_memory.csv  memory samples
#   results/stress_24h_<seed>_stdout.txt  full log

param(
    [switch]$Quick,
    [string]$Seeds = "1,2,3,4,5,6,7,8,9,10",
    [int]$Hours = 24
)

$simDir = "e:\puppyfangzhen\cpp_src\sim"
$exe = "$simDir\sim_test.exe"
$resultsDir = "$simDir\results"

if (-not (Test-Path $resultsDir)) {
    New-Item -ItemType Directory -Path $resultsDir | Out-Null
}

# ---- Compute parameters ----
if ($Quick) {
    $Seeds = "1"
    $totalFrames = 18000
    $reportInterval = 3600
} else {
    $totalFrames = 108000 * $Hours
    $reportInterval = 108000
}

$seedList = $Seeds.Split(",") | ForEach-Object { [int]$_.Trim() }

$hoursStr = '{0:N1}' -f ($totalFrames / 30.0 / 3600.0)
$reportMinStr = '{0:N1}' -f ($reportInterval / 30.0 / 60.0)

Write-Host "================================================================"
Write-Host "  24h Stress Test + Memory Leak Detection"
Write-Host "  Seeds: $Seeds"
Write-Host "  Duration: $hoursStr hours ($totalFrames frames)"
Write-Host "  Report interval: $reportInterval frames ($reportMinStr min)"
Write-Host "================================================================"

# ---- Summary CSV header ----
$summaryCsv = "$resultsDir\stress_24h_summary.csv"
"seed,collisions,astar_rate,avg_err,max_err,confidence,stuck_ratio,distance,rooms,rounds,elapsed_s,peak_memory_mb,status" |
    Set-Content $summaryCsv -Encoding ASCII

# ---- Run each seed ----
foreach ($seed in $seedList) {
    Write-Host ""
    Write-Host "==== Seed $seed ===="

    $stdoutFile = "$resultsDir\stress_24h_${seed}_stdout.txt"
    $hourlyCsv = "$resultsDir\stress_24h_${seed}_hourly.csv"
    $memCsv = "$resultsDir\stress_24h_${seed}_memory.csv"

    "hour,collisions,near_miss,skip_count,stall_events,rounds,robot_x,robot_y,amcl_err,avg_err,max_err,conf" |
        Set-Content $hourlyCsv -Encoding ASCII
    "timestamp,elapsed_s,working_set_mb,private_memory_mb" |
        Set-Content $memCsv -Encoding ASCII

    # ---- Start sim_test.exe (non-blocking) ----
    $proc = Start-Process -FilePath $exe -ArgumentList "$totalFrames $reportInterval $seed" `
        -RedirectStandardOutput $stdoutFile -PassThru -NoNewWindow

    Write-Host "  PID: $($proc.Id), waiting..."

    # ---- Memory sampling loop (every 30s) ----
    $peakMem = 0
    while (-not $proc.HasExited) {
        Start-Sleep -Seconds 30
        try {
            $p = Get-Process -Id $proc.Id -ErrorAction Stop
            $ws = [math]::Round($p.WorkingSet64 / 1MB, 1)
            $pm = [math]::Round($p.PrivateMemorySize64 / 1MB, 1)
            $elapsed = [math]::Round(((Get-Date) - $p.StartTime).TotalSeconds, 0)
            "$((Get-Date).ToString('HH:mm:ss')),$elapsed,$ws,$pm" |
                Add-Content $memCsv -Encoding ASCII
            if ($ws -gt $peakMem) { $peakMem = $ws }
        } catch {
            break
        }
    }

    $exitCode = $proc.ExitCode
    Write-Host "  Process exited, code: $exitCode"

    # ---- Parse output ----
    $stdout = Get-Content $stdoutFile -Raw -ErrorAction SilentlyContinue
    if (-not $stdout) {
        Write-Host "  [ERROR] No output" -ForegroundColor Red
        "$seed,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,$peakMem,NO_OUTPUT" |
            Add-Content $summaryCsv -Encoding ASCII
        continue
    }

    # Parse SUMMARY line
    $summaryLine = ($stdout -split "`n") | Where-Object { $_ -match "^SUMMARY:" } | Select-Object -Last 1
    $collisions = -1; $astarRate = -1; $avgErr = -1; $maxErr = -1
    $conf = -1; $stuckRatio = -1; $distance = -1; $rooms = -1; $rounds = -1; $elapsed = -1

    if ($summaryLine -match "collisions=(\d+)") { $collisions = [int]$matches[1] }
    if ($summaryLine -match "astar_rate=([\d.]+)") { $astarRate = [double]$matches[1] }
    if ($summaryLine -match "avg_err=([\d.]+)") { $avgErr = [double]$matches[1] }
    if ($summaryLine -match "max_err=([\d.]+)") { $maxErr = [double]$matches[1] }
    if ($summaryLine -match "confidence=([\d.]+)") { $conf = [double]$matches[1] }
    if ($summaryLine -match "stuck_ratio=([\d.]+)") { $stuckRatio = [double]$matches[1] }
    if ($summaryLine -match "distance=([\d.]+)") { $distance = [double]$matches[1] }
    if ($summaryLine -match "rooms=(\d+)") { $rooms = [int]$matches[1] }
    if ($summaryLine -match "rounds=(\d+)") { $rounds = [int]$matches[1] }
    if ($summaryLine -match "elapsed=([\d.]+)") { $elapsed = [double]$matches[1] }

    # Parse hourly report lines (lines starting with whitespace + [N])
    $hourlyLines = ($stdout -split "`n") | Where-Object { $_ -match "^\s+\[\d+\]" }
    $hour = 0
    foreach ($line in $hourlyLines) {
        $hCollisions = if ($line -match "=(\d+)") { [int]$matches[1] } else { -1 }
        $hRounds = if ($line -match "=(\d+)") { [int]$matches[1] } else { -1 }
        $hRobotX = if ($line -match "robot=\(([\d.\-]+)") { [double]$matches[1] } else { -1 }
        $hRobotY = if ($line -match "robot=\([\d.\-]+,([\d.\-]+)\)") { [double]$matches[1] } else { -1 }

        "$hour,$hCollisions,-1,-1,-1,$hRounds,$hRobotX,$hRobotY,-1,-1,-1,-1" |
            Add-Content $hourlyCsv -Encoding ASCII
        $hour++
    }

    # ---- Status determination ----
    $status = "PASS"
    if ($collisions -ne 0) { $status = "FAIL_COLLISION" }
    elseif ($astarRate -lt 95.0) { $status = "FAIL_ASTAR" }
    elseif ($stuckRatio -ge 20.0) { $status = "FAIL_STUCK" }

    Write-Host "  collisions: $collisions | A*: $astarRate% | avg_err: $avgErr | max_err: $maxErr | conf: $conf | rooms: $rooms | rounds: $rounds | peak_mem: ${peakMem}MB | status: $status"

    "$seed,$collisions,$astarRate,$avgErr,$maxErr,$conf,$stuckRatio,$distance,$rooms,$rounds,$elapsed,$peakMem,$status" |
        Add-Content $summaryCsv -Encoding ASCII
}

# ---- Summary report ----
Write-Host ""
Write-Host "================================================================"
Write-Host "  Stress test complete"
Write-Host "================================================================"
Get-Content $summaryCsv | Format-Table

# ---- Memory trend analysis ----
Write-Host ""
Write-Host "==== Memory Trend Analysis ===="
foreach ($seed in $seedList) {
    $memFile = "$resultsDir\stress_24h_${seed}_memory.csv"
    if (Test-Path $memFile) {
        $memData = Import-Csv $memFile
        if ($memData.Count -gt 2) {
            $firstMem = [double]$memData[0].working_set_mb
            $lastMem = [double]$memData[-1].working_set_mb
            $growth = $lastMem - $firstMem
            $growthPct = if ($firstMem -gt 0) { [math]::Round($growth / $firstMem * 100, 1) } else { 0 }
            $verdict = if ($growthPct -lt 5) { "PASS" } else { "SUSPECT" }
            Write-Host "  Seed ${seed}: first ${firstMem}MB -> last ${lastMem}MB (growth ${growthPct}%) [$verdict]"
        }
    }
}

Write-Host ""
Write-Host "Results: $summaryCsv"
Write-Host "Done."
