# v3.2.18c 10种子1小时批量验证
$env:USE_AMCL = "1"
$env:USE_IMU_FUSION = "1"
$env:USE_ICP = "0"
Remove-Item Env:SEED -ErrorAction SilentlyContinue

$logFile = "v3218c3_results.txt"
"=== v3.2.18c 10种子验证 (0.3s+1.0s混合fallback预测) ===" | Out-File $logFile -Encoding utf8

for ($seed = 1; $seed -le 10; $seed++) {
    Write-Host "`n===== Running seed $seed =====" -ForegroundColor Cyan
    $startTime = Get-Date
    $output = & .\sim_test.exe 108000 54000 $seed 2>&1
    $elapsed = ((Get-Date) - $startTime).TotalSeconds
    $outputStr = $output -join "`n"

    $collision = if ($outputStr -match '碰撞次数:\s*(\d+)') { $matches[1] } else { "?" }
    $astarRate = if ($outputStr -match 'A\* 成功率:\s*([\d.]+)%') { $matches[1] } else { "?" }
    $avgErr = if ($outputStr -match '定位误差\(平均\):\s*([\d.]+)') { $matches[1] } else { "?" }
    $conf = if ($outputStr -match '置信度:\s*([\d.]+)') { $matches[1] } else { "?" }
    $rooms = if ($outputStr -match '访问房间数:\s*(\d+)') { $matches[1] } else { "?" }
    $rounds = if ($outputStr -match '完成轮次:\s*(\d+)') { $matches[1] } else { "?" }

    $line = "seed=$seed | coll=$collision | A*=$astarRate% | avg_err=${avgErr}m | conf=$conf | rooms=$rooms | rounds=$rounds | time=${elapsed}s"
    $line | Out-File $logFile -Append -Encoding utf8
    Write-Host $line -ForegroundColor Green
}

Write-Host "`n===== Summary =====" -ForegroundColor Cyan
Get-Content $logFile
