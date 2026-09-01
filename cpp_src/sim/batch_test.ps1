# 10种子1小时综合验证脚本
# v3.2.16最终配置：A*起点偏移+IMU融合+聚类估计，无ICP
$env:USE_AMCL = "1"
$env:USE_IMU_FUSION = "1"
$env:USE_ICP = "0"
Remove-Item Env:SEED -ErrorAction SilentlyContinue

$results = @()
$logFile = "batch_results.txt"
"" | Out-File $logFile -Encoding utf8

for ($seed = 1; $seed -le 10; $seed++) {
    Write-Host "`n===== Running seed $seed =====" -ForegroundColor Cyan
    $startTime = Get-Date

    # 运行1小时测试（108000帧 = 30fps * 3600s）
    $output = & .\sim_test.exe 108000 54000 $seed 2>&1

    $elapsed = ((Get-Date) - $startTime).TotalSeconds
    $outputStr = $output -join "`n"

    # 提取关键指标
    $collision = if ($outputStr -match '碰撞次数:\s*(\d+)') { $matches[1] } else { "?" }
    $astarRate = if ($outputStr -match 'A\* 成功率:\s*([\d.]+)%') { $matches[1] } else { "?" }
    $avgErr = if ($outputStr -match '定位误差\(平均\):\s*([\d.]+)') { $matches[1] } else { "?" }
    $maxErr = if ($outputStr -match '最大\s*([\d.]+)\s*m') { $matches[1] } else { "?" }
    $conf = if ($outputStr -match '置信度:\s*([\d.]+)') { $matches[1] } else { "?" }
    $rooms = if ($outputStr -match '访问房间数:\s*(\d+)') { $matches[1] } else { "?" }
    $rounds = if ($outputStr -match '完成轮次:\s*(\d+)') { $matches[1] } else { "?" }

    $result = [PSCustomObject]@{
        Seed      = $seed
        Collision = $collision
        AStarRate = $astarRate
        AvgErr    = $avgErr
        MaxErr    = $maxErr
        Conf      = $conf
        Rooms     = $rooms
        Rounds    = $rounds
        Time      = [math]::Round($elapsed, 1)
    }
    $results += $result

    # 输出到日志
    $line = "seed=$seed | collision=$collision | A*=$astarRate% | avg_err=${avgErr}m | max_err=${maxErr}m | conf=$conf | rooms=$rooms | rounds=$rounds | time=${elapsed}s"
    $line | Out-File $logFile -Append -Encoding utf8
    Write-Host $line -ForegroundColor Green
}

Write-Host "`n===== Summary =====" -ForegroundColor Cyan
$results | Format-Table -AutoSize
