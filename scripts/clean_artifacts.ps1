param(
    [switch]$Apply,
    [switch]$IncludeResults
)

$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$patterns = @(
    "__pycache__",
    ".pytest_cache",
    "*.pyc",
    "*.pyo",
    "*.obj",
    "*.exe",
    "*.log",
    "*_log.txt",
    "reattach_out.txt",
    "Ogre.log",
    "visual_status.json"
)

$rootFilePatterns = @("*.png", "*.jpg", "*.jpeg", "frames_*.gv", "frames_*.pdf")
$resultDirs = @("ablation_results", "ablation_results_v2", "ablation_results_v3", "experiment_results", "eval_results", "eval_run", "eval_run2", "eval_run3", "eval_run4", "sensitivity_results", "screenshots")

function Is-InResultDir {
    param([string]$Path)
    $rootPath = $root.Path.TrimEnd('\')
    $fullPath = (Resolve-Path -LiteralPath $Path).Path
    if (-not $fullPath.StartsWith($rootPath, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $false
    }
    $relative = $fullPath.Substring($rootPath.Length).TrimStart('\')
    foreach ($dir in $resultDirs) {
        if ($relative -eq $dir -or $relative.StartsWith("$dir\")) {
            return $true
        }
    }
    return $false
}

$targets = New-Object System.Collections.Generic.List[System.IO.FileSystemInfo]

foreach ($pattern in $patterns) {
    Get-ChildItem -LiteralPath $root -Recurse -Force -Filter $pattern -ErrorAction SilentlyContinue |
        Where-Object { $IncludeResults -or -not (Is-InResultDir $_.FullName) } |
        ForEach-Object { $targets.Add($_) }
}

foreach ($pattern in $rootFilePatterns) {
    Get-ChildItem -LiteralPath $root -File -Force -Filter $pattern -ErrorAction SilentlyContinue |
        ForEach-Object { $targets.Add($_) }
}

$targets = $targets | Sort-Object FullName -Unique
$bytes = ($targets | Where-Object { -not $_.PSIsContainer } | Measure-Object Length -Sum).Sum

if (-not $targets) {
    Write-Host "No generated artifacts matched the cleanup rules."
    exit 0
}

$mode = if ($Apply) { "APPLY" } else { "DRY RUN" }
Write-Host "$mode - matched $($targets.Count) paths, file bytes: $bytes"
$targets | ForEach-Object { Write-Host $_.FullName }

if ($Apply) {
    foreach ($target in $targets) {
        $resolved = Resolve-Path -LiteralPath $target.FullName
        if (-not $resolved.Path.StartsWith($root.Path, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove path outside workspace: $($resolved.Path)"
        }
        Remove-Item -LiteralPath $resolved.Path -Recurse -Force
    }
}
else {
    Write-Host "Preview only. Re-run with -Apply to delete these artifacts."
}
