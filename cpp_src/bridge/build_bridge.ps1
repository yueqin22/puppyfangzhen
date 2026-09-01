# Build bridge using MSVC via PowerShell
$ErrorActionPreference = "Stop"

# Find VS2022
$vcvars = $null
foreach ($edition in @("Community", "Professional", "Enterprise", "BuildTools")) {
    $p = "C:\Program Files\Microsoft Visual Studio\2022\$edition\VC\Auxiliary\Build\vcvars64.bat"
    if (Test-Path $p) {
        $vcvars = $p
        break
    }
}

if (-not $vcvars) {
    Write-Error "VS2022 not found"
    exit 1
}
Write-Host "Found VS: $vcvars"

# Import MSVC environment
$output = cmd /c "`"$vcvars`" >nul 2>&1 && set"
$output | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') {
        [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], "Process")
    }
}
Write-Host "MSVC environment loaded"

# Configure
Write-Host "`n[1/2] CMake configure..."
cmake -S "e:\puppyfangzhen\cpp_src\bridge\." -B "e:\puppyfangzhen\cpp_src\bridge\build" -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed"; exit 1 }

# Build
Write-Host "`n[2/2] Building..."
cmake --build "e:\puppyfangzhen\cpp_src\bridge\build" --config Release
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit 1 }

Write-Host "`n[OK] Build succeeded!"
Get-ChildItem "e:\puppyfangzhen\cpp_src\bridge\build\Release\nav_ue_bridge.exe" -ErrorAction SilentlyContinue | Select-Object FullName, Length, LastWriteTime
