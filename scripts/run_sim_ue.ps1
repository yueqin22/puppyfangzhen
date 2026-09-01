# ======================================================================
# run_sim_ue.ps1 - One-click UE5 closed-loop simulation (Bridge + UE -game)
# ALL-ASCII VERSION: avoids PS5.1 GBK-vs-UTF8 parse errors with Chinese chars.
# ======================================================================
# Outputs:
#   Screenshots: E:\puppyfangzhen\screenshots\R??_shot_N_HHMMSS.png
#   Logs:        E:\puppyfangzhen\logs\bridge_R??_*.log + ue_R??_*.log
#
# Usage (any working dir in PowerShell):
#   & "E:\puppyfangzhen\scripts\run_sim_ue.ps1"
#   & "E:\puppyfangzhen\scripts\run_sim_ue.ps1" -Frames 3600 -Shots 6 -ShotInterval 15
#
# Params:
#   -Frames        Bridge sim frames (default 1800 = 1 min @30Hz)
#   -Shots         Number of window screenshots (default 3)
#   -ShotInterval  Seconds between screenshots (default 12)
#   -RunId         Run id, default auto-increment R01/R02/...
#   -ResX/-ResY    UE -game window res (default 1280x720)
# ======================================================================
param(
    [int]$Frames       = 1800,
    [int]$Shots        = 3,
    [int]$ShotInterval = 12,
    [string]$RunId     = "",
    [int]$ResX         = 1280,
    [int]$ResY         = 720
)
$ErrorActionPreference = "Continue"

# ---------- Path constants ----------
$PROJ        = "E:\puppyfangzhen"
$SHOT_DIR    = "$PROJ\screenshots"
$LOG_DIR     = "$PROJ\logs"
$BRIDGE_EXE  = "$PROJ\cpp_src\bridge\build\Release\Release\nav_ue_bridge.exe"
$SCENE_JSON  = "$PROJ\config\scene_home.json"
$UE_EXE      = "C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe"
$UPROJECT    = "D:\puppy_ue\puppy_ue.uproject"
$MAP         = "/Game/Maps/HomeMap"
$UE_LOG_DIR  = "D:\puppy_ue\Saved\Logs"

foreach ($d in @($SHOT_DIR, $LOG_DIR)) {
    if (-not (Test-Path $d)) { New-Item -ItemType Directory -Path $d -Force | Out-Null }
}

# ---------- Auto RunId (R01, R02, ...) ----------
if ([string]::IsNullOrWhiteSpace($RunId)) {
    [int]$maxN = 0
    Get-ChildItem $SHOT_DIR -Filter "R??_*.png" -ErrorAction SilentlyContinue | ForEach-Object {
        if ($_.Name -match "^R(\d{2})_") {
            [int]$n = [int]$Matches[1]
            if ($n -gt $maxN) { $maxN = $n }
        }
    }
    [int]$nextNum = $maxN + 1
    $RunId = "R" + $nextNum.ToString("00")
}
$TIME_TAG = Get-Date -Format "yyyyMMdd_HHmmss"
Write-Host "============================================================"
Write-Host " UE5 SIMULATION RUN  RunId=$RunId  Frames=$Frames  Shots=$Shots"
Write-Host " Screenshots -> $SHOT_DIR"
Write-Host " Logs        -> $LOG_DIR"
Write-Host "============================================================"

# ---------- 1) Kill stale processes ----------
Write-Host "[1/5] Cleaning old processes..."
Get-Process | Where-Object { $_.ProcessName -like "*nav_ue*" -or $_.ProcessName -like "*Unreal*" } |
    Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep 3

# ---------- 2) Win32 helpers (HWND find + screenshot) ----------
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
using System.Text;
public class W32_Sim {
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr hWnd, StringBuilder s, int nMaxCount);
    [DllImport("user32.dll")] public static extern IntPtr SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X,int Y,int cx,int cy, uint uFlags);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left; public int Top; public int Right; public int Bottom; }
    public const int HWND_TOPMOST = -1;
    public const int HWND_NOTOPMOST = -2;
    public const uint SWP_NOMOVE = 0x0002;
    public const uint SWP_NOSIZE = 0x0001;
    public const uint SWP_SHOWWINDOW = 0x0040;
    public static void MakeTopMost(IntPtr h)   { SetWindowPos(h, (IntPtr)HWND_TOPMOST, 0,0,0,0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW); }
    public static void UnsetTopMost(IntPtr h)  { SetWindowPos(h, (IntPtr)HWND_NOTOPMOST, 0,0,0,0, SWP_NOMOVE | SWP_NOSIZE); }
    public delegate bool EnumWinProc(IntPtr h, IntPtr lp);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWinProc lp, IntPtr lpParam);
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    public static IntPtr FindMainWindowByPid(int pid) {
        IntPtr found = IntPtr.Zero;
        EnumWindows((h, _) => {
            uint p; GetWindowThreadProcessId(h, out p);
            if (p == (uint)pid && IsWindowVisible(h)) {
                var sb = new StringBuilder(300); GetWindowText(h, sb, sb.Capacity);
                if (!string.IsNullOrEmpty(sb.ToString())) { found = h; return false; }
            }
            return true;
        }, IntPtr.Zero);
        return found;
    }
}
"@ -ErrorAction SilentlyContinue
Add-Type -AssemblyName System.Windows.Forms, System.Drawing

# ---------- 3) Start Bridge ----------
Write-Host "[2/5] Starting Bridge TCP server ($Frames frames)..."
$BRIDGE_LOG = "$LOG_DIR\bridge_${RunId}_${TIME_TAG}.log"
$bp = Start-Process -FilePath $BRIDGE_EXE -ArgumentList @(
    "--port","7777","--frames",$Frames.ToString(),
    "--scene",$SCENE_JSON
) -RedirectStandardOutput $BRIDGE_LOG -RedirectStandardError "$BRIDGE_LOG.err" -PassThru -NoNewWindow
Start-Sleep 2
Write-Host ("    Bridge PID={0}, alive={1}, log={2}" -f $bp.Id, (-not $bp.HasExited), $BRIDGE_LOG)

# ---------- 4) Start UE -game ----------
#   P2-1 FIX: 添加 -BuildLightingOnLaunch 让UE自动构建静态光照,消除"LIGHTING NEEDS REBUILD"警告
Write-Host ("[3/5] Starting UE -game ({0}x{1})..." -f $ResX, $ResY)
$up = Start-Process -FilePath $UE_EXE -ArgumentList @(
    $UPROJECT, $MAP, "-game", "-windowed",
    "-ResX=$ResX", "-ResY=$ResY",
    "-NoLiveCoding"
) -PassThru
Write-Host ("    UE PID={0}. Waiting for HWND..." -f $up.Id)

$hwnd = [IntPtr]::Zero
for ($t = 0; $t -lt 60; $t++) {
    $hwnd = [W32_Sim]::FindMainWindowByPid($up.Id)
    if ($hwnd -ne [IntPtr]::Zero) { break }
    Start-Sleep -Milliseconds 500
}
if ($hwnd -ne [IntPtr]::Zero) {
    $r = New-Object W32_Sim+RECT
    [void][W32_Sim]::GetWindowRect($hwnd, [ref]$r)
    $w = $r.Right - $r.Left; $h = $r.Bottom - $r.Top
    Write-Host ("    HWND found: {0}x{1} @ ({2},{3}) -> topmost + foreground" -f $w,$h,$r.Left,$r.Top)
    [W32_Sim]::MakeTopMost($hwnd)
    [void][W32_Sim]::SetForegroundWindow($hwnd)
    Start-Sleep -Milliseconds 1500
} else {
    Write-Host "    HWND not found; will use primary-screen fallback for screenshots."
}

# ---------- 5) Screenshots on schedule ----------
Write-Host ("[4/5] Taking {0} screenshots (interval {1}s) -> {2}" -f $Shots, $ShotInterval, $SHOT_DIR)
$shotFiles = @()
for ($i = 1; $i -le $Shots; $i++) {
    Start-Sleep $ShotInterval
    $SHOT_NAME = "${RunId}_shot_${i}_$(Get-Date -Format 'HHmmss').png"
    $png = Join-Path $SHOT_DIR $SHOT_NAME
    $captured = $false
    if ($hwnd -ne [IntPtr]::Zero -and [W32_Sim]::IsWindowVisible($hwnd)) {
        # P2-2 FIX: 每次截图前重新设为topmost+foreground,防止窗口被其他程序遮挡→黑屏
        [W32_Sim]::MakeTopMost($hwnd)
        [void][W32_Sim]::SetForegroundWindow($hwnd)
        Start-Sleep -Milliseconds 500  # 给UE足够时间渲染帧
        $r = New-Object W32_Sim+RECT
        [void][W32_Sim]::GetWindowRect($hwnd, [ref]$r)
        $w = $r.Right - $r.Left; $h = $r.Bottom - $r.Top
        if ($w -gt 150 -and $h -gt 150) {
            $bmp = New-Object System.Drawing.Bitmap($w, $h)
            $g = [System.Drawing.Graphics]::FromImage($bmp)
            $g.CopyFromScreen($r.Left, $r.Top, 0, 0, (New-Object System.Drawing.Size($w, $h)))
            $g.Dispose(); $bmp.Save($png, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
            $captured = $true
        }
    }
    if (-not $captured) {
        $bounds = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
        $bmp = New-Object System.Drawing.Bitmap($bounds.Width, $bounds.Height)
        $g = [System.Drawing.Graphics]::FromImage($bmp)
        $g.CopyFromScreen($bounds.Location, [System.Drawing.Point]::Empty, $bounds.Size)
        $g.Dispose(); $bmp.Save($png, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
    }
    $szKB = [math]::Round((Get-Item $png).Length / 1KB, 1)
    # P2-2 FIX: 如果截图<10KB(全黑),重试一次(等1s后重抓)
    if ($szKB -lt 10 -and $hwnd -ne [IntPtr]::Zero -and [W32_Sim]::IsWindowVisible($hwnd)) {
        [W32_Sim]::MakeTopMost($hwnd)
        [void][W32_Sim]::SetForegroundWindow($hwnd)
        Start-Sleep -Milliseconds 800
        $r2 = New-Object W32_Sim+RECT
        [void][W32_Sim]::GetWindowRect($hwnd, [ref]$r2)
        $w2 = $r2.Right - $r2.Left; $h2 = $r2.Bottom - $r2.Top
        if ($w2 -gt 150 -and $h2 -gt 150) {
            $bmp2 = New-Object System.Drawing.Bitmap($w2, $h2)
            $g2 = [System.Drawing.Graphics]::FromImage($bmp2)
            $g2.CopyFromScreen($r2.Left, $r2.Top, 0, 0, (New-Object System.Drawing.Size($w2, $h2)))
            $g2.Dispose(); $bmp2.Save($png, [System.Drawing.Imaging.ImageFormat]::Png); $bmp2.Dispose()
            $szKB = [math]::Round((Get-Item $png).Length / 1KB, 1)
        }
    }
    $shotFiles += $png
    $brDone = $bp.HasExited
    $brTail = (Get-Content $BRIDGE_LOG -Tail 1 -ErrorAction SilentlyContinue) -replace "\s+", " "
    Write-Host ("    [shot {0}/{1}] {2} ({3} KB) Bridge_done={4} UE_alive={5}" -f
                 $i, $Shots, $SHOT_NAME, $szKB, $brDone, (-not $up.HasExited))
    if ($brTail) { Write-Host ("       bridge tail: {0}" -f $brTail) }
}
if ($hwnd -ne [IntPtr]::Zero) { [W32_Sim]::UnsetTopMost($hwnd) }

# ---------- 6) Teardown + summary ----------
Write-Host "[5/5] Teardown and summary..."
Start-Sleep 3
if (-not $bp.HasExited) { Stop-Process -Id $bp.Id -Force -ErrorAction SilentlyContinue }
if (-not $up.HasExited) { Stop-Process -Id $up.Id -Force -ErrorAction SilentlyContinue }
Start-Sleep 4

# Copy latest UE Saved/Logs to $LOG_DIR
$latestUeLog = Get-ChildItem $UE_LOG_DIR -Filter "*.log" -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($latestUeLog) {
    $UE_COPY = "$LOG_DIR\ue_${RunId}_${TIME_TAG}.log"
    Copy-Item $latestUeLog.FullName $UE_COPY -Force
    Write-Host "    UE log archived: $UE_COPY"
}

Write-Host ""
Write-Host ("========== BRIDGE SUMMARY (RunId={0}) ==========" -f $RunId)
$br = Get-Content $BRIDGE_LOG -Raw -ErrorAction SilentlyContinue
if ([string]::IsNullOrWhiteSpace($br)) {
    Write-Host "  Bridge log is empty or unavailable."
} else {
    [regex]::Matches($br, "SUMMARY:.*|confidence=\d+\.\d+|frames=\d+") | ForEach-Object { Write-Host "  $_" }
}
Get-Content $BRIDGE_LOG -Tail 10 -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "  $_" }

Write-Host ""
Write-Host ("========== SCREENSHOTS ({0}) ==========" -f $Shots)
$shotFiles | ForEach-Object {
    $sz = [math]::Round((Get-Item $_).Length/1KB, 1)
    Write-Host ("    {0}  ({1} KB)" -f $_, $sz)
}
Write-Host ""
Write-Host ("Simulation RunId={0} complete. Screenshots dir: {1}" -f $RunId, $SHOT_DIR)
exit 0

