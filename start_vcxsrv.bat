@echo off
REM ============================================================
REM VcXsrv 启动脚本 - 用于显示 WSL2 中的 Linux GUI 应用
REM 配置：多窗口模式，允许来自 WSL2 的连接
REM ============================================================

REM 设置 VcXsrv 安装路径
set VCXSRV_PATH=C:\Program Files\VcXsrv\vcxsrv.exe

REM 检查 VcXsrv 是否安装
if not exist "%VCXSRV_PATH%" (
    echo [错误] 找不到 VcXsrv: %VCXSRV_PATH%
    echo 请先从 https://sourceforge.net/projects/vcxsrv/ 下载安装
    pause
    exit /b 1
)

echo ========================================
echo   VcXsrv X Server 启动中...
echo ========================================
echo.
echo 显示设置:
echo   - 显示编号: 0 (DISPLAY=:0)
echo   - 多窗口模式
echo   - 允许来自 WSL2 的连接
echo   - 禁用访问控制（方便 WSL2 连接）
echo.
echo 启动后会在 Windows 任务栏右下角出现 X 图标
echo 保持 VcXsrv 运行，然后启动仿真脚本
echo.
echo 按任意键继续...
pause

REM 启动 VcXsrv
REM -multiwindow  : 多窗口模式（每个 Linux 窗口 = 一个 Windows 窗口）
REM -ac          : 禁用访问控制（允许 WSL2 连接）
REM -nowgl       : 禁用 WGL（减少兼容性问题）
REM -clipboard   : 启用剪贴板共享
REM :0           : 显示编号 0
start "" "%VCXSRV_PATH%" :0 -multiwindow -ac -nowgl -clipboard

echo.
echo VcXsrv 已启动！请检查任务栏右下角是否有 X 图标
echo.
echo 现在可以启动仿真了！
echo.
pause
