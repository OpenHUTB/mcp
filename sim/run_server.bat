@echo off
REM HUTB MCP 服务器启动脚本
REM 作者: 徐杨杨

echo ========================================
echo 启动 HUTB MCP 服务器
echo ========================================

REM 获取脚本目录
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "PROJECT_ROOT=%SCRIPT_DIR%\.."
set "VENV_DIR=%PROJECT_ROOT%\env.UE4-hutb"

REM 检查虚拟环境是否存在
if not exist "%VENV_DIR%\Scripts\activate.bat" (
    echo 错误: 虚拟环境未找到
    echo 请先运行 setup_env.bat 创建虚拟环境
    pause
    exit /b 1
)

REM 激活虚拟环境
echo 激活虚拟环境: %VENV_DIR%
call "%VENV_DIR%\Scripts\activate.bat"

REM 检查环境是否激活成功
python --version >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo 错误: Python 环境未正确激活
    pause
    exit /b 1
)

echo Python 版本:
python --version

REM 检查 CARLA 模块
python -c "import carla" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo 警告: CARLA 模块未安装，部分功能将不可用
)

REM 启动服务器
echo 启动 HUTB MCP 服务器...
python -m hutb_mcp

pause
