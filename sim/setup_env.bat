@echo off
REM HUTB MCP 服务器环境设置脚本
REM 作者: 徐杨杨
REM 用途: 设置虚拟环境并安装依赖

echo ========================================
echo HUTB MCP 服务器环境设置
echo ========================================

REM 获取脚本目录
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "PROJECT_ROOT=%SCRIPT_DIR%\.."
set "VENV_DIR=%PROJECT_ROOT%\env.UE4-hutb"

REM 检查 Python 是否可用
where python >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo 错误: 未找到 Python，请先安装 Python 3.10+
    pause
    exit /b 1
)

echo 当前 Python 版本:
python --version

REM 检查虚拟环境是否已存在
if exist "%VENV_DIR%" (
    echo 虚拟环境已存在: %VENV_DIR%
    echo.
    choice /C YN /M "是否重新创建虚拟环境"
    if errorlevel 2 goto :install_deps
    echo 删除旧的虚拟环境...
    rmdir /s /q "%VENV_DIR%"
)

REM 创建虚拟环境
echo 创建虚拟环境: %VENV_DIR%
python -m venv "%VENV_DIR%"
if %ERRORLEVEL% neq 0 (
    echo 错误: 创建虚拟环境失败
    pause
    exit /b 1
)

echo 虚拟环境创建成功！

:install_deps
REM 激活虚拟环境
echo 激活虚拟环境...
call "%VENV_DIR%\Scripts\activate.bat"

REM 升级 pip
echo.
echo 升级 pip...
python -m pip install --upgrade pip

REM 检查 HUTB wheel 文件
set "HUTB_WHEEL=D:\hutb\PythonAPI\carla\dist\hutb-2.9.16-cp310-cp310-win_amd64.whl"
if exist "%HUTB_WHEEL%" (
    echo.
    echo 安装 HUTB Python API...
    pip install "%HUTB_WHEEL%"
) else (
    echo.
    echo 警告: 未找到 HUTB wheel 文件: %HUTB_WHEEL%
    echo 请手动安装 HUTB Python API
)

REM 安装项目依赖
echo.
echo 安装项目依赖...
pip install -e .

REM 复制环境变量文件
if not exist ".env" (
    if exist ".env.example" (
        echo.
        echo 创建 .env 配置文件...
        copy .env.example .env
    )
)

echo.
echo ========================================
echo 环境设置完成！
echo ========================================
echo.
echo 虚拟环境位置: %VENV_DIR%
echo.
echo 使用方法:
echo   1. 启动 CarlaUE4.exe (HUTB 仿真器)
echo   2. 运行: run_server.bat
echo.
echo 手动激活虚拟环境:
echo   %VENV_DIR%\Scripts\activate.bat
echo.
echo 测试命令:
echo   python -c "import carla; print('CARLA 模块导入成功')"
echo   python -c "from hutb_mcp import main; print('HUTB MCP 模块导入成功')"
echo.
pause
