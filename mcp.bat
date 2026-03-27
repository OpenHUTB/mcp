@echo off

REM Enable delayed expansion
setlocal enabledelayedexpansion

REM Get script directory
set "PROJECT_ROOT=%~dp0"
set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"

REM Set virtual environment and Python paths
set "VENV_PATH=%PROJECT_ROOT%\dependencies\prerequisites\miniconda3\envs\hutb_3.10"
set "PYTHON_EXE=%VENV_PATH%\python.exe"

REM Set CarlaUE4.exe path
set "CARLA_EXE=%PROJECT_ROOT%\hutb\CarlaUE4.exe"

REM Set main_ai.py path
set "MAIN_AI_PY=%PROJECT_ROOT%\llm\main_ai.py"

REM Define port and URL to check
for /f "tokens=16" %%i in ('ipconfig ^|find /i "ipv4"') do set host_ip=%%i
echo IP:%host_ip%
set "PORT=3000"
set "CHECK_URL=http://%host_ip%:%PORT%"

REM Maximum wait time in seconds for main_ai.py to start
set "MAX_WAIT=60"

REM Wait time after starting CarlaUE4.exe
set "POST_CARLA_WAIT=3"

if not exist "%PROJECT_ROOT%\hutb_downloader.exe" (
    curl -L -o "hutb_downloader.exe" "https://gitee.com/OpenHUTB/sw/releases/download/up/hutb_downloader.exe"
) else (
    echo hutb_downloader.exe already exists.
)

REM 如果 dependencies 目录不存在，则下载
if not exist "%PROJECT_ROOT%\dependencies" (
    echo dependencies directory not found. Downloading...
    start /wait "" "%PROJECT_ROOT%\hutb_downloader.exe" --repository dependencies
    echo Download and extraction dependencies completed.
) else (
    echo dependencies repository already exists.
)

REM 如果之前存在模拟器进程（包括后台进程），则先杀掉
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :2000') do taskkill /F /PID %%a

REM 如果 hutb 目录不存在，则下载
if not exist "%PROJECT_ROOT%\hutb" (
    echo hutb directory not found. Downloading...

    REM 调用 hutb_downloader.exe，等待执行完成
    start /wait "" "%PROJECT_ROOT%\hutb_downloader.exe"
    echo Download and extraction completed.
) else (
    echo hutb repository already exists.
    REM Check if CarlaUE4.exe exists
    if not exist "%CARLA_EXE%" (
        echo Warning: CarlaUE4.exe not found, skipping startup
        echo CarlaUE4.exe path: %CARLA_EXE%
    )
    start "CarlaUE4" "%CARLA_EXE%"
    
    REM Wait for specified time after starting CarlaUE4
    timeout /t %POST_CARLA_WAIT% /nobreak >nul
)

REM 为了解压miniconda3
if not exist "dependencies\prerequisites\7zip" (
    echo Unzipping 7zip ...
    powershell -Command "Expand-Archive -Path 'dependencies\prerequisites\7zip.zip' -DestinationPath 'dependencies\prerequisites\' -Force" || exit /b
) else (
    echo 7zip folder already exists.
)
if not exist "dependencies\prerequisites\miniconda3\" (
    echo Unzipping miniconda...
    "dependencies\prerequisites\7zip\7z.exe" x "dependencies\prerequisites\miniconda3.zip" -o"dependencies\prerequisites\" -y >nul
) else (
    echo miniconda3 folder already exists.
)


REM Check if virtual environment exists
if not exist "%VENV_PATH%" (
    echo Error: Virtual environment not found at %VENV_PATH%
    pause
    exit /b 1
)

REM Check if Python interpreter exists
if not exist "%PYTHON_EXE%" (
    echo Error: Python interpreter not found at %PYTHON_EXE%
    pause
    exit /b 1
)

REM Check if main_ai.py exists
if not exist "%MAIN_AI_PY%" (
    echo Error: main_ai.py not found at %MAIN_AI_PY%
    pause
    exit /b 1
)

REM Print activation information
echo Activating virtual environment...

REM Print Python version
echo Virtual environment activated successfully!
echo Python version:
%PYTHON_EXE% --version

echo Install hutb package:
REM 需要关闭代理，解决安装 whl 时的代理问题: WARNING: Retrying (Retry(total=4, connect=None, read=None, redirect=None, status=None)) after connection broken by 'ProxyError('Cannot connect to proxy.', ConnectionResetError(10054, '远程主机强迫关闭了一个现有的连接。', None, 10054, None))': /simple/msgpack-rpc-python/
REM 制作 Python 环境步骤：
REM dependencies/prerequisites/miniconda3/envs/hutb_3.10/python.exe -m pip install hutb\PythonAPI\carla\dist\hutb-2.9.16-cp310-cp310-win_amd64.whl
REM dependencies/prerequisites/miniconda3/envs/hutb_3.10/python.exe -m pip install fastapi uvicorn aiohttp fastmcp loguru

REM Set environment variables
set "PATH=%VENV_PATH%\Scripts;%VENV_PATH%;%PATH%"
set "VIRTUAL_ENV=%VENV_PATH%"

REM Check if OpenCV (cv2) is installed
echo Checking OpenCV (cv2)...
"%PYTHON_EXE%" "%PROJECT_ROOT%\llm\check_opencv.py"
if errorlevel 1 (
    pause
    exit /b 1
)

REM 1. First, run main_ai.py
echo Running main_ai.py...
start "main_ai" "%PYTHON_EXE%" "%MAIN_AI_PY%"

REM Wait for 5 seconds initially to give main_ai.py time to start
timeout /t 5 /nobreak >nul

REM Wait for main_ai.py to be fully ready to handle requests
echo Waiting for main_ai.py to be ready at %CHECK_URL%...
set "WAIT_COUNT=0"
:WAIT_LOOP
REM Check if port is in use first
netstat -an | findstr ":%PORT% LISTENING" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    REM Port not listening yet, wait and retry
    goto :CHECK_TIMER
)

REM If port is listening, try to get a successful HTTP response
curl -s -o NUL -w "%%{http_code}" "%CHECK_URL%" | findstr "200" >nul 2>&1
if %ERRORLEVEL% equ 0 (
    echo main_ai.py is ready and responding at %CHECK_URL%!
    goto :START_CARLA
)

:CHECK_TIMER
REM Increment wait count
set /a WAIT_COUNT=WAIT_COUNT+1

REM Check if maximum wait time exceeded
if !WAIT_COUNT! geq %MAX_WAIT% (
    echo Warning: Maximum wait time exceeded (%MAX_WAIT% seconds)
    echo main_ai.py may not be fully ready, but continuing with CarlaUE4.exe startup...
    goto :START_CARLA
)

REM Wait for 2 seconds before checking again
echo Waiting for %PORT%... (Attempt !WAIT_COUNT! of %MAX_WAIT%)
timeout /t 2 /nobreak >nul
goto :WAIT_LOOP

:START_CARLA
REM 2. Then, start CarlaUE4.exe asynchronously
if exist "%CARLA_EXE%" (
    echo Existing CarlaUE4.exe...
)

REM 3. Finally, open browser to localhost:3000
echo Opening browser to %CHECK_URL%...
start "" "%CHECK_URL%"

REM Set custom prompt
prompt [hutb_3.10] $P$G

REM Keep terminal open
cmd /k