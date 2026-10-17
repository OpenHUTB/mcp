@echo off

set PYTHONUTF8=1

where conda >nul 2>nul

if %errorlevel% neq 0 (
    echo conda is not installed. Please install Anaconda or Miniconda first.
    curl https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe -o .\miniconda.exe
    start /wait "" .\miniconda.exe /S
    del .\miniconda.exe
    echo conda has been installed. Please restart the command prompt and run this script again.
)

echo conda is installed in:
where conda


conda info --envs | findstr /C:"mkdocs" >nul 2>nul
if %errorlevel% neq 0 (
    echo mkdocs virtual environment does not exist. Creating it now...
    conda create -n mkdocs python=3.11 -y
    echo mkdocs virtual environment has been created.
)
pip install -r requirements.txt
pip install git+https://github.com/OpenHUTB/mkdocs.git


for /f "tokens=*" %%i in ('where conda') do set CONDA_PATH=%%i
echo conda is installed at: %CONDA_PATH%
for %%i in ("%CONDA_PATH%\..\..") do set "CONDA_DIR=%%~fi"
echo conda directory is: %CONDA_DIR%


set host_ip=127.0.0.1
set "PORT=8000"
set "CHECK_URL=http://%host_ip%:%PORT%"

%WINDIR%\System32\WindowsPowerShell\v1.0\powershell.exe -ExecutionPolicy ByPass -NoExit -Command "& '%CONDA_DIR%\shell\condabin\conda-hook.ps1' ; conda activate '%CONDA_DIR%' "; conda activate mkdocs; mkdocs build; start "" "%CHECK_URL%"; mkdocs serve --livereload; 
