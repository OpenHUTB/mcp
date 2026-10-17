#!/bin/bash

export PYTHONUTF8=1

# 检查 conda 是否安装
if ! command -v conda >/dev/null 2>&1; then
    echo "conda is not installed. Please install Anaconda or Miniconda first."
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
    bash ~/miniconda.sh -b -p $HOME/miniconda
    eval "$(/$HOME/miniconda/bin/conda shell.bash hook)"
    conda init
fi

echo "conda is installed in:"
which conda

# 初始化 conda（用于当前 shell）
CONDA_PATH=$(which conda)
CONDA_DIR=$(dirname "$(dirname "$CONDA_PATH")")
echo "conda is installed at: $CONDA_PATH"
echo "conda directory is: $CONDA_DIR"

# shellcheck disable=SC1091
source "$CONDA_DIR/etc/profile.d/conda.sh"

# 检查 mkdocs 虚拟环境是否存在
if ! conda env list | grep -q "mkdocs"; then
    echo "mkdocs virtual environment does not exist. Creating it now..."
    conda create -n mkdocs python=3.11 -y
    echo "mkdocs virtual environment has been created."
fi

# 激活 mkdocs 环境
conda activate mkdocs

# 安装依赖
pip install -r requirements.txt
pip install git+https://github.com/OpenHUTB/mkdocs.git

# 配置
host_ip=127.0.0.1
PORT=8000
CHECK_URL="http://${host_ip}:${PORT}"

# 构建并启动服务
mkdocs build
xdg-open "$CHECK_URL" >/dev/null 2>&1 &
mkdocs serve --livereload
