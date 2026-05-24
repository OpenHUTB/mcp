"""
HUTB MCP 命令行入口

允许通过 `python -m hutb_mcp` 启动服务器。
"""

from .server import main

if __name__ == "__main__":
    main()
