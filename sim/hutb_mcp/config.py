"""
HUTB MCP 配置管理模块

从环境变量和配置文件加载配置参数。
"""

import os
from dataclasses import dataclass, field
from typing import Optional
from dotenv import load_dotenv

# 加载 .env 文件
load_dotenv()


@dataclass
class HutbConfig:
    """HUTB 仿真器连接配置"""
    host: str = field(default_factory=lambda: os.getenv("HUTB_HOST", "localhost"))
    port: int = field(default_factory=lambda: int(os.getenv("HUTB_PORT", "2000")))
    timeout: float = field(default_factory=lambda: float(os.getenv("HUTB_TIMEOUT", "10.0")))


@dataclass
class MCPConfig:
    """MCP 服务器配置"""
    server_name: str = field(default_factory=lambda: os.getenv("MCP_SERVER_NAME", "hutb-mcp"))
    server_version: str = field(default_factory=lambda: os.getenv("MCP_SERVER_VERSION", "0.1.0"))


@dataclass
class LogConfig:
    """日志配置"""
    level: str = field(default_factory=lambda: os.getenv("LOG_LEVEL", "INFO"))
    log_file: Optional[str] = field(default_factory=lambda: os.getenv("LOG_FILE", "hutb_mcp.log"))


@dataclass
class Config:
    """全局配置类"""
    hutb: HutbConfig = field(default_factory=HutbConfig)
    mcp: MCPConfig = field(default_factory=MCPConfig)
    log: LogConfig = field(default_factory=LogConfig)
    default_map: str = field(default_factory=lambda: os.getenv("DEFAULT_MAP", "Town10"))
    
    def validate(self) -> bool:
        """验证配置是否有效"""
        if self.hutb.port < 1 or self.hutb.port > 65535:
            return False
        if self.hutb.timeout <= 0:
            return False
        return True


# 全局配置实例
config = Config()


def get_config() -> Config:
    """获取全局配置实例"""
    return config


def reload_config() -> Config:
    """重新加载配置"""
    global config
    load_dotenv(override=True)
    config = Config()
    return config
