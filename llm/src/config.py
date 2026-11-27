"""
配置管理模块
负责加载和管理所有配置设置
"""

import os
from pathlib import Path
from dotenv import load_dotenv
from typing import Optional

# 显式加载项目根目录与 llm/ 目录下的 .env，避免依赖启动时的工作目录
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_LLM_DIR = Path(__file__).resolve().parents[1]
load_dotenv(_PROJECT_ROOT / ".env")
load_dotenv(_LLM_DIR / ".env")

class Config:
    """配置类，包含所有应用程序设置"""
    
    # GitHub API配置
    GITHUB_TOKEN: str = os.getenv("GITHUB_TOKEN", "")
    GITHUB_BASE_URL: str = os.getenv("GITHUB_BASE_URL", "https://api.github.com")
    
    # Deepseek AI配置（Key 请放在项目根目录 .env，不要写进代码）
    DEEPSEEK_API_KEY: str = os.getenv("DEEPSEEK_API_KEY", "")
    DEEPSEEK_API_URL: str = os.getenv("DEEPSEEK_API_URL", "https://api.deepseek.com/chat/completions")
    DEEPSEEK_MODEL: str = os.getenv("DEEPSEEK_MODEL", "deepseek-chat")
    
    # MCP服务器配置
    MCP_SERVER_NAME: str = os.getenv("MCP_SERVER_NAME", "github-search-mcp")
    MCP_SERVER_VERSION: str = os.getenv("MCP_SERVER_VERSION", "1.0.0")
    
    # 日志配置
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    
    # 缓存配置
    CACHE_TTL: int = int(os.getenv("CACHE_TTL", "300"))
    
    # API限制配置
    GITHUB_API_RATE_LIMIT: int = int(os.getenv("GITHUB_API_RATE_LIMIT", "5000"))
    GITHUB_API_TIMEOUT: int = int(os.getenv("GITHUB_API_TIMEOUT", "30"))
    
    @classmethod
    def validate(cls) -> bool:
        """验证配置是否完整"""
        valid = True
            
        if not cls.DEEPSEEK_API_KEY:
            print("错误: DEEPSEEK_API_KEY 未设置，AI功能将无法使用")
            valid = False
            
        return valid
    
    @classmethod
    def get_github_headers(cls) -> dict:
        """获取GitHub API请求头"""
        headers = {
            "Accept": "application/vnd.github.v3+json",
            "User-Agent": f"{cls.MCP_SERVER_NAME}/{cls.MCP_SERVER_VERSION}"
        }
        
        if cls.GITHUB_TOKEN:
            headers["Authorization"] = f"token {cls.GITHUB_TOKEN}"
            
        return headers
    
    @classmethod
    def get_deepseek_headers(cls) -> dict:
        """获取Deepseek API请求头"""
        return {
            "Authorization": f"Bearer {cls.DEEPSEEK_API_KEY}",
            "Content-Type": "application/json"
        }

# 全局配置实例
config = Config() 