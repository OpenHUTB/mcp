"""
配置管理模块
负责加载和管理所有配置设置
"""

import os
from dotenv import load_dotenv
from typing import Optional

# 加载环境变量
load_dotenv()

class Config:
    """配置类，包含所有应用程序设置"""
    
    # GitHub API配置
    GITHUB_TOKEN: str = os.getenv("GITHUB_TOKEN", "")
    GITHUB_BASE_URL: str = os.getenv("GITHUB_BASE_URL", "https://api.github.com")
    
    # Deepseek AI配置
    DEEPSEEK_API_KEY: str = os.getenv("DEEPSEEK_API_KEY", "")
    DEEPSEEK_API_URL: str = os.getenv("DEEPSEEK_API_URL", "https://api.deepseek.com/chat/completions")
    
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
        
        if not cls.GITHUB_TOKEN:
            print("警告: GITHUB_TOKEN 未设置，API调用可能受限")
            valid = False
            
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