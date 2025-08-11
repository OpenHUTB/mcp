"""
GitHub API客户端
用于与GitHub API进行交互的客户端类
"""

import aiohttp
import asyncio
from typing import Dict, List, Optional, Any
from src.config import config
from src.utils.logger import app_logger

class GitHubClient:
    """GitHub API客户端"""
    
    def __init__(self):
        self.base_url = config.GITHUB_BASE_URL
        self.headers = config.get_github_headers()
        self.timeout = config.GITHUB_API_TIMEOUT
        
    async def _make_request(self, method: str, endpoint: str, params: Optional[Dict] = None) -> Dict:
        """发送HTTP请求到GitHub API"""
        url = f"{self.base_url}/{endpoint.lstrip('/')}"
        
        app_logger.debug(f"Making {method} request to: {url}")
        
        async with aiohttp.ClientSession() as session:
            try:
                async with session.request(
                    method,
                    url,
                    headers=self.headers,
                    params=params,
                    timeout=aiohttp.ClientTimeout(total=self.timeout)
                ) as response:
                    
                    if response.status == 200:
                        data = await response.json()
                        return data
                    elif response.status == 403:
                        raise Exception("GitHub API rate limit exceeded or access forbidden")
                    elif response.status == 404:
                        raise Exception("Resource not found")
                    else:
                        raise Exception(f"GitHub API error: HTTP {response.status}")
                        
            except aiohttp.ClientError as e:
                app_logger.error(f"Network error: {str(e)}")
                raise Exception(f"Network error: {str(e)}")
            except asyncio.TimeoutError:
                app_logger.error("Request timeout")
                raise Exception("Request timeout")
    
    async def search_repositories(self, query: str, language: Optional[str] = None, 
                                sort: str = "stars", order: str = "desc", per_page: int = 10) -> List[Dict]:
        """搜索GitHub仓库"""
        search_query = query
        if language:
            search_query += f" language:{language}"
        
        params = {
            "q": search_query,
            "sort": sort,
            "order": order,
            "per_page": min(per_page, 100)
        }
        
        app_logger.info(f"Searching repositories with query: {search_query}")
        
        try:
            data = await self._make_request("GET", "search/repositories", params)
            repositories = data.get("items", [])
            app_logger.info(f"Found {len(repositories)} repositories")
            return repositories
        except Exception as e:
            app_logger.error(f"Error searching repositories: {str(e)}")
            raise
    
    async def get_repository_info(self, owner: str, repo: str) -> Dict:
        """获取特定仓库的详细信息"""
        endpoint = f"repos/{owner}/{repo}"
        app_logger.info(f"Getting repository info for: {owner}/{repo}")
        
        try:
            data = await self._make_request("GET", endpoint)
            return data
        except Exception as e:
            app_logger.error(f"Error getting repository info: {str(e)}")
            raise
    
    async def search_users(self, query: str, type: Optional[str] = None, per_page: int = 10) -> List[Dict]:
        """搜索GitHub用户"""
        search_query = query
        if type:
            search_query += f" type:{type}"
        
        params = {
            "q": search_query,
            "per_page": min(per_page, 100)
        }
        
        app_logger.info(f"Searching users with query: {search_query}")
        
        try:
            data = await self._make_request("GET", "search/users", params)
            users = data.get("items", [])
            app_logger.info(f"Found {len(users)} users")
            
            # 并行获取每个用户的详细信息（包含完整统计数据）
            detailed_users = await self._get_users_details_parallel(users)
            return detailed_users
        except Exception as e:
            app_logger.error(f"Error searching users: {str(e)}")
            raise

    async def get_user_info(self, username: str) -> Dict:
        """获取特定用户的详细信息"""
        endpoint = f"users/{username}"
        app_logger.debug(f"Getting user info for: {username}")
        
        try:
            data = await self._make_request("GET", endpoint)
            return data
        except Exception as e:
            app_logger.error(f"Error getting user info: {str(e)}")
            raise

    async def _get_user_details_safe(self, user: Dict) -> Dict:
        """安全获取单个用户详细信息"""
        try:
            detailed_user = await self.get_user_info(user['login'])
            return detailed_user
        except Exception as e:
            app_logger.warning(f"Failed to get details for user {user['login']}: {str(e)}")
            # 如果获取详细信息失败，使用基本信息
            return user

    async def _get_users_details_parallel(self, users: List[Dict]) -> List[Dict]:
        """并行获取多个用户的详细信息"""
        if not users:
            return []
        
        app_logger.info(f"并行获取 {len(users)} 个用户的详细信息")
        
        # 使用 asyncio.gather 并行处理，但限制并发数量避免触发rate limit
        semaphore = asyncio.Semaphore(5)  # 最多同时5个请求
        
        async def limited_get_details(user):
            async with semaphore:
                return await self._get_user_details_safe(user)
        
        try:
            detailed_users = await asyncio.gather(
                *[limited_get_details(user) for user in users],
                return_exceptions=False
            )
            return detailed_users
        except Exception as e:
            app_logger.error(f"Error in parallel user details fetch: {str(e)}")
            # 如果并行获取失败，返回基本信息
            return users
    

    async def get_api_info(self) -> Dict:
        """获取API信息"""
        try:
            data = await self._make_request("GET", "rate_limit")
            return {
                "api_status": "connected",
                "rate_limit": data.get("rate", {}),
                "authenticated": bool(config.GITHUB_TOKEN),
                "base_url": self.base_url
            }
        except Exception as e:
            return {
                "api_status": "error",
                "error": str(e),
                "authenticated": bool(config.GITHUB_TOKEN),
                "base_url": self.base_url
            }
