"""
GitHub Search FastMCP Server
基于FastMCP框架的GitHub仓库搜索服务器
使用装饰器方式注册MCP工具
"""

import asyncio
import sys
import os
from typing import Optional

# 添加父目录到Python路径，以便导入src模块
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from fastmcp import FastMCP
from src.config import config
from src.utils.logger import app_logger
from src.github_client import GitHubClient

# 创建FastMCP实例
mcp = FastMCP("GitHub搜索助手")

# 创建GitHub客户端实例
github_client = GitHubClient()

@mcp.tool()
def search_repositories(query: str, language: Optional[str] = None, 
                       sort: str = "stars", per_page: int = 10) -> str:
    """搜索GitHub仓库工具
    
    根据关键词、编程语言等条件搜索GitHub仓库，返回热门匹配结果。
    
    Args:
        query: 搜索关键词（建议使用英文）,如 'python web framework', 'machine learning'
        language: 可选的编程语言筛选（如python, javascript, java等）
        sort: 排序方式，可选值：stars（按星数）、forks（按分叉数）、updated（按更新时间）
        per_page: 返回结果数量（1-20之间）
    
    Returns:
        格式化的仓库搜索结果文本
    """
    try:
        app_logger.info(f"搜索仓库: query={query}, language={language}, sort={sort}")
        
        # 这里使用同步调用，因为FastMCP工具函数需要同步
        # 在实际应用中，您需要使用asyncio.run或其他方式处理异步调用
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            repositories = loop.run_until_complete(
                github_client.search_repositories(
                    query=query, language=language, sort=sort, per_page=per_page
                )
            )
        finally:
            loop.close()
        
        if not repositories:
            return f"未找到与查询 '{query}' 匹配的仓库"
        
        # 格式化结果
        results = [f"🔍 找到 {len(repositories)} 个仓库:\n"]
        
        for i, repo in enumerate(repositories, 1):
            stars = repo.get('stargazers_count', 0)
            forks = repo.get('forks_count', 0)
            language_info = repo.get('language', '未知')
            description = repo.get('description', '无描述')
            
            results.append(
                f"{i}. **{repo['full_name']}** ⭐ {stars:,}\n"
                f"   📝 {description}\n"
                f"   💻 {language_info} | 🍴 {forks:,} 个分叉\n"
                f"   🔗 {repo.get('html_url', '')}\n"
            )
        
        return "\n".join(results)
        
    except Exception as e:
        app_logger.error(f"搜索仓库时出错: {str(e)}")
        return f"搜索仓库时出错: {str(e)}"

@mcp.tool()
def get_repository_info(owner: str, repo: str) -> str:
    """获取仓库详细信息工具
    
    获取指定GitHub仓库的详细信息，包括统计数据、描述、许可证等。
    
    Args:
        owner: 仓库所有者/组织名称
        repo: 仓库名称
    
    Returns:
        格式化的仓库详细信息文本
    """
    try:
        app_logger.info(f"获取仓库信息: {owner}/{repo}")
        
        # 处理异步调用
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            repo_info = loop.run_until_complete(
                github_client.get_repository_info(owner, repo)
            )
        finally:
            loop.close()
        
        return f"""📦 **{repo_info['full_name']}**

📝 **描述:** {repo_info.get('description', '无描述')}
⭐ **星标:** {repo_info.get('stargazers_count', 0):,}
🍴 **分叉:** {repo_info.get('forks_count', 0):,}
👀 **关注者:** {repo_info.get('watchers_count', 0):,}
🐛 **议题:** {repo_info.get('open_issues_count', 0):,}
💻 **主要语言:** {repo_info.get('language', '未知')}
📦 **大小:** {repo_info.get('size', 0):,} KB
📅 **创建时间:** {repo_info.get('created_at', '未知')[:10]}
📅 **更新时间:** {repo_info.get('updated_at', '未知')[:10]}
🔗 **链接:** {repo_info.get('html_url', '')}

📄 **许可证:** {repo_info.get('license', {}).get('name', '无许可证') if repo_info.get('license') else '无许可证'}
🏠 **主页:** {repo_info.get('homepage', '无主页')}"""
        
    except Exception as e:
        app_logger.error(f"获取仓库信息时出错: {str(e)}")
        return f"获取仓库信息时出错: {str(e)}"

@mcp.tool()
def search_users(query: str, user_type: Optional[str] = None) -> str:
    """搜索GitHub用户工具
    
    搜索GitHub用户和组织账号。
    
    Args:
        query: 用户名或组织名搜索关键词
        user_type: 可选的账号类型筛选，可选值：user（用户）、org（组织）
    
    Returns:
        格式化的用户搜索结果文本
    """
    try:
        app_logger.info(f"搜索用户: query={query}, type={user_type}")
        
        # 处理异步调用
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            users = loop.run_until_complete(
                github_client.search_users(query=query, type=user_type)
            )
        finally:
            loop.close()
        
        if not users:
            return f"未找到与查询 '{query}' 匹配的用户"
        
        # 格式化结果
        results = [f"👥 找到 {len(users)} 个用户:\n"]
        
        for i, user in enumerate(users, 1):
            user_type_emoji = "👤" if user.get('type') == 'User' else "🏢"
            results.append(
                f"{i}. {user_type_emoji} **{user['login']}**\n"
                f"   🔗 {user.get('html_url', '')}\n"
                f"   📊 公开仓库: {user.get('public_repos', 0)}\n"
                f"   👥 关注者: {user.get('followers', 0)}\n"
            )
        
        return "\n".join(results)
        
    except Exception as e:
        app_logger.error(f"搜索用户时出错: {str(e)}")
        return f"搜索用户时出错: {str(e)}"

@mcp.tool()
def get_trending_repositories(language: Optional[str] = None, since: str = "daily") -> str:
    """获取热门趋势仓库工具
    
    获取GitHub上的热门趋势仓库。
    
    Args:
        language: 可选的编程语言筛选
        since: 时间范围，可选值：daily（每日）、weekly（每周）、monthly（每月）
    
    Returns:
        格式化的热门仓库列表文本
    """
    try:
        app_logger.info(f"获取热门仓库: language={language}, since={since}")
        
        # 构造搜索查询以获取热门仓库
        # 使用创建时间和星标数作为热门度指标
        if since == "daily":
            query = "created:>=$(date -d '1 day ago' '+%Y-%m-%d')"
        elif since == "weekly":
            query = "created:>=$(date -d '1 week ago' '+%Y-%m-%d')"
        else:  # monthly
            query = "created:>=$(date -d '1 month ago' '+%Y-%m-%d')"
        
        # 使用现有的搜索功能
        return search_repositories(
            query=f"stars:>10 {query}",
            language=language,
            sort="stars",
            per_page=10
        )
        
    except Exception as e:
        app_logger.error(f"获取热门仓库时出错: {str(e)}")
        return f"获取热门仓库时出错: {str(e)}"

def main():
    """启动FastMCP服务器的主函数"""
    app_logger.info("启动GitHub搜索FastMCP服务器...")
    
    # 验证配置
    if not config.validate():
        app_logger.error("配置验证失败，请检查环境变量设置")
        print("❌ 配置验证失败，请检查环境变量设置")
        print("📋 请确保 .env 文件包含以下必要配置：")
        print("   - GITHUB_TOKEN=your_github_token")
        return
    
    app_logger.info("配置验证通过")
    print("✅ FastMCP GitHub搜索服务器已启动")
    print("🔧 已注册的工具:")
    print("   - search_repositories: 搜索GitHub仓库")
    print("   - get_repository_info: 获取仓库详细信息")
    print("   - search_users: 搜索GitHub用户")
    print("   - get_trending_repositories: 获取热门趋势仓库")
    
    # 运行FastMCP服务器
    mcp.run()

if __name__ == "__main__":
    main() 
