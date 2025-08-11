#!/usr/bin/env python3
"""
FastMCP GitHub Assistant - 使用FastMCP框架的智能GitHub助手
集成Deepseek AI模型，支持自然语言查询GitHub仓库
使用FastMCP装饰器方式实现MCP工具调用机制
"""

import sys
import json
import re
from pathlib import Path
from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
import uvicorn
import aiohttp
from typing import Optional

# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

from fastmcp import FastMCP
from src.github_client import GitHubClient
from src.config import config
from src.utils.logger import app_logger

# 创建FastMCP实例
mcp = FastMCP("GitHub智能助手")

# 创建GitHub客户端实例
github_client = GitHubClient()

# ============ GitHub工具函数定义 ============

async def search_github_repositories_impl(query: str, language: Optional[str] = None, 
                              sort: str = "stars", limit: int = 8) -> str:
    """搜索GitHub仓库工具
    
    用户只需要传入搜索关键词和筛选条件即可搜索GitHub仓库。
    
    Args:
        query: 搜索关键词（英文效果更好），如 'python web framework', 'machine learning'
        language: 可选的编程语言筛选，如 python, javascript, java 等
        sort: 排序方式，默认stars（按星数），也可以是forks、updated
        limit: 返回结果数量，默认8个，范围1-20
    
    Returns:
        格式化的GitHub仓库搜索结果
    """
    try:
        # 输入验证
        if not query or not query.strip():
            return "❌ 搜索关键词不能为空"
        
        query = query.strip()
        if len(query) > 256:
            return "❌ 搜索关键词过长，请限制在256字符以内"
        
        app_logger.info(f"🔍 搜索GitHub仓库: {query}")
        
        # 直接使用await处理异步GitHub API调用
        repositories = await github_client.search_repositories(
            query=query, 
            language=language, 
            sort=sort, 
            per_page=limit
        )
        
        if not repositories:
            return f"❌ 未找到与 '{query}' 匹配的仓库"
        
        # 格式化搜索结果
        result_lines = [f"🔍 找到 {len(repositories)} 个相关仓库:\n"]
        
        for i, repo in enumerate(repositories, 1):
            stars = repo.get('stargazers_count', 0)
            forks = repo.get('forks_count', 0)
            lang = repo.get('language', '未知')
            desc = repo.get('description', '无描述')
            
            result_lines.append(
                f"**{i}. {repo['full_name']}** ⭐ {stars:,}\n"
                f"   📝 {desc}\n" 
                f"   💻 {lang} | 🍴 {forks:,} forks\n"
                f"   🔗 {repo.get('html_url', '')}\n"
            )
        
        return "\n".join(result_lines)
        
    except Exception as e:
        app_logger.error(f"❌ 搜索仓库失败: {str(e)}")
        return f"❌ 搜索失败: {str(e)}"

async def get_repository_details_impl(owner: str, repo: str) -> str:
    """获取仓库详细信息工具
    
    获取指定GitHub仓库的完整详细信息。
    
    Args:
        owner: 仓库所有者用户名或组织名
        repo: 仓库名称
    
    Returns:
        仓库的详细信息
    """
    try:
        # 输入验证
        if not owner or not owner.strip():
            return "❌ 仓库所有者不能为空"
        if not repo or not repo.strip():
            return "❌ 仓库名称不能为空"
        
        owner = owner.strip()
        repo = repo.strip()
        
        # GitHub用户名和仓库名的基本限制
        if len(owner) > 39 or len(repo) > 100:
            return "❌ 用户名或仓库名过长"
        
        app_logger.info(f"📦 获取仓库详情: {owner}/{repo}")
        
        # 直接使用await处理异步GitHub API调用
        repo_info = await github_client.get_repository_info(owner, repo)
        
        # 格式化仓库信息
        return f"""📦 **{repo_info['full_name']}**

📝 **描述**: {repo_info.get('description', '无描述')}
⭐ **星标**: {repo_info.get('stargazers_count', 0):,}
🍴 **分叉**: {repo_info.get('forks_count', 0):,}
👀 **关注者**: {repo_info.get('watchers_count', 0):,}
🐛 **开放议题**: {repo_info.get('open_issues_count', 0):,}
💻 **主要语言**: {repo_info.get('language', '未知')}
📦 **大小**: {repo_info.get('size', 0):,} KB
📅 **创建时间**: {repo_info.get('created_at', '未知')[:10]}
📅 **最后更新**: {repo_info.get('updated_at', '未知')[:10]}
🔗 **仓库链接**: {repo_info.get('html_url', '')}

📄 **开源许可**: {repo_info.get('license', {}).get('name', '无许可证') if repo_info.get('license') else '无许可证'}
🏠 **项目主页**: {repo_info.get('homepage') or '无'}
🔄 **默认分支**: {repo_info.get('default_branch', 'main')}"""
        
    except Exception as e:
        app_logger.error(f"❌ 获取仓库详情失败: {str(e)}")
        return f"❌ 获取仓库 {owner}/{repo} 的详情失败: {str(e)}"

async def search_github_users_impl(query: str, user_type: Optional[str] = None) -> str:
    """搜索GitHub用户工具
    
    搜索GitHub平台上的用户和组织账号。
    
    Args:
        query: 用户名或组织名搜索关键词
        user_type: 账号类型筛选，可选值：user（个人用户）、org（组织）
    
    Returns:
        匹配的用户列表
    """
    try:
        # 输入验证
        if not query or not query.strip():
            return "❌ 用户名搜索关键词不能为空"
        
        query = query.strip()
        if len(query) > 256:
            return "❌ 搜索关键词过长，请限制在256字符以内"
        
        app_logger.info(f"👤 搜索GitHub用户: {query}")
        
        # 如果查询看起来像完整的用户名，先尝试直接获取用户信息
        if query and not ' ' in query and len(query) <= 39:  # GitHub用户名最大长度39
            try:
                app_logger.info(f"尝试直接获取用户 {query} 的详细信息")
                direct_user = await github_client.get_user_info(query)
                
                # 如果指定了用户类型且不匹配，则进行搜索
                if user_type and direct_user.get('type', '').lower() != user_type:
                    raise Exception("用户类型不匹配，进行搜索")
                
                # 格式化单个用户的详细信息
                user_emoji = "👤" if direct_user.get('type') == 'User' else "🏢"
                
                result = f"我找到了用户 **{direct_user['login']}** 的信息:\n\n"
                result += f"- **GitHub主页**: {direct_user.get('html_url', '')}\n"
                result += f"- **公开仓库数量**: {direct_user.get('public_repos', 0)}\n"
                result += f"- **关注者数量**: {direct_user.get('followers', 0)}\n"
                
                if direct_user.get('name'):
                    result += f"- **真实姓名**: {direct_user['name']}\n"
                if direct_user.get('bio'):
                    result += f"- **个人简介**: {direct_user['bio']}\n"
                if direct_user.get('location'):
                    result += f"- **位置**: {direct_user['location']}\n"
                if direct_user.get('company'):
                    result += f"- **公司**: {direct_user['company']}\n"
                
                result += f"\n目前该用户有 **{direct_user.get('public_repos', 0)}** 个公开的仓库。"
                
                return result
                
            except Exception as e:
                app_logger.info(f"直接获取用户失败，转为搜索模式: {str(e)}")
        
        # 使用搜索API查找用户
        users = await github_client.search_users(query=query, type=user_type)
        
        if not users:
            return f"❌ 未找到与 '{query}' 匹配的用户"
        
        # 格式化搜索结果
        result_lines = [f"👥 找到 {len(users)} 个相关用户:\n"]
        
        for i, user in enumerate(users, 1):
            user_emoji = "👤" if user.get('type') == 'User' else "🏢"
            
            result_lines.append(
                f"**{i}. {user_emoji} {user['login']}**\n"
                f"   🔗 {user.get('html_url', '')}\n"
                f"   📊 公开仓库: {user.get('public_repos', 0)}\n"
                f"   👥 关注者: {user.get('followers', 0)}\n"
            )
        
        return "\n".join(result_lines)
        
    except Exception as e:
        app_logger.error(f"❌ 搜索用户失败: {str(e)}")
        return f"❌ 搜索用户失败: {str(e)}"

async def get_trending_repositories_impl(language: Optional[str] = None, period: str = "daily") -> str:
    """获取GitHub热门趋势仓库工具
    
    获取当前GitHub上的热门趋势项目。
    
    Args:
        language: 可选的编程语言筛选，如 python, javascript, go 等
        period: 趋势时间范围，默认daily（每日），也可以是weekly（每周）、monthly（每月）
    
    Returns:
        热门趋势仓库列表
    """
    try:
        # 输入验证
        if language and len(language.strip()) > 50:
            return "❌ 编程语言名称过长"
        
        if period not in ["daily", "weekly", "monthly"]:
            return "❌ 时间周期只能是 daily、weekly 或 monthly"
        
        app_logger.info(f"🔥 获取热门仓库: language={language}, period={period}")
        
        # 根据时间范围构造更合理的趋势查询
        import datetime
        
        # 构造查询：获取最近一段时间内有一定活跃度的高星仓库
        if period == "daily":
            # 今日趋势：最近7天更新过且星数较高的仓库
            date_filter = (datetime.datetime.now() - datetime.timedelta(days=7)).strftime('%Y-%m-%d')
            query = f"pushed:>{date_filter} stars:>50"
            period_desc = "最近7天活跃"
        elif period == "weekly":
            # 周趋势：最近30天创建或更新的高星仓库
            date_filter = (datetime.datetime.now() - datetime.timedelta(days=30)).strftime('%Y-%m-%d')
            query = f"created:>{date_filter} stars:>10"
            period_desc = "最近30天"
        else:  # monthly
            # 月趋势：最近90天创建的热门仓库
            date_filter = (datetime.datetime.now() - datetime.timedelta(days=90)).strftime('%Y-%m-%d')
            query = f"created:>{date_filter} stars:>5"
            period_desc = "最近90天"
        
        app_logger.info(f"趋势查询: {query}")
        
        # 调用搜索仓库功能
        repositories = await github_client.search_repositories(
            query=query,
            language=language,
            sort="stars",
            order="desc",
            per_page=10
        )
        
        if not repositories:
            return f"❌ 未找到 {language or '所有语言'} 的{period_desc}热门仓库"
        
        # 格式化趋势仓库结果
        result_lines = [f"🔥 找到 {len(repositories)} 个{language or '全部语言'}{period_desc}热门仓库:\n"]
        
        for i, repo in enumerate(repositories, 1):
            stars = repo.get('stargazers_count', 0)
            forks = repo.get('forks_count', 0)
            lang = repo.get('language', '未知')
            desc = repo.get('description', '无描述')
            created = repo.get('created_at', '')[:10] if repo.get('created_at') else '未知'
            updated = repo.get('updated_at', '')[:10] if repo.get('updated_at') else '未知'
            
            result_lines.append(
                f"**{i}. {repo['full_name']}** ⭐ {stars:,}\n"
                f"   📝 {desc}\n"
                f"   💻 {lang} | 🍴 {forks:,} forks\n"
                f"   📅 创建: {created} | 更新: {updated}\n"
                f"   🔗 {repo.get('html_url', '')}\n"
            )
        
        return "\n".join(result_lines)
        
    except Exception as e:
        app_logger.error(f"❌ 获取热门仓库失败: {str(e)}")
        return f"❌ 获取热门仓库失败: {str(e)}"

# ============ FastMCP 工具装饰器版本 ============

@mcp.tool()
async def search_github_repositories(query: str, language: Optional[str] = None, 
                              sort: str = "stars", limit: int = 8) -> str:
    """搜索GitHub仓库工具 - FastMCP版本"""
    return await search_github_repositories_impl(query, language, sort, limit)

@mcp.tool()
async def get_repository_details(owner: str, repo: str) -> str:
    """获取仓库详细信息工具 - FastMCP版本"""
    return await get_repository_details_impl(owner, repo)

@mcp.tool()
async def search_github_users(query: str, user_type: Optional[str] = None) -> str:
    """搜索GitHub用户工具 - FastMCP版本"""
    return await search_github_users_impl(query, user_type)

@mcp.tool()
async def get_trending_repositories(language: Optional[str] = None, period: str = "daily") -> str:
    """获取GitHub热门趋势仓库工具 - FastMCP版本"""
    return await get_trending_repositories_impl(language, period)

# ============ AI助手类（集成Deepseek AI） ============

class FastMCPGitHubAssistant:
    """FastMCP GitHub AI助手 - 集成Deepseek AI与FastMCP工具"""
    
    def __init__(self):
        # 将FastMCP工具转换为标准MCP工具格式供AI使用
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "search_github_repositories",
                    "description": "搜索GitHub仓库工具。根据关键词、编程语言等条件搜索GitHub仓库，返回热门匹配结果。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "搜索关键词（建议使用英文），如 'python web framework', 'machine learning', 'face detection'"
                            },
                            "language": {
                                "type": "string",
                                "description": "可选的编程语言筛选，如 python, javascript, java 等"
                            },
                            "sort": {
                                "type": "string",
                                "enum": ["stars", "forks", "updated"],
                                "description": "排序方式，默认stars（按星数），也可以是forks、updated"
                            },
                            "limit": {
                                "type": "integer",
                                "description": "返回结果数量，默认8个，范围1-20",
                                "minimum": 1,
                                "maximum": 20
                            }
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function", 
                "function": {
                    "name": "get_repository_details",
                    "description": "获取指定GitHub仓库的完整详细信息，包括统计数据、描述、许可证等。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "owner": {
                                "type": "string",
                                "description": "仓库所有者用户名或组织名"
                            },
                            "repo": {
                                "type": "string", 
                                "description": "仓库名称"
                            }
                        },
                        "required": ["owner", "repo"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "search_github_users",
                    "description": "搜索GitHub平台上的用户和组织账号。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "用户名或组织名搜索关键词"
                            },
                            "user_type": {
                                "type": "string",
                                "enum": ["user", "org"],
                                "description": "账号类型筛选，可选值：user（个人用户）、org（组织）"
                            }
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_trending_repositories", 
                    "description": "获取当前GitHub上的热门趋势项目。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "language": {
                                "type": "string",
                                "description": "可选的编程语言筛选，如 python, javascript, go 等"
                            },
                            "period": {
                                "type": "string",
                                "enum": ["daily", "weekly", "monthly"],
                                "description": "趋势时间范围，默认daily（每日），也可以是weekly（每周）、monthly（每月）"
                            }
                        },
                        "required": []
                    }
                }
            }
        ]
    
    def process_markdown(self, text):
        """在Python端处理Markdown格式"""
        result = text
        
        # 处理标题
        result = re.sub(r'^### (.+)$', r'<h3><strong>\1</strong></h3>', result, flags=re.MULTILINE)
        result = re.sub(r'^## (.+)$', r'<h2><strong>\1</strong></h2>', result, flags=re.MULTILINE)
        result = re.sub(r'^# (.+)$', r'<h1><strong>\1</strong></h1>', result, flags=re.MULTILINE)
        
        # 处理粗体链接 **[text](url)**
        result = re.sub(r'\*\*\[([^\]]+)\]\(([^)]+)\)\*\*', r'<strong><a href="\2" target="_blank">\1</a></strong>', result)
        
        # 处理普通链接 [text](url)
        result = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2" target="_blank">\1</a>', result)
        
        # 处理粗体文本 **text**
        result = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', result)
        
        # 处理换行
        result = result.replace('\n', '<br>')
        
        return result
    
    async def call_deepseek_with_tools(self, messages):
        """调用Deepseek API，包含FastMCP工具定义"""
        headers = config.get_deepseek_headers()

        data = {
            "model": "deepseek-chat",
            "messages": messages,
            "tools": self.tools,
            "tool_choice": "auto",
            "max_tokens": 2000,
            "temperature": 0.7
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(config.DEEPSEEK_API_URL, headers=headers, json=data) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    error_text = await response.text()
                    raise Exception(f"Deepseek API调用失败: {response.status} - {error_text}")

    async def execute_fastmcp_tool_call(self, tool_call):
        """执行FastMCP工具调用 - 桥接到FastMCP装饰器函数"""
        function_name = tool_call["function"]["name"]
        arguments = json.loads(tool_call["function"]["arguments"])

        app_logger.info(f"🔧 执行FastMCP工具: {function_name}")
        app_logger.info(f"📝 参数: {arguments}")

        try:
            # 调用实际的工具实现函数（避免FastMCP装饰器问题）
            if function_name == "search_github_repositories":
                result = await search_github_repositories_impl(
                    query=arguments["query"],
                    language=arguments.get("language"),
                    sort=arguments.get("sort", "stars"),
                    limit=arguments.get("limit", 8)
                )
                return {
                    "success": True,
                    "data": result
                }
                
            elif function_name == "get_repository_details":
                result = await get_repository_details_impl(
                    owner=arguments["owner"],
                    repo=arguments["repo"]
                )
                return {
                    "success": True,
                    "data": result
                }
                
            elif function_name == "search_github_users":
                result = await search_github_users_impl(
                    query=arguments["query"],
                    user_type=arguments.get("user_type")
                )
                return {
                    "success": True,
                    "data": result
                }
                
            elif function_name == "get_trending_repositories":
                result = await get_trending_repositories_impl(
                    language=arguments.get("language"),
                    period=arguments.get("period", "daily")
                )
                return {
                    "success": True,
                    "data": result
                }
            else:
                return {
                    "success": False,
                    "error": f"未知的工具: {function_name}"
                }

        except Exception as e:
            app_logger.error(f"❌ FastMCP工具执行失败: {str(e)}")
            return {
                "success": False,
                "error": str(e)
            }

    async def chat(self, user_message):
        """处理聊天请求 - 使用FastMCP工具的AI对话"""
        # 初始消息
        messages = [
            {
                "role": "system",
                "content": """你是一个GitHub搜索助手，基于FastMCP框架提供服务。你有以下工具可以使用：

1. search_github_repositories - 搜索GitHub仓库
2. get_repository_details - 获取仓库详细信息（需要用户名和仓库名）
3. search_github_users - 搜索GitHub用户和组织
4. get_trending_repositories - 获取热门趋势仓库

处理用户查询的策略：
- 如果用户询问特定用户的特定项目，优先使用get_repository_details工具
- 如果用户询问某类项目的推荐，使用search_github_repositories
- 如果用户询问某个用户的信息，使用search_github_users
- 如果用户询问热门或趋势项目，使用get_trending_repositories

重要提示：
- 搜索时使用英文关键词效果更好
- 可以根据用户需求调用多个工具获得更全面的结果
- 必须先获取数据，再基于实际数据回答用户问题
- 如果没有找到结果，要明确告知用户

本助手基于FastMCP框架构建，提供高效、类型安全的工具调用体验。"""
            },
            {"role": "user", "content": user_message}
        ]

        # 第一次API调用
        app_logger.info(f"💬 用户消息: {user_message}")
        response = await self.call_deepseek_with_tools(messages)
        assistant_message = response["choices"][0]["message"]

        # 检查是否有工具调用
        tool_calls = assistant_message.get("tool_calls", [])
        messages.append(assistant_message)

        # 执行FastMCP工具调用
        if tool_calls:
            app_logger.info(f"🔧 检测到 {len(tool_calls)} 个FastMCP工具调用")
            
            for tool_call in tool_calls:
                app_logger.info(f"🔨 执行FastMCP工具: {tool_call['function']['name']}")
                tool_result = await self.execute_fastmcp_tool_call(tool_call)
                app_logger.info(f"✅ FastMCP工具执行完成，结果长度: {len(str(tool_result))}")
                
                # 添加工具结果到消息历史
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call["id"],
                    "content": json.dumps(tool_result, ensure_ascii=False)
                })

            # 再次调用API获取最终回答
            app_logger.info("🤖 正在生成最终回答...")
            try:
                final_response = await self.call_deepseek_with_tools(messages)
                final_message = final_response["choices"][0]["message"]["content"]
                app_logger.info(f"✅ 最终回答生成成功，长度: {len(final_message)}")
             
                if not final_message or final_message.strip() == "":
                    app_logger.info("❌ 警告：最终回答为空")
                    final_message = "抱歉，我无法生成回答。请稍后重试。"

                return {
                    "message": self.process_markdown(final_message),
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
            except Exception as e:
                app_logger.error(f"❌ 生成最终回答时出错: {str(e)}")
                return {
                    "message": f"FastMCP工具调用成功，但生成最终回答时出错: {str(e)}",
                    "tool_calls": tool_calls,
                    "conversation": messages
                }
        else:
            return {
                "message": self.process_markdown(assistant_message["content"]),
                "tool_calls": None,
                "conversation": messages
            }

# ============ FastAPI Web界面（AI对话版） ============

app = FastAPI(title="FastMCP GitHub Assistant")

def get_web_interface():
    """生成AI对话Web界面HTML"""
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>FastMCP GitHub Assistant - 智能GitHub助手</title>
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
        <style>
            * { 
                margin: 0; 
                padding: 0; 
                box-sizing: border-box; 
            }
            
            body {
                font-family: 'Segoe UI', 'Microsoft YaHei', Tahoma, Geneva, Verdana, sans-serif;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                min-height: 100vh;
                line-height: 1.6;
            }
            
            .container {
                max-width: 900px;
                margin: 0 auto;
                padding: 20px;
                min-height: 100vh;
                display: flex;
                flex-direction: column;
            }
            
            .header {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                padding: 12px 20px;
                border-radius: 15px;
                text-align: center;
                margin-bottom: 15px;
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }
            
            .header h1 {
                color: #2d3748;
                font-size: 1.5em;
                margin: 0;
                font-weight: 700;
                background: linear-gradient(135deg, #667eea, #764ba2);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
                background-clip: text;
            }
            
            .chat-container {
                background: rgba(255, 255, 255, 0.95);
                backdrop-filter: blur(10px);
                border-radius: 20px;
                padding: 20px;
                flex: 1;
                display: flex;
                flex-direction: column;
                box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }
            
            .messages {
                flex: 1;
                overflow-y: auto;
                overflow-x: hidden;
                padding: 15px;
                margin-bottom: 15px;
                background: rgba(248, 250, 252, 0.5);
                border-radius: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                height: calc(100vh - 280px);
                min-height: 400px;
                max-height: calc(100vh - 280px);
                scroll-behavior: smooth;
            }
            
            .message {
                margin-bottom: 15px;
                padding: 15px 20px;
                border-radius: 15px;
                max-width: 85%;
                word-wrap: break-word;
                position: relative;
                animation: messageSlide 0.3s ease-out;
            }
            
            @keyframes messageSlide {
                from {
                    opacity: 0;
                    transform: translateY(10px);
                }
                to {
                    opacity: 1;
                    transform: translateY(0);
                }
            }
            
            .user-message {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                margin-left: auto;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                border-bottom-right-radius: 5px;
            }
            
            .assistant-message {
                background: linear-gradient(135deg, #f8fafc, #e2e8f0);
                color: #2d3748;
                margin-right: auto;
                border-left: 4px solid #667eea;
                box-shadow: 0 4px 15px rgba(0, 0, 0, 0.05);
                border-bottom-left-radius: 5px;
            }
            
            .tools-used {
                background: rgba(102, 126, 234, 0.05);
                margin-top: 10px;
                border-radius: 10px;
                font-size: 0.9em;
                border: 1px solid rgba(102, 126, 234, 0.2);
                overflow: hidden;
            }
            
            .tools-header {
                background: rgba(102, 126, 234, 0.1);
                padding: 10px 12px;
                cursor: pointer;
                display: flex;
                align-items: center;
                justify-content: space-between;
                font-weight: 600;
                color: #667eea;
                transition: all 0.3s ease;
            }
            
            .tools-header:hover {
                background: rgba(102, 126, 234, 0.15);
            }
            
            .tools-toggle {
                font-size: 0.9em;
                transition: all 0.3s ease;
                font-weight: bold;
            }
            
            .tools-content {
                padding: 12px;
                display: none;
                border-top: 1px solid rgba(102, 126, 234, 0.1);
            }
            
            .tools-content.show {
                display: block;
            }
            
            .input-form {
                display: flex;
                gap: 12px;
                align-items: flex-end;
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.95), rgba(248, 250, 252, 0.9));
                padding: 15px;
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
                box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
                backdrop-filter: blur(10px);
            }
            
            .message-input {
                flex: 1;
                padding: 12px 16px;
                border: 2px solid transparent;
                border-radius: 12px;
                background: white;
                font-size: 0.95em;
                resize: none;
                min-height: 44px;
                max-height: 120px;
                transition: all 0.3s ease;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
                font-family: inherit;
                line-height: 1.4;
            }
            
            .message-input:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.15), 0 4px 15px rgba(0, 0, 0, 0.15);
                transform: translateY(-1px);
            }
            
            .message-input::placeholder {
                color: #9ca3af;
                font-style: italic;
            }
            
            .send-button {
                width: 44px;
                height: 44px;
                background: linear-gradient(135deg, #667eea, #764ba2);
                border: none;
                border-radius: 50%;
                cursor: pointer;
                transition: all 0.3s ease;
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
                display: flex;
                align-items: center;
                justify-content: center;
                flex-shrink: 0;
                position: relative;
            }
            
            .send-button i {
                color: white;
                font-size: 16px;
            }
            
            .send-button:hover:not(:disabled) {
                transform: translateY(-2px);
                box-shadow: 0 6px 25px rgba(102, 126, 234, 0.4);
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }
            
            .send-button:active:not(:disabled) {
                transform: translateY(0px);
                box-shadow: 0 2px 10px rgba(102, 126, 234, 0.3);
            }
            
            .send-button:disabled {
                opacity: 0.5;
                cursor: not-allowed;
                transform: none;
                box-shadow: 0 2px 8px rgba(102, 126, 234, 0.2);
                background: linear-gradient(135deg, #9ca3af, #6b7280);
            }
            
            .loading {
                display: none;
                text-align: center;
                padding: 25px;
                margin: 15px 0;
                background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
                border-radius: 15px;
                border: 1px solid rgba(102, 126, 234, 0.2);
            }
            
            .loading.show { 
                display: block; 
            }
            
            .loading-content {
                display: flex;
                flex-direction: column;
                align-items: center;
                gap: 15px;
            }
            
            .loading-text {
                color: #667eea;
                font-weight: 600;
                font-size: 1.2em;
                display: flex;
                align-items: center;
                gap: 12px;
            }
            
            .loading-spinner {
                width: 24px;
                height: 24px;
                border: 3px solid rgba(102, 126, 234, 0.2);
                border-top: 3px solid #667eea;
                border-radius: 50%;
                animation: spin 1s linear infinite;
            }
            
            @keyframes spin {
                from { transform: rotate(0deg); }
                to { transform: rotate(360deg); }
            }
            
            .example-questions {
                background: linear-gradient(135deg, rgba(248, 250, 252, 0.8), rgba(241, 245, 249, 0.8));
                border-radius: 15px;
                padding: 20px;
                margin-bottom: 15px;
                border: 1px solid rgba(226, 232, 240, 0.5);
                backdrop-filter: blur(5px);
            }
            
            .welcome-message {
                color: #4a5568;
                margin-bottom: 15px;
                font-size: 1em;
                line-height: 1.5;
                text-align: center;
                padding: 15px;
                background: rgba(255, 255, 255, 0.6);
                border-radius: 12px;
                border-left: 4px solid #667eea;
            }
            
            .example-questions h3 {
                color: #2d3748;
                margin-bottom: 15px;
                font-size: 1em;
                text-align: center;
                font-weight: 600;
            }
            
            .examples-grid {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 12px;
            }
            
            .example-item {
                background: linear-gradient(135deg, rgba(255, 255, 255, 0.9), rgba(248, 250, 252, 0.9));
                border-radius: 10px;
                padding: 12px 16px;
                cursor: pointer;
                transition: all 0.3s ease;
                border-left: 3px solid #667eea;
                font-size: 0.9em;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
                border: 1px solid rgba(226, 232, 240, 0.3);
                text-align: center;
            }
            
            .example-item:hover {
                background: linear-gradient(135deg, #667eea, #764ba2);
                color: white;
                transform: translateY(-2px) scale(1.02);
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
            }
            
            .assistant-message h1 {
                font-size: 1.4em;
                color: #2d3748;
                margin: 15px 0 10px 0;
                font-weight: 700;
            }
            
            .assistant-message h2 {
                font-size: 1.2em;
                color: #2d3748;
                margin: 12px 0 8px 0;
                font-weight: 600;
            }
            
            .assistant-message h3 {
                font-size: 1.1em;
                color: #2d3748;
                margin: 10px 0 6px 0;
                font-weight: 600;
            }
            
            /* 响应式设计 */
            @media (max-width: 768px) {
                .container {
                    padding: 10px;
                }
                
                .header h1 {
                    font-size: 1.5em;
                }
                
                .message {
                    max-width: 95%;
                    padding: 12px 15px;
                }
                
                .examples-grid {
                    grid-template-columns: 1fr;
                    gap: 8px;
                }
                
                .input-form {
                    flex-direction: column;
                    gap: 12px;
                    padding: 12px;
                }
                
                .message-input {
                    min-height: 40px;
                }
                
                .send-button {
                    width: 100%;
                    height: 44px;
                }
                
                .messages {
                    height: calc(100vh - 320px);
                }
            }
            
            /* 滚动条美化 */
            .messages::-webkit-scrollbar {
                width: 6px;
            }
            
            .messages::-webkit-scrollbar-track {
                background: rgba(226, 232, 240, 0.3);
                border-radius: 3px;
            }
            
            .messages::-webkit-scrollbar-thumb {
                background: linear-gradient(135deg, #667eea, #764ba2);
                border-radius: 3px;
            }
            
            .messages::-webkit-scrollbar-thumb:hover {
                background: linear-gradient(135deg, #5a67d8, #6b46c1);
            }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🚀 FastMCP GitHub Assistant</h1>
            </div>
            
            <div class="chat-container">
                <div class="messages" id="messages">
                    <div class="example-questions">
                        <div class="welcome-message">
                            👋 欢迎使用基于FastMCP框架的GitHub智能助手！我可以帮你搜索仓库、查看项目详情、分析用户信息。
                            <br><br>
                            🔧 <strong>技术特色</strong>：本助手使用FastMCP装饰器实现工具定义，提供类型安全、自动化的MCP体验！
                        </div>
                        <h3>💡 试试这些问题：</h3>
                        <div class="examples-grid">
                            <div class="example-item" onclick="askExample('查看最近热门的项目')">
                                🔥 查看近期热门项目
                            </div>
                            <div class="example-item" onclick="askExample('找一些机器学习库')">
                                🤖 查找机器学习库
                            </div>
                            <div class="example-item" onclick="askExample('查看microsoft/vscode仓库详情')">
                                📦 查看仓库详情
                            </div>
                            <div class="example-item" onclick="askExample('推荐一些优秀的Go语言框架')">
                                🚀 Go语言框架推荐
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="loading" id="loading">
                    <div class="loading-content">
                        <div class="loading-text">
                            <div class="loading-spinner"></div>
                            <span>FastMCP工具调用中...</span>
                        </div>
                    </div>
                </div>
                
                <form class="input-form" onsubmit="return submitForm(event)">
                    <textarea 
                        id="messageInput" 
                        class="message-input" 
                        placeholder="问我任何GitHub相关问题，我会使用FastMCP工具来帮你搜索..."
                        rows="2"
                        onkeydown="handleKeyPress(event)"
                    ></textarea>
                    <button type="submit" class="send-button" id="sendButton">
                        <i class="fas fa-paper-plane"></i>
                    </button>
                </form>
            </div>
        </div>

<script>
function askExample(text) {
    document.getElementById('messageInput').value = text;
    submitMessage();
}

function handleKeyPress(event) {
    if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        submitMessage();
    }
}

function submitForm(event) {
    event.preventDefault();
    submitMessage();
    return false;
}

async function submitMessage() {
    const input = document.getElementById('messageInput');
    const message = input.value.trim();
    if (!message) return;

    addMessage(message, 'user');
    input.value = '';
    showLoading(true);

    try {
        const response = await fetch('/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'message=' + encodeURIComponent(message)
        });

        if (response.ok) {
            const result = await response.json();
            addMessage(result.message, 'assistant', result.tool_calls);
        } else {
            addMessage('抱歉，发生了错误，请稍后重试。', 'assistant');
        }
    } catch (error) {
        console.error('Error:', error);
        addMessage('网络连接错误，请检查网络后重试。', 'assistant');
    } finally {
        showLoading(false);
    }
}

function addMessage(content, sender, toolCalls) {
    const messages = document.getElementById('messages');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}-message`;
    
    let html = `<div>${content}</div>`;
    
    if (toolCalls && toolCalls.length > 0) {
        const toolsId = 'tools-' + Date.now();
        html += `
            <div class="tools-used">
                <div class="tools-header" onclick="toggleTools('${toolsId}')">
                    <span>🔧 使用的FastMCP工具 (${toolCalls.length}个)</span>
                    <span class="tools-toggle" id="toggle-${toolsId}">▼</span>
                </div>
                <div class="tools-content" id="${toolsId}">`;
        
        for (let i = 0; i < toolCalls.length; i++) {
            const tool = toolCalls[i];
            const args = JSON.parse(tool.function.arguments);
            let argStr = '';
            for (const k in args) {
                if (argStr) argStr += ', ';
                argStr += `${k}: "${args[k]}"`;
            }
            html += `<div>• <strong>@mcp.tool() ${tool.function.name}</strong>(${argStr})</div>`;
        }
        
        html += `
                </div>
            </div>`;
    }
    
    messageDiv.innerHTML = html;
    messages.appendChild(messageDiv);
    messages.scrollTop = messages.scrollHeight;
}

function toggleTools(toolsId) {
    const content = document.getElementById(toolsId);
    const toggle = document.getElementById('toggle-' + toolsId);
    
    if (content.classList.contains('show')) {
        content.classList.remove('show');
        toggle.classList.remove('expanded');
        toggle.textContent = '▼';
    } else {
        content.classList.add('show');
        toggle.classList.add('expanded');
        toggle.textContent = '▲';
    }
}

function showLoading(show) {
    const loading = document.getElementById('loading');
    const sendButton = document.getElementById('sendButton');
    
    if (show) {
        loading.classList.add('show');
        sendButton.disabled = true;
    } else {
        loading.classList.remove('show');
        sendButton.disabled = false;
    }
}
</script>
    </body>
    </html>
    """
    return html_content

@app.get("/", response_class=HTMLResponse)
async def index():
    """主页面 - AI对话界面"""
    return get_web_interface()

@app.post("/chat")
async def chat(message: str = Form(...)):
    """处理聊天请求 - 使用FastMCP工具的AI对话"""
    try:
        result = await assistant.chat(message)
        return {
            "success": True,
            "message": result["message"],
            "tool_calls": result["tool_calls"]
        }
    except Exception as e:
        app_logger.error(f"❌ FastMCP聊天处理失败: {str(e)}")
        return {
            "success": False,
            "message": f"抱歉，处理您的请求时出现错误: {str(e)}",
            "tool_calls": None
        }

# 创建全局AI助手实例
assistant = FastMCPGitHubAssistant()

def main():
    """主函数 - 可以选择启动Web界面或MCP服务器"""
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "mcp":
        # 启动MCP服务器模式
        print("[MCP] 启动FastMCP GitHub助手MCP服务器...")
        
        # 验证配置
        if not config.validate():
            print("[ERROR] 配置验证失败")
            print("[INFO] 请确保环境变量包含:")
            print("   - GITHUB_TOKEN=your_github_token")
            return
            
        print("[OK] 配置验证通过")
        print("[TOOLS] 已注册MCP工具:")
        print("   - search_github_repositories")
        print("   - get_repository_details") 
        print("   - search_github_users")
        print("   - get_trending_repositories")
        print("[READY] 等待AI连接...")
        
        # 启动FastMCP服务器
        mcp.run()
    else:
        # 默认启动Web AI对话界面
        print("[WEB] 启动FastMCP GitHub助手AI对话界面...")
        print("[AI] 集成Deepseek AI + FastMCP工具")
        
        # 验证配置
        if not config.validate():
            print("[ERROR] 配置验证失败，请检查环境变量设置")
            print("[INFO] 请确保 .env 文件包含以下必要配置：")
            print("   - GITHUB_TOKEN=your_github_token")
            print("   - DEEPSEEK_API_KEY=your_deepseek_api_key")
            return
        
        print("[OK] 配置验证通过")
        print("[TOOLS] FastMCP工具已注册:")
        print("   - @mcp.tool() search_github_repositories")
        print("   - @mcp.tool() get_repository_details")
        print("   - @mcp.tool() search_github_users")  
        print("   - @mcp.tool() get_trending_repositories")
        print("[URL] 访问地址: http://localhost:3000")
        print("[INFO] 基于FastMCP框架 + Deepseek AI智能对话")
        print()
        
        uvicorn.run(app, host="localhost", port=3000)

if __name__ == "__main__":
    main() 
