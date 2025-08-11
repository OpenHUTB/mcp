#!/usr/bin/env python3
"""
GitHub Search Web Interface - 简单的GitHub搜索界面
提供基础的Web表单搜索功能，无AI智能对话
直接调用GitHub API进行搜索查询
"""

import sys
import os
import asyncio
from pathlib import Path

# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

from fastapi import FastAPI, Request, Form
from fastapi.responses import HTMLResponse
import uvicorn

from src.github_client import GitHubClient

app = FastAPI(title="GitHub Search Web - 简单搜索界面")

# 全局GitHub客户端
github_client = GitHubClient()

@app.get("/", response_class=HTMLResponse)
async def index():
    """主页面"""
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>GitHub搜索器</title>
        <style>
            * { margin: 0; padding: 0; box-sizing: border-box; }
            body { 
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                min-height: 100vh;
                padding: 20px;
            }
            .container { 
                max-width: 1200px; 
                margin: 0 auto; 
                background: white;
                border-radius: 15px;
                box-shadow: 0 20px 40px rgba(0,0,0,0.1);
                overflow: hidden;
            }
            .header {
                background: linear-gradient(45deg, #2d3748, #4a5568);
                color: white;
                padding: 30px;
                text-align: center;
            }
            .header h1 { font-size: 2.5em; margin-bottom: 10px; }
            .header p { font-size: 1.2em; opacity: 0.9; }
            .content { padding: 40px; }
            .search-form {
                background: #f8f9fa;
                border-radius: 10px;
                padding: 30px;
                margin-bottom: 30px;
            }
            .form-group { margin-bottom: 20px; }
            label { 
                display: block; 
                margin-bottom: 8px; 
                font-weight: 600;
                color: #2d3748;
            }
            input, select, button {
                width: 100%;
                padding: 12px;
                border: 2px solid #e2e8f0;
                border-radius: 8px;
                font-size: 16px;
                transition: all 0.3s;
            }
            input:focus, select:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
            }
            .button-group {
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                gap: 15px;
                margin-top: 20px;
            }
            button {
                background: linear-gradient(45deg, #667eea, #764ba2);
                color: white;
                border: none;
                cursor: pointer;
                font-weight: 600;
                text-transform: uppercase;
                letter-spacing: 1px;
            }
            button:hover {
                transform: translateY(-2px);
                box-shadow: 0 10px 20px rgba(102, 126, 234, 0.3);
            }
            .results {
                margin-top: 30px;
                padding: 30px;
                background: #f8f9fa;
                border-radius: 10px;
                border-left: 5px solid #667eea;
            }
            .repo-item {
                background: white;
                border-radius: 8px;
                padding: 20px;
                margin-bottom: 20px;
                box-shadow: 0 5px 15px rgba(0,0,0,0.1);
                transition: transform 0.3s;
            }
            .repo-item:hover {
                transform: translateY(-3px);
            }
            .repo-name {
                font-size: 1.3em;
                font-weight: bold;
                color: #2d3748;
                margin-bottom: 10px;
            }
            .repo-stats {
                display: flex;
                gap: 20px;
                margin: 10px 0;
                font-size: 0.9em;
                color: #666;
            }
            .repo-link {
                color: #667eea;
                text-decoration: none;
                font-weight: 600;
            }
            .repo-link:hover { text-decoration: underline; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🔍 GitHub搜索器</h1>
                <p>简单的GitHub仓库搜索工具 - 无AI对话功能</p>
            </div>
            
            <div class="content">
                <div class="search-form">
                    <h2>🎯 搜索选项</h2>
                    
                    <form method="post" action="/search">
                        <div class="form-group">
                            <label for="query">🔍 搜索关键词</label>
                            <input type="text" id="query" name="query" placeholder="例如: python web framework" required>
                        </div>
                        
                        <div class="form-group">
                            <label for="language">💻 编程语言 (可选)</label>
                            <select id="language" name="language">
                                <option value="">所有语言</option>
                                <option value="python">Python</option>
                                <option value="javascript">JavaScript</option>
                                <option value="java">Java</option>
                                <option value="go">Go</option>
                                <option value="rust">Rust</option>
                                <option value="typescript">TypeScript</option>
                                <option value="cpp">C++</option>
                                <option value="csharp">C#</option>
                                <option value="php">PHP</option>
                                <option value="ruby">Ruby</option>
                            </select>
                        </div>
                        
                        <div class="form-group">
                            <label for="sort">📊 排序方式</label>
                            <select id="sort" name="sort">
                                <option value="stars">⭐ 按星数</option>
                                <option value="forks">🍴 按fork数</option>
                                <option value="updated">📅 按更新时间</option>
                            </select>
                        </div>
                        
                        <div class="button-group">
                            <button type="submit" name="action" value="search">🔍 搜索仓库</button>
                        </div>
                    </form>
                    
                    <form method="post" action="/repo_info" style="margin-top: 30px;">
                        <h3>📦 获取仓库详情</h3>
                        <div class="form-group">
                            <label for="owner">👤 所有者</label>
                            <input type="text" id="owner" name="owner" placeholder="例如: microsoft">
                        </div>
                        <div class="form-group">
                            <label for="repo">📁 仓库名</label>
                            <input type="text" id="repo" name="repo" placeholder="例如: vscode">
                        </div>
                        <button type="submit">📦 获取详情</button>
                    </form>
                    
                    <form method="post" action="/search_users" style="margin-top: 30px;">
                        <h3>👥 搜索用户</h3>
                        <div class="form-group">
                            <label for="user_query">🔍 用户搜索</label>
                            <input type="text" id="user_query" name="user_query" placeholder="例如: microsoft">
                        </div>
                        <button type="submit">👥 搜索用户</button>
                    </form>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    return html_content

@app.post("/search", response_class=HTMLResponse)
async def search_repositories(query: str = Form(...), language: str = Form(""), sort: str = Form("stars")):
    """搜索仓库"""
    try:
        # 处理语言参数
        lang = language if language else None
        
        # 搜索仓库
        repositories = await github_client.search_repositories(
            query=query, language=lang, sort=sort, per_page=10
        )
        
        # 生成结果HTML
        results_html = generate_results_html(repositories, f"搜索结果: {query}")
        
    except Exception as e:
        results_html = f"""
        <div class="results">
            <h2>❌ 搜索失败</h2>
            <p>错误信息: {str(e)}</p>
            <a href="/" style="color: #667eea;">← 返回搜索</a>
        </div>
        """
    
    return f"""
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>搜索结果 - GitHub搜索器</title>
        <style>
            {get_css()}
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🔍 GitHub搜索器</h1>
                <a href="/" style="color: white; text-decoration: none;">← 返回搜索</a>
            </div>
            <div class="content">
                {results_html}
            </div>
        </div>
    </body>
    </html>
    """

@app.post("/repo_info", response_class=HTMLResponse)
async def get_repository_info(owner: str = Form(...), repo: str = Form(...)):
    """获取仓库详情"""
    try:
        repo_info = await github_client.get_repository_info(owner, repo)
        
        results_html = f"""
        <div class="results">
            <h2>📦 仓库详情</h2>
            <div class="repo-item">
                <div class="repo-name">{repo_info.get('full_name')}</div>
                <p><strong>📝 描述:</strong> {repo_info.get('description', '无描述')}</p>
                <div class="repo-stats">
                    <span>⭐ {repo_info.get('stargazers_count', 0):,} 星</span>
                    <span>🍴 {repo_info.get('forks_count', 0):,} fork</span>
                    <span>👀 {repo_info.get('watchers_count', 0):,} 关注</span>
                    <span>🐛 {repo_info.get('open_issues_count', 0):,} 问题</span>
                </div>
                <p><strong>💻 主要语言:</strong> {repo_info.get('language', '未知')}</p>
                <p><strong>📦 大小:</strong> {repo_info.get('size', 0):,} KB</p>
                <p><strong>📅 创建:</strong> {repo_info.get('created_at', '未知')[:10]}</p>
                <p><strong>📅 更新:</strong> {repo_info.get('updated_at', '未知')[:10]}</p>
                <p><strong>🔗 链接:</strong> <a href="{repo_info.get('html_url', '')}" class="repo-link" target="_blank">查看仓库</a></p>
            </div>
        </div>
        """
        
    except Exception as e:
        results_html = f"""
        <div class="results">
            <h2>❌ 获取失败</h2>
            <p>错误信息: {str(e)}</p>
        </div>
        """
    
    return f"""
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <title>仓库详情 - GitHub搜索器</title>
        <style>{get_css()}</style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>📦 仓库详情</h1>
                <a href="/" style="color: white; text-decoration: none;">← 返回搜索</a>
            </div>
            <div class="content">
                {results_html}
            </div>
        </div>
    </body>
    </html>
    """

@app.post("/search_users", response_class=HTMLResponse) 
async def search_users(user_query: str = Form(...)):
    """搜索用户"""
    try:
        users = await github_client.search_users(query=user_query)
        
        users_html = ""
        for user in users[:10]:
            user_type = "🏢 组织" if user.get('type') == 'Organization' else "👤 用户"
            users_html += f"""
            <div class="repo-item">
                <div class="repo-name">{user.get('login')} ({user_type})</div>
                <p><strong>🔗 链接:</strong> <a href="{user.get('html_url', '')}" class="repo-link" target="_blank">查看主页</a></p>
            </div>
            """
        
        results_html = f"""
        <div class="results">
            <h2>👥 用户搜索结果</h2>
            {users_html}
        </div>
        """
        
    except Exception as e:
        results_html = f"""
        <div class="results">
            <h2>❌ 搜索失败</h2>
            <p>错误信息: {str(e)}</p>
        </div>
        """
    
    return f"""
    <!DOCTYPE html>
    <html lang="zh-CN">
    <head>
        <meta charset="UTF-8">
        <title>用户搜索 - GitHub搜索器</title>
        <style>{get_css()}</style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>👥 用户搜索</h1>
                <a href="/" style="color: white; text-decoration: none;">← 返回搜索</a>
            </div>
            <div class="content">
                {results_html}
            </div>
        </div>
    </body>
    </html>
    """

def generate_results_html(repositories, title):
    """生成结果HTML"""
    if not repositories:
        return f"""
        <div class="results">
            <h2>📭 {title}</h2>
            <p>未找到相关仓库</p>
        </div>
        """
    
    results_html = f"""
    <div class="results">
        <h2>🎯 {title}</h2>
        <p>找到 {len(repositories)} 个仓库:</p>
    """
    
    for repo in repositories:
        stars = repo.get('stargazers_count', 0)
        forks = repo.get('forks_count', 0)
        language = repo.get('language', '未知')
        description = repo.get('description', '无描述')
        
        results_html += f"""
        <div class="repo-item">
            <div class="repo-name">{repo['full_name']}</div>
            <p>{description}</p>
            <div class="repo-stats">
                <span>⭐ {stars:,} 星</span>
                <span>🍴 {forks:,} fork</span>
                <span>💻 {language}</span>
            </div>
            <p><strong>🔗 链接:</strong> <a href="{repo.get('html_url', '')}" class="repo-link" target="_blank">查看仓库</a></p>
        </div>
        """
    
    results_html += "</div>"
    return results_html

def get_css():
    """获取CSS样式"""
    return """
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
            background: white;
            border-radius: 15px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(45deg, #2d3748, #4a5568);
            color: white;
            padding: 30px;
            text-align: center;
        }
        .header h1 { font-size: 2.5em; margin-bottom: 10px; }
        .content { padding: 40px; }
        .results {
            margin-top: 30px;
            padding: 30px;
            background: #f8f9fa;
            border-radius: 10px;
            border-left: 5px solid #667eea;
        }
        .repo-item {
            background: white;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 5px 15px rgba(0,0,0,0.1);
            transition: transform 0.3s;
        }
        .repo-item:hover {
            transform: translateY(-3px);
        }
        .repo-name {
            font-size: 1.3em;
            font-weight: bold;
            color: #2d3748;
            margin-bottom: 10px;
        }
        .repo-stats {
            display: flex;
            gap: 20px;
            margin: 10px 0;
            font-size: 0.9em;
            color: #666;
        }
        .repo-link {
            color: #667eea;
            text-decoration: none;
            font-weight: 600;
        }
        .repo-link:hover { text-decoration: underline; }
    """

if __name__ == "__main__":
    print("🌐 启动GitHub搜索Web界面...")
    print("📱 在浏览器中访问: http://localhost:8000")
    print("⏹️  按 Ctrl+C 停止服务器")
    
    uvicorn.run(app, host="0.0.0.0", port=8000) 