# Contributing to AI GitHub Assistant

感谢您对 AI GitHub Assistant 项目的兴趣！我们欢迎所有形式的贡献，包括代码、文档、测试、设计等。

## 🤝 贡献方式

### 报告问题
- 使用 [GitHub Issues](https://github.com/wink-wink-wink555/ai-github-assistant/issues) 报告 bug
- 提交功能请求和建议
- 改进文档和示例

### 提交代码
- 修复 bug
- 添加新功能
- 改进性能
- 优化代码结构

### 改进文档
- 更新 README
- 完善 API 文档
- 添加使用示例
- 翻译文档

## 📋 开发指南

### 开发环境设置

1. **Fork 项目**
   ```bash
   # Fork 项目到您的 GitHub 账户
   # 然后克隆您fork的版本到本地
   git clone https://github.com/YOUR_USERNAME/ai-github-assistant.git
   cd ai-github-assistant
   ```

2. **设置开发环境**
   ```bash
   # 创建虚拟环境
   python -m venv venv
   source venv/bin/activate  # Linux/macOS
   # 或
   venv\Scripts\activate     # Windows
   
   # 安装依赖
   pip install -r requirements.txt
   
   # 安装开发依赖
   pip install black isort flake8 pytest
   ```

3. **配置环境变量**
   ```bash
   cp config.env.example .env
   # 编辑 .env 文件，添加必要的 API keys
   ```

### 代码标准

#### 代码风格
- 遵循 [PEP 8](https://www.python.org/dev/peps/pep-0008/) 代码风格
- 使用 4 个空格缩进
- 行长度不超过 88 字符
- 使用有意义的变量和函数名

#### 代码格式化
```bash
# 格式化代码
black .
isort .

# 检查代码质量
flake8 .
```

#### 类型注解
- 为所有函数添加类型注解
- 使用 `typing` 模块的类型提示
- 示例：
  ```python
  from typing import Dict, List, Optional
  
  def search_repositories(query: str, limit: int = 10) -> List[Dict]:
      """Search GitHub repositories."""
      pass
  ```

### 提交规范

#### 提交信息格式
```
type(scope): description

[optional body]

[optional footer]
```

#### 提交类型
- `feat`: 新功能
- `fix`: 错误修复
- `docs`: 文档更新
- `style`: 代码格式（不影响代码运行的变更）
- `refactor`: 重构（既不是新增功能，也不是修复bug）
- `test`: 测试相关
- `chore`: 构建过程或辅助工具的变动

#### 示例
```
feat(search): add repository language filter

Added support for filtering repositories by programming language
in the search functionality. This allows users to find repositories
specific to their preferred programming language.

Closes #123
```

### 测试

#### 运行测试
```bash
# 运行所有测试
pytest

# 运行特定测试
pytest tests/test_github_client.py

# 运行测试并生成覆盖率报告
pytest --cov=src tests/
```

#### 编写测试
- 为新功能编写测试
- 确保测试覆盖率 > 80%
- 使用有意义的测试名称
- 测试文件命名格式：`test_*.py`

## 🔄 开发工作流

### 1. 创建 Issue
- 在开始开发前，先创建或选择一个 Issue
- 描述问题或功能需求
- 获得维护者的确认

### 2. 创建分支
```bash
# 创建并切换到新分支
git checkout -b feature/your-feature-name

# 或者修复bug
git checkout -b fix/issue-number
```

### 3. 开发和测试
- 编写代码
- 添加测试
- 确保所有测试通过
- 格式化代码

### 4. 提交更改
```bash
# 添加文件
git add .

# 提交更改
git commit -m "feat(scope): your change description"

# 推送到远程仓库
git push origin feature/your-feature-name
```

### 5. 创建 Pull Request
- 在 GitHub 上创建 Pull Request
- 填写 PR 模板
- 等待代码审查
- 根据反馈进行修改

## 📝 Pull Request 指南

### PR 标题
- 使用清晰简洁的标题
- 遵循提交信息规范
- 示例：`feat(api): add user search functionality`

### PR 描述
- 描述更改的内容和原因
- 列出相关的 Issue
- 包含测试说明
- 添加截图（如果适用）

### PR 模板
```markdown
## 描述
简要描述此 PR 的更改内容

## 相关 Issue
- Closes #123
- Fixes #456

## 更改类型
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## 测试
- [ ] 添加了测试
- [ ] 所有测试通过
- [ ] 手动测试完成

## 截图
（如果适用）

## 检查清单
- [ ] 代码遵循项目风格指南
- [ ] 进行了自我代码审查
- [ ] 代码有适当的注释
- [ ] 相关文档已更新
```

## 🎯 贡献领域

### 优先级高的贡献
1. **Bug 修复**: 修复已知问题
2. **性能优化**: 提高应用性能
3. **文档改进**: 完善文档和示例
4. **测试覆盖**: 增加测试用例

### 功能建议
- 批量查询功能
- 更多的 GitHub API 集成
- 数据可视化
- 缓存机制优化
- 国际化支持

### 技术改进
- 异步处理优化
- 错误处理改进
- 日志系统增强
- 配置管理优化

## 📞 获取帮助

如果您在贡献过程中遇到问题，请通过以下方式获取帮助：

- 💬 [GitHub Discussions](https://github.com/wink-wink-wink555/ai-github-assistant/discussions)
- 🐛 [GitHub Issues](https://github.com/wink-wink-wink555/ai-github-assistant/issues)
- 📧 Email: yfsun.jeff@gmail.com

## 🏆 贡献者

感谢所有贡献者！您的贡献将会被记录在项目的贡献者列表中。

## 📜 行为准则

我们致力于为每个人提供一个友好、安全和受欢迎的环境。请遵循以下准则：

- 使用友好和包容的语言
- 尊重不同的观点和经验
- 优雅地接受建设性的批评
- 关注对社区最好的事情
- 对其他社区成员表现出同情心

## 🎉 感谢

感谢您考虑为 AI GitHub Assistant 做出贡献！每一个贡献都很重要，无论多小。

---

*Happy Contributing!* 🚀 