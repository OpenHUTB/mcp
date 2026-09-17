# OpenHUTB/mcp Linux 环境更新说明

> 适用场景：已在 Linux 部署 OpenHUTB/mcp 旧版本，需同步上游仓库最新代码。
> 目标版本：`https://github.com/OpenHUTB/mcp` 的 `main` 分支（含第 1~4 周全部场景工具与网页快捷按钮面板）。
> 更新日期参考：2026-09-17。

---

## 一、更新前必读

| 项目 | 结论 |
| --- | --- |
| CARLA 模拟器 | **无需重新下载**，保留现有 Linux 版即可（版本匹配要求见第三节） |
| 现有配置 | **可以保留**，`.env` 不被 git 跟踪，更新不会覆盖 |
| Python 环境 | 在项目虚拟环境内升级依赖即可，**不要动系统 Python** |
| 启动参数 | 不变，与旧版完全一致 |
| 已知问题 | 上游 main 曾存在 6 个工具重复注册导致 DeepSeek 报错，修复 PR 合并后拉取即可（见第五节） |

---

## 二、更新流程

### 第 1 步：备份现有配置

```bash
cp /你的项目路径/llm/.env ~/mcp_env_backup.env
```

`.env` 包含 GITHUB_TOKEN、DEEPSEEK_API_KEY 等密钥，更新虽然不会覆盖它，但操作前备份是好习惯。

### 第 2 步：同步代码

**情况 A：当初是直接 clone 上游仓库**

```bash
cd /你的项目路径
git pull origin main
```

**情况 B：当初是 fork 后 clone 自己的仓库**

```bash
cd /你的项目路径
git remote add upstream https://github.com/OpenHUTB/mcp.git   # 已添加过则跳过
git fetch upstream
git merge upstream/main
```

如有本地改动导致冲突，先 `git stash` 再合并，完成后 `git stash pop`。

### 第 3 步：更新 Python 依赖

```bash
cd /你的项目路径/llm
source /你的虚拟环境路径/bin/activate      # 例如: source ../venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` 钉死的依赖包括：`fastmcp>=0.9.0`、`fastapi>=0.104.1`、`uvicorn>=0.24.0`、`carla==0.9.16` 等。

### 第 4 步：核对 CARLA 版本

**CARLA 的 Python 客户端与服务端大版本必须一致**，判断方式：

```bash
# 查看服务端版本（进入 CARLA 解压目录执行，或看目录名）
ls /你的CARLA目录/            # 目录名如 CARLA_0.9.16 即服务端 0.9.16

# 查看客户端版本
python -c "import carla; print(carla.__version__ if hasattr(carla,'__version__') else 'unknown')"
```

| 你的服务端版本 | 处理方式 |
| --- | --- |
| **0.9.16** | 什么都不用动，第 3 步 pip 已装好匹配的客户端 ✅ |
| 0.9.15 及更早 | **推荐**：升级服务端到 0.9.16（见下方说明） |
| 暂时不想升服务端 | `pip install carla==0.9.15`（改成你的服务端版本），但第 2~4 周部分功能（如轮椅行人 `use_wheelchair`，需 ≥ 0.9.15）可能不可用 |

**升级服务端到 0.9.16（仅服务端为旧版时需要）：**

1. 到 <https://github.com/carla-simulator/carla/releases> 下载 **0.9.16 的 Linux 包**（`CARLA_0.9.16.tar.gz`）；
2. 解压到新目录，旧版本可保留不动；
3. 启动时改用新版本目录下的 `CarlaUE4.sh`。

> 如果 pip 装 `carla` 失败（Python 版本没有对应 wheel），可用 CARLA Linux 发行包内自带的安装文件：
> `pip install /你的CARLA目录/PythonAPI/carla-0.9.16-*.whl`

### 第 5 步：启动（命令与旧版完全一致）

```bash
python main_ai.py        # Web 对话界面（默认），端口 3000
python main_ai.py mcp    # MCP stdio 模式
python main_ai.py sse    # SSE 模式，端口 3001，监听 0.0.0.0
```

启动时会先校验 `.env` 配置，看到 `[OK] 环境配置验证通过` 即配置保留成功。

---

## 三、配置说明（均保留，无需重填）

| 配置项 | 位置 | 说明 |
| --- | --- | --- |
| GITHUB_TOKEN / DEEPSEEK_API_KEY | `llm/.env` | 更新不覆盖，原样保留 |
| DEEPSEEK_API_URL / GITHUB_BASE_URL | `llm/.env` | 同上 |
| CARLA 连接地址 | Web 界面「连接 CARLA」按钮 | 界面内填写，不随代码更新丢失 |
| SUMO_HOME | 环境变量 | 仅 SUMO 路网功能需要，不涉及 CARLA 更新 |

> ⚠️ 安全提醒：`llm/src/config.py` 中存在一个硬编码的默认 DEEPSEEK_API_KEY。`.env` 里配置了自己的 key 时会覆盖它，不影响使用；但建议确认 `.env` 中 `DEEPSEEK_API_KEY` 是**自己的 key**。

---

## 四、更新后验证清单

依次确认，全部通过即更新完成：

1. **启动无报错**：`python main_ai.py` 出现 `[OK] 环境配置验证通过` 和 Web 地址；
2. **网页可打开**：浏览器访问 `http://<服务器IP>:3000`，能看到对话界面和场景快捷按钮面板；
3. **CARLA 可连接**：先启动 CARLA 服务端，网页上点「连接 CARLA」成功；
4. **工具无重复**（对应上游已修复的问题）：

   ```bash
   python -c "
   import re
   from collections import Counter
   src = open('llm/main_ai.py', encoding='utf-8').read()
   names = re.findall(r'\"name\":\s*\"(\w+)\"', src)
   dup = [n for n,c in Counter(names).items() if c>1 and 'ego_vehicle' not in n]
   print('重复工具:', dup if dup else '无 ✅')
   "
   ```

   预期输出 `无 ✅`。（`ego_vehicle` 出现两次是正常的，那是 OpenSCENARIO XML 的元素名，不是工具注册。）
5. **对话可用**：网页里发一句"生成特斯拉并开启自动驾驶"，能正常返回结果而非 `duplicate tool name` 报错；
6. **场景按钮抽测**：点 2~3 个快捷按钮（如"高速匝道""辅路场景"），CARLA 端能看到车辆生成。

---

## 五、关于"DeepSeek 报错 duplicate tool name"的说明

- 上游 main 在合并第 1~4 周功能时曾把 `control_walker`、`spawn_vehicle_param`、`scenario_highway_ramp`、`scenario_lane_merge`、`scenario_diverge_merge`、`scenario_side_road` 这 6 个工具注册了两次，DeepSeek 会因此拒绝请求。
- 修复方案：删除重复的 schema 定义（96 行），工具列表恢复为 40 项。修复后文件与已通过"四周 25 项工具真机回归测试"的干净版本逐字节一致。
- **更新代码前请确认该修复已合并进上游 main**（查看 <https://github.com/OpenHUTB/mcp/blob/main/llm/main_ai.py> 中 `"name": "control_walker"` 是否只出现一次）。
  - 已合并：直接按第二节流程更新即可；
  - 未合并：先只执行第 1、3 步（备份配置 + 更新依赖）等待合并，或手动删除 `self.tools` 中第二份重复的 6 个 schema 块（约 5070~5163 行）。

---

## 六、常见问题

**Q1：更新后网页打不开 3000 端口？**
检查防火墙/安全组是否放行 3000；确认启动日志里打印的访问地址中的 IP 是服务器实际网卡 IP。

**Q2：连接 CARLA 失败？**
确认 CARLA 服务端已启动（`./CarlaUE4.sh`），且网页填写的地址端口正确（默认 2000）。客户端与服务端版本不一致也会连接失败或随即崩溃。

**Q3：切换地图时崩溃？**
个别地图包可能损坏，先换 Town01/Town03 等常用地图验证；仍崩溃则考虑重下该地图或重装 CARLA 服务端。

**Q4：pip 安装 carla 报"找不到匹配版本"？**
用 CARLA 发行包自带的 wheel 本地安装（见第 4 步说明），并确认 Python 版本在 CARLA 0.9.16 支持范围内。
