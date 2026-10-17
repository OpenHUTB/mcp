from datetime import datetime
import shutil
import os


# 动态获取页脚版权页面的年份
def on_config(config, **kwargs):
    year = str(datetime.now().year)
    config.copyright = config.copyright.format(year=year)


# 在 MkDocs 构建开始前，将根目录下的 requirements.txt 拷贝到 docs 目录下，
# 确保文档站点能够引用到它（仅用于示例，实际使用时请根据需要修改 copied_file）。
# （名称必须为 on_pre_build）
def on_pre_build(**kwargs):
    copied_file = 'requirements.txt'
    # 源文件：项目根目录下的文件
    src_file = os.path.join(os.path.dirname(__file__), copied_file)
    # 目标文件：文档目录（相对于 docs_dir）
    dest_file = os.path.join(os.path.dirname(__file__), 'docs', copied_file)
    
    # 执行拷贝
    if os.path.exists(src_file):
        shutil.copy2(src_file, dest_file)
        print(f"Successfully copied {copied_file} to docs/{copied_file}")

 # 构建完成后，输出文档构建统计信息
def on_post_build(config, **kwargs):
    import os
    docs_path = config["docs_dir"]
    md_count = 0
    # 遍历统计所有md文档数量
    for root, dirs, files in os.walk(docs_path):
        for f in files:
            if f.endswith(".md"):
                md_count += 1
    print(f"✅ 文档网站构建完成！")
    print(f"📄 检测到Markdown文档总数：{md_count}") 