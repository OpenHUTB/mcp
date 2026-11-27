"""DeepSeek API Key 连通性自检（不会打印 Key 本身）。

用法（在项目根目录）:
    dependencies\\prerequisites\\miniconda3\\envs\\hutb_3.10\\python.exe llm\\check_deepseek_key.py
"""
import asyncio
import sys
from pathlib import Path

import aiohttp
from dotenv import load_dotenv

ROOT = Path(__file__).resolve().parents[1]
load_dotenv(ROOT / ".env")
load_dotenv(Path(__file__).resolve().parent / ".env")


async def main():
    import os
    key = (os.getenv("DEEPSEEK_API_KEY") or "").strip()
    url = os.getenv("DEEPSEEK_API_URL", "https://api.deepseek.com/chat/completions")
    if not key:
        print("[FAIL] .env 里没有 DEEPSEEK_API_KEY")
        return 1
    print(f"[INFO] Key 长度 {len(key)}，前 3 位 {key[:3]}，后 4 位 {key[-4:]}，请求地址 {url}")
    payload = {"model": os.getenv("DEEPSEEK_MODEL", "deepseek-chat"),
               "messages": [{"role": "user", "content": "ping"}],
               "max_tokens": 1}
    headers = {"Authorization": f"Bearer {key}", "Content-Type": "application/json"}
    try:
        async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=30)) as s:
            async with s.post(url, headers=headers, json=payload) as r:
                text = await r.text()
                print(f"[RESULT] HTTP {r.status}: {text[:300]}")
                if r.status == 200:
                    print("[OK] Key 可用")
                    return 0
                if r.status == 401:
                    print("[FAIL] Key 无效 / 不是 DeepSeek 官方 Key")
                    return 2
                if r.status == 402:
                    print("[FAIL] 账户余额不足")
                    return 3
                return 4
    except Exception as e:
        print(f"[ERROR] {type(e).__name__}: {e!r}")
        return 5


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
