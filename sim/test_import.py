"""
HUTB MCP 模块导入测试脚本

用于验证所有模块是否可以正确导入。
"""

import sys

def test_imports():
    """测试所有模块导入"""
    print("=" * 50)
    print("HUTB MCP 模块导入测试")
    print("=" * 50)
    
    errors = []
    
    # 测试核心模块
    print("\n[1] 测试核心模块...")
    
    try:
        from hutb_mcp import main
        print("  ✓ hutb_mcp 主模块")
    except Exception as e:
        print(f"  ✗ hutb_mcp 主模块: {e}")
        errors.append(("hutb_mcp", str(e)))
    
    try:
        from hutb_mcp.config import get_config
        cfg = get_config()
        print(f"  ✓ config 模块 (服务器: {cfg.mcp.server_name})")
    except Exception as e:
        print(f"  ✗ config 模块: {e}")
        errors.append(("config", str(e)))
    
    try:
        from hutb_mcp.connection import HutbConnection
        print("  ✓ connection 模块")
    except Exception as e:
        print(f"  ✗ connection 模块: {e}")
        errors.append(("connection", str(e)))
    
    try:
        from hutb_mcp.server import mcp
        print("  ✓ server 模块")
    except Exception as e:
        print(f"  ✗ server 模块: {e}")
        errors.append(("server", str(e)))
    
    # 测试工具模块
    print("\n[2] 测试工具模块...")
    
    tool_modules = [
        ("vehicle_tools", "hutb_mcp.tools.vehicle_tools"),
        ("editor_tools", "hutb_mcp.tools.editor_tools"),
        ("weather_tools", "hutb_mcp.tools.weather_tools"),
        ("sensor_tools", "hutb_mcp.tools.sensor_tools"),
        ("air_tools", "hutb_mcp.tools.air_tools"),
    ]
    
    for name, module_path in tool_modules:
        try:
            __import__(module_path)
            print(f"  ✓ {name}")
        except Exception as e:
            print(f"  ✗ {name}: {e}")
            errors.append((name, str(e)))
    
    # 测试工具模块
    print("\n[3] 测试工具函数...")
    
    try:
        from hutb_mcp.utils.logger import setup_logging, get_logger
        print("  ✓ logger 工具")
    except Exception as e:
        print(f"  ✗ logger 工具: {e}")
        errors.append(("logger", str(e)))
    
    # 测试 CARLA 模块
    print("\n[4] 测试 CARLA 模块...")
    
    try:
        import carla
        print(f"  ✓ carla 模块 (版本: {carla.__version__ if hasattr(carla, '__version__') else '未知'})")
    except ImportError:
        print("  ⚠ carla 模块未安装 (需要连接仿真器时安装)")
    except Exception as e:
        print(f"  ✗ carla 模块: {e}")
    
    # 测试 FastMCP
    print("\n[5] 测试 FastMCP...")
    
    try:
        from mcp.server.fastmcp import FastMCP
        print("  ✓ FastMCP 框架")
    except Exception as e:
        print(f"  ✗ FastMCP 框架: {e}")
        errors.append(("FastMCP", str(e)))
    
    # 总结
    print("\n" + "=" * 50)
    if errors:
        print(f"测试完成，发现 {len(errors)} 个错误:")
        for name, error in errors:
            print(f"  - {name}: {error}")
        return 1
    else:
        print("✓ 所有模块导入测试通过！")
        return 0


if __name__ == "__main__":
    sys.exit(test_imports())
