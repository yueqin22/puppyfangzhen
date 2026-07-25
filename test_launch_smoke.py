"""ROS2 Launch Smoke Test (v8.5)

验证所有核心launch文件可以被正确解析和加载。
不启动完整的ROS2系统，只检查launch文件语法和依赖完整性。

运行方式: python test_launch_smoke.py
"""
import sys
import os
import importlib.util
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def test_launch_file(launch_path):
    """测试单个launch文件是否可以被解析

    Args:
        launch_path: launch文件路径

    Returns:
        (bool, str): (是否成功, 消息)
    """
    try:
        # 尝试导入launch模块
        spec = importlib.util.spec_from_file_location("launch_module", launch_path)  # noqa
        if spec is None or spec.loader is None:
            return False, f"无法创建模块规格: {launch_path}"
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        # 检查是否有generate_launch_description函数
        if hasattr(module, 'generate_launch_description'):
            # 尝试调用（不实际启动节点）
            desc = module.generate_launch_description()
            if desc is not None:
                return True, f"OK ({len(desc.entities)} entities)"
            return False, "generate_launch_description返回None"
        else:
            return False, "缺少generate_launch_description函数"
    except ImportError as e:
        # ROS2 launch模块未安装属于环境问题，标记为SKIP而非FAIL
        if 'launch' in str(e).lower() or 'ament' in str(e).lower():
            return None, f"SKIP (ROS2未安装): {e}"
        return False, f"导入失败: {e}"
    except Exception as e:
        return False, f"异常: {e}"


def main():
    print("=" * 60)
    print("ROS2 Launch Smoke Test (v8.5)")
    print("=" * 60)

    # 查找所有launch文件
    launch_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'src', 'puppy_bringup', 'launch')
    if not os.path.isdir(launch_dir):
        print(f"Launch目录不存在: {launch_dir}")
        return 1

    launch_files = [f for f in os.listdir(launch_dir)
                    if f.endswith('.py') and not f.startswith('__')]

    if not launch_files:
        print("未找到launch文件")
        return 1

    print(f"找到 {len(launch_files)} 个launch文件\n")

    passed = 0
    failed = 0
    skipped = 0

    for fname in sorted(launch_files):
        path = os.path.join(launch_dir, fname)
        ok, msg = test_launch_file(path)
        if ok is None:
            status = "SKIP"
            skipped += 1
        else:
            status = "PASS" if ok else "FAIL"
            if ok:
                passed += 1
            else:
                failed += 1
        print(f"  [{status}] {fname}: {msg}")

    print(f"\n{'='*60}")
    print(f"结果: {passed} passed, {failed} failed, {skipped} skipped, {passed + failed + skipped} total")
    print("=" * 60)
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
