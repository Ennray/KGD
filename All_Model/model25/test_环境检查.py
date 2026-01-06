"""
简化版快速测试 - 不下载模型
仅测试环境配置和基本功能
"""

import sys
from pathlib import Path

def print_header(text):
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def print_result(test_name, passed, message=""):
    status = " PASS" if passed else " FAIL"
    print(f"{status} | {test_name}")
    if message:
        print(f"     {message}")

def main():
    print_header("简化版环境测试（无需下载模型）")

    all_passed = True

    # 测试1: Python版本
    print_header("1. Python 环境")
    print_result("Python版本", True, f"{sys.version.split()[0]}")
    print_result("Python路径", True, f"{sys.executable}")

    # 测试2: 导入基础库
    print_header("2. 基础库导入测试")

    try:
        import numpy as np
        print_result("NumPy", True, f"版本 {np.__version__}")
    except ImportError as e:
        print_result("NumPy", False, str(e))
        all_passed = False

    try:
        import cv2
        print_result("OpenCV", True, f"版本 {cv2.__version__}")
    except ImportError as e:
        print_result("OpenCV", False, str(e))
        all_passed = False

    try:
        import torch
        print_result("PyTorch", True, f"版本 {torch.__version__}")
        cuda = torch.cuda.is_available()
        print_result("CUDA", cuda,
                    f"设备: {torch.cuda.get_device_name(0)}" if cuda else "使用CPU模式")
    except ImportError as e:
        print_result("PyTorch", False, str(e))
        all_passed = False

    # 测试3: 测试框架
    print_header("3. 测试框架")

    try:
        import pytest
        print_result("pytest", True, f"已安装")
    except ImportError:
        print_result("pytest", False, "未安装，运行: python -m pip install pytest")
        all_passed = False

    # 测试4: YOLO库
    print_header("4. Ultralytics (YOLO)")

    try:
        from ultralytics import YOLO
        print_result("ultralytics", True, "已安装")
    except ImportError as e:
        print_result("ultralytics", False, str(e))
        all_passed = False

    # 测试5: 基本数组操作
    print_header("5. 基本功能测试")

    try:
        import numpy as np
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        print_result("创建测试图像", True, f"尺寸: {img.shape}")
    except Exception as e:
        print_result("创建测试图像", False, str(e))
        all_passed = False

    try:
        import cv2
        import tempfile
        import os

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir) / "test.jpg"
            cv2.imwrite(str(tmp_path), img)
            loaded = cv2.imread(str(tmp_path))
            assert loaded is not None
            print_result("图像读写", True, "成功")
    except Exception as e:
        print_result("图像读写", False, str(e))
        all_passed = False

    # 测试6: 文件检查
    print_header("6. 测试文件检查")

    test_files = [
        "test_detection.py",
        "test_quick.py",
        "运行快速测试.bat",
        "运行完整测试.bat",
    ]

    for file in test_files:
        exists = Path(file).exists()
        print_result(file, exists, "存在" if exists else "缺失")

    # 总结
    print_header("测试总结")

    if all_passed:
        print("\n 所有环境测试通过！")
        print("\n下一步：")
        print("  1. 如果想运行快速测试，双击: 运行快速测试.bat")
        print("  2. 如果想运行完整测试，双击: 运行完整测试.bat")
        print("  3. 或使用命令: python -m pytest test_detection.py -v")
    else:
        print("\n 部分测试失败")
        print("\n请安装缺失的依赖：")
        print("  python -m pip install -r test_requirements.txt")

    print("\n" + "=" * 60)

    return all_passed

if __name__ == "__main__":
    success = main()

    input("\n按回车键退出...")
    sys.exit(0 if success else 1)
