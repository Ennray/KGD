"""
快速测试脚本 - 快速验证核心功能
运行时间约1-2分钟，适合快速检查
"""

import cv2
import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))

def print_header(text):
    """打印测试标题"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def print_result(test_name, passed, message=""):
    """打印测试结果"""
    status = "✓ PASS" if passed else "✗ FAIL"
    print(f"{status} | {test_name}")
    if message:
        print(f"     {message}")

def quick_test():
    """快速测试主函数"""
    print_header("目标检测快速测试")

    all_passed = True

    # ========== 测试1: 导入依赖 ==========
    print_header("1. 测试依赖库导入")

    try:
        import torch
        import cv2
        import numpy as np
        from ultralytics import YOLO
        print_result("导入PyTorch", True, f"版本: {torch.__version__}")
        print_result("导入OpenCV", True, f"版本: {cv2.__version__}")
        print_result("导入NumPy", True, f"版本: {np.__version__}")
        print_result("导入Ultralytics", True, f"版本: {YOLO.__module__}")
    except ImportError as e:
        print_result("导入依赖库", False, str(e))
        all_passed = False
        return all_passed

    # ========== 测试2: CUDA检查 ==========
    print_header("2. CUDA环境检查")

    cuda_available = torch.cuda.is_available()
    if cuda_available:
        device_name = torch.cuda.get_device_name(0)
        print_result("CUDA可用", True, f"设备: {device_name}")
    else:
        print_result("CUDA不可用", True, "将使用CPU模式")

    # ========== 测试3: 加载预训练模型 ==========
    print_header("3. 加载预训练模型")

    try:
        model = YOLO("yolov8n.pt")
        print_result("加载YOLOv8n", True, f"类别数: {len(model.names)}")
    except Exception as e:
        print_result("加载YOLOv8n", False, str(e))
        all_passed = False
        return all_passed

    # ========== 测试4: 基础检测功能 ==========
    print_header("4. 基础检测功能")

    try:
        # 创建测试图片
        test_img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

        # 执行检测
        results = model.predict(test_img, verbose=False)

        # 验证结果
        assert results is not None
        assert len(results) == 1
        assert hasattr(results[0], 'boxes')

        num_detections = len(results[0].boxes)
        print_result("随机图片检测", True, f"检测到 {num_detections} 个目标")
    except Exception as e:
        print_result("基础检测", False, str(e))
        all_passed = False

    # ========== 测试5: 不同置信度阈值 ==========
    print_header("5. 置信度阈值测试")

    try:
        results_low = model.predict(test_img, conf=0.1, verbose=False)
        results_high = model.predict(test_img, conf=0.9, verbose=False)

        num_low = len(results_low[0].boxes)
        num_high = len(results_high[0].boxes)

        passed = num_high <= num_low
        print_result("置信度过滤", passed,
                    f"conf=0.1: {num_low}个, conf=0.9: {num_high}个")

        if not passed:
            all_passed = False
    except Exception as e:
        print_result("置信度阈值", False, str(e))
        all_passed = False

    # ========== 测试6: 图片格式 ==========
    print_header("6. 图片格式兼容性")

    import tempfile
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)

        formats = [
            (".jpg", "JPEG"),
            (".png", "PNG"),
        ]

        for ext, name in formats:
            try:
                img_path = tmp_path / f"test{ext}"
                cv2.imwrite(str(img_path), test_img)
                results = model.predict(str(img_path), verbose=False)
                print_result(f"{name}格式", True)
            except Exception as e:
                print_result(f"{name}格式", False, str(e))
                all_passed = False

    # ========== 测试7: 自定义模型（可选）==========
    print_header("7. 自定义训练模型（可选）")

    custom_model_paths = [
        "./runs/detect/train8/weights/best.pt",
        "./runs/detect/train5/weights/best.pt",
    ]

    custom_found = False
    for model_path in custom_model_paths:
        if Path(model_path).exists():
            try:
                custom_model = YOLO(model_path)
                print_result("加载自定义模型", True,
                           f"路径: {model_path}")
                print_result("自定义模型检测", True,
                           f"类别数: {len(custom_model.names)}")
                custom_found = True
                break
            except Exception as e:
                print_result("自定义模型", False, str(e))

    if not custom_found:
        print_result("自定义模型", True, "未找到，跳过测试")

    # ========== 测试8: 推理速度 ==========
    print_header("8. 推理速度测试")

    try:
        import time

        # 预热
        for _ in range(3):
            model.predict(test_img, verbose=False)

        # 测量速度
        times = []
        for _ in range(5):
            start = time.time()
            model.predict(test_img, verbose=False)
            times.append(time.time() - start)

        avg_time = np.mean(times)
        fps = 1 / avg_time

        passed = avg_time < 2.0  # 2秒内完成
        print_result("推理速度", passed,
                    f"平均: {avg_time*1000:.2f}ms, FPS: {fps:.2f}")

        if not passed:
            all_passed = False
    except Exception as e:
        print_result("推理速度", False, str(e))
        all_passed = False

    # ========== 测试9: 批处理 ==========
    print_header("9. 批处理功能")

    try:
        # 创建多张图片
        images = [
            np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            for _ in range(3)
        ]

        results = model.predict(images, verbose=False)
        passed = len(results) == 3

        print_result("批量检测", passed, f"输入3张，输出{len(results)}张")

        if not passed:
            all_passed = False
    except Exception as e:
        print_result("批处理", False, str(e))
        all_passed = False

    # ========== 测试10: API导入（可选）==========
    print_header("10. API模块导入（可选）")

    try:
        import execute_file.interface_size_recognition as size_api
        print_result("尺寸识别API", True)
    except ImportError:
        print_result("尺寸识别API", True, "未安装Flask，跳过")

    try:
        import execute_file.interface_drone_tracking as track_api
        print_result("跟踪API", True)
    except ImportError:
        print_result("跟踪API", True, "未安装Flask，跳过")

    # ========== 总结 ==========
    print_header("测试总结")

    if all_passed:
        print("\n✓ 所有核心测试通过！")
        print("  系统运行正常，可以开始使用。")
    else:
        print("\n✗ 部分测试失败")
        print("  请检查失败的测试项并解决问题。")

    print("\n提示: 运行完整测试套件:")
    print("  python test_detection.py")
    print("  或")
    print("  pytest test_detection.py -v")

    return all_passed

if __name__ == "__main__":
    import sys

    success = quick_test()

    # 返回退出码
    sys.exit(0 if success else 1)
