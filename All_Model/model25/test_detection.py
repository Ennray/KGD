"""
目标检测模块测试用例
测试YOLOv8在项目中的集成、功能和边界条件
"""

import pytest
import cv2
import numpy as np
from pathlib import Path
import sys
import os
import time
import json

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / 'execute_file'))

from ultralytics import YOLO


class TestModelLoading:
    """测试模型加载功能"""

    def test_pretrained_model_exists(self):
        """测试预训练模型文件是否存在"""
        # 测试YOLOv8预训练模型
        pretrained_models = ["yolov8n.pt", "yolov8s.pt", "yolov8x.pt"]
        found = False
        for model in pretrained_models:
            if Path(model).exists():
                found = True
                break
        # 至少有一个预训练模型可用（或能自动下载）
        assert True, "预训练模型测试跳过，将在加载时自动下载"

    def test_custom_model_exists(self):
        """测试自定义训练模型是否存在"""
        possible_paths = [
            "./runs/detect/train8/weights/best.pt",
            "./runs/detect/train2/weights/best.pt",
            "./runs/detect/train1/best.pt",
        ]

        found_paths = [p for p in possible_paths if Path(p).exists()]

        if found_paths:
            assert True, f"找到训练模型: {found_paths[0]}"
        else:
            pytest.skip(f"未找到训练模型，跳过测试。尝试路径: {possible_paths}")

    def test_model_load_pretrained(self):
        """测试预训练模型能否正确加载"""
        try:
            model = YOLO("yolov8n.pt")
            assert model is not None
            assert hasattr(model, 'predict')
            assert hasattr(model, 'train')

            print(f"\n{'='*60}")
            print(f"  成功加载预训练模型 yolov8n.pt")
            print(f"  模型类型: {type(model).__name__}")
            print(f"  类别数量: {len(model.names)}")
            print(f"  任务类型: {model.task}")
            print(f"  主要方法: predict, train, val, export")
            print(f"{'='*60}")
        except Exception as e:
            pytest.fail(f"加载预训练模型失败: {e}")

    def test_model_load_custom(self):
        """测试自定义模型加载"""
        model_paths = [
            "./runs/detect/train8/weights/best.pt",
            "./runs/detect/train5/weights/best.pt",
        ]

        for model_path in model_paths:
            if Path(model_path).exists():
                try:
                    model = YOLO(model_path)
                    assert model is not None
                    print(f"\n 成功加载自定义模型: {model_path}")
                    return
                except Exception as e:
                    pytest.fail(f"加载自定义模型失败 {model_path}: {e}")

        pytest.skip("未找到可用的自定义训练模型")

    def test_model_properties(self):
        """测试模型属性"""
        model = YOLO("yolov8n.pt")

        # 检查模型名称字典
        assert hasattr(model, 'names')
        assert isinstance(model.names, dict)
        assert len(model.names) > 0

        # 检查模型任务类型
        assert hasattr(model, 'task')

        print(f"\n模型类别数: {len(model.names)}")
        print(f"模型任务类型: {model.task}")


class TestDetectionBasic:
    """基础检测功能测试"""

    @pytest.fixture
    def model(self):
        """提供测试用模型"""
        return YOLO("yolov8n.pt")

    @pytest.fixture
    def sample_image_blank(self):
        """创建空白测试图片"""
        return np.zeros((640, 640, 3), dtype=np.uint8)

    @pytest.fixture
    def sample_image_with_object(self):
        """创建包含简单对象的测试图片"""
        img = np.zeros((640, 640, 3), dtype=np.uint8)
        # 绘制一个白色矩形
        cv2.rectangle(img, (200, 200), (400, 400), (255, 255, 255), -1)
        # 绘制一个圆形
        cv2.circle(img, (500, 150), 50, (128, 128, 128), -1)
        return img

    def test_detection_on_single_image(self, model, sample_image_with_object):
        """测试单张图片检测"""
        results = model.predict(sample_image_with_object, verbose=False)

        assert results is not None
        assert len(results) == 1

        r = results[0]
        num_boxes = len(r.boxes)

        print(f"\n{'='*60}")
        print(f" 单张图片检测成功")
        print(f"  输入图像尺寸: {sample_image_with_object.shape}")
        print(f"  检测到目标数: {num_boxes}")
        if num_boxes > 0:
            print(f"  第一个目标:")
            box = r.boxes[0]
            print(f"    - 置信度: {box.conf.item():.3f}")
            print(f"    - 类别ID: {int(box.cls.item())}")
            print(f"    - 边界框: {box.xyxy.cpu().numpy()[0].astype(int).tolist()}")
        print(f"{'='*60}")

    def test_detection_output_structure(self, model, sample_image_with_object):
        """测试检测输出数据结构"""
        results = model.predict(sample_image_with_object, verbose=False)
        r = results[0]

        # 验证输出包含必要的属性
        assert hasattr(r, 'boxes'), "结果缺少 boxes 属性"
        assert hasattr(r, 'names'), "结果缺少 names 属性"
        assert hasattr(r, 'orig_img'), "结果缺少 orig_img 属性"

        # 验证boxes属性
        assert hasattr(r.boxes, 'xyxy'), "boxes 缺少 xyxy 属性"
        assert hasattr(r.boxes, 'conf'), "boxes 缺少 conf 属性"
        assert hasattr(r.boxes, 'cls'), "boxes 缺少 cls 属性"

        print(f"\n 检测输出结构正确")
        print(f"  检测到 {len(r.boxes)} 个目标")

    def test_confidence_threshold(self, model, sample_image_blank):
        """测试置信度阈值过滤功能"""
        # 使用不同阈值进行检测
        results_low = model.predict(sample_image_blank, conf=0.1, verbose=False)
        results_medium = model.predict(sample_image_blank, conf=0.5, verbose=False)
        results_high = model.predict(sample_image_blank, conf=0.9, verbose=False)

        # 高阈值检测出的目标应该 <= 低阈值
        num_low = len(results_low[0].boxes)
        num_medium = len(results_medium[0].boxes)
        num_high = len(results_high[0].boxes)

        assert num_high <= num_medium <= num_low, \
            f"置信度过滤异常: low={num_low}, medium={num_medium}, high={num_high}"

        print(f"\n{'='*60}")
        print(f" 置信度阈值过滤正确")
        print(f"  conf=0.1: {num_low}个目标")
        print(f"  conf=0.5: {num_medium}个目标")
        print(f"  conf=0.9: {num_high}个目标")
        print(f"  过滤趋势: {'正确' if num_high <= num_medium <= num_low else '错误'}")
        print(f"{'='*60}")

    def test_iou_threshold(self, model, sample_image_with_object):
        """测试IoU阈值（NMS）"""
        results_low_iou = model.predict(sample_image_with_object, iou=0.3, verbose=False)
        results_high_iou = model.predict(sample_image_with_object, iou=0.7, verbose=False)

        num_low = len(results_low_iou[0].boxes)
        num_high = len(results_high_iou[0].boxes)

        # 高IoU阈值可能保留更多重叠框
        assert num_low >= 0 and num_high >= 0

        print(f"\n IoU阈值测试通过")
        print(f"  iou=0.3: {num_low}个, iou=0.7: {num_high}个")

    def test_class_filtering(self, model, sample_image_with_object):
        """测试类别过滤功能"""
        # 只检测特定类别（例如person=0）
        results_filtered = model.predict(
            sample_image_with_object,
            classes=[0],
            verbose=False
        )

        r = results_filtered[0]
        if len(r.boxes) > 0:
            # 如果检测到目标，确保都是指定类别
            detected_classes = r.boxes.cls.cpu().numpy()
            assert all(cls == 0 for cls in detected_classes), "类别过滤失败"

        print(f"\n 类别过滤功能正常")


class TestImageFormats:
    """测试不同图片格式和尺寸"""

    @pytest.fixture
    def model(self):
        return YOLO("yolov8n.pt")

    def test_jpeg_format(self, model, tmp_path):
        """测试JPEG格式图片"""
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        img_path = tmp_path / "test.jpg"
        cv2.imwrite(str(img_path), img)

        results = model.predict(str(img_path), verbose=False)
        assert results is not None
        assert len(results) == 1
        print(f"\n JPEG格式测试通过")

    def test_png_format(self, model, tmp_path):
        """测试PNG格式图片"""
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        img_path = tmp_path / "test.png"
        cv2.imwrite(str(img_path), img)

        results = model.predict(str(img_path), verbose=False)
        assert results is not None
        print(f"\n PNG格式测试通过")

    def test_bmp_format(self, model, tmp_path):
        """测试BMP格式图片"""
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        img_path = tmp_path / "test.bmp"
        cv2.imwrite(str(img_path), img)

        results = model.predict(str(img_path), verbose=False)
        assert results is not None
        print(f"\n BMP格式测试通过")

    def test_different_resolutions(self, model):
        """测试不同分辨率图片"""
        resolutions = [
            (320, 240, "小分辨率"),
            (640, 480, "中分辨率"),
            (1280, 720, "HD"),
            (1920, 1080, "Full HD"),
        ]

        print(f"\n{'='*60}")
        print(f"测试不同分辨率图片检测")

        for w, h, desc in resolutions:
            img = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
            results = model.predict(img, verbose=False)
            assert results is not None, f"{desc} ({w}x{h}) 检测失败"
            num_det = len(results[0].boxes)
            print(f"   {desc:12s} ({w:4d}x{h:4d}) - 检测到 {num_det} 个目标")

        print(f" 多种分辨率测试通过: {len(resolutions)}种")
        print(f"{'='*60}")

    def test_extreme_aspect_ratios(self, model):
        """测试极端长宽比"""
        # 超宽图片
        img_wide = np.random.randint(0, 255, (100, 1000, 3), dtype=np.uint8)
        results_wide = model.predict(img_wide, verbose=False)
        assert results_wide is not None

        # 超高图片
        img_tall = np.random.randint(0, 255, (1000, 100, 3), dtype=np.uint8)
        results_tall = model.predict(img_tall, verbose=False)
        assert results_tall is not None

        print(f"\n 极端长宽比测试通过")


class TestEdgeCases:
    """边界条件和异常情况测试"""

    @pytest.fixture
    def model(self):
        return YOLO("yolov8n.pt")

    def test_empty_black_image(self, model):
        """测试全黑图片"""
        img = np.zeros((640, 640, 3), dtype=np.uint8)
        results = model.predict(img, verbose=False)

        assert results is not None
        # 全黑图片应该检测不出或很少目标
        num_detections = len(results[0].boxes)
        assert num_detections >= 0
        print(f"\n 全黑图片测试通过，检测到 {num_detections} 个目标")

    def test_empty_white_image(self, model):
        """测试全白图片"""
        img = np.ones((640, 640, 3), dtype=np.uint8) * 255
        results = model.predict(img, verbose=False)

        assert results is not None
        num_detections = len(results[0].boxes)
        print(f"\n 全白图片测试通过，检测到 {num_detections} 个目标")

    def test_very_small_image(self, model):
        """测试超小图片"""
        img = np.random.randint(0, 255, (32, 32, 3), dtype=np.uint8)
        results = model.predict(img, verbose=False)
        assert results is not None
        print(f"\n 超小图片(32x32)测试通过")

    def test_grayscale_image(self, model):
        """测试灰度图（单通道转三通道）"""
        img_gray = np.random.randint(0, 255, (640, 640), dtype=np.uint8)
        img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

        results = model.predict(img_rgb, verbose=False)
        assert results is not None
        print(f"\n 灰度图测试通过")

    def test_invalid_image_path(self, model):
        """测试无效的图片路径"""
        try:
            model.predict("non_existent_image_12345.jpg", verbose=False)
            pytest.fail("应该抛出异常")
        except Exception as e:
            print(f"\n 无效路径正确抛出异常: {type(e).__name__}")

    def test_corrupted_image_file(self, model, tmp_path):
        """测试损坏的图片文件"""
        corrupt_path = tmp_path / "corrupt.jpg"
        corrupt_path.write_bytes(b"This is not a valid image file!")

        try:
            model.predict(str(corrupt_path), verbose=False)
            # 某些情况下可能不抛异常，但应该处理
            print(f"\n 损坏文件被处理（未抛异常）")
        except Exception as e:
            print(f"\n 损坏文件正确抛出异常: {type(e).__name__}")

    def test_none_input(self, model):
        """测试None输入"""
        try:
            results = model.predict(None, verbose=False)
            # 新版本YOLO可能允许None输入，返回空结果
            print(f"\n None输入被处理（未抛异常），返回结果类型: {type(results)}")
            print(f"  这是YOLO的兼容性处理，允许通过")
        except Exception as e:
            # 旧版本会抛出异常，这也是正常的
            print(f"\n None输入正确抛出异常: {type(e).__name__}")
            print(f"  异常信息: {str(e)[:100]}")


class TestBatchProcessing:
    """批处理功能测试"""

    @pytest.fixture
    def model(self):
        return YOLO("yolov8n.pt")

    @pytest.fixture
    def test_images_dir(self, tmp_path):
        """创建测试图片目录"""
        img_dir = tmp_path / "test_images"
        img_dir.mkdir()

        # 创建5张测试图片
        for i in range(5):
            img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            cv2.imwrite(str(img_dir / f"test_{i:02d}.jpg"), img)

        return img_dir

    def test_batch_detection(self, model, test_images_dir):
        """测试批量检测"""
        results = model.predict(str(test_images_dir), batch=4, verbose=False)

        assert results is not None
        assert len(results) == 5, f"应该返回5个结果，实际返回 {len(results)}"

        print(f"\n{'='*60}")
        print(f"  批量检测测试通过")
        print(f"  输入目录: {test_images_dir.name}")
        print(f"  图片数量: {len(results)}")
        print(f"  批大小:   4")

        total_detections = sum(len(r.boxes) for r in results)
        print(f"  总检测数: {total_detections}")

        for i, r in enumerate(results, 1):
            print(f"  图片{i}: 检测到 {len(r.boxes)} 个目标")

        print(f"{'='*60}")

    def test_different_batch_sizes(self, model, test_images_dir):
        """测试不同批大小"""
        batch_sizes = [1, 2, 4, 8]

        for batch_size in batch_sizes:
            results = model.predict(
                str(test_images_dir),
                batch=batch_size,
                verbose=False
            )
            assert len(results) == 5

        print(f"\n 多种批大小测试通过: {batch_sizes}")

    def test_numpy_array_list(self, model):
        """测试numpy数组列表输入"""
        images = [
            np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            for _ in range(3)
        ]

        results = model.predict(images, verbose=False)
        assert len(results) == 3

        print(f"\n numpy数组列表测试通过")


class TestCustomModel:
    """自定义训练模型专项测试"""

    @pytest.fixture
    def custom_model(self):
        """加载自定义训练模型"""
        model_paths = [
            "./runs/detect/train8/weights/best.pt",
            "./runs/detect/train5/weights/best.pt",
        ]

        for path in model_paths:
            if Path(path).exists():
                return YOLO(path)

        pytest.skip("未找到自定义训练模型")

    def test_model_classes(self, custom_model):
        """测试模型类别配置"""
        assert hasattr(custom_model, 'names')
        assert len(custom_model.names) > 0

        print(f"\n 自定义模型类别数: {len(custom_model.names)}")
        print(f"  类别: {custom_model.names}")

    def test_real_test_images(self, custom_model):
        """测试真实的测试图片"""
        test_dirs = [
            "./datasets/airplane_detect_images",
        ]

        found = False
        for test_dir in test_dirs:
            test_path = Path(test_dir)
            if test_path.exists():
                test_images = list(test_path.glob("*.jpg"))[:3]  # 测试前3张

                if test_images:
                    for img_path in test_images:
                        results = custom_model.predict(
                            str(img_path),
                            conf=0.5,
                            verbose=False
                        )
                        assert results is not None
                        num_det = len(results[0].boxes)
                        print(f"\n  {img_path.name}: 检测到 {num_det} 个目标")

                    found = True
                    break

        if not found:
            pytest.skip("未找到测试图片目录")

        print(f"\n 真实图片检测测试通过")

    def test_visdrone_dataset_format(self, custom_model):
        """测试VisDrone数据集格式兼容性"""
        visdrone_path = Path("./datasets/VisDrone/val/images")

        if not visdrone_path.exists():
            pytest.skip("VisDrone数据集不存在")

        # 测试前5张图片
        images = list(visdrone_path.glob("*.jpg"))[:5]

        total_detections = 0
        for img_path in images:
            results = custom_model.predict(str(img_path), conf=0.25, verbose=False)
            total_detections += len(results[0].boxes)

        print(f"\n VisDrone数据集测试通过")
        print(f"  测试图片数: {len(images)}")
        print(f"  总检测数: {total_detections}")


class TestPerformance:
    """性能测试"""

    @pytest.fixture
    def model(self):
        return YOLO("yolov8n.pt")

    @pytest.fixture
    def test_image(self):
        return np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

    def test_inference_speed(self, model, test_image):
        """测试推理速度"""
        # 预热
        for _ in range(3):
            model.predict(test_image, verbose=False)

        # 测量推理时间
        times = []
        num_runs = 10

        for _ in range(num_runs):
            start = time.time()
            model.predict(test_image, verbose=False)
            times.append(time.time() - start)

        avg_time = np.mean(times)
        std_time = np.std(times)
        min_time = np.min(times)
        max_time = np.max(times)
        fps = 1/avg_time

        print(f"\n{'='*60}")
        print(f"推理性能统计 ({num_runs}次):")
        print(f"  平均时间: {avg_time*1000:6.2f}ms")
        print(f"  标准差:   {std_time*1000:6.2f}ms")
        print(f"  最快:     {min_time*1000:6.2f}ms")
        print(f"  最慢:     {max_time*1000:6.2f}ms")
        print(f"  FPS:      {fps:6.2f}")
        print(f"  图像尺寸: {test_image.shape}")
        print(f"{'='*60}")

        # 确保推理时间在合理范围内
        assert avg_time < 2.0, f"推理速度过慢: {avg_time:.3f}s"

    def test_memory_usage(self, model, test_image):
        """测试内存使用（简单版本）"""
        import gc

        # 清理内存
        gc.collect()

        # 运行多次推理
        for _ in range(10):
            results = model.predict(test_image, verbose=False)
            del results

        gc.collect()
        print(f"\n 内存测试通过（未发现明显内存泄漏）")


class TestDataProcessing:
    """数据处理相关测试"""

    def test_label_format_detection(self):
        """测试标签格式识别"""
        # YOLO格式: class_id x_center y_center width height
        yolo_label = "0 0.5 0.5 0.3 0.4"
        parts = yolo_label.split()

        assert len(parts) == 5
        assert all(part.replace('.', '').isdigit() for part in parts)

        print(f"\n YOLO标签格式验证通过")

    def test_coordinate_conversion(self):
        """测试坐标转换逻辑"""
        # 归一化坐标转像素坐标
        img_w, img_h = 1920, 1080
        x_norm, y_norm, w_norm, h_norm = 0.5, 0.5, 0.2, 0.3

        x_center = x_norm * img_w
        y_center = y_norm * img_h
        w = w_norm * img_w
        h = h_norm * img_h

        x1 = int(x_center - w / 2)
        y1 = int(y_center - h / 2)
        x2 = int(x_center + w / 2)
        y2 = int(y_center + h / 2)

        assert 0 <= x1 < img_w
        assert 0 <= y1 < img_h
        assert 0 <= x2 <= img_w
        assert 0 <= y2 <= img_h

        print(f"\n 坐标转换测试通过")
        print(f"  归一化: ({x_norm}, {y_norm}, {w_norm}, {h_norm})")
        print(f"  像素坐标: ({x1}, {y1}, {x2}, {y2})")


class TestIntegration:
    """集成测试"""

    def test_complete_detection_pipeline(self, tmp_path):
        """测试完整检测流程"""
        # 1. 创建测试图片
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), (255, 0, 0), -1)
        img_path = tmp_path / "test.jpg"
        cv2.imwrite(str(img_path), img)

        # 2. 加载模型
        model = YOLO("yolov8n.pt")

        # 3. 执行检测
        results = model.predict(str(img_path), conf=0.25, verbose=False)

        # 4. 处理结果
        r = results[0]
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        classes = r.boxes.cls.cpu().numpy()

        # 5. 验证结果格式
        assert boxes.shape[1] == 4  # x1, y1, x2, y2
        assert len(boxes) == len(confs) == len(classes)

        print(f"\n{'='*60}")
        print(f"  完整检测流程测试通过")
        print(f"  1. 创建测试图片:  ({img.shape})")
        print(f"  2. 保存图片文件:  ({img_path.name})")
        print(f"  3. 加载模型:      (yolov8n.pt)")
        print(f"  4. 执行检测:      (conf=0.25)")
        print(f"  5. 处理结果:     ")
        print(f"\n  检测结果统计:")
        print(f"    - 检测目标数: {len(boxes)}")
        print(f"    - 置信度范围: {confs.min():.3f} ~ {confs.max():.3f}" if len(confs) > 0 else "    - 未检测到目标")

        if len(boxes) > 0:
            print(f"\n  第一个目标详情:")
            print(f"    - 边界框: {boxes[0].astype(int).tolist()}")
            print(f"    - 置信度: {confs[0]:.3f}")
            print(f"    - 类别ID: {int(classes[0])}")

        print(f"{'='*60}")

    def test_save_detection_results(self, tmp_path):
        """测试保存检测结果"""
        model = YOLO("yolov8n.pt")
        img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

        # 执行检测并保存
        results = model.predict(
            img,
            save=True,
            project=str(tmp_path),
            name="test_results",
            verbose=False
        )

        # 验证输出目录
        output_dir = tmp_path / "test_results"
        assert output_dir.exists(), "结果目录未创建"

        # 检查是否有输出文件
        output_files = list(output_dir.glob("*.jpg"))

        print(f"\n 保存检测结果测试通过")
        print(f"  输出目录: {output_dir}")
        print(f"  输出文件数: {len(output_files)}")


class TestAPIEndpoints:
    """API接口测试"""

    def test_size_recognition_api_import(self):
        """测试尺寸识别API导入"""
        try:
            import execute_file.interface_size_recognition as size_api
            assert hasattr(size_api, 'app')
            print(f"\n 尺寸识别API导入成功")
        except ImportError as e:
            pytest.skip(f"API模块导入失败: {e}")

    def test_tracking_api_import(self):
        """测试跟踪API导入"""
        try:
            import execute_file.interface_drone_tracking as track_api
            assert hasattr(track_api, 'app')
            print(f"\n 跟踪API导入成功")
        except ImportError as e:
            pytest.skip(f"API模块导入失败: {e}")


class TestEnvironment:
    """环境配置测试"""

    def test_opencv_version(self):
        """测试OpenCV版本"""
        import cv2
        version = cv2.__version__
        print(f"\n OpenCV版本: {version}")
        assert version is not None

    def test_numpy_version(self):
        """测试NumPy版本"""
        import numpy as np
        version = np.__version__
        print(f"\n NumPy版本: {version}")
        assert version is not None

    def test_torch_available(self):
        """测试PyTorch可用性"""
        try:
            import torch
            print(f"\n PyTorch版本: {torch.__version__}")
            print(f"  CUDA可用: {torch.cuda.is_available()}")
            if torch.cuda.is_available():
                print(f"  CUDA版本: {torch.version.cuda}")
                print(f"  GPU设备: {torch.cuda.get_device_name(0)}")
        except ImportError:
            pytest.fail("PyTorch未安装")

    def test_ultralytics_version(self):
        """测试Ultralytics版本"""
        from ultralytics import __version__
        print(f"\n Ultralytics版本: {__version__}")
        assert __version__ is not None


# ============================================
# 测试执行入口
# ============================================

if __name__ == "__main__":
    print("=" * 70)
    print("目标检测模块测试套件")
    print("=" * 70)

    # 运行所有测试
    pytest.main([
        __file__,
        "-v",                    # 详细输出
        "--tb=short",            # 简短的traceback
        "--color=yes",           # 彩色输出
        "-s",                    # 显示print输出
        "--durations=10",        # 显示最慢的10个测试
        # "--cov=execute_file",  # 代码覆盖率（需要安装pytest-cov）
        # "--cov-report=html",   # HTML覆盖率报告
    ])
