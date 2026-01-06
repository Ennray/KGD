import unittest
import numpy as np
from shapely.geometry import Point, Polygon
import position_setting


class TestGenerateIrregularPolygon(unittest.TestCase):
    """测试生成不规则多边形功能"""

    def test_normal_radii(self):
        """测试正常半径输入"""
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        polygon = position_setting.generate_irregular_polygon(radii.copy())
        self.assertIsInstance(polygon, Polygon)
        self.assertTrue(polygon.is_valid)
        self.assertFalse(polygon.is_empty)

    def test_uniform_radii(self):
        """测试均匀半径（接近圆形）"""
        radii = [2500] * 8
        polygon = position_setting.generate_irregular_polygon(radii.copy())
        self.assertIsInstance(polygon, Polygon)
        self.assertTrue(polygon.is_valid)

    def test_varying_radii(self):
        """测试变化较大的半径"""
        radii = [2000, 2800, 2200, 2900, 2100, 2700, 2300, 2600]
        polygon = position_setting.generate_irregular_polygon(radii.copy())
        self.assertIsInstance(polygon, Polygon)
        self.assertTrue(polygon.is_valid)


class TestFirstSet(unittest.TestCase):
    """测试第一批次无人机位置设置"""

    def test_normal_detection(self):
        """测试正常探测参数"""
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        detection = [900, 600]
        first_points, first_num, rects, center = position_setting.first_set(radii, detection)

        self.assertIsInstance(first_points, list)
        self.assertGreater(first_num, 0)
        self.assertEqual(len(first_points), first_num)
        self.assertEqual(len(rects), first_num)
        self.assertIsInstance(center, Point)

        # 检查每个点都是三维坐标
        for point in first_points:
            self.assertEqual(len(point), 3)
            self.assertEqual(point[1], 0)  # y坐标应该为0

    def test_small_detection(self):
        """测试较小的探测尺寸（需要更多无人机）"""
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        detection = [500, 400]
        first_points, first_num, rects, center = position_setting.first_set(radii, detection)

        self.assertGreater(first_num, 0)
        self.assertEqual(len(first_points), first_num)

    def test_large_detection(self):
        """测试较大的探测尺寸（需要较少无人机）"""
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        detection = [1200, 800]
        first_points, first_num, rects, center = position_setting.first_set(radii, detection)

        self.assertGreater(first_num, 0)
        self.assertEqual(len(first_points), first_num)


class TestTotalNum(unittest.TestCase):
    """测试尾后探测无人机数量计算"""

    def test_normal_enemy_number(self):
        """测试正常敌机数量"""
        enemy_number = 350
        detection = [240, 160]
        total = position_setting.total_num(enemy_number, detection)

        self.assertIsInstance(total, int)
        self.assertGreater(total, 0)
        # 验证计算逻辑：total = ceil(enemy_number * 100^2 / detection[0]^2)
        expected = np.ceil(enemy_number * (100**2) / (detection[0]**2))
        self.assertEqual(total, expected)

    def test_small_enemy_number(self):
        """测试较少敌机数量"""
        enemy_number = 100
        detection = [240, 160]
        total = position_setting.total_num(enemy_number, detection)

        self.assertGreater(total, 0)

    def test_large_enemy_number(self):
        """测试较多敌机数量"""
        enemy_number = 1000
        detection = [240, 160]
        total = position_setting.total_num(enemy_number, detection)

        self.assertGreater(total, 0)

    def test_different_detections(self):
        """测试不同探测尺寸"""
        enemy_number = 350
        detection1 = [240, 160]
        detection2 = [480, 320]  # 更大的探测范围

        total1 = position_setting.total_num(enemy_number, detection1)
        total2 = position_setting.total_num(enemy_number, detection2)

        # 探测范围更大时，需要的无人机应该更少
        self.assertLess(total2, total1)


class TestSecondSet(unittest.TestCase):
    """测试第二批次无人机位置设置"""

    def test_normal_second_set(self):
        """测试正常第二批次设置"""
        # 先生成第一批次数据
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        detection = [900, 600]
        first_points, first_num, rects, first_center = position_setting.first_set(radii, detection)

        # 计算总数
        enemy_number = 350
        clear_detection = [240, 160]
        total = position_setting.total_num(enemy_number, clear_detection)

        # 设置第二批次
        interval = 500
        t = 5
        speed = 240
        second_points, second_num = position_setting.second_set(
            first_center, first_points, first_num, total, interval, t, speed
        )

        self.assertIsInstance(second_points, list)
        self.assertGreaterEqual(second_num, 0)
        self.assertEqual(second_num, total - first_num)

        # 如果有第二批次无人机，检查数量匹配
        if second_num > 0:
            self.assertEqual(len(second_points), second_num)

            # 检查每个点都是三维坐标
            for point in second_points:
                self.assertEqual(len(point), 3)

    def test_second_set_with_different_speeds(self):
        """测试不同速度下的第二批次设置"""
        radii = [2500, 2502, 2504, 2501, 2502, 2501, 2500, 2503]
        detection = [900, 600]
        first_points, first_num, rects, first_center = position_setting.first_set(radii, detection)

        total = position_setting.total_num(350, [240, 160])
        interval = 500
        t = 5

        # 不同速度
        for speed in [200, 240, 300]:
            second_points, second_num = position_setting.second_set(
                first_center, first_points, first_num, total, interval, t, speed
            )
            self.assertGreaterEqual(second_num, 0)


class TestPositionAdjust(unittest.TestCase):
    """测试阵位调整功能"""

    def setUp(self):
        """设置测试数据"""
        self.first_points = [(0, 0, 0), (100, 0, 100), (200, 0, 200)]
        self.second_points = [(0, -500, 0), (100, -500, 100), (200, -500, 200)]

    def test_type_1_no_adjustment(self):
        """测试类型1：中心点重合，无需调整"""
        new_first, new_second = position_setting.position_adjust(
            self.first_points.copy(), self.second_points.copy(), type=1
        )

        # 类型1不做调整
        self.assertEqual(new_first, self.first_points)
        self.assertEqual(new_second, self.second_points)

    def test_type_2_adjustment(self):
        """测试类型2：需要调整"""
        dist = [500, 0, 200]
        new_first, new_second = position_setting.position_adjust(
            self.first_points.copy(), self.second_points.copy(), type=2, dist=dist
        )

        # 检查是否正确调整
        for i, point in enumerate(new_first):
            expected = (
                self.first_points[i][0] - dist[0],
                self.first_points[i][1] - dist[1],
                self.first_points[i][2] - dist[2]
            )
            self.assertEqual(point, expected)

    def test_type_3_adjustment(self):
        """测试类型3：需要调整"""
        dist = [300, 100, 150]
        new_first, new_second = position_setting.position_adjust(
            self.first_points.copy(), self.second_points.copy(), type=3, dist=dist
        )

        # 检查是否正确调整
        for i, point in enumerate(new_first):
            expected = (
                self.first_points[i][0] - dist[0],
                self.first_points[i][1] - dist[1],
                self.first_points[i][2] - dist[2]
            )
            self.assertEqual(point, expected)


class TestConverter(unittest.TestCase):
    """测试坐标转换功能"""

    def test_coordinate_conversion(self):
        """测试坐标转换"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy = ["28:13:34.95W", "10:30:31.24N", "50.0"]
        first_uavs = [(0, 0, 0), (100, 0, 100)]

        result = position_setting.Converter(enemy, basepoint, first_uavs)

        self.assertIsInstance(result, list)
        self.assertEqual(len(result), len(first_uavs))

        # 检查每个结果都是列表格式（经纬度和高度）
        for coord in result:
            self.assertIsInstance(coord, list)
            self.assertEqual(len(coord), 3)  # 经度、纬度、高度

    def test_conversion_with_multiple_points(self):
        """测试多个点的坐标转换"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy = ["28:13:34.95W", "10:30:31.24N", "50.0"]
        first_uavs = [(i*100, 0, i*100) for i in range(10)]

        result = position_setting.Converter(enemy, basepoint, first_uavs)

        self.assertEqual(len(result), len(first_uavs))


class TestInitPositionSet(unittest.TestCase):
    """测试完整的初始阵位设置流程"""

    def test_normal_init(self):
        """测试正常的初始化流程"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy_approx = ["28:13:34.95W", "10:30:31.24N", "50.0"]
        radii = [2500, 2350, 2450, 2500, 2400, 2400, 2480, 2380]
        min_detect = [900, 600]
        clear_detect = [240, 160]
        enemy_number = 350
        interval = 500
        t = 5
        speed = 240

        first_num, first_uavs, second_num, second_uavs, rects, first_center = \
            position_setting.init_position_set(
                basepoint, enemy_approx, radii, min_detect, clear_detect,
                enemy_number, interval, t, speed
            )

        # 验证返回值类型和有效性
        self.assertIsInstance(first_num, int)
        self.assertGreater(first_num, 0)
        self.assertIsInstance(first_uavs, list)
        self.assertEqual(len(first_uavs), first_num)

        self.assertIsInstance(second_num, int)
        self.assertGreaterEqual(second_num, 0)
        self.assertIsInstance(second_uavs, list)
        self.assertEqual(len(second_uavs), second_num)

        self.assertIsInstance(rects, list)
        self.assertIsInstance(first_center, Point)

        # 验证总数合理性
        total_expected = position_setting.total_num(enemy_number, clear_detect)
        self.assertEqual(first_num + second_num, total_expected)

    def test_init_with_different_parameters(self):
        """测试不同参数的初始化"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy_approx = ["28:13:34.95W", "10:30:31.24N", "50.0"]

        # 测试参数组合1：较少敌机
        radii1 = [2500] * 8
        first_num1, first_uavs1, second_num1, second_uavs1, _, _ = \
            position_setting.init_position_set(
                basepoint, enemy_approx, radii1, [900, 600], [240, 160],
                100, 500, 5, 240
            )
        self.assertGreater(first_num1, 0)

        # 测试参数组合2：较多敌机
        radii2 = [2500] * 8
        first_num2, first_uavs2, second_num2, second_uavs2, _, _ = \
            position_setting.init_position_set(
                basepoint, enemy_approx, radii2, [900, 600], [240, 160],
                1000, 500, 5, 240
            )
        self.assertGreater(first_num2, 0)

        # 敌机数量更多时，总无人机数应该更多
        total1 = first_num1 + second_num1
        total2 = first_num2 + second_num2
        self.assertLess(total1, total2)

    def test_init_with_different_speeds(self):
        """测试不同速度参数"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy_approx = ["28:13:34.95W", "10:30:31.24N", "50.0"]
        base_radii = [2500, 2350, 2450, 2500, 2400, 2400, 2480, 2380]

        for speed in [200, 240, 300]:
            radii = base_radii.copy()  # 每次循环使用新副本
            first_num, first_uavs, second_num, second_uavs, _, _ = \
                position_setting.init_position_set(
                    basepoint, enemy_approx, radii, [900, 600], [240, 160],
                    350, 500, 5, speed
                )
            self.assertGreater(first_num, 0)
            self.assertGreaterEqual(second_num, 0)

    def test_init_with_different_intervals(self):
        """测试不同间距参数"""
        basepoint = ["28:11:34.95W", "10:30:35.24N", "55.0"]
        enemy_approx = ["28:13:34.95W", "10:30:31.24N", "50.0"]
        base_radii = [2500, 2350, 2450, 2500, 2400, 2400, 2480, 2380]

        for interval in [400, 500, 600]:
            radii = base_radii.copy()  # 每次循环使用新副本
            first_num, first_uavs, second_num, second_uavs, _, _ = \
                position_setting.init_position_set(
                    basepoint, enemy_approx, radii, [900, 600], [240, 160],
                    350, interval, 5, 240
                )
            self.assertGreater(first_num, 0)
            self.assertGreaterEqual(second_num, 0)


if __name__ == '__main__':
    # 运行所有测试
    unittest.main(verbosity=2)
