import unittest
import json
import sys
import os
from types import ModuleType

# 将当前目录添加到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 修复导入问题：创建一个model4模块，其中包含position_setting
position_setting_module = __import__('position_setting')
model4 = ModuleType('model4')
model4.position_setting = position_setting_module
sys.modules['model4'] = model4

from test import app


class TestPosiSetAPI(unittest.TestCase):
    """测试 /posiset 接口"""

    def setUp(self):
        """设置测试环境"""
        self.app = app.test_client()
        self.app.testing = True

    def test_posiset_with_default_parameters(self):
        """测试使用默认参数"""
        response = self.app.post('/posiset',
                                  data=json.dumps({}),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 验证返回字段
        self.assertIn('first_num', data)
        self.assertIn('first_uavs', data)
        self.assertIn('second_num', data)
        self.assertIn('second_uavs', data)

        # 验证数据类型
        self.assertIsInstance(data['first_num'], int)
        self.assertIsInstance(data['first_uavs'], list)
        self.assertIsInstance(data['second_num'], int)
        self.assertIsInstance(data['second_uavs'], list)

        # 验证数量匹配
        self.assertEqual(len(data['first_uavs']), data['first_num'])
        self.assertEqual(len(data['second_uavs']), data['second_num'])

    def test_posiset_with_custom_parameters(self):
        """测试使用自定义参数"""
        payload = {
            "basepoint": ["28:11:34.95W", "10:30:35.24N", "55.0"],
            "enemy_approx": ["28:13:34.95W", "10:30:31.24N", "50.0"],
            "radii": [2500, 2350, 2450, 2500, 2400, 2400, 2480, 2380],
            "min_detect": [900, 600],
            "clear_detect": [240, 160],
            "enemy_number": 350,
            "interval": 500,
            "time": 5,
            "enemy_speed": 240
        }

        response = self.app.post('/posiset',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 验证返回字段
        self.assertIn('first_num', data)
        self.assertIn('first_uavs', data)
        self.assertIn('second_num', data)
        self.assertIn('second_uavs', data)

        # 验证无人机数量为正数
        self.assertGreater(data['first_num'], 0)
        self.assertGreaterEqual(data['second_num'], 0)

    def test_posiset_with_small_enemy_number(self):
        """测试较少敌机数量"""
        payload = {
            "enemy_number": 100,
            "radii": [2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
        }

        response = self.app.post('/posiset',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        self.assertGreater(data['first_num'], 0)

    def test_posiset_with_large_enemy_number(self):
        """测试较多敌机数量"""
        payload = {
            "enemy_number": 1000,
            "radii": [2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
        }

        response = self.app.post('/posiset',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        self.assertGreater(data['first_num'], 0)
        self.assertGreater(data['second_num'], 0)

    def test_posiset_with_different_radii(self):
        """测试不同的探测半径"""
        # 测试较小半径
        payload1 = {
            "radii": [2000, 2100, 2050, 2000, 2100, 2050, 2000, 2100],
            "enemy_number": 350
        }

        response1 = self.app.post('/posiset',
                                   data=json.dumps(payload1),
                                   content_type='application/json')

        self.assertEqual(response1.status_code, 200)
        data1 = json.loads(response1.data)

        # 测试较大半径
        payload2 = {
            "radii": [3000, 3100, 3050, 3000, 3100, 3050, 3000, 3100],
            "enemy_number": 350
        }

        response2 = self.app.post('/posiset',
                                   data=json.dumps(payload2),
                                   content_type='application/json')

        self.assertEqual(response2.status_code, 200)
        data2 = json.loads(response2.data)

        # 半径更大时，第一批次无人机数量更多
        self.assertLess(data1['first_num'], data2['first_num'])

    def test_posiset_with_different_speeds(self):
        """测试不同的敌机速度"""
        for speed in [200, 240, 300, 360]:
            payload = {
                "enemy_speed": speed,
                "enemy_number": 350
            }

            response = self.app.post('/posiset',
                                      data=json.dumps(payload),
                                      content_type='application/json')

            self.assertEqual(response.status_code, 200)
            data = json.loads(response.data)
            self.assertGreater(data['first_num'], 0)

    def test_posiset_with_different_intervals(self):
        """测试不同的无人机间距"""
        for interval in [400, 500, 600, 700]:
            payload = {
                "interval": interval,
                "enemy_number": 350
            }

            response = self.app.post('/posiset',
                                      data=json.dumps(payload),
                                      content_type='application/json')

            self.assertEqual(response.status_code, 200)
            data = json.loads(response.data)
            self.assertGreater(data['first_num'], 0)

    def test_posiset_with_different_detection_sizes(self):
        """测试不同的探测尺寸"""
        # 较小探测尺寸
        payload1 = {
            "min_detect": [600, 400],
            "clear_detect": [200, 150],
            "enemy_number": 350
        }

        response1 = self.app.post('/posiset',
                                   data=json.dumps(payload1),
                                   content_type='application/json')

        self.assertEqual(response1.status_code, 200)
        data1 = json.loads(response1.data)

        # 较大探测尺寸
        payload2 = {
            "min_detect": [1200, 800],
            "clear_detect": [300, 200],
            "enemy_number": 350
        }

        response2 = self.app.post('/posiset',
                                   data=json.dumps(payload2),
                                   content_type='application/json')

        self.assertEqual(response2.status_code, 200)
        data2 = json.loads(response2.data)

        # 探测尺寸更大时，需要的无人机应该更少
        self.assertGreater(data1['first_num'], data2['first_num'])

    def test_posiset_coordinates_format(self):
        """测试坐标格式正确性"""
        response = self.app.post('/posiset',
                                  data=json.dumps({}),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 检查返回的坐标格式（应该是列表格式的地理坐标）
        for uav_coord in data['first_uavs']:
            self.assertIsInstance(uav_coord, list)
            self.assertEqual(len(uav_coord), 3)  # 经度、纬度、高度

        for uav_coord in data['second_uavs']:
            self.assertIsInstance(uav_coord, list)
            self.assertEqual(len(uav_coord), 3)  # 经度、纬度、高度


class TestPosiAdjustAPI(unittest.TestCase):
    """测试 /posiAdjust 接口"""

    def setUp(self):
        """设置测试环境"""
        self.app = app.test_client()
        self.app.testing = True

        # 先获取初始阵位
        response = self.app.post('/posiset',
                                  data=json.dumps({}),
                                  content_type='application/json')
        self.initial_data = json.loads(response.data)

    def test_adjust_with_type_1(self):
        """测试类型1调整（无需调整）"""
        payload = {
            "first_uavs": [(0, 0, 0), (100, 0, 100)],
            "second_uavs": [(0, -500, 0), (100, -500, 100)],
            "type": 1
        }

        response = self.app.post('/posiAdjust',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 验证返回字段
        self.assertIn('first_uavs', data)
        self.assertIn('second_uavs', data)

        # 类型1应该不做调整，但返回的是列表格式
        # 验证坐标值相同（不比较数据类型tuple vs list）
        self.assertEqual(len(data['first_uavs']), len(payload['first_uavs']))
        self.assertEqual(len(data['second_uavs']), len(payload['second_uavs']))

        for i in range(len(data['first_uavs'])):
            self.assertEqual(list(data['first_uavs'][i]), list(payload['first_uavs'][i]))

        for i in range(len(data['second_uavs'])):
            self.assertEqual(list(data['second_uavs'][i]), list(payload['second_uavs'][i]))

    def test_adjust_with_type_2(self):
        """测试类型2调整"""
        payload = {
            "first_uavs": [(0, 0, 0), (100, 0, 100), (200, 0, 200)],
            "second_uavs": [(0, -500, 0), (100, -500, 100), (200, -500, 200)],
            "type": 2
        }

        response = self.app.post('/posiAdjust',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 验证返回字段
        self.assertIn('first_uavs', data)
        self.assertIn('second_uavs', data)

        # 验证数据类型
        self.assertIsInstance(data['first_uavs'], list)
        self.assertIsInstance(data['second_uavs'], list)

        # 验证数量不变
        self.assertEqual(len(data['first_uavs']), len(payload['first_uavs']))
        self.assertEqual(len(data['second_uavs']), len(payload['second_uavs']))

    def test_adjust_with_type_3(self):
        """测试类型3调整"""
        payload = {
            "first_uavs": [(0, 0, 0), (100, 0, 100), (200, 0, 200)],
            "second_uavs": [(0, -500, 0), (100, -500, 100), (200, -500, 200)],
            "type": 3
        }

        response = self.app.post('/posiAdjust',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        # 验证返回字段
        self.assertIn('first_uavs', data)
        self.assertIn('second_uavs', data)

        # 验证数据类型
        self.assertIsInstance(data['first_uavs'], list)
        self.assertIsInstance(data['second_uavs'], list)

        # 验证数量不变
        self.assertEqual(len(data['first_uavs']), len(payload['first_uavs']))
        self.assertEqual(len(data['second_uavs']), len(payload['second_uavs']))

    def test_adjust_with_empty_uavs(self):
        """测试空无人机列表"""
        payload = {
            "first_uavs": [],
            "second_uavs": [],
            "type": 1
        }

        response = self.app.post('/posiAdjust',
                                  data=json.dumps(payload),
                                  content_type='application/json')

        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)

        self.assertEqual(data['first_uavs'], [])
        self.assertEqual(data['second_uavs'], [])

    def test_adjust_all_types(self):
        """测试所有调整类型"""
        base_payload = {
            "first_uavs": [(0, 0, 0), (100, 0, 100), (200, 0, 200)],
            "second_uavs": [(0, -500, 0), (100, -500, 100)]
        }

        for adjust_type in [1, 2, 3]:
            payload = base_payload.copy()
            payload['type'] = adjust_type

            response = self.app.post('/posiAdjust',
                                      data=json.dumps(payload),
                                      content_type='application/json')

            self.assertEqual(response.status_code, 200)
            data = json.loads(response.data)

            # 所有类型都应该返回相同数量的无人机
            self.assertEqual(len(data['first_uavs']), len(payload['first_uavs']))
            self.assertEqual(len(data['second_uavs']), len(payload['second_uavs']))


class TestAPIIntegration(unittest.TestCase):
    """测试API集成场景"""

    def setUp(self):
        """设置测试环境"""
        self.app = app.test_client()
        self.app.testing = True

    def test_full_workflow(self):
        """测试完整工作流程：初始化 -> 调整"""
        # 步骤1：初始化阵位
        init_payload = {
            "enemy_number": 350,
            "radii": [2500, 2350, 2450, 2500, 2400, 2400, 2480, 2380]
        }

        init_response = self.app.post('/posiset',
                                       data=json.dumps(init_payload),
                                       content_type='application/json')

        self.assertEqual(init_response.status_code, 200)
        init_data = json.loads(init_response.data)

        # 使用初始化的结果进行调整（模拟场景)
        adjust_payload = {
            "first_uavs": [(i*100, 0, i*100) for i in range(min(init_data['first_num'], 5))],
            "second_uavs": [(i*100, -500, i*100) for i in range(min(init_data['second_num'], 5))],
            "type": 2
        }

        adjust_response = self.app.post('/posiAdjust',
                                         data=json.dumps(adjust_payload),
                                         content_type='application/json')

        self.assertEqual(adjust_response.status_code, 200)
        adjust_data = json.loads(adjust_response.data)

        # 验证调整后的数据
        self.assertIsInstance(adjust_data['first_uavs'], list)
        self.assertIsInstance(adjust_data['second_uavs'], list)

    def test_multiple_posiset_calls(self):
        """测试多次调用 /posiset 接口"""
        payload = {
            "enemy_number": 350
        }

        # 多次调用应该返回一致的结果（相同输入）
        response1 = self.app.post('/posiset',
                                   data=json.dumps(payload),
                                   content_type='application/json')
        data1 = json.loads(response1.data)

        response2 = self.app.post('/posiset',
                                   data=json.dumps(payload),
                                   content_type='application/json')
        data2 = json.loads(response2.data)

        # 相同参数应该产生相同数量的无人机
        self.assertEqual(data1['first_num'], data2['first_num'])
        self.assertEqual(data1['second_num'], data2['second_num'])

    def test_different_scenarios(self):
        """测试不同作战场景"""
        scenarios = [
            {
                "name": "小规模敌群",
                "payload": {"enemy_number": 100, "radii": [2000]*8}
            },
            {
                "name": "中规模敌群",
                "payload": {"enemy_number": 350, "radii": [2500]*8}
            },
            {
                "name": "大规模敌群",
                "payload": {"enemy_number": 800, "radii": [3000]*8}
            },
            {
                "name": "高速敌群",
                "payload": {"enemy_number": 350, "enemy_speed": 360}
            },
            {
                "name": "低速敌群",
                "payload": {"enemy_number": 350, "enemy_speed": 180}
            }
        ]

        for scenario in scenarios:
            with self.subTest(scenario=scenario["name"]):
                response = self.app.post('/posiset',
                                          data=json.dumps(scenario["payload"]),
                                          content_type='application/json')

                self.assertEqual(response.status_code, 200)
                data = json.loads(response.data)

                # 所有场景都应该能够成功计算阵位
                self.assertGreater(data['first_num'], 0)
                # second_num可能为0或负数（当第一批次就够用时）
                # 但无人机列表应该对应
                self.assertEqual(len(data['first_uavs']), data['first_num'])
                self.assertEqual(len(data['second_uavs']), max(0, data['second_num']))


if __name__ == '__main__':
    # 运行所有测试
    unittest.main(verbosity=2)
