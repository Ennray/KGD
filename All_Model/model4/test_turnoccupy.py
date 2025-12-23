"""
Model4 TurnOccupy 模块测试用例
包含多个不同规模和场景的测试
"""

import TurnOccupy
import json
import time


def test_case_1_minimal():
    """
    测试用例1: 最小规模测试（3架第1波次 + 2架第2波次，单纵队）
    目的：快速验证基本功能
    """
    print("=" * 80)
    print("测试用例 1: 最小规模测试")
    print("=" * 80)

    config = {
        # 基础位置
        "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
        "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"],
        "enemy_lonrange": ["126:08:00.00E", "126:09:30.00E"],
        "enemy_latrange": ["26:34:00.00N", "26:35:00.00N"],

        # 速度参数
        "maximum_speed": 500,
        "minimum_speed": 200,
        "speed": 240,
        "acceleration": 80,

        # 探测参数
        "near_detect_distance": [900, 600],
        "detect_distance": 30000,

        # 敌群参数
        "radii": [2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500],
        "enemy_number": 100,

        # 第1波次（3架）
        "first_num": 3,
        "first_uavs": [
            ["125:4:52.52E", "26:36:48.33N", "5856.38"],
            ["125:4:52.25E", "26:36:48.33N", "4956.42"],
            ["125:4:51.98E", "26:36:48.34N", "4056.45"]
        ],

        # 第2波次（2架，单纵队）
        "column": 1,
        "num_of_column": 2,
        "num_per_column": [2],
        "second_num": 2,
        "second_uavs": [
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"]
        ],
        "second_groups": [
            [
                ["125:4:39.41E", "26:38:10.29N", "5065.61"],
                ["125:4:21.38E", "26:38:11.22N", "5069.82"]
            ]
        ],
        "second_groups_counts": [2],
        "second_num_1": 2,
        "second_uavs_1": [
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"]
        ]
    }

    try:
        print("\n开始执行...")
        start_time = time.time()
        result = TurnOccupy.main(config)
        end_time = time.time()

        print(f"\n[SUCCESS] 测试通过!")
        print(f"执行时间: {end_time - start_time:.2f} 秒")
        print(f"返回数据字段: {list(result.keys())}")
        print(f"第1波次转弯时间: {result.get('first_uav_turn_time', 'N/A')} 秒")
        print(f"第2波次转弯时间: {result.get('second_uav_turn_time', 'N/A')} 秒")

        return True, result
    except Exception as e:
        print(f"\n[FAILED] 测试失败!")
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def test_case_2_medium():
    """
    测试用例2: 中等规模测试（10架第1波次 + 10架第2波次，2个纵队）
    目的：验证多纵队协同
    """
    print("\n" + "=" * 80)
    print("测试用例 2: 中等规模测试")
    print("=" * 80)

    config = {
        "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
        "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"],
        "enemy_lonrange": ["126:06:19.94E", "126:11:10.06E"],
        "enemy_latrange": ["26:32:21.25N", "26:36:40.71N"],

        "maximum_speed": 500,
        "minimum_speed": 200,
        "speed": 240,
        "acceleration": 80,
        "near_detect_distance": [900, 600],
        "detect_distance": 30000,
        "radii": [2500, 2350, 2550, 2500, 2400, 2400, 2480, 2380],
        "enemy_number": 200,

        # 第1波次（10架）
        "first_num": 10,
        "first_uavs": [
            ["125:4:52.52E", "26:36:48.33N", "5856.38"],
            ["125:4:52.25E", "26:36:48.33N", "4956.42"],
            ["125:4:51.98E", "26:36:48.34N", "4056.45"],
            ["125:4:51.71E", "26:36:48.34N", "3156.48"],
            ["125:4:53.70E", "26:37:4.54N", "6335.56"],
            ["125:4:53.43E", "26:37:4.54N", "5435.59"],
            ["125:4:53.16E", "26:37:4.54N", "4535.63"],
            ["125:4:52.89E", "26:37:4.55N", "3635.66"],
            ["125:4:52.62E", "26:37:4.55N", "2735.69"],
            ["125:4:54.82E", "26:37:20.73N", "6629.78"]
        ],

        # 第2波次（10架，2个纵队，每个5架）
        "column": 2,
        "num_of_column": 5,
        "num_per_column": [5, 5],
        "second_num": 10,
        "second_uavs": [
            # 第1纵队
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"],
            ["125:4:3.34E", "26:38:12.14N", "5074.08"],
            ["125:3:45.31E", "26:38:13.07N", "5078.37"],
            ["125:3:27.28E", "26:38:13.99N", "5082.70"],
            # 第2纵队
            ["125:4:2.31E", "26:37:55.94N", "5073.21"],
            ["125:3:44.28E", "26:37:56.86N", "5077.50"],
            ["125:3:26.25E", "26:37:57.79N", "5081.84"],
            ["125:3:8.22E", "26:37:58.71N", "5086.21"],
            ["125:2:50.18E", "26:37:59.63N", "5090.62"]
        ],
        "second_groups": [
            [
                ["125:4:39.41E", "26:38:10.29N", "5065.61"],
                ["125:4:21.38E", "26:38:11.22N", "5069.82"],
                ["125:4:3.34E", "26:38:12.14N", "5074.08"],
                ["125:3:45.31E", "26:38:13.07N", "5078.37"],
                ["125:3:27.28E", "26:38:13.99N", "5082.70"]
            ],
            [
                ["125:4:2.31E", "26:37:55.94N", "5073.21"],
                ["125:3:44.28E", "26:37:56.86N", "5077.50"],
                ["125:3:26.25E", "26:37:57.79N", "5081.84"],
                ["125:3:8.22E", "26:37:58.71N", "5086.21"],
                ["125:2:50.18E", "26:37:59.63N", "5090.62"]
            ]
        ],
        "second_groups_counts": [5, 5],
        "second_num_1": 5,
        "second_uavs_1": [
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"],
            ["125:4:3.34E", "26:38:12.14N", "5074.08"],
            ["125:3:45.31E", "26:38:13.07N", "5078.37"],
            ["125:3:27.28E", "26:38:13.99N", "5082.70"]
        ],
        "second_num_2": 5,
        "second_uavs_2": [
            ["125:4:2.31E", "26:37:55.94N", "5073.21"],
            ["125:3:44.28E", "26:37:56.86N", "5077.50"],
            ["125:3:26.25E", "26:37:57.79N", "5081.84"],
            ["125:3:8.22E", "26:37:58.71N", "5086.21"],
            ["125:2:50.18E", "26:37:59.63N", "5090.62"]
        ]
    }

    try:
        print("\n开始执行...")
        start_time = time.time()
        result = TurnOccupy.main(config)
        end_time = time.time()

        print(f"\n[SUCCESS] 测试通过!")
        print(f"执行时间: {end_time - start_time:.2f} 秒")
        print(f"第1波次无人机数: {len(result.get('first_init_point', []))}")
        print(f"第2波次无人机数: {len(result.get('second_init_point', []))}")
        print(f"最后提前转弯无人机数: {len(result.get('last_init_point', []))}")

        return True, result
    except Exception as e:
        print(f"\n[FAILED] 测试失败!")
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def test_case_3_default():
    """
    测试用例3: 默认数据集测试（完整规模）
    目的：验证完整功能
    """
    print("\n" + "=" * 80)
    print("测试用例 3: 默认数据集测试（完整规模）")
    print("=" * 80)

    try:
        print("\n开始执行（使用默认数据）...")
        start_time = time.time()
        result = TurnOccupy.main()  # 不传参数，使用默认 dataset
        end_time = time.time()

        print(f"\n[SUCCESS] 测试通过!")
        print(f"执行时间: {end_time - start_time:.2f} 秒")
        print(f"第1波次无人机数: {len(result.get('first_init_point', []))}")
        print(f"第2波次无人机数: {len(result.get('second_init_point', []))}")
        print(f"最后提前转弯无人机数: {len(result.get('last_init_point', []))}")
        print(f"相对位置记录数: {len(result.get('relative_position', []))}")

        return True, result
    except Exception as e:
        print(f"\n[FAILED] 测试失败!")
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def test_case_4_high_speed():
    """
    测试用例4: 高速场景测试
    目的：测试高速情况下的算法表现
    """
    print("\n" + "=" * 80)
    print("测试用例 4: 高速场景测试")
    print("=" * 80)

    config = {
        "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
        "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"],
        "enemy_lonrange": ["126:06:19.94E", "126:11:10.06E"],
        "enemy_latrange": ["26:32:21.25N", "26:36:40.71N"],

        # 高速参数
        "maximum_speed": 800,  # 更高的最大速度
        "minimum_speed": 300,  # 更高的最小速度
        "speed": 400,           # 更高的敌机速度
        "acceleration": 150,    # 更大的加速度

        "near_detect_distance": [900, 600],
        "detect_distance": 30000,
        "radii": [2500, 2350, 2550, 2500, 2400, 2400, 2480, 2380],
        "enemy_number": 150,

        "first_num": 5,
        "first_uavs": [
            ["125:4:52.52E", "26:36:48.33N", "5856.38"],
            ["125:4:52.25E", "26:36:48.33N", "4956.42"],
            ["125:4:51.98E", "26:36:48.34N", "4056.45"],
            ["125:4:51.71E", "26:36:48.34N", "3156.48"],
            ["125:4:53.70E", "26:37:4.54N", "6335.56"]
        ],

        "column": 1,
        "num_of_column": 5,
        "num_per_column": [5],
        "second_num": 5,
        "second_uavs": [
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"],
            ["125:4:3.34E", "26:38:12.14N", "5074.08"],
            ["125:3:45.31E", "26:38:13.07N", "5078.37"],
            ["125:3:27.28E", "26:38:13.99N", "5082.70"]
        ],
        "second_groups": [
            [
                ["125:4:39.41E", "26:38:10.29N", "5065.61"],
                ["125:4:21.38E", "26:38:11.22N", "5069.82"],
                ["125:4:3.34E", "26:38:12.14N", "5074.08"],
                ["125:3:45.31E", "26:38:13.07N", "5078.37"],
                ["125:3:27.28E", "26:38:13.99N", "5082.70"]
            ]
        ],
        "second_groups_counts": [5],
        "second_num_1": 5,
        "second_uavs_1": [
            ["125:4:39.41E", "26:38:10.29N", "5065.61"],
            ["125:4:21.38E", "26:38:11.22N", "5069.82"],
            ["125:4:3.34E", "26:38:12.14N", "5074.08"],
            ["125:3:45.31E", "26:38:13.07N", "5078.37"],
            ["125:3:27.28E", "26:38:13.99N", "5082.70"]
        ]
    }

    try:
        print("\n开始执行...")
        start_time = time.time()
        result = TurnOccupy.main(config)
        end_time = time.time()

        print(f"\n[SUCCESS] 测试通过!")
        print(f"执行时间: {end_time - start_time:.2f} 秒")
        print(f"第1波次转弯时间: {result.get('first_uav_turn_time', 'N/A')} 秒")
        print(f"高速场景下的追击时间: {result.get('first_uav_chase_time', ['N/A'])[0] if result.get('first_uav_chase_time') else 'N/A'} 秒")

        return True, result
    except Exception as e:
        print(f"\n[FAILED] 测试失败!")
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def save_result_to_file(test_name, result):
    """保存测试结果到JSON文件"""
    if result:
        filename = f"test_result_{test_name}.json"
        try:
            # 将 numpy 类型转换为 Python 原生类型
            def convert_to_native(obj):
                import numpy as np
                if isinstance(obj, np.integer):
                    return int(obj)
                elif isinstance(obj, np.floating):
                    return float(obj)
                elif isinstance(obj, np.ndarray):
                    return obj.tolist()
                elif isinstance(obj, dict):
                    return {k: convert_to_native(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [convert_to_native(i) for i in obj]
                return obj

            result_native = convert_to_native(result)

            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(result_native, f, indent=2, ensure_ascii=False)
            print(f"结果已保存到: {filename}")
        except Exception as e:
            print(f"保存结果失败: {e}")


def main():
    """运行所有测试用例"""
    print("\n")
    print("=" * 80)
    print("Model4 TurnOccupy 模块测试套件".center(80))
    print("=" * 80)
    print("\n包含4个测试用例:")
    print("  1. 最小规模测试 - 快速验证基本功能")
    print("  2. 中等规模测试 - 验证多纵队协同")
    print("  3. 默认数据集测试 - 验证完整功能")
    print("  4. 高速场景测试 - 测试极限参数")
    print()

    results = []

    # 运行所有测试
    success_1, result_1 = test_case_1_minimal()
    results.append(("最小规模", success_1))
    if success_1:
        save_result_to_file("case1_minimal", result_1)

    success_2, result_2 = test_case_2_medium()
    results.append(("中等规模", success_2))
    if success_2:
        save_result_to_file("case2_medium", result_2)

    success_3, result_3 = test_case_3_default()
    results.append(("默认数据集", success_3))
    if success_3:
        save_result_to_file("case3_default", result_3)

    success_4, result_4 = test_case_4_high_speed()
    results.append(("高速场景", success_4))
    if success_4:
        save_result_to_file("case4_high_speed", result_4)

    # 总结
    print("\n" + "=" * 80)
    print("测试总结")
    print("=" * 80)

    passed = sum(1 for _, success in results if success)
    total = len(results)

    for name, success in results:
        status = "[PASS]" if success else "[FAIL]"
        print(f"{status} {name}测试")

    print(f"\n通过率: {passed}/{total} ({passed*100//total}%)")

    if passed == total:
        print("\n[SUCCESS] 所有测试通过!")
    else:
        print(f"\n[WARNING] 有 {total - passed} 个测试失败")

    print("=" * 80)


if __name__ == "__main__":
    main()
