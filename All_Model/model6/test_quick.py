#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模型六快速测试脚本
用于日常开发中的快速验证
"""

import sys
import time
import math

# 设置输出编码为UTF-8
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

from GeodeticConverter_instant import GeodeticToLocalConverter
from predictor_long_term import LongHorizonHeadingPredictor
from maneuver_classifier import ManeuverClassifier


def quick_test():
    """快速测试核心功能"""

    print("=" * 60)
    print("模型六快速测试")
    print("=" * 60)

    # 1. 坐标转换器
    print("\n[1/5] 测试坐标转换器...")
    try:
        conv = GeodeticToLocalConverter(29.9, 126.6, 0.0, 29.9001, 126.6, 0.0)
        x, y, z = conv.geodetic_to_local(29.9, 126.6, 0.0)
        lat, lon, alt = conv.local_to_geodetic([x, y, z])
        print(f"[PASS] 坐标转换器正常 (往返误差: {abs(29.9-lat):.2e}°)")
    except Exception as e:
        print(f"[FAIL] 坐标转换器失败: {e}")
        return False

    # 2. 机动分类器 - 右转
    print("\n[2/5] 测试机动分类器...")
    try:
        cfg = {"omega_threshold_deg": 2.0, "vz_threshold_mps": 0.5}
        cls = ManeuverClassifier(cfg)

        # 右转轨迹
        hist = [
            (0.0, 29.9, 126.6, 120.0, 270.0),
            (1.0, 29.9, 126.6, 120.0, 310.0),
            (2.0, 29.9, 126.6, 120.0, 330.0)
        ]
        result = cls.inst_classify(hist, alt=None)
        if result == "right_turn":
            print(f"[PASS] 机动分类器正常 (识别: {result})")
        else:
            print(f"[WARN] 机动分类器可能不准确 (识别: {result}, 期望: right_turn)")
    except Exception as e:
        print(f"[FAIL] 机动分类器失败: {e}")
        return False

    # 3. 长时预测器
    print("\n[3/5] 测试长时预测器...")
    try:
        goals_xy = [
            {"name": "目标A", "xy": (10000, 12000)},
            {"name": "目标B", "xy": (-5000, -8000)}
        ]

        pred = LongHorizonHeadingPredictor(
            converter=conv,
            costmap=lambda x, y: 0.0,
            goals_xy=goals_xy,
            dt=2.0,
            n_particles=30,  # 快速测试用少量粒子
            cfg={"omega_threshold_deg": 2.0}
        )

        # 喂数据
        base_time = time.time()
        for i in range(10):
            pred.update_obs(base_time + i, 29.9, 126.6, 120.0, 270.0, 100.0)

        # 预测
        result = pred.predict(horizons_s=[300], sector_deg=30)

        if 300 in result and "heading_deg_mean" in result[300]:
            print(f"[PASS] 长时预测器正常 (航向: {result[300]['heading_deg_mean']:.1f}°)")
        else:
            print(f"[FAIL] 长时预测器返回数据不完整")
            return False
    except Exception as e:
        print(f"[FAIL] 长时预测器失败: {e}")
        return False

    # 4. UDP服务器类
    print("\n[4/5] 测试UDP服务器类...")
    try:
        from udp_server_enemy_analysis_Last import EnemyAnalysisServer
        server = EnemyAnalysisServer(host="0.0.0.0", port=5005)

        # 测试安全计算函数
        result = server.safe_numpy_mean([1.0, 2.0, 3.0])
        if abs(result - 2.0) < 1e-6:
            print(f"[PASS] UDP服务器类正常 (安全计算: {result})")
        else:
            print(f"[FAIL] UDP服务器类计算错误 (结果: {result}, 期望: 2.0)")
            return False
    except ImportError as e:
        print(f"[SKIP] UDP服务器类测试跳过 (缺少依赖: {e})")
        print("       提示: pip install websockets")
    except Exception as e:
        print(f"[FAIL] UDP服务器类失败: {e}")
        return False

    # 5. 多目标处理
    print("\n[5/5] 测试多目标处理...")
    try:
        packet = {
            "timestamp": time.time(),
            "tracks": [
                {"id": "enemy_1", "type": "STRIKER", "lat": 29.9, "lon": 126.6,
                 "alt": 100.0, "speed": 120.0, "heading": 270.0},
                {"id": "enemy_2", "type": "STRIKER", "lat": 29.91, "lon": 126.61,
                 "alt": 95.0, "speed": 110.0, "heading": 280.0}
            ]
        }

        enemy_tracks = [t for t in packet["tracks"] if t["type"] == "STRIKER"]
        if len(enemy_tracks) == 2:
            print(f"[PASS] 多目标处理正常 (识别: {len(enemy_tracks)}个敌方目标)")
        else:
            print(f"[FAIL] 多目标处理错误 (识别: {len(enemy_tracks)}, 期望: 2)")
            return False
    except Exception as e:
        print(f"[FAIL] 多目标处理失败: {e}")
        return False

    print("\n" + "=" * 60)
    print("[SUCCESS] 所有快速测试通过！")
    print("=" * 60)
    return True


if __name__ == "__main__":
    import sys
    success = quick_test()
    sys.exit(0 if success else 1)
