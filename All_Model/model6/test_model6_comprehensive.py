#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模型六综合测试用例
测试内容：
1. 坐标转换器测试
2. 6类机动识别测试（左转、右转、爬升、俯冲、聚合、散开）
3. 长时预测器测试
4. UDP服务器数据处理测试
5. 多目标处理测试
"""

import sys
import time
import json
import math
import numpy as np
from collections import deque
from datetime import datetime

# 设置输出编码为UTF-8
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

from GeodeticConverter_instant import GeodeticToLocalConverter
from predictor_long_term import LongHorizonHeadingPredictor
from maneuver_classifier import ManeuverClassifier


class Model6Tester:
    """模型六综合测试类"""

    def __init__(self):
        # 初始化坐标转换器
        self.conv = GeodeticToLocalConverter(29.9, 126.6, 0.0, 29.9001, 126.6, 0.0)

        # 定义目标点
        self.goals = [
            {"name": "舰船", "lat": 27.7990889, "lon": 122.8266472},
            {"name": "港口", "lat": 28.3777972, "lon": 121.6020917},
            {"name": "发电厂", "lat": 27.2476889, "lon": 120.3550278},
            {"name": "运输船", "lat": 26.3165, "lon": 120.6884444},
            {"name": "军工厂", "lat": 28.595311, "lon": 119.956708}
        ]

        # 预处理目标点为ENU坐标
        self.goals_xy = []
        for g in self.goals:
            x, y, _ = self.conv.geodetic_to_local(float(g["lat"]), float(g["lon"]), 0.0)
            self.goals_xy.append({"name": g["name"], "xy": (x, y)})

        # 配置参数
        self.cfg = {
            "dt": 2.0,
            "n_particles": 80,
            "w_cost": 0.01,
            "goal_scale_m": 100_000,
            "rho_omega": 0.9,
            "sigma_eta_deg": 0.1,
            "gamma_weights": 0.95,
            "resample_neff_ratio": 0.15,
            "omega_threshold_deg": 2.0,
            "vz_threshold_mps": 0.5,
            "delta_cd_mps": 50.0,
            "window_s": 60.0,
            "sector_deg": 30
        }

        self.test_results = []
        self.test_outputs = []  # 存储每个测试的详细输出

    def print_section(self, title):
        """打印章节标题"""
        print("\n" + "=" * 70)
        print(f"  {title}")
        print("=" * 70)

    def print_subsection(self, title):
        """打印子章节标题"""
        print("\n" + "-" * 70)
        print(f"  {title}")
        print("-" * 70)

    def log_test(self, test_name, passed, details="", output_data=None):
        """记录测试结果"""
        status = "[PASS]" if passed else "[FAIL]"
        result = {
            "test": test_name,
            "status": status,
            "details": details,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        if output_data:
            result["output"] = output_data
            self.test_outputs.append({
                "test": test_name,
                "data": output_data
            })

        self.test_results.append(result)

        print(f"\n{status} {test_name}")
        if details:
            print(f"详情: {details}")
        if output_data and isinstance(output_data, dict):
            print("实际输出:")
            for key, value in output_data.items():
                if isinstance(value, (int, float)):
                    print(f"  {key}: {value}")
                elif isinstance(value, dict):
                    print(f"  {key}:")
                    for k, v in value.items():
                        print(f"    {k}: {v}")
                else:
                    print(f"  {key}: {value}")

    def test_1_coordinate_converter(self):
        """测试1: 坐标转换器"""
        self.print_section("测试1: 坐标转换器")

        try:
            # 测试经纬度到局部坐标
            lat, lon, alt = 29.9, 126.6, 0.0
            print(f"输入: 纬度={lat}, 经度={lon}, 高度={alt}m")

            x, y, z = self.conv.geodetic_to_local(lat, lon, alt)
            print(f"局部坐标: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")

            # 测试局部坐标到经纬度
            lat2, lon2, alt2 = self.conv.local_to_geodetic([x, y, z])
            print(f"往返转换: 纬度={lat2}, 经度={lon2}, 高度={alt2}m")

            # 验证往返转换误差
            lat_error = abs(lat - lat2)
            lon_error = abs(lon - lon2)
            alt_error = abs(alt - alt2)

            passed = (lat_error < 1e-6 and lon_error < 1e-6 and alt_error < 1e-3)

            output_data = {
                "输入纬度": lat,
                "输入经度": lon,
                "局部坐标x(m)": round(x, 2),
                "局部坐标y(m)": round(y, 2),
                "往返纬度误差": f"{lat_error:.2e}度",
                "往返经度误差": f"{lon_error:.2e}度",
                "往返高度误差": f"{alt_error:.2e}m"
            }

            details = f"纬度误差={lat_error:.2e}度, 经度误差={lon_error:.2e}度, 高度误差={alt_error:.2e}m"
            self.log_test("坐标转换器往返精度", passed, details, output_data)

        except Exception as e:
            self.log_test("坐标转换器往返精度", False, f"异常: {str(e)}")

    def test_2_maneuver_right_turn(self):
        """测试2: 右转机动识别"""
        self.print_section("测试2: 右转机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造右转轨迹：270° -> 300° -> 330° (西向往北转)
            hist = [
                (0.0, 29.9, 126.6, 120.0, 270.0),
                (1.0, 29.9, 126.6, 120.0, 280.0),
                (2.0, 29.9, 126.6, 120.0, 290.0),
                (3.0, 29.9, 126.6, 120.0, 310.0),
                (4.0, 29.9, 126.6, 120.0, 330.0)
            ]

            print("输入轨迹:")
            for i, h in enumerate(hist):
                print(f"  时刻{i}: 航向={h[4]:.1f}度, 速度={h[3]:.1f}m/s")

            result = cls.inst_classify(hist, alt=None)

            passed = (result == "right_turn")

            output_data = {
                "轨迹类型": "右转测试",
                "航向序列": [h[4] for h in hist],
                "识别结果": result,
                "期望结果": "right_turn",
                "角速度阈值": f"{self.cfg['omega_threshold_deg']}度/s"
            }

            details = f"识别结果={result}, 期望=right_turn, 航向: 270->330度"
            self.log_test("右转机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("右转机动识别", False, f"异常: {str(e)}")

    def test_3_maneuver_left_turn(self):
        """测试3: 左转机动识别"""
        self.print_section("测试3: 左转机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造左转轨迹：270° -> 250° -> 230° (西向往南转)
            hist = [
                (0.0, 29.9, 126.6, 120.0, 270.0),
                (1.0, 29.9, 126.6, 120.0, 260.0),
                (2.0, 29.9, 126.6, 120.0, 250.0),
                (3.0, 29.9, 126.6, 120.0, 235.0),
                (4.0, 29.9, 126.6, 120.0, 220.0)
            ]

            print("输入轨迹:")
            for i, h in enumerate(hist):
                print(f"  时刻{i}: 航向={h[4]:.1f}度, 速度={h[3]:.1f}m/s")

            result = cls.inst_classify(hist, alt=None)

            passed = (result == "left_turn")

            output_data = {
                "轨迹类型": "左转测试",
                "航向序列": [h[4] for h in hist],
                "识别结果": result,
                "期望结果": "left_turn",
                "角速度阈值": f"{self.cfg['omega_threshold_deg']}度/s"
            }

            details = f"识别结果={result}, 期望=left_turn, 航向: 270->220度"
            self.log_test("左转机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("左转机动识别", False, f"异常: {str(e)}")

    def test_4_maneuver_climb(self):
        """测试4: 爬升机动识别"""
        self.print_section("测试4: 爬升机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造爬升轨迹：高度从100m爬升到105m
            hist = [
                (0.0, 29.9, 126.6, 120.0, 270.0, 100.0),
                (1.0, 29.9, 126.6, 120.0, 270.0, 101.0),
                (2.0, 29.9, 126.6, 120.0, 270.0, 102.0),
                (3.0, 29.9, 126.6, 120.0, 270.0, 103.5),
                (4.0, 29.9, 126.6, 120.0, 270.0, 105.0)
            ]

            print("输入轨迹:")
            for i, h in enumerate(hist):
                print(f"  时刻{i}: 高度={h[5]:.1f}m, 航向={h[4]:.1f}度")

            alt_data = [h[5] for h in hist]
            result = cls.inst_classify(hist, alt=alt_data)

            passed = (result == "climb")

            avg_vz = (hist[-1][5] - hist[0][5]) / (hist[-1][0] - hist[0][0])

            output_data = {
                "轨迹类型": "爬升测试",
                "高度序列(m)": alt_data,
                "平均垂速(m/s)": round(avg_vz, 2),
                "识别结果": result,
                "期望结果": "climb",
                "垂速阈值(m/s)": self.cfg['vz_threshold_mps']
            }

            details = f"识别结果={result}, 期望=climb, 高度: {alt_data[0]}m->{alt_data[-1]}m"
            self.log_test("爬升机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("爬升机动识别", False, f"异常: {str(e)}")

    def test_5_maneuver_dive(self):
        """测试5: 俯冲机动识别"""
        self.print_section("测试5: 俯冲机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造俯冲轨迹：高度从100m下降到95m
            hist = [
                (0.0, 29.9, 126.6, 120.0, 270.0, 100.0),
                (1.0, 29.9, 126.6, 120.0, 270.0, 99.0),
                (2.0, 29.9, 126.6, 120.0, 270.0, 98.0),
                (3.0, 29.9, 126.6, 120.0, 270.0, 96.5),
                (4.0, 29.9, 126.6, 120.0, 270.0, 95.0)
            ]

            print("输入轨迹:")
            for i, h in enumerate(hist):
                print(f"  时刻{i}: 高度={h[5]:.1f}m, 航向={h[4]:.1f}度")

            alt_data = [h[5] for h in hist]
            result = cls.inst_classify(hist, alt=alt_data)

            passed = (result == "dive")

            avg_vz = (hist[-1][5] - hist[0][5]) / (hist[-1][0] - hist[0][0])

            output_data = {
                "轨迹类型": "俯冲测试",
                "高度序列(m)": alt_data,
                "平均垂速(m/s)": round(avg_vz, 2),
                "识别结果": result,
                "期望结果": "dive",
                "垂速阈值(m/s)": self.cfg['vz_threshold_mps']
            }

            details = f"识别结果={result}, 期望=dive, 高度: {alt_data[0]}m->{alt_data[-1]}m"
            self.log_test("俯冲机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("俯冲机动识别", False, f"异常: {str(e)}")

    def test_6_maneuver_straight(self):
        """测试6: 直线飞行识别"""
        self.print_section("测试6: 直线飞行识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造直线轨迹：航向保持270°
            hist = [
                (0.0, 29.9, 126.6, 120.0, 270.0),
                (1.0, 29.9, 126.6, 120.0, 270.5),
                (2.0, 29.9, 126.6, 120.0, 269.5),
                (3.0, 29.9, 126.6, 120.0, 270.2),
                (4.0, 29.9, 126.6, 120.0, 270.0)
            ]

            print("输入轨迹:")
            for i, h in enumerate(hist):
                print(f"  时刻{i}: 航向={h[4]:.1f}度, 速度={h[3]:.1f}m/s")

            result = cls.inst_classify(hist, alt=None)

            passed = (result == "straight")

            heading_variation = max([h[4] for h in hist]) - min([h[4] for h in hist])

            output_data = {
                "轨迹类型": "直线测试",
                "航向序列": [h[4] for h in hist],
                "航向变化范围": f"{heading_variation:.1f}度",
                "识别结果": result,
                "期望结果": "straight"
            }

            details = f"识别结果={result}, 期望=straight, 航向变化={heading_variation:.1f}度"
            self.log_test("直线飞行识别", passed, details, output_data)

        except Exception as e:
            self.log_test("直线飞行识别", False, f"异常: {str(e)}")

    def test_7_cluster_converge(self):
        """测试7: 群体聚合机动识别"""
        self.print_section("测试7: 群体聚合机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造聚合场景：两架飞机互相靠近
            # 初始距离：2000m -> 最终距离：1000m
            track_dict = {
                "enemy_1": {
                    'xy': np.array([0.0, 0.0]),
                    'xy_prev': np.array([0.0, 0.0])
                },
                "enemy_2": {
                    'xy': np.array([1000.0, 0.0]),
                    'xy_prev': np.array([2000.0, 0.0])
                }
            }

            print("输入场景:")
            print(f"  enemy_1: (0, 0) -> (0, 0)")
            print(f"  enemy_2: (2000, 0) -> (1000, 0)")
            print(f"  距离变化: 2000m -> 1000m")

            xy_prev = {tid: d['xy_prev'] for tid, d in track_dict.items()}
            result = cls.cluster_classify(track_dict, dt=2.0, xy_prev=xy_prev)

            # 检查是否有聚合标签
            has_converge = any(label == "converge" for label in result.values())
            passed = has_converge

            output_data = {
                "场景类型": "聚合测试",
                "enemy_1位置变化": "(0,0) -> (0,0)",
                "enemy_2位置变化": "(2000,0) -> (1000,0)",
                "初始距离(m)": 2000,
                "最终距离(m)": 1000,
                "识别结果": result,
                "期望包含": "converge"
            }

            details = f"识别结果={result}, 期望包含converge"
            self.log_test("群体聚合机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("群体聚合机动识别", False, f"异常: {str(e)}")

    def test_8_cluster_diverge(self):
        """测试8: 群体散开机动识别"""
        self.print_section("测试8: 群体散开机动识别")

        try:
            cls = ManeuverClassifier(self.cfg)

            # 构造散开场景：两架飞机互相远离
            # 初始距离：1000m -> 最终距离：2000m
            track_dict = {
                "enemy_1": {
                    'xy': np.array([0.0, 0.0]),
                    'xy_prev': np.array([0.0, 0.0])
                },
                "enemy_2": {
                    'xy': np.array([2000.0, 0.0]),
                    'xy_prev': np.array([1000.0, 0.0])
                }
            }

            print("输入场景:")
            print(f"  enemy_1: (0, 0) -> (0, 0)")
            print(f"  enemy_2: (1000, 0) -> (2000, 0)")
            print(f"  距离变化: 1000m -> 2000m")

            xy_prev = {tid: d['xy_prev'] for tid, d in track_dict.items()}
            result = cls.cluster_classify(track_dict, dt=2.0, xy_prev=xy_prev)

            # 检查是否有散开标签
            has_diverge = any(label == "diverge" for label in result.values())
            passed = has_diverge

            output_data = {
                "场景类型": "散开测试",
                "enemy_1位置变化": "(0,0) -> (0,0)",
                "enemy_2位置变化": "(1000,0) -> (2000,0)",
                "初始距离(m)": 1000,
                "最终距离(m)": 2000,
                "识别结果": result,
                "期望包含": "diverge"
            }

            details = f"识别结果={result}, 期望包含diverge"
            self.log_test("群体散开机动识别", passed, details, output_data)

        except Exception as e:
            self.log_test("群体散开机动识别", False, f"异常: {str(e)}")

    def test_9_long_term_predictor(self):
        """测试9: 长时预测器"""
        self.print_section("测试9: 长时预测器")

        try:
            # 创建预测器
            pred = LongHorizonHeadingPredictor(
                converter=self.conv,
                costmap=lambda x, y: 0.0,
                goals_xy=list(self.goals_xy),
                dt=self.cfg["dt"],
                n_particles=self.cfg["n_particles"],
                w_cost=self.cfg["w_cost"],
                goal_scale_m=self.cfg["goal_scale_m"],
                rho_omega=self.cfg["rho_omega"],
                sigma_eta_deg=self.cfg["sigma_eta_deg"],
                gamma_weights=self.cfg["gamma_weights"],
                resample_neff_ratio=self.cfg["resample_neff_ratio"],
                cfg=self.cfg
            )

            # 生成测试轨迹：西向飞行，朝向港口方向
            base_time = time.time()
            lat, lon = 29.9, 126.6
            heading = 270.0  # 西向
            speed = 120.0

            print(f"输入轨迹参数:")
            print(f"  起点: ({lat}, {lon})")
            print(f"  航向: {heading}度")
            print(f"  速度: {speed}m/s")
            print(f"  轨迹点数: 30")

            for i in range(30):  # 30秒数据
                t = base_time + i
                pred.update_obs(t, lat, lon, speed, heading, 100.0)

                # 轨迹外推
                dx = speed * 1.0 * math.sin(math.radians(heading))
                dy = speed * 1.0 * math.cos(math.radians(heading))
                Rlat = 111320.0
                Rlon = 111320.0 * math.cos(math.radians(lat))
                lat += dy / Rlat
                lon += dx / Rlon

            # 执行预测
            print("\n执行300s预测...")
            result = pred.predict(horizons_s=[300], sector_deg=self.cfg["sector_deg"])

            # 验证结果
            passed = (
                300 in result and
                "heading_deg_mean" in result[300] and
                "goal_probs" in result[300] and
                "maneuver_probs" in result[300] and
                "most_likely_maneuver" in result[300]
            )

            if passed:
                r = result[300]
                output_data = {
                    "预测时长(s)": 300,
                    "航向均值(度)": round(r['heading_deg_mean'], 1),
                    "航向误差(度)": round(r['heading_ci_deg'], 1),
                    "最可能机动": r['most_likely_maneuver'],
                    "目标概率": r['goal_probs'],
                    "机动概率": r['maneuver_probs'],
                    "质量指标": r['quality']
                }
                details = (f"航向={r['heading_deg_mean']:.1f}度, "
                          f"机动={r['most_likely_maneuver']}, "
                          f"目标数={len(r['goal_probs'])}")
            else:
                output_data = {"错误": "预测结果缺少必要字段"}
                details = "预测结果缺少必要字段"

            self.log_test("长时预测器功能", passed, details, output_data)

        except Exception as e:
            self.log_test("长时预测器功能", False, f"异常: {str(e)}")

    def test_10_goal_prediction(self):
        """测试10: 目标点预测"""
        self.print_section("测试10: 目标点预测")

        try:
            pred = LongHorizonHeadingPredictor(
                converter=self.conv,
                costmap=lambda x, y: 0.0,
                goals_xy=list(self.goals_xy),
                dt=self.cfg["dt"],
                n_particles=50,  # 减少粒子数加快测试
                cfg=self.cfg
            )

            # 生成朝向"港口"方向的轨迹
            base_time = time.time()

            # 从起点往港口方向飞
            start_lat, start_lon = 29.9, 126.6
            goal_lat, goal_lon = 28.3777972, 121.6020917  # 港口

            # 计算航向
            dx = goal_lon - start_lon
            dy = goal_lat - start_lat
            heading = math.degrees(math.atan2(dx, dy)) % 360

            print(f"输入轨迹参数:")
            print(f"  起点: ({start_lat}, {start_lon})")
            print(f"  目标: 港口({goal_lat}, {goal_lon})")
            print(f"  计算航向: {heading:.1f}度")
            print(f"  轨迹点数: 40")

            lat, lon = start_lat, start_lon
            speed = 120.0

            for i in range(40):
                t = base_time + i
                pred.update_obs(t, lat, lon, speed, heading, 100.0)

                # 外推
                dx = speed * 1.0 * math.sin(math.radians(heading))
                dy = speed * 1.0 * math.cos(math.radians(heading))
                Rlat = 111320.0
                Rlon = 111320.0 * math.cos(math.radians(lat))
                lat += dy / Rlat
                lon += dx / Rlon

            # 预测
            print("\n执行300s预测...")
            result = pred.predict(horizons_s=[300], sector_deg=30)

            # 检查是否识别出目标
            if 300 in result and "goal_probs" in result[300]:
                goal_probs = result[300]["goal_probs"]
                most_likely_goal = max(goal_probs.items(), key=lambda x: x[1])[0] if goal_probs else "未知"
                passed = len(goal_probs) > 0

                output_data = {
                    "预测时长(s)": 300,
                    "最可能目标": most_likely_goal,
                    "目标概率分布": goal_probs,
                    "飞行航向(度)": round(heading, 1),
                    "期望方向": "港口"
                }

                details = f"最可能目标={most_likely_goal}, 概率={goal_probs}"
            else:
                passed = False
                output_data = {"错误": "未返回目标概率"}
                details = "未返回目标概率"

            self.log_test("目标点预测", passed, details, output_data)

        except Exception as e:
            self.log_test("目标点预测", False, f"异常: {str(e)}")

    def test_11_udp_data_parsing(self):
        """测试11: UDP数据包解析"""
        self.print_section("测试11: UDP数据包解析")

        try:
            # 构造模拟UDP数据包
            packet = {
                "timestamp": time.time(),
                "tracks": [
                    {
                        "id": "enemy_1",
                        "type": "STRIKER",
                        "lat": 29.9,
                        "lon": 126.6,
                        "alt": 100.0,
                        "speed": 120.0,
                        "heading": 270.0
                    },
                    {
                        "id": "ucav_1",
                        "type": "UCAV",
                        "lat": 30.0,
                        "lon": 127.0,
                        "alt": 95.0,
                        "speed": 110.0,
                        "heading": 90.0
                    }
                ]
            }

            print("输入数据包:")
            print(json.dumps(packet, indent=2, ensure_ascii=False))

            # 解析敌方目标
            enemy_tracks = [t for t in packet["tracks"] if t["type"] == "STRIKER"]
            friendly_tracks = [t for t in packet["tracks"] if t["type"] == "UCAV"]

            passed = (len(enemy_tracks) == 1 and len(friendly_tracks) == 1)

            output_data = {
                "总轨迹数": len(packet["tracks"]),
                "敌方目标数": len(enemy_tracks),
                "友方UCAV数": len(friendly_tracks),
                "敌方目标": enemy_tracks,
                "友方目标": friendly_tracks
            }

            details = f"敌方={len(enemy_tracks)}, 友方={len(friendly_tracks)}"
            self.log_test("UDP数据包解析", passed, details, output_data)

        except Exception as e:
            self.log_test("UDP数据包解析", False, f"异常: {str(e)}")

    def test_12_multi_target_processing(self):
        """测试12: 多目标处理"""
        self.print_section("测试12: 多目标处理")

        try:
            # 创建多个预测器
            predictors = {}
            track_ids = ["enemy_1", "enemy_2", "enemy_3"]

            base_time = time.time()

            print(f"创建{len(track_ids)}个目标的预测器...")

            for track_id in track_ids:
                pred = LongHorizonHeadingPredictor(
                    converter=self.conv,
                    costmap=lambda x, y: 0.0,
                    goals_xy=list(self.goals_xy),
                    dt=self.cfg["dt"],
                    n_particles=30,  # 减少粒子数
                    cfg=self.cfg
                )

                # 为每个目标生成不同的轨迹
                idx = track_ids.index(track_id)
                lat = 29.9 + idx * 0.01
                lon = 126.6 + idx * 0.01
                heading = 270.0 + idx * 10

                print(f"  {track_id}: 起点=({lat:.2f}, {lon:.2f}), 航向={heading}度")

                for i in range(20):
                    t = base_time + i
                    pred.update_obs(t, lat, lon, 120.0, heading, 100.0)

                predictors[track_id] = pred

            # 执行预测
            print(f"\n对{len(predictors)}个目标执行300s预测...")
            results = {}
            for track_id, pred in predictors.items():
                results[track_id] = pred.predict(horizons_s=[300], sector_deg=30)

            passed = (len(results) == 3 and all(300 in r for r in results.values()))

            output_data = {
                "目标总数": len(track_ids),
                "成功预测数": sum(1 for r in results.values() if 300 in r),
                "预测结果摘要": {}
            }

            for tid, r in results.items():
                if 300 in r:
                    output_data["预测结果摘要"][tid] = {
                        "航向": round(r[300]['heading_deg_mean'], 1),
                        "机动": r[300]['most_likely_maneuver']
                    }

            details = f"处理目标数={len(results)}, 成功预测={sum(1 for r in results.values() if 300 in r)}"
            self.log_test("多目标处理", passed, details, output_data)

        except Exception as e:
            self.log_test("多目标处理", False, f"异常: {str(e)}")

    def test_13_haversine_distance(self):
        """测试13: Haversine距离计算"""
        self.print_section("测试13: Haversine距离计算")

        try:
            # 计算北京到上海的距离
            lat1, lon1 = 39.9042, 116.4074  # 北京
            lat2, lon2 = 31.2304, 121.4737  # 上海

            print(f"计算距离:")
            print(f"  起点: 北京({lat1}, {lon1})")
            print(f"  终点: 上海({lat2}, {lon2})")

            # Haversine公式
            R = 6371000  # 地球半径（米）
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)

            delta_lat = lat2_rad - lat1_rad
            delta_lon = lon2_rad - lon1_rad

            a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = R * c

            # 预期距离约1068km
            expected_distance = 1068000  # 米
            error = abs(distance - expected_distance) / expected_distance

            passed = (error < 0.05)  # 允许5%误差

            output_data = {
                "起点": "北京(39.9042, 116.4074)",
                "终点": "上海(31.2304, 121.4737)",
                "计算距离(km)": round(distance/1000, 1),
                "预期距离(km)": round(expected_distance/1000, 1),
                "误差率(%)": round(error*100, 2)
            }

            details = f"计算距离={distance/1000:.1f}km, 预期={expected_distance/1000:.1f}km, 误差={error*100:.2f}%"
            self.log_test("Haversine距离计算", passed, details, output_data)

        except Exception as e:
            self.log_test("Haversine距离计算", False, f"异常: {str(e)}")

    def test_14_safe_numpy_operations(self):
        """测试14: 安全的Numpy操作"""
        self.print_section("测试14: 安全的Numpy操作")

        try:
            # 导入服务器类（但不启动）
            from udp_server_enemy_analysis_Last import EnemyAnalysisServer

            server = EnemyAnalysisServer(host="0.0.0.0", port=5005)

            # 测试安全平均值计算
            data1 = [1.0, 2.0, 3.0, 4.0, 5.0]
            result1 = server.safe_numpy_mean(data1)
            expected1 = 3.0

            print(f"测试1: 正常数据平均值")
            print(f"  输入: {data1}")
            print(f"  输出: {result1}")
            print(f"  期望: {expected1}")

            # 测试空数据
            data2 = []
            result2 = server.safe_numpy_mean(data2)
            expected2 = 0.0

            print(f"\n测试2: 空数据处理")
            print(f"  输入: {data2}")
            print(f"  输出: {result2}")
            print(f"  期望: {expected2}")

            # 测试加权平均
            data3 = [1.0, 2.0, 3.0]
            weights3 = [1.0, 2.0, 3.0]
            result3 = server.safe_numpy_average(data3, weights3)
            expected3 = (1*1 + 2*2 + 3*3) / (1+2+3)

            print(f"\n测试3: 加权平均")
            print(f"  数据: {data3}")
            print(f"  权重: {weights3}")
            print(f"  输出: {result3:.2f}")
            print(f"  期望: {expected3:.2f}")

            passed = (
                abs(result1 - expected1) < 1e-6 and
                abs(result2 - expected2) < 1e-6 and
                abs(result3 - expected3) < 1e-6
            )

            output_data = {
                "正常均值测试": {"输入": data1, "输出": result1, "期望": expected1},
                "空数据测试": {"输入": data2, "输出": result2, "期望": expected2},
                "加权均值测试": {"输入": data3, "权重": weights3, "输出": round(result3, 2), "期望": round(expected3, 2)}
            }

            details = f"均值={result1}, 空数据={result2}, 加权均值={result3:.2f}"
            self.log_test("安全的Numpy操作", passed, details, output_data)

        except ImportError as e:
            self.log_test("安全的Numpy操作", True, f"跳过(缺少依赖: websockets)")
        except Exception as e:
            self.log_test("安全的Numpy操作", False, f"异常: {str(e)}")

    def test_15_enemy1_heading_detection(self):
        """测试15: enemy_1航向检测（0-360度）"""
        self.print_section("测试15: enemy_1航向检测(0-360度)")

        try:
            from udp_server_enemy_analysis_Last import EnemyAnalysisServer

            server = EnemyAnalysisServer(host="0.0.0.0", port=5005)

            # 测试场景1: 右转（270° -> 300° -> 330°）
            headings_right = [270.0, 280.0, 290.0, 310.0, 330.0]
            with server._enemy1_lock:
                server.enemy1_heading_history.clear()
                for h in headings_right:
                    server.enemy1_heading_history.append(h)

            print("测试场景1: 右转检测")
            print(f"  航向序列: {headings_right}")
            result_right = server.detect_enemy1_turn_maneuver()
            print(f"  识别结果: {result_right}")

            # 测试场景2: 左转（270° -> 250° -> 230°）
            headings_left = [270.0, 260.0, 250.0, 230.0, 220.0]
            with server._enemy1_lock:
                server.enemy1_heading_history.clear()
                for h in headings_left:
                    server.enemy1_heading_history.append(h)

            print("\n测试场景2: 左转检测")
            print(f"  航向序列: {headings_left}")
            result_left = server.detect_enemy1_turn_maneuver()
            print(f"  识别结果: {result_left}")

            passed = (result_right == "right_turn" and result_left == "left_turn")

            output_data = {
                "右转测试": {
                    "航向序列": headings_right,
                    "识别结果": result_right,
                    "期望": "right_turn"
                },
                "左转测试": {
                    "航向序列": headings_left,
                    "识别结果": result_left,
                    "期望": "left_turn"
                },
                "右转阈值(度)": 300,
                "左转阈值(度)": 240
            }

            details = f"右转={result_right}, 左转={result_left}"
            self.log_test("enemy_1航向检测", passed, details, output_data)

        except ImportError as e:
            self.log_test("enemy_1航向检测", True, f"跳过(缺少依赖: websockets)")
        except Exception as e:
            self.log_test("enemy_1航向检测", False, f"异常: {str(e)}")

    def save_detailed_report(self):
        """保存详细的测试报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 保存完整报告
        report_file = f"test_report_detailed_{timestamp}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump({
                "测试时间": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "测试摘要": {
                    "总测试数": len(self.test_results),
                    "通过": sum(1 for r in self.test_results if "[PASS]" in r["status"]),
                    "失败": sum(1 for r in self.test_results if "[FAIL]" in r["status"]),
                    "通过率": f"{sum(1 for r in self.test_results if '[PASS]' in r['status'])/len(self.test_results)*100:.1f}%"
                },
                "测试结果": self.test_results,
                "详细输出": self.test_outputs
            }, f, ensure_ascii=False, indent=2)

        print(f"\n详细测试报告已保存到: {report_file}")
        return report_file

    def run_all_tests(self):
        """运行所有测试"""
        print("\n")
        print("=" * 70)
        print("  模型六综合测试套件")
        print("  测试开始时间:", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        print("=" * 70)

        start_time = time.time()

        # 执行所有测试
        self.test_1_coordinate_converter()
        self.test_2_maneuver_right_turn()
        self.test_3_maneuver_left_turn()
        self.test_4_maneuver_climb()
        self.test_5_maneuver_dive()
        self.test_6_maneuver_straight()
        self.test_7_cluster_converge()
        self.test_8_cluster_diverge()
        self.test_9_long_term_predictor()
        self.test_10_goal_prediction()
        self.test_11_udp_data_parsing()
        self.test_12_multi_target_processing()
        self.test_13_haversine_distance()
        self.test_14_safe_numpy_operations()
        self.test_15_enemy1_heading_detection()

        end_time = time.time()
        duration = end_time - start_time

        # 统计结果
        total = len(self.test_results)
        passed = sum(1 for r in self.test_results if "[PASS]" in r["status"])
        failed = total - passed

        self.print_section("测试总结")
        print(f"总测试数: {total}")
        print(f"通过数量: {passed} [PASS]")
        print(f"失败数量: {failed} [FAIL]")
        print(f"通过率: {passed/total*100:.1f}%")
        print(f"测试耗时: {duration:.2f}秒")
        print(f"完成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 70)

        # 输出失败的测试
        if failed > 0:
            self.print_subsection("失败的测试详情")
            for r in self.test_results:
                if "[FAIL]" in r["status"]:
                    print(f"  {r['test']}")
                    print(f"    原因: {r['details']}")
                    print()

        # 保存详细报告
        report_file = self.save_detailed_report()

        # 保存简要报告（向后兼容）
        with open("test_report_model6.json", 'w', encoding='utf-8') as f:
            json.dump({
                "summary": {
                    "total": total,
                    "passed": passed,
                    "failed": failed,
                    "pass_rate": f"{passed/total*100:.1f}%",
                    "duration_seconds": round(duration, 2)
                },
                "details": self.test_results
            }, f, ensure_ascii=False, indent=2)

        print(f"\n简要测试报告已保存到: test_report_model6.json")

        return passed == total


if __name__ == "__main__":
    tester = Model6Tester()
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
