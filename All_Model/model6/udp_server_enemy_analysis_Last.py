#!/usr/bin/env python3
# udp_server_enemy_analysis_simple.py - 完全修复numpy警告版本
import socket
import json
import time
import numpy as np
from collections import defaultdict, deque
from GeodeticConverter_instant import GeodeticToLocalConverter
from predictor_long_term import LongHorizonHeadingPredictor
from maneuver_classifier import ManeuverClassifier
import math
import threading
import warnings
import websockets
import asyncio


class EnemyAnalysisServer:
    """完全修复numpy警告版本"""

    def __init__(self, host="0.0.0.0", port=5005, websocket_port=7070):
        self.host = host
        self.port = port
        self.websocket_port = websocket_port
        self.websocket_clients = set()

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

        # 优化配置参数
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

        # 数据缓冲区
        self.buffers = defaultdict(lambda: deque(maxlen=200))
        self.predictors = {}

        # enemy_1历史数据缓存
        self.enemy1_history = deque(maxlen=50)
        self.enemy1_heading_history = deque(maxlen=50)

        # 群体分析历史数据
        self._prev_positions = {}
        self._cluster_distance_history = deque(maxlen=12)
        self._cluster_trend_history = deque(maxlen=8)  # 存储趋势变化

        # 全局数据缓存
        self._base_data_cache = {
            "friendly_group": None,
            "enemy_group": None,
            "last_update_time": 0
        }

        self._prediction_cache = {
            "maneuver": "",
            "target_goal": "未知目标",
            "last_update_time": 0
        }

        # 线程锁
        self._base_data_lock = threading.Lock()
        self._prediction_lock = threading.Lock()
        self._buffer_lock = threading.Lock()
        self._websocket_lock = threading.Lock()
        self._enemy1_lock = threading.Lock()

        # 线程控制
        self._running = True
        self._packet_counter = 0

        print(" 完全修复numpy警告版本服务器初始化完成")

    async def websocket_handler(self, websocket, path=None):
        """WebSocket连接处理器"""
        with self._websocket_lock:
            self.websocket_clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            with self._websocket_lock:
                self.websocket_clients.discard(websocket)

    def start_websocket_server(self):
        """启动WebSocket服务器"""

        def run_websocket_server():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

                async def start_server():
                    return await websockets.serve(
                        self.websocket_handler,
                        "0.0.0.0",
                        self.websocket_port
                    )

                server = loop.run_until_complete(start_server())

                async def send_worker():
                    last_send_time = 0
                    while self._running:
                        try:
                            current_time = time.time()
                            if current_time - last_send_time >= 0.5:
                                # 从全局缓存读取数据
                                with self._base_data_lock:
                                    friendly_data = self._base_data_cache["friendly_group"]
                                    enemy_base_data = self._base_data_cache["enemy_group"]

                                with self._prediction_lock:
                                    maneuver = self._prediction_cache["maneuver"]
                                    target_goal = self._prediction_cache["target_goal"]

                                # 构建发送数据
                                broadcast_data = {
                                    "friendly_group": friendly_data,
                                    "enemy_group": None
                                }

                                # 构建敌方群体数据
                                if enemy_base_data:
                                    broadcast_data["enemy_group"] = {
                                        "lat": enemy_base_data["lat"],
                                        "lon": enemy_base_data["lon"],
                                        "alt": enemy_base_data["alt"],
                                        "speed": enemy_base_data["speed"],
                                        "heading": enemy_base_data["heading"],
                                        "maneuver": self.get_maneuver_chinese(maneuver),
                                        "target_goal": target_goal
                                    }

                                # 发送数据
                                if broadcast_data["friendly_group"] or broadcast_data["enemy_group"]:
                                    message = json.dumps(broadcast_data, ensure_ascii=False)

                                    with self._websocket_lock:
                                        clients_to_remove = []
                                        clients_to_send = list(self.websocket_clients)

                                        send_count = 0
                                        for client in clients_to_send:
                                            try:
                                                await asyncio.wait_for(client.send(message), timeout=0.1)
                                                send_count += 1
                                            except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed,
                                                    Exception):
                                                clients_to_remove.append(client)

                                        for client in clients_to_remove:
                                            if client in self.websocket_clients:
                                                self.websocket_clients.remove(client)

                                        if send_count > 0 and maneuver:
                                            print(f" 发送: {self.get_maneuver_chinese(maneuver)}->{target_goal}")

                                    last_send_time = current_time

                            await asyncio.sleep(0.01)
                        except Exception:
                            await asyncio.sleep(0.1)

                loop.create_task(send_worker())
                loop.run_forever()
            except Exception as e:
                print(f"WebSocket服务器错误: {e}")

        websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
        websocket_thread.start()
        return websocket_thread

    def update_obs(self, track_id, t, lat, lon, spd, heading_deg, alt=None):
        """更新观测数据"""
        with self._buffer_lock:
            if alt is not None:
                self.buffers[track_id].append(
                    (float(t), float(lat), float(lon), float(spd), float(heading_deg), float(alt)))
            else:
                self.buffers[track_id].append((float(t), float(lat), float(lon), float(spd), float(heading_deg)))

            if track_id == "enemy_1":
                with self._enemy1_lock:
                    if alt is not None:
                        self.enemy1_history.append(
                            (float(t), float(lat), float(lon), float(spd), float(heading_deg), float(alt)))
                    else:
                        self.enemy1_history.append((float(t), float(lat), float(lon), float(spd), float(heading_deg)))
                    self.enemy1_heading_history.append(float(heading_deg))

    def _get_predictor(self, track_id):
        """获取长时预测器"""
        if track_id not in self.predictors:
            self.predictors[track_id] = LongHorizonHeadingPredictor(
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
        return self.predictors[track_id]

    def predict_long_term(self, track_id):
        """长时预测"""
        with self._buffer_lock:
            if track_id not in self.buffers or not self.buffers[track_id]:
                return None

        pred = self._get_predictor(track_id)

        with self._buffer_lock:
            t_now = self.buffers[track_id][-1][0]
            for item in self.buffers[track_id]:
                if t_now - item[0] <= self.cfg["window_s"]:
                    if len(item) == 6:
                        pred.update_obs(item[0], item[1], item[2], item[3], item[4], item[5])
                    else:
                        pred.update_obs(item[0], item[1], item[2], item[3], item[4])

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = pred.predict(horizons_s=[300], sector_deg=self.cfg["sector_deg"])
            return result.get(300) or {}
        except Exception:
            return {}

    def safe_numpy_mean(self, data):
        """安全的numpy平均值计算"""
        if not data or len(data) == 0:
            return 0.0
        try:
            arr = np.array(data)
            if len(arr) == 0 or not np.isfinite(arr).all():
                return 0.0
            result = np.mean(arr)
            return result if np.isfinite(result) else 0.0
        except (ValueError, RuntimeWarning):
            return 0.0

    def safe_numpy_average(self, data, weights):
        """安全的numpy加权平均计算"""
        if not data or len(data) == 0:
            return 0.0
        try:
            arr = np.array(data)
            w_arr = np.array(weights)
            if len(arr) == 0 or not np.isfinite(arr).all() or not np.isfinite(w_arr).all():
                return 0.0
            if np.sum(w_arr) <= 0:
                return np.mean(arr) if len(arr) > 0 else 0.0
            result = np.average(arr, weights=w_arr)
            return result if np.isfinite(result) else 0.0
        except (ValueError, RuntimeWarning):
            return 0.0

    def detect_enemy1_turn_maneuver(self):
        """检测enemy_1的转向机动 - 直接使用0-360度航向"""
        with self._enemy1_lock:
            if len(self.enemy1_heading_history) < 3:
                print(f" 数据不足: {len(self.enemy1_heading_history)}个航向点")
                return ""

            headings = list(self.enemy1_heading_history)

            #  当前航向本来就是0-360度
            current_heading = headings[-1] if headings else 0
            print(f" 当前航向: {current_heading:.1f}°")
            print(f" 最近航向数据: {[f'{h:.1f}°' for h in headings[-8:]]}")

            # 阈值设置
            right_turn_threshold = 300.0  # 大于300度就是右转（往北）
            left_turn_threshold = 240.0  # 小于240度就是左转（往南）

            # 检查最近3个点的航向
            recent_headings = headings[-3:]

            print(f" 西向基准: 270°, 右转阈值: >{right_turn_threshold}°, 左转阈值: <{left_turn_threshold}°")
            print(f" 检查点航向: {[f'{h:.1f}°' for h in recent_headings]}")

            # 统计符合转向条件的点数
            right_turn_count = 0
            left_turn_count = 0

            for i, current_heading in enumerate(recent_headings):
                if current_heading > right_turn_threshold:  # 大于300度 → 右转（往北）
                    right_turn_count += 1
                    print(f"   点{i + 1}: {current_heading:.1f}° > {right_turn_threshold}° → 符合右转")
                elif current_heading < left_turn_threshold:  # 小于240度 → 左转（往南）
                    left_turn_count += 1
                    print(f"   点{i + 1}: {current_heading:.1f}° < {left_turn_threshold}° → 符合左转")
                else:
                    print(f"   点{i + 1}: {current_heading:.1f}° → 直线飞行")

            # 判断转向（需要连续2个点符合条件）
            if right_turn_count >= 2:
                print(f" 检测到右转: {right_turn_count}个点航向 > {right_turn_threshold}°")
                return "right_turn"
            elif left_turn_count >= 2:
                print(f" 检测到左转: {left_turn_count}个点航向 < {left_turn_threshold}°")
                return "left_turn"
            else:
                print(f" 未检测到转向: 右转计数{right_turn_count}, 左转计数{left_turn_count}")
                return ""

    def detect_enemy1_vertical_maneuver(self):
        """检测enemy_1的垂直机动 - 完全修复numpy警告"""
        with self._enemy1_lock:
            if len(self.enemy1_history) < 3:
                return ""

            latest_data = self.enemy1_history[-1]
            if len(latest_data) < 6:
                return ""

            altitudes = [data[5] for data in list(self.enemy1_history) if len(data) >= 6]
            timestamps = [data[0] for data in list(self.enemy1_history) if len(data) >= 6]

            if len(altitudes) < 2:
                return ""

            vertical_changes = []
            for i in range(1, len(altitudes)):
                dt = timestamps[i] - timestamps[i - 1]
                if dt > 0:
                    vertical_changes.append((altitudes[i] - altitudes[i - 1]) / dt)

            #  完全修复：使用安全计算函数
            if len(vertical_changes) == 0:
                return ""

            avg_vz = self.safe_numpy_mean(vertical_changes)

            if avg_vz > self.cfg["vz_threshold_mps"]:
                return "climb"
            elif avg_vz < -self.cfg["vz_threshold_mps"]:
                return "dive"
            else:
                return ""

        return ""

    def classify_enemy1_maneuver(self):
        """分类enemy_1的机动类型"""
        turn_maneuver = self.detect_enemy1_turn_maneuver()
        if turn_maneuver:
            return turn_maneuver

        vertical_maneuver = self.detect_enemy1_vertical_maneuver()
        if vertical_maneuver:
            return vertical_maneuver

        return ""

    def calculate_cluster_distance(self, targets):
        """计算群体内所有目标之间的平均距离"""
        if len(targets) < 2:
            return 0

        positions = []
        for target_data in targets.values():
            lat = target_data["position"]["lat"]
            lon = target_data["position"]["lon"]
            positions.append((lat, lon))

        total_distance = 0
        count = 0
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                distance = self.haversine_distance(positions[i][0], positions[i][1],
                                                   positions[j][0], positions[j][1])
                total_distance += distance
                count += 1

        return total_distance / count if count > 0 else 0

    def classify_cluster_maneuver_enhanced(self, enemy_targets_data):
        """基于当前状态的聚散检测 - 1.3倍散开、0.7倍聚合"""
        if len(enemy_targets_data) < 2:
            return ""

        try:
            current_distance = self.calculate_cluster_distance(enemy_targets_data)
            self._cluster_distance_history.append(current_distance)

            # 需要足够的历史数据来判断趋势
            if len(self._cluster_distance_history) < 4:
                return ""

            distances = list(self._cluster_distance_history)

            #  基于当前状态：计算最近的变化趋势
            recent_changes = []
            for i in range(1, min(4, len(distances))):
                if distances[-i - 1] > 10:  # 避免除零
                    change_ratio = distances[-i] / distances[-i - 1]
                    if np.isfinite(change_ratio):
                        recent_changes.append(change_ratio)

            if len(recent_changes) < 2:
                return ""

            #  使用安全计算
            avg_change = self.safe_numpy_mean(recent_changes)

            if not np.isfinite(avg_change) or avg_change <= 0:
                return ""

            #print(f" 当前间距: {current_distance:.1f}m, 平均变化: {avg_change:.3f}x")

            #  1.3倍散开阈值、0.7倍聚合阈值
            diverge_threshold = 1.3  # 1.3倍算散开
            converge_threshold = 0.7  # 0.7倍算聚合

            if avg_change > diverge_threshold:
                print(f" 当前状态散开: 平均变化{avg_change:.3f}倍 (> {diverge_threshold}x)")
                return "diverge"
            elif avg_change < converge_threshold:
                print(f" 当前状态聚合: 平均变化{avg_change:.3f}倍 (< {converge_threshold}x)")
                return "converge"

            return ""
        except Exception:
            return ""

    def classify_cluster_maneuver(self, enemy_targets_data):
        """综合聚散检测"""
        if len(enemy_targets_data) < 2:
            return ""

        # 优先使用基于当前状态的检测
        enhanced_result = self.classify_cluster_maneuver_enhanced(enemy_targets_data)
        if enhanced_result:
            return enhanced_result

        # 备用：使用原版群体分类器
        try:
            xy_now = {}
            xy_prev = {}

            for track_id, target_data in enemy_targets_data.items():
                lat = target_data["position"]["lat"]
                lon = target_data["position"]["lon"]
                x, y, _ = self.conv.geodetic_to_local(lat, lon, 0.0)
                xy_now[track_id] = np.array([x, y])

                if track_id in self._prev_positions:
                    xy_prev[track_id] = self._prev_positions[track_id]
                else:
                    xy_prev[track_id] = np.array([x, y])

            self._prev_positions = xy_now.copy()

            cls = ManeuverClassifier(self.cfg)
            track_dict = {tid: {'xy': xy_now[tid], 'xy_prev': xy_prev[tid]} for tid in xy_now.keys()}

            cluster_labels = cls.cluster_classify(track_dict, dt=self.cfg["dt"], xy_prev=xy_prev)

            if cluster_labels:
                maneuver_counts = {}
                for maneuver in cluster_labels.values():
                    if maneuver in ["converge", "diverge"]:
                        maneuver_counts[maneuver] = maneuver_counts.get(maneuver, 0) + 1

                if maneuver_counts:
                    most_common = max(maneuver_counts.items(), key=lambda x: x[1])[0]
                    return most_common

            return ""
        except Exception:
            return ""

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两个经纬度点之间的距离"""
        R = 6371000
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def calculate_group_center(self, targets):
        """计算群体中心位置"""
        if not targets:
            return {}

        lats = [target["position"]["lat"] for target in targets.values()]
        lons = [target["position"]["lon"] for target in targets.values()]
        alts = [target["position"]["alt"] for target in targets.values()]
        speeds = [target["motion"]["speed"] for target in targets.values()]
        headings = [target["motion"]["heading"] for target in targets.values()]

        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
        center_alt = sum(alts) / len(alts)
        avg_alt = sum(alts) / len(alts)
        avg_speed = sum(speeds) / len(speeds)

        avg_heading = math.degrees(
            math.atan2(
                sum(math.sin(math.radians(h)) for h in headings),
                sum(math.cos(math.radians(h)) for h in headings)
            )
        )

        max_distance = 0
        for lat, lon in zip(lats, lons):
            distance = self.haversine_distance(center_lat, center_lon, lat, lon)
            if distance > max_distance:
                max_distance = distance

        return {
            "center_lat": center_lat,
            "center_lon": center_lon,
            "center_alt": center_alt,
            "avg_alt": avg_alt,
            "avg_speed": avg_speed,
            "avg_heading": avg_heading,
            "radius": max_distance,
            "target_count": len(targets)
        }

    def get_maneuver_chinese(self, maneuver_english):
        """将英文机动类型转换为中文"""
        maneuver_map = {
            "left_turn": "向左机动",
            "right_turn": "向右机动",
            "climb": "爬升机动",
            "dive": "俯冲机动",
            "converge": "正在聚合",
            "diverge": "正在散开"
        }
        return maneuver_map.get(maneuver_english, "")

    def process_friendly_ucavs(self, packet):
        """处理我方UCAV数据"""
        raw_tracks = packet.get("tracks") or packet.get("Platforms", [])
        friendly_ucavs = {}

        for plat in raw_tracks:
            if plat.get("type") == "UCAV":
                try:
                    lat_val = plat["lat"][0] if isinstance(plat["lat"], list) else plat["lat"]
                    lon_val = plat["lon"][0] if isinstance(plat["lon"], list) else plat["lon"]
                    alt_val = plat["alt"][0] if isinstance(plat["alt"], list) else plat.get("alt", 0.0)
                    heading_val = plat["heading"][0] if isinstance(plat["heading"], list) else plat["heading"]

                    ucav_data = {
                        "id": plat["id"],
                        "lat": float(lat_val),
                        "lon": float(lon_val),
                        "alt": float(alt_val),
                        "spd": float(plat["speed"]),
                        "heading": float(heading_val)
                    }
                    friendly_ucavs[plat["id"]] = ucav_data
                except (KeyError, IndexError, ValueError):
                    continue

        return friendly_ucavs

    def update_base_data_cache(self, packet):
        """更新基础数据缓存"""
        try:
            # 处理我方UCAV基础数据
            friendly_ucavs = self.process_friendly_ucavs(packet)
            friendly_group_data = None

            if friendly_ucavs:
                friendly_targets = {}
                for ucav_id, ucav_data in friendly_ucavs.items():
                    friendly_targets[ucav_id] = {
                        "position": {
                            "lat": ucav_data['lat'],
                            "lon": ucav_data['lon'],
                            "alt": ucav_data.get('alt', 0.0)
                        },
                        "motion": {
                            "speed": ucav_data['spd'],
                            "heading": ucav_data['heading']
                        }
                    }

                friendly_group_stats = self.calculate_group_center(friendly_targets)
                if friendly_group_stats:
                    friendly_group_data = {
                        "lat": round(friendly_group_stats["center_lat"], 6),
                        "lon": round(friendly_group_stats["center_lon"], 6),
                        "alt": round(friendly_group_stats["center_alt"], 1),
                        "speed": round(friendly_group_stats["avg_speed"], 1),
                        "heading": round(friendly_group_stats["avg_heading"], 1)
                    }

            # 处理敌方基础数据
            enemy_group_data = None
            raw_tracks = packet.get("tracks") or packet.get("Platforms", [])
            enemy_targets_data = {}

            for plat in raw_tracks:
                if plat.get("type") != "STRIKER":
                    continue

                try:
                    lat_val = plat["lat"][0] if isinstance(plat["lat"], list) else plat["lat"]
                    lon_val = plat["lon"][0] if isinstance(plat["lon"], list) else plat["lon"]
                    alt_val = plat["alt"][0] if isinstance(plat["alt"], list) else plat.get("alt", 0.0)
                    heading_val = plat["heading"][0] if isinstance(plat["heading"], list) else plat["heading"]

                    trk = {
                        "id": plat["id"],
                        "lat": float(lat_val),
                        "lon": float(lon_val),
                        "alt": float(alt_val),
                        "spd": float(plat["speed"]),
                        "heading": float(heading_val)
                    }

                    track_id = trk['id']

                    self.update_obs(
                        track_id,
                        packet['timestamp'],
                        trk['lat'],
                        trk['lon'],
                        trk['spd'],
                        trk['heading'],
                        trk.get('alt')
                    )

                    enemy_targets_data[track_id] = {
                        "position": {
                            "lat": trk['lat'],
                            "lon": trk['lon'],
                            "alt": trk.get('alt', 0.0)
                        },
                        "motion": {
                            "speed": trk['spd'],
                            "heading": trk['heading']
                        }
                    }

                except (KeyError, IndexError, ValueError):
                    continue

            if enemy_targets_data:
                enemy_group_stats = self.calculate_group_center(enemy_targets_data)
                if enemy_group_stats:
                    enemy_group_data = {
                        "lat": round(enemy_group_stats["center_lat"], 6),
                        "lon": round(enemy_group_stats["center_lon"], 6),
                        "alt": round(enemy_group_stats["center_alt"], 1),
                        "speed": round(enemy_group_stats["avg_speed"], 1),
                        "heading": round(enemy_group_stats["avg_heading"], 1)
                    }

            with self._base_data_lock:
                self._base_data_cache["friendly_group"] = friendly_group_data
                self._base_data_cache["enemy_group"] = enemy_group_data
                self._base_data_cache["last_update_time"] = time.time()

        except Exception:
            pass

    def start_prediction_thread(self):
        """启动独立预测线程"""

        def prediction_worker():
            while self._running:
                try:
                    enemy_track_ids = []
                    with self._buffer_lock:
                        for track_id, buffer_data in self.buffers.items():
                            if buffer_data and len(buffer_data) > 0:
                                enemy_track_ids.append(track_id)

                    if enemy_track_ids:
                        all_goals = []
                        enemy_targets_data = {}

                        for track_id in enemy_track_ids:
                            with self._buffer_lock:
                                if self.buffers[track_id]:
                                    latest_data = self.buffers[track_id][-1]
                                    lat, lon, spd, hdg = latest_data[1], latest_data[2], latest_data[3], latest_data[4]
                                    alt = latest_data[5] if len(latest_data) > 5 else 0.0

                                    enemy_targets_data[track_id] = {
                                        "position": {"lat": lat, "lon": lon, "alt": alt},
                                        "motion": {"speed": spd, "heading": hdg}
                                    }

                        # 1. 首先检测群体机动
                        cluster_maneuver = self.classify_cluster_maneuver(enemy_targets_data)

                        # 2. 检测个体机动
                        individual_maneuver = ""
                        if "enemy_1" in enemy_track_ids:
                            individual_maneuver = self.classify_enemy1_maneuver()

                        # 3. 确定最终机动类型：个体优先
                        final_maneuver = ""

                        if individual_maneuver in ["left_turn", "right_turn", "climb", "dive"]:
                            final_maneuver = individual_maneuver
                        elif cluster_maneuver:
                            final_maneuver = cluster_maneuver

                        # 4. 目标预测
                        for track_id in enemy_track_ids:
                            try:
                                prediction_result = self.predict_long_term(track_id)
                                if prediction_result:
                                    goal_probs = prediction_result.get("goal_probs", {})
                                    if goal_probs:
                                        goal = max(goal_probs.items(), key=lambda x: x[1])[0]
                                        all_goals.append(goal)
                            except Exception:
                                continue

                        most_common_goal = "未知目标"
                        if all_goals:
                            goal_counts = {}
                            for goal in all_goals:
                                goal_counts[goal] = goal_counts.get(goal, 0) + 1
                            most_common_goal = max(goal_counts.items(), key=lambda x: x[1])[0]

                        with self._prediction_lock:
                            self._prediction_cache["maneuver"] = final_maneuver
                            self._prediction_cache["target_goal"] = most_common_goal
                            self._prediction_cache["last_update_time"] = time.time()

                        if final_maneuver:
                            print(f" 最终输出: {self.get_maneuver_chinese(final_maneuver)}->{most_common_goal}")

                    else:
                        with self._prediction_lock:
                            self._prediction_cache["maneuver"] = ""
                            self._prediction_cache["target_goal"] = "未知目标"

                    time.sleep(0.1)

                except Exception:
                    time.sleep(1.0)

        prediction_thread = threading.Thread(target=prediction_worker, daemon=True)
        prediction_thread.start()

    def start_server(self):
        """启动UDP服务器"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))

        print(f" 完全修复numpy警告服务器启动在 {self.host}:{self.port}")
        print(f" WebSocket服务器启动在端口 {self.websocket_port}")


        self.start_websocket_server()
        self.start_prediction_thread()

        try:
            while self._running:
                try:
                    data, addr = sock.recvfrom(65536)
                    packet = json.loads(data.decode('utf-8'))
                    self._packet_counter += 1

                    # 每个包都处理
                    self.update_base_data_cache(packet)

                except json.JSONDecodeError:
                    pass
                except Exception:
                    pass
        except KeyboardInterrupt:
            print("\n 服务器正在关闭...")
            self._running = False
        finally:
            sock.close()


if __name__ == "__main__":
    server = EnemyAnalysisServer("0.0.0.0", 5005)
    server.start_server()