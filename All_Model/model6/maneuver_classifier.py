# maneuver_classifier.py
import numpy as np
from scipy.spatial.distance import cdist
from math import radians, degrees, sqrt

class ManeuverClassifier:
    """
    6 类机动：left_turn | right_turn | climb | dive | converge | diverge | straight
    ① 左右：角速度阈值
    ② 上下：垂速阈值（需 alt）
    ③ 聚散：邻机距离变化率阈值
    """
    DEF_OMEGA_TH     = 2.0   # °/s
    DEF_VZ_TH        = 0.5   # m/s
    DEF_DELTA_CD     = 50.0  # m/s

    def __init__(self, cfg=None):
        self.cfg = cfg or {}
        self.omega_th = self.cfg.get("omega_threshold_deg", self.DEF_OMEGA_TH)
        self.vz_th    = self.cfg.get("vz_threshold_mps",    self.DEF_VZ_TH)
        self.delta_cd = self.cfg.get("delta_cd_mps",        self.DEF_DELTA_CD)

    # ------------- ① 单机瞬时 ----------------
    # maneuver_classifier.py
    def inst_classify(self, hist, alt=None):
        if len(hist) < 2:
            print("    ❌ 历史数据不足")
            return "straight"

        dt = hist[-1][0] - hist[-2][0]
        if dt <= 0:
            #print("    ❌ 时间间隔无效")
            return "straight"

        # 获取航向并正确处理负值
        heading1_raw = hist[-2][4]  # 原始航向（可能是负值）
        heading2_raw = hist[-1][4]  # 原始航向（可能是负值）

        # 将航向转换到 0-360 范围
        heading1 = heading1_raw % 360
        heading2 = heading2_raw % 360

        # 计算最短角度差（处理360度边界）
        diff = heading2 - heading1
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        omega = diff / dt

        # 详细调试输出
        print(f"    📐 航向分析:")
        print(f"        原始航向: {heading1_raw:.1f}° -> {heading2_raw:.1f}°")
        print(f"        归一航向: {heading1:.1f}° -> {heading2:.1f}°")
        print(f"        角度差: {diff:.1f}°")
        print(f"        角速度: {omega:.2f}°/s")
        print(f"        阈值: ±{self.omega_th}°/s")
        print(f"        时间间隔: {dt:.1f}s")

        # 检查是否达到转弯阈值
        if omega > self.omega_th:
            print(f"    🟢 检测到右转 (角速度: {omega:.2f}°/s > 阈值: {self.omega_th}°/s)")
            return "right_turn"
        if omega < -self.omega_th:
            print(f"    🟢 检测到左转 (角速度: {omega:.2f}°/s < 阈值: -{self.omega_th}°/s)")
            return "left_turn"

        # 计算垂向速度
        if alt is not None and len(alt) >= 2:
            vz = (alt[-1] - alt[-2]) / dt
            print(f"    📐 高度分析:")
            print(f"        垂向速度: {vz:.2f}m/s")
            print(f"        阈值: ±{self.vz_th}m/s")
            if vz > self.vz_th:
                print(f"    🟢 检测到爬升 (垂速: {vz:.2f}m/s > 阈值: {self.vz_th}m/s)")
                return "climb"
            if vz < -self.vz_th:
                print(f"    🟢 检测到俯冲 (垂速: {vz:.2f}m/s < 阈值: -{self.vz_th}m/s)")
                return "dive"

        print(f"    🔵 检测到直线飞行 (角速度: {omega:.2f}°/s, 未达到阈值)")
        return "straight"

    # ------------- ② 群级聚散 ----------------
    def cluster_classify(self, track_dict, dt, xy_prev=None):
        """
        track_dict: {tid: {'xy':(x,y), 'xy_prev':(x_prev,y_prev)}}
        dt        : 时间间隔 s
        return    : {tid: label}
        """
        if xy_prev is None:
            xy_prev = {tid: np.array(d['xy_prev']) for tid, d in track_dict.items()}
        xy_now  = {tid: np.array(d['xy']) for tid, d in track_dict.items()}
        tids = list(xy_now.keys())
        if len(tids) < 2:
            return {tid: "straight" for tid in tids}

        # 距离矩阵
        prev_mat = cdist([xy_prev[tid] for tid in tids],
                         [xy_prev[tid] for tid in tids])
        now_mat  = cdist([xy_now[tid]  for tid in tids],
                         [xy_now[tid]  for tid in tids])
        dD = (now_mat - prev_mat) / dt       # 变化率矩阵

        # 每架机取邻域平均（排除自距=0）
        labels = {}
        for i, tid in enumerate(tids):
            neigh_rate = np.mean(dD[i, dD[i] != 0])
            if neigh_rate < -self.delta_cd:
                labels[tid] = "converge"
            elif neigh_rate > self.delta_cd:
                labels[tid] = "diverge"
            else:
                labels[tid] = "straight"
        return labels