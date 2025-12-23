# predictor_instant_full.py  瞬时预测 + 6 机动
import numpy as np
from math import sin, cos, atan2, sqrt, radians, degrees
from maneuver_classifier import ManeuverClassifier   # ① 机动插头

class LongHorizonHeadingPredictor:
    """
    瞬时专用：单点 EKF 思想，但保留类名兼容；
    输出当前 ψ、ω、下一拍坐标 + 6 类机动标签
    """
    def __init__(self, converter, costmap=None, goals_xy=None,
                 dt=1.0, rho_omega=0.9, sigma_eta_deg=0.05, cfg=None):
        self.conv = converter
        self.dt = float(dt)
        self.rho_omega = float(rho_omega)
        self.sigma_eta = radians(sigma_eta_deg)
        self.cfg = cfg or {}
        self.hist = []          # [(t,lat,lon,spd,hdg,?alt), ...]
        self._w_ema = 0.0       # 角速度 EMA

    # ---------------- 喂观测 ----------------
    def update_obs(self, t, lat, lon, spd, heading_deg, alt=None):
        # alt 可选，用于上下机动
        if alt is not None:
            self.hist.append((float(t), float(lat), float(lon),
                              float(spd), float(heading_deg), float(alt)))
        else:
            self.hist.append((float(t), float(lat), float(lon),
                              float(spd), float(heading_deg)))
        # 只留最近 5 点
        if len(self.hist) > 5:
            self.hist.pop(0)

    # ---------------- 鲁棒角速度估计 ----------------
    def _estimate_turnrate(self):
        if len(self.hist) < 2:
            return 0.0, self.sigma_eta**2
        # Python3 兼容：先转 list 再切片
        tmp = list(zip(*self.hist))[:5]
        ts, heads = tmp[0], tmp[4]
        dts = np.diff(ts)
        dt_mean = np.mean(dts) if len(dts) > 0 else self.dt
        heads = np.unwrap(np.radians(np.array(heads)))
        raw_w = np.diff(heads) / np.maximum(dt_mean, 1e-3)
        # Hampel 去极值
        med = np.median(raw_w)
        mad = np.median(np.abs(raw_w - med)) + 1e-6
        mask = np.abs(raw_w - med) < 3 * 1.4826 * mad
        w = raw_w[mask] if np.any(mask) else raw_w
        # EMA 平滑
        w_now = float(np.mean(w)) if w.size else 0.0
        self._w_ema = self.rho_omega * w_now + (1 - self.rho_omega) * self._w_ema
        var = float(np.var(w)) if w.size > 1 else (radians(1.0) / 10.0) ** 2
        return self._w_ema, var + 1e-12

    # ---------------- 机动分类 ----------------
    def _classify_now(self):
        alt_hist = [h[5] for h in self.hist] if len(self.hist[0]) == 6 else None
        return ManeuverClassifier(self.cfg).inst_classify(self.hist, alt_hist)

    # ---------------- 主预测接口 ----------------
    def predict(self, horizons_s=None, sector_deg=None):
        # horizons_s / sector_deg 已失效，保留兼容
        if not self.hist:
            return {}
        # 最新状态
        t, lat, lon, spd, hdg = self.hist[-1][:5]
        x, y, _ = self.conv.geodetic_to_local(lat, lon, 0.0)
        psi = radians(hdg)
        v = max(spd, 0.1)
        dt = self.dt

        # 1. 估角速度
        w_mean, w_var = self._estimate_turnrate()

        # 2. 单步 CTRV 推下一拍
        small = abs(w_mean) < 1e-5
        if small:
            x1 = x + v * dt * sin(psi)
            y1 = y + v * dt * cos(psi)
        else:
            sin1 = sin(psi + w_mean * dt) - sin(psi)
            cos1 = -cos(psi + w_mean * dt) + cos(psi)
            x1 = x + v / w_mean * sin1
            y1 = y + v / w_mean * cos1
        lat1, lon1, _ = self.conv.local_to_geodetic([x1, y1, 0.])

        # 3. 不确定度
        psi_std = sqrt(w_var * dt ** 2 + radians(1.0) ** 2)

        # 4. 机动标签
        maneuver = self._classify_now()
        conf = 1.0 if maneuver != "straight" else 0.0

        # 5. 返回（key=0 兼容旧接口）
        return {
            0: {
                "heading_deg_mean": float(degrees(psi) % 360.),
                "heading_ci_deg": float(degrees(psi_std)),
                "turnrate_deg_s": float(degrees(w_mean)),
                "next_lat": float(lat1),
                "next_lon": float(lon1),
                "timestamp": float(t + dt),
                "maneuver": maneuver,
                "maneuver_confidence": conf,
                "quality": {"R": 1.0, "kappa": 1e3, "Neff": 1000}
            }
        }