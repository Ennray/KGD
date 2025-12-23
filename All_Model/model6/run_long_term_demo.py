#!/usr/bin/env python3
# run_long_term_demo.py
import time, math, json
from GeodeticConverter_instant import GeodeticToLocalConverter
from predictor_long_term import LongHorizonHeadingPredictor

# ---------------- ① 坐标系 ----------------
A_lat, A_lon, A_alt = 29.9, 126.6, 0.0
B_lat, B_lon, B_alt = 29.9001, 126.6, 0.0          # 正北 100 m
conv = GeodeticToLocalConverter(A_lat, A_lon, A_alt, B_lat, B_lon, B_alt)

# ---------------- ② 目标点（要地） ----------------
goals_xy = [
    {"name": "RadarSite", "xy": (10e3, 12e3)},      # 10 km 处雷达
    {"name": "Airport",   "xy": (-5e3, -8e3)},
]

# ---------------- ③ 预测器 ----------------
pred = LongHorizonHeadingPredictor(
    converter=conv,
    costmap=lambda x, y: 0.001 * max(0, abs(x - 5e3)) + 0.001 * max(0, abs(y - 5e3)),  # 假代价山
    goals_xy=goals_xy,
    cfg={
        "omega_threshold_deg": 2.0,
        "vz_threshold_mps": 0.5,
        "n_particles": 800,
        "horizons": [300, 600, 1800, 3600],
        "sector_deg": 30,
        "w_cost": 0.02,
        "beta_goal": 0.3,
        "R_min": 500,                 # 最小转弯半径
    }
)

# ---------------- ④ 造一条“机动轨迹” ----------------
# 0-30 s：右转+爬升 → 30-60 s：直线 → 60-90 s：左转+俯冲
def generate_maneuver_track():
    hist, alt = [], 100.0
    lat, lon, spd, hdg = A_lat, A_lon, 120.0, 70.0
    for i in range(90):                      # 90 s @ 1 Hz
        t = time.time() + i
        hist.append((t, lat, lon, spd, hdg, alt))
        # ---- 机动段 ----
        if i < 30:                           # 右转+爬升
            hdg += 3.0
            alt += 1.0
        elif i < 60:                         # 直线
            pass
        else:                                # 左转+俯冲
            hdg -= 2.5
            alt -= 1.2
        # 直线外推
        dx = spd * 1.0 * math.sin(math.radians(hdg))
        dy = spd * 1.0 * math.cos(math.radians(hdg))
        Rlat = 111320.0
        Rlon = 111320.0 * math.cos(math.radians(lat))
        lat += dy / Rlat
        lon += dx / Rlon
    return hist

# ---------------- ⑤ 喂数据 & 预测 ----------------
hist = generate_maneuver_track()
for t, lat, lon, spd, hdg, alt in hist:
    pred.update_obs(t, lat, lon, spd, hdg, alt)

out = pred.predict([300, 600, 1800, 3600])

# ---------------- ⑥ 漂亮打印所有信息 ----------------
def pretty(t, v):
    print(f"\n====== {t}s 预测结果 ======")
    print(f"航向均值 : {v['heading_deg_mean']:7.2f}°")
    print(f"航向误差 : ±{v['heading_ci_deg']:5.2f}° (1σ)")
    print(f"扇区概率 : {max(v['sector_probs'], key=v['sector_probs'].get)} "
          f"{v['sector_probs'][max(v['sector_probs'], key=v['sector_probs'].get)]:.2f}")
    print(f"目标概率 : {max(v['goal_probs'], key=v['goal_probs'].get)} "
          f"{v['goal_probs'][max(v['goal_probs'], key=v['goal_probs'].get)]:.2f}")
    print(f"最可能机动: {v['most_likely_maneuver']} "
          f"{v['maneuver_probs'].get(v['most_likely_maneuver'], 0):.2f}")
    print(f"机动分布 : {json.dumps(v['maneuver_probs'], indent=0)}")
    print(f"质量指标 : R={v['quality']['R']:.2f}, κ={v['quality']['kappa']:.0f}, "
          f"Neff={v['quality']['Neff']:.0f}")

for t, v in out.items():
    pretty(t, v)