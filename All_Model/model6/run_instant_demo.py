#!/usr/bin/env python3
# run_instant_demo.py
import time, math
from GeodeticConverter_instant import GeodeticToLocalConverter
from predictor_instant_full import LongHorizonHeadingPredictor

# ① 坐标系
conv = GeodeticToLocalConverter(29.9, 126.6, 0.0, 29.9001, 126.6, 0.0)

# ② 预测器
pred = LongHorizonHeadingPredictor(
    converter=conv,
    cfg={"omega_threshold_deg": 2.0, "vz_threshold_mps": 0.5}
)

# ③ 造轨迹：0-20 s 右转+爬升，20-40 s 直线，40-60 s 左转+俯冲
hist = []
lat, lon, alt, spd, hdg = 29.9, 126.6, 100.0, 120.0, 70.0
for i in range(60):
    t = time.time() + i
    hist.append((t, lat, lon, spd, hdg, alt))
    # 机动段
    if i < 20:
        hdg += 3.0
        alt += 1.0
    elif i < 40:
        pass
    else:
        hdg -= 2.5
        alt -= 1.2
    # 直线外推
    dx = spd * 1.0 * math.sin(math.radians(hdg))
    dy = spd * 1.0 * math.cos(math.radians(hdg))
    Rlat = 111320.0
    Rlon = 111320.0 * math.cos(math.radians(lat))
    lat += dy / Rlat
    lon += dx / Rlon

# ④ 喂数据 & 瞬时预测
for t, lat, lon, spd, hdg, alt in hist:
    pred.update_obs(t, lat, lon, spd, hdg, alt)

inst = pred.predict()[0]          # 只拿 key=0

# ⑤ 漂亮打印所有信息
print("====== 瞬时预测（6 类机动）======")
print(f"当前航向 : {inst['heading_deg_mean']:7.2f}°")
print(f"航向误差 : ±{inst['heading_ci_deg']:5.2f}° (1σ)")
print(f"角速度   : {inst['turnrate_deg_s']:6.2f}°/s")
print(f"机动类型 : {inst['maneuver']} (confidence={inst['maneuver_confidence']})")
print(f"下一拍坐标: ({inst['next_lat']:9.5f}, {inst['next_lon']:9.5f})")
print(f"时间戳   : {inst['timestamp']:.1f}")