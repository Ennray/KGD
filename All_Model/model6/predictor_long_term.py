# predictor_long_term.py  (长时 + 6 机动)
import numpy as np
from math import sin, cos, atan2, sqrt, radians, degrees
from collections import Counter
from maneuver_classifier import ManeuverClassifier   # ① 机动分类器

class LongHorizonHeadingPredictor:
    """
    长时粒子滤波 + 6 类机动投票（left/right/climb/dive/converge/diverge）
    接口与原瞬时版 100 % 兼容
    """
    def __init__(self, converter, costmap=None, goals_xy=None,
                 dt=1.0, n_particles=1000, w_cost=0.02, beta_goal=0.3,
                 goal_scale_m=50_000, rho_omega=0.98, sigma_eta_deg=0.05,
                 R_min=None, gamma_weights=0.8, resample_neff_ratio=0.4,
                 rng_seed=0, cfg=None):
        self.conv = converter
        self.costmap = costmap or (lambda x, y: 0.0)
        self.goals = goals_xy or []
        self.dt = float(dt)
        self.N = int(n_particles)
        self.w_cost = float(w_cost)
        self.beta_goal = float(beta_goal)
        self.goal_scale_m = float(goal_scale_m)
        self.rho_omega = float(rho_omega)
        self.sigma_eta = radians(sigma_eta_deg)
        self.R_min = R_min
        self.gamma_weights = float(gamma_weights)
        self.resample_neff_ratio = float(resample_neff_ratio)
        self.cfg = cfg or {}               # 机动阈值入口
        self.rng = np.random.default_rng(rng_seed)
        self.hist = []

    # ---------------- 喂观测 ----------------
    def update_obs(self, t, lat, lon, spd, heading_deg, alt=None):
        # alt 可选，用于上下机动
        if alt is not None:
            self.hist.append((float(t), float(lat), float(lon),
                              float(spd), float(heading_deg), float(alt)))
        else:
            self.hist.append((float(t), float(lat), float(lon),
                              float(spd), float(heading_deg)))
        if len(self.hist) > 6000:
            self.hist = self.hist[-6000:]

    # ---------------- 鲁棒角速度估计 ----------------
    def _estimate_turnrate(self, k_win=8, alpha=0.35):
        if len(self.hist) < 3:
            return 0.0, self.sigma_eta**2
        ts, lats, lons, spds, heads = list(zip(*self.hist[-(k_win+1):]))[:5]
        heads = np.unwrap(np.radians(np.array(heads)))
        dts = np.diff(ts)
        dt_mean = np.mean(dts) if len(dts) > 0 else self.dt
        raw_w = np.diff(heads) / np.maximum(dt_mean, 1e-3)
        # Hampel 去极值
        med = np.median(raw_w)
        mad = np.median(np.abs(raw_w - med)) + 1e-6
        mask = np.abs(raw_w - med) < 3*1.4826*mad
        w = raw_w[mask] if np.any(mask) else raw_w
        w_hat = 0.0
        for wi in w:
            w_hat = alpha*wi + (1-alpha)*w_hat
        var = np.var(w) if len(w) > 1 else (np.deg2rad(3)/10.0)**2
        return float(w_hat), float(var + 1e-6)

    # ---------------- 粒子采样 ----------------
    def _sample_omega(self, size):
        w_mean, w_var = self._estimate_turnrate()
        std = max(np.sqrt(max(w_var, 1e-6)), np.deg2rad(0.2))
        mix = [
            (0.70, 0.0, np.deg2rad(0.2)),      # 直飞
            (0.25, w_mean, max(std, np.deg2rad(0.5))),  # 缓转
            (0.04, np.sign(w_mean + 1e-6) * np.deg2rad(6), np.deg2rad(2)),  # 急转
            (0.01, 0.0, np.deg2rad(8))         # 盘旋
        ]
        cat = self.rng.choice(len(mix), size=size, p=[m[0] for m in mix])
        return np.array([self.rng.normal(mix[i][1], mix[i][2]) for i in cat])

    # ---------------- 目标吸引 ----------------
    def _goal_weights(self, x, y):
        if not self.goals:
            return None, None
        d = np.array([np.hypot(x - g['xy'][0], y - g['xy'][1]) for g in self.goals], dtype=float)
        z = -d / max(self.goal_scale_m, 1.0)
        z = z - np.max(z)
        e = np.exp(np.clip(z, -60, 60))
        p = e / (e.sum() + 1e-12)
        return p, d

    # ---------------- 扇区统计 ----------------
    def _sector_probs(self, angles_rad, weights, sector_deg=30):
        K = int(360 / sector_deg)
        edges = np.linspace(0.0, 2 * np.pi, K + 1)
        idx = np.digitize(np.mod(angles_rad, 2 * np.pi), edges, right=False) - 1
        idx = np.clip(idx, 0, K - 1)
        probs = np.zeros(K, dtype=float)
        for i, w in zip(idx, weights):
            probs[i] += w
        s = probs.sum()
        if s > 0:
            probs /= s
        return {f"{k * sector_deg}-{(k + 1) * sector_deg}": float(probs[k]) for k in range(K)}

    # ================== 主预测 ==================
    def predict(self, horizons_s=(300, 600, 1800, 3600), sector_deg=30):
        if not self.hist:
            return {}
        # 最新状态
        t, lat, lon, v0, hdg_deg = self.hist[-1][:5]
        x0, y0, _ = self.conv.geodetic_to_local(lat, lon, 0.0)
        psi0 = radians(hdg_deg)
        v_sigma = max(0.05 * max(v0, 0.1), 0.8)

        N = self.N
        xs   = np.full(N, x0, dtype=float)
        ys   = np.full(N, y0, dtype=float)
        psis = self.rng.normal(psi0, radians(2), size=N)
        vs   = np.clip(self.rng.normal(v0, v_sigma, size=N), 0.1, None)
        ws   = self._sample_omega(N)
        wts  = np.full(N, 1.0 / N, dtype=float)

        goal_names = [g['name'] for g in self.goals] if self.goals else []
        results = {}
        t_acc = 0.0
        horizons = sorted(set(int(h) for h in horizons_s))
        maxT = horizons[-1]

        # 实例化机动分类器（只建一次）
        cls = ManeuverClassifier(self.cfg)

        while t_acc < maxT:
            dt = self.dt
            # CTRV 积分
            small = (np.abs(ws) < 1e-5)
            sin1 = np.sin(psis + ws*dt) - np.sin(psis)
            cos1 = -np.cos(psis + ws*dt) + np.cos(psis)
            step = np.where(small, dt, 1.0 / ws)
            xs   = xs + vs * step * sin1
            ys   = ys + vs * step * cos1
            psis = psis + ws*dt
            vs   = np.clip(vs + self.rng.normal(0, 0.10, size=N), 0.1, None)

            # 最小转弯半径
            if self.R_min is not None:
                w_cap = np.clip(vs / max(self.R_min, 1e-3), 1e-6, None)
                ws = np.clip(ws, -w_cap, w_cap)

            # 地图代价
            costs = np.array([self.costmap(xi, yi) for xi, yi in zip(xs, ys)], dtype=float)
            wts *= np.exp(-self.w_cost * costs)

            # 目标吸引
            if self.goals:
                for i in range(N):
                    pw, _ = self._goal_weights(xs[i], ys[i])
                    wts[i] *= float(pw[np.argmax(pw)])
            wts = np.power(wts, self.gamma_weights)
            s = wts.sum()
            if s <= 0 or not np.isfinite(s):
                wts[:] = 1.0 / N
            else:
                wts /= s

            # ===== ⑥ 类机动投票（关键插入） =====
            labels = np.empty(N, dtype='U12')
            for i in range(N):
                # 伪 hist：2 点即可估角速度
                fake_hist = [(0, 0, 0, vs[i], degrees(psis[i])),
                             (0, 0, 0, vs[i], degrees(psis[i] - ws[i]*self.dt))]
                labels[i] = cls.inst_classify(fake_hist, alt=None)
            # 权重投票
            maneuver_probs = {}
            for lab in np.unique(labels):
                maneuver_probs[lab] = float(wts[labels == lab].sum() / (wts.sum() + 1e-12))
            # 存结果
            maneuver_most = max(maneuver_probs, key=maneuver_probs.get) if maneuver_probs else 'straight'

            # 自适应重采样
            neff = 1.0 / np.sum(wts**2)
            if neff < self.resample_neff_ratio * N:
                idx = self.rng.choice(N, size=N, p=wts)
                xs, ys, psis, vs, ws = xs[idx], ys[idx], psis[idx], vs[idx], ws[idx]
                wts = np.full(N, 1.0 / N, dtype=float)

            t_acc += dt
            if int(t_acc) in set(horizons):
                # 航向统计
                cs, ss = np.cos(psis), np.sin(psis)
                C, S = float((wts*cs).sum()), float((wts*ss).sum())
                ang = atan2(S, C)
                R = np.hypot(C, S)
                kappa = max(R * N, 1e-6)
                ci = degrees(2.0 / np.sqrt(kappa + 1e-9))

                # 目标概率
                goal_probs = {}
                if self.goals:
                    gsum = np.zeros(len(self.goals), dtype=float)
                    for i in range(N):
                        pw, _ = self._goal_weights(xs[i], ys[i])
                        gsum += wts[i] * pw
                    gsum /= (gsum.sum() + 1e-9)
                    goal_probs = {goal_names[j]: float(gsum[j]) for j in range(len(self.goals))}

                # 存入结果
                results[int(t_acc)] = {
                    "heading_deg_mean": float(degrees(ang) % 360.0),
                    "heading_ci_deg": float(ci),
                    "sector_probs": self._sector_probs(psis, wts, sector_deg=sector_deg),
                    "goal_probs": goal_probs,
                    "maneuver_probs": maneuver_probs,          
                    "most_likely_maneuver": maneuver_most,    
                    "quality": {"R": float(R), "kappa": float(kappa), "Neff": float(neff)}
                }
        return results

'''
可参考参数：cfg = {
  "dt": 1.0,
  "n_particles": 1000,
  "window_s": 120,
  "sector_deg": 30,
  "w_cost": 0.02,            # 接入真实代价图后再微调
  "goal_scale_m": 50_000,    # 40–60 km 之间看场景
  "rho_omega": 0.96,
  "sigma_eta_deg": 0.12,
  "gamma_weights": 0.85,
  "resample_neff_ratio": 0.4,
  # 可选：平台约束
  # "R_min": 500   # 有最小转弯半径就加上
}

'''