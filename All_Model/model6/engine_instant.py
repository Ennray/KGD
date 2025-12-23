# engine.py
from collections import defaultdict, deque
from predictor_instant_full import LongHorizonHeadingPredictor

class Model6Engine:
    """
    独立实时模块：
      - update_obs(track_id, t, lat, lon, spd, heading_deg)
      - predict(track_id, horizons=(300,600,1800,3600))
    """
    def __init__(self, converter, costmap, goals, cfg=None):
        """
        converter : 复用模型4的 GeodeticToLocalConverter
        costmap   : callable(x,y)->cost
        goals     : [{'name':..., 'lat':..., 'lon':...}]
        cfg       : dict 可选 {dt, n_particles, w_cost, beta_goal, window_s, sector_deg}
        """
        self.conv = converter
        self.costmap = costmap
        self.cfg = cfg or {}
        self.window_s = float(self.cfg.get("window_s", 120.0))
        self.sector_deg = int(self.cfg.get("sector_deg", 30))
        # 目标预处理为 ENU
        self.goals_xy = []
        for g in goals:
            x, y, _ = self.conv.geodetic_to_local(float(g["lat"]), float(g["lon"]), 0.0)
            self.goals_xy.append({"name": g["name"], "xy": (x, y)})

        self.buffers = defaultdict(lambda: deque(maxlen=8192))
        self.predictors = {}

    def update_obs(self, track_id, t, lat, lon, spd, heading_deg):
        self.buffers[track_id].append((float(t), float(lat), float(lon), float(spd), float(heading_deg)))

    def _get_predictor(self, track_id):
        if track_id not in self.predictors:
            self.predictors[track_id] = LongHorizonHeadingPredictor(
                converter=self.conv, costmap=self.costmap, goals_xy=list(self.goals_xy),
                dt=self.cfg.get("dt", 1.0),
            )
        return self.predictors[track_id]

    def predict(self, track_id, horizons=(300, 600, 1800, 3600)):
        if track_id not in self.buffers or not self.buffers[track_id]:
            return {}
        pred = self._get_predictor(track_id)
        # 只喂最近 window_s 内的观测
        t_now = self.buffers[track_id][-1][0]
        for (t, lat, lon, spd, hdg) in self.buffers[track_id]:
            if t_now - t <= self.window_s:
                pred.update_obs(t, lat, lon, spd, hdg)
        return pred.predict(horizons_s=horizons, sector_deg=self.sector_deg)
