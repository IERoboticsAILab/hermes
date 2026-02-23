# gesture/imu_filter.py
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class ImuFilterConfig:
    alpha: float = 0.25  # low-pass: higher = more responsive, lower = smoother


class ImuFilter:
    def __init__(self, config: Optional[ImuFilterConfig] = None):
        self.cfg = config or ImuFilterConfig()
        self._y = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self._initialized = False

    def update(self, imu_raw: Dict[str, float]) -> Dict[str, float]:
        if not self._initialized:
            self._y = {k: float(imu_raw.get(k, 0.0)) for k in self._y.keys()}
            self._initialized = True
            return dict(self._y)

        a = self.cfg.alpha
        for k in self._y.keys():
            x = float(imu_raw.get(k, 0.0))
            self._y[k] = a * x + (1.0 - a) * self._y[k]
        return dict(self._y)