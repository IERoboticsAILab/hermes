"""HUD state aggregators. Pure logic — no ROS, no Vuer."""

from __future__ import annotations

import math
from collections import deque
from typing import Optional


class LatencyWindow:
    """Rolling window of latency samples. Computes p50/p95 over the window.

    Nearest-rank percentile; ties go to the lower index. Window is evicted
    lazily on add() and on each percentile query (when now_s is supplied).
    """

    def __init__(self, window_seconds: float):
        if window_seconds <= 0:
            raise ValueError("window_seconds must be > 0")
        self._window_s = window_seconds
        # deque of (now_s, latency_ms)
        self._samples: deque[tuple[float, float]] = deque()

    def add(self, latency_ms: float, now_s: float) -> None:
        self._samples.append((now_s, float(latency_ms)))
        self._evict(now_s)

    def _evict(self, now_s: float) -> None:
        cutoff = now_s - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _percentile(self, p: float, now_s: Optional[float]) -> Optional[float]:
        if now_s is not None:
            self._evict(now_s)
        if not self._samples:
            return None
        values = sorted(v for _, v in self._samples)
        # nearest-rank: index = round(p/100 * (n-1))
        idx = max(0, min(len(values) - 1, int(round(p / 100.0 * (len(values) - 1)))))
        return values[idx]

    def p50(self, now_s: Optional[float] = None) -> Optional[float]:
        return self._percentile(50.0, now_s)

    def p95(self, now_s: Optional[float] = None) -> Optional[float]:
        return self._percentile(95.0, now_s)


class PacketRateMeter:
    """Estimates packets/s using a single-pole exponential moving average.

    On each `tick(now_s)`, the instantaneous rate (1 / dt) is mixed into the
    running estimate with weight `1 - exp(-dt / tau)`. When read with
    `rate_hz(now_s)`, the estimate decays by `exp(-idle / tau)` to reflect
    silence since the last tick.
    """

    def __init__(self, time_constant_s: float):
        if time_constant_s <= 0:
            raise ValueError("time_constant_s must be > 0")
        self._tau = time_constant_s
        self._last_tick_s: Optional[float] = None
        self._rate: float = 0.0

    def tick(self, now_s: float) -> None:
        if self._last_tick_s is None:
            self._last_tick_s = now_s
            return
        dt = max(1e-6, now_s - self._last_tick_s)
        instant = 1.0 / dt
        alpha = 1.0 - math.exp(-dt / self._tau)
        self._rate = self._rate + alpha * (instant - self._rate)
        self._last_tick_s = now_s

    def rate_hz(self, now_s: float) -> float:
        if self._last_tick_s is None:
            return 0.0
        idle = max(0.0, now_s - self._last_tick_s)
        return self._rate * math.exp(-idle / self._tau)
