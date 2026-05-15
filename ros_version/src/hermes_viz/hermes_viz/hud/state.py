"""HUD state aggregators. Pure logic — no ROS, no Vuer."""

from __future__ import annotations

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
