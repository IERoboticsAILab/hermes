import pytest
from hermes_viz.hud.state import LatencyWindow


def test_latency_window_empty_returns_none():
    w = LatencyWindow(window_seconds=5.0)
    assert w.p50() is None
    assert w.p95() is None


def test_latency_window_single_sample():
    w = LatencyWindow(window_seconds=5.0)
    w.add(latency_ms=17.5, now_s=100.0)
    assert w.p50(now_s=100.1) == pytest.approx(17.5)
    assert w.p95(now_s=100.1) == pytest.approx(17.5)


def test_latency_window_evicts_old_samples():
    w = LatencyWindow(window_seconds=1.0)
    w.add(latency_ms=10.0, now_s=100.0)
    w.add(latency_ms=20.0, now_s=100.5)
    w.add(latency_ms=30.0, now_s=102.0)  # the first two are now > 1s old
    assert w.p50(now_s=102.0) == pytest.approx(30.0)


def test_latency_window_p50_p95_distribution():
    w = LatencyWindow(window_seconds=10.0)
    base = 1000.0
    for i, v in enumerate(range(1, 101)):  # 100 samples 1..100
        w.add(latency_ms=float(v), now_s=base + i * 0.01)
    now = base + 1.0
    p50 = w.p50(now_s=now)
    p95 = w.p95(now_s=now)
    assert p50 is not None and 49.0 <= p50 <= 52.0, f"p50={p50}"
    assert p95 is not None and 93.0 <= p95 <= 97.0, f"p95={p95}"
