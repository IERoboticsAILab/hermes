"""DeadmanWatchdog is pure Python — testable without rclpy or Vuer."""

import pytest

# DeadmanWatchdog lives in viz_bridge_node, which imports rclpy at module level.
# Skip on machines without rclpy.
pytest.importorskip("rclpy")

from hermes_viz.viz_bridge_node import DeadmanWatchdog


def test_deadman_starts_dead():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    assert w.is_live(now_s=0.0) is False


def test_deadman_becomes_live_on_true_observation():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman_active=True, now_s=1.0)
    assert w.is_live(now_s=1.1) is True


def test_deadman_goes_dead_on_false_observation():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman_active=True, now_s=1.0)
    w.observe(deadman_active=False, now_s=1.1)
    assert w.is_live(now_s=1.2) is False


def test_deadman_goes_dead_on_silence():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman_active=True, now_s=1.0)
    assert w.is_live(now_s=1.4) is True   # within window
    assert w.is_live(now_s=1.6) is False  # > 500 ms silence


def test_deadman_rejects_nonpositive_timeout():
    with pytest.raises(ValueError):
        DeadmanWatchdog(silence_timeout_s=0.0)
    with pytest.raises(ValueError):
        DeadmanWatchdog(silence_timeout_s=-1.0)
