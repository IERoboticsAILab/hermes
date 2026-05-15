import pytest
from hermes_viz.hud.state import PacketRateMeter


def test_packet_rate_zero_before_any_packet():
    m = PacketRateMeter(time_constant_s=1.0)
    assert m.rate_hz(now_s=10.0) == pytest.approx(0.0)


def test_packet_rate_steady_state():
    m = PacketRateMeter(time_constant_s=1.0)
    for i in range(50):
        m.tick(now_s=100.0 + i * 0.1)
    rate = m.rate_hz(now_s=100.0 + 49 * 0.1)
    assert 8.0 <= rate <= 12.0, f"expected ~10 Hz, got {rate}"


def test_packet_rate_decays_when_silent():
    m = PacketRateMeter(time_constant_s=1.0)
    for i in range(20):
        m.tick(now_s=i * 0.1)
    rate_at_silence_start = m.rate_hz(now_s=2.0)
    rate_after_5s_silence = m.rate_hz(now_s=7.0)
    assert rate_after_5s_silence < rate_at_silence_start * 0.05


def test_packet_rate_rejects_nonpositive_tau():
    with pytest.raises(ValueError):
        PacketRateMeter(time_constant_s=0.0)
    with pytest.raises(ValueError):
        PacketRateMeter(time_constant_s=-1.0)
