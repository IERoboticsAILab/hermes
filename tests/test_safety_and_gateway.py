from copy import deepcopy

import pytest

from gestures.models import GestureEvent, GestureState
from gestures.registry import COMMAND_REGISTRY
from gestures.safety import SafetyEvaluator


def test_deadman_imu_gates_motion_off_then_on() -> None:
    safety = SafetyEvaluator()
    state = GestureState()

    # Palm-up (AZ ~= -1g) should gate OFF after debounce.
    ev_up = GestureEvent(accel_L={"AX": 0.0, "AY": 0.0, "AZ": -1.0})
    safety.tick(ev_up, state, COMMAND_REGISTRY, now_ms=0)
    pkt_off = safety.tick(ev_up, state, COMMAND_REGISTRY, now_ms=130)
    assert pkt_off is not None
    assert pkt_off["effect"]["type"] == "gate_motion"
    assert pkt_off["effect"]["value"] is False

    # Palm-down (AZ ~= +1g) should gate back ON after debounce.
    ev_down = GestureEvent(accel_L={"AX": 0.0, "AY": 0.0, "AZ": 1.0})
    safety.tick(ev_down, state, COMMAND_REGISTRY, now_ms=260)
    pkt_on = safety.tick(ev_down, state, COMMAND_REGISTRY, now_ms=390)
    assert pkt_on is not None
    assert pkt_on["effect"]["type"] == "gate_motion"
    assert pkt_on["effect"]["value"] is True


def test_shake_estop_fires_after_required_hold() -> None:
    safety = SafetyEvaluator()
    state = GestureState()

    # 2g vector magnitude => dynamic accel ~= 1g above gravity baseline.
    ev_shake = GestureEvent(
        accel_L={"AX": 2.0, "AY": 0.0, "AZ": 0.0},
    )
    safety.tick(ev_shake, state, COMMAND_REGISTRY, now_ms=0)
    pkt = safety.tick(ev_shake, state, COMMAND_REGISTRY, now_ms=250)

    assert pkt is not None
    assert pkt["effect"]["type"] == "emergency_stop"
    assert pkt["command_key"] == "ESTOP"


def test_gateway_timeout_forces_deadman_false() -> None:
    pytest.importorskip("serial")

    from espnow_testbed.pi_gateway.hermes_gateway import HermesGateway, TimedPacket

    cfg = {
        "serial_port": "/dev/null",
        "baudrate": 115200,
        "glove_timeout_ms": 100,
        "robot_ids": ["r1", "r2"],
        "centroid": [0.0, 0.0],
        "command_output": {"mode": "print", "udp_host": "127.0.0.1", "udp_port": 5005},
        "behavior_runtime": {"home_xy": [0.0, 0.0], "path_waypoints": []},
    }
    gw = HermesGateway(deepcopy(cfg))
    gw.gesture_state.deadman_active = True

    stale_ts = gw._now_ms() - gw.glove_timeout_ms - 1
    gw.left_latest = TimedPacket(packet={}, rx_monotonic_ms=stale_ts)
    gw.right_latest = TimedPacket(packet={}, rx_monotonic_ms=stale_ts)

    raw = gw._build_raw()
    assert raw is None
    assert gw.gesture_state.deadman_active is False
