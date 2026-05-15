"""Bridge-callback tests. Skipped without rclpy installed."""

import pytest

rclpy = pytest.importorskip("rclpy")

from std_msgs.msg import String                       # noqa: E402
from hermes_viz.viz_bridge_node import VizBridgeNode  # noqa: E402


class RecordingSink:
    def __init__(self):
        self.events: list[tuple[str, tuple, dict]] = []

    def update_glove(self, *a, **kw):       self.events.append(("glove", a, kw))
    def update_robot(self, *a, **kw):       self.events.append(("robot", a, kw))
    def update_vest_motors(self, *a, **kw): self.events.append(("vest", a, kw))
    def update_gesture(self, *a, **kw):     self.events.append(("gesture", a, kw))
    def update_intent(self, *a, **kw):      self.events.append(("intent", a, kw))
    def push_hud(self, *a, **kw):           self.events.append(("hud", a, kw))


def _msg(text: str) -> String:
    m = String()
    m.data = text
    return m


def test_robot_state_callback_invokes_sink():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_robot_state(_msg(
            '{"schema": "hermes.robot_state_beacon.v1", "stamp_ms": 1, "robot_id": "r2",'
            ' "frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.5, "vx": 0, "vy": 0}'
        ))
        kinds = [e[0] for e in sink.events]
        assert "robot" in kinds
        # Validate args
        robot_evt = next(e for e in sink.events if e[0] == "robot")
        assert robot_evt[1] == ("r2", 1.0, 2.0, 0.5)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_swarm_intent_callback_invokes_sink_with_deadman():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_swarm_intent(_msg(
            '{"stamp_ms": 1000, "mode": "ACTIVE", "deadman_active": true,'
            ' "active_formation_type": "V"}'
        ))
        intent_evts = [e for e in sink.events if e[0] == "intent"]
        assert len(intent_evts) == 1
        assert intent_evts[0][2]["deadman_active"] is True
        assert intent_evts[0][2]["mode"] == "ACTIVE"
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_command_packet_callback_uses_command_key_as_gesture_label():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_command(_msg(
            '{"domain": "teleop", "command_id": "teleop.mode.follow",'
            ' "command_key": "MODE_FOLLOW"}'
        ))
        gesture_evts = [e for e in sink.events if e[0] == "gesture"]
        assert len(gesture_evts) == 1
        assert gesture_evts[0][2]["gesture"] == "MODE_FOLLOW"
        assert gesture_evts[0][2]["command_id"] == "teleop.mode.follow"
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_vest_callback_passes_levels_tuple():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_vest(_msg("V1,42,200,0,0,255,128,0"))
        vest_evts = [e for e in sink.events if e[0] == "vest"]
        assert len(vest_evts) == 1
        assert vest_evts[0][1] == ((200, 0, 0, 255, 128, 0),)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_raw_input_left_glove_fresh_emits_left_glove_event():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_raw_input(_msg(
            '{"time_ms": 1, '
            '"flex": {"L": {"index": 0.2, "middle": 0.2, "ring": 0.2, "pinky": 0.2}}, '
            '"fsr_pressed": {"L": {"INDEX": true, "MIDDLE": false, "RING": false, "PINKY": false}}, '
            '"imu": {"L": {"PITCH": 0, "ROLL": 0, "YAW": 0, "AX": 0, "AY": 0, "AZ": 0}}}'
        ))
        gloves = [e for e in sink.events if e[0] == "glove"]
        assert len(gloves) == 1
        assert gloves[0][2]["side"] == "left"
        assert gloves[0][2]["grip"] is True
        assert gloves[0][2]["curl"] == pytest.approx(0.2)
    finally:
        rclpy.shutdown()


def test_raw_input_deadman_off_emits_nothing():
    """Empty imu dict means left glove is stale; no glove events should fire."""
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_raw_input(_msg(
            '{"time_ms": 1, "flex": {}, "fsr_pressed": {}, "imu": {}}'
        ))
        assert [e for e in sink.events if e[0] == "glove"] == []
    finally:
        rclpy.shutdown()


def test_all_callbacks_no_op_on_malformed_input():
    """Each callback must accept garbage without raising or invoking sink."""
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        garbage = _msg("not json at all")
        node._on_raw_input(garbage)
        node._on_robot_state(garbage)
        node._on_swarm_intent(garbage)
        node._on_command(garbage)
        node._on_vest(garbage)
        assert sink.events == []
    finally:
        rclpy.shutdown()
