"""Bridge skeleton tests. Skipped on machines without rclpy installed."""

import pytest

rclpy = pytest.importorskip("rclpy")


from std_msgs.msg import String                          # noqa: E402
from hermes_viz.viz_bridge_node import VizBridgeNode     # noqa: E402


class _NullSink:
    def update_glove(self, *a, **kw): pass
    def update_robot(self, *a, **kw): pass
    def update_vest_motors(self, *a, **kw): pass
    def update_gesture(self, *a, **kw): pass
    def update_intent(self, *a, **kw): pass
    def push_hud(self, *a, **kw): pass


def test_bridge_subscribes_to_five_topics():
    rclpy.init()
    try:
        node = VizBridgeNode(scene_sink=_NullSink())
        subs = {s.topic_name for s in node.subscriptions}
        assert "/hermes/raw_input" in subs
        assert "/hermes/robot_state_beacon" in subs
        assert "/hermes/swarm_intent" in subs
        assert "/hermes/command_packets" in subs
        assert "/hermes/vest_serial_tx" in subs
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_bridge_publishes_nothing():
    """Read-only invariant: the bridge must not create any publishers."""
    rclpy.init()
    try:
        node = VizBridgeNode(scene_sink=_NullSink())
        assert list(node.publishers) == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_bridge_callbacks_are_safe_on_garbage_input():
    """All five callbacks must accept malformed String messages without raising."""
    rclpy.init()
    try:
        node = VizBridgeNode(scene_sink=_NullSink())
        garbage = String()
        garbage.data = "not_json_at_all{["
        node._on_raw_input(garbage)        # noqa: SLF001
        node._on_robot_state(garbage)      # noqa: SLF001
        node._on_swarm_intent(garbage)     # noqa: SLF001
        node._on_command(garbage)          # noqa: SLF001
        node._on_vest(garbage)             # noqa: SLF001
        node.destroy_node()
    finally:
        rclpy.shutdown()
