import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hermes_control.gestures.matcher import match_gesture
from hermes_control.gestures.models import GestureState
from hermes_control.gestures.recognizer import GestureRecognizer
from hermes_control.gestures.registry import COMMAND_REGISTRY
from hermes_control.gestures.safety import SafetyEvaluator


class GesturePipelineNode(Node):
    def __init__(self) -> None:
        super().__init__("gesture_pipeline_node")

        self.declare_parameter("raw_topic", "/hermes/raw_input")
        self.declare_parameter("packet_topic", "/hermes/command_packets")
        self.declare_parameter("gesture_state_topic", "/hermes/gesture_state")
        self.declare_parameter("deadman_always_true", False)

        raw_topic = str(self.get_parameter("raw_topic").value)
        packet_topic = str(self.get_parameter("packet_topic").value)
        state_topic = str(self.get_parameter("gesture_state_topic").value)
        self._deadman_always_true = bool(self.get_parameter("deadman_always_true").value)

        self._state = GestureState()
        self._recognizer = GestureRecognizer()
        self._safety = SafetyEvaluator()

        self._packet_pub = self.create_publisher(String, packet_topic, 20)
        self._state_pub = self.create_publisher(String, state_topic, 20)
        self._raw_sub = self.create_subscription(String, raw_topic, self._on_raw, 50)

        self.get_logger().info(
            "Gesture pipeline ready. "
            f"raw_topic={raw_topic}, packet_topic={packet_topic}, state_topic={state_topic}, "
            f"deadman_always_true={self._deadman_always_true}"
        )

    def _decode_raw(self, data: str) -> Optional[Dict[str, Any]]:
        try:
            raw = json.loads(data)
            if not isinstance(raw, dict):
                self.get_logger().warning("Raw input must be a JSON object.")
                return None
            return raw
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid JSON payload: {exc}")
            return None

    def _publish_packet(self, packet: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(packet, separators=(",", ":"))
        self._packet_pub.publish(msg)

    def _publish_state(self) -> None:
        snapshot = {
            "mode": self._state.mode,
            "deadman_active": self._state.deadman_active,
            "selection_op": self._state.selection_op,
            "params": self._state.params,
            "modifiers": self._state.modifiers,
        }
        msg = String()
        msg.data = json.dumps(snapshot, separators=(",", ":"))
        self._state_pub.publish(msg)

    def _on_raw(self, msg: String) -> None:
        raw = self._decode_raw(msg.data)
        if raw is None:
            return

        if "time_ms" not in raw:
            raw["time_ms"] = int(self.get_clock().now().nanoseconds / 1_000_000)

        event = self._recognizer.recognize(raw)
        now_ms = int(raw["time_ms"])

        safety_packet = self._safety.tick(
            event,
            self._state,
            COMMAND_REGISTRY,
            now_ms,
            force_deadman_true=self._deadman_always_true,
        )
        if safety_packet:
            self._publish_packet(safety_packet)

        gesture_packet = match_gesture(event, self._state, COMMAND_REGISTRY)
        if gesture_packet:
            self._publish_packet(gesture_packet)

        self._publish_state()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GesturePipelineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
