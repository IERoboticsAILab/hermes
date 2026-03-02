import json
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String

from hermes_control.gestures.models import GestureState
from hermes_control.swarm.swarm_controller import SwarmController


class SwarmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("swarm_control_node")

        self.declare_parameter("packet_topic", "/hermes/command_packets")
        self.declare_parameter("centroid_topic", "/hermes/centroid")
        self.declare_parameter("swarm_state_topic", "/hermes/swarm_state")
        self.declare_parameter("swarm_intent_topic", "/hermes/swarm_intent")
        self.declare_parameter("robot_ids", ["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8"])

        packet_topic = str(self.get_parameter("packet_topic").value)
        centroid_topic = str(self.get_parameter("centroid_topic").value)
        swarm_state_topic = str(self.get_parameter("swarm_state_topic").value)
        swarm_intent_topic = str(self.get_parameter("swarm_intent_topic").value)
        robot_ids = [str(v) for v in self.get_parameter("robot_ids").value]

        self._centroid: Tuple[float, float] = (0.0, 0.0)
        self._gesture_state = GestureState()
        self._swarm = SwarmController(robot_ids=robot_ids)
        self._intent_seq = 0

        self._state_pub = self.create_publisher(String, swarm_state_topic, 20)
        self._intent_pub = self.create_publisher(String, swarm_intent_topic, 20)
        self._packet_sub = self.create_subscription(String, packet_topic, self._on_packet, 50)
        self._centroid_sub = self.create_subscription(Point, centroid_topic, self._on_centroid, 20)

        self.get_logger().info(
            f"Swarm control ready. packet_topic={packet_topic}, centroid_topic={centroid_topic}, "
            f"state_topic={swarm_state_topic}, intent_topic={swarm_intent_topic}"
        )

    def _on_centroid(self, msg: Point) -> None:
        self._centroid = (float(msg.x), float(msg.y))

    def _decode_packet(self, data: str) -> Optional[Dict[str, Any]]:
        try:
            packet = json.loads(data)
            if not isinstance(packet, dict):
                self.get_logger().warning("Packet must be a JSON object.")
                return None
            return packet
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid packet JSON: {exc}")
            return None

    def _snapshot(self) -> Dict[str, Any]:
        return {
            "centroid": {"x": self._centroid[0], "y": self._centroid[1]},
            "gesture_state": {
                "mode": self._gesture_state.mode,
                "deadman_active": self._gesture_state.deadman_active,
                "selection_op": self._gesture_state.selection_op,
                "params": self._gesture_state.params,
                "modifiers": self._gesture_state.modifiers,
            },
            "swarm": {
                "selection": sorted(self._swarm.selection),
                "last_selection": sorted(self._swarm.last_selection),
                "groups": {k: sorted(v) for k, v in self._swarm.groups.items()},
                "pending_formation_type": self._swarm.pending_formation_type,
                "active_formation_type": self._swarm.active_formation_type,
                "formation_heading": self._swarm.formation_heading,
                "formation_spacing": self._swarm.formation_spacing,
                "active_behavior": self._swarm.active_behavior,
                "behavior_params": self._swarm.behavior_params,
                "paused": self._swarm.paused,
                "last_targets": self._swarm.last_targets,
                "last_cmd_vel": self._swarm.last_cmd_vel,
            },
        }

    def _intent_snapshot(self) -> Dict[str, Any]:
        self._intent_seq += 1
        return {
            "type": "SWARM_INTENT",
            "schema": "hermes.swarm_intent.v1",
            "seq": self._intent_seq,
            "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
            "mode": self._gesture_state.mode,
            "deadman_active": self._gesture_state.deadman_active,
            "paused": self._swarm.paused,
            "selection": sorted(self._swarm.selection),
            "robot_ids": sorted(self._swarm.robot_ids),
            "centroid": {"x": self._centroid[0], "y": self._centroid[1]},
            "active_formation_type": self._swarm.active_formation_type,
            "formation_heading": self._swarm.formation_heading,
            "formation_spacing": self._swarm.formation_spacing,
            "active_behavior": self._swarm.active_behavior,
            "behavior_params": self._swarm.behavior_params,
            "home_xy": {"x": self._swarm.home_xy[0], "y": self._swarm.home_xy[1]},
            "path_waypoints": [list(p) for p in self._swarm.path_waypoints],
            "drive_cmd_vel": self._swarm.last_cmd_vel,
            "groups": {k: sorted(v) for k, v in self._swarm.groups.items()},
        }

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._snapshot(), separators=(",", ":"))
        self._state_pub.publish(msg)

    def _publish_intent(self) -> None:
        msg = String()
        msg.data = json.dumps(self._intent_snapshot(), separators=(",", ":"))
        self._intent_pub.publish(msg)

    def _on_packet(self, msg: String) -> None:
        packet = self._decode_packet(msg.data)
        if packet is None:
            return

        self._swarm.handle_packet(packet, self._gesture_state, centroid_xy=self._centroid)
        self._publish_state()
        self._publish_intent()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SwarmControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
