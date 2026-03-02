import json
import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


class RobotStateBeaconNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_beacon_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_topic", "/hermes/robot_state_beacon")

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        odom_topic = str(self.get_parameter("odom_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)

        self._state_pub = self.create_publisher(String, state_topic, 50)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)

        self.get_logger().info(
            f"Robot state beacon ready. robot_id={self._robot_id}, odom_topic={odom_topic}, state_topic={state_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        payload = {
            "schema": "hermes.robot_state_beacon.v1",
            "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
            "robot_id": self._robot_id,
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(_yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._state_pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotStateBeaconNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
