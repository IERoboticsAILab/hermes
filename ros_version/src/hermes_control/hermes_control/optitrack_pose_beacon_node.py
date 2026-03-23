import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


class OptiTrackPoseBeaconNode(Node):
    def __init__(self) -> None:
        super().__init__("optitrack_pose_beacon_node")

        self.declare_parameter("state_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("frame_id", "optitrack")
        self.declare_parameter("robot_ids", [])
        self.declare_parameter("rigid_body_names", [])
        self.declare_parameter("robot_id", "")
        self.declare_parameter("rigid_body_name", "")

        state_topic = str(self.get_parameter("state_topic").value).strip()
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or "optitrack"

        robot_ids = [str(v).strip().lower() for v in list(self.get_parameter("robot_ids").value or []) if str(v).strip()]
        rigid_body_names = [str(v).strip() for v in list(self.get_parameter("rigid_body_names").value or []) if str(v).strip()]

        single_robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        single_rigid_body = str(self.get_parameter("rigid_body_name").value).strip()

        self._body_to_robot: Dict[str, str] = {}
        if robot_ids and rigid_body_names:
            if len(robot_ids) != len(rigid_body_names):
                raise ValueError("robot_ids and rigid_body_names must have the same length")
            self._body_to_robot.update({body: rid for rid, body in zip(robot_ids, rigid_body_names)})
        elif single_robot_id and single_rigid_body:
            self._body_to_robot[single_rigid_body] = single_robot_id
        else:
            raise ValueError(
                "Provide either robot_ids + rigid_body_names lists, or robot_id + rigid_body_name for single-robot mode"
            )

        self._state_pub = self.create_publisher(String, state_topic, 50)
        self._last_pose_by_robot: Dict[str, Tuple[float, float, int]] = {}
        self._subs = []

        for body_name, robot_id in sorted(self._body_to_robot.items()):
            topic = f"/{body_name}/pose"
            self._subs.append(self.create_subscription(PoseStamped, topic, self._make_pose_cb(robot_id, body_name), 20))

        self.get_logger().info(
            f"OptiTrack beacon bridge ready. frame_id={self._frame_id}, state_topic={state_topic}, "
            f"rigid_bodies={sorted(self._body_to_robot.items())}"
        )

    def _make_pose_cb(self, robot_id: str, body_name: str):
        def _cb(msg: PoseStamped) -> None:
            self._publish_beacon(robot_id, body_name, msg)

        return _cb

    @staticmethod
    def _stamp_ns(msg: PoseStamped) -> int:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        total = (sec * 1_000_000_000) + nanosec
        return total if total > 0 else 0

    def _publish_beacon(self, robot_id: str, body_name: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        x = float(p.x)
        y = float(p.y)
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))

        now_ns = self._stamp_ns(msg)
        if now_ns <= 0:
            now_ns = int(self.get_clock().now().nanoseconds)

        vx = 0.0
        vy = 0.0
        prev = self._last_pose_by_robot.get(robot_id)
        if prev is not None:
            prev_x, prev_y, prev_ns = prev
            dt = (now_ns - prev_ns) / 1_000_000_000.0
            if dt > 1e-4:
                vx = (x - prev_x) / dt
                vy = (y - prev_y) / dt
        self._last_pose_by_robot[robot_id] = (x, y, now_ns)

        payload = {
            "schema": "hermes.robot_state_beacon.v1",
            "stamp_ms": int(now_ns / 1_000_000),
            "robot_id": robot_id,
            "rigid_body_name": body_name,
            "frame_id": self._frame_id,
            "x": x,
            "y": y,
            "yaw": float(yaw),
            "vx": float(vx),
            "vy": float(vy),
        }

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._state_pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OptiTrackPoseBeaconNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
