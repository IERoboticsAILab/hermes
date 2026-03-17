import json
import math
import socket
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _as_bool(value, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "t", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "f", "no", "n", "off", ""}:
            return False
    return default


class RobotStateBeaconNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_beacon_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("enable_ros_local_io", True)
        self.declare_parameter("use_tf_pose", True)
        self.declare_parameter("fallback_to_odom", True)
        self.declare_parameter("pose_udp_bind_host", "")
        self.declare_parameter("pose_udp_port", 0)
        self.declare_parameter("pose_udp_timeout_ms", 500)

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        odom_topic = str(self.get_parameter("odom_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._global_frame = str(self.get_parameter("global_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        self._enable_ros_local_io = _as_bool(self.get_parameter("enable_ros_local_io").value, True)
        self._use_tf_pose = _as_bool(self.get_parameter("use_tf_pose").value, True)
        self._fallback_to_odom = _as_bool(self.get_parameter("fallback_to_odom").value, True)
        self._pose_udp_bind_host = str(self.get_parameter("pose_udp_bind_host").value).strip()
        self._pose_udp_port = int(self.get_parameter("pose_udp_port").value)
        self._pose_udp_timeout_ms = int(self.get_parameter("pose_udp_timeout_ms").value)

        self._latest_odom: Optional[Odometry] = None
        self._last_tf_warn_ns = 0
        self._latest_udp_pose: Optional[dict] = None
        self._latest_udp_pose_ns: int = 0

        self._state_pub = self.create_publisher(String, state_topic, 50)
        self._odom_sub = None
        if self._enable_ros_local_io and odom_topic:
            self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._timer = self.create_timer(max(0.01, 1.0 / max(1e-6, self._publish_hz)), self._tick)

        self._pose_udp_sock = None
        if self._pose_udp_port > 0:
            self._pose_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._pose_udp_sock.bind((self._pose_udp_bind_host, self._pose_udp_port))
            self._pose_udp_sock.setblocking(False)

        self.get_logger().info(
            f"Robot state beacon ready. robot_id={self._robot_id}, odom_topic={odom_topic}, state_topic={state_topic}, "
            f"use_tf_pose={self._use_tf_pose}, global_frame={self._global_frame}, base_frame={self._base_frame}, "
            f"pose_udp_port={self._pose_udp_port}, enable_ros_local_io={self._enable_ros_local_io}"
        )

    def destroy_node(self) -> bool:
        if self._pose_udp_sock is not None:
            try:
                self._pose_udp_sock.close()
            except OSError:
                pass
        return super().destroy_node()

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _publish_pose(self, x: float, y: float, yaw: float, frame_id: str, vx: float, vy: float) -> None:
        payload = {
            "schema": "hermes.robot_state_beacon.v1",
            "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
            "robot_id": self._robot_id,
            "frame_id": frame_id,
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "vx": float(vx),
            "vy": float(vy),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._state_pub.publish(out)

    def _publish_from_tf(self) -> bool:
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            now_ns = int(self.get_clock().now().nanoseconds)
            if (now_ns - self._last_tf_warn_ns) > 2_000_000_000:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warning(f"TF lookup failed {self._global_frame}->{self._base_frame}: {exc}")
            return False

        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        yaw = _yaw_from_quaternion(float(rot.x), float(rot.y), float(rot.z), float(rot.w))
        vx = 0.0
        vy = 0.0
        if self._latest_odom is not None:
            twist = self._latest_odom.twist.twist
            vx_body = float(twist.linear.x)
            vy_body = float(twist.linear.y)
            vx = (math.cos(yaw) * vx_body) - (math.sin(yaw) * vy_body)
            vy = (math.sin(yaw) * vx_body) + (math.cos(yaw) * vy_body)

        self._publish_pose(float(tr.x), float(tr.y), float(yaw), self._global_frame, vx, vy)
        return True

    def _publish_from_odom(self) -> bool:
        if self._latest_odom is None:
            return False
        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        frame_id = str(self._latest_odom.header.frame_id or "odom")
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        twist = self._latest_odom.twist.twist
        vx_body = float(twist.linear.x)
        vy_body = float(twist.linear.y)
        vx = (math.cos(yaw) * vx_body) - (math.sin(yaw) * vy_body)
        vy = (math.sin(yaw) * vx_body) + (math.cos(yaw) * vy_body)
        self._publish_pose(float(p.x), float(p.y), float(yaw), frame_id, vx, vy)
        return True

    def _drain_udp_pose(self) -> None:
        if self._pose_udp_sock is None:
            return
        while True:
            try:
                packet, _addr = self._pose_udp_sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return

            try:
                payload = json.loads(packet.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(payload, dict):
                continue
            if payload.get("schema") != "hermes.local_pose.v1":
                continue
            rid = str(payload.get("robot_id", "")).strip().lower()
            if rid and rid != self._robot_id:
                continue
            self._latest_udp_pose = payload
            self._latest_udp_pose_ns = int(self.get_clock().now().nanoseconds)

    def _publish_from_udp_pose(self) -> bool:
        self._drain_udp_pose()
        if self._latest_udp_pose is None or self._latest_udp_pose_ns <= 0:
            return False
        age_ms = (int(self.get_clock().now().nanoseconds) - self._latest_udp_pose_ns) / 1_000_000.0
        if age_ms > float(self._pose_udp_timeout_ms):
            return False

        payload = self._latest_udp_pose
        self._publish_pose(
            float(payload.get("x", 0.0)),
            float(payload.get("y", 0.0)),
            float(payload.get("yaw", 0.0)),
            str(payload.get("frame_id", "")),
            float(payload.get("vx", 0.0)),
            float(payload.get("vy", 0.0)),
        )
        return True

    def _tick(self) -> None:
        if self._publish_from_udp_pose():
            return
        if not self._enable_ros_local_io:
            return
        if self._use_tf_pose and self._publish_from_tf():
            return
        if self._fallback_to_odom:
            self._publish_from_odom()


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
