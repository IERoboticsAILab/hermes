import json
import math
import socket
from typing import Any, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _as_bool(value: Any, default: bool = False) -> bool:
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


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class LocalRobotIONode(Node):
    def __init__(self) -> None:
        super().__init__("local_robot_io_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", False)
        self.declare_parameter("cmd_vel_frame_id", "")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_tf_pose", True)
        self.declare_parameter("fallback_to_odom", True)
        self.declare_parameter("pose_udp_host", "127.0.0.1")
        self.declare_parameter("pose_udp_port", 15000)
        self.declare_parameter("cmd_udp_bind_host", "127.0.0.1")
        self.declare_parameter("cmd_udp_port", 15001)
        self.declare_parameter("cmd_timeout_ms", 500)

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_stamped = _as_bool(self.get_parameter("cmd_vel_stamped").value, False)
        self._cmd_vel_frame_id = str(self.get_parameter("cmd_vel_frame_id").value).strip()
        self._publish_hz = max(1.0, _as_float(self.get_parameter("publish_hz").value, 20.0))
        self._global_frame = str(self.get_parameter("global_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        self._use_tf_pose = _as_bool(self.get_parameter("use_tf_pose").value, True)
        self._fallback_to_odom = _as_bool(self.get_parameter("fallback_to_odom").value, True)

        pose_udp_host = str(self.get_parameter("pose_udp_host").value).strip()
        pose_udp_port = int(self.get_parameter("pose_udp_port").value)
        cmd_udp_bind_host = str(self.get_parameter("cmd_udp_bind_host").value).strip()
        cmd_udp_port = int(self.get_parameter("cmd_udp_port").value)
        self._cmd_timeout_ms = int(self.get_parameter("cmd_timeout_ms").value)

        self._latest_odom: Optional[Odometry] = None
        self._last_tf_warn_ns = 0
        self._last_cmd_rx_ns = 0
        self._active_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_published_cmd: Optional[Tuple[float, float, float]] = None

        self._pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pose_target = (pose_udp_host, pose_udp_port)

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind((cmd_udp_bind_host, cmd_udp_port))
        self._cmd_sock.setblocking(False)

        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        if self._cmd_vel_stamped:
            self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 20)
        else:
            self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 20)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._pose_timer = self.create_timer(max(0.01, 1.0 / self._publish_hz), self._pose_tick)
        self._cmd_timer = self.create_timer(0.02, self._command_tick)

        self.get_logger().info(
            f"Local robot I/O ready. robot_id={self._robot_id}, odom_topic={odom_topic}, cmd_vel_topic={cmd_vel_topic}, "
            f"pose_udp={pose_udp_host}:{pose_udp_port}, cmd_udp_bind={cmd_udp_bind_host}:{cmd_udp_port}, "
            f"use_tf_pose={self._use_tf_pose}, fallback_to_odom={self._fallback_to_odom}"
        )

    def destroy_node(self) -> bool:
        try:
            self._pose_sock.close()
        except OSError:
            pass
        try:
            self._cmd_sock.close()
        except OSError:
            pass
        return super().destroy_node()

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _send_pose(self, x: float, y: float, yaw: float, frame_id: str, vx: float, vy: float) -> None:
        payload = {
            "schema": "hermes.local_pose.v1",
            "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
            "robot_id": self._robot_id,
            "frame_id": frame_id,
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "vx": float(vx),
            "vy": float(vy),
        }
        self._pose_sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self._pose_target)

    def _pose_from_tf(self) -> bool:
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
        self._send_pose(float(tr.x), float(tr.y), float(yaw), self._global_frame, vx, vy)
        return True

    def _pose_from_odom(self) -> bool:
        if self._latest_odom is None:
            return False
        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        twist = self._latest_odom.twist.twist
        vx_body = float(twist.linear.x)
        vy_body = float(twist.linear.y)
        vx = (math.cos(yaw) * vx_body) - (math.sin(yaw) * vy_body)
        vy = (math.sin(yaw) * vx_body) + (math.cos(yaw) * vy_body)
        frame_id = str(self._latest_odom.header.frame_id or "odom")
        self._send_pose(float(p.x), float(p.y), float(yaw), frame_id, vx, vy)
        return True

    def _pose_tick(self) -> None:
        if self._use_tf_pose and self._pose_from_tf():
            return
        if self._fallback_to_odom:
            self._pose_from_odom()

    def _new_cmd_msg(self) -> Tuple[Any, Twist]:
        if self._cmd_vel_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            if self._cmd_vel_frame_id:
                msg.header.frame_id = self._cmd_vel_frame_id
            return msg, msg.twist
        msg = Twist()
        return msg, msg

    def _publish_cmd(self, vx: float, vy: float, omega: float) -> None:
        cmd = (float(vx), float(vy), float(omega))
        msg, twist = self._new_cmd_msg()
        twist.linear.x = cmd[0]
        twist.linear.y = cmd[1]
        twist.angular.z = cmd[2]
        self._cmd_pub.publish(msg)
        self._last_published_cmd = cmd

    def _drain_cmd_socket(self) -> None:
        while True:
            try:
                packet, _addr = self._cmd_sock.recvfrom(65535)
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
            if payload.get("schema") != "hermes.local_cmd.v1":
                continue
            rid = str(payload.get("robot_id", "")).strip().lower()
            if rid and rid != self._robot_id:
                continue

            self._active_cmd = (
                _as_float(payload.get("vx"), 0.0),
                _as_float(payload.get("vy"), 0.0),
                _as_float(payload.get("omega"), 0.0),
            )
            self._last_cmd_rx_ns = int(self.get_clock().now().nanoseconds)

    def _command_tick(self) -> None:
        self._drain_cmd_socket()
        now_ns = int(self.get_clock().now().nanoseconds)
        fresh = self._last_cmd_rx_ns > 0 and ((now_ns - self._last_cmd_rx_ns) / 1_000_000.0) <= float(self._cmd_timeout_ms)
        desired = self._active_cmd if fresh else (0.0, 0.0, 0.0)
        if self._last_published_cmd != desired:
            self._publish_cmd(*desired)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalRobotIONode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
