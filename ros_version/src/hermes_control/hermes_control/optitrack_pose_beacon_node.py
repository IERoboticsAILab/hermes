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


def _normalize_axis_spec(value: str, default: str) -> str:
    spec = str(value).strip().lower()
    return spec if spec in {"x", "y", "z", "-x", "-y", "-z"} else default


def _component_from_xyz(xyz: Tuple[float, float, float], axis_spec: str) -> float:
    sign = -1.0 if axis_spec.startswith("-") else 1.0
    axis = axis_spec[-1]
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    return sign * float(xyz[idx])


def _rotation_matrix_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[Tuple[float, float, float], ...]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - (2.0 * (yy + zz)), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - (2.0 * (xx + zz)), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - (2.0 * (xx + yy))),
    )


def _world_vector_from_local_axis(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    axis_spec: str,
) -> Tuple[float, float, float]:
    sign = -1.0 if axis_spec.startswith("-") else 1.0
    axis = axis_spec[-1]
    local = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[axis]
    rot = _rotation_matrix_from_quaternion(qx, qy, qz, qw)
    lx, ly, lz = local
    wx = (rot[0][0] * lx) + (rot[0][1] * ly) + (rot[0][2] * lz)
    wy = (rot[1][0] * lx) + (rot[1][1] * ly) + (rot[1][2] * lz)
    wz = (rot[2][0] * lx) + (rot[2][1] * ly) + (rot[2][2] * lz)
    return (sign * wx, sign * wy, sign * wz)


def _planar_heading_from_quaternion(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    forward_axis: str,
    planar_x_axis: str,
    planar_y_axis: str,
) -> float:
    world_forward = _world_vector_from_local_axis(qx, qy, qz, qw, forward_axis)
    fx = _component_from_xyz(world_forward, planar_x_axis)
    fy = _component_from_xyz(world_forward, planar_y_axis)
    if math.hypot(fx, fy) <= 1e-9:
        return _yaw_from_quaternion(qx, qy, qz, qw)
    return math.atan2(fy, fx)


class OptiTrackPoseBeaconNode(Node):
    def __init__(self) -> None:
        super().__init__("optitrack_pose_beacon_node")

        self.declare_parameter("state_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("frame_id", "optitrack")
        self.declare_parameter("robot_ids", [])
        self.declare_parameter("rigid_body_names", [])
        self.declare_parameter("robot_id", "")
        self.declare_parameter("rigid_body_name", "")
        self.declare_parameter("planar_x_axis", "x")
        self.declare_parameter("planar_y_axis", "y")
        self.declare_parameter("forward_axis", "x")

        state_topic = str(self.get_parameter("state_topic").value).strip()
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or "optitrack"
        self._planar_x_axis = _normalize_axis_spec(str(self.get_parameter("planar_x_axis").value), "x")
        self._planar_y_axis = _normalize_axis_spec(str(self.get_parameter("planar_y_axis").value), "y")
        self._forward_axis = _normalize_axis_spec(str(self.get_parameter("forward_axis").value), "x")
        if self._planar_x_axis[-1] == self._planar_y_axis[-1]:
            raise ValueError("planar_x_axis and planar_y_axis must reference different world axes")

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
            f"rigid_bodies={sorted(self._body_to_robot.items())}, "
            f"planar_axes=({self._planar_x_axis},{self._planar_y_axis}), forward_axis={self._forward_axis}"
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
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)
        pos_xyz = (float(p.x), float(p.y), float(p.z))
        x = _component_from_xyz(pos_xyz, self._planar_x_axis)
        y = _component_from_xyz(pos_xyz, self._planar_y_axis)
        yaw = _planar_heading_from_quaternion(
            qx,
            qy,
            qz,
            qw,
            self._forward_axis,
            self._planar_x_axis,
            self._planar_y_axis,
        )

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
