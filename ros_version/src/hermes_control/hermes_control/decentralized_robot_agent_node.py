import json
import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from hermes_control.swarm.behavior_engine import execute_behavior
from hermes_control.swarm.formation_engine import FormationParams, compute_formation_targets


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # Standard ZYX yaw extraction.
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


class DecentralizedRobotAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("decentralized_robot_agent_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("intent_topic", "/hermes/swarm_intent")
        self.declare_parameter("robot_states_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("stop_on_missing_intent_ms", 600)
        self.declare_parameter("stop_on_missing_states_ms", 600)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 1.8)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 1.6)
        self.declare_parameter("position_tolerance_m", 0.08)
        self.declare_parameter("heading_tolerance_rad", 0.12)
        self.declare_parameter("heading_slowdown_rad", 0.8)

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        intent_topic = str(self.get_parameter("intent_topic").value)
        robot_states_topic = str(self.get_parameter("robot_states_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._control_hz = _as_float(self.get_parameter("control_hz").value, 20.0)
        self._missing_intent_ms = int(self.get_parameter("stop_on_missing_intent_ms").value)
        self._missing_states_ms = int(self.get_parameter("stop_on_missing_states_ms").value)
        self._kp_linear = _as_float(self.get_parameter("kp_linear").value, 0.8)
        self._kp_angular = _as_float(self.get_parameter("kp_angular").value, 1.8)
        self._max_linear = max(0.0, _as_float(self.get_parameter("max_linear_speed").value, 0.8))
        self._max_angular = max(0.0, _as_float(self.get_parameter("max_angular_speed").value, 1.6))
        self._pos_tol = max(0.0, _as_float(self.get_parameter("position_tolerance_m").value, 0.08))
        self._yaw_tol = max(0.0, _as_float(self.get_parameter("heading_tolerance_rad").value, 0.12))
        self._heading_slowdown = max(0.1, _as_float(self.get_parameter("heading_slowdown_rad").value, 0.8))

        self._intent: Optional[Dict[str, Any]] = None
        self._intent_rx_ns: int = 0
        self._robot_states: Dict[str, Dict[str, float]] = {}
        self._states_rx_ns: int = 0
        self._self_pose: Optional[Tuple[float, float, float]] = None

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 20)
        self._intent_sub = self.create_subscription(String, intent_topic, self._on_intent, 50)
        self._states_sub = self.create_subscription(String, robot_states_topic, self._on_robot_states, 50)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self._timer = self.create_timer(max(0.01, 1.0 / max(1e-6, self._control_hz)), self._tick)

        self.get_logger().info(
            f"Robot agent ready. robot_id={self._robot_id}, intent_topic={intent_topic}, "
            f"robot_states_topic={robot_states_topic}, odom_topic={odom_topic}, cmd_vel_topic={cmd_vel_topic}"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _is_stale(self, rx_ns: int, timeout_ms: int) -> bool:
        if rx_ns <= 0:
            return True
        age_ms = (self._now_ns() - rx_ns) / 1_000_000.0
        return age_ms > float(timeout_ms)

    def _on_intent(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return
        self._intent = payload
        self._intent_rx_ns = self._now_ns()

    def _normalize_state_map(self, payload: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
        robots_obj: Any
        if isinstance(payload.get("robots"), dict):
            robots_obj = payload["robots"]
        else:
            robots_obj = payload

        out: Dict[str, Dict[str, float]] = {}
        if not isinstance(robots_obj, dict):
            return out

        for key, value in robots_obj.items():
            rid = str(key).strip().lower()
            if not isinstance(value, dict):
                continue
            out[rid] = {
                "x": _as_float(value.get("x"), _as_float(value.get("X"), 0.0)),
                "y": _as_float(value.get("y"), _as_float(value.get("Y"), 0.0)),
                "yaw": _as_float(value.get("yaw"), _as_float(value.get("theta"), 0.0)),
            }
        return out

    def _on_robot_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return

        # Accept either full map payload {"robots": {...}} or single-beacon payload.
        if "robot_id" in payload and "x" in payload and "y" in payload:
            rid = str(payload.get("robot_id")).strip().lower()
            self._robot_states[rid] = {
                "x": _as_float(payload.get("x"), 0.0),
                "y": _as_float(payload.get("y"), 0.0),
                "yaw": _as_float(payload.get("yaw"), 0.0),
            }
        else:
            self._robot_states = self._normalize_state_map(payload)
        self._states_rx_ns = self._now_ns()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self._self_pose = (float(p.x), float(p.y), float(yaw))

    def _selection_from_intent(self, intent: Dict[str, Any]) -> List[str]:
        raw = intent.get("selection")
        if not isinstance(raw, list):
            return []
        return sorted(str(r).strip().lower() for r in raw)

    def _centroid_from_state(self, ids: List[str], fallback: Tuple[float, float]) -> Tuple[float, float]:
        pts: List[Tuple[float, float]] = []
        for rid in ids:
            st = self._robot_states.get(rid)
            if not st:
                continue
            pts.append((st["x"], st["y"]))
        if not pts:
            return fallback
        mx = sum(x for x, _ in pts) / float(len(pts))
        my = sum(y for _, y in pts) / float(len(pts))
        return (mx, my)

    def _home_xy(self, intent: Dict[str, Any], behavior_params: Dict[str, Any]) -> Tuple[float, float]:
        if isinstance(intent.get("home_xy"), dict):
            home = intent["home_xy"]
            return (_as_float(home.get("x"), 0.0), _as_float(home.get("y"), 0.0))
        if isinstance(behavior_params.get("home_xy"), dict):
            home = behavior_params["home_xy"]
            return (_as_float(home.get("x"), 0.0), _as_float(home.get("y"), 0.0))
        return (0.0, 0.0)

    def _path_waypoints(self, intent: Dict[str, Any], behavior_params: Dict[str, Any]) -> List[List[float]]:
        val = intent.get("path_waypoints")
        if isinstance(val, list):
            return val
        val = behavior_params.get("path_waypoints")
        if isinstance(val, list):
            return val
        return []

    def _compute_targets(self, intent: Dict[str, Any], selection: List[str]) -> Dict[str, Tuple[float, float, float]]:
        centroid_obj = intent.get("centroid", {})
        fallback_centroid = (
            _as_float(centroid_obj.get("x"), 0.0),
            _as_float(centroid_obj.get("y"), 0.0),
        )
        centroid = self._centroid_from_state(selection, fallback_centroid)
        heading = _as_float(intent.get("formation_heading"), 0.0)
        spacing = max(0.2, _as_float(intent.get("formation_spacing"), 1.0))

        behavior = intent.get("active_behavior")
        if isinstance(behavior, str) and behavior:
            params = dict(intent.get("behavior_params") or {})
            if "path_waypoints" not in params:
                params["path_waypoints"] = self._path_waypoints(intent, params)
            if behavior == "FOLLOW_ME_TOGGLE":
                params["follow_me_enabled"] = bool(params.get("follow_me_enabled", True))
            result = execute_behavior(
                behavior_name=behavior,
                robot_ids=selection,
                centroid_xy=centroid,
                behavior_params=params,
                heading_rad=heading,
                home_xy=self._home_xy(intent, params),
                active_formation_type=intent.get("active_formation_type"),
            )
            return result.targets

        formation = intent.get("active_formation_type")
        if isinstance(formation, str) and formation:
            return compute_formation_targets(
                formation,
                selection,
                centroid_xy=centroid,
                params=FormationParams(spacing_m=spacing, heading_rad=heading),
            )

        return {}

    def _publish_stop(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)

    def _publish_drive_stream(self, drive_cmd: Dict[str, Any]) -> None:
        vx = _as_float(drive_cmd.get("vx"), 0.0)
        omega = _as_float(drive_cmd.get("omega"), _as_float(drive_cmd.get("steer"), 0.0))
        msg = Twist()
        msg.linear.x = _clamp(vx, -self._max_linear, self._max_linear)
        msg.angular.z = _clamp(omega, -self._max_angular, self._max_angular)
        self._cmd_pub.publish(msg)

    def _current_pose(self) -> Optional[Tuple[float, float, float]]:
        if self._self_pose is not None:
            return self._self_pose
        own = self._robot_states.get(self._robot_id)
        if own is None:
            return None
        return (own["x"], own["y"], own["yaw"])

    def _publish_target_tracking(self, target: Tuple[float, float, float]) -> None:
        pose = self._current_pose()
        if pose is None:
            self._publish_stop()
            return

        x, y, yaw = pose
        tx, ty, tyaw = target
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        if dist > self._pos_tol:
            desired_yaw = math.atan2(dy, dx)
            heading_err = _wrap_to_pi(desired_yaw - yaw)
            v = _clamp(self._kp_linear * dist, -self._max_linear, self._max_linear)
            if abs(heading_err) > self._heading_slowdown:
                v *= 0.35
            w = _clamp(self._kp_angular * heading_err, -self._max_angular, self._max_angular)
        else:
            yaw_err = _wrap_to_pi(tyaw - yaw)
            v = 0.0
            if abs(yaw_err) <= self._yaw_tol:
                w = 0.0
            else:
                w = _clamp(self._kp_angular * yaw_err, -self._max_angular, self._max_angular)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub.publish(msg)

    def _tick(self) -> None:
        if self._intent is None or self._is_stale(self._intent_rx_ns, self._missing_intent_ms):
            self._publish_stop()
            return

        intent = self._intent
        if not bool(intent.get("deadman_active", False)) or bool(intent.get("paused", False)):
            self._publish_stop()
            return

        selection = self._selection_from_intent(intent)
        if self._robot_id not in selection:
            self._publish_stop()
            return

        mode = str(intent.get("mode") or "")
        drive_cmd = intent.get("drive_cmd_vel", {})
        if mode == "DRIVE" and isinstance(drive_cmd, dict) and drive_cmd:
            self._publish_drive_stream(drive_cmd)
            return

        if self._is_stale(self._states_rx_ns, self._missing_states_ms):
            self._publish_stop()
            return

        targets = self._compute_targets(intent, selection)
        target = targets.get(self._robot_id)
        if target is None:
            self._publish_stop()
            return

        self._publish_target_tracking(target)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DecentralizedRobotAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
