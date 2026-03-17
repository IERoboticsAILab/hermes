import json
import math
import socket
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
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


class DecentralizedRobotAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("decentralized_robot_agent_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("intent_topic", "/hermes/swarm_intent")
        self.declare_parameter("robot_states_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("bid_topic", "/hermes/slot_bids")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", False)
        self.declare_parameter("cmd_vel_frame_id", "")
        self.declare_parameter("enable_ros_local_io", True)
        self.declare_parameter("cmd_udp_host", "")
        self.declare_parameter("cmd_udp_port", 0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("expected_state_frame", "map")
        self.declare_parameter("require_global_state_frame_for_multi_robot", True)
        self.declare_parameter("stop_on_missing_intent_ms", 600)
        self.declare_parameter("stop_on_missing_states_ms", 600)
        self.declare_parameter("bid_timeout_ms", 400)
        self.declare_parameter("bid_settle_ms", 180)
        self.declare_parameter("assignment_lock_ms", 1000)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 1.8)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 1.6)
        self.declare_parameter("position_tolerance_m", 0.08)
        self.declare_parameter("heading_tolerance_rad", 0.12)
        self.declare_parameter("heading_slowdown_rad", 0.8)

        self.declare_parameter("enable_collision_avoidance", True)
        self.declare_parameter("neighbor_influence_radius_m", 3.0)
        self.declare_parameter("collision_time_horizon_s", 2.0)
        self.declare_parameter("robot_radius_m", 0.22)
        self.declare_parameter("safety_margin_m", 0.10)
        self.declare_parameter("velocity_samples", 20)

        self.declare_parameter("enable_deadlock_recovery", True)
        self.declare_parameter("deadlock_window_ms", 1800)
        self.declare_parameter("deadlock_progress_epsilon_m", 0.03)
        self.declare_parameter("deadlock_recovery_ms", 1200)
        self.declare_parameter("deadlock_turn_rate", 0.9)

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        intent_topic = str(self.get_parameter("intent_topic").value)
        robot_states_topic = str(self.get_parameter("robot_states_topic").value)
        bid_topic = str(self.get_parameter("bid_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_stamped = _as_bool(self.get_parameter("cmd_vel_stamped").value, False)
        self._cmd_vel_frame_id = str(self.get_parameter("cmd_vel_frame_id").value).strip()
        self._enable_ros_local_io = _as_bool(self.get_parameter("enable_ros_local_io").value, True)
        self._cmd_udp_host = str(self.get_parameter("cmd_udp_host").value).strip()
        self._cmd_udp_port = int(self.get_parameter("cmd_udp_port").value)

        self._control_hz = _as_float(self.get_parameter("control_hz").value, 20.0)
        self._expected_state_frame = str(self.get_parameter("expected_state_frame").value).strip()
        self._require_global_state_frame_for_multi_robot = _as_bool(
            self.get_parameter("require_global_state_frame_for_multi_robot").value,
            True,
        )
        self._missing_intent_ms = int(self.get_parameter("stop_on_missing_intent_ms").value)
        self._missing_states_ms = int(self.get_parameter("stop_on_missing_states_ms").value)
        self._bid_timeout_ms = int(self.get_parameter("bid_timeout_ms").value)
        self._bid_settle_ms = int(self.get_parameter("bid_settle_ms").value)
        self._assignment_lock_ms = int(self.get_parameter("assignment_lock_ms").value)

        self._kp_linear = _as_float(self.get_parameter("kp_linear").value, 0.8)
        self._kp_angular = _as_float(self.get_parameter("kp_angular").value, 1.8)
        self._max_linear = max(0.0, _as_float(self.get_parameter("max_linear_speed").value, 0.8))
        self._max_angular = max(0.0, _as_float(self.get_parameter("max_angular_speed").value, 1.6))
        self._pos_tol = max(0.0, _as_float(self.get_parameter("position_tolerance_m").value, 0.08))
        self._yaw_tol = max(0.0, _as_float(self.get_parameter("heading_tolerance_rad").value, 0.12))
        self._heading_slowdown = max(0.1, _as_float(self.get_parameter("heading_slowdown_rad").value, 0.8))

        self._enable_ca = _as_bool(self.get_parameter("enable_collision_avoidance").value, True)
        self._neighbor_radius = max(0.5, _as_float(self.get_parameter("neighbor_influence_radius_m").value, 3.0))
        self._time_horizon = max(0.2, _as_float(self.get_parameter("collision_time_horizon_s").value, 2.0))
        self._robot_radius = max(0.05, _as_float(self.get_parameter("robot_radius_m").value, 0.22))
        self._safety_margin = max(0.01, _as_float(self.get_parameter("safety_margin_m").value, 0.10))
        self._velocity_samples = max(8, int(self.get_parameter("velocity_samples").value))

        self._enable_deadlock_recovery = _as_bool(self.get_parameter("enable_deadlock_recovery").value, True)
        self._deadlock_window_ms = int(self.get_parameter("deadlock_window_ms").value)
        self._deadlock_eps = max(0.001, _as_float(self.get_parameter("deadlock_progress_epsilon_m").value, 0.03))
        self._deadlock_recovery_ms = int(self.get_parameter("deadlock_recovery_ms").value)
        self._deadlock_turn_rate = _as_float(self.get_parameter("deadlock_turn_rate").value, 0.9)

        self._intent: Optional[Dict[str, Any]] = None
        self._intent_rx_ns: int = 0
        self._robot_states: Dict[str, Dict[str, float]] = {}
        self._robot_state_rx_ns: Dict[str, int] = {}
        self._states_rx_ns: int = 0
        self._self_pose: Optional[Tuple[float, float, float]] = None
        self._self_vel_world: Tuple[float, float] = (0.0, 0.0)

        self._slot_bid_book: Dict[int, Dict[str, Dict[str, Any]]] = {}
        self._intent_seq_first_seen_ns: Dict[int, int] = {}
        self._assigned_slot: Optional[str] = None
        self._slot_lock_until_ns: int = 0

        self._last_goal_dist: Optional[float] = None
        self._stuck_since_ns: int = 0
        self._recovery_until_ns: int = 0
        self._last_geometry_warn_ns: int = 0
        self._use_udp_cmd = self._cmd_udp_port > 0 and bool(self._cmd_udp_host)

        self._cmd_pub = None
        if cmd_vel_topic and (self._enable_ros_local_io or not self._use_udp_cmd):
            if self._cmd_vel_stamped:
                self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 20)
            else:
                self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 20)
        self._cmd_udp_sock = None
        if self._use_udp_cmd:
            self._cmd_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._bid_pub = self.create_publisher(String, bid_topic, 50)
        self._intent_sub = self.create_subscription(String, intent_topic, self._on_intent, 50)
        self._states_sub = self.create_subscription(String, robot_states_topic, self._on_robot_states, 50)
        self._bid_sub = self.create_subscription(String, bid_topic, self._on_slot_bid, 50)
        self._odom_sub = None
        if self._enable_ros_local_io and odom_topic:
            self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self._timer = self.create_timer(max(0.01, 1.0 / max(1e-6, self._control_hz)), self._tick)

        self.get_logger().info(
            f"Robot agent ready. robot_id={self._robot_id}, intent_topic={intent_topic}, "
            f"robot_states_topic={robot_states_topic}, bid_topic={bid_topic}, odom_topic={odom_topic}, "
            f"cmd_vel_topic={cmd_vel_topic}, cmd_vel_stamped={self._cmd_vel_stamped}, "
            f"expected_state_frame={self._expected_state_frame}, enable_ros_local_io={self._enable_ros_local_io}, "
            f"cmd_udp_port={self._cmd_udp_port}"
        )

    def destroy_node(self) -> bool:
        if self._cmd_udp_sock is not None:
            try:
                self._cmd_udp_sock.close()
            except OSError:
                pass
        return super().destroy_node()

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
                "vx": _as_float(value.get("vx"), 0.0),
                "vy": _as_float(value.get("vy"), 0.0),
            }
        return out

    def _on_robot_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return

        now_ns = self._now_ns()

        # Accept either full map payload {"robots": {...}} or single-beacon payload.
        if "robot_id" in payload and "x" in payload and "y" in payload:
            frame_id = str(payload.get("frame_id", "")).strip()
            if self._expected_state_frame and frame_id and frame_id != self._expected_state_frame:
                return
            rid = str(payload.get("robot_id")).strip().lower()
            self._robot_states[rid] = {
                "x": _as_float(payload.get("x"), 0.0),
                "y": _as_float(payload.get("y"), 0.0),
                "yaw": _as_float(payload.get("yaw"), 0.0),
                "vx": _as_float(payload.get("vx"), 0.0),
                "vy": _as_float(payload.get("vy"), 0.0),
            }
            if rid == self._robot_id and not self._enable_ros_local_io:
                self._self_pose = (
                    self._robot_states[rid]["x"],
                    self._robot_states[rid]["y"],
                    self._robot_states[rid]["yaw"],
                )
                self._self_vel_world = (
                    self._robot_states[rid]["vx"],
                    self._robot_states[rid]["vy"],
                )
            self._robot_state_rx_ns[rid] = now_ns
        else:
            normalized = self._normalize_state_map(payload)
            self._robot_states = normalized
            self._robot_state_rx_ns = {rid: now_ns for rid in normalized.keys()}
            if self._robot_id in normalized and not self._enable_ros_local_io:
                own = normalized[self._robot_id]
                self._self_pose = (own["x"], own["y"], own["yaw"])
                self._self_vel_world = (own["vx"], own["vy"])
        self._states_rx_ns = now_ns

    def _on_slot_bid(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return

        if payload.get("schema") != "hermes.slot_bids.v1":
            return
        seq = int(payload.get("intent_seq", -1))
        if seq < 0:
            return
        rid = str(payload.get("robot_id", "")).strip().lower()
        costs = payload.get("costs", {})
        if not rid or not isinstance(costs, dict):
            return

        normalized_costs: Dict[str, float] = {}
        for sid, val in costs.items():
            normalized_costs[str(sid)] = _as_float(val, 1e9)

        per_seq = self._slot_bid_book.setdefault(seq, {})
        per_seq[rid] = {
            "stamp_ns": self._now_ns(),
            "costs": normalized_costs,
        }

        # Keep bid memory bounded.
        if len(self._slot_bid_book) > 8:
            for old_seq in sorted(self._slot_bid_book.keys())[:-8]:
                self._slot_bid_book.pop(old_seq, None)
                self._intent_seq_first_seen_ns.pop(old_seq, None)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self._self_pose = (float(p.x), float(p.y), float(yaw))

        # Twist is body-frame; rotate into world frame.
        twist = msg.twist.twist
        vx_body = float(twist.linear.x)
        vy_body = float(twist.linear.y)
        self._self_vel_world = (
            (math.cos(yaw) * vx_body) - (math.sin(yaw) * vy_body),
            (math.sin(yaw) * vx_body) + (math.cos(yaw) * vy_body),
        )

    def _selection_from_intent(self, intent: Dict[str, Any]) -> List[str]:
        raw = intent.get("selection")
        if not isinstance(raw, list):
            return []
        return sorted(str(r).strip().lower() for r in raw)

    def _fresh_state_for(self, rid: str, now_ns: Optional[int] = None) -> Optional[Dict[str, float]]:
        if now_ns is None:
            now_ns = self._now_ns()
        rx_ns = int(self._robot_state_rx_ns.get(rid, 0))
        if rx_ns <= 0:
            return None
        age_ms = (now_ns - rx_ns) / 1_000_000.0
        if age_ms > float(self._missing_states_ms):
            return None
        return self._robot_states.get(rid)

    def _fresh_selection(self, selection: List[str], now_ns: Optional[int] = None) -> List[str]:
        if now_ns is None:
            now_ns = self._now_ns()
        fresh: List[str] = []
        for rid in selection:
            if rid == self._robot_id:
                fresh.append(rid)
                continue
            if self._fresh_state_for(rid, now_ns) is not None:
                fresh.append(rid)
        return sorted(set(fresh))

    def _centroid_from_state(self, ids: List[str], fallback: Tuple[float, float]) -> Tuple[float, float]:
        now_ns = self._now_ns()
        pts: List[Tuple[float, float]] = []
        for rid in ids:
            st = self._fresh_state_for(rid, now_ns)
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

    def _slot_ids(self, n: int) -> List[str]:
        return [f"slot_{i:03d}" for i in range(n)]

    def _compute_slot_targets(
        self,
        intent: Dict[str, Any],
        selection: List[str],
    ) -> Dict[str, Tuple[float, float, float]]:
        centroid_obj = intent.get("centroid", {})
        fallback_centroid = (
            _as_float(centroid_obj.get("x"), 0.0),
            _as_float(centroid_obj.get("y"), 0.0),
        )
        centroid = self._centroid_from_state(selection, fallback_centroid)
        heading = _as_float(intent.get("formation_heading"), 0.0)
        spacing = max(0.2, _as_float(intent.get("formation_spacing"), 1.0))

        slot_ids = self._slot_ids(len(selection))
        behavior = intent.get("active_behavior")
        if isinstance(behavior, str) and behavior:
            params = dict(intent.get("behavior_params") or {})
            if "path_waypoints" not in params:
                params["path_waypoints"] = self._path_waypoints(intent, params)
            if behavior == "FOLLOW_ME_TOGGLE":
                params["follow_me_enabled"] = bool(params.get("follow_me_enabled", True))
            result = execute_behavior(
                behavior_name=behavior,
                robot_ids=slot_ids,
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
                slot_ids,
                centroid_xy=centroid,
                params=FormationParams(spacing_m=spacing, heading_rad=heading),
            )

        return {}

    def _publish_slot_bid(
        self,
        intent_seq: int,
        selection: List[str],
        slot_targets: Dict[str, Tuple[float, float, float]],
    ) -> None:
        if self._robot_id not in selection:
            return
        pose = self._current_pose()
        if pose is None:
            return
        x, y, yaw = pose

        costs: Dict[str, float] = {}
        for sid, (tx, ty, _tyaw) in slot_targets.items():
            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            heading_err = abs(_wrap_to_pi(math.atan2(dy, dx) - yaw)) if dist > 1e-9 else 0.0
            costs[sid] = float(dist + (0.12 * heading_err))

        payload = {
            "schema": "hermes.slot_bids.v1",
            "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
            "intent_seq": int(intent_seq),
            "robot_id": self._robot_id,
            "selection": selection,
            "costs": costs,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._bid_pub.publish(msg)

    def _resolve_assignment(
        self,
        intent_seq: int,
        selection: List[str],
        slot_ids: List[str],
    ) -> Tuple[Dict[str, str], bool]:
        if not selection or not slot_ids:
            return {}, False

        fallback = {rid: sid for rid, sid in zip(sorted(selection), slot_ids)}

        book = self._slot_bid_book.get(intent_seq, {})
        now_ns = self._now_ns()
        valid: Dict[str, Dict[str, float]] = {}
        for rid in selection:
            entry = book.get(rid)
            if not entry:
                continue
            age_ms = (now_ns - int(entry.get("stamp_ns", 0))) / 1_000_000.0
            if age_ms > float(self._bid_timeout_ms):
                continue
            costs = entry.get("costs")
            if isinstance(costs, dict):
                valid[rid] = {str(k): _as_float(v, 1e9) for k, v in costs.items()}

        first_seen_ns = self._intent_seq_first_seen_ns.setdefault(intent_seq, now_ns)
        settle_elapsed_ms = (now_ns - first_seen_ns) / 1_000_000.0

        # Require complete bid set for stable decentralized assignment.
        if len(valid) < len(selection):
            if settle_elapsed_ms < float(self._bid_settle_ms):
                return fallback, False
            return fallback, False

        pairs: List[Tuple[float, str, str]] = []
        for rid in selection:
            rcost = valid.get(rid)
            if not rcost:
                continue
            for sid in slot_ids:
                pairs.append((_as_float(rcost.get(sid), 1e9), rid, sid))

        pairs.sort(key=lambda t: (t[0], t[1], t[2]))
        assigned: Dict[str, str] = {}
        used_robots = set()
        used_slots = set()
        for cost, rid, sid in pairs:
            if rid in used_robots or sid in used_slots:
                continue
            assigned[rid] = sid
            used_robots.add(rid)
            used_slots.add(sid)
            if len(used_slots) >= len(slot_ids):
                break

        # Fill missing robots deterministically.
        remaining_slots = [sid for sid in slot_ids if sid not in used_slots]
        for rid in sorted(selection):
            if rid in assigned:
                continue
            if not remaining_slots:
                break
            assigned[rid] = remaining_slots.pop(0)

        return (assigned if len(assigned) == len(selection) else fallback), True

    def _stable_own_slot(self, desired_slot: Optional[str]) -> Optional[str]:
        if desired_slot is None:
            self._assigned_slot = None
            return None

        now_ns = self._now_ns()
        if (
            self._assigned_slot is None
            or desired_slot == self._assigned_slot
            or now_ns >= self._slot_lock_until_ns
        ):
            if desired_slot != self._assigned_slot:
                self._assigned_slot = desired_slot
                self._slot_lock_until_ns = now_ns + int(self._assignment_lock_ms * 1_000_000)
        return self._assigned_slot

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
        if self._cmd_udp_sock is not None and self._use_udp_cmd:
            payload = {
                "schema": "hermes.local_cmd.v1",
                "stamp_ms": int(self.get_clock().now().nanoseconds / 1_000_000),
                "robot_id": self._robot_id,
                "vx": float(vx),
                "vy": float(vy),
                "omega": float(omega),
                "frame_id": self._cmd_vel_frame_id,
            }
            try:
                self._cmd_udp_sock.sendto(
                    json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                    (self._cmd_udp_host, self._cmd_udp_port),
                )
            except OSError:
                pass
            return
        if self._cmd_pub is None:
            return
        msg, twist = self._new_cmd_msg()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(omega)
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_drive_stream(self, drive_cmd: Dict[str, Any]) -> None:
        vx = _as_float(drive_cmd.get("vx"), 0.0)
        vy = _as_float(drive_cmd.get("vy"), 0.0)
        omega = _as_float(drive_cmd.get("omega"), _as_float(drive_cmd.get("steer"), 0.0))
        self._publish_cmd(
            _clamp(vx, -self._max_linear, self._max_linear),
            _clamp(vy, -self._max_linear, self._max_linear),
            _clamp(omega, -self._max_angular, self._max_angular),
        )

    def _current_pose(self) -> Optional[Tuple[float, float, float]]:
        if self._self_pose is not None:
            return self._self_pose
        own = self._fresh_state_for(self._robot_id)
        if own is None:
            return None
        return (own["x"], own["y"], own["yaw"])

    def _neighbors(self, selection: List[str], own_x: float, own_y: float) -> List[Dict[str, float]]:
        now_ns = self._now_ns()
        out: List[Dict[str, float]] = []
        for rid in selection:
            if rid == self._robot_id:
                continue
            st = self._fresh_state_for(rid, now_ns)
            if not st:
                continue
            dx = st["x"] - own_x
            dy = st["y"] - own_y
            if math.hypot(dx, dy) > self._neighbor_radius:
                continue
            out.append(st)
        return out

    def _sampled_ca_velocity(
        self,
        preferred_world_v: Tuple[float, float],
        own_pos: Tuple[float, float],
        neighbors: List[Dict[str, float]],
    ) -> Tuple[float, float]:
        if (not self._enable_ca) or (not neighbors):
            return preferred_world_v

        own_vx, own_vy = self._self_vel_world
        pref_vx, pref_vy = preferred_world_v
        safe_sep = (2.0 * self._robot_radius) + self._safety_margin

        candidates: List[Tuple[float, float]] = [
            (pref_vx, pref_vy),
            (own_vx, own_vy),
            (0.0, 0.0),
        ]
        scales = [1.0, 0.85, 0.65, 0.45, 0.25]
        for scale in scales:
            speed = self._max_linear * scale
            for k in range(self._velocity_samples):
                ang = (2.0 * math.pi * k) / float(self._velocity_samples)
                candidates.append((speed * math.cos(ang), speed * math.sin(ang)))

        best = candidates[0]
        best_cost = float("inf")

        for cand_vx, cand_vy in candidates:
            pref_err = math.hypot(cand_vx - pref_vx, cand_vy - pref_vy)
            smooth_err = math.hypot(cand_vx - own_vx, cand_vy - own_vy)
            cost = (1.0 * pref_err) + (0.35 * smooth_err)
            violation = 0.0
            min_sep = float("inf")

            for nb in neighbors:
                rx = nb["x"] - own_pos[0]
                ry = nb["y"] - own_pos[1]
                rvx = cand_vx - nb.get("vx", 0.0)
                rvy = cand_vy - nb.get("vy", 0.0)
                v2 = (rvx * rvx) + (rvy * rvy)
                if v2 < 1e-8:
                    t = 0.0
                else:
                    t = _clamp(-((rx * rvx) + (ry * rvy)) / v2, 0.0, self._time_horizon)

                sx = rx + (rvx * t)
                sy = ry + (rvy * t)
                sep = math.hypot(sx, sy)
                min_sep = min(min_sep, sep)
                if sep < safe_sep:
                    violation = max(violation, safe_sep - sep)

            if violation > 0.0:
                cost += 100.0 + (40.0 * violation)
            else:
                cost += -0.05 * min_sep

            if cost < best_cost:
                best_cost = cost
                best = (cand_vx, cand_vy)

        return best

    def _maybe_deadlock_recovery(self, dist_to_goal: float) -> bool:
        if not self._enable_deadlock_recovery:
            return False

        now_ns = self._now_ns()
        if now_ns < self._recovery_until_ns:
            sign = -1.0 if (sum(ord(c) for c in self._robot_id) % 2 == 0) else 1.0
            self._publish_cmd(0.0, 0.0, _clamp(sign * self._deadlock_turn_rate, -self._max_angular, self._max_angular))
            return True

        if self._last_goal_dist is None or dist_to_goal < (self._last_goal_dist - self._deadlock_eps):
            self._stuck_since_ns = now_ns
        elif self._stuck_since_ns == 0:
            self._stuck_since_ns = now_ns

        self._last_goal_dist = dist_to_goal
        if self._stuck_since_ns > 0:
            stalled_ms = (now_ns - self._stuck_since_ns) / 1_000_000.0
            if stalled_ms >= float(self._deadlock_window_ms):
                self._recovery_until_ns = now_ns + int(self._deadlock_recovery_ms * 1_000_000)
                self._stuck_since_ns = now_ns
                return True
        return False

    def _publish_target_tracking(self, target: Tuple[float, float, float], selection: List[str]) -> None:
        pose = self._current_pose()
        if pose is None:
            self._publish_stop()
            return

        x, y, yaw = pose
        tx, ty, tyaw = target
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        if dist <= self._pos_tol:
            yaw_err = _wrap_to_pi(tyaw - yaw)
            omega = 0.0
            if abs(yaw_err) <= self._yaw_tol:
                omega = 0.0
            else:
                omega = _clamp(self._kp_angular * yaw_err, -self._max_angular, self._max_angular)
            self._publish_cmd(0.0, 0.0, omega)
            self._last_goal_dist = dist
            self._stuck_since_ns = self._now_ns()
            return

        if self._maybe_deadlock_recovery(dist):
            return

        # Preferred world velocity toward goal.
        dir_x = dx / max(dist, 1e-9)
        dir_y = dy / max(dist, 1e-9)
        pref_speed = _clamp(self._kp_linear * dist, 0.0, self._max_linear)
        heading_to_goal = math.atan2(dy, dx)
        heading_err = abs(_wrap_to_pi(heading_to_goal - yaw))
        if heading_err > self._heading_slowdown:
            pref_speed *= 0.35
        preferred_world_v = (pref_speed * dir_x, pref_speed * dir_y)

        safe_world_v = self._sampled_ca_velocity(
            preferred_world_v,
            own_pos=(x, y),
            neighbors=self._neighbors(selection, x, y),
        )

        safe_speed = math.hypot(safe_world_v[0], safe_world_v[1])
        if safe_speed < 1e-4:
            desired_heading = heading_to_goal
        else:
            desired_heading = math.atan2(safe_world_v[1], safe_world_v[0])
        heading_err = _wrap_to_pi(desired_heading - yaw)

        self._publish_cmd(
            _clamp(safe_speed * max(0.0, math.cos(heading_err)), -self._max_linear, self._max_linear),
            0.0,
            _clamp(self._kp_angular * heading_err, -self._max_angular, self._max_angular),
        )

    def _geometry_ready(self, selection: List[str]) -> bool:
        if len(selection) <= 1 or not self._require_global_state_frame_for_multi_robot:
            return True
        frame = self._expected_state_frame.strip().lower()
        if frame not in {"", "odom", "base_link", "base_footprint"}:
            return True
        now_ns = self._now_ns()
        if (now_ns - self._last_geometry_warn_ns) > 2_000_000_000:
            self._last_geometry_warn_ns = now_ns
            self.get_logger().warning(
                "Multi-robot formation/behavior disabled because expected_state_frame is a local frame "
                f"('{self._expected_state_frame}'). Use a shared global frame such as 'map' with TF-based beacons."
            )
        return False

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
            self._assigned_slot = None
            return

        mode = str(intent.get("mode") or "")
        drive_cmd = intent.get("drive_cmd_vel", {})
        if mode == "DRIVE" and isinstance(drive_cmd, dict) and drive_cmd:
            self._publish_drive_stream(drive_cmd)
            return

        if self._is_stale(self._states_rx_ns, self._missing_states_ms):
            self._publish_stop()
            return

        now_ns = self._now_ns()
        active_selection = self._fresh_selection(selection, now_ns)
        if self._robot_id not in active_selection:
            self._publish_stop()
            self._assigned_slot = None
            return

        if not self._geometry_ready(active_selection):
            self._publish_stop()
            self._assigned_slot = None
            return

        slot_targets = self._compute_slot_targets(intent, active_selection)
        if not slot_targets:
            self._publish_stop()
            return

        intent_seq = int(intent.get("seq", 0))
        self._intent_seq_first_seen_ns.setdefault(intent_seq, now_ns)
        if len(self._intent_seq_first_seen_ns) > 8:
            for old_seq in sorted(self._intent_seq_first_seen_ns.keys())[:-8]:
                self._intent_seq_first_seen_ns.pop(old_seq, None)

        self._publish_slot_bid(intent_seq, active_selection, slot_targets)

        slot_ids = sorted(slot_targets.keys())
        assignment, bids_complete = self._resolve_assignment(intent_seq, active_selection, slot_ids)
        if bids_complete:
            desired_slot = assignment.get(self._robot_id)
        else:
            # Keep current slot while waiting for complete bids for this intent epoch.
            desired_slot = self._assigned_slot or assignment.get(self._robot_id)
        own_slot = self._stable_own_slot(desired_slot)
        if own_slot is None:
            self._publish_stop()
            return

        own_target = slot_targets.get(own_slot)
        if own_target is None:
            self._publish_stop()
            return

        self._publish_target_tracking(own_target, active_selection)


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
