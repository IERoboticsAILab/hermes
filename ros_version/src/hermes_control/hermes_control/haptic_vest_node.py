import json
import math
import os
import termios
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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


def _windowed_pulse(now_ms: int, period_ms: int, windows: Sequence[Tuple[int, int]]) -> bool:
    if period_ms <= 0:
        return False
    phase = now_ms % period_ms
    return any(start_ms <= phase < end_ms for start_ms, end_ms in windows)


def _nearest_neighbor_mean(points: Sequence[Tuple[float, float]]) -> Optional[float]:
    if len(points) < 2:
        return None

    nearest: List[float] = []
    for idx, (x0, y0) in enumerate(points):
        best = float("inf")
        for jdx, (x1, y1) in enumerate(points):
            if idx == jdx:
                continue
            dist = math.hypot(x1 - x0, y1 - y0)
            if dist < best:
                best = dist
        if math.isfinite(best):
            nearest.append(best)

    if not nearest:
        return None
    return float(sum(nearest) / len(nearest))


class _SerialWriter:
    _BAUD_MAP = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    _OPTIONAL_BAUDS = {
        460800: getattr(termios, "B460800", None),
        921600: getattr(termios, "B921600", None),
    }
    _BAUD_MAP.update({rate: code for rate, code in _OPTIONAL_BAUDS.items() if code is not None})

    def __init__(self, logger, port: str, baud_rate: int, retry_s: float) -> None:
        self._logger = logger
        self._port = port
        self._baud_rate = baud_rate
        self._retry_ns = int(max(0.2, retry_s) * 1_000_000_000)
        self._fd: Optional[int] = None
        self._next_retry_ns = 0
        self._warned_missing_baud = False

    def close(self) -> None:
        if self._fd is None:
            return
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._fd = None

    def ensure_open(self, now_ns: int) -> bool:
        if self._fd is not None:
            return True
        if now_ns < self._next_retry_ns:
            return False

        speed = self._BAUD_MAP.get(self._baud_rate)
        if speed is None:
            if not self._warned_missing_baud:
                self._logger.error(f"Unsupported serial baud rate: {self._baud_rate}")
                self._warned_missing_baud = True
            self._next_retry_ns = now_ns + self._retry_ns
            return False

        try:
            fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcflush(fd, termios.TCIOFLUSH)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            self._fd = fd
            self._logger.info(f"Opened vest serial port {self._port} @ {self._baud_rate} baud")
            return True
        except OSError as exc:
            self._next_retry_ns = now_ns + self._retry_ns
            self._logger.warning(f"Failed to open vest serial port {self._port}: {exc}")
            return False

    def write_line(self, payload: str, now_ns: int) -> bool:
        if not self.ensure_open(now_ns):
            return False
        assert self._fd is not None
        try:
            os.write(self._fd, payload.encode("ascii", errors="ignore"))
            return True
        except OSError as exc:
            self._logger.warning(f"Vest serial write failed on {self._port}: {exc}")
            self.close()
            self._next_retry_ns = now_ns + self._retry_ns
            return False


class HapticVestNode(Node):
    def __init__(self) -> None:
        super().__init__("haptic_vest_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 921600)
        self.declare_parameter("serial_hz", 20.0)
        self.declare_parameter("serial_retry_s", 2.0)
        self.declare_parameter("use_serial_output", True)
        self.declare_parameter("serial_frame_topic", "/hermes/vest_serial_tx")

        self.declare_parameter("robot_ids", ["r1", "r2", "r3", "r4", "r5", "r6"])
        self.declare_parameter("motor_robot_ids", ["r1", "r2", "r3", "r4", "r5", "r6"])
        self.declare_parameter(
            "motor_labels",
            ["left_sleeve", "right_sleeve", "left_shoulder", "right_shoulder", "left_torso", "right_torso"],
        )

        self.declare_parameter("robot_state_topic", "/hermes/robot_state_beacon")
        self.declare_parameter("robot_status_topic", "/hermes/robot_haptic_status")
        self.declare_parameter("swarm_intent_topic", "/hermes/swarm_intent")
        self.declare_parameter("swarm_state_topic", "/hermes/swarm_state")
        self.declare_parameter("command_packet_topic", "/hermes/command_packets")
        self.declare_parameter("debug_topic", "/hermes/haptic_vest_state")

        self.declare_parameter("robot_state_timeout_ms", 1000)
        self.declare_parameter("robot_status_timeout_ms", 1200)
        self.declare_parameter("require_robot_status", False)
        self.declare_parameter("gesture_feedback_ms", 450)
        self.declare_parameter("gesture_min_gap_ms", 250)
        self.declare_parameter("formation_reached_feedback_ms", 700)
        self.declare_parameter("formation_reached_position_tolerance_m", 0.18)
        self.declare_parameter("ignore_command_effect_types", ["cmd_vel_stream", "set_param_stream", "gate_motion"])

        self.declare_parameter("dense_ratio_threshold", 0.80)
        self.declare_parameter("sparse_ratio_threshold", 1.25)
        self.declare_parameter("default_target_spacing_m", 1.00)

        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        serial_hz = max(5.0, _as_float(self.get_parameter("serial_hz").value, 20.0))
        serial_retry_s = max(0.2, _as_float(self.get_parameter("serial_retry_s").value, 2.0))
        use_serial_output = _as_bool(self.get_parameter("use_serial_output").value, True)
        serial_frame_topic = str(self.get_parameter("serial_frame_topic").value)

        self._robot_ids = sorted(str(v).strip().lower() for v in self.get_parameter("robot_ids").value if str(v).strip())
        self._motor_robot_ids = [
            str(v).strip().lower() for v in list(self.get_parameter("motor_robot_ids").value or []) if str(v).strip()
        ]
        self._motor_labels = [str(v).strip() for v in list(self.get_parameter("motor_labels").value or []) if str(v).strip()]
        if len(self._motor_robot_ids) != 6:
            raise ValueError("motor_robot_ids must contain exactly 6 robot ids")
        if len(self._motor_labels) != 6:
            raise ValueError("motor_labels must contain exactly 6 labels")

        robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        robot_status_topic = str(self.get_parameter("robot_status_topic").value)
        swarm_intent_topic = str(self.get_parameter("swarm_intent_topic").value)
        swarm_state_topic = str(self.get_parameter("swarm_state_topic").value)
        command_packet_topic = str(self.get_parameter("command_packet_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self._state_timeout_ms = max(100, int(self.get_parameter("robot_state_timeout_ms").value))
        self._status_timeout_ms = max(100, int(self.get_parameter("robot_status_timeout_ms").value))
        self._require_robot_status = _as_bool(self.get_parameter("require_robot_status").value, False)
        self._gesture_feedback_ms = max(100, int(self.get_parameter("gesture_feedback_ms").value))
        self._gesture_min_gap_ms = max(50, int(self.get_parameter("gesture_min_gap_ms").value))
        self._formation_feedback_ms = max(100, int(self.get_parameter("formation_reached_feedback_ms").value))
        self._formation_pos_tol_m = max(
            0.01, _as_float(self.get_parameter("formation_reached_position_tolerance_m").value, 0.18)
        )
        self._ignored_effect_types = {
            str(v).strip() for v in list(self.get_parameter("ignore_command_effect_types").value or []) if str(v).strip()
        }
        self._dense_ratio_threshold = max(0.1, _as_float(self.get_parameter("dense_ratio_threshold").value, 0.80))
        self._sparse_ratio_threshold = max(
            self._dense_ratio_threshold + 0.01,
            _as_float(self.get_parameter("sparse_ratio_threshold").value, 1.25),
        )
        self._default_target_spacing_m = max(
            0.1, _as_float(self.get_parameter("default_target_spacing_m").value, 1.00)
        )

        self._use_serial_output = use_serial_output
        self._serial_frame_topic = serial_frame_topic
        self._serial: Optional[_SerialWriter] = None
        self._frame_pub = None
        if self._use_serial_output:
            self._serial = _SerialWriter(self.get_logger(), serial_port, baud_rate, serial_retry_s)
        else:
            self._frame_pub = self.create_publisher(String, self._serial_frame_topic, 20)
        self._debug_pub = self.create_publisher(String, debug_topic, 20)
        self.create_subscription(String, robot_state_topic, self._on_robot_state, 50)
        self.create_subscription(String, robot_status_topic, self._on_robot_status, 50)
        self.create_subscription(String, swarm_intent_topic, self._on_swarm_intent, 50)
        self.create_subscription(String, swarm_state_topic, self._on_swarm_state, 50)
        self.create_subscription(String, command_packet_topic, self._on_command_packet, 50)
        self._timer = self.create_timer(1.0 / serial_hz, self._tick)

        self._robot_states: Dict[str, Dict[str, float]] = {}
        self._robot_state_rx_ns: Dict[str, int] = {}
        self._robot_status: Dict[str, Dict[str, Any]] = {}
        self._robot_status_rx_ns: Dict[str, int] = {}
        self._selection: List[str] = []
        self._intent_robot_ids: List[str] = list(self._robot_ids)
        self._formation_spacing_m = self._default_target_spacing_m
        self._active_formation_type: Optional[str] = None
        self._last_targets: Dict[str, Tuple[float, float, float]] = {}
        self._formation_signature: str = ""
        self._formation_reached_latched = False
        self._gesture_until_ms = 0
        self._last_gesture_feedback_ms = 0
        self._formation_until_ms = 0
        self._tx_seq = 0
        self._last_debug_ns = 0

        self.get_logger().info(
            f"Haptic vest ready. output_mode={'serial' if self._use_serial_output else 'topic'}, "
            f"serial_port={serial_port}, baud_rate={baud_rate}, serial_frame_topic={self._serial_frame_topic}, "
            f"motor_robot_ids={self._motor_robot_ids}, motor_labels={self._motor_labels}"
        )

    def destroy_node(self) -> bool:
        if self._serial is not None:
            self._serial.close()
        return super().destroy_node()

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _is_stale(self, rx_ns: int, timeout_ms: int) -> bool:
        if rx_ns <= 0:
            return True
        return (self._now_ns() - rx_ns) > (timeout_ms * 1_000_000)

    def _on_robot_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        if "robot_id" not in payload or "x" not in payload or "y" not in payload:
            return

        rid = str(payload.get("robot_id", "")).strip().lower()
        if not rid:
            return

        self._robot_states[rid] = {
            "x": _as_float(payload.get("x"), 0.0),
            "y": _as_float(payload.get("y"), 0.0),
            "yaw": _as_float(payload.get("yaw"), 0.0),
            "vx": _as_float(payload.get("vx"), 0.0),
            "vy": _as_float(payload.get("vy"), 0.0),
        }
        self._robot_state_rx_ns[rid] = self._now_ns()

    def _on_robot_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        rid = str(payload.get("robot_id", "")).strip().lower()
        if not rid:
            return
        self._robot_status[rid] = payload
        self._robot_status_rx_ns[rid] = self._now_ns()

    def _on_swarm_intent(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        selection = payload.get("selection", [])
        if isinstance(selection, list):
            self._selection = [str(v).strip().lower() for v in selection if str(v).strip()]

        robot_ids = payload.get("robot_ids", [])
        if isinstance(robot_ids, list):
            parsed = [str(v).strip().lower() for v in robot_ids if str(v).strip()]
            parsed = [rid for rid in parsed if rid in self._robot_ids]
            if parsed:
                self._intent_robot_ids = parsed

        spacing = payload.get("formation_spacing")
        if spacing is not None:
            self._formation_spacing_m = max(0.1, _as_float(spacing, self._default_target_spacing_m))

    def _on_swarm_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        swarm = payload.get("swarm", payload)
        if not isinstance(swarm, dict):
            return

        active_formation_type = swarm.get("active_formation_type")
        self._active_formation_type = str(active_formation_type).strip() if active_formation_type else None

        parsed_targets: Dict[str, Tuple[float, float, float]] = {}
        raw_targets = swarm.get("last_targets", {})
        if isinstance(raw_targets, dict):
            for rid_raw, values in raw_targets.items():
                rid = str(rid_raw).strip().lower()
                if rid not in self._robot_ids:
                    continue
                if isinstance(values, (list, tuple)) and len(values) >= 2:
                    x = _as_float(values[0], 0.0)
                    y = _as_float(values[1], 0.0)
                    yaw = _as_float(values[2], 0.0) if len(values) >= 3 else 0.0
                    parsed_targets[rid] = (x, y, yaw)
        self._last_targets = parsed_targets

        signature_obj = {
            "type": self._active_formation_type,
            "targets": {rid: list(vals) for rid, vals in sorted(self._last_targets.items())},
        }
        signature = json.dumps(signature_obj, sort_keys=True, separators=(",", ":"))
        if signature != self._formation_signature:
            self._formation_signature = signature
            self._formation_reached_latched = False

    def _on_command_packet(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        effect = payload.get("effect", {})
        effect_type = str(effect.get("type", "")).strip()
        now_ms = int(self._now_ns() / 1_000_000)
        if effect_type in self._ignored_effect_types:
            return
        if (now_ms - self._last_gesture_feedback_ms) < self._gesture_min_gap_ms:
            return

        self._last_gesture_feedback_ms = now_ms
        self._gesture_until_ms = now_ms + self._gesture_feedback_ms

    def _selected_or_all(self) -> List[str]:
        selected = [rid for rid in self._selection if rid in self._robot_ids]
        if selected:
            return selected

        fresh = [
            rid
            for rid in self._robot_ids
            if rid in self._robot_states and not self._is_stale(self._robot_state_rx_ns.get(rid, 0), self._state_timeout_ms)
        ]
        fallback = [rid for rid in self._intent_robot_ids if rid in self._robot_ids]
        return fresh if fresh else fallback

    def _density_mode(self) -> Tuple[Optional[str], List[str], Optional[float]]:
        active_ids = self._selected_or_all()
        points: List[Tuple[float, float]] = []
        fresh_ids: List[str] = []

        for rid in active_ids:
            state = self._robot_states.get(rid)
            rx_ns = self._robot_state_rx_ns.get(rid, 0)
            if state is None or self._is_stale(rx_ns, self._state_timeout_ms):
                continue
            points.append((float(state["x"]), float(state["y"])))
            fresh_ids.append(rid)

        mean_spacing = _nearest_neighbor_mean(points)
        if mean_spacing is None:
            return None, fresh_ids, None

        target = max(0.1, self._formation_spacing_m or self._default_target_spacing_m)
        ratio = mean_spacing / target
        if ratio < self._dense_ratio_threshold:
            return "dense", fresh_ids, ratio
        if ratio > self._sparse_ratio_threshold:
            return "sparse", fresh_ids, ratio
        return None, fresh_ids, ratio

    def _lost_comm_on(self, now_ms: int) -> bool:
        return _windowed_pulse(now_ms, 700, [(0, 260)])

    def _error_on(self, now_ms: int) -> bool:
        return _windowed_pulse(now_ms, 900, [(0, 120), (220, 340), (440, 560)])

    def _obstacle_on(self, now_ms: int, obstacle_level: float) -> bool:
        clipped = max(0.0, min(1.0, obstacle_level))
        period_ms = int(round(900.0 - (500.0 * clipped)))
        return _windowed_pulse(now_ms, max(240, period_ms), [(0, 120)])

    def _gesture_on(self, now_ms: int) -> bool:
        if now_ms >= self._gesture_until_ms:
            return False
        elapsed = self._gesture_feedback_ms - (self._gesture_until_ms - now_ms)
        return elapsed < 80 or (180 <= elapsed < 260)

    def _formation_reached_on(self, now_ms: int) -> bool:
        if now_ms >= self._formation_until_ms:
            return False
        elapsed = self._formation_feedback_ms - (self._formation_until_ms - now_ms)
        return elapsed < 90 or (160 <= elapsed < 250) or (320 <= elapsed < 410)

    def _formation_reached_now(self) -> Tuple[bool, List[str]]:
        if not self._active_formation_type or not self._last_targets:
            return False, []

        active_ids = self._selected_or_all()
        if not active_ids:
            return False, []

        checked_ids: List[str] = []
        for rid in active_ids:
            target = self._last_targets.get(rid)
            state = self._robot_states.get(rid)
            rx_ns = self._robot_state_rx_ns.get(rid, 0)
            if target is None or state is None or self._is_stale(rx_ns, self._state_timeout_ms):
                return False, checked_ids

            checked_ids.append(rid)
            distance = math.hypot(float(state["x"]) - target[0], float(state["y"]) - target[1])
            if distance > self._formation_pos_tol_m:
                return False, checked_ids

        return bool(checked_ids), checked_ids

    def _dense_on(self, now_ms: int) -> bool:
        return _windowed_pulse(now_ms, 420, [(0, 90)])

    def _sparse_on(self, now_ms: int) -> bool:
        return _windowed_pulse(now_ms, 1200, [(0, 220)])

    def _tick(self) -> None:
        now_ns = self._now_ns()
        now_ms = int(now_ns / 1_000_000)
        density_mode, density_robot_ids, density_ratio = self._density_mode()
        formation_reached_now, formation_robot_ids = self._formation_reached_now()
        if formation_reached_now and not self._formation_reached_latched:
            self._formation_until_ms = now_ms + self._formation_feedback_ms
            self._formation_reached_latched = True
        elif not formation_reached_now:
            self._formation_reached_latched = False

        levels: List[int] = []
        debug_motors: List[Dict[str, Any]] = []

        for idx, rid in enumerate(self._motor_robot_ids):
            state_rx_ns = self._robot_state_rx_ns.get(rid, 0)
            status_rx_ns = self._robot_status_rx_ns.get(rid, 0)
            lost_comm = self._is_stale(state_rx_ns, self._state_timeout_ms)

            status = self._robot_status.get(rid, {})
            status_fresh = not self._is_stale(status_rx_ns, self._status_timeout_ms)
            status_error = bool(status.get("error", False)) if status_fresh else False
            status_missing = self._require_robot_status and not status_fresh
            obstacle_level = _as_float(status.get("obstacle_level"), 0.0) if status_fresh else 0.0

            event = "idle"
            is_on = False

            if lost_comm:
                event = "lost_comm"
                is_on = self._lost_comm_on(now_ms)
            elif status_error or status_missing:
                event = "robot_error" if status_error else "status_missing"
                is_on = self._error_on(now_ms)
            elif obstacle_level > 0.01:
                event = "obstacle"
                is_on = self._obstacle_on(now_ms, obstacle_level)
            elif self._formation_reached_on(now_ms) and rid in formation_robot_ids:
                event = "formation_reached"
                is_on = True
            elif self._gesture_on(now_ms):
                event = "gesture_ok"
                is_on = True
            elif density_mode == "dense" and rid in density_robot_ids:
                event = "swarm_dense"
                is_on = self._dense_on(now_ms)
            elif density_mode == "sparse" and rid in density_robot_ids:
                event = "swarm_sparse"
                is_on = self._sparse_on(now_ms)

            levels.append(255 if is_on else 0)
            debug_motors.append(
                {
                    "index": idx + 1,
                    "robot_id": rid,
                    "label": self._motor_labels[idx],
                    "event": event,
                    "level": levels[-1],
                }
            )

        self._tx_seq += 1
        frame = "V1,{seq},{levels}\n".format(seq=self._tx_seq, levels=",".join(str(v) for v in levels))
        if self._serial is not None:
            self._serial.write_line(frame, now_ns)
        elif self._frame_pub is not None:
            out = String()
            out.data = frame
            self._frame_pub.publish(out)

        if (now_ns - self._last_debug_ns) >= 250_000_000:
            self._last_debug_ns = now_ns
            debug_payload = {
                "schema": "hermes.haptic_vest_state.v1",
                "stamp_ms": now_ms,
                "selection": self._selected_or_all(),
                "formation_spacing_m": float(self._formation_spacing_m),
                "active_formation_type": self._active_formation_type,
                "formation_reached": formation_reached_now,
                "density_mode": density_mode,
                "density_ratio": density_ratio,
                "motors": debug_motors,
                "serial_frame": frame.strip(),
            }
            out = String()
            out.data = json.dumps(debug_payload, separators=(",", ":"))
            self._debug_pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HapticVestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
