import json
import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
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


def _finite_or_none(value: float) -> Optional[float]:
    return float(value) if math.isfinite(value) else None


class RobotHapticStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_haptic_status_node")

        self.declare_parameter("robot_id", "r1")
        self.declare_parameter("status_topic", "/hermes/robot_haptic_status")
        self.declare_parameter("publish_hz", 10.0)

        self.declare_parameter("enable_scan", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("front_arc_deg", 35.0)
        self.declare_parameter("scan_stale_ms", 600)
        self.declare_parameter("front_scan_hit_distance_m", 0.55)
        self.declare_parameter("front_scan_clear_distance_m", 1.20)

        self.declare_parameter("front_range_topics", [])
        self.declare_parameter("rear_range_topics", [])
        self.declare_parameter("range_stale_ms", 600)
        self.declare_parameter("front_range_hit_distance_m", 0.20)
        self.declare_parameter("front_range_clear_distance_m", 0.50)
        self.declare_parameter("rear_range_hit_distance_m", 0.15)
        self.declare_parameter("rear_range_clear_distance_m", 0.40)

        self.declare_parameter("enable_diagnostics", True)
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("diagnostic_name_filters", ["rosbot", "base", "controller", "motor"])
        self.declare_parameter("treat_warn_as_error", False)

        self._robot_id = str(self.get_parameter("robot_id").value).strip().lower()
        status_topic = str(self.get_parameter("status_topic").value)
        publish_hz = max(1.0, _as_float(self.get_parameter("publish_hz").value, 10.0))

        self._enable_scan = _as_bool(self.get_parameter("enable_scan").value, True)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._front_arc_deg = max(1.0, _as_float(self.get_parameter("front_arc_deg").value, 35.0))
        self._scan_stale_ms = max(100, int(self.get_parameter("scan_stale_ms").value))
        self._front_scan_hit_m = max(0.01, _as_float(self.get_parameter("front_scan_hit_distance_m").value, 0.55))
        self._front_scan_clear_m = max(
            self._front_scan_hit_m + 0.01,
            _as_float(self.get_parameter("front_scan_clear_distance_m").value, 1.20),
        )

        self._front_range_topics = [
            str(v).strip() for v in list(self.get_parameter("front_range_topics").value or []) if str(v).strip()
        ]
        self._rear_range_topics = [
            str(v).strip() for v in list(self.get_parameter("rear_range_topics").value or []) if str(v).strip()
        ]
        self._range_stale_ms = max(100, int(self.get_parameter("range_stale_ms").value))
        self._front_range_hit_m = max(0.01, _as_float(self.get_parameter("front_range_hit_distance_m").value, 0.20))
        self._front_range_clear_m = max(
            self._front_range_hit_m + 0.01,
            _as_float(self.get_parameter("front_range_clear_distance_m").value, 0.50),
        )
        self._rear_range_hit_m = max(0.01, _as_float(self.get_parameter("rear_range_hit_distance_m").value, 0.15))
        self._rear_range_clear_m = max(
            self._rear_range_hit_m + 0.01,
            _as_float(self.get_parameter("rear_range_clear_distance_m").value, 0.40),
        )

        self._enable_diagnostics = _as_bool(self.get_parameter("enable_diagnostics").value, True)
        self._diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self._diagnostic_name_filters = [
            str(v).strip().lower()
            for v in list(self.get_parameter("diagnostic_name_filters").value or [])
            if str(v).strip()
        ]
        self._treat_warn_as_error = _as_bool(self.get_parameter("treat_warn_as_error").value, False)

        self._last_scan: Optional[LaserScan] = None
        self._last_scan_rx_ns = 0
        self._front_ranges: Dict[str, Tuple[float, int]] = {}
        self._rear_ranges: Dict[str, Tuple[float, int]] = {}
        self._diag_level = 0
        self._diag_message = ""
        self._diag_names: List[str] = []
        self._diag_rx_ns = 0

        self._status_pub = self.create_publisher(String, status_topic, 20)

        if self._enable_scan and self._scan_topic:
            self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 20)

        for topic in self._front_range_topics:
            self.create_subscription(Range, topic, self._make_range_cb(topic, self._front_ranges), 20)
        for topic in self._rear_range_topics:
            self.create_subscription(Range, topic, self._make_range_cb(topic, self._rear_ranges), 20)

        if self._enable_diagnostics and self._diagnostics_topic:
            self.create_subscription(DiagnosticArray, self._diagnostics_topic, self._on_diagnostics, 20)

        self._timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f"Robot haptic status ready. robot_id={self._robot_id}, status_topic={status_topic}, "
            f"scan_topic={self._scan_topic}, front_range_topics={self._front_range_topics}, "
            f"rear_range_topics={self._rear_range_topics}, diagnostics_topic={self._diagnostics_topic}"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _make_range_cb(self, topic: str, bucket: Dict[str, Tuple[float, int]]):
        def _cb(msg: Range) -> None:
            distance = float(msg.range)
            if not math.isfinite(distance) or distance < 0.0:
                distance = float("inf")
            bucket[topic] = (distance, self._now_ns())

        return _cb

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_rx_ns = self._now_ns()

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        worst_level = 0
        worst_message = ""
        matched_names: List[str] = []

        for status in msg.status:
            name = str(status.name).strip()
            lowered = name.lower()
            if self._diagnostic_name_filters and not any(token in lowered for token in self._diagnostic_name_filters):
                continue
            matched_names.append(name)
            level = int(status.level)
            if level > worst_level:
                worst_level = level
                worst_message = str(status.message)

        self._diag_level = worst_level
        self._diag_message = worst_message
        self._diag_names = matched_names[:8]
        self._diag_rx_ns = self._now_ns()

    def _is_stale(self, rx_ns: int, timeout_ms: int) -> bool:
        if rx_ns <= 0:
            return True
        return (self._now_ns() - rx_ns) > (timeout_ms * 1_000_000)

    def _scan_front_min(self) -> float:
        if self._last_scan is None or self._is_stale(self._last_scan_rx_ns, self._scan_stale_ms):
            return float("inf")

        msg = self._last_scan
        arc_rad = math.radians(self._front_arc_deg)
        best = float("inf")

        for idx, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle = float(msg.angle_min) + (idx * float(msg.angle_increment))
            if abs(angle) > arc_rad:
                continue
            if distance < best:
                best = float(distance)

        return best

    def _fresh_min(self, bucket: Dict[str, Tuple[float, int]], timeout_ms: int) -> float:
        best = float("inf")
        now_ns = self._now_ns()
        for distance, rx_ns in bucket.values():
            if rx_ns <= 0 or (now_ns - rx_ns) > (timeout_ms * 1_000_000):
                continue
            if math.isfinite(distance) and distance < best:
                best = float(distance)
        return best

    def _bucket_missing(self, bucket: Dict[str, Tuple[float, int]], expected_topics: List[str], timeout_ms: int) -> bool:
        if not expected_topics:
            return False
        now_ns = self._now_ns()
        for topic in expected_topics:
            _, rx_ns = bucket.get(topic, (float("inf"), 0))
            if rx_ns > 0 and (now_ns - rx_ns) <= (timeout_ms * 1_000_000):
                return False
        return True

    def _risk_from_distance(self, distance: float, hit_m: float, clear_m: float) -> float:
        if not math.isfinite(distance):
            return 0.0
        if distance <= hit_m:
            return 1.0
        if distance >= clear_m:
            return 0.0
        span = max(1e-6, clear_m - hit_m)
        return max(0.0, min(1.0, (clear_m - distance) / span))

    def _tick(self) -> None:
        front_scan_m = self._scan_front_min()
        front_range_m = self._fresh_min(self._front_ranges, self._range_stale_ms)
        rear_range_m = self._fresh_min(self._rear_ranges, self._range_stale_ms)

        obstacle_level = max(
            self._risk_from_distance(front_scan_m, self._front_scan_hit_m, self._front_scan_clear_m),
            self._risk_from_distance(front_range_m, self._front_range_hit_m, self._front_range_clear_m),
            self._risk_from_distance(rear_range_m, self._rear_range_hit_m, self._rear_range_clear_m),
        )

        scan_missing = self._enable_scan and self._is_stale(self._last_scan_rx_ns, self._scan_stale_ms)
        front_ranges_missing = self._bucket_missing(self._front_ranges, self._front_range_topics, self._range_stale_ms)
        rear_ranges_missing = self._bucket_missing(self._rear_ranges, self._rear_range_topics, self._range_stale_ms)
        diag_stale = self._enable_diagnostics and self._diag_rx_ns > 0 and self._is_stale(self._diag_rx_ns, 2000)
        diag_error = self._diag_level >= 2 or (self._treat_warn_as_error and self._diag_level >= 1)

        error = bool(scan_missing or front_ranges_missing or rear_ranges_missing or diag_error or diag_stale)

        payload = {
            "schema": "hermes.robot_haptic_status.v1",
            "stamp_ms": int(self._now_ns() / 1_000_000),
            "robot_id": self._robot_id,
            "obstacle": obstacle_level > 0.01,
            "obstacle_level": float(obstacle_level),
            "front_scan_min_m": _finite_or_none(front_scan_m),
            "front_range_min_m": _finite_or_none(front_range_m),
            "rear_range_min_m": _finite_or_none(rear_range_m),
            "error": error,
            "error_flags": {
                "scan_missing": scan_missing,
                "front_ranges_missing": front_ranges_missing,
                "rear_ranges_missing": rear_ranges_missing,
                "diag_error": diag_error,
                "diag_stale": diag_stale,
            },
            "diag_level": int(self._diag_level),
            "diag_message": self._diag_message,
            "diag_names": self._diag_names,
        }

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotHapticStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
