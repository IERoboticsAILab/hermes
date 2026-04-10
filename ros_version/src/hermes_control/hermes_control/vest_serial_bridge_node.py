import json
import os
import termios
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class _SerialPort:
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
        self._buffer = bytearray()
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
        self._buffer.clear()

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

    def read_lines(self, now_ns: int) -> List[str]:
        if not self.ensure_open(now_ns):
            return []

        assert self._fd is not None
        try:
            while True:
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    break
                self._buffer.extend(chunk)
                if len(chunk) < 4096:
                    break
        except BlockingIOError:
            pass
        except OSError as exc:
            self._logger.warning(f"Vest serial read failed on {self._port}: {exc}")
            self.close()
            self._next_retry_ns = now_ns + self._retry_ns
            return []

        lines: List[str] = []
        while True:
            newline_idx = self._buffer.find(b"\n")
            if newline_idx < 0:
                break
            raw = self._buffer[:newline_idx]
            del self._buffer[: newline_idx + 1]
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                lines.append(line)
        return lines


class VestSerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vest_serial_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 921600)
        self.declare_parameter("serial_retry_s", 2.0)
        self.declare_parameter("poll_hz", 60.0)
        self.declare_parameter("glove_timeout_ms", 200)
        self.declare_parameter("raw_topic", "/hermes/raw_input")
        self.declare_parameter("serial_frame_topic", "/hermes/vest_serial_tx")
        self.declare_parameter("hub_topic", "/hermes/vest_hub_rx")
        self.declare_parameter("debug_topic", "/hermes/vest_serial_state")

        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        serial_retry_s = max(0.2, _as_float(self.get_parameter("serial_retry_s").value, 2.0))
        poll_hz = max(5.0, _as_float(self.get_parameter("poll_hz").value, 60.0))
        self._glove_timeout_ms = max(50, int(self.get_parameter("glove_timeout_ms").value))

        raw_topic = str(self.get_parameter("raw_topic").value)
        serial_frame_topic = str(self.get_parameter("serial_frame_topic").value)
        hub_topic = str(self.get_parameter("hub_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self._serial = _SerialPort(self.get_logger(), serial_port, baud_rate, serial_retry_s)
        self._raw_pub = self.create_publisher(String, raw_topic, 50)
        self._hub_pub = self.create_publisher(String, hub_topic, 50)
        self._debug_pub = self.create_publisher(String, debug_topic, 20)
        self.create_subscription(String, serial_frame_topic, self._on_serial_frame, 50)
        self._timer = self.create_timer(1.0 / poll_hz, self._tick)

        self._latest_packets: Dict[str, Dict[str, Any]] = {}
        self._latest_rx_ms: Dict[str, int] = {}
        self._line_count = 0
        self._write_count = 0
        self._raw_count = 0
        self._hub_status_count = 0
        self._invalid_json_count = 0
        self._last_serial_frame = ""
        self._last_debug_ns = 0
        self._last_glove_ids: List[str] = []

        self.get_logger().info(
            f"Vest serial bridge ready. serial_port={serial_port}, baud_rate={baud_rate}, "
            f"raw_topic={raw_topic}, serial_frame_topic={serial_frame_topic}"
        )

    def destroy_node(self) -> bool:
        self._serial.close()
        return super().destroy_node()

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    @staticmethod
    def _safe_flex(pkt: Dict[str, Any]) -> Dict[str, float]:
        flex = pkt.get("flex", {})
        return {
            "index": _as_float(flex.get("index"), 0.0),
            "middle": _as_float(flex.get("middle"), 0.0),
            "ring": _as_float(flex.get("ring"), 0.0),
            "pinky": _as_float(flex.get("pinky"), 0.0),
        }

    @staticmethod
    def _safe_fsr(pkt: Dict[str, Any]) -> Dict[str, bool]:
        fsr = pkt.get("fsr", {})
        return {
            "INDEX": bool(fsr.get("INDEX", False)),
            "MIDDLE": bool(fsr.get("MIDDLE", False)),
            "RING": bool(fsr.get("RING", False)),
            "PINKY": bool(fsr.get("PINKY", False)),
        }

    @staticmethod
    def _safe_right_imu(pkt: Dict[str, Any]) -> Optional[Dict[str, float]]:
        imu = pkt.get("imu", {})
        if not isinstance(imu, dict):
            return None
        if not any(key in imu for key in ("PITCH", "ROLL", "YAW", "AX", "AY", "AZ", "X", "Y", "Z")):
            return None
        return {
            "PITCH": _as_float(imu.get("PITCH"), 0.0),
            "ROLL": _as_float(imu.get("ROLL"), 0.0),
            "YAW": _as_float(imu.get("YAW"), 0.0),
            "AX": _as_float(imu.get("AX", imu.get("X")), 0.0),
            "AY": _as_float(imu.get("AY", imu.get("Y")), 0.0),
            "AZ": _as_float(imu.get("AZ", imu.get("Z")), 0.0),
        }

    @staticmethod
    def _safe_left_imu(pkt: Dict[str, Any]) -> Dict[str, float]:
        imu = pkt.get("imu", {})
        return {
            "PITCH": _as_float(imu.get("PITCH"), 0.0),
            "ROLL": _as_float(imu.get("ROLL"), 0.0),
            "YAW": _as_float(imu.get("YAW"), 0.0),
            "AX": _as_float(imu.get("AX", imu.get("X")), 0.0),
            "AY": _as_float(imu.get("AY", imu.get("Y")), 0.0),
            "AZ": _as_float(imu.get("AZ", imu.get("Z")), 0.0),
        }

    def _publish_string(self, pub, payload: Dict[str, Any]) -> None:
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        pub.publish(out)

    def _publish_raw_input(self, now_ms: int) -> None:
        left_fresh = self._is_fresh("L", now_ms)
        right_fresh = self._is_fresh("R", now_ms)

        if left_fresh:
            left = self._latest_packets["L"]
            raw = {
                "time_ms": now_ms,
                "flex": {"L": self._safe_flex(left)},
                "fsr_pressed": {"L": self._safe_fsr(left)},
                "imu": {"L": self._safe_left_imu(left)},
            }
            if right_fresh:
                right = self._latest_packets["R"]
                raw["flex"]["R"] = self._safe_flex(right)
                raw["fsr_pressed"]["R"] = self._safe_fsr(right)
                right_imu = self._safe_right_imu(right)
                if right_imu is not None:
                    raw["imu"]["R"] = right_imu
            self._last_glove_ids = [gid for gid in ("L", "R") if self._is_fresh(gid, now_ms)]
        else:
            # The left glove owns posture, deadman, and active control IMU. If it
            # goes stale, force the gesture pipeline into a fail-safe deadman-off
            # tick. A stale right glove should only remove right-hand commands.
            raw = {
                "time_ms": now_ms,
                "flex": {},
                "fsr_pressed": {},
                "imu": {},
            }
            self._last_glove_ids = [gid for gid in ("L", "R") if self._is_fresh(gid, now_ms)]

        self._publish_string(self._raw_pub, raw)
        self._raw_count += 1

    def _is_fresh(self, glove_id: str, now_ms: int) -> bool:
        rx_ms = self._latest_rx_ms.get(glove_id)
        if rx_ms is None:
            return False
        return (now_ms - rx_ms) <= self._glove_timeout_ms

    def _on_serial_frame(self, msg: String) -> None:
        now_ns = self._now_ns()
        frame = msg.data if msg.data.endswith("\n") else f"{msg.data}\n"
        if self._serial.write_line(frame, now_ns):
            self._write_count += 1
            self._last_serial_frame = frame.strip()

    def _ingest_line(self, line: str, now_ms: int) -> None:
        self._line_count += 1
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            self._invalid_json_count += 1
            return

        if not isinstance(payload, dict):
            return

        self._publish_string(self._hub_pub, payload)

        schema = str(payload.get("schema", ""))
        if schema == "hermes.hub.status":
            self._hub_status_count += 1
            return
        if schema != "hermes.hub.v1":
            return
        if not bool(payload.get("valid_json", False)):
            return

        glove_id = str(payload.get("glove_id", "")).upper()
        packet = payload.get("packet")
        if glove_id not in {"L", "R"} or not isinstance(packet, dict):
            return

        self._latest_packets[glove_id] = packet
        self._latest_rx_ms[glove_id] = now_ms

    def _tick(self) -> None:
        now_ns = self._now_ns()
        now_ms = int(now_ns / 1_000_000)
        for line in self._serial.read_lines(now_ns):
            self._ingest_line(line, now_ms)

        self._publish_raw_input(now_ms)

        if (now_ns - self._last_debug_ns) >= 500_000_000:
            self._last_debug_ns = now_ns
            debug_payload = {
                "schema": "hermes.vest_serial_state.v1",
                "stamp_ms": now_ms,
                "fresh_gloves": [gid for gid in ("L", "R") if self._is_fresh(gid, now_ms)],
                "last_glove_ids": self._last_glove_ids,
                "line_count": self._line_count,
                "raw_count": self._raw_count,
                "write_count": self._write_count,
                "hub_status_count": self._hub_status_count,
                "invalid_json_count": self._invalid_json_count,
                "last_serial_frame": self._last_serial_frame,
            }
            self._publish_string(self._debug_pub, debug_payload)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VestSerialBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
