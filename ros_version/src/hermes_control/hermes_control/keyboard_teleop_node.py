import json
import select
import sys
import termios
import tty
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _clamp_level(level: int) -> int:
    return max(1, min(4, int(level)))


SPEED_SCALE = {
    1: 0.35,
    2: 0.55,
    3: 0.75,
    4: 1.00,
}

AGGRESSION_SCALE = {
    1: 0.75,
    2: 1.00,
    3: 1.20,
    4: 1.40,
}


class KeyboardTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop_node")

        self.declare_parameter("packet_topic", "/hermes/command_packets")
        self.declare_parameter("robot_ids", ["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8"])
        self.declare_parameter("tick_hz", 30.0)
        self.declare_parameter("base_linear", 1.0)
        self.declare_parameter("base_angular", 1.0)

        packet_topic = str(self.get_parameter("packet_topic").value)
        self._robot_ids = sorted(str(v).strip().lower() for v in self.get_parameter("robot_ids").value)
        tick_hz = max(5.0, float(self.get_parameter("tick_hz").value))
        self._base_linear = float(self.get_parameter("base_linear").value)
        self._base_angular = float(self.get_parameter("base_angular").value)

        self._mode = "DRIVE"
        self._deadman_active = False
        self._speed_level = 2
        self._spacing_level = 2
        self._aggression_level = 2

        self._stdin_fd: Optional[int] = None
        self._stdin_old_settings: Optional[Any] = None
        self._terminal_ready = False

        self._packet_pub = self.create_publisher(String, packet_topic, 50)
        self._setup_terminal()

        # Initialize swarm control state for keyboard-only testing.
        self._publish_set_selection(self._robot_ids)
        self._publish_set_mode(self._mode)
        self._publish_gate_motion(False)
        self._publish_drive(0.0, 0.0, 0.0)

        self._timer = self.create_timer(1.0 / tick_hz, self._tick)
        self._print_help()
        self.get_logger().info(f"Keyboard teleop ready. packet_topic={packet_topic}")
        self._log_status()

    def _setup_terminal(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires an interactive TTY stdin.")
        self._stdin_fd = sys.stdin.fileno()
        self._stdin_old_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._terminal_ready = True

    def _restore_terminal(self) -> None:
        if self._terminal_ready and self._stdin_fd is not None and self._stdin_old_settings is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old_settings)
        self._terminal_ready = False

    def destroy_node(self) -> bool:
        self._restore_terminal()
        return super().destroy_node()

    def _read_key(self) -> Optional[str]:
        if self._stdin_fd is None:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        return sys.stdin.read(1)

    def _publish_packet(
        self,
        *,
        command_id: str,
        command_key: str,
        effect: Dict[str, Any],
        resolved: Optional[Dict[str, Any]] = None,
    ) -> None:
        packet = {
            "domain": "teleop",
            "command_id": command_id,
            "command_key": command_key,
            "effect": effect,
            "resolved": resolved or {},
        }
        msg = String()
        msg.data = json.dumps(packet, separators=(",", ":"))
        self._packet_pub.publish(msg)

    def _publish_set_mode(self, mode: str) -> None:
        self._mode = mode
        self._publish_packet(
            command_id=f"teleop.mode.{mode.lower()}",
            command_key=f"MODE_{mode}",
            effect={"type": "set_mode", "value": mode},
        )

    def _publish_set_selection(self, robot_ids: list[str]) -> None:
        self._publish_packet(
            command_id="teleop.selection.all",
            command_key="SELECT_ALL",
            effect={"type": "set_selection", "value": robot_ids},
        )

    def _publish_gate_motion(self, enabled: bool) -> None:
        self._deadman_active = bool(enabled)
        self._publish_packet(
            command_id="teleop.deadman",
            command_key="DEADMAN",
            effect={"type": "gate_motion", "value": bool(enabled)},
        )

    def _publish_drive(self, raw_vx: float, raw_vy: float, raw_omega: float) -> None:
        speed_scale = SPEED_SCALE.get(self._speed_level, 0.55)
        turn_scale = AGGRESSION_SCALE.get(self._aggression_level, 1.0)
        cmd_vel = {
            "vx": float(raw_vx) * self._base_linear * speed_scale,
            "vy": float(raw_vy) * self._base_linear * speed_scale,
            "omega": float(raw_omega) * self._base_angular * speed_scale * turn_scale,
        }
        self._publish_packet(
            command_id="teleop.drive.cmd_vel",
            command_key="DRIVE_CMD_VEL",
            effect={"type": "cmd_vel_stream"},
            resolved={"cmd_vel": cmd_vel},
        )

    def _step_param(self, key: str, delta: int) -> None:
        if key == "speed_level":
            self._speed_level = _clamp_level(self._speed_level + delta)
        elif key == "spacing_level":
            self._spacing_level = _clamp_level(self._spacing_level + delta)
        elif key == "aggression_level":
            self._aggression_level = _clamp_level(self._aggression_level + delta)

        self._publish_packet(
            command_id=f"teleop.params.{key}",
            command_key=f"STEP_{key.upper()}",
            effect={
                "type": "step_param",
                "key": key,
                "delta": int(delta),
                "min": 1,
                "max": 4,
            },
        )
        self._log_status()

    def _apply_formation(self, formation: str) -> None:
        self._publish_packet(
            command_id="teleop.formation.pending",
            command_key=f"SET_{formation}",
            effect={"type": "set_state", "key": "pending_formation_type", "value": formation},
        )
        self._publish_packet(
            command_id="teleop.formation.apply",
            command_key="APPLY_FORMATION",
            effect={"type": "apply_formation"},
        )
        self.get_logger().info(f"Applied formation: {formation}")

    def _break_formation(self) -> None:
        self._publish_packet(
            command_id="teleop.formation.break",
            command_key="BREAK_FORMATION",
            effect={"type": "break_formation"},
        )
        self.get_logger().info("Requested break formation")

    def _log_status(self) -> None:
        self.get_logger().info(
            "Status | mode=%s deadman=%s speed=%d spacing=%d aggression=%d"
            % (
                self._mode,
                "ON" if self._deadman_active else "OFF",
                self._speed_level,
                self._spacing_level,
                self._aggression_level,
            )
        )

    def _print_help(self) -> None:
        print(
            "\nH.E.R.M.E.S Keyboard Teleop\n"
            "Global:\n"
            "  1: DRIVE mode, 2: FORMATION mode, 3: PARAMS mode\n"
            "  m: toggle deadman gate, g: select all robots, space: drive stop, h: help\n"
            "DRIVE mode keys:\n"
            "  w/s: forward/reverse, a/d: yaw left/right, q/e: strafe left/right, x: zero cmd_vel\n"
            "FORMATION mode keys:\n"
            "  l: LINE, c: COLUMN, w: WEDGE, b: break formation\n"
            "PARAMS mode keys:\n"
            "  w/s: speed +/- | e/d: spacing +/- | r/f: aggression +/-\n"
            "Exit: Ctrl-C\n",
            flush=True,
        )

    def _handle_drive_key(self, key: str) -> bool:
        drive_map = {
            "w": (1.0, 0.0, 0.0),
            "s": (-1.0, 0.0, 0.0),
            "a": (0.0, 0.0, 1.0),
            "d": (0.0, 0.0, -1.0),
            "q": (0.0, 1.0, 0.0),
            "e": (0.0, -1.0, 0.0),
            "x": (0.0, 0.0, 0.0),
        }
        axes = drive_map.get(key)
        if axes is None:
            return False
        self._publish_drive(*axes)
        return True

    def _handle_formation_key(self, key: str) -> bool:
        if key == "l":
            self._apply_formation("LINE")
            return True
        if key == "c":
            self._apply_formation("COLUMN")
            return True
        if key == "w":
            self._apply_formation("WEDGE")
            return True
        if key == "b":
            self._break_formation()
            return True
        return False

    def _handle_params_key(self, key: str) -> bool:
        if key == "w":
            self._step_param("speed_level", +1)
            return True
        if key == "s":
            self._step_param("speed_level", -1)
            return True
        if key == "e":
            self._step_param("spacing_level", +1)
            return True
        if key == "d":
            self._step_param("spacing_level", -1)
            return True
        if key == "r":
            self._step_param("aggression_level", +1)
            return True
        if key == "f":
            self._step_param("aggression_level", -1)
            return True
        return False

    def _tick(self) -> None:
        while True:
            key = self._read_key()
            if key is None:
                return

            if key == "\x03":
                raise KeyboardInterrupt

            if key == "h":
                self._print_help()
                continue
            if key == "1":
                self._publish_set_mode("DRIVE")
                self._log_status()
                continue
            if key == "2":
                self._publish_set_mode("FORMATION")
                self._log_status()
                continue
            if key == "3":
                self._publish_set_mode("PARAMS")
                self._log_status()
                continue
            if key == "m":
                self._publish_gate_motion(not self._deadman_active)
                self._log_status()
                continue
            if key == "g":
                self._publish_set_selection(self._robot_ids)
                self.get_logger().info(f"Selection set to all robots: {self._robot_ids}")
                continue
            if key == " ":
                self._publish_drive(0.0, 0.0, 0.0)
                continue

            if self._mode == "DRIVE" and self._handle_drive_key(key):
                continue
            if self._mode == "FORMATION" and self._handle_formation_key(key):
                continue
            if self._mode == "PARAMS" and self._handle_params_key(key):
                continue


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Best-effort stop on exit.
            try:
                node._publish_gate_motion(False)
                node._publish_drive(0.0, 0.0, 0.0)
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
