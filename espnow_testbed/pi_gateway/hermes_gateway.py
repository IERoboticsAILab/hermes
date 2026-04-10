#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import serial

# Reuse current non-ROS pipeline modules from repo root.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gestures.matcher import match_gesture
from gestures.models import GestureState
from gestures.recognizer import GestureRecognizer
from gestures.registry import COMMAND_REGISTRY
from gestures.safety import SafetyEvaluator
from swarm.swarm_controller import SwarmController


DEFAULT_CONFIG = {
    "serial_port": "/dev/ttyUSB0",
    "baudrate": 921600,
    "glove_timeout_ms": 200,
    "robot_ids": ["r1", "r2", "r3", "r4", "r5", "r6"],
    "centroid": [0.0, 0.0],
    "command_output": {
        "mode": "print",  # print | udp
        "udp_host": "192.168.1.200",
        "udp_port": 5005,
    },
    "behavior_runtime": {
        "home_xy": [0.0, 0.0],
        "path_waypoints": [],
    },
}


@dataclass
class TimedPacket:
    packet: Dict[str, Any]
    rx_monotonic_ms: int


class SwarmCommandAdapter:
    def __init__(self, output_cfg: Dict[str, Any]) -> None:
        self.mode = str(output_cfg.get("mode", "print"))
        self.udp_host = str(output_cfg.get("udp_host", "127.0.0.1"))
        self.udp_port = int(output_cfg.get("udp_port", 5005))
        self.sock: Optional[socket.socket] = None
        if self.mode == "udp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _emit(self, command: Dict[str, Any]) -> None:
        line = json.dumps(command, separators=(",", ":"))
        if self.mode == "udp" and self.sock is not None:
            self.sock.sendto(line.encode("utf-8"), (self.udp_host, self.udp_port))
        else:
            print(f"[SWARM_CMD] {line}")

    def _emit_state_update(
        self,
        command_id: Optional[str],
        source_effect: str,
        gesture_state: GestureState,
        swarm: SwarmController,
    ) -> None:
        self._emit(
            {
                "type": "STATE_UPDATE",
                "command_id": command_id,
                "source_effect": source_effect,
                "mode": gesture_state.mode,
                "deadman_active": gesture_state.deadman_active,
                "params": gesture_state.params,
                "selection": sorted(swarm.selection),
                "groups": {k: sorted(v) for k, v in swarm.groups.items()},
                "active_group_edit": swarm.active_group_edit,
                "pending_group_members": sorted(swarm.pending_group_members),
                "active_formation_type": swarm.active_formation_type,
                "pending_formation_type": swarm.pending_formation_type,
                "formation_heading": swarm.formation_heading,
                "formation_spacing": swarm.formation_spacing,
                "active_behavior": swarm.active_behavior,
                "behavior_params": swarm.behavior_params,
                "last_targets": swarm.last_targets,
            }
        )

    def dispatch(
        self,
        packet: Dict[str, Any],
        gesture_state: GestureState,
        swarm: SwarmController,
    ) -> None:
        effect = packet.get("effect", {})
        etype = effect.get("type")
        command_id = packet.get("command_id")

        if etype == "emergency_stop":
            self._emit({"type": "ESTOP", "command_id": command_id})
            return

        if etype == "pause":
            self._emit({"type": "PAUSE", "command_id": command_id})
            return

        if etype == "resume":
            self._emit({"type": "RESUME", "command_id": command_id})
            return

        if etype == "gate_motion" and not bool(effect.get("value", True)):
            self._emit({"type": "DEADMAN_STOP", "command_id": command_id})
            return

        if etype == "cmd_vel_stream":
            self._emit(
                {
                    "type": "CMD_VEL",
                    "command_id": command_id,
                    "payload": packet.get("resolved", {}).get("cmd_vel", {}),
                }
            )
            return

        if etype == "apply_formation":
            self._emit(
                {
                    "type": "FORMATION_TARGETS",
                    "command_id": command_id,
                    "formation": swarm.active_formation_type,
                    "targets": swarm.last_targets,
                }
            )
            return

        if etype == "break_formation":
            self._emit({"type": "BREAK_FORMATION", "command_id": command_id})
            self._emit_state_update(command_id, etype, gesture_state, swarm)
            return

        if etype == "start_behavior":
            self._emit(
                {
                    "type": "BEHAVIOR",
                    "command_id": command_id,
                    "behavior": swarm.active_behavior,
                    "behavior_params": swarm.behavior_params,
                    "targets": swarm.last_targets,
                }
            )
            return

        if etype == "confirm_group_assignment":
            self._emit({"type": "GROUP_ASSIGNMENT_CONFIRMED", "command_id": command_id})
            self._emit_state_update(command_id, etype, gesture_state, swarm)
            return

        if etype == "cancel_group_assignment":
            self._emit({"type": "GROUP_ASSIGNMENT_CANCELED", "command_id": command_id})
            self._emit_state_update(command_id, etype, gesture_state, swarm)
            return

        # Optional telemetry for parameter/state updates
        if etype in {
            "toggle",
            "set_while_held",
            "set_param",
            "step_param",
            "set_state",
            "set_param_stream",
            "select_group",
            "select_robot",
        }:
            self._emit_state_update(command_id, etype, gesture_state, swarm)


class HermesGateway:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.cfg = config

        self.left_latest: Optional[TimedPacket] = None
        self.right_latest: Optional[TimedPacket] = None

        self.glove_timeout_ms = int(config.get("glove_timeout_ms", 200))
        centroid = config.get("centroid", [0.0, 0.0])
        self.centroid_xy = (float(centroid[0]), float(centroid[1]))

        robot_ids = [str(r) for r in config.get("robot_ids", ["r1", "r2", "r3", "r4", "r5", "r6"])]
        self.gesture_state = GestureState()
        self.recognizer = GestureRecognizer()
        self.safety = SafetyEvaluator()
        self.swarm = SwarmController(robot_ids=robot_ids)
        self._apply_behavior_runtime(config.get("behavior_runtime", {}))
        self.adapter = SwarmCommandAdapter(config.get("command_output", {}))

    @staticmethod
    def _xy_pair(value: Any, default: Tuple[float, float]) -> Tuple[float, float]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            return (float(value[0]), float(value[1]))
        if isinstance(value, dict) and "x" in value and "y" in value:
            return (float(value["x"]), float(value["y"]))
        return default

    @staticmethod
    def _parse_path_waypoints(value: Any) -> List[Tuple[float, float]]:
        if not isinstance(value, list):
            return []

        out: List[Tuple[float, float]] = []
        for item in value:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                out.append((float(item[0]), float(item[1])))
                continue
            if isinstance(item, dict) and "x" in item and "y" in item:
                out.append((float(item["x"]), float(item["y"])))
        return out

    def _apply_behavior_runtime(self, runtime_cfg: Any) -> None:
        if not isinstance(runtime_cfg, dict):
            return
        self.swarm.home_xy = self._xy_pair(runtime_cfg.get("home_xy"), self.swarm.home_xy)
        self.swarm.path_waypoints = self._parse_path_waypoints(runtime_cfg.get("path_waypoints"))

    @staticmethod
    def _now_ms() -> int:
        return int(time.monotonic() * 1000)

    def _ingest_hub_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        if not isinstance(msg, dict):
            return

        if msg.get("schema") == "hermes.hub.status":
            print(f"[HUB] {msg}")
            return

        if msg.get("schema") != "hermes.hub.v1":
            return
        if not bool(msg.get("valid_json", False)):
            return

        glove_id = str(msg.get("glove_id", "")).upper()
        packet = msg.get("packet")
        if glove_id not in {"L", "R"} or not isinstance(packet, dict):
            return

        wrapped = TimedPacket(packet=packet, rx_monotonic_ms=self._now_ms())
        if glove_id == "L":
            self.left_latest = wrapped
        else:
            self.right_latest = wrapped

    def _is_fresh(self, pkt: Optional[TimedPacket]) -> bool:
        if pkt is None:
            return False
        age = self._now_ms() - pkt.rx_monotonic_ms
        return age <= self.glove_timeout_ms

    @staticmethod
    def _safe_flex(pkt: Dict[str, Any]) -> Dict[str, float]:
        flex = pkt.get("flex", {})
        return {
            "index": float(flex.get("index", 0.0)),
            "middle": float(flex.get("middle", 0.0)),
            "ring": float(flex.get("ring", 0.0)),
            "pinky": float(flex.get("pinky", 0.0)),
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
    def _safe_left_imu(pkt: Dict[str, Any]) -> Dict[str, float]:
        imu = pkt.get("imu", {})
        return {
            "PITCH": float(imu.get("PITCH", 0.0)),
            "ROLL": float(imu.get("ROLL", 0.0)),
            "YAW": float(imu.get("YAW", 0.0)),
            "AX": float(imu.get("AX", imu.get("X", 0.0))),
            "AY": float(imu.get("AY", imu.get("Y", 0.0))),
            "AZ": float(imu.get("AZ", imu.get("Z", 0.0))),
        }

    def _build_raw(self) -> Optional[Dict[str, Any]]:
        if not self._is_fresh(self.left_latest):
            # Deadman failsafe if the left glove stream is stale. The right glove
            # only carries FSR commands and should not kill motion on its own.
            self.gesture_state.deadman_active = False
            return None

        assert self.left_latest is not None
        left = self.left_latest.packet

        raw = {
            "time_ms": int(time.time() * 1000),
            "flex": {
                "L": self._safe_flex(left),
            },
            "fsr_pressed": {
                "L": self._safe_fsr(left),
            },
            "imu": {
                "L": self._safe_left_imu(left),
            },
        }

        if self._is_fresh(self.right_latest):
            assert self.right_latest is not None
            right = self.right_latest.packet
            raw["flex"]["R"] = self._safe_flex(right)
            raw["fsr_pressed"]["R"] = self._safe_fsr(right)

        return raw

    def _run_pipeline(self, raw: Dict[str, Any]) -> None:
        event = self.recognizer.recognize(raw)
        now_ms = int(raw["time_ms"])

        safety_packet = self.safety.tick(event, self.gesture_state, COMMAND_REGISTRY, now_ms)
        if safety_packet:
            self.swarm.handle_packet(safety_packet, self.gesture_state, centroid_xy=self.centroid_xy)
            self.adapter.dispatch(safety_packet, self.gesture_state, self.swarm)

        packet = match_gesture(event, self.gesture_state, COMMAND_REGISTRY)
        if packet:
            self.swarm.handle_packet(packet, self.gesture_state, centroid_xy=self.centroid_xy)
            self.adapter.dispatch(packet, self.gesture_state, self.swarm)

    def run(self) -> None:
        port = str(self.cfg["serial_port"])
        baudrate = int(self.cfg["baudrate"])

        print(f"[GW] Opening serial: {port} @ {baudrate}")
        with serial.Serial(port, baudrate=baudrate, timeout=0.02) as ser:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self._ingest_hub_line(line)

                raw = self._build_raw()
                if raw is not None:
                    self._run_pipeline(raw)


def load_config(config_path: Path) -> Dict[str, Any]:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if config_path.exists():
        user_cfg = json.loads(config_path.read_text())
        # shallow merge + nested command_output merge
        cfg.update({k: v for k, v in user_cfg.items() if k != "command_output"})
        if "command_output" in user_cfg and isinstance(user_cfg["command_output"], dict):
            cfg["command_output"].update(user_cfg["command_output"])
    return cfg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H.E.R.M.E.S ESP-NOW test gateway for Raspberry Pi")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("config.example.json")),
        help="Path to gateway config JSON",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(Path(args.config))
    gateway = HermesGateway(cfg)
    gateway.run()


if __name__ == "__main__":
    main()
