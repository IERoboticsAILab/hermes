"""H.E.R.M.E.S 3D viz ROS bridge. Read-only — subscribes only, publishes nothing.

Tasks 16 and 17 layer callback bodies, the DeadmanWatchdog, the HUD timer,
and the Vuer-aware main() function on top of this skeleton. Keep this file's
imports minimal until then.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hermes_viz.transforms import (
    parse_raw_input, parse_robot_state_beacon, parse_swarm_intent,
    parse_command_packet, parse_vest_motors,
)
from hermes_viz.scene.hands import euler_to_quat


@runtime_checkable
class SceneSink(Protocol):
    """Contract the bridge expects from any scene-state consumer."""

    def update_glove(self, side: str, position: tuple, quat: tuple, curl: float, grip: bool) -> None: ...
    def update_robot(self, robot_id: str, x: float, y: float, yaw: float) -> None: ...
    def update_vest_motors(self, motors: tuple) -> None: ...
    def update_gesture(self, gesture: str, command_id: str) -> None: ...
    def update_intent(self, mode: str, deadman_active: bool, active_formation_type: str, stamp_ms: int) -> None: ...
    def push_hud(self) -> None: ...


class VizBridgeNode(Node):
    """Read-only subscriber. Five topics in, zero topics out."""

    def __init__(self, scene_sink: SceneSink):
        super().__init__("hermes_viz_bridge")
        self._sink = scene_sink

        qos = 10  # newest-wins lossy queue (set to keep_last for low latency)
        self.create_subscription(String, "/hermes/raw_input", self._on_raw_input, qos)
        self.create_subscription(String, "/hermes/robot_state_beacon", self._on_robot_state, qos)
        self.create_subscription(String, "/hermes/swarm_intent", self._on_swarm_intent, qos)
        self.create_subscription(String, "/hermes/command_packets", self._on_command, qos)
        self.create_subscription(String, "/hermes/vest_serial_tx", self._on_vest, qos)

    def _on_raw_input(self, msg: String) -> None:
        s = parse_raw_input(msg.data)
        if s is None:
            return

        # Left glove (has IMU + flex + fsr_pressed)
        if s.left is not None and s.left.imu is not None:
            quat = euler_to_quat(
                pitch=s.left.imu.pitch,
                roll=s.left.imu.roll,
                yaw=s.left.imu.yaw,
            )
            flex = s.left.flex
            curl = (flex.index + flex.middle + flex.ring + flex.pinky) / 4.0
            fsr = s.left.fsr_pressed
            grip = fsr.index or fsr.middle or fsr.ring or fsr.pinky
            self._sink.update_glove(
                side="left",
                position=(-0.2, 1.1, 0.5),
                quat=quat,
                curl=curl,
                grip=grip,
            )

        # Right glove (flex + fsr_pressed only — no IMU)
        if s.right is not None:
            flex = s.right.flex
            curl = (flex.index + flex.middle + flex.ring + flex.pinky) / 4.0
            fsr = s.right.fsr_pressed
            grip = fsr.index or fsr.middle or fsr.ring or fsr.pinky
            self._sink.update_glove(
                side="right",
                position=(0.2, 1.1, 0.5),
                quat=(0.0, 0.0, 0.0, 1.0),  # right glove has no orientation
                curl=curl,
                grip=grip,
            )

    def _on_robot_state(self, msg: String) -> None:
        p = parse_robot_state_beacon(msg.data)
        if p is not None:
            self._sink.update_robot(p.robot_id, p.x, p.y, p.yaw)

    def _on_swarm_intent(self, msg: String) -> None:
        intent = parse_swarm_intent(msg.data)
        if intent is not None:
            self._sink.update_intent(
                mode=intent.mode,
                deadman_active=intent.deadman_active,
                active_formation_type=intent.active_formation_type,
                stamp_ms=intent.stamp_ms,
            )

    def _on_command(self, msg: String) -> None:
        c = parse_command_packet(msg.data)
        if c is not None:
            # HUD shows command_key as the human-readable gesture label.
            self._sink.update_gesture(gesture=c.command_key or c.command_id, command_id=c.command_id)

    def _on_vest(self, msg: String) -> None:
        v = parse_vest_motors(msg.data)
        if v is not None:
            self._sink.update_vest_motors(v.levels)


def main(args=None):
    """Placeholder. Task 17 replaces this with the Vuer-aware main()."""
    raise SystemExit("viz_bridge_node.main() is wired by Task 17 (Vuer integration). "
                     "Run via `ros2 launch hermes_viz viz.launch.py` once Task 18 lands.")


if __name__ == "__main__":
    main()
