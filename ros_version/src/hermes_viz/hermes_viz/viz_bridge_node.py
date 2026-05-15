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

    # Callback stubs. Task 16 fills these.
    def _on_raw_input(self, msg: String) -> None:
        pass

    def _on_robot_state(self, msg: String) -> None:
        pass

    def _on_swarm_intent(self, msg: String) -> None:
        pass

    def _on_command(self, msg: String) -> None:
        pass

    def _on_vest(self, msg: String) -> None:
        pass


def main(args=None):
    """Placeholder. Task 17 replaces this with the Vuer-aware main()."""
    raise SystemExit("viz_bridge_node.main() is wired by Task 17 (Vuer integration). "
                     "Run via `ros2 launch hermes_viz viz.launch.py` once Task 18 lands.")


if __name__ == "__main__":
    main()
