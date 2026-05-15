"""H.E.R.M.E.S 3D viz ROS bridge. Read-only — subscribes only, publishes nothing.

Tasks 16 and 17 layer callback bodies, the DeadmanWatchdog, the HUD timer,
and the Vuer-aware main() function on top of this skeleton. Keep this file's
imports minimal until then.
"""

from __future__ import annotations

import time
from typing import Protocol, runtime_checkable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hermes_viz.transforms import (
    parse_raw_input, parse_robot_state_beacon, parse_swarm_intent,
    parse_command_packet, parse_vest_motors,
)
from hermes_viz.scene.hands import euler_to_quat


class DeadmanWatchdog:
    """Live iff last observed deadman_active was True AND we saw a message
    from /hermes/swarm_intent within `silence_timeout_s`.

    Source: swarm_control_node._intent_snapshot publishes deadman_active.
    A silent /hermes/swarm_intent means the swarm pipeline has stopped
    publishing — treat as dead.
    """

    def __init__(self, silence_timeout_s: float):
        if silence_timeout_s <= 0:
            raise ValueError("silence_timeout_s must be > 0")
        self._timeout = silence_timeout_s
        self._last_value: bool = False
        self._last_ts: float | None = None

    def observe(self, deadman_active: bool, now_s: float) -> None:
        self._last_value = bool(deadman_active)
        self._last_ts = now_s

    def is_live(self, now_s: float) -> bool:
        if self._last_ts is None:
            return False
        if now_s - self._last_ts > self._timeout:
            return False
        return self._last_value


@runtime_checkable
class SceneSink(Protocol):
    """Contract the bridge expects from any scene-state consumer."""

    def update_glove(self, side: str, position: tuple, quat: tuple, curl: float, grip: bool) -> None: ...
    def update_robot(self, robot_id: str, x: float, y: float, yaw: float) -> None: ...
    def update_vest_motors(self, motors: tuple) -> None: ...
    def update_gesture(self, gesture: str, command_id: str) -> None: ...
    def update_intent(self, mode: str, deadman_active: bool, active_formation_type: str, stamp_ms: int) -> None: ...
    def set_deadman(self, live: bool) -> None: ...
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

        # Deadman watchdog and HUD timer (4 Hz).
        self._deadman = DeadmanWatchdog(silence_timeout_s=0.5)
        self._monotonic = time.monotonic
        self._hud_timer = self.create_timer(0.25, self._tick_hud)

    def is_deadman_live(self) -> bool:
        """Snapshot of the deadman watchdog. Callable from any thread."""
        return self._deadman.is_live(now_s=self._monotonic())

    def _tick_hud(self) -> None:
        """4 Hz timer. Pushes deadman live-state to the sink, then asks
        the sink to render the HUD."""
        self._sink.set_deadman(self.is_deadman_live())
        self._sink.push_hud()

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
            self._deadman.observe(intent.deadman_active, now_s=self._monotonic())
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
    """Spawn Vuer + the bridge node in the same process."""
    import asyncio
    import threading
    from pathlib import Path

    from vuer import Vuer
    from hermes_viz.vuer_sink import VuerSink

    rclpy.init(args=args)

    # The Vuer server serves the staged URDF + meshes from this directory.
    assets_dir = Path(__file__).resolve().parent / "assets" / "rosbot_urdf"

    # Read launch parameters via a transient node (params haven't been
    # declared on the bridge node yet — keep it simple for MVP).
    host = "localhost"
    port = 8012

    app = Vuer(host=host, port=port, static_root=str(assets_dir))

    lab_dims = {"room_w": 6.0, "room_d": 4.0, "room_h": 2.7,
                "optitrack_w": 4.0, "optitrack_d": 3.0}
    robot_ids = ["r1", "r2", "r3", "r4"]

    @app.spawn(start=False)
    async def boot(sess):
        sink = VuerSink(
            session=sess,
            lab_dims=lab_dims,
            robot_ids=robot_ids,
            urdf_src="/static/rosbot.urdf",
        )
        node = VizBridgeNode(scene_sink=sink)

        def _spin():
            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
                rclpy.shutdown()
        threading.Thread(target=_spin, daemon=True).start()

        # Keep the session alive forever (Vuer event loop owns the foreground).
        while True:
            await asyncio.sleep(1.0)

    app.run()


if __name__ == "__main__":
    main()
