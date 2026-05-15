"""SceneSink implementation that drives a live Vuer session."""

from __future__ import annotations

import time
from typing import Optional

from vuer.schemas import Html

from .scene.builder import build_root_scene, camera_presets, CAMERA_DEFAULT_KEY  # noqa: F401
from .scene.robots import RobotsScene
from .scene.hands import HandsScene
from .hud.hud import HudState, render_hud
from .hud.state import LatencyWindow, PacketRateMeter


class VuerSink:
    """Aggregates state from the bridge and pushes it into a Vuer session.

    Implements the SceneSink protocol implicitly (duck-typed)."""

    def __init__(self, *, session, lab_dims: dict, robot_ids: list[str],
                 urdf_src: str = "/static/rosbot.urdf", trail_max_len: int = 200):
        self._sess = session
        self._robots = RobotsScene(robot_ids=robot_ids, urdf_src=urdf_src,
                                   trail_max_len=trail_max_len)
        self._hands = HandsScene()

        self._latency = LatencyWindow(window_seconds=5.0)
        self._rate_l = PacketRateMeter(time_constant_s=1.0)
        self._rate_r = PacketRateMeter(time_constant_s=1.0)

        self._deadman_live: bool = False
        self._motors: tuple[int, int, int, int, int, int] = (0, 0, 0, 0, 0, 0)
        self._gesture: Optional[str] = None
        self._command_id: str = ""
        self._trails_on: bool = True
        self._trail_n: int = trail_max_len

        # Initial render: push the root scene with current (empty) state.
        self._sess.set @ build_root_scene(
            lab_dims=lab_dims, robots=self._robots, hands=self._hands,
        )

    # ---- SceneSink protocol methods ----

    def update_glove(self, side: str, position, quat, curl: float, grip: bool) -> None:
        self._hands.update_hand(
            side=side,
            position=tuple(position),
            quat=tuple(quat),
            curl=curl,
            grip=grip,
        )
        meter = self._rate_l if side == "left" else self._rate_r
        meter.tick(now_s=time.monotonic())
        # Push the changed hand node into the live scene.
        for kn in self._hands.snapshot_nodes():
            self._sess.upsert @ kn.node

    def update_robot(self, robot_id: str, x: float, y: float, yaw: float) -> None:
        self._robots.update_pose(robot_id, x, y, yaw)
        # Upsert just this robot's bot + trail nodes.
        for kn in self._robots.snapshot_nodes():
            if kn.key.endswith(f"_{robot_id}"):
                self._sess.upsert @ kn.node

    def update_vest_motors(self, motors) -> None:
        self._motors = tuple(motors)  # type: ignore[assignment]

    def update_gesture(self, gesture: str, command_id: str) -> None:
        self._gesture = gesture
        self._command_id = command_id

    def update_intent(self, *, mode: str, deadman_active: bool,
                      active_formation_type: str, stamp_ms: int) -> None:
        # Mode/formation could be shown elsewhere later; deadman flag here is
        # informational — the watchdog in the bridge is the authoritative source
        # because it also tracks silence.
        pass

    def set_deadman(self, live: bool) -> None:
        self._deadman_live = bool(live)

    def push_hud(self) -> None:
        now = time.monotonic()
        state = HudState(
            deadman=self._deadman_live,
            latency_p50=self._latency.p50(now_s=now),
            latency_p95=self._latency.p95(now_s=now),
            rate_left=self._rate_l.rate_hz(now_s=now),
            rate_right=self._rate_r.rate_hz(now_s=now),
            gesture=self._gesture,
            command_id=self._command_id,
            motors=self._motors,
            trails_on=self._trails_on,
            trail_n=self._trail_n,
        )
        self._sess.upsert @ Html(src=render_hud(state), key="hud")

    # ---- Optional latency recording (callable from the bridge if desired) ----

    def record_latency(self, latency_ms: float) -> None:
        self._latency.add(latency_ms, now_s=time.monotonic())
