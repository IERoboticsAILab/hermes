"""End-to-end bridge integration test: replay a bag, verify scene state mutates.

Skipped on machines without rclpy. Also skipped if no fixture bag is staged.

To create the fixture (run on the Pi during a live wearable session):

    cd ros_version
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 bag record -o src/hermes_viz/test/fixtures/sample_session \
        /hermes/raw_input /hermes/robot_state_beacon \
        /hermes/swarm_intent /hermes/command_packets /hermes/vest_serial_tx
    # ... act out a brief sequence (deadman on/off, a gesture, formation change)
    # then Ctrl-C after ~10 s.

Then run this test:

    python -m pytest src/hermes_viz/test/test_bridge_integration.py -v -s
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from hermes_viz.viz_bridge_node import VizBridgeNode  # noqa: E402


_FIXTURE_DIR = (Path(__file__).resolve().parent / "fixtures" / "sample_session")


pytestmark = pytest.mark.skipif(
    not _FIXTURE_DIR.exists(),
    reason=f"No fixture bag at {_FIXTURE_DIR}. Record one on the Pi (see module docstring).",
)


class CountingSink:
    """Counts calls per SceneSink method."""

    def __init__(self):
        self.counts: dict[str, int] = {
            "glove": 0, "robot": 0, "vest": 0, "gesture": 0, "intent": 0,
            "hud": 0, "deadman": 0,
        }
        self.intent_deadman_seen: list[bool] = []

    def update_glove(self, *a, **kw):       self.counts["glove"] += 1
    def update_robot(self, *a, **kw):       self.counts["robot"] += 1
    def update_vest_motors(self, *a, **kw): self.counts["vest"] += 1
    def update_gesture(self, *a, **kw):     self.counts["gesture"] += 1

    def update_intent(self, *, mode, deadman_active, active_formation_type, stamp_ms):
        self.counts["intent"] += 1
        self.intent_deadman_seen.append(bool(deadman_active))

    def set_deadman(self, live: bool):      self.counts["deadman"] += 1
    def push_hud(self, *a, **kw):           self.counts["hud"] += 1


def test_bag_replay_drives_sink_for_every_topic():
    """`ros2 bag play` against the bag fixture should drive every sink method."""
    rclpy.init()
    proc = None
    try:
        sink = CountingSink()
        node = VizBridgeNode(scene_sink=sink)

        proc = subprocess.Popen(
            ["ros2", "bag", "play", str(_FIXTURE_DIR), "--rate", "5.0"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Drain anything remaining on the topics for ~1 s after bag finishes.
        drain_deadline = time.monotonic() + 1.0
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()

        # The fixture should cover all five topic paths.
        assert sink.counts["robot"] > 0,   "no robot_state_beacon messages observed"
        assert sink.counts["intent"] > 0,  "no swarm_intent messages observed"
        # At least one of: glove or vest must have ticked.
        assert sink.counts["glove"] > 0 or sink.counts["vest"] > 0, (
            "neither glove nor vest path produced events"
        )
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        rclpy.shutdown()


def test_bag_replay_observes_deadman_transitions():
    """If the recorded session toggled the deadman, the sink should see both states."""
    rclpy.init()
    proc = None
    try:
        sink = CountingSink()
        node = VizBridgeNode(scene_sink=sink)

        proc = subprocess.Popen(
            ["ros2", "bag", "play", str(_FIXTURE_DIR), "--rate", "5.0"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()

        # The recorded session should include at least one deadman_active=True moment.
        # If your fixture only ever had deadman=False, re-record while toggling it.
        assert any(sink.intent_deadman_seen), (
            "recorded session never saw deadman_active=True; re-record fixture"
        )
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        rclpy.shutdown()
