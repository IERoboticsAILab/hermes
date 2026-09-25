#!/usr/bin/env python3
"""Autonomous scripted run of the three-task H.E.R.M.E.S evaluation.

No human in the loop: this publishes the same /hermes/command_packets a participant
would produce, closes the loop on /hermes/robot_state_beacon, and writes a per-task CSV.

    Task 1  select r1,r2,r3 -> drive as a group into Zone A -> stationary 2 s
    Task 2  drive to Zone B -> LINE -> spacing +1 twice -> COLUMN
    Task 3  PATROL -> vest alert + pause -> wait for the experimenter's clear -> Home

Run on the Pi, with wearables_pi.launch.py (or hermes_ros.launch.py) and the three
robot agents already up:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 ros_version/scripts/experiment_autonomous_run.py --out session01.csv

Geometry check without ROS (works on a laptop):

    python3 ros_version/scripts/experiment_autonomous_run.py --selftest

!! ZONE COORDINATES BELOW ARE PLACEHOLDERS -- measure them in the arena first. !!
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# Hardcoded experiment configuration
# ---------------------------------------------------------------------------

ROBOT_IDS = ["r1", "r2", "r3"]

# Axis-aligned rectangles in the OptiTrack world frame, metres. PLACEHOLDERS.
@dataclass(frozen=True)
class Zone:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def centre(self) -> Tuple[float, float]:
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)


ZONE_A = Zone("Zone A (staging)", 1.20, -0.80, 2.60, 0.80)
ZONE_B = Zone("Zone B (formation)", -0.70, 1.00, 0.70, 2.40)
ZONE_HOME = Zone("Home Zone", -2.60, -0.80, -1.20, 0.80)

TASK_CAP_S = 360.0        # six-minute cap, per task
SETTLE_S = 2.0            # "stationary for two seconds"
STILL_V = 0.03            # m/s, below this a robot counts as stationary
ARRIVE_M = 0.15           # centroid tolerance on a zone centre

DRIVE_HZ = 20.0
DRIVE_KP = 0.6            # m/s per metre of centroid error
DRIVE_V_MAX = 0.25        # m/s, hard cap on the commanded group velocity
YAW_KP = 1.2              # rad/s per rad of heading error (differential mode)
YAW_TOL = 0.25            # rad, turn-in-place threshold (differential mode)
OMEGA_MAX = 0.8           # rad/s
HOLONOMIC = False         # ROSbot 3 PRO is differential: turn-then-go, never commands vy.

FORMATION_SETTLE_S = 8.0  # let the slot auction + drive converge
PATROL_S = 25.0           # patrol before the alert is presented
BEACON_STALE_S = 0.5

ALERT_PULSE_S = 0.8       # vest buzz marking the alert

CMD_TOPIC = "/hermes/command_packets"
VEST_TOPIC = "/hermes/vest_serial_tx"
BEACON_TOPIC = "/hermes/robot_state_beacon"
INTENT_TOPIC = "/hermes/swarm_intent"


# ---------------------------------------------------------------------------
# Pure geometry -- importable and testable without ROS
# ---------------------------------------------------------------------------

def zone_contains(zone: Zone, x: float, y: float) -> bool:
    return zone.x0 <= x <= zone.x1 and zone.y0 <= y <= zone.y1


def centroid(states: Dict[str, dict]) -> Tuple[float, float]:
    n = len(states)
    return (sum(s["x"] for s in states.values()) / n, sum(s["y"] for s in states.values()) / n)


def mean_yaw(states: Dict[str, dict]) -> float:
    """Circular mean -- a plain average wraps badly near +-pi."""
    sx = sum(math.cos(s["yaw"]) for s in states.values())
    sy = sum(math.sin(s["yaw"]) for s in states.values())
    return math.atan2(sy, sx)


def group_velocity(ex: float, ey: float, yaw: float, holonomic: bool = HOLONOMIC) -> Tuple[float, float, float]:
    """World-frame centroid error -> body-frame (vx, vy, omega) for the whole group.

    Every selected robot receives the same body-frame command (that is what the
    teleop/gesture path does), so the rotation uses the group's mean heading.
    """
    dist = math.hypot(ex, ey)
    if dist < 1e-6:
        return (0.0, 0.0, 0.0)
    speed = min(DRIVE_KP * dist, DRIVE_V_MAX)
    bx = (ex * math.cos(yaw) + ey * math.sin(yaw)) / dist
    by = (-ex * math.sin(yaw) + ey * math.cos(yaw)) / dist

    if holonomic:
        return (speed * bx, speed * by, 0.0)

    err = math.atan2(ey, ex) - yaw
    heading_err = math.atan2(math.sin(err), math.cos(err))
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, YAW_KP * heading_err))
    if abs(heading_err) > YAW_TOL:
        return (0.0, 0.0, omega)          # turn in place first
    return (speed * bx, 0.0, omega)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

class ExperimentRunner:
    def __init__(self, node, out_path: str) -> None:
        from std_msgs.msg import String

        self._node = node
        self._String = String
        self._cmd = node.create_publisher(String, CMD_TOPIC, 10)
        self._vest = node.create_publisher(String, VEST_TOPIC, 10)
        node.create_subscription(String, BEACON_TOPIC, self._on_beacon, 50)
        node.create_subscription(String, INTENT_TOPIC, self._on_intent, 10)

        self._states: Dict[str, dict] = {}
        self._intent: dict = {}
        self._vest_seq = 0
        self._lock = threading.Lock()
        self._rows = []
        self._out_path = out_path

    # --- subscriptions -----------------------------------------------------

    def _on_beacon(self, msg) -> None:
        try:
            b = json.loads(msg.data)
            rid = str(b["robot_id"])
        except (ValueError, KeyError, TypeError):
            return
        if rid not in ROBOT_IDS:
            return
        with self._lock:
            self._states[rid] = {
                "x": float(b.get("x", 0.0)), "y": float(b.get("y", 0.0)),
                "yaw": float(b.get("yaw", 0.0)),
                "vx": float(b.get("vx", 0.0)), "vy": float(b.get("vy", 0.0)),
                "rx": time.monotonic(),
            }

    def _on_intent(self, msg) -> None:
        try:
            with self._lock:
                self._intent = json.loads(msg.data)
        except ValueError:
            pass

    def fresh_states(self) -> Dict[str, dict]:
        now = time.monotonic()
        with self._lock:
            return {k: v for k, v in self._states.items() if now - v["rx"] < BEACON_STALE_S}

    def paused(self) -> bool:
        with self._lock:
            return bool(self._intent.get("paused", False))

    # --- command packets ---------------------------------------------------

    def packet(self, command_id: str, command_key: str, effect: dict, resolved: Optional[dict] = None) -> None:
        msg = self._String()
        msg.data = json.dumps(
            {"domain": "teleop", "command_id": command_id, "command_key": command_key,
             "effect": effect, "resolved": resolved or {}},
            separators=(",", ":"),
        )
        self._cmd.publish(msg)

    def select_all(self) -> None:
        self.packet("teleop.selection.all", "SELECT_ALL",
                    {"type": "set_selection", "value": list(ROBOT_IDS)})

    def set_mode(self, mode: str) -> None:
        self.packet(f"teleop.mode.{mode.lower()}", f"MODE_{mode}", {"type": "set_mode", "value": mode})

    def deadman(self, on: bool) -> None:
        self.packet("teleop.deadman", "DEADMAN", {"type": "gate_motion", "value": bool(on)})

    def drive(self, vx: float, vy: float, omega: float) -> None:
        self.packet("teleop.drive.cmd_vel", "DRIVE_CMD_VEL", {"type": "cmd_vel_stream"},
                    {"cmd_vel": {"vx": float(vx), "vy": float(vy), "omega": float(omega)}})

    def apply_formation(self, formation: str) -> None:
        self.packet("teleop.formation.pending", f"SET_{formation}",
                    {"type": "set_state", "key": "pending_formation_type", "value": formation})
        self.packet("teleop.formation.apply", "APPLY_FORMATION", {"type": "apply_formation"})

    def step_spacing(self, delta: int) -> None:
        self.packet("teleop.params.spacing_level", "STEP_SPACING_LEVEL",
                    {"type": "step_param", "key": "spacing_level", "delta": int(delta), "min": 1, "max": 4})

    def start_behavior(self, name: str) -> None:
        self.packet(f"teleop.behavior.{name.lower()}", f"START_{name}",
                    {"type": "start_behavior"}, {"binding": name})

    def pause(self) -> None:
        self.packet("teleop.pause", "PAUSE", {"type": "pause"})

    def resume(self) -> None:
        self.packet("teleop.resume", "RESUME", {"type": "resume"})

    def emergency_stop(self) -> None:
        self.packet("teleop.estop", "EMERGENCY_STOP", {"type": "emergency_stop"})

    def alert_pulse(self) -> float:
        """All six motors on for ALERT_PULSE_S. Returns the alert timestamp."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < ALERT_PULSE_S:
            self._vest_frame(255)
            time.sleep(0.05)
        self._vest_frame(0)
        return t0

    def _vest_frame(self, level: int) -> None:
        self._vest_seq += 1
        msg = self._String()
        msg.data = "V1,{seq},{lv}\n".format(seq=self._vest_seq, lv=",".join([str(level)] * 6))
        self._vest.publish(msg)

    # --- motion primitives -------------------------------------------------

    def drive_to(self, zone: Zone, deadline: float) -> bool:
        """Closed-loop group drive until every robot is inside `zone` and settled."""
        cx_t, cy_t = zone.centre
        self.set_mode("DRIVE")
        period = 1.0 / DRIVE_HZ
        while time.monotonic() < deadline:
            st = self.fresh_states()
            if len(st) < len(ROBOT_IDS):
                self.drive(0.0, 0.0, 0.0)      # missing beacon -> stop, do not dead-reckon
                time.sleep(period)
                continue
            cx, cy = centroid(st)
            inside = all(zone_contains(zone, s["x"], s["y"]) for s in st.values())
            if inside and math.hypot(cx_t - cx, cy_t - cy) < ARRIVE_M:
                break
            self.drive(*group_velocity(cx_t - cx, cy_t - cy, mean_yaw(st)))
            time.sleep(period)
        else:
            self.drive(0.0, 0.0, 0.0)
            return False

        self.drive(0.0, 0.0, 0.0)
        return self.wait_still(zone, deadline)

    def wait_still(self, zone: Optional[Zone], deadline: float, dwell: float = SETTLE_S) -> bool:
        """Every robot below STILL_V (and inside `zone`, if given) for `dwell` seconds."""
        still_since: Optional[float] = None
        while time.monotonic() < deadline:
            st = self.fresh_states()
            ok = len(st) == len(ROBOT_IDS) and all(
                math.hypot(s["vx"], s["vy"]) < STILL_V for s in st.values()
            )
            if ok and zone is not None:
                ok = all(zone_contains(zone, s["x"], s["y"]) for s in st.values())
            now = time.monotonic()
            if not ok:
                still_since = None
            elif still_since is None:
                still_since = now
            elif now - still_since >= dwell:
                return True
            time.sleep(0.05)
        return False

    # --- logging -----------------------------------------------------------

    def record(self, task: str, label: str, t0: float, status: str, notes: str = "") -> None:
        row = {
            "task": task,
            "label": label,
            "start_utc": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(time.time() - (time.monotonic() - t0))),
            "duration_s": round(time.monotonic() - t0, 2),
            "status": status,
            "notes": notes,
        }
        self._rows.append(row)
        self._node.get_logger().info(
            f"[{task}] {status} in {row['duration_s']}s {('- ' + notes) if notes else ''}"
        )

    def write_csv(self) -> None:
        with open(self._out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["task", "label", "start_utc", "duration_s", "status", "notes"])
            w.writeheader()
            w.writerows(self._rows)
        print(f"wrote {self._out_path} ({len(self._rows)} rows)")

    # --- the experiment ----------------------------------------------------

    def task1(self) -> None:
        t0 = time.monotonic()
        deadline = t0 + TASK_CAP_S
        self.select_all()
        self.deadman(True)
        time.sleep(0.3)
        ok = self.drive_to(ZONE_A, deadline)
        self.record("task1", "selection and repositioning", t0,
                    "completed" if ok else "timeout", f"target={ZONE_A.name}")

    def task2(self) -> None:
        t0 = time.monotonic()
        deadline = t0 + TASK_CAP_S
        moved = self.drive_to(ZONE_B, deadline)

        self.set_mode("FORMATION")          # must leave DRIVE or cmd_vel keeps priority
        self.apply_formation("LINE")
        line_ok = self.wait_still(None, min(deadline, time.monotonic() + FORMATION_SETTLE_S), dwell=1.0)

        for _ in range(2):
            self.step_spacing(+1)
            self.wait_still(None, min(deadline, time.monotonic() + FORMATION_SETTLE_S), dwell=1.0)

        self.apply_formation("COLUMN")
        col_ok = self.wait_still(None, min(deadline, time.monotonic() + FORMATION_SETTLE_S), dwell=1.0)

        status = "completed" if (moved and line_ok and col_ok) else "timeout"
        self.record("task2", "formation and spacing reconfiguration", t0, status,
                    f"line={line_ok} spacing_steps=2 column={col_ok} reached_B={moved}")

    def task3(self) -> None:
        t0 = time.monotonic()
        deadline = t0 + TASK_CAP_S

        self.set_mode("BEHAVIOR")
        self.start_behavior("PATROL")
        time.sleep(min(PATROL_S, max(0.0, deadline - time.monotonic())))

        alert_t = self.alert_pulse()        # participant-facing cue; clock starts here
        self.pause()
        while time.monotonic() < deadline and not self.paused():
            time.sleep(0.01)
        pause_latency_ms = round((time.monotonic() - alert_t) * 1000.0, 1)

        print("\n*** ALERT acknowledged, swarm paused. Press ENTER to give the CLEAR signal. ***")
        input()
        self.resume()

        self.set_mode("DRIVE")              # DRIVE short-circuits the active behaviour
        home_ok = self.drive_to(ZONE_HOME, deadline)
        self.record("task3", "behaviour dispatch and interrupt response", t0,
                    "completed" if home_ok else "timeout",
                    f"pause_latency_ms={pause_latency_ms} returned_home={home_ok}")

    def shutdown(self) -> None:
        self.drive(0.0, 0.0, 0.0)
        self.emergency_stop()
        self.deadman(False)
        self._vest_frame(0)
        time.sleep(0.2)


# ---------------------------------------------------------------------------

def selftest() -> None:
    assert zone_contains(ZONE_A, *ZONE_A.centre)
    assert not zone_contains(ZONE_A, ZONE_A.x0 - 0.01, ZONE_A.centre[1])
    for z in (ZONE_A, ZONE_B, ZONE_HOME):
        assert z.x1 > z.x0 and z.y1 > z.y0, f"{z.name} has inverted bounds"

    st = {"r1": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "r2": {"x": 2.0, "y": 0.0, "yaw": 0.0},
          "r3": {"x": 1.0, "y": 3.0, "yaw": 0.0}}
    assert centroid(st) == (1.0, 1.0)

    # circular mean must not average +179 deg and -179 deg into 0
    near_pi = {"a": {"yaw": math.pi - 0.05}, "b": {"yaw": -math.pi + 0.05}}
    assert abs(abs(mean_yaw(near_pi)) - math.pi) < 1e-6

    # differential (the configured mode): 90 deg off -> turn in place, never sideways
    vx, vy, om = group_velocity(1.0, 0.0, math.pi / 2, holonomic=False)
    assert (vx, vy) == (0.0, 0.0) and om < 0, (vx, vy, om)
    # aligned -> forward only, capped, no lateral
    vx, vy, om = group_velocity(10.0, 0.0, 0.0, holonomic=False)
    assert abs(vx - DRIVE_V_MAX) < 1e-9 and vy == 0.0 and abs(om) < 1e-9
    # wrap: target behind at world -x, robot facing +x -> turn, not a 2*pi excursion
    _, _, om = group_velocity(-1.0, -0.01, 0.0, holonomic=False)
    assert abs(om) <= OMEGA_MAX and om < 0, om
    # holonomic branch still correct if the flag is ever flipped back
    vx, vy, _ = group_velocity(1.0, 0.0, math.pi / 2, holonomic=True)
    assert abs(vx) < 1e-9 and vy < 0, (vx, vy)
    assert group_velocity(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
    assert not HOLONOMIC, "ROSbot 3 PRO is differential"
    print("selftest ok")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=f"hermes_experiment_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    ap.add_argument("--selftest", action="store_true", help="check the geometry, no ROS needed")
    ap.add_argument("--task", type=int, choices=[1, 2, 3], action="append",
                    help="run only these tasks (repeatable); default is all three")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0

    import rclpy

    rclpy.init()
    node = rclpy.create_node("hermes_experiment_autonomous")
    runner = ExperimentRunner(node, args.out)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("waiting for all three robot beacons ...")
    while len(runner.fresh_states()) < len(ROBOT_IDS):
        time.sleep(0.2)
    print("beacons up. starting.\n")

    tasks = args.task or [1, 2, 3]
    try:
        if 1 in tasks:
            runner.task1()
        if 2 in tasks:
            runner.task2()
        if 3 in tasks:
            runner.task3()
    except KeyboardInterrupt:
        runner.record("aborted", "operator interrupt", time.monotonic(), "aborted")
    finally:
        runner.shutdown()
        runner.write_csv()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
