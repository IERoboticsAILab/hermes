# H.E.R.M.E.S 3D Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a browser-based 3D scene on the lab Pi 5 that mirrors live H.E.R.M.E.S wearable + ROSbot state for the thesis defense, with a debug HUD and per-robot trails.

**Architecture:** New ROS 2 package `hermes_viz`. A read-only `rclpy` bridge subscribes to existing `/hermes/*` topics and pushes scene state into a Vuer server running on the same Pi. Chromium in kiosk mode renders the scene to HDMI. Existing nodes are untouched.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`), Vuer (browser 3D, Three.js under the hood), Chromium kiosk, pytest.

**Spec reference:** [`docs/superpowers/specs/2026-05-14-hermes-3d-viz-design.md`](../specs/2026-05-14-hermes-3d-viz-design.md)

---

## Phase 0 — Foundation

### Task 1: Scaffold the `hermes_viz` ROS 2 package

**Files:**
- Create: `ros_version/src/hermes_viz/package.xml`
- Create: `ros_version/src/hermes_viz/setup.py`
- Create: `ros_version/src/hermes_viz/setup.cfg`
- Create: `ros_version/src/hermes_viz/hermes_viz/__init__.py`
- Create: `ros_version/src/hermes_viz/resource/hermes_viz`
- Create: `ros_version/src/hermes_viz/test/__init__.py`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>hermes_viz</name>
  <version>0.1.0</version>
  <description>Browser-based 3D digital twin for H.E.R.M.E.S</description>
  <maintainer email="saleh@deanna.pro">Saleh Abd-Elrahman</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `setup.py`**

```python
from setuptools import find_packages, setup

package_name = "hermes_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/viz.launch.py"]),
    ],
    install_requires=["setuptools", "vuer", "numpy"],
    zip_safe=True,
    maintainer="Saleh Abd-Elrahman",
    maintainer_email="saleh@deanna.pro",
    description="Browser-based 3D digital twin for H.E.R.M.E.S",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "viz_bridge_node = hermes_viz.viz_bridge_node:main",
        ],
    },
)
```

- [ ] **Step 3: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/hermes_viz
[install]
install_scripts=$base/lib/hermes_viz
```

- [ ] **Step 4: Create marker file and empty `__init__.py` files**

```bash
touch ros_version/src/hermes_viz/resource/hermes_viz
touch ros_version/src/hermes_viz/hermes_viz/__init__.py
touch ros_version/src/hermes_viz/test/__init__.py
```

- [ ] **Step 5: Verify package builds**

Run:
```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_viz
```
Expected: build succeeds, `Summary: 1 package finished`.

- [ ] **Step 6: Commit**

```bash
git add ros_version/src/hermes_viz/
git commit -m "Feat: scaffold hermes_viz ROS 2 package"
```

---

### Task 2: Install Vuer and verify it serves on the Pi

**Files:**
- Modify: `ros_version/src/hermes_viz/setup.py` (already lists vuer in install_requires)

- [ ] **Step 1: Install Vuer in the active Python environment**

```bash
pip install --user vuer
```

- [ ] **Step 2: Smoke-test Vuer with a minimal scene**

Run a one-shot Python command (not a committed file):
```bash
python3 -c "
from vuer import Vuer, VuerSession
from vuer.schemas import Box, Scene
import asyncio

app = Vuer()

@app.spawn(start=True)
async def main(sess: VuerSession):
    sess.set @ Scene(Box(args=[1,1,1], position=[0,0,0]))
    while True:
        await asyncio.sleep(1)
"
```
Expected: server logs `Vuer running on ws://localhost:8012` (or similar port). Open `http://localhost:8012` in a browser — see a cube.

- [ ] **Step 3: Record exact Vuer API surface used**

Note in `ros_version/src/hermes_viz/hermes_viz/__init__.py`:
```python
"""H.E.R.M.E.S 3D visualization package.

Vuer reference (API surface used):
- vuer.Vuer (app/server)
- vuer.VuerSession (per-client session)
- vuer.schemas: Scene, Box, Sphere, Gltf, Plane, AmbientLight, DirectionalLight, Html
- session.set @ <root>   # replace root scene
- session.upsert @ <node>  # add or update node by key
"""
```

- [ ] **Step 4: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/__init__.py
git commit -m "Feat: document Vuer API surface used by hermes_viz"
```

---

## Phase 1 — Pure transforms (TDD, no ROS, no Vuer)

These are pure functions that turn JSON strings into typed dataclasses. They are the *only* place message parsing lives. Test first.

### Task 3: `GloveState` dataclass + `parse_raw_input`

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/transforms.py`
- Create: `ros_version/src/hermes_viz/test/test_transforms_raw_input.py`

> **STOP — ask the user for the sample JSON now.** Required input: a captured `/hermes/raw_input` message from a live run, or the closest equivalent from existing code. Without this, the parsing contract below may be wrong. If the user provides a different schema, adjust the dataclass and tests to match before continuing.

- [ ] **Step 1: Write the failing test (`test_transforms_raw_input.py`)**

```python
import math
import pytest
from hermes_viz.transforms import parse_raw_input, GloveState, FusedSample


def test_parse_raw_input_valid_both_gloves():
    raw = (
        '{"deadman": true, "ts": 1000,'
        ' "left":  {"quat": [0,0,0,1], "flex": [100,100,100,100], "fsr": null},'
        ' "right": {"quat": [0,0,1,0], "flex": null, "fsr": [50,50]}}'
    )
    sample = parse_raw_input(raw)
    assert isinstance(sample, FusedSample)
    assert sample.deadman is True
    assert sample.glove_ts_ms == 1000
    assert sample.left.quat == (0.0, 0.0, 0.0, 1.0)
    assert sample.left.flex == (100, 100, 100, 100)
    assert sample.left.fsr is None
    assert sample.right.quat == (0.0, 0.0, 1.0, 0.0)
    assert sample.right.flex is None
    assert sample.right.fsr == (50, 50)


def test_parse_raw_input_deadman_false():
    raw = '{"deadman": false, "ts": 0, "left": null, "right": null}'
    sample = parse_raw_input(raw)
    assert sample.deadman is False
    assert sample.left is None
    assert sample.right is None


def test_parse_raw_input_malformed_returns_none():
    assert parse_raw_input("not json") is None
    assert parse_raw_input('{"deadman": true}') is None  # missing required fields
    assert parse_raw_input('{"deadman": true, "ts": 0, "left": "bad", "right": null}') is None


def test_parse_raw_input_nan_quat_returns_none():
    raw = '{"deadman": true, "ts": 1, "left": {"quat": [NaN,0,0,1], "flex": null, "fsr": null}, "right": null}'
    # JSON does not allow NaN literal; we expect parse failure
    assert parse_raw_input(raw) is None
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd ros_version/src/hermes_viz
python -m pytest test/test_transforms_raw_input.py -v
```
Expected: ImportError or 4 FAILs ("cannot import name").

- [ ] **Step 3: Write the minimal implementation in `transforms.py`**

```python
"""Pure JSON → dataclass transforms for every H.E.R.M.E.S topic the bridge consumes.

No ROS, no Vuer. Every function returns None on malformed input — never raises.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional


Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class GloveState:
    quat: Quat
    flex: Optional[tuple[int, ...]]
    fsr: Optional[tuple[int, ...]]


@dataclass(frozen=True)
class FusedSample:
    deadman: bool
    glove_ts_ms: int
    left: Optional[GloveState]
    right: Optional[GloveState]


def _parse_glove(d) -> Optional[GloveState]:
    if d is None:
        return None
    if not isinstance(d, dict):
        raise ValueError("glove entry must be object or null")
    quat = d.get("quat")
    if (not isinstance(quat, list) or len(quat) != 4
            or any(not isinstance(v, (int, float)) or math.isnan(float(v)) for v in quat)):
        raise ValueError("bad quat")
    flex = d.get("flex")
    if flex is not None:
        if not isinstance(flex, list) or any(not isinstance(v, int) for v in flex):
            raise ValueError("bad flex")
        flex = tuple(flex)
    fsr = d.get("fsr")
    if fsr is not None:
        if not isinstance(fsr, list) or any(not isinstance(v, int) for v in fsr):
            raise ValueError("bad fsr")
        fsr = tuple(fsr)
    return GloveState(quat=tuple(float(v) for v in quat), flex=flex, fsr=fsr)


def parse_raw_input(raw: str) -> Optional[FusedSample]:
    try:
        d = json.loads(raw)
        return FusedSample(
            deadman=bool(d["deadman"]),
            glove_ts_ms=int(d["ts"]),
            left=_parse_glove(d["left"]),
            right=_parse_glove(d["right"]),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None
```

- [ ] **Step 4: Run tests, verify all pass**

```bash
python -m pytest test/test_transforms_raw_input.py -v
```
Expected: 4 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/transforms.py ros_version/src/hermes_viz/test/test_transforms_raw_input.py
git commit -m "Feat: parse_raw_input transform with malformed-input safety"
```

---

### Task 4: `RobotPose` + `parse_robot_state_beacon`

**Files:**
- Modify: `ros_version/src/hermes_viz/hermes_viz/transforms.py`
- Create: `ros_version/src/hermes_viz/test/test_transforms_robot.py`

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.transforms import parse_robot_state_beacon, RobotPose


def test_parse_robot_state_beacon_valid():
    raw = '{"id": "r2", "x": 1.5, "y": -0.25, "theta": 1.57, "ts": 12345}'
    p = parse_robot_state_beacon(raw)
    assert p == RobotPose(robot_id="r2", x=1.5, y=-0.25, theta=1.57, ts_ms=12345)


def test_parse_robot_state_beacon_unknown_id_accepted():
    # The bridge filters unknown IDs; the transform is dumb on purpose.
    p = parse_robot_state_beacon('{"id": "r99", "x": 0, "y": 0, "theta": 0, "ts": 0}')
    assert p.robot_id == "r99"


def test_parse_robot_state_beacon_malformed_returns_none():
    assert parse_robot_state_beacon("nope") is None
    assert parse_robot_state_beacon('{"id": "r1"}') is None
    assert parse_robot_state_beacon('{"id": "r1", "x": "oops", "y": 0, "theta": 0, "ts": 0}') is None
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_transforms_robot.py -v
```
Expected: ImportError.

- [ ] **Step 3: Add to `transforms.py`**

```python
@dataclass(frozen=True)
class RobotPose:
    robot_id: str
    x: float
    y: float
    theta: float
    ts_ms: int


def parse_robot_state_beacon(raw: str) -> Optional[RobotPose]:
    try:
        d = json.loads(raw)
        return RobotPose(
            robot_id=str(d["id"]),
            x=float(d["x"]),
            y=float(d["y"]),
            theta=float(d["theta"]),
            ts_ms=int(d["ts"]),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/ -v
```
Expected: 7 PASSED total.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/transforms.py ros_version/src/hermes_viz/test/test_transforms_robot.py
git commit -m "Feat: parse_robot_state_beacon transform"
```

---

### Task 5: `parse_vest_motors`, `parse_swarm_intent`, `parse_command_packet`

**Files:**
- Modify: `ros_version/src/hermes_viz/hermes_viz/transforms.py`
- Create: `ros_version/src/hermes_viz/test/test_transforms_misc.py`

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.transforms import (
    parse_vest_motors, parse_swarm_intent, parse_command_packet,
    GestureCommand,
)


def test_parse_vest_motors_valid():
    assert parse_vest_motors("V1,42,200,0,0,255,128,0") == (200, 0, 0, 255, 128, 0)


def test_parse_vest_motors_wrong_prefix_returns_none():
    assert parse_vest_motors("X1,42,0,0,0,0,0,0") is None


def test_parse_vest_motors_wrong_arity_returns_none():
    assert parse_vest_motors("V1,42,0,0,0") is None


def test_parse_swarm_intent_passes_through():
    assert parse_swarm_intent("FORM_V") == "FORM_V"
    assert parse_swarm_intent("") is None


def test_parse_command_packet_with_confidence():
    g = parse_command_packet('{"gesture": "OPEN_PALM", "confidence": 0.87}')
    assert g == GestureCommand(label="OPEN_PALM", confidence=0.87)


def test_parse_command_packet_no_confidence_defaults_to_one():
    g = parse_command_packet('{"gesture": "FIST"}')
    assert g == GestureCommand(label="FIST", confidence=1.0)


def test_parse_command_packet_malformed():
    assert parse_command_packet("not json") is None
    assert parse_command_packet('{"confidence": 0.5}') is None
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_transforms_misc.py -v
```
Expected: ImportError.

- [ ] **Step 3: Add to `transforms.py`**

```python
@dataclass(frozen=True)
class GestureCommand:
    label: str
    confidence: float


def parse_vest_motors(raw: str) -> Optional[tuple[int, int, int, int, int, int]]:
    parts = raw.strip().split(",")
    if len(parts) != 8 or parts[0] != "V1":
        return None
    try:
        m = tuple(int(p) for p in parts[2:8])
    except ValueError:
        return None
    return m  # type: ignore[return-value]


def parse_swarm_intent(raw: str) -> Optional[str]:
    s = raw.strip()
    return s if s else None


def parse_command_packet(raw: str) -> Optional[GestureCommand]:
    try:
        d = json.loads(raw)
        return GestureCommand(
            label=str(d["gesture"]),
            confidence=float(d.get("confidence", 1.0)),
        )
    except (json.JSONDecodeError, KeyError, ValueError, TypeError):
        return None
```

- [ ] **Step 4: Run all transforms tests, verify PASS**

```bash
python -m pytest test/ -v
```
Expected: 14 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/transforms.py ros_version/src/hermes_viz/test/test_transforms_misc.py
git commit -m "Feat: vest motors, swarm intent, command packet transforms"
```

---

### Task 6: `LatencyWindow` — rolling p50/p95

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/hud/__init__.py`
- Create: `ros_version/src/hermes_viz/hermes_viz/hud/state.py`
- Create: `ros_version/src/hermes_viz/test/test_hud_state.py`

- [ ] **Step 1: Create empty package marker**

```bash
touch ros_version/src/hermes_viz/hermes_viz/hud/__init__.py
```

- [ ] **Step 2: Write the failing test**

```python
import pytest
from hermes_viz.hud.state import LatencyWindow


def test_latency_window_empty_returns_none():
    w = LatencyWindow(window_seconds=5.0)
    assert w.p50() is None
    assert w.p95() is None


def test_latency_window_evicts_old_samples():
    w = LatencyWindow(window_seconds=1.0)
    w.add(latency_ms=10.0, now_s=100.0)
    w.add(latency_ms=20.0, now_s=100.5)
    w.add(latency_ms=30.0, now_s=102.0)  # the first two are now > 1s old
    assert w.p50(now_s=102.0) == pytest.approx(30.0)


def test_latency_window_p50_p95():
    w = LatencyWindow(window_seconds=10.0)
    base = 1000.0
    # 100 samples from 1..100 ms
    for i, v in enumerate(range(1, 101)):
        w.add(latency_ms=float(v), now_s=base + i * 0.01)
    now = base + 1.0
    assert w.p50(now_s=now) == pytest.approx(50.5, abs=1.0)
    assert w.p95(now_s=now) == pytest.approx(95.0, abs=1.0)
```

- [ ] **Step 3: Run test, verify FAIL**

```bash
python -m pytest test/test_hud_state.py -v
```
Expected: ImportError.

- [ ] **Step 4: Implement `hud/state.py`**

```python
"""HUD state aggregators. Pure logic. No Vuer, no ROS."""

from __future__ import annotations

import bisect
from collections import deque
from typing import Optional


class LatencyWindow:
    """Rolling window of latency samples. Computes p50/p95 over the window."""

    def __init__(self, window_seconds: float):
        self._window_s = window_seconds
        # deque of (now_s, latency_ms)
        self._samples: deque[tuple[float, float]] = deque()

    def add(self, latency_ms: float, now_s: float) -> None:
        self._samples.append((now_s, latency_ms))
        self._evict(now_s)

    def _evict(self, now_s: float) -> None:
        cutoff = now_s - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _percentile(self, p: float, now_s: Optional[float]) -> Optional[float]:
        if now_s is not None:
            self._evict(now_s)
        if not self._samples:
            return None
        values = sorted(v for _, v in self._samples)
        # nearest-rank
        rank = max(0, min(len(values) - 1, int(round(p / 100.0 * (len(values) - 1)))))
        return values[rank]

    def p50(self, now_s: Optional[float] = None) -> Optional[float]:
        return self._percentile(50.0, now_s)

    def p95(self, now_s: Optional[float] = None) -> Optional[float]:
        return self._percentile(95.0, now_s)
```

- [ ] **Step 5: Run tests, verify PASS**

```bash
python -m pytest test/test_hud_state.py -v
```
Expected: 3 PASSED.

- [ ] **Step 6: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/hud/ ros_version/src/hermes_viz/test/test_hud_state.py
git commit -m "Feat: LatencyWindow rolling p50/p95 aggregator"
```

---

### Task 7: `PacketRateMeter` — exponential decay rate per source

**Files:**
- Modify: `ros_version/src/hermes_viz/hermes_viz/hud/state.py`
- Create: `ros_version/src/hermes_viz/test/test_packet_rate.py`

- [ ] **Step 1: Write the failing test**

```python
import pytest
from hermes_viz.hud.state import PacketRateMeter


def test_packet_rate_zero_before_any_packet():
    m = PacketRateMeter(time_constant_s=1.0)
    assert m.rate_hz(now_s=10.0) == pytest.approx(0.0)


def test_packet_rate_steady_state():
    m = PacketRateMeter(time_constant_s=1.0)
    # tick 50 packets at exactly 10 Hz
    for i in range(50):
        m.tick(now_s=100.0 + i * 0.1)
    rate = m.rate_hz(now_s=100.0 + 49 * 0.1)
    assert 8.0 <= rate <= 12.0, f"expected ~10 Hz, got {rate}"


def test_packet_rate_decays_when_silent():
    m = PacketRateMeter(time_constant_s=1.0)
    for i in range(20):
        m.tick(now_s=i * 0.1)
    rate_at_silence_start = m.rate_hz(now_s=2.0)
    rate_after_5s_silence = m.rate_hz(now_s=7.0)
    assert rate_after_5s_silence < rate_at_silence_start * 0.05
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_packet_rate.py -v
```
Expected: 3 FAILs.

- [ ] **Step 3: Add `PacketRateMeter` to `hud/state.py`**

```python
import math


class PacketRateMeter:
    """Estimates packets/s using single-pole exponential decay.

    rate = EMA(1 / dt) with time constant `time_constant_s`.
    Reads decay toward 0 when idle.
    """

    def __init__(self, time_constant_s: float):
        self._tau = time_constant_s
        self._last_tick_s: Optional[float] = None
        self._rate: float = 0.0

    def tick(self, now_s: float) -> None:
        if self._last_tick_s is None:
            self._last_tick_s = now_s
            return
        dt = max(1e-6, now_s - self._last_tick_s)
        instant = 1.0 / dt
        alpha = 1.0 - math.exp(-dt / self._tau)
        self._rate = self._rate + alpha * (instant - self._rate)
        self._last_tick_s = now_s

    def rate_hz(self, now_s: float) -> float:
        if self._last_tick_s is None:
            return 0.0
        idle = max(0.0, now_s - self._last_tick_s)
        return self._rate * math.exp(-idle / self._tau)
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/test_packet_rate.py -v
```
Expected: 3 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/hud/state.py ros_version/src/hermes_viz/test/test_packet_rate.py
git commit -m "Feat: PacketRateMeter with exponential decay"
```

---

### Task 8: `Trail` — per-robot pose ring buffer

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/__init__.py`
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/trail.py`
- Create: `ros_version/src/hermes_viz/test/test_trail.py`

- [ ] **Step 1: Create scene package marker**

```bash
touch ros_version/src/hermes_viz/hermes_viz/scene/__init__.py
```

- [ ] **Step 2: Write the failing test**

```python
from hermes_viz.scene.trail import Trail


def test_trail_starts_empty():
    t = Trail(max_len=5)
    assert t.points() == []


def test_trail_appends_in_order():
    t = Trail(max_len=5)
    t.push(1.0, 2.0)
    t.push(3.0, 4.0)
    assert t.points() == [(1.0, 2.0), (3.0, 4.0)]


def test_trail_evicts_oldest_when_full():
    t = Trail(max_len=3)
    for i in range(5):
        t.push(float(i), float(i))
    assert t.points() == [(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)]


def test_trail_resize_truncates_oldest():
    t = Trail(max_len=5)
    for i in range(5):
        t.push(float(i), 0.0)
    t.resize(2)
    assert t.points() == [(3.0, 0.0), (4.0, 0.0)]


def test_trail_clear():
    t = Trail(max_len=5)
    t.push(1.0, 1.0)
    t.clear()
    assert t.points() == []
```

- [ ] **Step 3: Run test, verify FAIL**

```bash
python -m pytest test/test_trail.py -v
```
Expected: ImportError.

- [ ] **Step 4: Implement `scene/trail.py`**

```python
"""Per-robot pose ring buffer for trail rendering."""

from __future__ import annotations

from collections import deque


class Trail:
    def __init__(self, max_len: int):
        if max_len < 1:
            raise ValueError("max_len must be >= 1")
        self._buf: deque[tuple[float, float]] = deque(maxlen=max_len)

    def push(self, x: float, y: float) -> None:
        self._buf.append((x, y))

    def resize(self, max_len: int) -> None:
        if max_len < 1:
            raise ValueError("max_len must be >= 1")
        # deque maxlen is read-only; rebuild
        kept = list(self._buf)[-max_len:]
        self._buf = deque(kept, maxlen=max_len)

    def clear(self) -> None:
        self._buf.clear()

    def points(self) -> list[tuple[float, float]]:
        return list(self._buf)
```

- [ ] **Step 5: Run tests, verify PASS**

```bash
python -m pytest test/test_trail.py -v
```
Expected: 5 PASSED.

- [ ] **Step 6: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/scene/ ros_version/src/hermes_viz/test/test_trail.py
git commit -m "Feat: Trail ring buffer for robot motion history"
```

---

## Phase 2 — Vuer scene primitives

### Task 9: Convert ROSbot URDF → GLB

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/assets/rosbot.glb` (binary, committed via git-lfs if available, otherwise direct)
- Create: `ros_version/src/hermes_viz/scripts/urdf_to_glb.py`

> **STOP — ask the user for the URDF path.** Required input: absolute path to the canonical ROSbot URDF used in the lab today, and the path to its mesh dependencies. Without this, this task cannot proceed.

- [ ] **Step 1: Write the conversion helper script**

```python
# scripts/urdf_to_glb.py
"""One-shot converter: URDF (with mesh refs) → single GLB.

Usage:
    python scripts/urdf_to_glb.py <urdf_path> <out_glb_path>

Uses trimesh + yourdfpy. Both pip-installable.
"""
import sys
import yourdfpy
import trimesh


def main(urdf_path: str, out_path: str) -> None:
    robot = yourdfpy.URDF.load(urdf_path)
    scene = trimesh.Scene()
    for link_name, geom in robot.scene.geometry.items():
        scene.add_geometry(geom, node_name=link_name)
    scene.export(out_path)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
```

- [ ] **Step 2: Install deps and run conversion**

```bash
pip install --user yourdfpy trimesh
mkdir -p ros_version/src/hermes_viz/hermes_viz/assets
python scripts/urdf_to_glb.py <USER_URDF_PATH> ros_version/src/hermes_viz/hermes_viz/assets/rosbot.glb
```
Expected: `wrote ros_version/src/hermes_viz/hermes_viz/assets/rosbot.glb`.

- [ ] **Step 3: Visual sanity check**

Open the GLB in an online viewer (e.g., `https://gltf-viewer.donmccurdy.com/`) or VS Code's glTF preview. Confirm bounding box looks like a ROSbot and not garbled.

- [ ] **Step 4: Commit**

```bash
git add ros_version/src/hermes_viz/scripts/ ros_version/src/hermes_viz/hermes_viz/assets/rosbot.glb
git commit -m "Feat: ROSbot URDF→GLB conversion + asset"
```

---

### Task 10: Lab geometry builder (`scene/lab.py`)

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/lab.py`
- Create: `ros_version/src/hermes_viz/hermes_viz/assets/lab/wall.jpg` (placeholder texture for now)
- Create: `ros_version/src/hermes_viz/test/test_lab_builder.py`

> **STOP — ask the user for lab dimensions and lab photos.** Required inputs: rough room W × D × H (meters), OptiTrack volume W × D (meters), and 3–5 photos covering walls + floor. If photos unavailable, use plain gray walls and proceed.

- [ ] **Step 1: Write the failing test (structure-only, no rendering)**

```python
from hermes_viz.scene.lab import build_lab_nodes


def test_build_lab_nodes_returns_expected_groups():
    nodes = build_lab_nodes(room_w=6.0, room_d=4.0, room_h=2.7,
                            optitrack_w=4.0, optitrack_d=3.0)
    keys = {n.key for n in nodes}
    assert "floor" in keys
    assert "wall_north" in keys
    assert "wall_south" in keys
    assert "wall_east" in keys
    assert "wall_west" in keys
    assert "optitrack_volume" in keys


def test_build_lab_nodes_rejects_nonpositive_dims():
    import pytest
    with pytest.raises(ValueError):
        build_lab_nodes(0, 4, 2.7, 4, 3)
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_lab_builder.py -v
```
Expected: ImportError.

- [ ] **Step 3: Implement `scene/lab.py`**

```python
"""Lab geometry: 4 wall planes, floor plane, OptiTrack volume overlay."""

from __future__ import annotations

from dataclasses import dataclass
from vuer.schemas import Plane


@dataclass
class KeyedNode:
    key: str
    node: object  # Vuer schema node


def build_lab_nodes(room_w: float, room_d: float, room_h: float,
                    optitrack_w: float, optitrack_d: float) -> list[KeyedNode]:
    for v in (room_w, room_d, room_h, optitrack_w, optitrack_d):
        if v <= 0:
            raise ValueError("all dimensions must be > 0")

    floor = Plane(args=[room_w, room_d], position=[0, 0, 0],
                  rotation=[-1.5708, 0, 0], material={"color": "#d9d6cf"})
    # Walls placed on the four sides, facing inward.
    n = Plane(args=[room_w, room_h], position=[0, room_h / 2, -room_d / 2],
              material={"color": "#eceae4"})
    s = Plane(args=[room_w, room_h], position=[0, room_h / 2,  room_d / 2],
              rotation=[0, 3.1416, 0], material={"color": "#eceae4"})
    e = Plane(args=[room_d, room_h], position=[ room_w / 2, room_h / 2, 0],
              rotation=[0, -1.5708, 0], material={"color": "#eceae4"})
    w = Plane(args=[room_d, room_h], position=[-room_w / 2, room_h / 2, 0],
              rotation=[0,  1.5708, 0], material={"color": "#eceae4"})

    volume = Plane(args=[optitrack_w, optitrack_d], position=[0, 0.001, 0],
                   rotation=[-1.5708, 0, 0],
                   material={"color": "#c97d4a", "transparent": True, "opacity": 0.15})

    return [
        KeyedNode("floor", floor),
        KeyedNode("wall_north", n),
        KeyedNode("wall_south", s),
        KeyedNode("wall_east", e),
        KeyedNode("wall_west", w),
        KeyedNode("optitrack_volume", volume),
    ]
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/test_lab_builder.py -v
```
Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/scene/lab.py ros_version/src/hermes_viz/test/test_lab_builder.py
git commit -m "Feat: lab geometry builder"
```

---

### Task 11: ROSbot scene nodes (`scene/robots.py`)

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/robots.py`
- Create: `ros_version/src/hermes_viz/test/test_robots_scene.py`

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.scene.robots import RobotsScene


def test_robots_scene_initial_nodes():
    rs = RobotsScene(robot_ids=["r1", "r2"], trail_max_len=10)
    nodes = rs.snapshot_nodes()
    keys = {n.key for n in nodes}
    assert "robot_r1" in keys
    assert "robot_r2" in keys
    assert "trail_r1" in keys
    assert "trail_r2" in keys


def test_robots_scene_update_pose_updates_node_position():
    rs = RobotsScene(robot_ids=["r1"], trail_max_len=10)
    rs.update_pose("r1", x=2.0, y=1.0, theta=0.5)
    nodes = {n.key: n.node for n in rs.snapshot_nodes()}
    bot = nodes["robot_r1"]
    # Vuer uses three-axis position [x, y, z]; we map robot frame xy → world x, z (y=0 floor)
    assert getattr(bot, "position", None) == [2.0, 0.0, 1.0]


def test_robots_scene_update_pose_appends_trail():
    rs = RobotsScene(robot_ids=["r1"], trail_max_len=10)
    rs.update_pose("r1", x=1.0, y=0.0, theta=0.0)
    rs.update_pose("r1", x=1.5, y=0.0, theta=0.0)
    assert rs.trail_points("r1") == [(1.0, 0.0), (1.5, 0.0)]


def test_robots_scene_update_pose_unknown_id_ignored():
    rs = RobotsScene(robot_ids=["r1"], trail_max_len=10)
    rs.update_pose("r99", x=0, y=0, theta=0)  # must not raise
    assert rs.trail_points("r1") == []
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_robots_scene.py -v
```
Expected: ImportError.

- [ ] **Step 3: Implement `scene/robots.py`**

```python
"""ROSbot scene state. Owns 4 GLB instances + per-bot trails."""

from __future__ import annotations

from typing import Iterable
from vuer.schemas import Gltf, Line

from .lab import KeyedNode
from .trail import Trail


_BOT_COLORS = {
    "r1": "#0c8a8e",  # teal
    "r2": "#d49a3c",  # amber
    "r3": "#d05a6e",  # rose
    "r4": "#3f3f9e",  # indigo
}


class RobotsScene:
    """Mutable scene state for the swarm. snapshot_nodes() returns current Vuer nodes."""

    def __init__(self, robot_ids: Iterable[str], trail_max_len: int,
                 glb_path: str = "/static/rosbot.glb"):
        self._ids = list(robot_ids)
        self._poses: dict[str, tuple[float, float, float]] = {rid: (0.0, 0.0, 0.0) for rid in self._ids}
        self._trails: dict[str, Trail] = {rid: Trail(max_len=trail_max_len) for rid in self._ids}
        self._glb_path = glb_path

    def update_pose(self, robot_id: str, x: float, y: float, theta: float) -> None:
        if robot_id not in self._poses:
            return
        self._poses[robot_id] = (x, y, theta)
        self._trails[robot_id].push(x, y)

    def trail_points(self, robot_id: str) -> list[tuple[float, float]]:
        return self._trails[robot_id].points() if robot_id in self._trails else []

    def resize_trails(self, max_len: int) -> None:
        for t in self._trails.values():
            t.resize(max_len)

    def snapshot_nodes(self) -> list[KeyedNode]:
        nodes: list[KeyedNode] = []
        for rid in self._ids:
            x, y, theta = self._poses[rid]
            bot = Gltf(key=f"robot_{rid}", src=self._glb_path,
                       position=[x, 0.0, y], rotation=[0, theta, 0])
            nodes.append(KeyedNode(f"robot_{rid}", bot))
            pts = [[px, 0.02, py] for px, py in self._trails[rid].points()]
            if len(pts) >= 2:
                line = Line(key=f"trail_{rid}", points=pts,
                            color=_BOT_COLORS.get(rid, "#888"))
            else:
                line = Line(key=f"trail_{rid}", points=[[0, -1, 0], [0, -1, 0]],
                            color=_BOT_COLORS.get(rid, "#888"), opacity=0.0)
            nodes.append(KeyedNode(f"trail_{rid}", line))
        return nodes
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/test_robots_scene.py -v
```
Expected: 4 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/scene/robots.py ros_version/src/hermes_viz/test/test_robots_scene.py
git commit -m "Feat: RobotsScene with per-bot trails"
```

---

### Task 12: Floating hand nodes (`scene/hands.py`)

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/hands.py`
- Create: `ros_version/src/hermes_viz/test/test_hands_scene.py`

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.scene.hands import HandsScene


def test_hands_scene_initial_nodes():
    hs = HandsScene()
    keys = {n.key for n in hs.snapshot_nodes()}
    assert "hand_left" in keys
    assert "hand_right" in keys


def test_hands_scene_update_pose():
    hs = HandsScene()
    hs.update_hand("left", position=(0.1, 1.2, 0.3), quat=(0, 0, 0, 1), curl=0.5, grip=False)
    n = {n.key: n.node for n in hs.snapshot_nodes()}["hand_left"]
    assert getattr(n, "position", None) == [0.1, 1.2, 0.3]
    assert getattr(n, "quaternion", None) == [0, 0, 0, 1]


def test_hands_scene_grip_sets_color():
    hs = HandsScene()
    hs.update_hand("right", position=(0, 0, 0), quat=(0, 0, 0, 1), curl=0.0, grip=True)
    n = {n.key: n.node for n in hs.snapshot_nodes()}["hand_right"]
    color = getattr(n, "material", {}).get("color")
    assert color is not None and color != "#d49a6a"  # default skin
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_hands_scene.py -v
```
Expected: ImportError.

- [ ] **Step 3: Implement `scene/hands.py`**

```python
"""Floating hand scene nodes. MVP: pose + binary open/closed curl + grip tint."""

from __future__ import annotations

from typing import Literal
from vuer.schemas import Box

from .lab import KeyedNode


_SKIN = "#d49a6a"
_GRIP = "#b04a3a"


class HandsScene:
    def __init__(self):
        self._left = {"position": (0.0, 0.0, 0.0), "quat": (0.0, 0.0, 0.0, 1.0),
                      "curl": 0.0, "grip": False}
        self._right = dict(self._left)

    def update_hand(self, side: Literal["left", "right"],
                    position: tuple[float, float, float],
                    quat: tuple[float, float, float, float],
                    curl: float, grip: bool) -> None:
        state = self._left if side == "left" else self._right
        state.update(position=position, quat=quat, curl=max(0.0, min(1.0, curl)), grip=grip)

    def snapshot_nodes(self) -> list[KeyedNode]:
        nodes: list[KeyedNode] = []
        for side, state in (("left", self._left), ("right", self._right)):
            # Open hand wider (along x), closed hand contracted on the curl axis.
            curl = state["curl"]
            sx = 0.12 - 0.06 * curl
            color = _GRIP if state["grip"] else _SKIN
            box = Box(key=f"hand_{side}",
                      args=[sx, 0.04, 0.18],
                      position=list(state["position"]),
                      quaternion=list(state["quat"]),
                      material={"color": color})
            nodes.append(KeyedNode(f"hand_{side}", box))
        return nodes
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/test_hands_scene.py -v
```
Expected: 3 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/scene/hands.py ros_version/src/hermes_viz/test/test_hands_scene.py
git commit -m "Feat: HandsScene with floating hand meshes"
```

---

### Task 13: Scene assembly + camera presets (`scene/builder.py`)

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/scene/builder.py`
- Create: `ros_version/src/hermes_viz/test/test_scene_builder.py`

> **STOP — ask the user for OptiTrack frame convention** (assumed: +X right, +Y forward, +Z up). If different, camera preset positions must be adjusted before this task lands.

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.scene.builder import camera_presets, CAMERA_DEFAULT_KEY


def test_camera_presets_keys():
    presets = camera_presets()
    assert "front_elevated" in presets
    assert "top_down" in presets
    assert "orbit" in presets
    assert CAMERA_DEFAULT_KEY == "front_elevated"


def test_camera_preset_shape():
    p = camera_presets()["front_elevated"]
    assert "position" in p and len(p["position"]) == 3
    assert "lookAt" in p and len(p["lookAt"]) == 3
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_scene_builder.py -v
```
Expected: ImportError.

- [ ] **Step 3: Implement `scene/builder.py`**

```python
"""Top-level Vuer scene assembly and camera presets.

Coordinate convention assumed: +X right, +Y up (Vuer/Three.js), +Z toward audience.
Lab frame (OptiTrack) (+X right, +Y forward, +Z up) is mapped by:
    world.x = lab.x
    world.y = lab.z (height)
    world.z = lab.y (depth)
"""

from __future__ import annotations

from vuer.schemas import Scene, AmbientLight, DirectionalLight

from .lab import build_lab_nodes
from .robots import RobotsScene
from .hands import HandsScene


CAMERA_DEFAULT_KEY = "front_elevated"


def camera_presets() -> dict[str, dict]:
    return {
        "front_elevated": {
            "position": [0.0, 2.0, 3.0],
            "lookAt":   [0.0, 0.3, 0.0],
            "fov": 50,
        },
        "top_down": {
            "position": [0.0, 6.0, 0.001],
            "lookAt":   [0.0, 0.0, 0.0],
            "fov": 60,
        },
        "orbit": {
            "position": [4.0, 1.8, 0.0],
            "lookAt":   [0.0, 0.3, 0.0],
            "fov": 50,
            "orbit": {"radius": 4.0, "height": 1.8, "period_s": 30.0},
        },
    }


def build_root_scene(lab_dims: dict, robots: RobotsScene, hands: HandsScene) -> Scene:
    """Compose the full scene graph for the initial render.

    lab_dims: {"room_w": ..., "room_d": ..., "room_h": ..., "optitrack_w": ..., "optitrack_d": ...}
    """
    children = []
    for kn in build_lab_nodes(**lab_dims):
        children.append(kn.node)
    for kn in robots.snapshot_nodes():
        children.append(kn.node)
    for kn in hands.snapshot_nodes():
        children.append(kn.node)
    children.append(AmbientLight(intensity=0.4))
    children.append(DirectionalLight(intensity=0.8, position=[2, 4, 2]))
    return Scene(*children)
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest test/test_scene_builder.py -v
```
Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/scene/builder.py ros_version/src/hermes_viz/test/test_scene_builder.py
git commit -m "Feat: scene assembly + 3 camera presets"
```

---

## Phase 3 — HUD overlay

### Task 14: HUD state aggregator + render template

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/hud/hud.py`
- Create: `ros_version/src/hermes_viz/hermes_viz/hud/hud_template.html`
- Create: `ros_version/src/hermes_viz/test/test_hud_render.py`

- [ ] **Step 1: Write the failing test**

```python
from hermes_viz.hud.hud import HudState, render_hud


def test_render_hud_deadman_off():
    s = HudState(deadman=False, latency_p50=None, latency_p95=None,
                 rate_left=0.0, rate_right=0.0, gesture=None, confidence=0.0,
                 motors=(0, 0, 0, 0, 0, 0), trails_on=True, trail_n=200)
    html = render_hud(s)
    assert "STOPPED" in html
    assert "LIVE" not in html


def test_render_hud_live_with_values():
    s = HudState(deadman=True, latency_p50=12.3, latency_p95=33.4,
                 rate_left=48.0, rate_right=47.5, gesture="FIST", confidence=0.82,
                 motors=(255, 0, 0, 128, 0, 0), trails_on=True, trail_n=200)
    html = render_hud(s)
    assert "LIVE" in html
    assert "12" in html  # p50 rendered
    assert "FIST" in html
    assert "82%" in html or "0.82" in html


def test_render_hud_latency_unavailable_label():
    s = HudState(deadman=True, latency_p50=None, latency_p95=None,
                 rate_left=10.0, rate_right=10.0, gesture=None, confidence=0.0,
                 motors=(0,)*6, trails_on=False, trail_n=200)
    html = render_hud(s)
    assert "unavailable" in html.lower()
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest test/test_hud_render.py -v
```
Expected: ImportError.

- [ ] **Step 3: Create `hud/hud_template.html`**

```html
<div class="hud" style="position:fixed;top:16px;right:16px;width:280px;
     background:#1a1a1aee;color:#f6f6f4;font-family:ui-sans-serif,system-ui,sans-serif;
     padding:14px 16px;border-radius:10px;box-shadow:0 4px 18px #0006">
  <div class="deadman" style="font-size:28px;font-weight:700;text-align:center;
       padding:10px;border-radius:6px;margin-bottom:10px;background:{{DEADMAN_BG}}">
    {{DEADMAN_TEXT}}
  </div>
  <div style="font-size:11px;text-transform:uppercase;letter-spacing:1px;color:#aaa">Latency</div>
  <div style="font-size:14px;margin-bottom:8px">{{LATENCY_LINE}}</div>
  <div style="font-size:11px;text-transform:uppercase;letter-spacing:1px;color:#aaa">Packet rate</div>
  <div style="font-size:14px;margin-bottom:8px">L {{RATE_L}} Hz · R {{RATE_R}} Hz</div>
  <div style="font-size:11px;text-transform:uppercase;letter-spacing:1px;color:#aaa">Gesture</div>
  <div style="font-size:16px;font-weight:600">{{GESTURE_LABEL}}</div>
  <div style="height:6px;background:#333;border-radius:3px;margin:4px 0 8px 0;overflow:hidden">
    <div style="height:100%;width:{{CONFIDENCE_PCT}}%;background:#0c8a8e"></div>
  </div>
  <div style="font-size:11px;text-transform:uppercase;letter-spacing:1px;color:#aaa">Vest motors</div>
  <div style="display:flex;gap:6px;margin:4px 0 8px 0">{{MOTOR_DOTS}}</div>
  <div style="font-size:11px;color:#aaa">Trails: {{TRAIL_TEXT}}</div>
</div>
```

- [ ] **Step 4: Implement `hud/hud.py`**

```python
"""HUD state + HTML render. Pure template substitution; no Vuer specifics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional


_TEMPLATE_PATH = Path(__file__).parent / "hud_template.html"


@dataclass(frozen=True)
class HudState:
    deadman: bool
    latency_p50: Optional[float]
    latency_p95: Optional[float]
    rate_left: float
    rate_right: float
    gesture: Optional[str]
    confidence: float
    motors: tuple[int, int, int, int, int, int]
    trails_on: bool
    trail_n: int


def _latency_line(s: HudState) -> str:
    if s.latency_p50 is None or s.latency_p95 is None:
        return "<span style='color:#999'>unavailable</span>"
    return f"p50 {int(s.latency_p50)} ms · p95 {int(s.latency_p95)} ms"


def _motor_dots(motors: tuple[int, ...]) -> str:
    out = []
    for m in motors:
        opacity = max(0.15, m / 255.0)
        out.append(
            f"<div style='width:24px;height:24px;border-radius:50%;"
            f"background:#0c8a8e;opacity:{opacity:.2f}'></div>"
        )
    return "".join(out)


def render_hud(s: HudState) -> str:
    template = _TEMPLATE_PATH.read_text()
    deadman_text = "LIVE" if s.deadman else "STOPPED"
    deadman_bg = "#2a8c4a" if s.deadman else "#a83232"
    return (
        template
        .replace("{{DEADMAN_TEXT}}", deadman_text)
        .replace("{{DEADMAN_BG}}", deadman_bg)
        .replace("{{LATENCY_LINE}}", _latency_line(s))
        .replace("{{RATE_L}}", f"{s.rate_left:.0f}")
        .replace("{{RATE_R}}", f"{s.rate_right:.0f}")
        .replace("{{GESTURE_LABEL}}", s.gesture or "—")
        .replace("{{CONFIDENCE_PCT}}", f"{int(s.confidence * 100)}")
        .replace("{{MOTOR_DOTS}}", _motor_dots(s.motors))
        .replace("{{TRAIL_TEXT}}", f"{'on' if s.trails_on else 'off'} · N={s.trail_n}")
    )
```

- [ ] **Step 5: Run tests, verify PASS**

```bash
python -m pytest test/test_hud_render.py -v
```
Expected: 3 PASSED.

- [ ] **Step 6: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/hud/ ros_version/src/hermes_viz/test/test_hud_render.py
git commit -m "Feat: HUD state aggregator + HTML template"
```

---

## Phase 4 — ROS bridge

### Task 15: `VizBridgeNode` skeleton with subscriptions

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py`
- Create: `ros_version/src/hermes_viz/test/test_bridge_subscriptions.py`

- [ ] **Step 1: Write the failing test**

```python
import rclpy
from rclpy.node import Node
from hermes_viz.viz_bridge_node import VizBridgeNode


def test_bridge_node_subscribes_to_expected_topics():
    rclpy.init()
    try:
        node = VizBridgeNode(scene_sink=_NullSink())
        subs = {s.topic_name for s in node.subscriptions}
        assert "/hermes/raw_input" in subs
        assert "/hermes/robot_state_beacon" in subs
        assert "/hermes/swarm_intent" in subs
        assert "/hermes/command_packets" in subs
        assert "/hermes/vest_serial_tx" in subs
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_bridge_node_publishes_nothing():
    rclpy.init()
    try:
        node = VizBridgeNode(scene_sink=_NullSink())
        assert list(node.publishers) == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


class _NullSink:
    def update_glove(self, *a, **kw): pass
    def update_robot(self, *a, **kw): pass
    def update_vest_motors(self, *a, **kw): pass
    def update_gesture(self, *a, **kw): pass
    def update_intent(self, *a, **kw): pass
    def push_hud(self, *a, **kw): pass
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
python -m pytest src/hermes_viz/test/test_bridge_subscriptions.py -v
```
Expected: ImportError.

- [ ] **Step 3: Implement `viz_bridge_node.py` skeleton**

```python
"""H.E.R.M.E.S 3D viz ROS bridge. Read-only: subscribes only, publishes nothing."""

from __future__ import annotations

from typing import Protocol
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SceneSink(Protocol):
    def update_glove(self, side: str, position, quat, curl: float, grip: bool) -> None: ...
    def update_robot(self, robot_id: str, x: float, y: float, theta: float) -> None: ...
    def update_vest_motors(self, motors: tuple) -> None: ...
    def update_gesture(self, label: str, confidence: float) -> None: ...
    def update_intent(self, intent: str) -> None: ...
    def push_hud(self) -> None: ...


class VizBridgeNode(Node):
    def __init__(self, scene_sink: SceneSink):
        super().__init__("hermes_viz_bridge")
        self._sink = scene_sink
        qos = 10  # newest-wins lossy
        self.create_subscription(String, "/hermes/raw_input", self._on_raw_input, qos)
        self.create_subscription(String, "/hermes/robot_state_beacon", self._on_robot_state, qos)
        self.create_subscription(String, "/hermes/swarm_intent", self._on_swarm_intent, qos)
        self.create_subscription(String, "/hermes/command_packets", self._on_command, qos)
        self.create_subscription(String, "/hermes/vest_serial_tx", self._on_vest, qos)

    # Callback stubs (Task 16 wires them).
    def _on_raw_input(self, msg: String) -> None: pass
    def _on_robot_state(self, msg: String) -> None: pass
    def _on_swarm_intent(self, msg: String) -> None: pass
    def _on_command(self, msg: String) -> None: pass
    def _on_vest(self, msg: String) -> None: pass


def main(args=None):
    rclpy.init(args=args)
    # The real sink wires Vuer in Task 17. For now, abort if run standalone.
    raise SystemExit("viz_bridge_node entry point is wired in Task 17 (vuer integration)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest src/hermes_viz/test/test_bridge_subscriptions.py -v
```
Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py ros_version/src/hermes_viz/test/test_bridge_subscriptions.py
git commit -m "Feat: VizBridgeNode skeleton with read-only subscriptions"
```

---

### Task 16: Wire transform callbacks → SceneSink

**Files:**
- Modify: `ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py`
- Create: `ros_version/src/hermes_viz/test/test_bridge_callbacks.py`

- [ ] **Step 1: Write the failing test**

```python
import rclpy
from std_msgs.msg import String
from hermes_viz.viz_bridge_node import VizBridgeNode


class RecordingSink:
    def __init__(self):
        self.events = []
    def update_glove(self, *a, **kw): self.events.append(("glove", a, kw))
    def update_robot(self, *a, **kw): self.events.append(("robot", a, kw))
    def update_vest_motors(self, *a, **kw): self.events.append(("vest", a, kw))
    def update_gesture(self, *a, **kw): self.events.append(("gesture", a, kw))
    def update_intent(self, *a, **kw): self.events.append(("intent", a, kw))
    def push_hud(self, *a, **kw): pass


def _msg(text):
    m = String()
    m.data = text
    return m


def test_robot_state_callback_invokes_sink():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_robot_state(_msg('{"id": "r2", "x": 1.0, "y": 2.0, "theta": 0.5, "ts": 1}'))
        kinds = [e[0] for e in sink.events]
        assert "robot" in kinds
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_malformed_raw_input_does_not_raise():
    rclpy.init()
    try:
        sink = RecordingSink()
        node = VizBridgeNode(scene_sink=sink)
        node._on_raw_input(_msg("not json"))  # must not raise
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest src/hermes_viz/test/test_bridge_callbacks.py -v
```
Expected: AssertionError on first test (no "robot" event because callback is pass).

- [ ] **Step 3: Replace stub callbacks in `viz_bridge_node.py`**

Replace the five `pass` callback bodies with:

```python
    def _on_raw_input(self, msg: String) -> None:
        from hermes_viz.transforms import parse_raw_input
        s = parse_raw_input(msg.data)
        if s is None:
            return
        if s.left is not None:
            curl = 1.0 - (sum(s.left.flex) / (4 * 1023)) if s.left.flex else 0.0
            self._sink.update_glove(
                side="left",
                position=(-0.2, 1.1, 0.5),  # placeholder; integrate IMU position later
                quat=s.left.quat,
                curl=max(0.0, min(1.0, curl)),
                grip=bool(s.left.fsr and any(v > 200 for v in s.left.fsr)),
            )
        if s.right is not None:
            curl = 1.0 - (sum(s.right.flex) / (4 * 1023)) if s.right.flex else 0.0
            self._sink.update_glove(
                side="right",
                position=(0.2, 1.1, 0.5),
                quat=s.right.quat,
                curl=max(0.0, min(1.0, curl)),
                grip=bool(s.right.fsr and any(v > 200 for v in s.right.fsr)),
            )

    def _on_robot_state(self, msg: String) -> None:
        from hermes_viz.transforms import parse_robot_state_beacon
        p = parse_robot_state_beacon(msg.data)
        if p is not None:
            self._sink.update_robot(p.robot_id, p.x, p.y, p.theta)

    def _on_swarm_intent(self, msg: String) -> None:
        from hermes_viz.transforms import parse_swarm_intent
        intent = parse_swarm_intent(msg.data)
        if intent is not None:
            self._sink.update_intent(intent)

    def _on_command(self, msg: String) -> None:
        from hermes_viz.transforms import parse_command_packet
        g = parse_command_packet(msg.data)
        if g is not None:
            self._sink.update_gesture(g.label, g.confidence)

    def _on_vest(self, msg: String) -> None:
        from hermes_viz.transforms import parse_vest_motors
        motors = parse_vest_motors(msg.data)
        if motors is not None:
            self._sink.update_vest_motors(motors)
```

- [ ] **Step 4: Run tests, verify PASS**

```bash
python -m pytest src/hermes_viz/test/test_bridge_callbacks.py -v
```
Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py ros_version/src/hermes_viz/test/test_bridge_callbacks.py
git commit -m "Feat: wire bridge callbacks through transforms"
```

---

### Task 17: Vuer-backed `SceneSink` implementation + 4 Hz HUD tick + deadman silence

**Files:**
- Create: `ros_version/src/hermes_viz/hermes_viz/vuer_sink.py`
- Modify: `ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py` (add timer + deadman watchdog + `main`)
- Create: `ros_version/src/hermes_viz/test/test_deadman_watchdog.py`

- [ ] **Step 1: Write the failing deadman watchdog test**

```python
from hermes_viz.viz_bridge_node import DeadmanWatchdog


def test_deadman_starts_dead():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    assert w.is_live(now_s=0.0) is False


def test_deadman_becomes_live_on_true_msg():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman=True, now_s=1.0)
    assert w.is_live(now_s=1.1) is True


def test_deadman_goes_dead_on_false_msg():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman=True, now_s=1.0)
    w.observe(deadman=False, now_s=1.1)
    assert w.is_live(now_s=1.2) is False


def test_deadman_goes_dead_on_silence():
    w = DeadmanWatchdog(silence_timeout_s=0.5)
    w.observe(deadman=True, now_s=1.0)
    assert w.is_live(now_s=1.4) is True   # within window
    assert w.is_live(now_s=1.6) is False  # silent > 500 ms
```

- [ ] **Step 2: Run test, verify FAIL**

```bash
python -m pytest src/hermes_viz/test/test_deadman_watchdog.py -v
```
Expected: ImportError.

- [ ] **Step 3: Add `DeadmanWatchdog` to `viz_bridge_node.py`**

Insert near the top:

```python
class DeadmanWatchdog:
    """Live iff last observed deadman was True AND we saw a message within timeout."""

    def __init__(self, silence_timeout_s: float):
        self._timeout = silence_timeout_s
        self._last_value: bool = False
        self._last_ts: float | None = None

    def observe(self, deadman: bool, now_s: float) -> None:
        self._last_value = deadman
        self._last_ts = now_s

    def is_live(self, now_s: float) -> bool:
        if self._last_ts is None:
            return False
        if now_s - self._last_ts > self._timeout:
            return False
        return self._last_value
```

Also patch `_on_raw_input` to call `self._deadman.observe(s.deadman, self.get_clock().now().nanoseconds * 1e-9)` before processing.

Initialize in `__init__`:

```python
        self._deadman = DeadmanWatchdog(silence_timeout_s=0.5)
        self._hud_timer = self.create_timer(0.25, self._tick_hud)  # 4 Hz

    def _tick_hud(self) -> None:
        # Sink owns HUD aggregation/rendering.
        self._sink.push_hud()
```

- [ ] **Step 4: Implement `vuer_sink.py`**

```python
"""SceneSink implementation that drives a Vuer session."""

from __future__ import annotations

import time
from typing import Optional
from vuer import Vuer, VuerSession
from vuer.schemas import Html

from .scene.builder import build_root_scene, camera_presets, CAMERA_DEFAULT_KEY
from .scene.robots import RobotsScene
from .scene.hands import HandsScene
from .hud.hud import HudState, render_hud
from .hud.state import LatencyWindow, PacketRateMeter


class VuerSink:
    def __init__(self, app: Vuer, session: VuerSession, lab_dims: dict, robot_ids: list[str]):
        self._app = app
        self._sess = session
        self._robots = RobotsScene(robot_ids=robot_ids, trail_max_len=200)
        self._hands = HandsScene()

        self._latency = LatencyWindow(window_seconds=5.0)
        self._rate_l = PacketRateMeter(time_constant_s=1.0)
        self._rate_r = PacketRateMeter(time_constant_s=1.0)

        self._deadman_state = False
        self._motors = (0,) * 6
        self._gesture: Optional[str] = None
        self._confidence = 0.0
        self._intent: Optional[str] = None
        self._trails_on = True
        self._trail_n = 200

        # Initial render
        self._sess.set @ build_root_scene(lab_dims, self._robots, self._hands)
        preset = camera_presets()[CAMERA_DEFAULT_KEY]
        self._sess.upsert @ ("camera", preset)  # API surface may vary; verify via docs.vuer.ai skill before this task

    def update_glove(self, side, position, quat, curl, grip):
        self._hands.update_hand(side, tuple(position), tuple(quat), curl, grip)
        meter = self._rate_l if side == "left" else self._rate_r
        meter.tick(now_s=time.monotonic())
        for kn in self._hands.snapshot_nodes():
            self._sess.upsert @ kn.node

    def update_robot(self, robot_id, x, y, theta):
        self._robots.update_pose(robot_id, x, y, theta)
        for kn in self._robots.snapshot_nodes():
            if kn.key.startswith(f"robot_{robot_id}") or kn.key.startswith(f"trail_{robot_id}"):
                self._sess.upsert @ kn.node

    def update_vest_motors(self, motors):
        self._motors = motors

    def update_gesture(self, label, confidence):
        self._gesture = label
        self._confidence = confidence

    def update_intent(self, intent):
        self._intent = intent

    def set_deadman(self, live: bool) -> None:
        self._deadman_state = live

    def push_hud(self):
        now = time.monotonic()
        s = HudState(
            deadman=self._deadman_state,
            latency_p50=self._latency.p50(now_s=now),
            latency_p95=self._latency.p95(now_s=now),
            rate_left=self._rate_l.rate_hz(now_s=now),
            rate_right=self._rate_r.rate_hz(now_s=now),
            gesture=self._gesture,
            confidence=self._confidence,
            motors=self._motors,
            trails_on=self._trails_on,
            trail_n=self._trail_n,
        )
        self._sess.upsert @ Html(key="hud", src=render_hud(s))

    def record_latency(self, latency_ms: float) -> None:
        self._latency.add(latency_ms, now_s=time.monotonic())
```

- [ ] **Step 5: Wire `main()` to spawn Vuer + bridge together**

Replace `main()` in `viz_bridge_node.py`:

```python
def main(args=None):
    import threading
    import time
    from vuer import Vuer

    rclpy.init(args=args)

    app = Vuer()
    lab_dims = {"room_w": 6.0, "room_d": 4.0, "room_h": 2.7,
                "optitrack_w": 4.0, "optitrack_d": 3.0}  # overridable by launch params later
    robot_ids = ["r1", "r2", "r3", "r4"]

    sink_holder: dict = {}

    @app.spawn(start=False)
    async def _vuer_main(sess):
        from hermes_viz.vuer_sink import VuerSink
        sink = VuerSink(app, sess, lab_dims, robot_ids)
        sink_holder["sink"] = sink
        node = VizBridgeNode(scene_sink=sink)

        def _spin():
            rclpy.spin(node)
        t = threading.Thread(target=_spin, daemon=True)
        t.start()
        while True:
            await asyncio.sleep(0.25)
            sink.set_deadman(node._deadman.is_live(time.monotonic()))

    import asyncio
    app.run()
```

- [ ] **Step 6: Run all unit tests, verify PASS**

```bash
cd ros_version/src/hermes_viz
python -m pytest test/ -v
```
Expected: all green.

- [ ] **Step 7: Commit**

```bash
git add ros_version/src/hermes_viz/
git commit -m "Feat: Vuer-backed SceneSink + 4Hz HUD tick + deadman watchdog"
```

---

## Phase 5 — Launch, integration, manual verification

### Task 18: Launch file

**Files:**
- Create: `ros_version/src/hermes_viz/launch/viz.launch.py`

- [ ] **Step 1: Create launch file**

```python
"""Launch the viz bridge + Chromium kiosk pointed at the local Vuer port."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package="hermes_viz",
        executable="viz_bridge_node",
        name="hermes_viz_bridge",
        output="screen",
    )
    chromium = ExecuteProcess(
        cmd=["chromium-browser", "--kiosk", "--app=http://localhost:8012",
             "--disable-features=TranslateUI", "--noerrdialogs"],
        output="log",
    )
    return LaunchDescription([bridge, chromium])
```

- [ ] **Step 2: Build the package**

```bash
cd ros_version
colcon build --symlink-install --packages-select hermes_viz
source install/setup.bash
```
Expected: build succeeds.

- [ ] **Step 3: Smoke-launch (without wearables stack running)**

```bash
ros2 launch hermes_viz viz.launch.py
```
Expected: Vuer log line printed; Chromium opens to lab scene with 4 stationary bots, 2 hands, lab walls; HUD on the right showing `STOPPED` deadman.

- [ ] **Step 4: Kill cleanly, commit**

```bash
git add ros_version/src/hermes_viz/launch/
git commit -m "Feat: viz.launch.py — bridge + chromium kiosk"
```

---

### Task 19: Integration test — bag replay against stub renderer

**Files:**
- Create: `ros_version/src/hermes_viz/test/test_bridge_integration.py`
- Create: `ros_version/src/hermes_viz/test/fixtures/sample_session.bag/` (recorded in this task)

- [ ] **Step 1: Record a short bag with the wearables stack live (or use a synthetic bag)**

```bash
ros2 bag record -o ros_version/src/hermes_viz/test/fixtures/sample_session \
  /hermes/raw_input /hermes/robot_state_beacon \
  /hermes/swarm_intent /hermes/command_packets /hermes/vest_serial_tx
# ... record ~10s of motion, then Ctrl+C
```

If the wearables stack is unavailable in the current environment, generate a synthetic bag in Step 2 instead.

- [ ] **Step 2 (alt): Generate a synthetic bag**

```python
# scripts/make_synthetic_bag.py — one-time helper, not committed
# Use rosbag2_py to write 30 raw_input messages, 30 robot_state_beacon messages, etc.
# See https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html
```

- [ ] **Step 3: Write the integration test**

```python
import subprocess, time
import rclpy
from std_msgs.msg import String
from hermes_viz.viz_bridge_node import VizBridgeNode


class CountingSink:
    def __init__(self):
        self.counts = {"glove": 0, "robot": 0, "vest": 0, "gesture": 0, "intent": 0, "hud": 0}
    def update_glove(self, *a, **kw): self.counts["glove"] += 1
    def update_robot(self, *a, **kw): self.counts["robot"] += 1
    def update_vest_motors(self, *a, **kw): self.counts["vest"] += 1
    def update_gesture(self, *a, **kw): self.counts["gesture"] += 1
    def update_intent(self, *a, **kw): self.counts["intent"] += 1
    def push_hud(self, *a, **kw): self.counts["hud"] += 1


def test_bag_replay_drives_sink():
    rclpy.init()
    try:
        sink = CountingSink()
        node = VizBridgeNode(scene_sink=sink)
        proc = subprocess.Popen([
            "ros2", "bag", "play",
            "src/hermes_viz/test/fixtures/sample_session",
            "--rate", "5.0",
        ])
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        proc.terminate()
        assert sink.counts["robot"] > 0
        assert sink.counts["glove"] > 0 or sink.counts["vest"] > 0  # at least one wearable path
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

- [ ] **Step 4: Run the integration test**

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python -m pytest src/hermes_viz/test/test_bridge_integration.py -v -s
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros_version/src/hermes_viz/test/test_bridge_integration.py ros_version/src/hermes_viz/test/fixtures/
git commit -m "Test: bag replay drives bridge sink end-to-end"
```

---

### Task 20: Pi-side performance check + laptop-fallback knob

**Files:**
- Modify: `ros_version/src/hermes_viz/launch/viz.launch.py` (add `host` launch arg)

- [ ] **Step 1: Boot the full stack on Pi, observe FPS**

```bash
ros2 launch hermes_viz viz.launch.py
# In Chromium, open DevTools → Performance → record 10s
```
Expected: ≥ 30 FPS sustained. If < 30 FPS:
  - Reduce robot mesh detail (re-export GLB with mesh decimation).
  - Drop wall plane count from 4 to 2 (leave only walls visible in default camera).
  - Disable trails by default (`trails_on=False`).

- [ ] **Step 2: Add `host` launch arg for laptop fallback**

Replace `viz.launch.py` body:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="localhost",
                              description="Host the Vuer server binds to (use 0.0.0.0 for laptop fallback)"),
        Node(package="hermes_viz", executable="viz_bridge_node",
             name="hermes_viz_bridge", output="screen",
             parameters=[{"vuer_host": host}]),
        ExecuteProcess(cmd=["chromium-browser", "--kiosk", "--app=http://localhost:8012",
                            "--disable-features=TranslateUI", "--noerrdialogs"],
                       output="log"),
    ])
```

(Then read `vuer_host` param in `viz_bridge_node.main()` and pass to `Vuer(host=...)` — exact API per Vuer docs.)

- [ ] **Step 3: Commit**

```bash
git add ros_version/src/hermes_viz/launch/viz.launch.py ros_version/src/hermes_viz/hermes_viz/viz_bridge_node.py
git commit -m "Feat: viz launch host arg for laptop fallback"
```

---

### Task 21: README + pre-defense smoke checklist

**Files:**
- Create: `ros_version/src/hermes_viz/README.md`

- [ ] **Step 1: Write README**

```markdown
# hermes_viz

Browser-based 3D digital twin for H.E.R.M.E.S. Read-only — subscribes to existing topics; publishes nothing.

## Run

```bash
ros2 launch hermes_viz viz.launch.py
```

Open http://localhost:8012 if not auto-launched.

## Camera presets

- `1` — front-elevated (default, audience POV)
- `2` — top-down tactical
- `3` — cinematic orbit
- `R` — reset to `1`

## Replay

```bash
ros2 bag play <bag>
```
The viz subscribes to the same topics; no extra config needed.

## Pre-defense smoke checklist

- [ ] `colcon build` clean.
- [ ] `pytest src/hermes_viz/test/` all green.
- [ ] Launch viz with wearables stack live; verify hands track operator, all 4 bots track OptiTrack, deadman toggles correctly.
- [ ] Cycle camera presets 1/2/3/R.
- [ ] Verify HUD updates (latency, packet rate, gesture, motors).
- [ ] Kill the bridge mid-run — confirm wearables stack continues unaffected.
- [ ] Run on the actual projector ≥ 48 h before defense; confirm colors/contrast.

## Phase 2 backlog

- Per-joint finger articulation from individual flex channels.
- Photo-textured walls.
- OptiTrack camera meshes in the scene.
- In-viz bag scrubbing UI.
- `r5`/`r6` once they're live.
```

- [ ] **Step 2: Commit**

```bash
git add ros_version/src/hermes_viz/README.md
git commit -m "Docs: hermes_viz README + pre-defense checklist"
```

---

### Task 22: Final cross-check + push

- [ ] **Step 1: Full test pass**

```bash
cd ros_version
colcon build --symlink-install --packages-select hermes_viz
source install/setup.bash
python -m pytest src/hermes_viz/test/ -v
```
Expected: all green.

- [ ] **Step 2: Run the read-only invariant test (manual)**

Start the bridge, then:
```bash
ros2 topic list | grep hermes_viz   # should print nothing — bridge publishes no topics
```

- [ ] **Step 3: Tag the MVP**

```bash
git tag -a viz-mvp -m "H.E.R.M.E.S 3D viz MVP — digital twin for thesis defense"
```

- [ ] **Step 4: Push (only if user explicitly asks)**

```bash
git push origin main
git push origin viz-mvp
```

---

## Cross-task notes

### When to pause and ask the user

- **Task 3**: sample `/hermes/raw_input` JSON.
- **Task 9**: canonical ROSbot URDF path + mesh dependencies.
- **Task 10**: lab room dimensions + OptiTrack volume dimensions + 3–5 lab photos (or skip photos and use gray walls).
- **Task 13**: confirm OptiTrack frame convention.

Implementer should not invent these values. Halt and ask.

### Tests-only debug loop

Phase 1 (Tasks 3–8) is pure Python, no Vuer, no ROS. The implementer can iterate fast on a laptop without a Pi or a ROS workspace:

```bash
cd ros_version/src/hermes_viz
python -m pytest test/ -v
```

### Vuer API verification

The exact Vuer API surface (`session.set @`, `session.upsert @`, `Html(src=...)`) is documented in the `docs.vuer.ai` skill. The implementer should invoke that skill once before Tasks 11–17 to confirm signatures match the installed Vuer version. If the API drifts, adjust call sites — but transforms, HUD state, trail, and watchdog logic are pure and unaffected.

### What is intentionally NOT in this plan

- Per-joint finger articulation (Phase 2).
- In-viz bag scrubbing UI (Phase 2 — `ros2 bag play` is the MVP replay path).
- Photoreal lab textures (Phase 2).
- Driving simulated robots when wearables are offline (the viz is a read-only digital twin, not a simulator).
- Modifications to any existing `hermes_control` node or topic (read-only invariant).
