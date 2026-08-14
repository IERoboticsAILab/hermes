# H.E.R.M.E.S → Aerial Swarm (H.E.R.M.E.S-A)

**Adapting the wearable-to-swarm control system from ROSbots to a multirotor swarm.**

Status: design document + implementation plan. Nothing built yet.
Written against the code as of `main` @ `dfa0747`.

---

## 0. Scope, and what I assumed

You asked for everything: commands, gestures, how it works, what changes. This document covers
the full system — hardware, transport, coordinate frames, JSON schemas, control law, safety,
the complete gesture/command table, haptics, visualization, tests, regulation, and a phased plan.

Two things genuinely fork the design, so I picked defaults and marked them clearly rather than
stopping to ask:

| Fork | Assumed default | Where it matters |
|---|---|---|
| Platform | **Crazyflie 2.1+ / Crazyswarm2, indoors, OptiTrack pose** | §3, §6.4, §17 |
| Venue | **Indoor lab (your existing OptiTrack volume)** | §17 regulatory load ≈ 0 |

The outdoor / PX4 / GPS track is specified as a parallel path in §3.3 and §6.5 — everything from
§4 through §11 is platform-independent and applies to both. §19 lists the decisions I want from you.

---

## 1. Executive summary

### 1.1 The good news: half the system doesn't change at all

The entire wearable and gesture-recognition half is **platform-agnostic** and needs no changes to
work with drones:

- Both glove firmwares — unchanged.
- Vest ESP32 firmware — unchanged for transport (one optional change for PWM haptics, §11.4).
- ESP-NOW transport, USB serial framing, `hermes.hub.v1` envelope — unchanged.
- `vest_serial_bridge_node` — unchanged.
- `gestures/fsr_tracker.py`, `imu_filter.py`, `posture_classifier.py`, `recognizer.py`, `matcher.py` — unchanged (the *registry data* changes, not the matching engine).
- The mode/posture/FSR interaction model — unchanged in structure.

That is roughly 2,000 lines of working, tuned, hardware-validated code that carries over untouched.
The adaptation is concentrated in three places: **the target geometry (2D→3D)**, **the per-robot
agent (ground kinematics → flight stack)**, and **safety (stop → hover)**.

### 1.2 What changes, ranked by risk

| # | Change | Why it's mandatory | Risk if skipped |
|---|---|---|---|
| 1 | `_publish_stop()` → `hover` | A drone that stops receiving setpoints falls or drops out of OFFBOARD | Crash |
| 2 | ESTOP redefined from "kill" to "controlled descent" | Gesture-triggered motor cut = falling brick | Crash, injury |
| 3 | Geofence (box + floor + ceiling) | Walls bound a ROSbot; nothing bounds a drone | Flyaway |
| 4 | Staged link-loss failsafe (hover→RTL→land) | Current behaviour is "publish zero Twist forever" | Crash on battery exhaustion |
| 5 | Battery in the status path + haptics | ROSbots run for hours; a Crazyflie flies ~7 min | Crash |
| 6 | 3D targets `(x,y,yaw)` → `(x,y,z,yaw)` | Altitude is the whole point | No aerial capability |
| 7 | Downwash separation constraint | 2D collision avoidance permits stacking, which is fatal for rotorcraft | Mid-air collision |
| 8 | Commanded formation altitude (not centroid-derived) | Live-centroid feedback makes altitude sag (§13.6) | Slow uncommanded descent |
| 9 | Takeoff/land/arm lifecycle + gestures | ROSbots have no arm state | Cannot fly |
| 10 | Behavior phase clock | Current behaviors emit static targets; ORBIT/PATROL need time | Behaviors are decorative |

### 1.3 Effort shape

| Layer | New | Modified | Untouched |
|---|---|---|---|
| Glove/vest firmware | 0 | 0 (1 optional) | all |
| Gesture pipeline | 0 | `registry.py` (data), `posture_classifier.py` (+1 posture) | matcher, tracker, filters |
| Swarm logic | 1 file (`geofence.py`) | `formation_engine`, `behavior_engine`, `swarm_controller` | — |
| Per-robot agent | 1 node (`drone_agent_node.py`) | — | `decentralized_robot_agent_node.py` stays for ROSbots |
| Pose/status | — | `optitrack_pose_beacon_node`, `robot_haptic_status_node` | — |
| Haptics | — | `haptic_vest_node.py` | vest firmware |
| Adapter | 1 node (`flight_stack_adapter_node.py`) | — | — |

Total new/changed Python: ~1,400 lines. No new dependency beyond the flight stack itself.

---

## 2. Architecture: current vs aerial

### 2.1 Current (ground)

```
Left glove  (4 flex + MPU6050) ─┐
                                ├─ ESP-NOW ─► Vest ESP32 ─ USB 921600 ─► Raspberry Pi 5
Right glove (4 FSR)           ─┘                  ▲                          │
                                                  └── haptic frames V1,seq,m1..m6
Pi:  vest_serial_bridge ─► gesture_pipeline ─► swarm_control ─► haptic_vest
                             /hermes/raw_input  /hermes/command_packets  /hermes/swarm_intent

Per ROSbot: optitrack_pose_beacon ─► /hermes/robot_state_beacon
            decentralized_robot_agent ─► /cmd_vel   (+ /hermes/slot_bids auction)
            robot_haptic_status ─► /hermes/robot_haptic_status
```

### 2.2 Aerial

```
   ── wearable half: IDENTICAL ──

Pi:  vest_serial_bridge ─► gesture_pipeline ─► swarm_control ─► haptic_vest
                                                     │
                                          /hermes/swarm_intent  (now carries z, alt cmd, phase clock)
                                                     │
Per drone (onboard Pi Zero / companion, or all on the ground Pi for Crazyflie):
     ┌── drone_agent_node ──────────────────────────────────┐
     │   3D slot auction  →  4-DOF target  →  geofence      │
     │   hover-on-any-fault  →  failsafe ladder             │
     └──────────────┬───────────────────────────────────────┘
                    ▼
        flight_stack_adapter_node   (Crazyswarm2 | MAVROS | uXRCE-DDS)
                    ▼
              flight controller
                    │
     pose ◄── OptiTrack / Lighthouse / RTK-GPS
     status ─► /hermes/robot_haptic_status  (+battery, +armed, +AGL, +up/down ranges)
```

Key structural point: **`drone_agent_node` never talks to a flight controller directly.** All
vendor specificity lives in `flight_stack_adapter_node`. That keeps the swarm logic testable
offline and lets you swap Crazyflie→PX4 without touching the swarm layer.

---

## 3. Platform selection

### 3.1 Recommended: Crazyflie 2.1+ with Crazyswarm2 (indoor)

Why this is the right first platform for H.E.R.M.E.S:

- **Your OptiTrack volume already works.** Crazyswarm2 ingests external pose via
  `motion_capture_tracking` and supports OptiTrack natively. You are reusing the exact lab
  infrastructure that `optitrack_r1..r4.yaml` already describes.
- **Failure is cheap and safe.** 27 g airframe. A crash is a bent prop, not a hospital visit.
  This is what makes the six-week iteration loop possible.
- **Six units is a normal Crazyflie swarm**, not a stretch. Matches your existing `r1..r6` IDs.
- **No regulation indoors** (§17).
- **Cost.** ~€1,500 for six aircraft + radios + decks, vs €8,000+ for six PX4 quads.

Constraints you must design around:

| Parameter | Value | Consequence |
|---|---|---|
| Flight time | 5–7 min (Crazyflie 2.1+, stock LiPo) | Demos must be <4 min; battery haptics mandatory |
| Payload | ~15 g | No lidar. Multi-ranger + Flow deck only |
| Prop-to-prop | 92 mm | Min horizontal separation ~0.30 m |
| Downwash | strong, narrow column | Min **vertical** separation ~0.50–0.60 m; never hover directly under another |
| Radio | Crazyradio 2 / PA | ~10–15 aircraft per radio at 100 Hz; 6 is comfortable on one |
| Pose rate | OptiTrack 100–240 Hz | Far above the 30 Hz intent rate — no bottleneck |

Suggested BOM (6 aircraft):

| Item | Qty | Note |
|---|---|---|
| Crazyflie 2.1+ | 6 | |
| Crazyradio 2 | 1–2 | 2nd for redundancy/bandwidth headroom |
| Flow deck v2 | 6 | z-ranger, gives AGL + optical flow fallback |
| Multi-ranger deck | 2–6 | front/back/left/right/up ToF → obstacle haptics. 2 minimum for the demo |
| Marker sets / active markers | 6 | OptiTrack rigid bodies, one per aircraft |
| Spare props, motors, batteries | plenty | assume 3 crashes/week early on |
| Charging deck or multi-charger | 1 | 7-min flights mean charge cycles dominate lab time |

### 3.2 Verify before committing

Crazyswarm2 API surface (topic/service names, `cmd_position` vs `cmd_full_state` vs the
high-level `go_to`/`takeoff`/`land` services) has changed across releases. **Read the docs for
the exact version you install** and pin it. Do not take the names in §6.4 as gospel.

### 3.3 Alternative: PX4 or ArduPilot multirotor (outdoor / larger indoor)

Take this path only if you need payload (real camera, lidar) or outdoor operation.

| | Crazyflie | PX4 / ArduPilot quad |
|---|---|---|
| Pose source | OptiTrack / Lighthouse | RTK GPS (outdoor) or OptiTrack (indoor) |
| Interface | Crazyswarm2 ROS 2 | MAVROS, or uXRCE-DDS (`px4_ros2`) |
| Companion compute | none needed (ground Pi drives all) | Pi Zero 2 / Orange Pi per aircraft |
| Flight time | 7 min | 15–25 min |
| Regulatory | none indoors | Part 107 / EASA + swarm waiver (§17) |
| Cost/unit | ~€250 | ~€900–1,500 |
| Crash cost | €20 | €400 + liability |
| Frames | ENU throughout | **NED internally** — sign traps (§5.4) |

Recommendation: build on Crazyflie, keep the adapter boundary clean, port to PX4 later if the
application demands it. The swarm logic is identical; only the adapter changes.

### 3.4 Do not skip: simulation first

A ROSbot bug costs a bumped wall. A drone bug costs an aircraft and possibly a person. **No
flight until the logic passes in simulation.** Two tiers:

1. **Logic sim (cheap, immediate).** The `feat-hermes-3d-viz` branch already has
   `hermes_viz/simulator.py` that fabricates robot beacons. Extend it to 3D and you can exercise
   the entire gesture→intent→agent→target loop with zero hardware. This catches ~80% of bugs.
2. **Physics sim (before first flight).** Crazyflie: the Crazyswarm2 simulation backend.
   PX4: Gazebo multi-vehicle SITL. Validates the adapter, timing, and OFFBOARD/setpoint behaviour.

---

## 4. What the code does today (baseline, verified by reading)

Establishing this precisely, because every change below is stated as a delta from it.

**Gesture pipeline.** `vest_serial_bridge_node` polls serial at 60 Hz, requires the **left glove
fresh within 200 ms** or it emits an empty fail-safe sample. `gesture_pipeline_node` runs
`SafetyEvaluator.tick()` then `match_gesture()` per sample, publishing at most one safety packet
and one command packet per raw sample to `/hermes/command_packets`.

**Left posture selects mode** (`_mode_from_left_posture`): OPEN→DRIVE, POINT→SELECTION,
FIST→FORMATION, TWO→BEHAVIOR, THREE→PARAMS. Posture classification is stateful with per-finger
thresholds (0.66–0.76) and ±0.02 hysteresis.

**Right glove is FSR-only.** 4 sensors × {TAP, DOUBLE_TAP, HOLD} = 12 discrete slots per mode,
plus 2-event sequences via `R_fsr_sequence`. `PRESS`/`RELEASE` exist and `RELEASE` drives the
`set_while_held` off-edge.

**Left IMU is the only analog stick.** `PITCH→vx`, `ROLL→vy_or_steer`, `YAW→omega`, scaled by
`speed_level` {0.35, 0.55, 0.75, 1.00} and `aggression_level` {0.75, 1.00, 1.20, 1.40},
× 0.35 if `precision_drive`.

**Safety is two mechanisms.** (a) `DEADMAN_IMU`: left-hand palm-up (normalized `AZ · sign ≥ 0.80`,
120 ms debounce, 0.10 g hysteresis) **gates motion off**. Motion is allowed when the palm is *not*
up. (b) `ESTOP`: left-glove dynamic accel `|‖a‖−1| ≥ 0.75 g` sustained 220 ms → `emergency_stop`,
which calls `SwarmController._stop_all()` and clears everything.

**`swarm_control_node`** folds packets into a `SwarmController` + `GestureState` and republishes
the whole world as `/hermes/swarm_intent` at 30 Hz (`hermes.swarm_intent.v1`).

**`decentralized_robot_agent_node`** (one per ROSbot, 20 Hz):
1. Stop if intent stale >600 ms, `deadman_active` false, `paused`, or self not in selection.
2. If `mode == DRIVE` and `drive_cmd_vel` present → publish it directly as `Twist`.
3. Else compute slot targets locally via `execute_behavior` / `compute_formation_targets` over
   **`slot_00N` ids** (not robot ids) centered on the **live centroid computed from fresh beacons**.
4. Bid on slots (`hermes.slot_bids.v1`, cost = distance + 0.12 × heading error), greedily resolve
   a complete bid set into an assignment, hold the assignment for ≥1000 ms.
5. Drive to the assigned slot with a nonholonomic law: `speed · cos(heading_err)` forward,
   `kp_angular · heading_err` turn, through a 2D sampled-velocity collision-avoidance pass
   (63 candidate velocities on a circle, 2 s time horizon, 0.22 m radius + 0.10 m margin).
6. Deadlock recovery: if goal distance fails to shrink by 0.03 m over 1800 ms → spin in place 1200 ms.

**`haptic_vest_node`** (20 Hz) maps 6 motors → 6 robots via `motor_robot_ids`, resolves one event
per motor by a fixed priority cascade, and emits `V1,<seq>,<m1..m6>\n`. Vest firmware treats each
level as **binary** (`level > 0` → `digitalWrite HIGH`) with a 300 ms failsafe.

**Nothing publishes `/hermes/centroid`.** It is always `(0,0)`; agents always use their own
beacon-derived centroid. This matters in §13.6.

---

## 5. Coordinate frames and schema changes

### 5.1 Principle: additive, not breaking

Every schema change below **adds** fields. Existing ground-robot code keeps working, the tests
keep passing, and a mixed fleet (ROSbots + drones) is possible. Do not bump schema versions where
an added optional field will do. Where a version bump is unavoidable, consumers accept both.

### 5.2 `hermes.robot_state_beacon.v1` — add altitude

```diff
 {
   "schema": "hermes.robot_state_beacon.v1",
   "stamp_ms": 1234567,
   "robot_id": "r1",
   "rigid_body_name": "cf1",
   "frame_id": "optitrack",
   "x": 1.20, "y": -0.35, "yaw": 0.78,
   "vx": 0.10, "vy": 0.02,
+  "z": 1.10,          // metres, up-positive, origin = volume floor
+  "vz": -0.05,
+  "agl_m": 1.08,      // from down-facing ranger; null if unavailable
+  "armed": true,
+  "flight_state": "FLYING"   // DISARMED|ARMED|TAKING_OFF|FLYING|LANDING|LANDED
 }
```

`optitrack_pose_beacon_node` change: add a `vertical_axis` parameter (`"z"` default, accepts
`-z` etc. like the existing `planar_x_axis`/`planar_y_axis`), extract `z` with the existing
`_component_from_xyz`, and finite-difference `vz` alongside the existing `vx`/`vy`. ~15 lines.

Keep `yaw` as the planar heading — for a multirotor it is the camera/nose direction, and the
existing `_planar_heading_from_quaternion` computes exactly the right thing.

### 5.3 `hermes.swarm_intent.v1` — add the aerial command set

```diff
 {
   "schema": "hermes.swarm_intent.v1",
   "seq": 8123, "stamp_ms": …,
   "mode": "PILOT",
   "deadman_active": true,
   "paused": false,
   "selection": ["r1","r2","r3"],
   "robot_ids": ["r1",…,"r6"],
   "centroid": {"x":0.0,"y":0.0},
   "active_formation_type": "STACK",
   "formation_heading": 0.78,
   "formation_spacing": 1.00,
   "active_behavior": "ORBIT_TARGET",
   "behavior_params": {…},
   "home_xy": {"x":0.0,"y":0.0},
   "path_waypoints": [],
   "drive_cmd_vel": {"vx":0.2,"vy":0.0,"omega":0.1},
   "groups": {…},
+  "centroid": {"x":0.0,"y":0.0,"z":1.20},        // z added to the existing object
+  "formation_altitude_m": 1.20,                   // COMMANDED, never derived (§13.6)
+  "formation_layer_gap_m": 0.60,                  // vertical spacing for STACK/WALL/DOME
+  "drive_cmd_vel": {"vx":0.2,"vy":0.0,"vz":0.0,"omega":0.1},   // vz added
+  "flight_command": null,                         // ARM|DISARM|TAKEOFF|LAND|RTL|EMERGENCY_DESCEND
+  "flight_command_seq": 41,                       // monotonic; agents act on change only
+  "behavior_phase_s": 12.480,                     // §10.2 phase clock
+  "geofence": {                                   // §7.4
+    "min_x":-3.0,"max_x":3.0,
+    "min_y":-2.5,"max_y":2.5,
+    "min_z":0.30,"max_z":2.20
+  },
+  "home_xyz": {"x":0.0,"y":0.0,"z":1.00}          // supersedes home_xy for drones
 }
```

`flight_command` is deliberately **latched state + sequence number**, not an event. The intent is a
periodic full-state snapshot at 30 Hz; a bare event field would be re-executed 30×/s. Agents
compare `flight_command_seq` against the last one they acted on.

### 5.4 Frame conventions — write this on the wall

| Layer | Convention |
|---|---|
| H.E.R.M.E.S internal | **ENU-like**: x forward/east, y left/north, **z up**, yaw CCW from +x |
| OptiTrack raw | Y-up by default → the `planar_*`/`vertical_axis` params already remap this |
| Crazyflie / Crazyswarm2 | ENU, z up — matches directly |
| MAVROS | ENU (it converts for you) — matches directly |
| PX4 uXRCE-DDS raw (`/fmu/in/*`) | **NED, z DOWN** — must negate z and swap x/y |
| ArduPilot via MAVROS | ENU |

If you go the raw-uXRCE route, **a missed sign on z commands full-throttle descent.** Put the
conversion in exactly one function in the adapter, unit-test it, and never inline it.

### 5.5 `hermes.robot_haptic_status.v1` — add flight health

```diff
 {
   "schema": "hermes.robot_haptic_status.v1",
   "robot_id": "r1",
   "obstacle": false, "obstacle_level": 0.0,
   "front_scan_min_m": null,
   "front_range_min_m": 0.8, "rear_range_min_m": null,
   "error": false, "error_flags": {…},
+  "left_range_min_m": 1.2, "right_range_min_m": 0.9,
+  "up_range_min_m": 0.7, "down_range_min_m": 1.05,
+  "battery_pct": 62.0,
+  "battery_v": 3.86,
+  "flight_time_remaining_s": 190,
+  "battery_state": "OK",          // OK|LOW|CRITICAL
+  "armed": true,
+  "flight_state": "FLYING",
+  "geofence_margin_m": 0.42,      // distance to nearest fence face; negative = breached
+  "pose_source": "mocap"          // mocap|flow|gps|dead_reckoning
 }
```

`robot_haptic_status_node` already supports arbitrary `front_range_topics` / `rear_range_topics`
lists with staleness handling — add `left_`/`right_`/`up_`/`down_` lists using the same
`_make_range_cb` + `_fresh_min` machinery (~40 lines), plus a `sensor_msgs/BatteryState`
subscription. The `_risk_from_distance` hit/clear ramp works unchanged for the new axes; give
`up`/`down` tighter thresholds because vertical clearance is scarcer.

### 5.6 `hermes.slot_bids.v1` — 3D cost

No schema change. Only the cost function moves (§6.3).

---

## 6. Control: from ground kinematics to flight

### 6.1 The kinematic difference

| | ROSbot | Multirotor |
|---|---|---|
| Translational DOF | 2 (x,y), often nonholonomic | 3 (x,y,z), holonomic |
| Heading coupling | must face where it drives | **decoupled** — yaw is free |
| Zero command | sits still, safe | must be an active hover |
| Vertical | n/a | gravity-loaded, always fighting |
| Command form | `Twist` (body frame) | position or velocity setpoint (world frame) + yaw |

The decoupling is a **simplification**. This whole block in `_publish_target_tracking` disappears:

```python
# ground: rotate toward the goal, then drive forward along the nose
heading_err = _wrap_to_pi(desired_heading - yaw)
self._publish_cmd(safe_speed * max(0.0, math.cos(heading_err)), 0.0,
                  self._kp_angular * heading_err)
```

For a drone the world-frame velocity is commanded directly and yaw is a separate, independent
channel. Also gone: the `heading_slowdown_rad` penalty and the `yaw_mode` steer/rotate-in-place
distinction (§8.6 repurposes that gesture slot).

### 6.2 New node: `drone_agent_node.py`

Fork rather than parameterize `decentralized_robot_agent_node.py`. Reasons: the control law
differs (holonomic 4-DOF vs nonholonomic 3-DOF), the fault semantics differ (hover vs stop), and
there is a lifecycle state machine that has no ground analogue. Bolting all of that into the
working ground agent behind `if self._is_drone:` branches would make both paths harder to reason
about, and the ground path currently flies ROSbots in your lab — do not destabilize it.

What *is* shared and must stay shared: `formation_engine`, `behavior_engine`, the slot-auction
protocol, the beacon/intent schemas. Extend those in place; both agents consume them.

Tick structure (20–30 Hz), fault-first:

```
_tick():
  ── ALWAYS publish a setpoint before returning. Every path. No exceptions. ──
  1. lifecycle gate    : DISARMED/LANDED  → publish disarmed-idle, return
  2. intent staleness  : >600 ms          → failsafe ladder (§7.3), return
  3. deadman / paused                     → HOVER at frozen target, return
  4. battery           : CRITICAL         → LAND now, ignore intent, return
                         LOW              → continue, flag haptics
  5. flight_command    : seq changed      → run lifecycle transition, return
  6. selection         : self not in it   → HOVER, drop slot assignment, return
  7. mode == PILOT and drive_cmd_vel      → velocity setpoint (+vz), clamp, geofence, publish
  8. beacon staleness  : >600 ms          → HOVER (no pose ⇒ no formation), return
  9. compute 3D slot targets → bid → resolve assignment → own target
 10. geofence clamp (§7.4)
 11. downwash deconfliction (§7.5)
 12. 3D collision avoidance
 13. publish position-or-velocity setpoint
```

Step 1's "always publish" is not stylistic. PX4 OFFBOARD drops out after ~0.5 s of setpoint
silence; the current ground agent has multiple `return` paths that publish nothing on some ticks
(e.g. after `_maybe_deadlock_recovery` returns True it publishes, but the early `return` in the
`dist <= pos_tol` branch publishes then returns — fine — while a stale-intent path publishes a
zero `Twist`, which for a drone means "descend"). Audit every return path.

### 6.3 Slot auction in 3D

Keep the protocol byte-for-byte. Change the cost:

```python
# ground
costs[sid] = dist_2d + 0.12 * heading_err

# drone: climbing is slower and costs more energy than lateral travel;
# crossing altitudes risks flying through someone's downwash.
dz = tz - z
costs[sid] = (math.hypot(dx, dy)
              + W_CLIMB * max(0.0, dz)      # up is expensive  (W_CLIMB ≈ 2.0)
              + W_DESCEND * max(0.0, -dz)   # down is cheap-ish (W_DESCEND ≈ 1.2)
              + 0.05 * heading_err)          # yaw is nearly free — de-weight it
```

The asymmetric climb/descend weights and the large `W_CLIMB` are what make the auction
*naturally* prefer assignments that keep each aircraft near its current layer, which is exactly
the anti-downwash behaviour you want. This is much cheaper than solving deconfliction in the
avoidance layer.

Keep `assignment_lock_ms` (1000 ms). Consider raising it to 1500 ms for drones — slot churn
mid-flight is more expensive than mid-drive.

### 6.4 Adapter: Crazyswarm2

`flight_stack_adapter_node.py`, one instance per aircraft (or one multiplexing node). It
subscribes to an internal `/hermes/<id>/setpoint` and calls out. Verify names against your
installed version (§3.2):

| H.E.R.M.E.S intent | Crazyswarm2 |
|---|---|
| takeoff | `/cf<N>/takeoff` service (height, duration) |
| land | `/cf<N>/land` service |
| position setpoint | `cmd_position` (x,y,z,yaw) — the workhorse for formations |
| velocity + yaw rate | `cmd_vel_legacy` / `cmd_full_state` — for PILOT mode |
| go to waypoint (smoothed) | `/cf<N>/go_to` service — good for RTL and takeoff staging |
| emergency | `/cf<N>/emergency` — **note: this is a motor cut. Never bind it to a gesture** (§7.2) |
| pose in | `motion_capture_tracking` from OptiTrack |
| battery/state out | `/cf<N>/status` (or the logging framework) → republish as haptic status |

Practical note: for Crazyflie you do **not** need a companion computer. All six `drone_agent_node`
instances plus adapters run on the ground-side machine alongside the wearable stack. That is a
significant simplification vs the current ROSbot topology, where each robot runs its own agent.
The slot auction still works — it just runs between processes on one host.

### 6.5 Adapter: PX4 / ArduPilot

| H.E.R.M.E.S intent | MAVROS | uXRCE-DDS (`px4_ros2`) |
|---|---|---|
| arm/disarm | `/mavros/cmd/arming` | `VehicleCommand` |
| offboard | `/mavros/set_mode` → `OFFBOARD` | `OffboardControlMode` @ ≥2 Hz |
| position setpoint | `/mavros/setpoint_raw/local` (`PositionTarget`) | `/fmu/in/trajectory_setpoint` |
| velocity setpoint | same, `type_mask` selects vel | same |
| takeoff/land | `/mavros/cmd/takeoff`, `/cmd/land` | `VehicleCommand` |
| pose in | `/mavros/local_position/pose` | `/fmu/out/vehicle_local_position` |
| battery | `/mavros/battery` | `/fmu/out/battery_status` |

Two hard rules for PX4:
1. **The `OffboardControlMode` heartbeat and setpoints must never stop.** Your 20–30 Hz tick is
   comfortable, but every `return` path must publish (§6.2).
2. **NED.** See §5.4.

### 6.6 Gains, retuned

| Param | Ground default | Drone starting point | Note |
|---|---|---|---|
| `control_hz` | 20 | 30 | matches the 30 Hz intent rate |
| `kp_linear` | 0.8 | 1.0 (Crazyflie) | onboard controller does the real work |
| `kp_angular` | 1.8 | 1.2 | yaw is decoupled; slower is calmer for cameras |
| `max_linear_speed` | 0.8 m/s | 0.8 m/s indoor, 0.4 for first flights | |
| `max_vz_up` | — | 0.5 m/s | new |
| `max_vz_down` | — | 0.35 m/s | **asymmetric**: descend slower than you climb — a fast descent enters its own downwash and loses lift |
| `position_tolerance_m` | 0.08 | 0.12 | mocap-good, but drones never sit still |
| `altitude_tolerance_m` | — | 0.08 | |
| `robot_radius_m` | 0.22 | 0.15 (CF) | |
| `safety_margin_m` | 0.10 | 0.20 | |
| `vertical_separation_m` | — | 0.60 | downwash (§7.5) |

Leave every one of these as a ROS parameter. The real numbers come off the aircraft, not off this
page — a real prop is not the prop in the datasheet.

---

## 7. Safety architecture

This is the section that matters. A ROSbot failure is embarrassing; a drone failure is dangerous.

### 7.1 The central semantic inversion: stop → hover

Every "stop" in the ground code means *cease motion, stay put safely*. For a multirotor, ceasing
motion means falling. Enumerate and replace:

| Ground site | Ground meaning | Drone replacement |
|---|---|---|
| `_publish_stop()` | zero `Twist` | **`_publish_hover()`**: position setpoint = pose captured at hover entry, current yaw held |
| `SwarmController._stop_all()` | clear all state | clear *task* state (behavior/formation/selection edits), **preserve flight state and altitude** |
| `emergency_stop` effect | full reset, deadman off | `EMERGENCY_DESCEND` (§7.2) |
| stale intent | zero `Twist` | failsafe ladder (§7.3) |
| `deadman_active == false` | zero `Twist` | hover, hold altitude |
| stale beacons | zero `Twist` | hover on last-known + onboard estimator |

`_publish_hover()` must **latch the target on entry**, not recompute from the live pose every tick.
Recomputing from live pose is a positive feedback loop: the aircraft drifts, the setpoint follows
the drift, and it walks away. Latch once, hold, and require an explicit re-engage to release.

### 7.2 ESTOP must not be a kill switch

Today: sustained left-glove shake ≥0.75 g for 220 ms → `emergency_stop` → everything cleared.

**Do not map any gesture to motor cutoff.** Motor cut on an airborne multirotor is a falling
object. Two changes:

1. **Redefine the gesture.** `ESTOP` becomes `EMERGENCY_DESCEND`: all aircraft (ignoring selection)
   spread horizontally by a small deconfliction offset, descend at `max_vz_down`, land, auto-disarm
   on ground contact. Non-interruptible except by an explicit re-arm sequence.
2. **Raise the trigger bar.** 0.75 g / 220 ms is *easily* reached by an operator walking briskly,
   pointing, or gesturing emphatically — and it currently fires while the ground robots are
   driving. For aerial use:

```diff
 "ESTOP": {
   "gesture": {"L_ACCEL_SHAKE": {
-    "threshold_g": 0.75,
-    "release_threshold_g": 0.35,
-    "hold_ms": 220
+    "threshold_g": 1.10,          // deliberate shake, not an emphatic gesture
+    "release_threshold_g": 0.40,
+    "hold_ms": 500,               // ~2-3 full shake cycles
+    "require_L_posture": "FIST"   // NEW: co-occurring posture gate
   }},
-  "effect": {"type": "emergency_stop"},
+  "effect": {"type": "emergency_descend"},
   "priority": 1000
 }
```

`require_L_posture` needs ~4 lines in `SafetyEvaluator._tick_shake_estop` (compare
`event.L_posture`). Making it a *conjunction* of an unusual posture and a deliberate shake is what
takes the false-positive rate to near zero. **Calibrate the threshold against a logged recording
of your own arm during a full demo run** before trusting it.

3. **A real kill switch must exist, in hardware, outside this stack.** A physical switch or an RC
   transmitter bound to each aircraft, held by a spotter who is not the gesture operator. On
   Crazyflie the `/cf<N>/emergency` service is the software equivalent — bind it to a keyboard key
   on the ground station, never to a gesture. This is non-negotiable for any flight with people
   in the room.

### 7.3 Staged link-loss failsafe

The ground agent's response to a stale intent is one branch: publish zero. A drone needs a ladder,
and it needs one at *every* link in the chain, because each link fails differently.

| Link | Detection | 0–0.6 s | 0.6–3 s | 3–10 s | >10 s |
|---|---|---|---|---|---|
| Glove → vest (ESP-NOW) | `vest_serial_bridge` glove_timeout_ms=200 → empty sample → deadman off | hover | hover | hover | hover (operator present, aircraft safe) |
| Vest → Pi (USB) | no lines read | hover | hover | RTL | land |
| Pi → agent (intent) | `stop_on_missing_intent_ms` | continue | **hover** | **RTL** | **land in place** |
| Agent → aircraft (radio) | adapter link watchdog | onboard hover | onboard hover | **onboard RTL** (firmware-level) | **onboard land** |
| Pose (mocap) | beacon staleness | hover on estimator | hover | **descend slowly** | land |

Two design rules that fall out of this table:

- **Glove loss ≠ aircraft loss.** If the operator's glove dies, the operator is still standing
  there watching. Hover indefinitely is correct — do not auto-land, that removes the human's
  ability to intervene. If the *Pi* dies, nobody is in the loop: RTL then land.
- **Recovery requires a deliberate re-engage.** After any failsafe hover, a single fresh packet
  must **not** silently restore control — the aircraft would lurch to whatever target the
  intent happens to carry. Add a `_requires_reengage` latch cleared only by an explicit gesture
  (§8.4, `RESUME`). Log and haptically announce the latch.

Also note the current fail-safe hole: `vest_serial_bridge_node._publish_raw_input` emits a sample
with **empty `flex`/`imu`** when the left glove is stale. Downstream, `SafetyEvaluator` sees
`event.accel_L is None` → `_set_gate(False)` → deadman off → drones hover. That chain is correct
and already works. Verify it end-to-end on the bench (pull the left glove's battery mid-hover)
before trusting it in air.

### 7.4 Geofence — new component

Nothing in the current system bounds the workspace; walls bound ROSbots. New file
`swarm/geofence.py`, pure function, no ROS:

```python
def clamp_target(x, y, z, fence, margin=0.15):
    """Clamp a target into the fence, inset by `margin`. Returns (x,y,z,clamped:bool)."""

def breach_severity(x, y, z, fence):
    """0.0 inside with margin → 1.0 outside. Drives haptics and the descend trigger."""
```

Applied in three places:
1. `drone_agent_node` — clamp every target every tick, before publishing. Cheap insurance.
2. `swarm_controller` / `formation_engine` — clamp *formation* targets at generation, so a huge
   spacing value doesn't ask for a wall. Better than clamping downstream, because clamping
   individual slots distorts the formation shape; clamping at generation lets the engine shrink
   the whole formation coherently.
3. Breach response — if measured pose is outside the fence (not just the target), command a
   controlled descent immediately. Position error large enough to exit the fence means the
   controller has lost authority; the ground is the safest place.

Fence values live in one config (`config/flight_volume.yaml`) and ride in the intent (§5.3) so
every agent uses the same numbers. Floor must be **above zero** (`min_z ≈ 0.30 m`) — a
zero floor means "land" is indistinguishable from "breach."

### 7.5 Downwash — the constraint 2D avoidance can't see

Rotor wash is a strong, narrow, downward column. An aircraft in another's wash loses lift
abruptly. The current `_sampled_ca_velocity` samples 63 velocities **on a circle** and enforces a
scalar separation `2·radius + margin`. Extend it to 3D naively and you get a sphere-shaped
constraint, which is *wrong*: the hazard region is not a sphere, it's a **cylinder below** each
aircraft, and there is no hazard above.

Two options:

**(a) Layer assignment — recommended, and lazier.** Deconflict by construction: give each
aircraft a distinct altitude layer during any transition, so horizontal conflicts within a layer
are all that avoidance must handle. Falls out almost for free from the auction's `W_CLIMB` weight
(§6.3) plus a `layer_of(slot)` derived from the formation's z offsets. Keep the existing 2D
circle sampling *within* a layer. Small diff, big safety win, and it's what production drone-show
software does.

**(b) Asymmetric 3D constraint — if you need free vertical maneuvering.** Replace the scalar
separation test with:

```python
horizontal = math.hypot(dx, dy)
vertical   = dz                      # positive = the other aircraft is above me
if horizontal < DOWNWASH_RADIUS_M:                   # ≈ 0.35 for a Crazyflie
    violated = (0.0 < vertical < DOWNWASH_HEIGHT_M)  # ≈ 0.60: I'm under them → hazard
else:
    violated = (horizontal < 2 * radius + margin)    # normal side-by-side spacing
```

Note the asymmetry: being *above* another aircraft is fine; being *below* is not. A symmetric
sphere both over-constrains (blocks safe overflight) and under-constrains (permits a 0.4 m
vertical gap that a sphere of radius 0.35 accepts but downwash does not).

Sample candidate velocities on a **cylinder shell** (the existing circle, at three vertical
levels: −vz_max, 0, +vz_max) rather than a full sphere — 3× the candidates instead of ~10×, and
it matches how the aircraft actually moves.

Do (a) first. Add (b) only when a behavior genuinely needs free vertical motion.

### 7.6 Battery as a safety input

Not a nicety. At 7 minutes of endurance, battery *is* the mission clock.

| State | Trigger | System response | Haptic |
|---|---|---|---|
| OK | >30% | normal | none |
| LOW | 15–30% | continue, announce; block new long behaviors | slow double-pulse, that drone's motor |
| CRITICAL | <15% or cell <3.3 V | **that aircraft lands, immediately, ignoring intent** | fast triple-pulse |
| Swarm-degraded | any aircraft CRITICAL | formation recomputes over remaining aircraft | swarm-wide pattern |

Two design points:
- **Per-aircraft, not swarm-wide.** One low battery must not ground the swarm. It leaves, the
  formation reflows — the auction already handles a shrinking selection gracefully, since
  `_fresh_selection` drops stale members and slot targets are recomputed over whoever's left.
- **CRITICAL overrides the operator.** No gesture may veto it. Put the check above the intent
  handling in the tick order (§6.2, step 4).

### 7.7 Preflight gate

Before `ARM` succeeds, all must hold — refuse with a distinct haptic pattern otherwise:

- pose fresh (<200 ms) and inside the geofence
- battery >50% (not >15% — you need margin for the whole flight)
- adapter link up, aircraft reports healthy, no active error flags
- geofence loaded and sane (`min_z > 0`, `max_z > min_z`, non-degenerate box)
- selection non-empty and every selected aircraft passes individually
- `deadman_active` **false** (palm-up) at the moment of arming — you should not be able to arm
  while your hand is in the position that commands motion
- no stale `_requires_reengage` latch outstanding

### 7.8 Change `auto_select_all_on_start` to `false`

`swarm_control_node` currently auto-selects all robots at startup. For ground robots that's
convenient. For drones it means the first stray recognized gesture commands **every aircraft**.
Default it off; require an explicit selection gesture. One-line parameter change, real risk removed.

---

## 8. Gestures and commands — the full design

### 8.1 Input budget, and why it's the binding constraint

What the hardware actually provides today:

| Source | Signal | Slots |
|---|---|---|
| Left flex ×4 | posture | 5 classified today; 16 patterns exist |
| Left MPU6050 | PITCH, ROLL (absolute), YAW (relative), AX/AY/AZ | 3 analog axes + accel events |
| Right FSR ×4 | TAP, DOUBLE_TAP, HOLD | **12 discrete per mode** |
| Right FSR sequences | 2-event chords via `R_fsr_sequence` | overflow capacity, higher effort |
| Right flex | — | **0 — hardware not fitted** |

Ground H.E.R.M.E.S uses 5 modes × 12 = 60 nominal slots and doesn't fill them. The aerial version
needs: a **6th mode** (flight lifecycle), **4 more formations** (3D variants), an **altitude axis**,
**altitude parameters**, and **more guarded safety commands**. The discrete budget gets tight and
the *analog* budget — 3 IMU axes for 4 DOF — is genuinely short.

Three ways out, in increasing cost:

1. **Overload with modifiers** (zero hardware). Hold a finger to remap an IMU axis. Recommended
   for v1, detailed in §8.6.
2. **Add 4 flex sensors to the right glove** (~€10, firmware + wiring only). See §8.2 — this is
   the highest-leverage change available and the Pi-side software already supports it.
3. **Add an MPU6050 to the right glove** (~€3). Gives a genuine second analog stick: dedicated
   altitude axis, two-handed guarded gestures, no overloading. Recommended for v2.

### 8.2 The free upgrade you already paid for

`matcher._match_simple_gesture` accepts `"R_posture"`. `models.GestureEvent` has an `R_posture`
field. `recognizer.recognize` classifies `flex["R"]` whenever it's present.
`vest_serial_bridge_node._publish_raw_input` already populates `raw["flex"]["R"]` from the right
glove packet. **The entire right-hand-posture path is built, wired, and dead** — because
`glove_right.ino` reads FSRs only.

Fit 4 flex sensors to the right glove and you get:

- **25 posture combinations** instead of 5 (left × right), so modes stop competing for postures.
- **Two-handed guarded commands** for free — the ideal safety property for ARM/TAKEOFF/LAND.
- **Zero changes to the Pi stack.** Firmware and wiring only.

Right-glove wiring caution from `CLAUDE.md`: FSR pins must be ADC1 to avoid the ADC2/Wi-Fi
conflict under ESP-NOW. Current pins are 34/35/32/33 (all ADC1) and they are **all four used**.
Adding four flex channels needs four more ADC1 pins — ESP32 ADC1 exposes GPIO 36, 37, 38, 39
(plus the four in use). 36 and 39 are commonly available on DevKit boards; 37/38 often are not
broken out. **Check your specific board before ordering sensors.** If ADC1 is full, the answer
is an external ADC over I2C (ADS1115, ~€4) — which also gives cleaner readings than the ESP32's
noisy internal ADC.

I've specified §8.3–§8.9 for the **current hardware** (single-hand postures), and noted in
§8.10 what changes if you fit the right glove.

### 8.3 Modes — six, needing one new posture

| Left posture (flex pattern) | Registry key | Mode | Change |
|---|---|---|---|
| all EXT | `OPEN` | **PILOT** (was DRIVE) | renamed, +altitude |
| index EXT, rest CURL | `POINT` | **SELECTION** | ~unchanged |
| all CURL | `FIST` | **FORMATION** | +3D formations |
| index+middle EXT, ring+pinky CURL | `TWO` | **BEHAVIOR** | +aerial behaviors |
| index+middle+ring EXT, pinky CURL | `THREE` | **PARAMS** | +altitude level |
| index+middle+ring CURL, pinky EXT | **`PINKY_EXT` (new)** | **FLIGHT** | new: arm/takeoff/land/RTL |

The new posture is defined by its flex pattern, not a name I've invented — you must **bench-verify
it against your actual per-finger thresholds** (`index 0.71, middle 0.76, ring 0.66, pinky 0.71`,
hysteresis 0.02) before relying on it. I chose this pattern because it is the exact complement of
`THREE` and maximally distant from `POINT` (`EXT,CURL,CURL,CURL`), so misclassification between
FLIGHT and any existing mode requires two simultaneous finger errors.

Classifier change is 4 lines in `posture_classifier.py`, after the `THREE` check:

```python
if is_curl["index"] and is_curl["middle"] and is_curl["ring"] and is_ext["pinky"]:
    return "PINKY_EXT"
```

Plus the `POSTURES` and `MODES` entries in `registry.py`. **Apply to both trees**
(`gestures/` and `ros_version/.../gestures/`) per `CLAUDE.md`.

Because FLIGHT mode holds the dangerous commands, add a guard the other modes don't have:
**dwell before FLIGHT mode arms its bindings.** Require the posture held ≥600 ms before any
FLIGHT-mode gesture is accepted, so passing *through* the pattern while transitioning between
other postures can never trigger a takeoff. Implement as a `min_mode_dwell_ms` on the mode
definition, checked in `match_gesture` before dispatching to the domain (~10 lines).

### 8.4 Safety commands (all modes)

| Command | Gesture | Effect | Priority |
|---|---|---|---|
| `EMERGENCY_DESCEND` | left-hand shake ≥1.10 g, 500 ms, **while `FIST`** | all aircraft descend + land + disarm. Uninterruptible. | 1000 |
| `DEADMAN_IMU` | palm-up (`AZ·sign ≥ 0.80`, 120 ms debounce) | **hover**, latched target, all aircraft | 900 |
| `HOVER_HOLD` (was SOFT_STOP) | `OPEN` + R MIDDLE TAP | freeze targets, hold altitude, pause behaviors | 500 |
| `RESUME` | `OPEN` + R MIDDLE DOUBLE_TAP | resume, **and clear `_requires_reengage`** | 500 |
| `GEOFENCE_BREACH` | *automatic* | clamp; descend if pose (not target) is outside | 950 |
| `LINK_LOSS` | *automatic* | staged ladder §7.3 | 950 |
| `BATTERY_CRITICAL` | *automatic, per aircraft* | that aircraft lands | 960 |
| hardware kill | **physical switch / RC, held by a spotter** | motor cut | — |

Note `RESUME` doing double duty as the re-engage clear. That's deliberate: one deliberate,
memorable gesture to get control back from any latched safe state, rather than a different
recovery ritual per fault.

### 8.5 FLIGHT mode — new (posture `PINKY_EXT`, ≥600 ms dwell)

Every command here is a **HOLD with a long minimum**, or a sequence. No bare taps. The matcher
already supports `min_hold_ms` (see `APPLY_FORMATION`, 500 ms) and edge-latches HOLD via
`_hold_latch_token`, so a single hold fires exactly once until release — which is precisely the
semantics you want for "take off."

| Gesture | Command | Effect |
|---|---|---|
| R INDEX HOLD ≥1500 ms | `ARM_SELECTED` | preflight gate (§7.7) then arm. Refuse + distinct haptic on failure |
| R INDEX DOUBLE_TAP | `DISARM_SELECTED` | **only if LANDED.** Silently refused in flight |
| R MIDDLE HOLD ≥1200 ms | `TAKEOFF` | climb to `default_takeoff_alt_m`, staggered 0.8 s per slot, layer-assigned |
| R RING HOLD ≥1200 ms | `LAND_IN_PLACE` | horizontal deconflict, descend, auto-disarm on ground |
| R RING DOUBLE_TAP | `RTL` | go to `home_xyz` at `rtl_altitude_m`, then land |
| R PINKY HOLD ≥1500 ms | `EMERGENCY_DESCEND` | same effect as the shake; a calmer way to invoke it |
| R PINKY TAP | `PREFLIGHT_REPORT` | run checks, report via haptic pass/fail pattern. No state change |
| R MIDDLE DOUBLE_TAP | `SET_HOME_HERE` | home = current swarm centroid. Guarded: only when LANDED |

Staggered takeoff is not cosmetic. Six aircraft climbing simultaneously from a tight launch grid
put each other in mutual downwash. Stagger by slot index and assign distinct target layers.

### 8.6 PILOT mode (posture `OPEN`) — the 4-DOF problem

Requires `deadman_active` (palm **not** up). Base mapping:

| Input | → | Note |
|---|---|---|
| L IMU PITCH | `vx` | absolute (complementary-filtered) — reliable |
| L IMU ROLL | `vy` | absolute — reliable. **True lateral now**, not steer: `yaw_mode` is gone |
| L IMU YAW | `omega` (yaw **rate**) | gyro-integrated, drifts — rate-only is the correct use (§8.7) |
| — | `vz` | **no axis left.** See below |

Three ways to get altitude, pick one:

**(a) Modifier remap — recommended for v1.** Hold R INDEX; while held, PITCH drives `vz`
instead of `vx` (and `vx` is forced to 0). One `set_while_held` modifier plus a branch in
`_resolve_cmd_vel`. Costs one HOLD slot, no hardware. Downside: you cannot climb and translate
forward simultaneously — acceptable for a demo, limiting for cinematography.

**(b) Discrete altitude steps.** `altitude_level` 1–4 in PARAMS mode (§8.9), aircraft climb to
the commanded band. Coarse but very legible, and safest for a first flight. **Do this one first,
regardless** — it's ~10 lines and gives you a safe way to change altitude on day one.

**(c) Right-glove IMU (v2).** Dedicated axis, fully simultaneous 4-DOF. The right answer
long-term (§8.2).

Full PILOT bindings:

| Gesture | Command | Effect |
|---|---|---|
| L IMU (3 axes) | `MANUAL_FLY` | `cmd_vel_stream` with vx, vy, omega |
| R INDEX HOLD | `ALTITUDE_MODIFIER` | remap PITCH→`vz` while held (option (a)) |
| R MIDDLE TAP | `HOVER_HOLD` | safety, cross-mode |
| R MIDDLE DOUBLE_TAP | `RESUME` | safety, cross-mode |
| R RING HOLD | `PRECISION_FLY` | 0.35× all gains (unchanged behaviour, new name) |
| R RING TAP | `FRAME_TOGGLE` | **repurposed from `YAW_MODE_TOGGLE`.** World-frame vs operator-relative control. Genuinely useful for aerial: "forward" meaning *away from me* is far more intuitive than *along the aircraft nose* when the aircraft is 10 m up and facing you |
| R PINKY TAP | `ZERO_HEADING_REF` | re-zero the glove's yaw integration to the current pose (§8.7) |
| R PINKY HOLD | `YAW_LOOK_AT_CENTROID` | while held, all aircraft yaw to face the formation centre — the camera-inward primitive |

`FRAME_TOGGLE` deserves emphasis. On a ground robot, body-frame vs world-frame control is a
preference. On a drone at altitude it's the difference between intuitive and unflyable. Implement
as: world-frame = rotate the commanded `(vx, vy)` by `−operator_heading` before publishing.

### 8.7 The yaw drift problem — a real defect for aerial use

`glove_left.ino` computes yaw as **pure gyro integration**:

```c
fused_yaw_rad += gz_rad_s * dt;   // no magnetometer, no absolute reference
```

PITCH and ROLL are corrected against gravity by the complementary filter (α = 0.98) and are
absolute. **YAW has no correction and drifts monotonically** — a few degrees per minute with a
calibrated MPU6050, worse if the bias calibration ran while your hand moved.

For ground H.E.R.M.E.S this mostly hides, because YAW feeds `omega` (a *rate*) where slow drift
reads as a faint turning bias you unconsciously correct. But two places consume YAW as an
**absolute angle** and there it matters:

1. `SET_FORMATION_ORIENTATION` → `formation_heading` (absolute). Over a 4-minute flight, the
   formation slowly rotates on its own. For drones with cameras that is very visible.
2. Any future "point the swarm that way" command.

Three fixes, do all three:

- **Use delta, not absolute, for `formation_heading`.** On HOLD entry, snapshot the glove yaw and
  the current `formation_heading`; while held, apply `heading = snapshot + (yaw_now − yaw_at_entry)`.
  Drift over a 2-second hold is negligible. ~8 lines in `_resolve_stream_value` plus a latch.
- **`ZERO_HEADING_REF` gesture** (§8.6) — re-zero on demand, cheap and always available.
- **Long term: fit a magnetometer** (MPU9250 / ICM-20948 drop-in replaces the MPU6050) or fuse the
  glove yaw against the operator's OptiTrack rigid body if you're already tracking them.

### 8.8 SELECTION, FORMATION, BEHAVIOR modes

**SELECTION** (posture `POINT`) — mechanically unchanged. Two-phase group edit
(`select_group` → toggle members → `confirm`) works as-is, including the auto-cancel when the
posture leaves POINT.

| Gesture | Command |
|---|---|
| R INDEX/MIDDLE/RING/PINKY TAP | select r1 / r2 / r3 / r4 |
| R INDEX/MIDDLE HOLD | select r5 / r6 |
| R INDEX/MIDDLE/RING/PINKY DOUBLE_TAP | group slot A / B / C / D |
| R PINKY DOUBLE_TAP | *(currently `CONFIRM_GROUP_ASSIGNMENT` — conflicts with group D)* |
| R RING HOLD | `SELECT_ALL_AIRBORNE` — **new**, one gesture to grab everything that's flying |

Keep robot ids `r1..r6`. Renaming to `d1..d6` would churn the auction, the beacon configs, the
six `optitrack_r*.yaml` files, and `motor_robot_ids` for zero functional gain. They are opaque
strings — leave them.

One pre-existing collision to resolve while you're in here: `GROUP_BINDINGS` maps
`(R,PINKY,DOUBLE_TAP)→"D"` and `CONFIRM_GROUP_ASSIGNMENT` also claims `PINKY DOUBLE_TAP`.
`match_gesture` iterates `SELECTION_COMMANDS` in dict order and gates on `group_edit_active`, so
in practice the confirm wins during an edit and group-D is unreachable then. Worth an explicit
decision rather than relying on dict ordering.

**FORMATION** (posture `FIST`) — 3D formations added. Slot budget is the constraint, so
`ECHELON_L`, `ECHELON_R`, and `DIAMOND` lose their finger bindings. They **stay in
`formation_engine`** and remain reachable from `keyboard_teleop_node` and config; they're just not
worth a scarce finger slot when 3D shapes are.

| Gesture | Formation | Plane |
|---|---|---|
| R INDEX TAP | `LINE` | horizontal |
| R MIDDLE TAP | `COLUMN` | horizontal |
| R RING TAP | `WEDGE` | horizontal |
| R PINKY TAP | `CIRCLE` | horizontal |
| R INDEX DOUBLE_TAP | **`STACK`** | vertical column, `layer_gap_m` apart |
| R MIDDLE DOUBLE_TAP | **`WALL`** | vertical plane grid |
| R RING DOUBLE_TAP | `GRID` | horizontal |
| R PINKY DOUBLE_TAP | **`DOME`** | hemispherical shell |
| R INDEX HOLD + L YAW | `SET_FORMATION_ORIENTATION` | *delta* yaw (§8.7) |
| R MIDDLE HOLD ≥500 ms | `APPLY_FORMATION` | unchanged |
| R RING HOLD + L PITCH | `SET_SPACING_CONTINUOUS` | horizontal spacing, unchanged |
| R PINKY HOLD + L PITCH | **`SET_ALTITUDE_CONTINUOUS`** | commanded formation altitude — the single most useful new continuous control |
| sequence [INDEX TAP, PINKY TAP] | `BREAK_FORMATION` | demoted to a sequence; slots are full |

`SET_ALTITUDE_CONTINUOUS` reuses the `set_param_stream` machinery exactly as spacing does —
map clamped PITCH ∈ [−1,1] onto `[min_z + 0.2, max_z − 0.2]` from the geofence, so the gesture
physically cannot command an out-of-bounds altitude.

**BEHAVIOR** (posture `TWO`):

| Gesture | Behavior | Note |
|---|---|---|
| R INDEX TAP | `PATROL` | rectangle sweep at current altitude |
| R INDEX DOUBLE_TAP | `PATROL_PERIMETER` | horizontal orbit |
| R INDEX HOLD | **`ORBIT_TARGET`** | circle the centroid at fixed radius/altitude, all noses inward. The signature aerial primitive |
| R MIDDLE TAP | `FOLLOW_PATH` | |
| R MIDDLE DOUBLE_TAP | `FOLLOW_PATH_LOOP` | |
| R MIDDLE HOLD | **`LAYERED_SCAN`** | same footprint, staggered altitudes — volumetric coverage |
| R RING TAP | `STATION_KEEP` (was `HOLD_ANCHOR`) | hover at assigned point |
| R RING HOLD | `FOLLOW_ME_TOGGLE` | **needs an operator position source — see below** |
| R PINKY TAP | `RTL` | |
| R PINKY HOLD | `DISPERSE_SCAN` | fan out; now also staggers altitude |
| R RING DOUBLE_TAP | **`SPIRAL_ASCEND`** | orbit while climbing — the reveal shot |
| R PINKY DOUBLE_TAP | **`CONVERGE_CENTROID`** | collapse to a tight cluster (for landing prep) |

`FOLLOW_ME_TOGGLE` is currently a gap in the ground system too: `execute_behavior` builds a WEDGE
around the passed `centroid_xy` (the *swarm's* centroid) with a `1.5 × spacing` backoff along the
heading. There is no operator position anywhere in the system, so "follow me" actually means
"sit behind your own centroid." For drones this is worth fixing properly — you already have
OptiTrack; add a rigid body for the operator (or their vest) and publish it as
`/hermes/operator_pose`. `optitrack_pose_beacon_node` can emit it with the existing code path by
adding one more rigid-body mapping. Then FOLLOW_ME becomes real, and it's the most compelling
demo in the whole system.

### 8.9 PARAMS mode (posture `THREE`) — altitude fits in the free slots

| Gesture | Param | Note |
|---|---|---|
| R INDEX TAP / MIDDLE TAP | `speed_level` − / + | unchanged |
| R INDEX HOLD / MIDDLE HOLD | `speed_level` min / max | unchanged |
| R RING TAP / PINKY TAP | `spacing_level` − / + | unchanged |
| R RING HOLD / PINKY HOLD | `spacing_level` min / max | unchanged |
| R RING DOUBLE_TAP / PINKY DOUBLE_TAP | `aggression_level` − / + | unchanged |
| R INDEX DOUBLE_TAP / MIDDLE DOUBLE_TAP | **`altitude_level` − / +** | *were free* |

`altitude_level` 1–4 → {0.6, 1.0, 1.5, 2.0} m, mirroring the existing
`_spacing_m_for_level` pattern in `SwarmController`. Clamped to the geofence. This gives you a safe,
legible altitude control on day one with no new modes and no analog-axis contention.

Also add, mirroring the existing level scalars:

```python
@staticmethod
def _altitude_m_for_level(level: int) -> float:
    return {1: 0.60, 2: 1.00, 3: 1.50, 4: 2.00}.get(level, 1.00)
```

### 8.10 If you fit the right glove (v2)

With right-hand postures live, the design simplifies substantially:

- **Modes become two-handed**: left posture = domain, right posture = sub-domain. 25 combinations,
  so FLIGHT no longer needs a marginal 6th left posture and the dwell guard becomes belt-and-braces
  rather than the primary protection.
- **ARM / TAKEOFF / LAND become genuinely two-handed** (specific left posture **and** specific
  right posture **and** a long hold). That is about as accident-proof as a gesture interface gets,
  and it's the standard interlock pattern in industrial machine control.
- **`BREAK_FORMATION` returns to a single tap**; the sequence hack goes away.
- With a right MPU6050 too: `vz` gets a dedicated axis, `ALTITUDE_MODIFIER` disappears, and you can
  fly all 4 DOF simultaneously.

Recommend: ship v1 on current hardware, fit the right glove during Phase 5–6 while you're
waiting on flight-test slots.

### 8.11 Complete command inventory

| Domain | Commands | vs ground |
|---|---|---|
| safety | 8 | 4 → 8 (+4 automatic) |
| flight | 8 | **all new** |
| pilot | 8 | 3 → 8 |
| selection | 11 | 10 → 11 |
| formation | 13 | 13 (4 formations swapped for 3D, +altitude, break→sequence) |
| behavior | 12 | 8 → 12 |
| params | 12 | 10 → 12 |
| **total** | **72** | **48 → 72** |

---

## 9. Formation library in 3D

### 9.1 Engine change

`compute_formation_offsets(type, n, spacing) -> [(x, y)]` becomes `-> [(x, y, z)]`.
`compute_formation_targets(...) -> {id: (x, y, yaw)}` becomes `-> {id: (x, y, z, yaw)}`.

`FormationParams` gains `layer_gap_m` and `altitude_m`. `_center_offsets` centres in x and y as
today but **must not centre z** — z is measured from the commanded formation altitude, and
re-centering it would fight the altitude command.

Existing planar formations return `z = 0` offsets, so all four current shapes and their tests
keep working with a trivial tuple widening. Ground callers ignore the third element.

### 9.2 Formation catalogue

| Formation | Offsets | Aerial notes |
|---|---|---|
| `LINE` | y-spaced, z = 0 | abreast at one altitude. Simplest, safest, use it for first flights |
| `COLUMN` | x-spaced, z = 0 | single file. **Watch downwash** if the heading means they're stacked front-to-back within `DOWNWASH_RADIUS` while descending |
| `WEDGE` | V, z = 0 | classic; leader ahead |
| `CIRCLE` | ring, r ≈ n·spacing/2π, z = 0 | orbit ring. Pairs with `ORBIT_TARGET` and inward yaw |
| `GRID` | ⌈√n⌉ cols, z = 0 | horizontal lattice |
| `ECHELON_L/R`, `DIAMOND` | as today, z = 0 | kept in engine, unbound from gestures |
| **`STACK`** | x=y=0, `z = (i − (n−1)/2) · layer_gap` | vertical column. `layer_gap ≥ 0.60 m` **enforced, not defaulted** — clamp it in the engine, do not trust the caller |
| **`WALL`** | grid in the y–z plane | vertical curtain. Visually the most striking formation, and completely impossible with ground robots — this is your demo shot |
| **`DOME`** | Fibonacci points on an upper hemisphere, radius from spacing | shell around the centroid. Sensor-coverage story |
| **`HELIX`** | `θ = 2πi/n`, `z` rising linearly | pairs with `SPIRAL_ASCEND` |
| **`V_ECHELON_3D`** | wedge in x–y with a linear z gradient | migrating-flock look; each aircraft out of the one ahead's wash |

### 9.3 Enforce vertical separation in the engine

Any formation whose z offsets are closer than `min_vertical_separation_m` **for aircraft within
`DOWNWASH_RADIUS_M` horizontally** must be rejected or auto-expanded. Put this check in
`compute_formation_targets`, not in the agent — one check, all callers covered, and it's a pure
function you can test exhaustively:

```python
def assert_downwash_safe(targets, radius_m, min_vert_m):
    """Raise (or auto-expand layer_gap) if any pair is horizontally within radius_m
    and vertically closer than min_vert_m."""
```

This is the single most valuable unit test in the aerial system. Write it before anything flies.

### 9.4 Formation altitude must be commanded

Elaborated in §13.6 because it's a defect-class finding, not a design choice.

---

## 10. Behaviors

### 10.1 Current behaviors, honestly assessed

`execute_behavior` is a **stateless target generator**. Called with `(behavior, ids, centroid,
params)` it returns one static target map. It does not integrate time. Consequences today:

- `PATROL` assigns each robot to a *corner* of a rectangle (`waypoints[i % 4]`) and leaves it
  there. It is a static 4-point formation, not a patrol.
- `PATROL_PERIMETER` places robots on a circle with tangential yaw. They sit on the circle. They
  do not orbit.
- `FOLLOW_PATH` stages everyone in a LINE at `waypoints[0]`. It does not traverse the path.

For ground robots this reads as "assume the patrol posture," which is defensible. For drones, the
behaviors that justify flying at all — orbit, spiral, sweep — are **inherently time-parameterized**.
This must be fixed.

### 10.2 The phase clock

Minimal, deterministic fix. `swarm_control_node` owns a monotonic behavior clock, advanced in the
30 Hz intent timer, reset when `active_behavior` changes, and published as `behavior_phase_s`
(§5.3). `execute_behavior` gains a `phase_s` keyword and time-varying behaviors evaluate their
path at that phase.

Why this design and not a trajectory follower per aircraft:

- **Determinism.** Every agent computes targets from the *same* phase value that arrived in the
  same intent message. No clock sync, no drift between aircraft, no distributed state.
- **The auction keeps working.** Slot targets are still a pure function of the intent, so bids
  stay comparable.
- **It stays testable offline.** `execute_behavior(..., phase_s=t)` is a pure function; sweep `t`
  in a unit test and assert the path is continuous, closed, and inside the geofence.
- **~40 lines total**, and it strictly generalizes the current behaviour (`phase_s = 0` reproduces
  today's output exactly, so existing tests pass unchanged).

```python
# ORBIT_TARGET
omega = speed_scale * base_rate / max(radius, 0.5)     # constant tangential speed
for i, sid in enumerate(slot_ids):
    ang = (2*pi*i/n) + omega * phase_s                  # phase-shifted per slot
    targets[sid] = (cx + r*cos(ang), cy + r*sin(ang), alt,
                    ang + pi)                            # nose inward
```

Guard rail: **rate-limit target motion.** A behavior that advances its target faster than the
aircraft can fly produces a permanent lag and, worse, a formation that stretches until the
slowest aircraft is outside the avoidance radius of the next slot. Clamp target velocity to
`0.8 × max_linear_speed` inside the behavior, not in the agent.

### 10.3 Aerial behavior specifications

| Behavior | Phase-dependent | Spec |
|---|---|---|
| `STATION_KEEP` | no | hover at assigned slot; unchanged semantics |
| `ORBIT_TARGET` | **yes** | radius from spacing×n, altitude commanded, nose inward, ω from `speed_scale` |
| `PATROL` | **yes** | traverse the rectangle; aircraft phase-offset by `i/n` of the loop |
| `PATROL_PERIMETER` | **yes** | actually orbit the circle |
| `FOLLOW_PATH` / `_LOOP` | **yes** | traverse waypoints at `speed_scale`; formation maintained about the moving path point |
| `LAYERED_SCAN` | **yes** | lawnmower footprint, aircraft *i* at `min_z + i·layer_gap` |
| `SPIRAL_ASCEND` | **yes** | orbit + linear climb; terminate at `max_z − margin`, then hold |
| `DISPERSE_SCAN` | no | radial fan-out; **add altitude stagger** so the fan isn't coplanar |
| `CONVERGE_CENTROID` | no | tight cluster at distinct layers — landing prep |
| `FOLLOW_ME` | no | wedge behind `/hermes/operator_pose` (§8.8), at commanded altitude |
| `RTL` | no | `home_xyz` at `rtl_altitude_m`, then land. **Climb to RTL altitude first**, then translate — never translate at low altitude toward a launch point where people stand |

### 10.4 Behavior transitions

Ground behaviors switch instantly; a ROSbot just changes direction. A drone mid-orbit switched to
STACK will fly a straight line through whatever is between. Add a transition rule to the agent:

- On `active_behavior` change, **hover for `transition_settle_ms` (≈400 ms)** before accepting the
  new target. Kills the lurch and gives the auction time to converge on the new slot set.
- If the new target is >`max_transition_jump_m` (≈1.5 m) away, insert a waypoint at the current
  altitude before translating — climb/descend and translate as separate phases, not diagonally.

---

## 11. Haptic feedback redesign

### 11.1 What the vest can actually express

Six motors. **Binary on/off** — `esp32_haptic_vest.ino` does
`digitalWrite(pin, level > 0 ? HIGH : LOW)`, so the 0–255 level from `haptic_vest_node` is
quantized to 1 bit. The only encoding dimensions today are **which motor** and **rhythm**.

Ground H.E.R.M.E.S has 6 event classes and it already crowds the rhythm space. Drones need 10+.
This does not fit.

### 11.2 Event priority ladder

| Pri | Event | Scope | Pattern (period / on-windows) |
|---|---|---|---|
| 1 | `lost_comm` | per aircraft | 700 ms / 0–260 (long buzz) — unchanged |
| 2 | `battery_critical` | per aircraft | 500 ms / 0–70, 130–200, 260–330 (urgent triple) — **new** |
| 3 | `geofence_breach` | per aircraft | 400 ms / 0–200 (harsh 50% duty) — **new** |
| 4 | `robot_error` / `status_missing` | per aircraft | 900 ms / 0–120, 220–340, 440–560 — unchanged |
| 5 | `battery_low` | per aircraft | 2000 ms / 0–90, 300–390 (lazy double) — **new** |
| 6 | `obstacle` | per aircraft | 900→400 ms by level / 0–120 — unchanged, now includes up/down |
| 7 | `descending` / `landing` | per aircraft | 600 ms / 0–300 ramp-feel — **new** |
| 8 | `formation_reached` | per aircraft | 700 ms window, triple tick — unchanged |
| 9 | `gesture_ok` | swarm-wide | 450 ms window, double tick — unchanged |
| 10 | `swarm_movement` | swarm-wide | 700 ms window, double tick — unchanged |

Insertion points 2, 3, 5, 7 slot into the existing `_tick` cascade in `haptic_vest_node` exactly
like the current `elif` chain. The `_windowed_pulse(now_ms, period, windows)` helper already does
all the pattern work — the new events are data, not code.

### 11.3 Ten patterns don't fit in one bit

Honest assessment: an operator can reliably discriminate maybe **4–5 rhythms** on a single
vibration motor under cognitive load. Ten is fantasy. Three mitigations, in order of cost:

**(a) Context-dependent mapping (free, config only).** The vest doesn't need to express all ten at
once. During `TAKING_OFF`/`LANDING`, only lifecycle + battery + geofence events can occur; during
cruise, only formation/obstacle/movement. Add a `vest_profile` selected by swarm flight state, each
profile mapping ≤5 events. `haptic_vest_node` already has `motor_robot_ids` and `motor_labels` as
parameters — add `vest_profiles` as a parameter dict and select by state. This is the right answer
and costs nothing.

**(b) PWM in the vest firmware (one small change, high value).** Replace `digitalWrite` with
`ledcWrite` to get intensity as a genuine second dimension: intensity = urgency, rhythm = kind.
That roughly doubles discriminable events.

```c
// setup(): one channel per motor
for (int i = 0; i < 6; ++i) { ledcSetup(i, 200 /*Hz*/, 8 /*bit*/); ledcAttachPin(kMotorPinPairs[i][1], i); }
// applyMotorLevels():
for (int i = 0; i < 6; ++i) ledcWrite(i, motorLevels[i]);
```

Caveat: ERM motors have a start-up threshold — below ~30% duty they hum without spinning. Find the
floor empirically per motor and map levels onto `[floor, 255]`, not `[0, 255]`. Real hardware needs
a real calibration table; leave it as a parameter.

**(c) Spatial re-mapping for directional events.** Motors are physically placed
(`left_sleeve, right_sleeve, left_shoulder, right_shoulder, left_torso, right_torso`) but mapped
to *aircraft*, so location carries no spatial meaning. For geofence breach, the natural encoding is
"buzz the side of your body the breach is on" — which requires temporarily abandoning the
per-aircraft mapping. Fold this into (a) as a `directional` profile used only during breach and
obstacle events.

Also note the current `motor_robot_ids` default is `["r5","r6","r1","r3","r2","r4"]` — a
deliberate physical wiring order, not a bug. Preserve it.

### 11.4 Altitude awareness

Altitude is the one state variable the operator **cannot see** — a drone at 1.2 m and one at 2.0 m
look nearly identical from the ground, and misjudging it is how ceilings get hit. Two options:

- **Shoulder = high band, torso = low band** during altitude-critical phases (part of the
  `directional` profile). Loses per-aircraft resolution, gains an altitude sense.
- **Leave altitude to the visual channel** (§12) and keep the vest for events. Simpler; recommended
  if the 3D viz is available to the operator.

Pick based on whether the operator can see a screen. If they're watching the aircraft, use the vest.

---

## 12. Visualization

The `feat-hermes-3d-viz` branch (Vuer-based digital twin, `hermes_viz` package) is **not on
`main`** — it lives on the worktree branch. For drones it goes from nice-to-have to
operationally necessary, because altitude and separation are not judgeable by eye.

Changes needed:

| Item | Change |
|---|---|
| `transforms.py` | carry `z`/`vz` from the beacon into scene coordinates |
| `scene/robots.py` | drone mesh instead of the ROSbot URDF; ~200-line file, straightforward swap |
| altitude reference | vertical drop-line from each aircraft to the floor + a numeric altitude tag. This is the single most valuable addition — it makes altitude readable at a glance |
| geofence | wireframe box, faces highlighted red on approach |
| downwash | translucent cone under each aircraft; visually obvious when one is in another's wash |
| separation | pair lines that turn red when below `min_separation` |
| battery | per-aircraft ring gauge, colour-coded |
| views | add a side elevation next to the existing perspective — altitude errors are invisible in top-down |
| target ghosts | render commanded slot targets as translucent markers, so operator sees intent vs actual |

Bandwidth note from prior work on that branch: the hands scene was removed and HUD HTML disabled
to cut ~600 upserts/s. Adding `z` widens existing payloads without adding messages, so it is
bandwidth-neutral. The geofence box and downwash cones are static or slowly-varying geometry —
upsert them once, not per frame.

---

## 13. Findings in the current code that become hazards in the air

These are behaviours I verified by reading the code. They're all defensible on the ground and all
dangerous airborne. Fix them as part of Phase 3, not "later."

**13.1 Deadman defaults OPEN on the first tick.**
`SafetyEvaluator.tick` line ~88: `gate_motion = state.deadman_active if last_gate is not None else True`.
On the first sample, if the accel is present but lands inside the palm-up/palm-down hysteresis
band, neither branch fires and `_set_gate(True)` publishes **motion allowed**. For a ROSbot that's
a shrug. For a drone, "motion allowed by default at startup" is wrong.
*Fix:* initialize `last_gate = False` and require an explicit, debounced palm-down observation
before the first `True`. Two lines.

**13.2 ESTOP threshold is too low for an operator with arms.**
0.75 g dynamic accel for 220 ms is reachable by walking briskly or gesturing emphatically.
*Fix:* §7.2 — raise to ~1.10 g / 500 ms and require a co-occurring posture. Calibrate against a
logged recording of a real demo run.

**13.3 `emergency_stop` clears flight-relevant state.**
`_stop_all()` wipes `active_behavior`, formation, targets, `last_cmd_vel`, group edits, and sets
`deadman_active = False`. On the ground that's a clean slate. Airborne, wiping the target set means
the agent has nothing to hold position against.
*Fix:* split into `_stop_tasks()` (behaviors, formations, group edits — safe airborne) and
`_stop_all()` (used only when landed and disarmed).

**13.4 Deadlock recovery spins in place.**
`_maybe_deadlock_recovery` publishes pure yaw rate for 1200 ms. A ROSbot spins and finds a new
heading. A drone spins and stays exactly as stuck, having burned 1200 ms of a 7-minute battery.
*Fix:* recover in **z**. Offset altitude by ±0.3 m (sign from `robot_id` hash, as the current code
already does for turn direction) to break the conflict plane, hold 800 ms, then retry. This is
both more effective and simpler than a horizontal escape.

**13.5 Every `return` path must publish a setpoint.**
Several ground paths return after publishing a zero `Twist`; a zero `Twist` means "descend" to a
velocity-controlled drone, and *no* publish means OFFBOARD timeout on PX4.
*Fix:* structural — `_publish_hover()` on every early return, and a unit test that walks all
branches of `_tick` asserting exactly one publish per call.

**13.6 Live-centroid formations make altitude sag.**
This is the subtlest and most important one. `_compute_slot_targets` calls `_centroid_from_state`,
which averages the **current beacon positions** of the selection; the intent's `centroid` is only a
fallback (and nothing publishes `/hermes/centroid`, so it's always `(0,0)`). Extend that pattern to
z naively and you get:

> target_z = mean(current_z) + offset → aircraft track slightly low (steady-state error, battery
> sag, controller lag) → next tick's mean is lower → target lowers → repeat.

A slow, uncommanded descent with no fault indicated anywhere. The horizontal axes don't have this
pathology because horizontal error is unbiased noise, while **altitude error is biased downward by
gravity.**
*Fix, mandatory:* **z of the formation centre is always the commanded `formation_altitude_m`,
never derived from measurement.** Horizontal centroid may stay live. Assert this in a unit test:
feed a sequence of beacons that all read 0.1 m low and assert the commanded z does not move.

**13.7 No absolute yaw reference.**
§8.7. Gyro-only integration; `formation_heading` drifts.
*Fix:* delta-based heading during hold, plus a re-zero gesture.

**13.8 `auto_select_all_on_start = True`.**
§7.8. Default it false for aerial.

**13.9 Vest motor levels are 1-bit.**
§11.1. The `0..255` field is a lie at the firmware boundary. Either implement PWM or stop pretending
the range exists in the node's debug output.

**13.10 Two trees must stay in sync.**
`gestures/` + `swarm/` (root) vs `ros_version/src/hermes_control/hermes_control/{gestures,swarm}/`.
Per `CLAUDE.md` every logic edit lands twice; a prior observation records the two trees having
**already drifted** on classifier thresholds. Every change in §8–§10 touches both. Add a CI check
(`diff -r` on the two directories, allowing only the import-path lines to differ) — three lines of
shell, and it removes a whole class of "works standalone, misbehaves in ROS" bug. Worth doing
before starting, not after.

---

## 14. File-by-file change map

### New

| File | Purpose | ~LOC |
|---|---|---|
| `ros_version/.../drone_agent_node.py` | 4-DOF agent, lifecycle, hover-first faults | 550 |
| `ros_version/.../flight_stack_adapter_node.py` | Crazyswarm2 / MAVROS / uXRCE isolation | 300 |
| `swarm/geofence.py` (+ ROS copy) | clamp + breach severity | 70 |
| `ros_version/.../config/flight_volume.yaml` | fence, altitudes, separations | 30 |
| `ros_version/.../config/drone_agent_*.yaml` | per-aircraft params ×6 | 6×25 |
| `ros_version/.../launch/aerial_swarm.launch.py` | full aerial bring-up | 120 |
| `ros_version/.../launch/drone_agent.launch.py` | one aircraft | 60 |
| `tests/test_formation_3d.py` | 3D shapes + **downwash assertion** | 200 |
| `tests/test_geofence.py` | clamp/breach edges | 120 |
| `tests/test_flight_lifecycle.py` | state machine, preflight refusals | 180 |
| `tests/test_failsafe_ladder.py` | staged timeouts, re-engage latch | 160 |
| `tests/test_frame_conversion.py` | ENU↔NED, sign traps | 60 |
| `tests/test_hover_semantics.py` | every `_tick` branch publishes once | 140 |

### Modified

| File | Change | Risk |
|---|---|---|
| `gestures/registry.py` ×2 | +FLIGHT mode, +6th posture, 3D formations, altitude, retuned ESTOP, new bindings | low — data |
| `gestures/posture_classifier.py` ×2 | +1 pattern | low |
| `gestures/matcher.py` ×2 | `require_L_posture` on safety, `min_mode_dwell_ms`, delta-yaw for heading, `vz` in `_resolve_cmd_vel` | **medium — this file gates every command** |
| `gestures/safety.py` ×2 | default gate False (13.1), posture-gated ESTOP, `emergency_descend` | medium |
| `swarm/formation_engine.py` ×2 | 3D offsets, 4 new shapes, downwash assertion | medium |
| `swarm/behavior_engine.py` ×2 | `phase_s`, 3D, 4 new behaviors, target rate limit | medium |
| `swarm/swarm_controller.py` ×2 | altitude level/commanded altitude, phase clock, `_stop_tasks` split | medium |
| `swarm_control_node.py` | intent fields, phase clock, `auto_select_all=False` | low |
| `optitrack_pose_beacon_node.py` | `vertical_axis`, `z`, `vz` | low |
| `robot_haptic_status_node.py` | lateral/up/down ranges, battery, flight state | low |
| `haptic_vest_node.py` | 4 new events, priority ladder, `vest_profiles` | low |
| `keyboard_teleop_node.py` | aerial equivalents — **your safest test harness, keep it current** | low |
| `esp32_haptic_vest.ino` | *optional* PWM | low |
| `glove_right.ino` | *optional* 4 flex sensors (v2) | low |

### Untouched

`glove_left.ino`, `vest_serial_bridge_node.py`, `fsr_tracker.py`, `imu_filter.py`,
`recognizer.py`, `models.py`, `decentralized_robot_agent_node.py` (keeps flying ROSbots),
`robot_state_beacon_node.py`, all ESP-NOW transport.

---

## 15. Phased plan

Exit criteria are the point. Do not advance on "it looks right."

### Phase 0 — Decide and procure (1 week, no code)
Resolve §19. Order hardware. Set up the OptiTrack rigid bodies for aircraft (and for the operator,
if doing FOLLOW_ME). Add the two-tree sync CI check (13.10) *first* — everything after this
touches both trees.
**Exit:** platform chosen, hardware ordered, sync check green.

### Phase 1 — 3D core, offline (1 week)
`formation_engine` and `behavior_engine` to 3D. Phase clock. Schema additions. `geofence.py`.
Commanded-altitude fix (13.6). All new unit tests. No ROS, no hardware.
**Exit:** `pytest -q` green including the downwash assertion and the altitude-sag test. Every
pre-existing test still passes unmodified — that's your proof the changes are additive.

### Phase 2 — Drone agent in simulation (1.5 weeks)
`drone_agent_node`, `flight_stack_adapter_node`, launch files. Extend
`hermes_viz/simulator.py` to 3D. Exercise the whole loop with fake beacons.
**Exit:** 6 simulated aircraft take off, hold LINE / STACK / WALL / DOME, orbit, and land, driven
from `keyboard_teleop_node`. Zero downwash violations logged across a 10-minute run.

### Phase 3 — Safety layer (1 week) — **do not skip or compress**
All of §7 and all ten items in §13. Failsafe ladder with fault injection. Geofence breach.
Battery-critical override. Preflight gate. Re-engage latch.
**Exit:** a fault-injection suite passes: kill the intent / kill beacons / drain a battery /
command outside the fence / stale the left glove — and in every case the simulated aircraft ends
in a safe state, and **a single fresh packet does not silently restore control.**

### Phase 4 — Gestures and haptics (1 week)
Registry v2, 6th posture, mode dwell, delta-yaw, altitude bindings. Haptic ladder + profiles.
Optional vest PWM.
**Exit:** the new posture classifies reliably on *your* hand over 100 trials (log it, don't
eyeball it). All 72 commands reachable and verified from the gloves against a simulated swarm.
Zero false ESTOPs across a full simulated demo with the operator moving normally.

### Phase 5 — First flight, one aircraft (1 week)
Tethered or netted. Spotter with a hardware kill. `LINE` formation, n=1. Manual PILOT only, then
takeoff/land, then a single-aircraft formation hold.
**Exit:** 20 consecutive takeoff→hover→land cycles with no unexpected behaviour. Deadman verified
in air (palm-up → it hovers, and stays hovering). Hardware kill verified. Fit the right glove
sensors this week while flight slots are the bottleneck.

### Phase 6 — Two aircraft (1 week)
The phase where downwash becomes real. LINE, STACK (validate the 0.60 m gap empirically — the
number in this document is a starting guess, your props decide), the slot auction with two live
bidders, `CONVERGE_CENTROID`.
**Exit:** measured minimum separation never below the configured floor across 10 formation
transitions. Empirically determined `min_vertical_separation_m` written into config.

### Phase 7 — Full swarm (1.5 weeks)
Six aircraft. Staggered takeoff. All formations. Behaviors with the phase clock. Battery-driven
attrition (let one actually go LOW and watch the formation reflow). Full haptic ladder in flight.
**Exit:** a 3-minute unbroken demo: takeoff → LINE → WALL → ORBIT → STACK → RTL → land, gesture-driven
end to end, no interventions.

### Phase 8 — Visualization and polish (1 week)
Merge `feat-hermes-3d-viz`, add §12. Operator-facing altitude readout. Documentation.
**Exit:** an observer who has never seen the system can read altitude and separation off the
screen and correctly predict what the swarm does next.

**Total: ~10 weeks**, of which 4 are software-only and hardware-independent (Phases 0–3) — start
those the day you order parts.

---

## 16. Test strategy

| Level | What | Where |
|---|---|---|
| Unit | 3D formations, downwash assertion, geofence, phase-clock continuity, ENU↔NED, altitude-sag | `tests/` |
| Unit | Lifecycle state machine, preflight refusals, failsafe ladder timing, re-engage latch | `tests/` |
| Unit | `_tick` publishes exactly once on **every** branch | `tests/test_hover_semantics.py` |
| Logic integration | Full gesture→intent→agent→target loop against the 3D simulator | extend `hermes_viz` sim |
| Fault injection | Kill each link in the §7.3 table, assert the ladder | scripted, must be automated |
| Physics sim | Crazyswarm2 sim / PX4 SITL, real timing, real controller | before first flight |
| Bench | Left glove battery pull mid-hover; right glove pull; USB unplug | manual, logged |
| Flight | Phases 5–7 exit criteria | manual, logged, spotter present |

Two non-negotiables:

- **`tests/test_formation_3d.py::test_downwash_separation` gates every commit.** No formation may
  place aircraft in each other's wash. This is a pure function over pure geometry — there is no
  excuse for not testing it exhaustively.
- **Fault injection is automated, not manual.** A failsafe you test by hand once is a failsafe
  that regresses in week 6.

---

## 17. Regulatory and operational

### Indoor (recommended)
Both FAA and EASA regulate flight in *navigable airspace*. Indoor flight in an enclosed structure
falls outside that. No Part 107 certificate, no registration, no waiver. **This is the main reason
to start indoors** — you skip a multi-month regulatory path and iterate at software speed.

You still need, and should treat as mandatory:

- Netting or a defined no-personnel volume, with the geofence set **inside** it with margin
- A spotter holding a hardware kill, who is not the gesture operator
- Eye protection for everyone in the room (props at eye height)
- Written pre/post-flight checklists (extend `docs/LAB_BRINGUP_CHECKLIST.md`)
- Battery handling discipline — LiPo charging bag, no unattended charging, log cycles
- An incident log. Every crash gets a root cause, exactly like a software bug

### Outdoor US
- **Part 107** remote pilot certificate.
- **§107.35** prohibits one pilot operating multiple aircraft — this is the binding constraint for
  swarms and requires a waiver. Waivers are granted (drone-show operators hold them) but the
  application demands a documented safety case: geofencing, failsafe behaviour, C2 link analysis,
  lost-link procedures, personnel separation. **Everything in §7 is the technical content of that
  application** — building it properly now is what makes the waiver achievable later.
- Registration per aircraft; Remote ID (broadcast module or standard RID) for anything ≥250 g.
- Airspace authorization (LAANC) in controlled airspace.

### Outdoor EU
- EASA Open category A1–A3 by aircraft class and proximity to people; multi-aircraft single-pilot
  operation generally pushes into Specific category → SORA risk assessment + operational
  authorization.

**Recommendation:** stay indoors for the research/demo phase. Revisit only if a specific
application (§18) requires outdoor operation.

---

## 18. Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Downwash-induced mid-air collision | med | high | layer assignment (§7.5a), engine-level assertion (§9.3), empirical gap in Phase 6 |
| Uncommanded descent from centroid feedback (13.6) | **high if unaddressed** | high | commanded altitude only; unit test |
| False ESTOP mid-flight | med | med | posture gate + raised threshold; calibrate against logged demo motion |
| Battery exhaustion mid-behavior | high | high | per-aircraft CRITICAL override; 4-min demo ceiling; charge rotation |
| Gesture misclassification of new posture | med | med | complement-of-THREE choice + 600 ms dwell + 100-trial validation |
| Radio saturation with 6 aircraft | low | med | Crazyradio headroom at 6; 2nd radio available; measure packet loss in Phase 7 |
| OptiTrack occlusion / marker loss | med | high | onboard estimator hover on beacon loss; descend after 3 s; Flow deck as fallback |
| OFFBOARD dropout (PX4 path) | med | high | always-publish (13.5) + a branch-coverage test |
| Two-tree drift (13.10) | **high** | med | CI diff check in Phase 0 |
| Yaw drift rotating the formation | high | low | delta-heading + re-zero gesture |
| Haptic overload — operator can't discriminate 10 events | high | med | context profiles (§11.3a) + PWM |
| Scope creep into outdoor/regulatory | med | high | explicit indoor decision; revisit only on application need |

---

## 19. Decisions I need from you

1. **Platform.** Crazyflie/Crazyswarm2 indoor (my recommendation, and everything above assumes it),
   or PX4/MAVROS? If PX4: indoor with OptiTrack, or outdoor with RTK?
2. **Swarm size.** Six, to match `r1..r6` and the six vest motors? Or a different count — the vest
   mapping is 6-motor-fixed and that's the real constraint on the haptic design.
3. **Right glove upgrade.** Fit 4 flex sensors (§8.2)? It's ~€10 and firmware-only, and it changes
   the gesture design substantially for the better. My recommendation: yes, during Phase 5.
4. **Altitude control for v1.** Modifier remap (§8.6a), discrete levels (§8.6b), or both? I'd ship
   both — levels for safety on day one, modifier for expressiveness.
5. **Mixed fleet?** Should ROSbots and drones be commandable in the same session? Everything above
   is designed to permit it (additive schemas, ground agent untouched), but it is extra test
   surface. If never, some simplifications open up.
6. **Operator tracking.** Add an OptiTrack rigid body for the operator to make FOLLOW_ME real?
   It's the most compelling demo in the system and it's nearly free given your existing setup.
7. **Timeline pressure.** Is there a demo date? Phases 0–4 are ~4.5 weeks of pure software with no
   hardware dependency — if there's a deadline, that's what parallelizes.

---

## Appendix A — Command reference, all modes

Left posture selects mode. All right-hand gestures are on the right glove FSRs.
`•` = unchanged from ground H.E.R.M.E.S, `+` = new, `~` = changed.

### Cross-mode safety
| | Gesture | Command |
|---|---|---|
|~| L shake ≥1.10 g / 500 ms while `FIST` | `EMERGENCY_DESCEND` |
|~| L palm-up | `DEADMAN` → hover |
|~| `OPEN` + R MIDDLE TAP | `HOVER_HOLD` |
|~| `OPEN` + R MIDDLE DOUBLE_TAP | `RESUME` (+ clears re-engage latch) |
|+| *automatic* | `GEOFENCE_BREACH`, `LINK_LOSS`, `BATTERY_CRITICAL` |
|+| hardware switch | motor kill (spotter) |

### FLIGHT — posture `PINKY_EXT` (new), 600 ms dwell
| | Gesture | Command |
|---|---|---|
|+| R INDEX HOLD 1500 | `ARM_SELECTED` |
|+| R INDEX DOUBLE_TAP | `DISARM_SELECTED` (landed only) |
|+| R MIDDLE HOLD 1200 | `TAKEOFF` (staggered) |
|+| R MIDDLE DOUBLE_TAP | `SET_HOME_HERE` (landed only) |
|+| R RING HOLD 1200 | `LAND_IN_PLACE` |
|+| R RING DOUBLE_TAP | `RTL` |
|+| R PINKY HOLD 1500 | `EMERGENCY_DESCEND` |
|+| R PINKY TAP | `PREFLIGHT_REPORT` |

### PILOT — posture `OPEN` (requires deadman)
| | Gesture | Command |
|---|---|---|
|~| L PITCH / ROLL / YAW | `MANUAL_FLY` → vx / vy / yaw-rate |
|+| R INDEX HOLD | `ALTITUDE_MODIFIER` (PITCH→vz while held) |
|•| R MIDDLE TAP / DOUBLE_TAP | `HOVER_HOLD` / `RESUME` |
|•| R RING HOLD | `PRECISION_FLY` |
|~| R RING TAP | `FRAME_TOGGLE` (world vs operator-relative) |
|+| R PINKY TAP | `ZERO_HEADING_REF` |
|+| R PINKY HOLD | `YAW_LOOK_AT_CENTROID` |

### SELECTION — posture `POINT`
| | Gesture | Command |
|---|---|---|
|•| R I/M/R/P TAP | select r1 / r2 / r3 / r4 |
|•| R I/M HOLD | select r5 / r6 |
|•| R I/M/R/P DOUBLE_TAP | group slot A / B / C / D |
|•| R PINKY DOUBLE_TAP (in edit) | `CONFIRM_GROUP_ASSIGNMENT` |
|+| R RING HOLD | `SELECT_ALL_AIRBORNE` |

### FORMATION — posture `FIST`
| | Gesture | Command |
|---|---|---|
|•| R I/M/R/P TAP | `LINE` / `COLUMN` / `WEDGE` / `CIRCLE` |
|+| R INDEX DOUBLE_TAP | `STACK` |
|+| R MIDDLE DOUBLE_TAP | `WALL` |
|•| R RING DOUBLE_TAP | `GRID` |
|+| R PINKY DOUBLE_TAP | `DOME` |
|~| R INDEX HOLD + L YAW | `SET_FORMATION_ORIENTATION` (delta yaw) |
|•| R MIDDLE HOLD 500 | `APPLY_FORMATION` |
|•| R RING HOLD + L PITCH | `SET_SPACING_CONTINUOUS` |
|+| R PINKY HOLD + L PITCH | `SET_ALTITUDE_CONTINUOUS` |
|~| sequence [INDEX TAP, PINKY TAP] | `BREAK_FORMATION` |

### BEHAVIOR — posture `TWO`
| | Gesture | Command |
|---|---|---|
|•| R INDEX TAP / DOUBLE_TAP | `PATROL` / `PATROL_PERIMETER` |
|+| R INDEX HOLD | `ORBIT_TARGET` |
|•| R MIDDLE TAP / DOUBLE_TAP | `FOLLOW_PATH` / `FOLLOW_PATH_LOOP` |
|+| R MIDDLE HOLD | `LAYERED_SCAN` |
|~| R RING TAP | `STATION_KEEP` |
|+| R RING DOUBLE_TAP | `SPIRAL_ASCEND` |
|•| R RING HOLD | `FOLLOW_ME_TOGGLE` |
|~| R PINKY TAP | `RTL` |
|+| R PINKY DOUBLE_TAP | `CONVERGE_CENTROID` |
|•| R PINKY HOLD | `DISPERSE_SCAN` (+altitude stagger) |

### PARAMS — posture `THREE`
| | Gesture | Command |
|---|---|---|
|•| R INDEX TAP / MIDDLE TAP | `speed_level` − / + |
|•| R INDEX HOLD / MIDDLE HOLD | `speed_level` min / max |
|+| R INDEX DOUBLE_TAP / MIDDLE DOUBLE_TAP | `altitude_level` − / + |
|•| R RING TAP / PINKY TAP | `spacing_level` − / + |
|•| R RING HOLD / PINKY HOLD | `spacing_level` min / max |
|•| R RING DOUBLE_TAP / PINKY DOUBLE_TAP | `aggression_level` − / + |

---

## Appendix B — Parameter starting values

Every one of these is a starting guess. Real props, real batteries, and a real OptiTrack volume
will move them. Keep them all as ROS parameters and expect to tune every one on hardware.

```yaml
# flight_volume.yaml
geofence:
  min_x: -3.0
  max_x:  3.0
  min_y: -2.5
  max_y:  2.5
  min_z:  0.30        # NEVER 0 — a zero floor makes "land" and "breach" the same thing
  max_z:  2.20
  clamp_margin_m: 0.15

altitudes:
  default_takeoff_m: 1.00
  rtl_m: 1.60
  levels_m: [0.60, 1.00, 1.50, 2.00]

separation:
  horizontal_min_m: 0.30
  vertical_min_m: 0.60          # downwash — VALIDATE EMPIRICALLY IN PHASE 6
  downwash_radius_m: 0.35
  downwash_height_m: 0.60

speeds:
  max_linear_mps: 0.80          # start at 0.40 for first flights
  max_vz_up_mps: 0.50
  max_vz_down_mps: 0.35         # asymmetric on purpose — see §6.6
  max_yaw_rate_rps: 1.20

timing:
  control_hz: 30
  intent_hz: 30
  stop_on_missing_intent_ms: 600
  hover_to_rtl_ms: 3000
  rtl_to_land_ms: 10000
  assignment_lock_ms: 1500
  transition_settle_ms: 400
  takeoff_stagger_ms: 800

battery:
  low_pct: 30.0
  critical_pct: 15.0
  critical_cell_v: 3.30

gestures:
  flight_mode_dwell_ms: 600
  arm_hold_ms: 1500
  takeoff_hold_ms: 1200
  land_hold_ms: 1200
  estop_threshold_g: 1.10
  estop_hold_ms: 500
  estop_require_posture: "FIST"
```
