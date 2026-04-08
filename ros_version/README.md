# H.E.R.M.E.S ROS2 Package (Duplicated Version)

This folder contains a ROS2-packaged copy of the current H.E.R.M.E.S software.
The original code at the repo root is unchanged.

## Layout

- `ros_version/src/hermes_control/`: ROS2 Python package
- `hermes_control/gestures/`: duplicated gesture pipeline modules
- `hermes_control/swarm/`: duplicated swarm modules
- `hermes_control/gesture_pipeline_node.py`: raw JSON -> command packet publisher
- `hermes_control/swarm_control_node.py`: command packet consumer -> swarm state + swarm intent publisher
- `hermes_control/decentralized_robot_agent_node.py`: per-robot local executor (`swarm_intent` + peer states -> local `/cmd_vel`)
- `hermes_control/robot_state_beacon_node.py`: per-robot odometry beacon publisher
- `launch/hermes_ros.launch.py`: launches both nodes
- `launch/robot_agent.launch.py`: launches one robot beacon + one robot local agent
- `launch/hermes_decentralized.launch.py`: full single-machine demo stack (pipeline + control + one robot agent)

## Build

From repository root:

```bash
cd ros_version
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch hermes_control hermes_ros.launch.py
```

## Keyboard Teleop (No Gloves)

Use this path to test command sending before glove hardware is ready:

```bash
ros2 launch hermes_control hermes_keyboard_teleop.launch.py
```

This launch starts:
- `swarm_control_node` (builds and publishes `/hermes/swarm_intent`)
- `keyboard_teleop_node` (publishes synthetic packets to `/hermes/command_packets`)

Keyboard controls:
- Global:
  - `1` DRIVE mode
  - `2` FORMATION mode
  - `3` PARAMS mode
  - `m` deadman on/off
  - `g` select all robots (`r1`-`r6`)
  - `space` send zero drive command
  - `h` help
- DRIVE mode:
  - `w/s` forward/reverse
  - `a/d` yaw left/right
  - `q/e` strafe left/right
  - `x` zero cmd_vel
- FORMATION mode:
  - `l` LINE
  - `c` COLUMN
  - `w` WEDGE
  - `b` break formation
- PARAMS mode:
  - `w/s` speed level +/-
  - `e/d` spacing level +/-
  - `r/f` aggression level +/-

## Decentralized Option (DDS Intent Broadcast)

This mode keeps the command intent centralized, but target computation + control local on each robot:

1. Operator side publishes high-level `/hermes/swarm_intent`.
2. Every robot publishes its own pose beacon to `/hermes/robot_state_beacon` in a shared frame (default `map`).
3. Every robot runs `decentralized_robot_agent_node`, computes the same target map locally, uses distributed slot bidding (`/hermes/slot_bids`) for dynamic slot assignment, and applies only its own target to `/cmd_vel`.

### Operator side (once)

```bash
ros2 launch hermes_control hermes_ros.launch.py
```

### Each robot side (one instance per robot)

```bash
ros2 launch hermes_control robot_agent.launch.py robot_id:=r3 odom_topic:=/r3/odom cmd_vel_topic:=/r3/cmd_vel
```

For multi-robot, run one `robot_agent.launch.py` per robot with that robot's IDs/topics.

Important:
- Robots must share one consistent frame (`map` recommended).
- `robot_state_beacon_node` now uses TF lookup `map -> base_link` by default.
- If TF is not ready yet, you can temporarily set `use_tf_pose:=false fallback_to_odom:=true` (not swarm-accurate across robots).
- If multiple physical ROSbots share one DDS graph, a global `/cmd_vel` can be heard by every base controller. Use per-robot namespaced command topics or a robot-local command bridge before treating multi-robot `/cmd_vel` execution as isolated.

Validation commands:

```bash
ros2 run tf2_ros tf2_echo map r3/base_link
ros2 topic echo /hermes/robot_state_beacon
ros2 topic echo /hermes/swarm_intent
```

## Multi-ROSbot Test Procedure

1. On operator machine (laptop/RPi with keyboard), run:

```bash
ros2 launch hermes_control hermes_keyboard_teleop.launch.py
```

2. On each ROSbot, run your base robot stack first (the stack that provides odometry and TF).
   - Required outputs:
     - odometry topic (for example `/r3/odom`)
     - transform `map -> <robot_base_frame>` (recommended), or set fallback options below.

3. On each ROSbot, run this package's robot-side nodes:

```bash
ros2 launch hermes_control robot_agent.launch.py \
  robot_id:=r3 \
  odom_topic:=/r3/odom \
  cmd_vel_topic:=/r3/cmd_vel \
  global_frame:=map \
  base_frame:=r3/base_link \
  use_tf_pose:=true \
  fallback_to_odom:=false
```

If your robot controller expects `geometry_msgs/TwistStamped` on `/cmd_vel` (common on some ROSbot setups), enable stamped output:

```bash
ros2 launch hermes_control robot_agent.launch.py \
  robot_id:=r3 \
  odom_topic:=/r3/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link \
  global_frame:=odom \
  base_frame:=base_link \
  use_tf_pose:=false \
  fallback_to_odom:=true
```

4. Repeat step 3 with per-robot IDs/topics for every robot in the swarm.

5. Verify command delivery:
   - On any robot: `ros2 topic echo /hermes/swarm_intent`
   - On any robot: `ros2 topic echo /hermes/slot_bids`
   - On that robot: `ros2 topic echo /r3/cmd_vel`

If TF global pose is not available yet, temporarily use:
- `use_tf_pose:=false`
- `fallback_to_odom:=true`
This is useful for bring-up only and is less accurate for multi-robot relative geometry.

## Topics

### Inputs

- `/hermes/raw_input` (`std_msgs/String`): JSON object with raw glove/flex/fsr/imu sample.
- `/hermes/centroid` (`geometry_msgs/Point`): optional centroid for formation targeting.

### Outputs

- `/hermes/command_packets` (`std_msgs/String`): JSON command packets emitted by gesture pipeline.
- `/hermes/gesture_state` (`std_msgs/String`): JSON snapshot of gesture state.
- `/hermes/swarm_state` (`std_msgs/String`): JSON snapshot of swarm + mirrored gesture state.
- `/hermes/swarm_intent` (`std_msgs/String`): JSON swarm intent used by decentralized robot agents.
- `/hermes/robot_state_beacon` (`std_msgs/String`): per-robot JSON pose beacons.
- `/hermes/slot_bids` (`std_msgs/String`): per-robot slot auction bids for dynamic reassignment.

## JSON Contracts

### `/hermes/swarm_intent`

```json
{
  "schema": "hermes.swarm_intent.v1",
  "seq": 1,
  "mode": "FORMATION",
  "deadman_active": true,
  "paused": false,
  "selection": ["r1", "r2"],
  "active_formation_type": "WEDGE",
  "formation_heading": 0.4,
  "formation_spacing": 1.0,
  "active_behavior": null,
  "behavior_params": {},
  "home_xy": {"x": 0.0, "y": 0.0},
  "path_waypoints": [],
  "drive_cmd_vel": {}
}
```

### `/hermes/robot_state_beacon`

```json
{
  "schema": "hermes.robot_state_beacon.v1",
  "robot_id": "r3",
  "frame_id": "map",
  "x": 1.2,
  "y": -0.4,
  "yaw": 0.15,
  "vx": 0.10,
  "vy": 0.00
}
```

### `/hermes/slot_bids`

```json
{
  "schema": "hermes.slot_bids.v1",
  "intent_seq": 42,
  "robot_id": "r3",
  "selection": ["r1", "r2", "r3"],
  "costs": {
    "slot_000": 1.2,
    "slot_001": 0.4,
    "slot_002": 2.0
  }
}
```

## Current Defaults

- `robot_ids` default to `r1`-`r6`.
- Selection groups are currently `A`-`G`.

## Example Raw Input

```json
{
  "time_ms": 1730000000000,
  "flex": {
    "L": {"index": 0.1, "middle": 0.2, "ring": 0.2, "pinky": 0.2},
    "R": {"index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}
  },
  "fsr_pressed": {
    "L": {"INDEX": false, "MIDDLE": false, "RING": false, "PINKY": false},
    "R": {"INDEX": false, "MIDDLE": false, "RING": false, "PINKY": false}
  },
  "imu": {
    "R": {"PITCH": 0.2, "ROLL": 0.1, "YAW": -0.1, "AX": 0.1, "AY": -0.2, "AZ": -1.1},
    "L": {"AX": 0.0, "AY": 0.0, "AZ": -1.0}
  }
}
```
