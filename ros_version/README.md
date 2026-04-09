# H.E.R.M.E.S ROS 2 Package

This directory contains the ROS 2 packaging of the H.E.R.M.E.S control stack.
It includes:

- glove/vest serial ingest on the Raspberry Pi
- gesture recognition and safety logic
- swarm state and swarm intent generation
- decentralized robot-side execution
- OptiTrack/NatNet integration
- robot haptic status aggregation
- vest haptic feedback generation

The package name is:
- `hermes_control`

## Package Layout

| Path | Purpose |
|---|---|
| `src/hermes_control/hermes_control/gesture_pipeline_node.py` | `/hermes/raw_input` -> `/hermes/command_packets` + `/hermes/gesture_state` |
| `src/hermes_control/hermes_control/swarm_control_node.py` | `/hermes/command_packets` -> `/hermes/swarm_state` + `/hermes/swarm_intent` |
| `src/hermes_control/hermes_control/keyboard_teleop_node.py` | keyboard-only synthetic command packet source |
| `src/hermes_control/hermes_control/decentralized_robot_agent_node.py` | robot-local agent that consumes swarm intent and robot states, then outputs `/cmd_vel` |
| `src/hermes_control/hermes_control/robot_state_beacon_node.py` | robot odometry / TF -> `/hermes/robot_state_beacon` |
| `src/hermes_control/hermes_control/optitrack_pose_beacon_node.py` | NatNet pose topics -> `/hermes/robot_state_beacon` |
| `src/hermes_control/hermes_control/robot_haptic_status_node.py` | robot-local obstacle / range / diagnostics -> `/hermes/robot_haptic_status` |
| `src/hermes_control/hermes_control/haptic_vest_node.py` | swarm + robot haptic state -> vest serial frames |
| `src/hermes_control/hermes_control/vest_serial_bridge_node.py` | vest USB serial -> `/hermes/raw_input` and `/hermes/vest_hub_rx`; `/hermes/vest_serial_tx` -> vest USB serial |
| `launch/` | packaged launch entry points |
| `config/` | operator, haptic, and OptiTrack YAML configs |
| `firmware/esp32_haptic_vest/` | final combined vest ESP32 firmware |

## Launch Files

| Launch file | Starts |
|---|---|
| `hermes_ros.launch.py` | `gesture_pipeline_node` + `swarm_control_node` |
| `hermes_keyboard_teleop.launch.py` | `swarm_control_node` + `keyboard_teleop_node` |
| `hermes_decentralized.launch.py` | gesture pipeline + swarm control + one local robot beacon + one local robot agent |
| `robot_agent.launch.py` | one `robot_state_beacon_node` + one `decentralized_robot_agent_node` |
| `optitrack_version1_pi.launch.py` | one NatNet client + one OptiTrack beacon bridge |
| `robot_agent_optitrack_version1.launch.py` | one robot agent consuming the shared OptiTrack beacons |
| `robot_agent_optitrack_version2.launch.py` | one NatNet client + one OptiTrack beacon bridge + one robot agent |
| `robot_haptic_status.launch.py` | one `robot_haptic_status_node` |
| `haptic_vest.launch.py` | one `haptic_vest_node` |
| `wearables_pi.launch.py` | `vest_serial_bridge_node` + `gesture_pipeline_node` + `swarm_control_node` + `haptic_vest_node` |

## Build

From the repository root:

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

## Runtime Dependencies

### ROS dependencies

The package manifest currently expects these ROS-side dependencies at runtime:

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `diagnostic_msgs`
- `tf2_ros`
- `python3-yaml`
- `natnet_ros2`

### Important note about `natnet_ros2`

`natnet_ros2` is **not vendored inside this repository**.

If you use either OptiTrack launch path, you must have `natnet_ros2` built and sourced in the environment before launching H.E.R.M.E.S.

Typical pattern:

```bash
cd ~/Desktop/optitrack_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select natnet_ros2
source install/setup.bash

cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source ~/Desktop/optitrack_ws/install/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

## Current Machine Roles

### Raspberry Pi 5

Current recommended role:
- owns the vest USB serial port
- runs wearable serial ingest
- runs gesture recognition and swarm control
- runs haptic vest generation
- optionally runs keyboard fallback

### ROSbots

Current recommended role:
- run one robot-local decentralized agent each
- publish or consume robot state beacons
- in OptiTrack Version 2, also run one NatNet client each

## Current Wearable Data Flow

The active ROS wearable path is:

1. left/right gloves -> vest ESP32 over ESP-NOW
2. vest ESP32 -> Raspberry Pi over USB serial
3. `vest_serial_bridge_node` -> `/hermes/vest_hub_rx` + `/hermes/raw_input`
4. `gesture_pipeline_node` -> `/hermes/command_packets` + `/hermes/gesture_state`
5. `swarm_control_node` -> `/hermes/swarm_state` + `/hermes/swarm_intent`
6. `haptic_vest_node` -> `/hermes/vest_serial_tx`
7. `vest_serial_bridge_node` writes that serial frame back to the vest ESP32

## Current Wearable Assumptions

These are the current code assumptions, not future plans:

- left glove supplies flex posture sensing
- left glove supplies the deadman IMU
- left glove shake triggers `ESTOP`
- the active control IMU is taken from `imu.R` if present, otherwise `imu.L`
- the current right-glove firmware does **not** send IMU, so the left glove is the active control IMU in practice
- the current right-glove firmware sends FSR data only
- the current vest serial bridge requires **both gloves** to be fresh before publishing a usable `/hermes/raw_input`
- if either glove goes stale, the bridge publishes an empty raw sample so the gesture pipeline fails safe with deadman off

## Gesture / Control Contract

### Left-hand posture selects the mode

| Left posture | Mode |
|---|---|
| `OPEN` | `DRIVE` |
| `POINT` | `SELECTION` |
| `FIST` | `FORMATION` |
| `TWO` | `BEHAVIOR` |
| `THREE` | `PARAMS` |

### Global safety commands

- left palm-up -> deadman off
- left shake -> `ESTOP`
- left `OPEN` + right middle tap -> soft stop / pause
- left `OPEN` + right middle double tap -> resume

### Drive mode (`L = OPEN`)

- active control IMU `PITCH` -> `vx`
- active control IMU `ROLL` -> `vy_or_steer`
- active control IMU `YAW` -> `omega`
- right ring tap -> toggle yaw mode (`steer` vs `rotate_in_place` semantics)
- right ring hold -> precision drive while held

### Selection mode (`L = POINT`)

Robot bindings:
- right index tap -> `r1`
- right middle tap -> `r2`
- right ring tap -> `r3`
- right pinky tap -> `r4`
- right index hold -> `r5`
- right middle hold -> `r6`

Group slot bindings:
- right index tap -> `A`
- right middle tap -> `B`
- right ring tap -> `C`
- right pinky tap -> `D`
- right index double tap -> `E`
- right middle double tap -> `F`
- right ring double tap -> `G`

Selection workflow:
1. choose a group slot
2. tap/hold robot bindings to build membership
3. right pinky double tap to confirm group assignment

### Formation mode (`L = FIST`)

Formation bindings:
- right index tap -> `LINE`
- right middle tap -> `COLUMN`
- right ring tap -> `WEDGE`
- right pinky tap -> `CIRCLE`
- right index double tap -> `ECHELON_L`
- right middle double tap -> `ECHELON_R`
- right ring double tap -> `GRID`
- right pinky double tap -> `DIAMOND`

Additional formation controls:
- right index hold -> stream formation heading from active control IMU `YAW`
- right ring hold -> stream formation spacing from active control IMU `PITCH`
- right middle hold -> apply the pending formation
- right middle double tap -> break formation

### Behavior mode (`L = TWO`)

Behavior bindings:
- right index tap -> `PATROL`
- right index double tap -> `PATROL_PERIMETER`
- right middle tap -> `FOLLOW_PATH`
- right middle double tap -> `FOLLOW_PATH_LOOP`
- right ring tap -> `HOLD_ANCHOR`
- right pinky tap -> `RETURN_HOME`
- right ring hold -> `FOLLOW_ME_TOGGLE`
- right pinky hold -> `DISPERSE_SCAN`

### Params mode (`L = THREE`)

- right index tap -> speed down
- right middle tap -> speed up
- right index hold -> speed min
- right middle hold -> speed max
- right ring tap -> spacing down
- right pinky tap -> spacing up
- right ring hold -> spacing min
- right pinky hold -> spacing max
- right ring double tap -> aggression down
- right pinky double tap -> aggression up

Level-based params currently clamp to `1..4`.

## Keyboard-Only Operator Path

This is the easiest non-glove ROS path.

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hermes_control hermes_keyboard_teleop.launch.py
```

This launch starts:
- `swarm_control_node`
- `keyboard_teleop_node`

Keyboard mapping:
- global:
  - `1` drive mode
  - `2` formation mode
  - `3` params mode
  - `m` toggle deadman
  - `g` select all robots
  - `space` stop drive command
  - `h` help
- drive:
  - `w/s` forward/reverse
  - `a/d` yaw left/right
  - `q/e` strafe left/right
  - `x` zero velocity
- formation:
  - `l` line
  - `c` column
  - `w` wedge
  - `b` break formation
- params:
  - `w/s` speed +/-
  - `e/d` spacing +/-
  - `r/f` aggression +/-

## Wearables Operator Path (Recommended Pi Launch)

### Prerequisites

1. the vest ESP32 is flashed with `firmware/esp32_haptic_vest`
2. the Pi user has permission to open the serial device
3. `hermes_control` is built and sourced

If `/dev/ttyUSB0` gives `Permission denied`, add the user to `dialout` and re-login:

```bash
sudo usermod -a -G dialout $USER
```

### Run the wearable stack

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hermes_control wearables_pi.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600
```

Current defaults come from:
- `config/vest_serial_bridge_pi.yaml`
- `config/haptic_vest_pi.yaml`

### What `wearables_pi.launch.py` starts

- `vest_serial_bridge_node`
- `gesture_pipeline_node`
- `swarm_control_node`
- `haptic_vest_node`

### Important concurrency rule

Do **not** also launch `hermes_keyboard_teleop.launch.py` at the same time, because that would start a second `swarm_control_node`.

If you want keyboard fallback while the wearable stack is running, launch only:

```bash
ros2 run hermes_control keyboard_teleop_node --ros-args \
  -p robot_ids:="['r1','r2','r3','r4','r5','r6']"
```

## Haptic Vest Only

If you want to run only the vest haptic node:

```bash
ros2 launch hermes_control haptic_vest.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600
```

Current defaults:
- direct serial output is enabled by default in this launch
- the serial frame topic is `/hermes/vest_serial_tx`

## Per-Robot Haptic Status Node

Run one of these on each robot if you want obstacle/error feedback to feed the vest:

```bash
ros2 launch hermes_control robot_haptic_status.launch.py \
  robot_id:=r1 \
  status_config:=/absolute/path/to/robot_haptic_status_r1.yaml
```

Current config files:
- `config/robot_haptic_status_r1.yaml`
- `config/robot_haptic_status_r2.yaml`
- `config/robot_haptic_status_r3.yaml`
- `config/robot_haptic_status_r4.yaml`
- `config/robot_haptic_status_r5.yaml`
- `config/robot_haptic_status_r6.yaml`
- `config/robot_haptic_status_defaults.yaml`

What the node publishes:
- `/hermes/robot_haptic_status`

Current status sources supported by the node:
- front arc LaserScan risk
- front / rear Range topics
- diagnostics-based error state
- stale sensor / stale diagnostics conditions

## Basic ROS Gesture + Swarm Pipeline Without Wearables

If you already have another node publishing `/hermes/raw_input`, or if you want the minimal ROS pipeline only:

```bash
ros2 launch hermes_control hermes_ros.launch.py
```

This launch starts only:
- `gesture_pipeline_node`
- `swarm_control_node`

## Single-Machine Decentralized Demo

For a single-machine demonstration path:

```bash
ros2 launch hermes_control hermes_decentralized.launch.py \
  robot_id:=r1 \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel
```

This launch starts:
- gesture pipeline
- swarm control
- one robot state beacon
- one decentralized robot agent

## Robot Agent Without OptiTrack

Use this when you want the robot-local agent with TF or odometry-based beaconing instead of NatNet.

```bash
ros2 launch hermes_control robot_agent.launch.py \
  robot_id:=r1 \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  global_frame:=map \
  base_frame:=base_link \
  use_tf_pose:=true \
  fallback_to_odom:=false
```

If your robot base expects `geometry_msgs/TwistStamped` on `/cmd_vel`, enable stamped output:

```bash
ros2 launch hermes_control robot_agent.launch.py \
  robot_id:=r1 \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link \
  global_frame:=odom \
  base_frame:=base_link \
  use_tf_pose:=false \
  fallback_to_odom:=true
```

## OptiTrack / NatNet Integration

There are two supported architectures.

### Version 1: one NatNet client on the operator side

Recommended when you want a single NatNet connection and shared OptiTrack beacons.

#### Pi / operator machine

```bash
ros2 launch hermes_control optitrack_version1_pi.launch.py \
  serverIP:=<MOTIVE_PC_IP> \
  clientIP:=<PI_IP> \
  serverType:=unicast \
  optitrack_config:=/absolute/path/to/optitrack_version1_placeholders.yaml
```

What this launch starts:
- NatNet client from `natnet_ros2`
- one `optitrack_pose_beacon_node`

#### Each robot

```bash
ros2 launch hermes_control robot_agent_optitrack_version1.launch.py \
  robot_id:=r1 \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link \
  expected_state_frame:=optitrack
```

### Version 2: one NatNet client per ROSbot

This is the architecture currently used in your lab bring-up.

Each ROSbot launches NatNet locally, converts its rigid-body pose to a beacon, and runs its local robot agent.

Example:

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r1.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

What this launch starts:
- NatNet client from `natnet_ros2`
- `optitrack_pose_beacon_node`
- `decentralized_robot_agent_node`

### OptiTrack config files

Current packaged configs:
- `config/optitrack_r1.yaml`
- `config/optitrack_r2.yaml`
- `config/optitrack_r3.yaml`
- `config/optitrack_r4.yaml`
- `config/optitrack_r5.yaml`
- `config/optitrack_r6.yaml`
- `config/optitrack_version1_placeholders.yaml`
- `config/optitrack_version2_single_robot_placeholder.yaml`

Current state of those configs:
- `r1` through `r4` contain live lab values committed in the repo
- `r5` and `r6` still contain placeholder-style values and should be edited before use

### Current OptiTrack coordinate handling

`optitrack_pose_beacon_node` supports:
- `frame_id`
- `planar_x_axis`
- `planar_y_axis`
- `forward_axis`

The current per-robot configs use:
- `planar_x_axis: x`
- `planar_y_axis: z`
- `forward_axis: x`

That means the current lab Motive setup is treated as an `x/z` ground plane with the rigid body’s local `x` axis as robot forward.

## Topics

### Wearables / serial topics

- `/hermes/vest_hub_rx` (`std_msgs/String`)
  - every decoded JSON line received from the vest serial stream
- `/hermes/raw_input` (`std_msgs/String`)
  - fused glove input consumed by the gesture pipeline
- `/hermes/vest_serial_tx` (`std_msgs/String`)
  - vest motor serial frames generated by the haptic node
- `/hermes/vest_serial_state` (`std_msgs/String`)
  - bridge debug / freshness summary
- `/hermes/haptic_vest_state` (`std_msgs/String`)
  - haptic output debug summary

### Gesture / swarm topics

- `/hermes/command_packets` (`std_msgs/String`)
- `/hermes/gesture_state` (`std_msgs/String`)
- `/hermes/swarm_state` (`std_msgs/String`)
- `/hermes/swarm_intent` (`std_msgs/String`)
- `/hermes/centroid` (`geometry_msgs/Point`)

### Robot-side topics

- `/hermes/robot_state_beacon` (`std_msgs/String`)
- `/hermes/slot_bids` (`std_msgs/String`)
- `/hermes/robot_haptic_status` (`std_msgs/String`)

## Current JSON Contracts

### `/hermes/raw_input`

Current wearable bridge output shape when both gloves are fresh:

```json
{
  "time_ms": 1730000000000,
  "flex": {
    "L": {"index": 0.10, "middle": 0.20, "ring": 0.80, "pinky": 0.75},
    "R": {"index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}
  },
  "fsr_pressed": {
    "L": {"INDEX": false, "MIDDLE": false, "RING": false, "PINKY": false},
    "R": {"INDEX": false, "MIDDLE": true, "RING": false, "PINKY": false}
  },
  "imu": {
    "L": {"PITCH": 0.2, "ROLL": 0.1, "YAW": -0.1, "AX": 0.1, "AY": -0.2, "AZ": -1.1}
  }
}
```

When a glove stream is stale, the bridge intentionally publishes the fail-safe empty shape:

```json
{
  "time_ms": 1730000000000,
  "flex": {},
  "fsr_pressed": {},
  "imu": {}
}
```

### `/hermes/swarm_intent`

```json
{
  "type": "SWARM_INTENT",
  "schema": "hermes.swarm_intent.v1",
  "seq": 1,
  "stamp_ms": 1730000000000,
  "mode": "FORMATION",
  "deadman_active": true,
  "paused": false,
  "selection": ["r1", "r2"],
  "robot_ids": ["r1", "r2", "r3", "r4", "r5", "r6"],
  "centroid": {"x": 0.0, "y": 0.0},
  "active_formation_type": "WEDGE",
  "formation_heading": 0.4,
  "formation_spacing": 1.0,
  "active_behavior": null,
  "behavior_params": {},
  "home_xy": {"x": 0.0, "y": 0.0},
  "path_waypoints": [],
  "drive_cmd_vel": {},
  "groups": {"A": ["r1", "r2"], "B": [], "C": [], "D": [], "E": [], "F": [], "G": []}
}
```

### `/hermes/robot_state_beacon`

```json
{
  "schema": "hermes.robot_state_beacon.v1",
  "stamp_ms": 1730000000000,
  "robot_id": "r3",
  "frame_id": "optitrack",
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

### `/hermes/robot_haptic_status`

```json
{
  "schema": "hermes.robot_haptic_status.v1",
  "stamp_ms": 1730000000000,
  "robot_id": "r1",
  "obstacle": true,
  "obstacle_level": 0.62,
  "front_scan_min_m": 0.41,
  "front_range_min_m": 0.19,
  "rear_range_min_m": null,
  "error": false,
  "error_flags": {
    "scan_missing": false,
    "front_ranges_missing": false,
    "rear_ranges_missing": true,
    "diag_error": false,
    "diag_stale": false
  }
}
```

## Debugging / Introspection Commands

### On the Pi

```bash
ros2 topic echo /hermes/vest_hub_rx --once
ros2 topic echo /hermes/raw_input --once
ros2 topic echo /hermes/gesture_state
ros2 topic echo /hermes/command_packets
ros2 topic echo /hermes/swarm_state --once
ros2 topic echo /hermes/swarm_intent --once
ros2 topic echo /hermes/vest_serial_state --once
ros2 topic echo /hermes/haptic_vest_state --once
```

### On a ROSbot

```bash
ros2 topic echo /hermes/robot_state_beacon --once
ros2 topic echo /hermes/swarm_intent --once
ros2 topic echo /hermes/slot_bids --once
ros2 topic echo /cmd_vel
```

### For NatNet / rigid-body topics

```bash
ros2 topic list | grep pose
ros2 topic echo /<rigid_body_name>/pose --once
```

## Important Caveats

- `natnet_ros2` must be sourced before the OptiTrack launch files can work.
- `wearables_pi.launch.py` is the recommended Pi bring-up when using the vest serial path.
- `hermes_ros.launch.py` does **not** start the vest serial bridge or the haptic node.
- `hermes_keyboard_teleop.launch.py` starts its own `swarm_control_node`; do not run it alongside `wearables_pi.launch.py`.
- If multiple physical robots share one DDS graph and one shared `/cmd_vel`, command isolation is not guaranteed.
- Current formation convergence quality still depends on good OptiTrack calibration, correct rigid-body forward axes, and robot-side motion/controller health.
