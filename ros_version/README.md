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

## Decentralized Option (DDS Intent Broadcast)

This mode keeps the command intent centralized, but target computation + control local on each robot:

1. Operator side publishes high-level `/hermes/swarm_intent`.
2. Every robot publishes its own pose beacon to `/hermes/robot_state_beacon`.
3. Every robot runs `decentralized_robot_agent_node`, computes the same target map locally, and applies only its own target to `/cmd_vel`.

### Operator side (once)

```bash
ros2 launch hermes_control hermes_ros.launch.py
```

### Each robot side (one instance per robot)

```bash
ros2 launch hermes_control robot_agent.launch.py robot_id:=r3 odom_topic:=/r3/odom cmd_vel_topic:=/r3/cmd_vel
```

For multi-robot, run one `robot_agent.launch.py` per robot with that robot's IDs/topics.

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
  "x": 1.2,
  "y": -0.4,
  "yaw": 0.15
}
```

## Current Defaults

- `robot_ids` default to `r1`-`r8`.
- Selection groups are currently `A`-`G`.

## Example Raw Input

```json
{
  "time_ms": 1730000000000,
  "flex": {
    "L": {"index": 0.1, "middle": 0.2, "ring": 0.2, "pinky": 0.2},
    "R": {"index": 0.7, "middle": 0.7, "ring": 0.7, "pinky": 0.7}
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
