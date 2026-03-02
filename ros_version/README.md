# H.E.R.M.E.S ROS2 Package (Duplicated Version)

This folder contains a ROS2-packaged copy of the current H.E.R.M.E.S software.
The original code at the repo root is unchanged.

## Layout

- `ros_version/src/hermes_control/`: ROS2 Python package
- `hermes_control/gestures/`: duplicated gesture pipeline modules
- `hermes_control/swarm/`: duplicated swarm modules
- `hermes_control/gesture_pipeline_node.py`: raw JSON -> command packet publisher
- `hermes_control/swarm_control_node.py`: command packet consumer -> swarm state publisher
- `launch/hermes_ros.launch.py`: launches both nodes

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

## Topics

### Inputs

- `/hermes/raw_input` (`std_msgs/String`): JSON object with raw glove/flex/fsr/imu sample.
- `/hermes/centroid` (`geometry_msgs/Point`): optional centroid for formation targeting.

### Outputs

- `/hermes/command_packets` (`std_msgs/String`): JSON command packets emitted by gesture pipeline.
- `/hermes/gesture_state` (`std_msgs/String`): JSON snapshot of gesture state.
- `/hermes/swarm_state` (`std_msgs/String`): JSON snapshot of swarm + mirrored gesture state.

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
