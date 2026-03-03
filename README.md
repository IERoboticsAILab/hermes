# H.E.R.M.E.S Project

H.E.R.M.E.S (Human-Encoded Recognition and Motion for Embodied Swarms) is a glove-driven swarm control stack.
It converts wearable sensor streams into high-level swarm intent, then executes that intent in either:

- a non-ROS ESP-NOW testbed path, or
- a ROS2 package path with decentralized robot-side execution.

## Repository Overview

| Area | Path | Purpose |
|---|---|---|
| Core gesture + swarm logic | `gestures/`, `swarm/`, `main.py` | Pure-Python command and state pipeline |
| Hardware + transmission testbed | `espnow_testbed/` | ESP32 glove firmware, ESP-NOW hub, Raspberry Pi gateway |
| ROS2 packaged version | `ros_version/` | ROS2 nodes/launch files duplicating current logic |
| Tests | `tests/` | Formation, behavior, and safety/gateway unit tests |

## End-to-End Data Flow

1. Left and right glove ESP32 boards sample sensors and create JSON payloads.
2. Both gloves send payloads over ESP-NOW to a hub ESP32.
3. Hub ESP32 forwards validated packets over USB serial to a Raspberry Pi.
4. Pi gateway fuses both gloves into one raw input sample.
5. Gesture/safety/swarm logic produces command packets and swarm intent.
6. Output is either:
   - printed/UDP commands (testbed), or
   - ROS2 topics consumed by robot agents.

## Quick Start Paths

### Path A: ESP-NOW + Raspberry Pi Testbed (No ROS)

Use this first to validate transmission and command logic.

1. Flash firmware in `espnow_testbed/firmware/`:
   - `glove_left/glove_left.ino`
   - `glove_right/glove_right.ino`
   - `hub_master/hub_master.ino`
2. Insert MAC addresses in firmware placeholders.
3. Configure Pi gateway:
   - `espnow_testbed/pi_gateway/config.example.json`
4. Install gateway dependency and run:

```bash
cd espnow_testbed/pi_gateway
python3 -m pip install -r requirements.txt
python3 hermes_gateway.py --config config.example.json
```

Details: [espnow_testbed/README.md](./espnow_testbed/README.md)

### Path B: ROS2 Package

Use this when you want ROS topics/launch integration and decentralized robot-side execution.

```bash
cd ros_version
colcon build
source install/setup.bash
ros2 launch hermes_control hermes_ros.launch.py
```

Per-robot local executor launch example:

```bash
ros2 launch hermes_control robot_agent.launch.py robot_id:=r3 odom_topic:=/r3/odom cmd_vel_topic:=/r3/cmd_vel
```

Details: [ros_version/README.md](./ros_version/README.md)

## Important Configuration Points

- ESP-NOW MAC placeholders:
  - `espnow_testbed/firmware/glove_left/glove_left.ino`
  - `espnow_testbed/firmware/glove_right/glove_right.ino`
  - `espnow_testbed/firmware/hub_master/hub_master.ino`
- Pi gateway runtime options:
  - `espnow_testbed/pi_gateway/config.example.json`
  - `command_output.mode`: `print` or `udp`
  - `command_output.udp_host` / `udp_port` for network output
- Default robot IDs in current logic: `r1` to `r8`
- Default selection group slots: `A` to `G`

## Development and Validation

### Compile ESP32 sketches (arduino-cli example)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/glove_left
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/glove_right
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/hub_master
```

### Python syntax check

```bash
python3 -m py_compile main.py
python3 -m py_compile gestures/*.py swarm/*.py
python3 -m py_compile espnow_testbed/pi_gateway/hermes_gateway.py
```

### Unit tests

```bash
python3 -m pip install pytest
python3 -m pytest -q
```

## Notes on Current Implementation

- Left glove firmware is configured without left-hand FSR inputs.
- Right glove firmware carries FSR inputs and IMU stream.
- ROS2 implementation includes decentralized slot bidding (`/hermes/slot_bids`) and robot-local tracking control.
- ROS1 is not the active implementation in this repository.

## License

MIT (see [LICENSE](./LICENSE)).
