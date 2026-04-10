# H.E.R.M.E.S Project

H.E.R.M.E.S stands for **Human-Encoded Recognition and Motion for Embodied Swarms**.
This repository contains the wearable input stack, swarm intent logic, decentralized ROS 2 robot execution, OptiTrack integration, and haptic vest feedback used to control a multi-robot swarm.

This README is the high-level map of the project. The detailed subsystem docs live in:

- [`espnow_testbed/README.md`](./espnow_testbed/README.md): glove / vest ESP32 firmware, MAC setup, legacy non-ROS gateway, diagnostics
- [`ros_version/README.md`](./ros_version/README.md): ROS 2 package, launch files, OptiTrack bring-up, wearables stack, topics/contracts
- [`docs/LAB_BRINGUP_CHECKLIST.md`](./docs/LAB_BRINGUP_CHECKLIST.md): one-page lab bring-up sequence
- [`docs/MACHINE_COMMANDS.md`](./docs/MACHINE_COMMANDS.md): machine-by-machine command sheet

## Current Implemented System

### Wearable architecture

The current codebase is built around this live hardware/software split:

1. **Left glove ESP32**
   - Reads **flex sensors** and a **MPU6050 IMU**
   - Sends packets over **ESP-NOW**
   - Owns:
     - left-hand posture / mode selection
     - deadman IMU
     - left-glove shake `ESTOP`
     - the active control IMU stream

2. **Right glove ESP32**
   - Reads **FSR sensors** only in the current configuration
   - Sends packets over **ESP-NOW**
   - Owns the discrete finger commands used for selection, formation, behaviors, and params
   - The right-glove IMU code still exists, but it is currently disabled in firmware

3. **Vest ESP32**
   - Receives left/right glove packets over **ESP-NOW**
   - Forwards validated glove packets to the **Raspberry Pi 5** over **USB serial**
   - Receives haptic motor commands back from the Pi over the same serial link
   - Drives the vest’s 6 haptic motors

4. **Raspberry Pi 5**
   - Reads vest serial traffic
   - Publishes ROS `/hermes/raw_input`
   - Runs gesture recognition, safety logic, swarm control, and haptic feedback generation

5. **ROSbots**
   - Run decentralized robot agents
   - Consume `/hermes/swarm_intent`
   - Consume shared robot pose beacons
   - Publish robot-local `/cmd_vel`

6. **OptiTrack / NatNet**
   - Supplies the shared world-frame robot poses used for multi-robot formation control
   - Supported in two architectures:
     - **Version 1**: one NatNet client on the Pi / operator side, shared beacon output
     - **Version 2**: one NatNet client per ROSbot

## Repository Layout

| Path | Purpose |
|---|---|
| `main.py` | Pure-Python non-ROS loop entry point using the root gesture/swarm modules |
| `gestures/` | Root gesture recognition, safety, posture, FSR, and matching logic |
| `swarm/` | Root formation, behavior, and swarm controller logic |
| `espnow_testbed/` | ESP32 firmware plus the legacy standalone Raspberry Pi gateway path |
| `ros_version/` | ROS 2 package, launch files, config, vest firmware, and robot-side execution |
| `tests/` | Unit tests for behavior engine, formation engine, safety, and gateway/raw-input handling |

## What The Project Supports Right Now

### Gesture / control modes

The left glove posture selects the active mode:

- `OPEN` -> `DRIVE`
- `POINT` -> `SELECTION`
- `FIST` -> `FORMATION`
- `TWO` -> `BEHAVIOR`
- `THREE` -> `PARAMS`

### Safety commands

Global safety handling is always active:

- Left glove palm-up -> deadman off / motion gated off
- Left glove shake -> `ESTOP`
- Left `OPEN` + right middle tap -> soft stop / pause
- Left `OPEN` + right middle double tap -> resume

### Formation types

Supported formation names in the code:

- `LINE`
- `COLUMN`
- `WEDGE`
- `CIRCLE`
- `ECHELON_L`
- `ECHELON_R`
- `GRID`
- `DIAMOND`

### Behavior types

Supported behavior names in the code:

- `PATROL`
- `PATROL_PERIMETER`
- `FOLLOW_PATH`
- `FOLLOW_PATH_LOOP`
- `HOLD_ANCHOR`
- `RETURN_HOME`
- `FOLLOW_ME_TOGGLE`
- `DISPERSE_SCAN`

### Haptic feedback features

The current ROS haptic stack supports:

- gesture acknowledgement pulse
- formation reached pulse
- obstacle cues from robot status topics
- robot error / stale-status cues
- swarm density cues (`dense` / `sparse`)
- 6-motor serial output frames to the vest ESP32

## Recommended Entry Paths

### 1. Firmware and transport debugging first

Use the ESP32 firmware and serial diagnostics first if you are validating MAC addresses, ESP-NOW, FSR/flex readings, or vest serial transport.

Start here:
- [`espnow_testbed/README.md`](./espnow_testbed/README.md)

### 2. Wearables + ROS operator stack

Use the Pi-side ROS launch when you want the full glove -> ROS -> haptic loop:

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash

ros2 launch hermes_control wearables_pi.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600
```

What this launch starts:

- `vest_serial_bridge_node`
- `gesture_pipeline_node`
- `swarm_control_node`
- `haptic_vest_node`

Use [`ros_version/README.md`](./ros_version/README.md) for the full operator-side details.

### 3. Full ROS swarm with OptiTrack

Use the ROS documentation when you want the decentralized robot-side stack plus OptiTrack/NatNet:

- [`ros_version/README.md`](./ros_version/README.md)

That doc covers:

- `hermes_ros.launch.py`
- `hermes_keyboard_teleop.launch.py`
- `robot_agent.launch.py`
- `robot_agent_optitrack_version1.launch.py`
- `robot_agent_optitrack_version2.launch.py`
- `optitrack_version1_pi.launch.py`
- `robot_haptic_status.launch.py`
- `haptic_vest.launch.py`

## Current Hardware / Sensor Ownership

| Device | Sensors / I/O | Current role |
|---|---|---|
| Left glove ESP32 | Flex + MPU6050 | Mode selection, deadman, left-shake `ESTOP`, active control IMU |
| Right glove ESP32 | FSR only | Discrete gesture commands |
| Vest ESP32 | ESP-NOW RX + USB serial + 6 motor outputs | Combined glove receiver and haptic actuator controller |
| Raspberry Pi 5 | USB serial + ROS 2 | Serial bridge, gesture pipeline, swarm control, haptic generation |
| ROSbot | Odometry / base control / NatNet client or beacon subscriber | Robot-local target tracking and motion execution |

## Root Python Path vs ROS Path

There are **two copies** of the gesture/swarm logic in this repository:

- root Python modules: `gestures/`, `swarm/`, `main.py`
- ROS package copies: `ros_version/src/hermes_control/hermes_control/...`

The project keeps these aligned conceptually, but they are still duplicated code trees.

- The standalone gateway in `espnow_testbed/pi_gateway/hermes_gateway.py` uses the **root** modules.
- The ROS nodes under `ros_version/src/hermes_control/hermes_control/` use the **ROS package** copies.

## Development Commands

### Run tests

From the repository root:

```bash
pytest -q
```

### Compile firmware sketches

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/get_mac
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/glove_left
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/glove_right
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/hub_master
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/fsr_raw_test
arduino-cli compile --fqbn esp32:esp32:esp32 ros_version/firmware/esp32_haptic_vest
```

### Build ROS package

```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

## Important Project Caveats

- The **current recommended wearable path** uses the vest ESP32 as the ESP-NOW receiver and serial hub. The legacy `hub_master` firmware remains available for isolated non-ROS testing only.
- The current ROS vest serial bridge publishes usable `/hermes/raw_input` only when **both gloves are fresh**. If a glove stream goes stale, it publishes an empty raw sample so the pipeline fails safe with deadman off.
- `wearables_pi.launch.py` already starts `swarm_control_node`. Do **not** launch `hermes_keyboard_teleop.launch.py` at the same time. If you want keyboard fallback while wearables are running, launch only `keyboard_teleop_node`.
- OptiTrack `Version 2` requires `natnet_ros2` to be available in the environment on every ROSbot running it.
- If multiple physical robots share one DDS graph and one non-namespaced `/cmd_vel`, command isolation is not guaranteed. Use per-robot topics or a robot-local bridge if needed.
- The repository currently includes real lab values for some MACs / OptiTrack configs and placeholder values for others. Treat committed MACs, IPs, and rigid body names as examples unless they match your current hardware.

## Subsystem Documentation

- Wearables / firmware / standalone gateway: [`espnow_testbed/README.md`](./espnow_testbed/README.md)
- ROS 2 package / OptiTrack / haptics / topics: [`ros_version/README.md`](./ros_version/README.md)

## License

MIT (see [LICENSE](./LICENSE)).
