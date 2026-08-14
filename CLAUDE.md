# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

H.E.R.M.E.S (**Human-Encoded Recognition and Motion for Embodied Swarms**) is a wearable-to-swarm control system. Two ESP32 gloves and an ESP32 haptic vest communicate over ESP-NOW; the vest forwards data to a Raspberry Pi 5 over USB serial; the Pi runs ROS 2 gesture recognition, swarm control, and haptic feedback; ROSbots execute the resulting swarm intents.

## Commands

### Tests
```bash
pytest -q
```
Run a single test file:
```bash
pytest tests/test_behavior_engine.py -q
```

### ROS 2 build (from repo root)
```bash
cd ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

### Firmware compile (from repo root)
```bash
export FQBN="esp32:esp32:esp32"
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_left
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_right
arduino-cli compile --fqbn "$FQBN" ros_version/firmware/esp32_haptic_vest
```

### Firmware upload
```bash
arduino-cli upload -p <PORT> --fqbn "$FQBN" --upload-property upload.speed=115200 <sketch_path>
```

### Serial monitor
```bash
arduino-cli monitor -p <PORT> -c baudrate=115200   # gloves
arduino-cli monitor -p <PORT> -c baudrate=921600   # vest
```

### Run the full wearable stack (on Pi)
```bash
cd ros_version && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch hermes_control wearables_pi.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
```

### Standalone gateway (non-ROS, for transport validation)
```bash
cd espnow_testbed/pi_gateway
python3 hermes_gateway.py --config config.example.json
```

## Architecture

### Hardware data flow
```
Left glove (flex + MPU6050)  ──┐
                               ├─ ESP-NOW ──► Vest ESP32 ──► USB serial (921600) ──► Raspberry Pi 5
Right glove (FSR only)       ──┘                │◄─────────────────────────────────── haptic frames
                                                 └─► 6 haptic motors
```

The Pi runs: `vest_serial_bridge_node` → `gesture_pipeline_node` → `swarm_control_node` → `haptic_vest_node`.

ROSbots subscribe to `/hermes/swarm_intent` and publish their pose via `/hermes/robot_state_beacon`. Formation slot assignment uses a distributed auction over `/hermes/slot_bids`.

### Two parallel code trees

The gesture and swarm logic exists in **two copies** that must be kept aligned:

| Usage | Path |
|---|---|
| Standalone gateway (`espnow_testbed/pi_gateway/hermes_gateway.py`) | `gestures/`, `swarm/` (repo root) |
| ROS nodes | `ros_version/src/hermes_control/hermes_control/gestures/`, `.../swarm/` |

Edits to gesture or swarm logic usually need to be applied in both trees.

### Key architectural constraints

- `vest_serial_bridge_node` requires **both gloves fresh** before publishing a usable `/hermes/raw_input`. A stale glove causes an empty fail-safe sample (deadman off).
- Do **not** launch `hermes_keyboard_teleop.launch.py` alongside `wearables_pi.launch.py` — both start `swarm_control_node`. For keyboard fallback while wearables run, use `ros2 run hermes_control keyboard_teleop_node` directly.
- Right-glove FSR pins must be **ADC1** pins to avoid the ADC2/Wi-Fi conflict when ESP-NOW is active.
- All ESP-NOW peers must share the same `ESPNOW_CHANNEL` (default `1`). MAC arrays in firmware (`HUB_MAC`, `LEFT_GLOVE_MAC`, `RIGHT_GLOVE_MAC`) are lab-specific and must be updated per board.

### OptiTrack

Two supported architectures — **Version 2 is current** (one NatNet client per ROSbot, requires `natnet_ros2` built and sourced on each robot). Version 1 (single NatNet client on the Pi side) is supported but not the active lab configuration.

`optitrack_r1.yaml` through `optitrack_r4.yaml` contain live lab values. `r5` and `r6` are placeholders.

### Legacy / test-only paths

- `espnow_testbed/firmware/hub_master/` — legacy ESP-NOW receiver; use the vest firmware instead.
- `ros_version/firmware/vest_motor_test/` — motor diagnostic only, not part of normal operation.
- `ros_version/src/hermes_control/hermes_control/legacy_main.py` — superseded entry point.

## ROS Topics Quick Reference

| Topic | Type | Description |
|---|---|---|
| `/hermes/raw_input` | `std_msgs/String` | Fused glove JSON into gesture pipeline |
| `/hermes/command_packets` | `std_msgs/String` | Gesture pipeline output |
| `/hermes/swarm_intent` | `std_msgs/String` | Swarm intent consumed by robot agents |
| `/hermes/robot_state_beacon` | `std_msgs/String` | Per-robot pose beacon |
| `/hermes/vest_serial_tx` | `std_msgs/String` | Motor frames to vest (`V1,<seq>,<m1..m6>`) |
| `/hermes/robot_haptic_status` | `std_msgs/String` | Per-robot obstacle/error state |

## Subsystem Docs

- Firmware, ESP-NOW config, serial protocols, flash commands: [`espnow_testbed/README.md`](./espnow_testbed/README.md)
- ROS package, launch files, topics, JSON contracts, OptiTrack: [`ros_version/README.md`](./ros_version/README.md)
- Lab bring-up sequence: [`docs/LAB_BRINGUP_CHECKLIST.md`](./docs/LAB_BRINGUP_CHECKLIST.md)
- Machine-by-machine commands: [`docs/MACHINE_COMMANDS.md`](./docs/MACHINE_COMMANDS.md)
