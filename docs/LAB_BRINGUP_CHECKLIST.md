# Lab Bring-Up Checklist

Use this as the shortest end-to-end checklist for a live lab session.

## 1. Before Powering Anything

- Confirm the latest repo is on:
  - Mac
  - Raspberry Pi 5
  - every ROSbot
- Confirm these values are edited for the hardware you are using:
  - glove `HUB_MAC` values
  - vest `LEFT_GLOVE_MAC` / `RIGHT_GLOVE_MAC`
  - OptiTrack YAMLs for the robots you will actually use
- Confirm the vest ESP serial device on the Pi is known:
  - usually `/dev/ttyUSB0` or `/dev/ttyACM0`
- If using OptiTrack Version 2, confirm `natnet_ros2` is built on every ROSbot

## 2. Flash / Verify The ESP32s

- Left glove flashed with:
  - `espnow_testbed/firmware/glove_left`
- Right glove flashed with:
  - `espnow_testbed/firmware/glove_right`
- Vest flashed with:
  - `ros_version/firmware/esp32_haptic_vest`

Quick checks:
- left glove serial monitor shows TX lines
- right glove serial monitor shows TX lines
- vest serial monitor shows `hermes.hub.status` and glove RX lines

## 3. Build The ROS Package

On the Pi:
- build `hermes_control`

On each ROSbot:
- build `natnet_ros2` in `optitrack_ws`
- build `hermes_control` in `ros_version`

## 4. Bring Up OptiTrack / Motive

- Motive is running
- NatNet streaming is enabled
- correct server IP is configured
- rigid body names match the YAMLs
- robots you plan to use are actively tracked

## 5. Start The ROSbots

On each robot you are using:
- start the base robot stack first
- then launch `robot_agent_optitrack_version2.launch.py`

Quick checks on each ROSbot:
- `/hermes/robot_state_beacon` updates
- local rigid-body pose topic exists if using Version 2
- `/cmd_vel` appears when commands are sent

## 6. Start The Pi Wearables Stack

Launch:
- `wearables_pi.launch.py`

Quick checks on the Pi:
- `/hermes/vest_hub_rx` is active
- `/hermes/raw_input` contains real glove data
- `/hermes/gesture_state` changes when you move the gloves
- `/hermes/swarm_intent` updates

Important:
- do not launch `hermes_keyboard_teleop.launch.py` at the same time
- if you want keyboard fallback, run only `keyboard_teleop_node`

## 7. Validate Glove Interpretation Before Moving Robots

Check:
- left glove posture changes the mode
- left-glove palm-up disables deadman
- left-glove shake triggers `ESTOP`
- right glove FSR gestures produce command packets

If this is wrong, stop here and debug wearables first.

## 8. Validate Motion With One Robot First

- start with one ROSbot only
- confirm drive / formation commands produce sensible motion
- confirm OptiTrack frame and heading look correct

If one robot is wrong, do not scale to multiple robots yet.

## 9. Validate Formation With Two Robots

- use two healthy robots first
- test `LINE`
- test `COLUMN`
- verify both robots settle toward consistent slots

If they drift or swirl:
- check OptiTrack rigid-body forward axis
- check frame mapping
- check robot base health / odometry / controller behavior

## 10. Scale Up To More Robots

- add robots one at a time
- verify each added robot publishes beacon updates
- verify the Pi still sees stable glove data
- verify formation motion remains stable as the group grows

## 11. Validate Haptics

- confirm the vest serial bridge is receiving glove packets
- confirm `/hermes/haptic_vest_state` is active
- trigger known events and verify haptic response:
  - gesture acknowledgement
  - formation reached pulse
  - obstacle cue
  - error / stale-status cue

## 12. Final Demo Sanity Check

Before the demo:
- deadman works
- `ESTOP` works
- selected robots match the intended swarm
- Motive is tracking the active rigid bodies
- vest serial port is open and stable
- no duplicate `swarm_control_node` is running
- no stale placeholder OptiTrack YAML is being used for an active robot

## Fast Failure Triage

If gloves are not affecting the system:
- check `/hermes/vest_hub_rx`
- check `/hermes/raw_input`
- check vest serial permissions on the Pi
- check ESP-NOW MACs and channel

If robots are not moving:
- check `/hermes/swarm_intent`
- check `/hermes/robot_state_beacon`
- check `/cmd_vel`
- check base controller health

If formations move badly:
- check OptiTrack axes / forward axis
- check rigid body names
- check which frame is being used in the beacon
