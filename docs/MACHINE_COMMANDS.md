# Machine-By-Machine Commands

This is the operator command sheet for the current project layout, filled with the real lab values currently present in the repo.

## Current Known Lab Mapping

### Operator / infrastructure

- Mac repo path:
  - `/Users/salehabdelrahman/Desktop/H.E.R.M.E.S Project`
- Raspberry Pi 5 hostname:
  - `hermes-desktop`
- Motive / NatNet server IP:
  - `10.205.3.3`
- Vest serial device on the Pi used in the current setup:
  - `/dev/ttyUSB0`

### ROSbot mapping currently committed in the OptiTrack YAMLs

- `r1` -> hostname `persephone` -> IP `10.205.3.43` -> rigid body `umh_3`
- `r2` -> hostname `hypnos` -> IP `10.205.3.41` -> rigid body `umh_4`
- `r3` -> hostname `icarus` -> IP `10.205.3.44` -> rigid body `umh_4_green`
- `r4` -> hostname `daemon` -> IP `10.205.3.45` -> rigid body `umh_5`
- `r5` -> still placeholder in repo -> IP `192.168.0.105` -> rigid body `rosbot_5`
- `r6` -> still placeholder in repo -> IP `192.168.0.106` -> rigid body `rosbot_6`

Important:
- `r5` and `r6` still need your real hostnames / IPs / rigid body names before use.
- The committed `r1`-`r4` values match the current YAMLs under `ros_version/src/hermes_control/config/`.

## 1. Mac: Flash ESP32 Firmware

Repository root:

```bash
cd "/Users/salehabdelrahman/Desktop/H.E.R.M.E.S Project"
arduino-cli board list
export FQBN="esp32:esp32:esp32"
```

The serial port seen in your recent flashing session was:

```bash
/dev/cu.usbserial-0001
```

Use that if the same board is still attached and `arduino-cli board list` still reports it.

### Get MAC addresses

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/get_mac
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/get_mac
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```

### Flash left glove

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_left
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/glove_left
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```

### Flash right glove

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_right
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/glove_right
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```

### Flash vest ESP

```bash
arduino-cli compile --fqbn "$FQBN" ros_version/firmware/esp32_haptic_vest
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn "$FQBN" --upload-property upload.speed=115200 ros_version/firmware/esp32_haptic_vest
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=921600
```

### Optional right-glove FSR diagnostic

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/fsr_raw_test
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/fsr_raw_test
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```

Note:
- if `arduino-cli board list` shows a different port, use the current one instead of `/dev/cu.usbserial-0001`
- only one board should be attached at a time if you are using the same port path for every flash

## 2. Raspberry Pi 5 (`hermes@hermes-desktop`): Build And Run Wearables Stack

### Build

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

### Verify vest serial port

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Current expected port:

```bash
/dev/ttyUSB0
```

### If serial permission is denied

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

If that still does not update the session, log out and back in.

### Run wearables stack

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch hermes_control wearables_pi.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600
```

### Pi debug commands

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

### Optional keyboard fallback while wearables are running

Do not run `hermes_keyboard_teleop.launch.py` here.
Run only:

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run hermes_control keyboard_teleop_node --ros-args \
  -p robot_ids:="['r1','r2','r3','r4','r5','r6']"
```

## 3. ROSbot Build Commands

Run on each ROSbot used with OptiTrack Version 2.

### Build `natnet_ros2`

```bash
cd ~/Desktop/optitrack_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select natnet_ros2
source install/setup.bash
```

### Build `hermes_control`

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source ~/Desktop/optitrack_ws/install/setup.bash
colcon build --symlink-install --packages-select hermes_control
source install/setup.bash
```

## 4. ROSbot Common Runtime Environment

Run before launching the robot agent:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/optitrack_ws/install/setup.bash
source ~/Desktop/H.E.R.M.E.S/ros_version/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Also make sure the ROSbot base stack is already running.

## 5. ROSbot Launch Commands

These commands assume the per-robot YAMLs are correct.

### `r1` on `husarion@persephone` (`10.205.3.43`, rigid body `umh_3`)

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r1.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

### `r2` on `husarion@hypnos` (`10.205.3.41`, rigid body `umh_4`)

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r2.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

### `r3` on `husarion@icarus` (`10.205.3.44`, rigid body `umh_4_green`)

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r3.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

### `r4` on `husarion@daemon` (`10.205.3.45`, rigid body `umh_5`)

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r4.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

### `r5` on its ROSbot once you replace the placeholder values

Current placeholder YAML values:
- IP `192.168.0.105`
- rigid body `rosbot_5`

Command after you finish updating the YAML:

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r5.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

### `r6` on its ROSbot once you replace the placeholder values

Current placeholder YAML values:
- IP `192.168.0.106`
- rigid body `rosbot_6`

Command after you finish updating the YAML:

```bash
ros2 launch hermes_control robot_agent_optitrack_version2.launch.py \
  optitrack_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/optitrack_r6.yaml \
  odom_topic:=/rosbot_base_controller/odom \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  cmd_vel_frame_id:=base_link
```

## 6. ROSbot Debug Commands

Run on a robot after launch:

```bash
ros2 topic echo /hermes/robot_state_beacon --once
ros2 topic echo /hermes/swarm_intent --once
ros2 topic echo /hermes/slot_bids --once
ros2 topic echo /cmd_vel
```

For NatNet pose topics:

```bash
ros2 topic list | grep pose
ros2 topic echo /<rigid_body_name>/pose --once
```

Known rigid bodies in the current mapping:

```bash
/umh_3/pose
/umh_4/pose
/umh_4_green/pose
/umh_5/pose
```

## 7. Optional Operator / Pi Keyboard-Only Path

If you are not using gloves and only want keyboard control:

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hermes_control hermes_keyboard_teleop.launch.py
```

## 8. Optional Pi Haptic-Only Path

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hermes_control haptic_vest.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600
```

## 9. Optional Robot Haptic Status Path

On each ROSbot:

```bash
cd ~/Desktop/H.E.R.M.E.S/ros_version
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hermes_control robot_haptic_status.launch.py \
  robot_id:=r1 \
  status_config:=/home/husarion/Desktop/H.E.R.M.E.S/ros_version/src/hermes_control/config/robot_haptic_status_r1.yaml
```

Swap the YAML and `robot_id` per robot.

## 10. Quick Failure Checks

### Pi sees no glove data

```bash
ros2 topic echo /hermes/vest_serial_state --once
ros2 topic echo /hermes/vest_hub_rx --once
ros2 topic echo /hermes/raw_input --once
```

### Robot does not get pose updates

```bash
ros2 topic list | grep pose
ros2 topic echo /<rigid_body_name>/pose --once
ros2 topic echo /hermes/robot_state_beacon --once
```

### Robot does not move

```bash
ros2 topic echo /hermes/swarm_intent --once
ros2 topic echo /cmd_vel
```
