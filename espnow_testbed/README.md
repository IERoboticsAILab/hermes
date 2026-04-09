# ESP-NOW Firmware And Standalone Gateway

This folder contains the firmware and the legacy non-ROS Raspberry Pi gateway used to validate the wearable transport path independently of the ROS 2 package.

There are **two ESP-NOW receiver options** in this repository:

1. **Current / recommended hardware path**
   - receiver firmware: `ros_version/firmware/esp32_haptic_vest/esp32_haptic_vest.ino`
   - role: receives glove packets, forwards them to the Pi over USB serial, receives haptic motor commands back, and drives the vest motors

2. **Legacy standalone testbed path**
   - receiver firmware: `firmware/hub_master/hub_master.ino`
   - role: receives glove packets and forwards them to the Pi over USB serial, but does **not** drive vest motors

The standalone Python gateway in `pi_gateway/hermes_gateway.py` can consume the serial JSON stream from either receiver, as long as the receiver emits the `hermes.hub.v1` line protocol.

## What Is In This Folder

| Path | Purpose |
|---|---|
| `firmware/get_mac/get_mac.ino` | Print an ESP32 STA MAC address so you can fill the peer arrays correctly |
| `firmware/glove_left/glove_left.ino` | Left glove ESP32 transmitter |
| `firmware/glove_right/glove_right.ino` | Right glove ESP32 transmitter |
| `firmware/hub_master/hub_master.ino` | Legacy standalone ESP-NOW receiver / serial forwarder |
| `firmware/fsr_raw_test/fsr_raw_test.ino` | Right-glove FSR raw-value diagnostic sketch |
| `pi_gateway/hermes_gateway.py` | Standalone Raspberry Pi gateway using the root Python gesture/swarm modules |
| `pi_gateway/config.example.json` | Example config for the standalone Pi gateway |

## Current Wearable Hardware Roles

### Left glove firmware

File:
- `firmware/glove_left/glove_left.ino`

Current role:
- flex sensors
- MPU6050 IMU
- ESP-NOW transmit to the vest / hub receiver

Current sensor ownership in the overall project:
- left-hand posture / mode selection
- deadman IMU
- left-glove shake `ESTOP`
- active control IMU in the current configuration

Current pins in the committed firmware:
- flex index -> `GPIO35`
- flex middle -> `GPIO32`
- flex ring -> `GPIO33`
- flex pinky -> `GPIO34`
- MPU6050 `SDA` -> `GPIO21`
- MPU6050 `SCL` -> `GPIO22`

### Right glove firmware

File:
- `firmware/glove_right/glove_right.ino`

Current role:
- FSR sensors
- ESP-NOW transmit to the vest / hub receiver
- right-glove IMU support remains in code, but is disabled by `RIGHT_GLOVE_HAS_IMU = false`

Current sensor ownership in the overall project:
- right-hand discrete gesture commands only

Current pins in the committed firmware:
- FSR index -> `GPIO34`
- FSR middle -> `GPIO35`
- FSR ring -> `GPIO32`
- FSR pinky -> `GPIO33`
- MPU6050 `SDA` -> `GPIO21`
- MPU6050 `SCL` -> `GPIO22`

Important:
- these right-glove FSR pins are **ADC1** pins on a classic ESP32, which avoids the ADC2/Wi-Fi conflict that breaks analog reads when ESP-NOW is active
- current threshold in firmware: `FSR_PRESS_THRESHOLD = 1200`

### Receiver firmware options

#### Final vest receiver

File:
- `../ros_version/firmware/esp32_haptic_vest/esp32_haptic_vest.ino`

Current role:
- receives left/right glove packets over ESP-NOW
- validates sender MAC addresses
- forwards newline-delimited JSON to the Pi over USB serial
- receives vest motor frames from the Pi over USB serial
- drives 6 vest motors

#### Legacy standalone hub receiver

File:
- `firmware/hub_master/hub_master.ino`

Current role:
- receives left/right glove packets over ESP-NOW
- forwards newline-delimited JSON to the Pi over USB serial
- no motor-driving role

## ESP-NOW Configuration You Must Set

All committed MAC values should be treated as lab defaults or examples. Replace them with the MACs of the actual boards you are using.

### 1. Find each board's MAC address

Flash the MAC utility:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/get_mac
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32 espnow_testbed/firmware/get_mac
arduino-cli monitor -p <PORT> -c baudrate=115200
```

Expected output:

```text
ESP32 STA MAC: XX:XX:XX:XX:XX:XX
```

Convert that to the array format used by the firmware.

Example:
- `24:6F:28:AA:BB:CC` -> `{0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}`

### 2. Set the receiver MAC in both gloves

Edit these files:
- `firmware/glove_left/glove_left.ino`
- `firmware/glove_right/glove_right.ino`

Set:

```cpp
static uint8_t HUB_MAC[6] = { ... };
```

to the **receiver** MAC.

Use:
- the vest ESP MAC if you are using the final vest firmware
- the hub ESP MAC if you are using the legacy `hub_master` path

### 3. Set the glove MACs in the receiver

If you are using the final vest firmware, edit:
- `../ros_version/firmware/esp32_haptic_vest/esp32_haptic_vest.ino`

Set:

```cpp
static uint8_t LEFT_GLOVE_MAC[6] = { ... };
static uint8_t RIGHT_GLOVE_MAC[6] = { ... };
```

If you are using the legacy hub, edit:
- `firmware/hub_master/hub_master.ino`

and set the same two arrays there.

### 4. Keep the ESP-NOW channel aligned

Current committed value:

```cpp
static const uint8_t ESPNOW_CHANNEL = 1;
```

Keep that identical on:
- left glove
- right glove
- vest receiver or `hub_master`

## Serial Protocols Used By The Receiver

### Receiver -> Pi

The receiver emits newline-delimited JSON. The current final vest firmware writes lines like:

```json
{"schema":"hermes.hub.v1","rx_ms":1234,"sender_mac":"AA:BB:CC:DD:EE:FF","valid_json":true,"glove_id":"L","packet":{...}}
```

It also emits status lines such as:

```json
{"schema":"hermes.hub.status","status":"ready","device":"vest_hub","serial_baud":921600,"espnow_channel":1}
```

### Pi -> vest receiver

Only the final vest firmware uses this direction.

The Pi sends ASCII lines of the form:

```text
V1,<seq>,<m1>,<m2>,<m3>,<m4>,<m5>,<m6>
```

where each motor value is `0..255`.

## Current Glove Payload Shapes

### Left glove payload

The left glove currently sends JSON like:

```json
{
  "v": 1,
  "id": "L",
  "seq": 42,
  "t": 123456,
  "flex": {
    "index": 0.10,
    "middle": 0.25,
    "ring": 0.80,
    "pinky": 0.77
  },
  "imu": {
    "PITCH": 0.12,
    "ROLL": -0.03,
    "YAW": 0.05,
    "AX": 0.02,
    "AY": -0.08,
    "AZ": -0.98
  }
}
```

### Right glove payload

The right glove currently sends JSON like:

```json
{
  "v": 1,
  "id": "R",
  "seq": 42,
  "t": 123456,
  "fsr": {
    "INDEX": false,
    "MIDDLE": true,
    "RING": false,
    "PINKY": false
  }
}
```

If you re-enable the right-glove IMU in firmware later, the packet can also include an `imu` object, but that is **not** the current deployed configuration.

## Flash Commands

From the repository root:

```bash
cd "/path/to/H.E.R.M.E.S Project"
export FQBN="esp32:esp32:esp32"
```

### Left glove

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_left
arduino-cli upload -p <LEFT_PORT> --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/glove_left
```

### Right glove

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/glove_right
arduino-cli upload -p <RIGHT_PORT> --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/glove_right
```

### Final vest receiver

```bash
arduino-cli compile --fqbn "$FQBN" ros_version/firmware/esp32_haptic_vest
arduino-cli upload -p <VEST_PORT> --fqbn "$FQBN" --upload-property upload.speed=115200 ros_version/firmware/esp32_haptic_vest
```

### Legacy hub receiver

```bash
arduino-cli compile --fqbn "$FQBN" espnow_testbed/firmware/hub_master
arduino-cli upload -p <HUB_PORT> --fqbn "$FQBN" --upload-property upload.speed=115200 espnow_testbed/firmware/hub_master
```

If your board is not a classic ESP32 dev board, replace the `FQBN` accordingly.

## Standalone Raspberry Pi Gateway

The standalone gateway is useful when you want to validate the transport + gesture + swarm logic without bringing up ROS.

### Install dependency

```bash
cd espnow_testbed/pi_gateway
python3 -m pip install -r requirements.txt
```

### Configure it

Edit:
- `pi_gateway/config.example.json`

Current config fields:
- `serial_port`
- `baudrate`
- `glove_timeout_ms`
- `robot_ids`
- `centroid`
- `behavior_runtime.home_xy`
- `behavior_runtime.path_waypoints`
- `command_output.mode`
- `command_output.udp_host`
- `command_output.udp_port`

### Run it

```bash
cd espnow_testbed/pi_gateway
python3 hermes_gateway.py --config config.example.json
```

Current behavior of the standalone gateway:
- ingests `hermes.hub.v1` serial lines
- requires both gloves to be fresh before building a usable raw sample
- feeds the root Python pipeline in `gestures/` and `swarm/`
- prints or UDP-emits simplified swarm command payloads

## Serial Monitoring / Debugging

### Left glove monitor

```bash
arduino-cli monitor -p <LEFT_PORT> -c baudrate=115200
```

Current firmware prints:
- readiness messages
- raw flex readings
- transmitted JSON debug lines
- ESP-NOW send failures if they occur

### Right glove monitor

```bash
arduino-cli monitor -p <RIGHT_PORT> -c baudrate=115200
```

Current firmware prints:
- readiness messages
- transmitted JSON debug lines
- ESP-NOW send failures if they occur

### Final vest monitor

```bash
arduino-cli monitor -p <VEST_PORT> -c baudrate=921600
```

Current final vest firmware prints:
- `hermes.hub.status` ready JSON
- forwarded glove packet JSON lines
- readable debug lines for glove receive events
- readable debug lines for motor frame receive events

Important:
- only one process can own the serial port at a time
- stop the Pi-side serial bridge or gateway before opening a serial monitor on the same device

## Right-Glove FSR Diagnostic Sketch

Use this when the right glove FSR booleans are not switching as expected.

Flash:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 espnow_testbed/firmware/fsr_raw_test
arduino-cli upload -p <RIGHT_PORT> --fqbn esp32:esp32:esp32 --upload-property upload.speed=115200 espnow_testbed/firmware/fsr_raw_test
arduino-cli monitor -p <RIGHT_PORT> -c baudrate=115200
```

The diagnostic sketch prints raw ADC values and the current threshold comparison so you can tune the FSR wiring or threshold before flashing the real glove firmware back.

## Common Failure Cases

### No glove data reaches the Pi

Check in this order:

1. the gloves are printing TX lines in their serial monitors
2. both gloves use the correct receiver MAC in `HUB_MAC`
3. the receiver has the correct left/right glove MACs
4. all three boards use the same `ESPNOW_CHANNEL`
5. the vest / hub receiver serial monitor shows incoming glove lines

### FSR diagnostic works, but the real right glove does not

That usually means one of these is still wrong:
- old firmware is still flashed
- the wrong board was flashed
- the right glove is not actually using the expected ADC1 pins from the current sketch
- the receiver MAC / channel is wrong, so the glove is reading correctly but not delivering packets

### Receiver serial is visible but the Pi gateway does nothing

Make sure the serial lines are actually `hermes.hub.v1` JSON and that the `serial_port` / `baudrate` match the receiver firmware.

## Relationship To The ROS Wearable Path

The standalone gateway path is **not** the default final stack anymore.

The current recommended full wearable path is:

- gloves -> vest ESP32 over ESP-NOW
- vest ESP32 -> Raspberry Pi over USB serial
- Raspberry Pi -> ROS via `vest_serial_bridge_node`
- ROS gesture pipeline / swarm control / haptic vest feedback in `ros_version`

See:
- [`../ros_version/README.md`](../ros_version/README.md)
