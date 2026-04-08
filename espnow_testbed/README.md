# ESP-NOW Testbed (Non-ROS)

This folder is a standalone transmission + logic test path outside the ROS package.
The final ROS wearable path uses the vest ESP32 as the ESP-NOW receiver and USB-serial haptic endpoint; the older `hub_master` sketch remains here for isolated non-ROS testing.

## What is included

- `firmware/glove_left/glove_left.ino`
  - Left glove ESP32 firmware (reads sensors, sends JSON via ESP-NOW)
- `firmware/glove_right/glove_right.ino`
  - Right glove ESP32 firmware (reads sensors, sends JSON via ESP-NOW)
- `firmware/hub_master/hub_master.ino`
  - Legacy standalone hub ESP32 firmware (receives ESP-NOW from both gloves, forwards newline JSON over serial to Raspberry Pi)
- `firmware/get_mac/get_mac.ino`
  - Utility sketch to print ESP32 STA MAC address
- `pi_gateway/hermes_gateway.py`
  - Raspberry Pi script:
    1. reads hub serial stream
    2. fuses left+right glove packets
    3. runs your existing gesture/safety/swarm logic
    4. emits swarm commands (print or UDP)

## MAC address placeholders (where to edit)

### 1) Left glove firmware
File: `firmware/glove_left/glove_left.ino`

Edit this line with the vest/hub ESP32 MAC:

```cpp
static uint8_t HUB_MAC[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
```

### 2) Right glove firmware
File: `firmware/glove_right/glove_right.ino`

Edit this line with the vest/hub ESP32 MAC:

```cpp
static uint8_t HUB_MAC[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
```

### 3) Final vest ESP firmware
File: `../ros_version/firmware/esp32_haptic_vest/esp32_haptic_vest.ino`

Edit both glove MAC placeholders:

```cpp
static uint8_t LEFT_GLOVE_MAC[6]  = {0xAA, 0xBB, 0xCC, 0x44, 0x55, 0x66};
static uint8_t RIGHT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x77, 0x88, 0x99};
```

### 4) Legacy standalone hub/master firmware
File: `firmware/hub_master/hub_master.ino`

Only use this path if you are testing the old non-ROS gateway without the vest motor firmware. Edit both glove MAC placeholders:

```cpp
static uint8_t LEFT_GLOVE_MAC[6]  = {0xAA, 0xBB, 0xCC, 0x44, 0x55, 0x66};
static uint8_t RIGHT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x77, 0x88, 0x99};
```

## How to get each ESP32 MAC address

1. Flash `firmware/get_mac/get_mac.ino` to a board.
2. Open serial monitor at `115200`.
3. Note `ESP32 STA MAC: XX:XX:XX:XX:XX:XX`.
4. Convert each byte to hex with `0x` prefix and place into arrays above.

Example:
- MAC `24:6F:28:AA:BB:CC` -> `{0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}`

## Firmware dependencies

Install in Arduino IDE (or PlatformIO):
- `ArduinoJson`
- ESP32 board support package (Espressif)

## Raspberry Pi gateway setup

### 1) Install Python dependency

```bash
cd espnow_testbed/pi_gateway
python3 -m pip install -r requirements.txt
```

### 2) Configure gateway

Edit:
- `pi_gateway/config.example.json`
  - serial port
  - output mode (`print` or `udp`)
  - optional UDP target
  - `robot_ids` defaults to `r1`-`r6`
  - `behavior_runtime.home_xy` for `RETURN_HOME`
  - `behavior_runtime.path_waypoints` for `FOLLOW_PATH` / `FOLLOW_PATH_LOOP`

### 3) Run gateway

```bash
cd espnow_testbed/pi_gateway
python3 hermes_gateway.py --config config.example.json
```

## Data flow in this testbed

1. Left/right glove ESP32 send JSON packets via ESP-NOW.
2. The vest ESP32 validates sender MAC and forwards packet stream over USB serial. If you are using the legacy `hub_master` sketch instead, that standalone hub performs the same receive/forward role without vest motor control.
3. Pi gateway fuses left+right stream into the expected raw input shape.
4. Pi gateway runs your current command logic and emits swarm command payloads.

## Notes

- If either glove stream is stale (`glove_timeout_ms`), gateway forces deadman false for safety.
- Left glove firmware currently assumes **no left FSR hardware** and sends flex + IMU only.
- Right glove firmware assumes FSR + IMU only; current FSR pins are GPIO25/GPIO26/GPIO27/GPIO14 and should match your wiring.
- MPU6050 IMU read functions are implemented in both glove firmware files; adjust I2C pins/config if your hardware differs.
- Selection group slots are currently `A`-`G` (7 groups), and robot bindings cover `r1`-`r6`.
- `FOLLOW_ME_TOGGLE` now behaves as a true toggle (first gesture enables, next gesture disables).
