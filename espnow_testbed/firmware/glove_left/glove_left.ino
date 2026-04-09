#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// -------------------------------
// GLOVE LEFT FIRMWARE (ESP-NOW TX)
// -------------------------------
// Fill in the vest/hub ESP32 MAC here.
// This is the ESP32 on the vest, connected to the Raspberry Pi over USB serial.
static uint8_t HUB_MAC[6] = {0x4c, 0xc3, 0x82, 0xcc, 0xd0, 0x04};

static const char* GLOVE_ID = "L";
static const uint8_t ESPNOW_CHANNEL = 1;
static const uint32_t SEND_INTERVAL_MS = 20;  // 50 Hz
static const bool SERIAL_DEBUG_TX = true;
static const uint32_t SERIAL_DEBUG_INTERVAL_MS = 250;

// Update these pins to match your wiring.
static const int FLEX_INDEX_PIN = 35;
static const int FLEX_MIDDLE_PIN = 32;
static const int FLEX_RING_PIN = 33;
static const int FLEX_PINKY_PIN = 34;

// MPU6050 IMU (I2C). Adjust pins/address only if your wiring differs.
static const int IMU_SDA_PIN = 21;
static const int IMU_SCL_PIN = 22;
static const uint32_t IMU_I2C_HZ = 400000;
static const float IMU_ACCEL_LSB_PER_G = 16384.0f;   // +/-2g
static const float IMU_GYRO_LSB_PER_DPS = 131.0f;    // +/-250 dps
static const float IMU_COMP_ALPHA = 0.98f;           // Complementary filter blend
static const uint16_t IMU_CALIBRATION_SAMPLES = 300;

static uint8_t imu_addr = 0x68;
static bool imu_ready = false;
static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;
static float gyro_bias_z_dps = 0.0f;
static float fused_pitch_rad = 0.0f;
static float fused_roll_rad = 0.0f;
static float fused_yaw_rad = 0.0f;
static float last_ax_g = 0.0f;
static float last_ay_g = 0.0f;
static float last_az_g = -1.0f;
static uint32_t imu_last_us = 0;

uint32_t seq_no = 0;
uint32_t last_send_ms = 0;
uint32_t last_debug_tx_ms = 0;
volatile bool send_in_flight = false;
uint32_t last_send_start_ms = 0;

float normalizeFlex(int raw) {
  float v = (float)raw / 4095.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return v;
}

bool imuProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0;
}

bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool imuReadRegs(uint8_t start_reg, uint8_t* out, size_t len) {
  Wire.beginTransmission(imu_addr);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t got = Wire.requestFrom((int)imu_addr, (int)len, (int)true);
  if (got != len) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    out[i] = (uint8_t)Wire.read();
  }
  return true;
}

bool imuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  if (!imuReadRegs(0x3B, buf, sizeof(buf))) {
    return false;
  }

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

bool initImu() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_HZ);
  delay(20);

  if (imuProbe(0x68)) {
    imu_addr = 0x68;
  } else if (imuProbe(0x69)) {
    imu_addr = 0x69;
  } else {
    return false;
  }

  // Wake MPU6050 and set ranges/filter.
  if (!imuWriteReg(0x6B, 0x00)) return false;  // PWR_MGMT_1
  delay(50);
  if (!imuWriteReg(0x1A, 0x03)) return false;  // DLPF_CFG
  if (!imuWriteReg(0x1B, 0x00)) return false;  // GYRO_CONFIG +/-250 dps
  if (!imuWriteReg(0x1C, 0x00)) return false;  // ACCEL_CONFIG +/-2g
  if (!imuWriteReg(0x19, 0x04)) return false;  // SMPLRT_DIV

  float sum_gx = 0.0f;
  float sum_gy = 0.0f;
  float sum_gz = 0.0f;
  int16_t ax, ay, az, gx, gy, gz;

  for (uint16_t i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) {
    if (!imuReadRaw(ax, ay, az, gx, gy, gz)) {
      return false;
    }
    sum_gx += ((float)gx) / IMU_GYRO_LSB_PER_DPS;
    sum_gy += ((float)gy) / IMU_GYRO_LSB_PER_DPS;
    sum_gz += ((float)gz) / IMU_GYRO_LSB_PER_DPS;
    delay(3);
  }

  gyro_bias_x_dps = sum_gx / (float)IMU_CALIBRATION_SAMPLES;
  gyro_bias_y_dps = sum_gy / (float)IMU_CALIBRATION_SAMPLES;
  gyro_bias_z_dps = sum_gz / (float)IMU_CALIBRATION_SAMPLES;

  if (!imuReadRaw(ax, ay, az, gx, gy, gz)) {
    return false;
  }

  float ax_g = ((float)ax) / IMU_ACCEL_LSB_PER_G;
  float ay_g = ((float)ay) / IMU_ACCEL_LSB_PER_G;
  float az_g = ((float)az) / IMU_ACCEL_LSB_PER_G;
  last_ax_g = ax_g;
  last_ay_g = ay_g;
  last_az_g = az_g;
  fused_roll_rad = atan2f(ay_g, az_g);
  fused_pitch_rad = atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g)));
  fused_yaw_rad = 0.0f;
  imu_last_us = micros();
  imu_ready = true;
  return true;
}

void readImu(float& pitch, float& roll, float& yaw, float& ax, float& ay, float& az) {
  if (!imu_ready) {
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    ax = 0.0f;
    ay = 0.0f;
    az = -1.0f;
    return;
  }

  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  if (!imuReadRaw(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)) {
    // Preserve orientation estimate if a sample read fails.
    pitch = fused_pitch_rad;
    roll = fused_roll_rad;
    yaw = fused_yaw_rad;
    ax = last_ax_g;
    ay = last_ay_g;
    az = last_az_g;
    return;
  }

  ax = ((float)raw_ax) / IMU_ACCEL_LSB_PER_G;
  ay = ((float)raw_ay) / IMU_ACCEL_LSB_PER_G;
  az = ((float)raw_az) / IMU_ACCEL_LSB_PER_G;
  last_ax_g = ax;
  last_ay_g = ay;
  last_az_g = az;

  float gx_dps = (((float)raw_gx) / IMU_GYRO_LSB_PER_DPS) - gyro_bias_x_dps;
  float gy_dps = (((float)raw_gy) / IMU_GYRO_LSB_PER_DPS) - gyro_bias_y_dps;
  float gz_dps = (((float)raw_gz) / IMU_GYRO_LSB_PER_DPS) - gyro_bias_z_dps;

  uint32_t now_us = micros();
  float dt = ((float)(now_us - imu_last_us)) * 1e-6f;
  imu_last_us = now_us;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = ((float)SEND_INTERVAL_MS) * 1e-3f;
  }

  float gx_rad_s = gx_dps * DEG_TO_RAD;
  float gy_rad_s = gy_dps * DEG_TO_RAD;
  float gz_rad_s = gz_dps * DEG_TO_RAD;

  float accel_roll_rad = atan2f(ay, az);
  float accel_pitch_rad = atan2f(-ax, sqrtf((ay * ay) + (az * az)));

  fused_roll_rad = (IMU_COMP_ALPHA * (fused_roll_rad + (gx_rad_s * dt))) +
                   ((1.0f - IMU_COMP_ALPHA) * accel_roll_rad);
  fused_pitch_rad = (IMU_COMP_ALPHA * (fused_pitch_rad + (gy_rad_s * dt))) +
                    ((1.0f - IMU_COMP_ALPHA) * accel_pitch_rad);
  fused_yaw_rad += gz_rad_s * dt;

  if (fused_yaw_rad > PI) {
    fused_yaw_rad -= (2.0f * PI);
  } else if (fused_yaw_rad < -PI) {
    fused_yaw_rad += (2.0f * PI);
  }

  pitch = fused_pitch_rad;
  roll = fused_roll_rad;
  yaw = fused_yaw_rad;
}

void onSendStatus(esp_now_send_status_t status) {
  send_in_flight = false;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[LEFT] ESP-NOW send failed");
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  onSendStatus(status);
}
#else
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  onSendStatus(status);
}
#endif

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[LEFT] ESP-NOW init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, HUB_MAC, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("[LEFT] Failed to add hub peer");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(FLEX_INDEX_PIN, INPUT);
  pinMode(FLEX_MIDDLE_PIN, INPUT);
  pinMode(FLEX_RING_PIN, INPUT);
  pinMode(FLEX_PINKY_PIN, INPUT);

  if (!initImu()) {
    imu_ready = false;
    Serial.println("[LEFT] WARNING: IMU init failed (MPU6050 not found or unreadable); continuing with default IMU values");
  }

  if (!initEspNow()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[LEFT] Ready");
}

void loop() {
  uint32_t now = millis();
  if ((now - last_send_ms) < SEND_INTERVAL_MS) {
    return;
  }

  if (send_in_flight) {
    if ((now - last_send_start_ms) < 200) {
      return;
    }
    send_in_flight = false;
  }

  last_send_ms = now;

  float pitch, roll, yaw, ax, ay, az;
  readImu(pitch, roll, yaw, ax, ay, az);

  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["id"] = GLOVE_ID;
  doc["seq"] = seq_no++;
  doc["t"] = now;

  int flex_index_raw = analogRead(FLEX_INDEX_PIN);
  int flex_middle_raw = analogRead(FLEX_MIDDLE_PIN);
  int flex_ring_raw = analogRead(FLEX_RING_PIN);
  int flex_pinky_raw = analogRead(FLEX_PINKY_PIN);

  JsonObject flex = doc.createNestedObject("flex");
  flex["index"] = normalizeFlex(flex_index_raw);
  flex["middle"] = normalizeFlex(flex_middle_raw);
  flex["ring"] = normalizeFlex(flex_ring_raw);
  flex["pinky"] = normalizeFlex(flex_pinky_raw);

  JsonObject imu = doc.createNestedObject("imu");
  imu["PITCH"] = pitch;
  imu["ROLL"] = roll;
  imu["YAW"] = yaw;
  imu["AX"] = ax;
  imu["AY"] = ay;
  imu["AZ"] = az;

  char payload[250];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  if (len == 0 || len >= sizeof(payload)) {
    Serial.println("[LEFT] JSON too large, dropped");
    return;
  }

  if (SERIAL_DEBUG_TX && (now - last_debug_tx_ms) >= SERIAL_DEBUG_INTERVAL_MS) {
    last_debug_tx_ms = now;
    Serial.printf("[LEFT] RAW flex INDEX=%4d MIDDLE=%4d RING=%4d PINKY=%4d\n",
                  flex_index_raw, flex_middle_raw, flex_ring_raw, flex_pinky_raw);
    Serial.print("[LEFT] TX ");
    Serial.println(payload);
  }

  esp_err_t result = esp_now_send(HUB_MAC, reinterpret_cast<const uint8_t*>(payload), len);
  if (result == ESP_OK) {
    send_in_flight = true;
    last_send_start_ms = now;
  } else {
    send_in_flight = false;
    Serial.printf("[LEFT] esp_now_send error=%d (%s)\n", result, esp_err_to_name(result));
  }
}
