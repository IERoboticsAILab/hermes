#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// --------------------------------------------
// GLOVE RIGHT FIRMWARE (ESP-NOW TX)
// Right glove hardware: FSR sensors only.
// Left glove owns flex/posture sensing and the only IMU used by the system.
// --------------------------------------------
// Fill in the vest/hub ESP32 MAC here.
// This is the ESP32 on the vest, connected to the Raspberry Pi over USB serial.
static uint8_t HUB_MAC[6] = {0x4c, 0xc3, 0x82, 0xcc, 0xd0, 0x04};

static const char* GLOVE_ID = "R";
static const uint8_t ESPNOW_CHANNEL = 1;
static const uint32_t SEND_INTERVAL_MS = 20;  // 50 Hz
static const bool SERIAL_DEBUG_TX = true;
static const uint32_t SERIAL_DEBUG_INTERVAL_MS = 250;

// Use ADC1 pins here. ADC2 pins such as GPIO25/GPIO26/GPIO27/GPIO14
// conflict with Wi-Fi/ESP-NOW on classic ESP32 boards.
static const int FSR_INDEX_PIN = 34;
static const int FSR_MIDDLE_PIN = 35;
static const int FSR_RING_PIN = 32;
static const int FSR_PINKY_PIN = 33;

static const int FSR_PRESS_THRESHOLD = 1200;  // ADC threshold, tune for your hardware

uint32_t seq_no = 0;
uint32_t last_send_ms = 0;
uint32_t last_debug_tx_ms = 0;
volatile bool send_in_flight = false;
uint32_t last_send_start_ms = 0;

bool fsrPressed(int pin) {
  return analogRead(pin) > FSR_PRESS_THRESHOLD;
}

void onSendStatus(esp_now_send_status_t status) {
  send_in_flight = false;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[RIGHT] ESP-NOW send failed");
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
    Serial.println("[RIGHT] ESP-NOW init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, HUB_MAC, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("[RIGHT] Failed to add hub peer");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(FSR_INDEX_PIN, INPUT);
  pinMode(FSR_MIDDLE_PIN, INPUT);
  pinMode(FSR_RING_PIN, INPUT);
  pinMode(FSR_PINKY_PIN, INPUT);

  Serial.println("[RIGHT] FSR-only right glove");

  if (!initEspNow()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[RIGHT] Ready");
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

  StaticJsonDocument<384> doc;
  doc["v"] = 1;
  doc["id"] = GLOVE_ID;
  doc["seq"] = seq_no++;
  doc["t"] = now;

  JsonObject fsr = doc.createNestedObject("fsr");
  fsr["INDEX"] = fsrPressed(FSR_INDEX_PIN);
  fsr["MIDDLE"] = fsrPressed(FSR_MIDDLE_PIN);
  fsr["RING"] = fsrPressed(FSR_RING_PIN);
  fsr["PINKY"] = fsrPressed(FSR_PINKY_PIN);

  char payload[220];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  if (len == 0 || len >= sizeof(payload)) {
    Serial.println("[RIGHT] JSON too large, dropped");
    return;
  }

  if (SERIAL_DEBUG_TX && (now - last_debug_tx_ms) >= SERIAL_DEBUG_INTERVAL_MS) {
    last_debug_tx_ms = now;
    Serial.print("[RIGHT] TX ");
    Serial.println(payload);
  }

  esp_err_t result = esp_now_send(HUB_MAC, reinterpret_cast<const uint8_t*>(payload), len);
  if (result == ESP_OK) {
    send_in_flight = true;
    last_send_start_ms = now;
  } else {
    send_in_flight = false;
    Serial.printf("[RIGHT] esp_now_send error=%d (%s)\n", result, esp_err_to_name(result));
  }
}
