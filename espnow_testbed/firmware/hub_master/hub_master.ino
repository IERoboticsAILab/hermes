#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ------------------------------------------
// HUB MASTER FIRMWARE (ESP-NOW RX -> Serial)
// ------------------------------------------
// Fill in glove MAC addresses here.
// LEFT_GLOVE_MAC: ESP32 on left glove
// RIGHT_GLOVE_MAC: ESP32 on right glove
static uint8_t LEFT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x44, 0x55, 0x66};
static uint8_t RIGHT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x77, 0x88, 0x99};

static const uint8_t ESPNOW_CHANNEL = 1;
static const uint32_t SERIAL_BAUD = 921600;

bool macEquals(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; ++i) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

String macToString(const uint8_t* mac) {
  char out[18];
  snprintf(out, sizeof(out), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(out);
}

const char* inferGloveId(const uint8_t* mac, JsonDocument& payload) {
  if (payload["id"].is<const char*>()) {
    const char* id = payload["id"];
    if (strcmp(id, "L") == 0 || strcmp(id, "R") == 0) {
      return id;
    }
  }

  if (macEquals(mac, LEFT_GLOVE_MAC)) return "L";
  if (macEquals(mac, RIGHT_GLOVE_MAC)) return "R";
  return "?";
}

bool isKnownPeer(const uint8_t* mac) {
  return macEquals(mac, LEFT_GLOVE_MAC) || macEquals(mac, RIGHT_GLOVE_MAC);
}

void handleDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) {
    return;
  }
  if (!isKnownPeer(mac)) {
    return;
  }

  char payload_buf[251];
  memcpy(payload_buf, data, len);
  payload_buf[len] = '\0';

  DynamicJsonDocument payload_doc(512);
  DeserializationError err = deserializeJson(payload_doc, payload_buf);

  DynamicJsonDocument out_doc(1024);
  out_doc["schema"] = "hermes.hub.v1";
  out_doc["rx_ms"] = millis();
  out_doc["sender_mac"] = macToString(mac);

  if (err) {
    out_doc["valid_json"] = false;
    out_doc["glove_id"] = macEquals(mac, LEFT_GLOVE_MAC) ? "L" : "R";
    out_doc["raw"] = payload_buf;
  } else {
    out_doc["valid_json"] = true;
    out_doc["glove_id"] = inferGloveId(mac, payload_doc);
    out_doc["packet"] = payload_doc.as<JsonObject>();
  }

  serializeJson(out_doc, Serial);
  Serial.println();
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info == nullptr) {
    return;
  }
  handleDataRecv(info->src_addr, data, len);
}
#else
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handleDataRecv(mac, data, len);
}
#endif

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, mac, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;

  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  return esp_now_add_peer(&peer_info) == ESP_OK;
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[HUB] ESP-NOW init failed");
    return false;
  }

  if (!addPeer(LEFT_GLOVE_MAC)) {
    Serial.println("[HUB] Failed to add LEFT peer");
    return false;
  }
  if (!addPeer(RIGHT_GLOVE_MAC)) {
    Serial.println("[HUB] Failed to add RIGHT peer");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  if (!initEspNow()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("{\"schema\":\"hermes.hub.status\",\"status\":\"ready\"}");
}

void loop() {
  // Receive callback handles all incoming data.
  delay(2);
}
