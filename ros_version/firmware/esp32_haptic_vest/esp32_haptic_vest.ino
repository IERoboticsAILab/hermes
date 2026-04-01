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

// ---------------------------------------------------------------------------
// H.E.R.M.E.S vest ESP32 firmware
//
// Final combined role:
// - Receives left/right glove packets over ESP-NOW
// - Forwards validated glove packets to the Raspberry Pi over USB serial
// - Receives haptic motor commands from the Raspberry Pi over the same serial
// - Drives the 6 vest motors with a serial-command failsafe
//
// Serial protocol:
// - Pi -> vest ESP: ASCII lines like:
//     V1,<seq>,<m1>,<m2>,<m3>,<m4>,<m5>,<m6>
// - vest ESP -> Pi: newline-delimited JSON lines:
//     {"schema":"hermes.hub.v1", ...}
//
// Important:
// - Set LEFT_GLOVE_MAC and RIGHT_GLOVE_MAC to the real glove ESP32 STA MACs.
// - Keep the Pi-side serial baud rate aligned with SERIAL_BAUD.
// ---------------------------------------------------------------------------

static uint8_t LEFT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x44, 0x55, 0x66};
static uint8_t RIGHT_GLOVE_MAC[6] = {0xAA, 0xBB, 0xCC, 0x77, 0x88, 0x99};

static const uint8_t ESPNOW_CHANNEL = 1;
static const uint32_t SERIAL_BAUD = 921600;

static const int kMotorPinPairs[6][2] = {
  {4, 21},
  {18, 19},
  {14, 33},
  {27, 32},
  {25, 26},
  {22, 23},
};

static const unsigned long kMotorFailsafeTimeoutMs = 300;

String serialLine;
unsigned long lastMotorPacketMs = 0;
int motorLevels[6] = {0, 0, 0, 0, 0, 0};

bool macEquals(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

bool isKnownPeer(const uint8_t* mac) {
  return macEquals(mac, LEFT_GLOVE_MAC) || macEquals(mac, RIGHT_GLOVE_MAC);
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

  if (macEquals(mac, LEFT_GLOVE_MAC)) {
    return "L";
  }
  if (macEquals(mac, RIGHT_GLOVE_MAC)) {
    return "R";
  }
  return "?";
}

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

void stopAllMotors() {
  for (int i = 0; i < 6; ++i) {
    motorLevels[i] = 0;
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], LOW);
  }
}

void applyMotorLevels() {
  for (int i = 0; i < 6; ++i) {
    const bool on = motorLevels[i] > 0;
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], on ? HIGH : LOW);
  }
}

void handleMotorFrame(const String& frame) {
  // Expected ASCII frame:
  // V1,<seq>,<m1>,<m2>,<m3>,<m4>,<m5>,<m6>
  if (!frame.startsWith("V1,")) {
    return;
  }

  int parsed[8];
  int count = 0;
  int start = 0;
  int nextComma = -1;

  while (count < 8) {
    nextComma = frame.indexOf(',', start);
    String token;
    if (nextComma < 0) {
      token = frame.substring(start);
    } else {
      token = frame.substring(start, nextComma);
    }
    token.trim();

    if (count == 0) {
      if (token != "V1") {
        return;
      }
      parsed[count] = 1;
    } else {
      if (token.length() == 0) {
        return;
      }
      parsed[count] = token.toInt();
    }

    count++;
    if (nextComma < 0) {
      break;
    }
    start = nextComma + 1;
  }

  if (count != 8) {
    return;
  }

  for (int i = 0; i < 6; ++i) {
    motorLevels[i] = constrain(parsed[i + 2], 0, 255);
  }
  lastMotorPacketMs = millis();
}

void readSerialFrames() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleMotorFrame(serialLine);
      serialLine = "";
      continue;
    }

    if (serialLine.length() < 95) {
      serialLine += ch;
    } else {
      serialLine = "";
    }
  }
}

void forwardGlovePacket(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) {
    return;
  }
  if (!isKnownPeer(mac)) {
    return;
  }

  char payloadBuf[251];
  memcpy(payloadBuf, data, len);
  payloadBuf[len] = '\0';

  DynamicJsonDocument payloadDoc(512);
  DeserializationError err = deserializeJson(payloadDoc, payloadBuf);

  DynamicJsonDocument outDoc(1024);
  outDoc["schema"] = "hermes.hub.v1";
  outDoc["rx_ms"] = millis();
  outDoc["sender_mac"] = macToString(mac);

  if (err) {
    outDoc["valid_json"] = false;
    outDoc["glove_id"] = macEquals(mac, LEFT_GLOVE_MAC) ? "L" : "R";
    outDoc["raw"] = payloadBuf;
  } else {
    outDoc["valid_json"] = true;
    outDoc["glove_id"] = inferGloveId(mac, payloadDoc);
    outDoc["packet"] = payloadDoc.as<JsonObject>();
  }

  serializeJson(outDoc, Serial);
  Serial.println();
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info == nullptr) {
    return;
  }
  forwardGlovePacket(info->src_addr, data, len);
}
#else
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  forwardGlovePacket(mac, data, len);
}
#endif

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[VEST] ESP-NOW init failed");
    return false;
  }

  if (!addPeer(LEFT_GLOVE_MAC)) {
    Serial.println("[VEST] Failed to add LEFT glove peer");
    return false;
  }
  if (!addPeer(RIGHT_GLOVE_MAC)) {
    Serial.println("[VEST] Failed to add RIGHT glove peer");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  serialLine.reserve(96);
  delay(300);

  for (int i = 0; i < 6; ++i) {
    pinMode(kMotorPinPairs[i][0], OUTPUT);
    pinMode(kMotorPinPairs[i][1], OUTPUT);
  }

  stopAllMotors();
  lastMotorPacketMs = millis();

  if (!initEspNow()) {
    while (true) {
      delay(1000);
    }
  }

  DynamicJsonDocument readyDoc(256);
  readyDoc["schema"] = "hermes.hub.status";
  readyDoc["status"] = "ready";
  readyDoc["device"] = "vest_hub";
  readyDoc["serial_baud"] = SERIAL_BAUD;
  readyDoc["espnow_channel"] = ESPNOW_CHANNEL;
  serializeJson(readyDoc, Serial);
  Serial.println();
}

void loop() {
  readSerialFrames();

  if ((millis() - lastMotorPacketMs) > kMotorFailsafeTimeoutMs) {
    stopAllMotors();
  } else {
    applyMotorLevels();
  }

  delay(2);
}
