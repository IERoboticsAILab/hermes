#include <Arduino.h>

// Temporary diagnostic sketch for the right-glove FSR wiring.
// Flash this to the right glove ESP32, open Serial Monitor at 115200,
// and press each FSR to watch raw ADC values change.

// Match the real right-glove firmware. These are ADC1 pins, so they still
// work when the real firmware enables ESP-NOW/Wi-Fi.
static const int FSR_INDEX_PIN = 32;
static const int FSR_MIDDLE_PIN = 33;
static const int FSR_RING_PIN = 34;
static const int FSR_PINKY_PIN = 35;
static const int FSR_PRESS_THRESHOLD = 1200;
static const uint32_t PRINT_INTERVAL_MS = 100;

uint32_t last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);

  pinMode(FSR_INDEX_PIN, INPUT);
  pinMode(FSR_MIDDLE_PIN, INPUT);
  pinMode(FSR_RING_PIN, INPUT);
  pinMode(FSR_PINKY_PIN, INPUT);

  Serial.println("FSR raw diagnostic ready");
  Serial.println("Expected wiring: 3.3V -> FSR -> ADC pin -> 10k resistor -> GND");
  Serial.printf("Pins: INDEX=%d MIDDLE=%d RING=%d PINKY=%d threshold=%d\n",
                FSR_INDEX_PIN, FSR_MIDDLE_PIN, FSR_RING_PIN, FSR_PINKY_PIN, FSR_PRESS_THRESHOLD);
}

void loop() {
  uint32_t now = millis();
  if ((now - last_print_ms) < PRINT_INTERVAL_MS) {
    return;
  }
  last_print_ms = now;

  int index_raw = analogRead(FSR_INDEX_PIN);
  int middle_raw = analogRead(FSR_MIDDLE_PIN);
  int ring_raw = analogRead(FSR_RING_PIN);
  int pinky_raw = analogRead(FSR_PINKY_PIN);

  Serial.printf(
    "raw INDEX=%4d %s | MIDDLE=%4d %s | RING=%4d %s | PINKY=%4d %s\n",
    index_raw, index_raw > FSR_PRESS_THRESHOLD ? "PRESSED" : "-",
    middle_raw, middle_raw > FSR_PRESS_THRESHOLD ? "PRESSED" : "-",
    ring_raw, ring_raw > FSR_PRESS_THRESHOLD ? "PRESSED" : "-",
    pinky_raw, pinky_raw > FSR_PRESS_THRESHOLD ? "PRESSED" : "-"
  );
}
