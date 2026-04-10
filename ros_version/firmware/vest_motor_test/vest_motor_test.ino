#include <Arduino.h>

// ---------------------------------------------------------------------------
// Vest motor hardware test
//
// Purpose:
// - Isolate vest motor wiring/driver/power issues from ROS, serial, and ESP-NOW
// - Use the same motor pin pairs and drive style as the main vest firmware
//
// Behavior:
// - Turns on one motor channel at a time for 2 seconds
// - Pauses 0.5 seconds between motors
// - After all six, turns all motors on together for 3 seconds
// - Repeats forever
//
// Serial monitor:
// - 115200 baud
// ---------------------------------------------------------------------------

static const int kMotorPinPairs[6][2] = {
  {4, 21},
  {18, 19},
  {14, 33},
  {27, 32},
  {25, 26},
  {22, 23},
};

static const uint32_t kSingleMotorOnMs = 2000;
static const uint32_t kInterMotorOffMs = 500;
static const uint32_t kAllMotorsOnMs = 3000;
static const uint32_t kCyclePauseMs = 1000;

void stopAllMotors() {
  for (int i = 0; i < 6; ++i) {
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], LOW);
  }
}

void setMotorOn(int idx) {
  stopAllMotors();
  digitalWrite(kMotorPinPairs[idx][0], LOW);
  digitalWrite(kMotorPinPairs[idx][1], HIGH);
}

void setAllMotorsOn() {
  for (int i = 0; i < 6; ++i) {
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 6; ++i) {
    pinMode(kMotorPinPairs[i][0], OUTPUT);
    pinMode(kMotorPinPairs[i][1], OUTPUT);
  }

  stopAllMotors();

  Serial.println("Vest motor test ready");
  Serial.println("Pattern: motor 1..6 individually, then all together");
}

void loop() {
  for (int i = 0; i < 6; ++i) {
    Serial.printf("Motor %d ON (pins %d,%d)\n", i + 1, kMotorPinPairs[i][0], kMotorPinPairs[i][1]);
    setMotorOn(i);
    delay(kSingleMotorOnMs);

    Serial.printf("Motor %d OFF\n", i + 1);
    stopAllMotors();
    delay(kInterMotorOffMs);
  }

  Serial.println("All motors ON");
  setAllMotorsOn();
  delay(kAllMotorsOnMs);

  Serial.println("All motors OFF");
  stopAllMotors();
  delay(kCyclePauseMs);
}
