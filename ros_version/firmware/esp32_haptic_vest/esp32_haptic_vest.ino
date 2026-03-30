const int kMotorPinPairs[6][2] = {
  {4, 21},
  {18, 19},
  {14, 33},
  {27, 32},
  {25, 26},
  {22, 23},
};

const unsigned long kFailsafeTimeoutMs = 300;

String serialLine;
unsigned long lastPacketMs = 0;
int motorLevels[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  serialLine.reserve(96);

  for (int i = 0; i < 6; ++i) {
    pinMode(kMotorPinPairs[i][0], OUTPUT);
    pinMode(kMotorPinPairs[i][1], OUTPUT);
  }

  stopAllMotors();
  lastPacketMs = millis();
}

void loop() {
  readSerialFrames();

  if ((millis() - lastPacketMs) > kFailsafeTimeoutMs) {
    stopAllMotors();
  } else {
    applyMotorLevels();
  }
}

void readSerialFrames() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleFrame(serialLine);
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

void handleFrame(const String& frame) {
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
  lastPacketMs = millis();
}

void applyMotorLevels() {
  for (int i = 0; i < 6; ++i) {
    const bool on = motorLevels[i] > 0;
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], on ? HIGH : LOW);
  }
}

void stopAllMotors() {
  for (int i = 0; i < 6; ++i) {
    motorLevels[i] = 0;
    digitalWrite(kMotorPinPairs[i][0], LOW);
    digitalWrite(kMotorPinPairs[i][1], LOW);
  }
}
