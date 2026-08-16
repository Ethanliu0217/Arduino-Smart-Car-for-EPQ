#include <string.h>
#include <Wire.h>

namespace {

// New I2C four-motor driver register map.
constexpr uint8_t kDriverAddress = 0x26;
constexpr uint8_t kMotorTypeRegister = 0x01;
constexpr uint8_t kMotorDeadZoneRegister = 0x02;
constexpr uint8_t kMotorPulseLineRegister = 0x03;
constexpr uint8_t kMotorReductionRatioRegister = 0x04;
constexpr uint8_t kWheelDiameterRegister = 0x05;
constexpr uint8_t kSpeedControlRegister = 0x06;
constexpr uint8_t kPwmControlRegister = 0x07;
constexpr uint8_t kEncoder10msRegisters[] = {0x10, 0x11, 0x12, 0x13};
constexpr uint8_t kEncoderTotalHighRegisters[] = {0x20, 0x22, 0x24, 0x26};
constexpr uint8_t kEncoderTotalLowRegisters[] = {0x21, 0x23, 0x25, 0x27};

constexpr uint8_t kUltrasonicEchoPin = 9;
constexpr uint8_t kUltrasonicTrigPin = 10;
constexpr unsigned long kUltrasonicIntervalMs = 250;
constexpr unsigned long kUltrasonicTimeoutUs = 30000;
constexpr float kObstacleStopDistanceCm = 20.0F;

constexpr uint8_t kLineLeftPin = 11;
constexpr uint8_t kLineRightPin = 12;
constexpr unsigned long kLineSensorIntervalMs = 50;

constexpr uint8_t kInfraredLeftPin = 3;
constexpr uint8_t kInfraredRightPin = 4;
constexpr unsigned long kInfraredSensorIntervalMs = 50;

// Yahboom type 3: encoder TT motor. Closed-loop speed is required; PWM is type 4 only.
constexpr uint8_t kTtMotorType = 3;
constexpr uint16_t kTtMotorPulseLine = 13;
constexpr uint16_t kTtMotorReductionRatio = 45;
constexpr uint16_t kTtMotorDeadZone = 1250;
constexpr float kTtWheelDiameterMm = 68.0F;
constexpr int16_t kTestPwm = 1900;
constexpr int16_t kTestSpeed = 800;
constexpr unsigned long kDriverConfigDelayMs = 100;
constexpr unsigned long kTestDurationMs = 5000;
constexpr unsigned long kEncoderMonitorIntervalMs = 200;

bool driverReady = false;
bool encoderMonitorEnabled = false;
bool ultrasonicMonitorEnabled = true;
bool motorsRunning = false;
bool obstacleDetected = false;
unsigned long lastEncoderMonitorMs = 0;
unsigned long lastUltrasonicMonitorMs = 0;
unsigned long lastLineSensorMs = 0;
int8_t lastLineLeftState = -1;
int8_t lastLineRightState = -1;
unsigned long lastInfraredSensorMs = 0;
int8_t lastInfraredLeftState = -1;
int8_t lastInfraredRightState = -1;

void printLineSensorState(bool forcePrint = false) {
  const int8_t leftState = static_cast<int8_t>(digitalRead(kLineLeftPin));
  const int8_t rightState = static_cast<int8_t>(digitalRead(kLineRightPin));

  if (!forcePrint && leftState == lastLineLeftState &&
      rightState == lastLineRightState) {
    return;
  }

  lastLineLeftState = leftState;
  lastLineRightState = rightState;

  Serial.print(F("Line sensors [LO,RO]: "));
  Serial.print(leftState);
  Serial.print(F(", "));
  Serial.println(rightState);
}

void printInfraredSensorState(bool forcePrint = false) {
  const int8_t leftState = static_cast<int8_t>(digitalRead(kInfraredLeftPin));
  const int8_t rightState =
      static_cast<int8_t>(digitalRead(kInfraredRightPin));

  if (!forcePrint && leftState == lastInfraredLeftState &&
      rightState == lastInfraredRightState) {
    return;
  }

  lastInfraredLeftState = leftState;
  lastInfraredRightState = rightState;

  Serial.print(F("Infrared sensors [L,R]: "));
  Serial.print(leftState);
  Serial.print(F(", "));
  Serial.println(rightState);
}

bool readAndPrintUltrasonicDistance(float &distanceCm) {
  digitalWrite(kUltrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(kUltrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kUltrasonicTrigPin, LOW);

  const unsigned long echoDurationUs =
      pulseIn(kUltrasonicEchoPin, HIGH, kUltrasonicTimeoutUs);

  if (echoDurationUs == 0) {
    Serial.println(F("Distance: no echo (out of range or wiring error)"));
    return false;
  }

  distanceCm = echoDurationUs * 0.0343F / 2.0F;
  //Serial.print(F("Distance: "));
  //Serial.print(distanceCm, 1);
  //Serial.println(F(" cm"));
  return true;
}

bool writeRegister(uint8_t registerAddress, const uint8_t *data,
                   uint8_t dataLength) {
  Wire.beginTransmission(kDriverAddress);
  Wire.write(registerAddress);

  for (uint8_t i = 0; i < dataLength; ++i) {
    Wire.write(data[i]);
  }

  return Wire.endTransmission() == 0;
}

bool writeByte(uint8_t registerAddress, uint8_t value) {
  return writeRegister(registerAddress, &value, 1);
}

bool writeMotorType(uint8_t motorType) {
  // The vendor example sends a two-byte motor-type configuration frame.
  const uint8_t data[] = {motorType, 0};
  return writeRegister(kMotorTypeRegister, data, sizeof(data));
}

bool writeWord(uint8_t registerAddress, uint16_t value) {
  const uint8_t data[] = {static_cast<uint8_t>(value >> 8),
                          static_cast<uint8_t>(value)};
  return writeRegister(registerAddress, data, sizeof(data));
}

bool writeFloat(uint8_t registerAddress, float value) {
  uint8_t data[sizeof(float)];
  memcpy(data, &value, sizeof(data));
  return writeRegister(registerAddress, data, sizeof(data));
}

bool readRegister(uint8_t registerAddress, uint8_t *data,
                  uint8_t dataLength) {
  Wire.beginTransmission(kDriverAddress);
  Wire.write(registerAddress);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(kDriverAddress, dataLength);
  if (received != dataLength) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < dataLength; ++i) {
    data[i] = Wire.read();
  }

  return true;
}

bool readSignedWord(uint8_t registerAddress, int16_t &value) {
  uint8_t data[2];
  if (!readRegister(registerAddress, data, sizeof(data))) {
    return false;
  }

  const uint16_t rawValue =
      (static_cast<uint16_t>(data[0]) << 8) | data[1];
  value = static_cast<int16_t>(rawValue);
  return true;
}

bool readEncoder10ms(int16_t encoderValues[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    if (!readSignedWord(kEncoder10msRegisters[i], encoderValues[i])) {
      return false;
    }
  }

  return true;
}

bool readEncoderTotal(int32_t encoderValues[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    int16_t highWord = 0;
    int16_t lowWord = 0;
    if (!readSignedWord(kEncoderTotalHighRegisters[i], highWord) ||
        !readSignedWord(kEncoderTotalLowRegisters[i], lowWord)) {
      return false;
    }

    const uint32_t rawValue =
        (static_cast<uint32_t>(static_cast<uint16_t>(highWord)) << 16) |
        static_cast<uint16_t>(lowWord);
    encoderValues[i] = static_cast<int32_t>(rawValue);
  }

  return true;
}

void printEncoder10ms() {
  int16_t encoderValues[4];
  if (!readEncoder10ms(encoderValues)) {
    driverReady = false;
    Serial.println(F("Encoder 10ms read failed."));
    return;
  }

  Serial.print(F("Encoder 10ms [M1,M2,M3,M4]: "));
  for (uint8_t i = 0; i < 4; ++i) {
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(encoderValues[i]);
  }
  Serial.println();
}

void printEncoderTotal() {
  int32_t encoderValues[4];
  if (!readEncoderTotal(encoderValues)) {
    driverReady = false;
    Serial.println(F("Encoder total read failed."));
    return;
  }

  Serial.print(F("Encoder total [M1,M2,M3,M4]: "));
  for (uint8_t i = 0; i < 4; ++i) {
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(static_cast<long>(encoderValues[i]));
  }
  Serial.println();
}

bool setMotorPwm(int16_t motor1, int16_t motor2, int16_t motor3,
                 int16_t motor4) {
  const int16_t motorValues[] = {motor1, motor2, motor3, motor4};
  uint8_t pwmValues[8];

  for (uint8_t i = 0; i < 4; ++i) {
    const uint16_t value = static_cast<uint16_t>(motorValues[i]);
    pwmValues[i * 2] = static_cast<uint8_t>(value >> 8);
    pwmValues[i * 2 + 1] = static_cast<uint8_t>(value);
  }

  const bool written =
      writeRegister(kPwmControlRegister, pwmValues, sizeof(pwmValues));
  if (written) {
    motorsRunning = motor1 != 0 || motor2 != 0 || motor3 != 0 || motor4 != 0;
  }
  return written;
}

bool setMotorSpeed(int16_t motor1, int16_t motor2, int16_t motor3,
                   int16_t motor4) {
  const int16_t motorValues[] = {motor1, motor2, motor3, motor4};
  uint8_t speedValues[8];

  for (uint8_t i = 0; i < 4; ++i) {
    const uint16_t value = static_cast<uint16_t>(motorValues[i]);
    speedValues[i * 2] = static_cast<uint8_t>(value >> 8);
    speedValues[i * 2 + 1] = static_cast<uint8_t>(value);
  }

  const bool written =
      writeRegister(kSpeedControlRegister, speedValues, sizeof(speedValues));
  if (written) {
    motorsRunning = motor1 != 0 || motor2 != 0 || motor3 != 0 || motor4 != 0;
  }
  return written;
}

bool stopAllMotors() {
  // Type 3 uses the speed register; also clear PWM in case a previous open-loop test ran.
  const bool speedStopped = setMotorSpeed(0, 0, 0, 0);
  const bool pwmStopped = setMotorPwm(0, 0, 0, 0);
  motorsRunning = false;
  return speedStopped || pwmStopped;
}

bool checkUltrasonicSafety() {
  float distanceCm = 0.0F;
  if (!readAndPrintUltrasonicDistance(distanceCm)) {
    return false;
  }

  const bool tooClose = distanceCm < kObstacleStopDistanceCm;

  if (tooClose && !obstacleDetected) {
//    Serial.println(F("WARNING: obstacle closer than 20 cm."));
  } else if (!tooClose && obstacleDetected) {
//    Serial.println(F("Obstacle cleared; distance is at least 20 cm."));
  }
  obstacleDetected = tooClose;

  if (tooClose && motorsRunning) {
    driverReady = stopAllMotors();
    Serial.println(driverReady ? F("SAFETY STOP: all motors stopped.")
                               : F("WARNING: safety stop command failed."));
  }

  return tooClose;
}

bool probeDriver() {
  Wire.beginTransmission(kDriverAddress);
  return Wire.endTransmission() == 0;
}

void printI2cStatus() {
  driverReady = probeDriver();
  Serial.print(F("I2C driver 0x26: "));
  Serial.println(driverReady ? F("FOUND") : F("NOT FOUND"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  i - probe I2C driver"));
  Serial.println(F("  1-4 - test motor M1-M4 at speed 800 for 5 s"));
  Serial.println(F("  b - test wheel motors M2 and M4 at speed 800 for 5 s"));
  Serial.println(F("  v - same as b (closed-loop speed)"));
  Serial.println(F("  p - test wheel motors M2 and M4 at PWM 1900 for 5 s"));
  Serial.println(F("  e - read the four 10ms encoder counts"));
  Serial.println(F("  t - read the four total encoder counts"));
  Serial.println(F("  m - toggle encoder monitoring every 200ms"));
  Serial.println(F("  u - toggle ultrasonic monitoring every 250ms"));
  Serial.println(F("  l - print line sensor LO/RO states"));
  Serial.println(F("  o - print infrared obstacle sensor states"));
  Serial.println(F("  s - stop all motors"));
  Serial.println(F("  h - show this help"));
}

void runMotorTest(char motorSelection, bool speedMode = true) {
  if (!driverReady) {
    printI2cStatus();
  }

  if (!driverReady) {
    Serial.println(F("Test blocked: motor driver is not responding."));
    return;
  }

  const int16_t testValue = speedMode ? kTestSpeed : kTestPwm;
  const int16_t motor1 = (motorSelection == '1') ? testValue : 0;
  const int16_t motor2 =
      (motorSelection == '2' || motorSelection == 'B' ||
       motorSelection == 'V')
          ? testValue
          : 0;
  const int16_t motor3 = (motorSelection == '3') ? testValue : 0;
  const int16_t motor4 =
      (motorSelection == '4' || motorSelection == 'B' ||
       motorSelection == 'V')
          ? testValue
          : 0;
  const bool started = speedMode
                           ? setMotorSpeed(motor1, motor2, motor3, motor4)
                           : setMotorPwm(motor1, motor2, motor3, motor4);

  if (!started) {
    driverReady = false;
    Serial.println(F("Test failed: I2C write error."));
    stopAllMotors();
    return;
  }

  if (motorSelection == 'B' || motorSelection == 'V') {
    Serial.print(F("Wheel motors M2 and M4 running at "));
    Serial.print(speedMode ? F("speed ") : F("PWM "));
    Serial.print(speedMode ? kTestSpeed : kTestPwm);
    Serial.println(F("."));
  } else {
    Serial.print(F("Motor M"));
    Serial.print(motorSelection);
    Serial.print(speedMode ? F(" running at speed ") : F(" running at PWM "));
    Serial.print(speedMode ? kTestSpeed : kTestPwm);
    Serial.println(F("."));
  }

  const unsigned long testStartMs = millis();
  unsigned long lastSampleMs = testStartMs - kEncoderMonitorIntervalMs;
  unsigned long lastUltrasonicSampleMs = testStartMs;
  unsigned long lastLineSampleMs = testStartMs - kLineSensorIntervalMs;
  unsigned long lastInfraredSampleMs =
      testStartMs - kInfraredSensorIntervalMs;

  while (millis() - testStartMs < kTestDurationMs) {
    if (millis() - lastInfraredSampleMs >= kInfraredSensorIntervalMs) {
      lastInfraredSampleMs = millis();
      printInfraredSensorState();
    }

    if (millis() - lastLineSampleMs >= kLineSensorIntervalMs) {
      lastLineSampleMs = millis();
      printLineSensorState();
    }

    if (ultrasonicMonitorEnabled &&
        millis() - lastUltrasonicSampleMs >= kUltrasonicIntervalMs) {
      lastUltrasonicSampleMs = millis();
      if (checkUltrasonicSafety()) {
        if (motorsRunning) {
          delay(10);
          driverReady = stopAllMotors();
        }
        Serial.println(driverReady
                           ? F("Motor test cancelled by ultrasonic safety.")
                           : F("DANGER: motor stop was not acknowledged."));
        return;
      }
    }

    if (millis() - lastSampleMs >= kEncoderMonitorIntervalMs) {
      lastSampleMs = millis();
      printEncoder10ms();
      printEncoderTotal();
    }
    delay(1);
  }

  driverReady = speedMode ? setMotorSpeed(0, 0, 0, 0) : stopAllMotors();
  if (driverReady) {
    Serial.println(F("Test complete. All motors stopped."));
  } else {
    Serial.println(F("Warning: stop command was not acknowledged."));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(kUltrasonicTrigPin, OUTPUT);
  pinMode(kUltrasonicEchoPin, INPUT);
  digitalWrite(kUltrasonicTrigPin, LOW);
  pinMode(kLineLeftPin, INPUT);
  pinMode(kLineRightPin, INPUT);
  pinMode(kInfraredLeftPin, INPUT);
  pinMode(kInfraredRightPin, INPUT);

  delay(250);
  Serial.println(F("Arduino Uno I2C motor test ready."));
  printI2cStatus();

  if (driverReady) {
    const bool typeConfigured = writeMotorType(kTtMotorType);
    delay(kDriverConfigDelayMs);
    const bool ratioConfigured =
        writeWord(kMotorReductionRatioRegister, kTtMotorReductionRatio);
    delay(kDriverConfigDelayMs);
    const bool pulseLineConfigured =
        writeWord(kMotorPulseLineRegister, kTtMotorPulseLine);
    delay(kDriverConfigDelayMs);
    const bool wheelConfigured =
        writeFloat(kWheelDiameterRegister, kTtWheelDiameterMm);
    delay(kDriverConfigDelayMs);
    const bool deadZoneConfigured =
        writeWord(kMotorDeadZoneRegister, kTtMotorDeadZone);
    delay(kDriverConfigDelayMs);
    const bool stopped = stopAllMotors();

    driverReady = typeConfigured && ratioConfigured && pulseLineConfigured &&
                  wheelConfigured && deadZoneConfigured && stopped;
    Serial.println(driverReady ? F("Driver configured as TT encoder (type 3); motors stopped.")
                               : F("Driver configuration failed."));
  }

  printHelp();
  Serial.println(F("Ultrasonic monitoring ON (TRIG=10, ECHO=9)."));
  Serial.println(F("Line sensors ready (LO=11, RO=12)."));
  printLineSensorState(true);
  Serial.println(F("Infrared obstacle sensors ready (L=3, R=4)."));
  printInfraredSensorState(true);
}

void loop() {
  if (millis() - lastInfraredSensorMs >= kInfraredSensorIntervalMs) {
    lastInfraredSensorMs = millis();
    printInfraredSensorState();
  }

  if (millis() - lastLineSensorMs >= kLineSensorIntervalMs) {
    lastLineSensorMs = millis();
    printLineSensorState();
  }

  if (ultrasonicMonitorEnabled &&
      millis() - lastUltrasonicMonitorMs >= kUltrasonicIntervalMs) {
    lastUltrasonicMonitorMs = millis();
    checkUltrasonicSafety();
  }

  if (encoderMonitorEnabled &&
      millis() - lastEncoderMonitorMs >= kEncoderMonitorIntervalMs) {
    lastEncoderMonitorMs = millis();
    printEncoder10ms();
    printEncoderTotal();

    if (!driverReady) {
      encoderMonitorEnabled = false;
      Serial.println(F("Encoder monitoring stopped after an I2C error."));
    }
  }

  if (!Serial.available()) {
    return;
  }

  const char command = static_cast<char>(Serial.read());

  switch (command) {
    case 'i':
    case 'I':
      printI2cStatus();
      break;
    case '1':
      runMotorTest('1');
      break;
    case '2':
      runMotorTest('2');
      break;
    case '3':
      runMotorTest('3');
      break;
    case '4':
      runMotorTest('4');
      break;
    case 'b':
    case 'B':
      runMotorTest('B');
      break;
    case 'v':
    case 'V':
      runMotorTest('V', true);
      break;
    case 'p':
    case 'P':
      runMotorTest('B', false);
      break;
    case 'e':
    case 'E':
      printEncoder10ms();
      break;
    case 't':
    case 'T':
      printEncoderTotal();
      break;
    case 'r':
    case 'R':
      Serial.println(
          F("Encoder reset is not supported by this driver register map."));
      break;
    case 'm':
    case 'M':
      encoderMonitorEnabled = !encoderMonitorEnabled;
      lastEncoderMonitorMs = millis() - kEncoderMonitorIntervalMs;
      Serial.println(encoderMonitorEnabled ? F("Encoder monitoring ON.")
                                           : F("Encoder monitoring OFF."));
      break;
    case 'u':
    case 'U':
      ultrasonicMonitorEnabled = !ultrasonicMonitorEnabled;
      lastUltrasonicMonitorMs = millis() - kUltrasonicIntervalMs;
      Serial.println(ultrasonicMonitorEnabled
                         ? F("Ultrasonic monitoring ON.")
                         : F("Ultrasonic monitoring OFF."));
      break;
    case 'l':
    case 'L':
      printLineSensorState(true);
      break;
    case 'o':
    case 'O':
      printInfraredSensorState(true);
      break;
    case 's':
    case 'S':
      driverReady = stopAllMotors();
      Serial.println(driverReady ? F("All motors stopped.")
                                 : F("Stop command failed."));
      break;
    case 'h':
    case 'H':
      printHelp();
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.println(F("Unknown command. Send h for help."));
      break;
  }
}
