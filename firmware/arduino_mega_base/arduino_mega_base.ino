#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

// =========================
// Motor and encoder pin configuration
// =========================
static const uint8_t LEFT_RPWM_PIN = 6;
static const uint8_t LEFT_LPWM_PIN = 5;
static const uint8_t RIGHT_RPWM_PIN = 8;
static const uint8_t RIGHT_LPWM_PIN = 7;

static const uint8_t LEFT_ENC_A_PIN = 2;
static const uint8_t LEFT_ENC_B_PIN = 3;
static const uint8_t RIGHT_ENC_A_PIN = 19;
static const uint8_t RIGHT_ENC_B_PIN = 18;

// =========================
// Arm servo pin configuration
// =========================
static const uint8_t SERVO_1_PIN = 22;
static const uint8_t SERVO_2_LEFT_PIN = 23;
static const uint8_t SERVO_2_RIGHT_PIN = 24;
static const uint8_t SERVO_3_PIN = 25;
static const uint8_t SERVO_4_PIN = 26;
static const uint8_t SERVO_5_PIN = 27;

// =========================
// MPU6500 configuration
// =========================
const int MPU_ADDR = 0x68;
int16_t accelX = 0, accelY = 0, accelZ = 0;
int16_t gyroX = 0, gyroY = 0, gyroZ = 0;

// =========================
// System parameters
// =========================
static const float TELEMETRY_DT_SEC = 0.1f;
static const uint32_t CMD_TIMEOUT_MS = 500;
static const float WHEEL_SEPARATION_M = 0.42f;
static const float MAX_LINEAR_CMD_MPS = 0.16f;
static const float MAX_ANGULAR_CMD_RADPS = 0.85f;
static const int16_t MIN_DRIVE_PWM = 70;
static const int16_t MAX_DRIVE_PWM = 220;
static const bool LEFT_MOTOR_INVERTED = false;
static const bool RIGHT_MOTOR_INVERTED = false;

volatile int32_t g_left_ticks = 0;
volatile int32_t g_right_ticks = 0;

int16_t target_left_pwm = 0;
int16_t target_right_pwm = 0;

bool estop = false;
bool motor_enabled = true;
String mode = "MANUAL";

uint32_t last_cmd_ms = 0;
uint32_t last_telemetry_us = 0;

String usb_cmd_buffer;
String bt_cmd_buffer;
Servo g_servo_1, g_servo_2_left, g_servo_2_right, g_servo_3, g_servo_4, g_servo_5;

// =========================
// Joint and encoder helpers
// =========================
void setJoint2Angle(int deg) {
  deg = constrain(deg, 0, 180);
  g_servo_2_left.write(deg);
  g_servo_2_right.write(180 - deg);
}

void leftEncISR() {
  bool a = digitalRead(LEFT_ENC_A_PIN);
  bool b = digitalRead(LEFT_ENC_B_PIN);
  g_left_ticks += (a == b) ? 1 : -1;
}

void rightEncISR() {
  bool a = digitalRead(RIGHT_ENC_A_PIN);
  bool b = digitalRead(RIGHT_ENC_B_PIN);
  g_right_ticks += (a == b) ? 1 : -1;
}

// =========================
// MPU6500 accelerometer and gyro
// =========================
void setupMPU() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  // Wake up MPU6500.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(MPU_ADDR, 14, true);
    if (Wire.available() == 14) {
      accelX = Wire.read() << 8 | Wire.read();
      accelY = Wire.read() << 8 | Wire.read();
      accelZ = Wire.read() << 8 | Wire.read();
      Wire.read();
      Wire.read();  // Skip temperature sensor.
      gyroX = Wire.read() << 8 | Wire.read();
      gyroY = Wire.read() << 8 | Wire.read();
      gyroZ = Wire.read() << 8 | Wire.read();
    }
  }
}

// =========================
// Motor control
// =========================
int16_t wheelSpeedToPwm(float wheel_speed_mps) {
  if (fabs(wheel_speed_mps) < 0.001f) return 0;

  const float max_wheel_speed_mps =
      MAX_LINEAR_CMD_MPS + 0.5f * MAX_ANGULAR_CMD_RADPS * WHEEL_SEPARATION_M;
  float normalized = fabs(wheel_speed_mps) / max(max_wheel_speed_mps, 0.001f);
  normalized = constrain(normalized, 0.0f, 1.0f);

  int16_t pwm = MIN_DRIVE_PWM + (int16_t)((MAX_DRIVE_PWM - MIN_DRIVE_PWM) * normalized);
  if (wheel_speed_mps < 0.0f) pwm = -pwm;
  return constrain(pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
}

void setMotorPWM(int16_t left_pwm, int16_t right_pwm) {
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  if (LEFT_MOTOR_INVERTED) left_pwm = -left_pwm;
  if (RIGHT_MOTOR_INVERTED) right_pwm = -right_pwm;

  if (!motor_enabled || estop) {
    left_pwm = 0;
    right_pwm = 0;
  }

  if (left_pwm >= 0) {
    analogWrite(LEFT_RPWM_PIN, left_pwm);
    analogWrite(LEFT_LPWM_PIN, 0);
  } else {
    analogWrite(LEFT_RPWM_PIN, 0);
    analogWrite(LEFT_LPWM_PIN, -left_pwm);
  }

  if (right_pwm >= 0) {
    analogWrite(RIGHT_RPWM_PIN, right_pwm);
    analogWrite(RIGHT_LPWM_PIN, 0);
  } else {
    analogWrite(RIGHT_RPWM_PIN, 0);
    analogWrite(RIGHT_LPWM_PIN, -right_pwm);
  }
}

void stopMotors() {
  target_left_pwm = 0;
  target_right_pwm = 0;
  setMotorPWM(0, 0);
}

// =========================
// Command parser for USB Serial and HC-05 Bluetooth
// =========================
void parseCommand(const String &line) {
  if (line.length() == 0) return;

  if (line.length() == 1) {
    char c = line[0];
    int16_t speed_pwm = 150;
    int16_t turn_pwm = 120;

    if (c == 'F') {
      target_left_pwm = speed_pwm;
      target_right_pwm = speed_pwm;
    } else if (c == 'B') {
      target_left_pwm = -speed_pwm;
      target_right_pwm = -speed_pwm;
    } else if (c == 'L') {
      target_left_pwm = -turn_pwm;
      target_right_pwm = turn_pwm;
    } else if (c == 'R') {
      target_left_pwm = turn_pwm;
      target_right_pwm = -turn_pwm;
    } else if (c == 'S') {
      target_left_pwm = 0;
      target_right_pwm = 0;
    }

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      mode = "MANUAL";
      last_cmd_ms = millis();
      setMotorPWM(target_left_pwm, target_right_pwm);
      return;
    }
  }

  int p1 = line.indexOf(',');
  String cmd = (p1 < 0) ? line : line.substring(0, p1);

  if (cmd == "CMD_VEL") {
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) return;
    float v = line.substring(p1 + 1, p2).toFloat();
    float w = line.substring(p2 + 1).toFloat();

    float left_wheel_mps = v - 0.5f * w * WHEEL_SEPARATION_M;
    float right_wheel_mps = v + 0.5f * w * WHEEL_SEPARATION_M;

    target_left_pwm = wheelSpeedToPwm(left_wheel_mps);
    target_right_pwm = wheelSpeedToPwm(right_wheel_mps);

    setMotorPWM(target_left_pwm, target_right_pwm);
    mode = "AUTO";
    last_cmd_ms = millis();
    return;
  }

  if (cmd == "ESTOP") {
    estop = (line.substring(p1 + 1).toInt() != 0);
    if (estop) stopMotors();
    return;
  }

  if (cmd == "MOTOR") {
    motor_enabled = (line.substring(p1 + 1).toInt() != 0);
    if (!motor_enabled) stopMotors();
    return;
  }

  if (cmd == "SERVO") {
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) return;
    int id = line.substring(p1 + 1, p2).toInt();
    int deg = constrain(line.substring(p2 + 1).toInt(), 0, 180);

    switch (id) {
      case 1:
        g_servo_1.write(deg);
        break;
      case 2:
        setJoint2Angle(deg);
        break;
      case 3:
        g_servo_3.write(deg);
        break;
      case 4:
        g_servo_4.write(deg);
        break;
      case 5:
        g_servo_5.write(deg);
        break;
    }
    return;
  }
}

void readCommandStream(Stream &stream, String &buffer) {
  while (stream.available() > 0) {
    char c = (char)stream.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        parseCommand(buffer);
        buffer = "";
      }
    } else {
      if (buffer.length() < 120) buffer += c;
    }
  }
}

void readCommands() {
  readCommandStream(Serial, usb_cmd_buffer);
  readCommandStream(Serial2, bt_cmd_buffer);
}

void checkSafetyLoop() {
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_left_pwm = 0;
    target_right_pwm = 0;
    setMotorPWM(0, 0);
  }
}

// =========================
// Telemetry to Pi and phone
// =========================
void sendTelemetryLine(Print &out, int32_t left_ticks, int32_t right_ticks) {
  out.print("STAT,");
  out.print(mode);
  out.print(",");
  out.print(estop ? 1 : 0);
  out.print(",");
  out.print(motor_enabled ? 1 : 0);
  out.print(",");
  out.print(left_ticks);
  out.print(",");
  out.print(right_ticks);
  out.print(",");
  out.print(accelX);
  out.print(",");
  out.print(accelY);
  out.print(",");
  out.print(accelZ);
  out.print(",");
  out.print(gyroX);
  out.print(",");
  out.print(gyroY);
  out.print(",");
  out.print(gyroZ);
  out.print(",");
  out.print(target_left_pwm);
  out.print(",");
  out.println(target_right_pwm);
}

void sendTelemetry() {
  uint32_t now_us = micros();
  if ((now_us - last_telemetry_us) * 1e-6f < TELEMETRY_DT_SEC) return;
  last_telemetry_us = now_us;

  noInterrupts();
  int32_t left_ticks = g_left_ticks;
  int32_t right_ticks = g_right_ticks;
  interrupts();

  readMPU();

  sendTelemetryLine(Serial, left_ticks, right_ticks);
  sendTelemetryLine(Serial2, left_ticks, right_ticks);
}

// =========================
// Setup and loop
// =========================
void setupPins() {
  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);

  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncISR, CHANGE);

  g_servo_1.attach(SERVO_1_PIN);
  g_servo_2_left.attach(SERVO_2_LEFT_PIN);
  g_servo_2_right.attach(SERVO_2_RIGHT_PIN);
  g_servo_3.attach(SERVO_3_PIN);
  g_servo_4.attach(SERVO_4_PIN);
  g_servo_5.attach(SERVO_5_PIN);

  g_servo_1.write(90);
  setJoint2Angle(90);
  g_servo_3.write(90);
  g_servo_4.write(90);
  g_servo_5.write(90);

  stopMotors();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  setupMPU();
  setupPins();
  usb_cmd_buffer.reserve(128);
  bt_cmd_buffer.reserve(128);

  last_cmd_ms = millis();
  last_telemetry_us = micros();
}

void loop() {
  readCommands();
  checkSafetyLoop();
  sendTelemetry();
}
