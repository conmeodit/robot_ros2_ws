#include <Arduino.h>

// Motor pins match firmware/arduino_mega_base/arduino_mega_base.ino
static const uint8_t LEFT_RPWM_PIN = 6;
static const uint8_t LEFT_LPWM_PIN = 5;
static const uint8_t RIGHT_RPWM_PIN = 8;
static const uint8_t RIGHT_LPWM_PIN = 7;

static const int16_t DEFAULT_PWM = 110;
static const uint32_t CMD_TIMEOUT_MS = 700;

static const bool LEFT_MOTOR_INVERTED = false;
static const bool RIGHT_MOTOR_INVERTED = false;

int16_t target_left_pwm = 0;
int16_t target_right_pwm = 0;
bool motor_enabled = true;
bool estop = false;
uint32_t last_cmd_ms = 0;
uint32_t last_status_ms = 0;

String usb_buffer;
String bt_buffer;

int16_t clampPwm(int value) {
  return constrain(value, -255, 255);
}

void applyMotorPWM(int16_t left_pwm, int16_t right_pwm) {
  left_pwm = clampPwm(left_pwm);
  right_pwm = clampPwm(right_pwm);

  target_left_pwm = left_pwm;
  target_right_pwm = right_pwm;

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
  applyMotorPWM(0, 0);
}

void printHelp(Stream &out) {
  out.println(F("Mega motor test commands:"));
  out.println(F("  F/B/L/R/S          forward/back/left/right/stop"));
  out.println(F("  PWM,<left>,<right> set raw PWM -255..255"));
  out.println(F("  TEST               short safe sequence"));
  out.println(F("  MOTOR,1|0          enable/disable motor output"));
  out.println(F("  ESTOP,1|0          emergency stop on/off"));
  out.println(F("  HELP               show this help"));
}

void publishStatus() {
  Serial.print(F("STAT,"));
  Serial.print(motor_enabled ? 1 : 0);
  Serial.print(F(","));
  Serial.print(estop ? 1 : 0);
  Serial.print(F(","));
  Serial.print(target_left_pwm);
  Serial.print(F(","));
  Serial.println(target_right_pwm);

  Serial2.print(F("STAT,"));
  Serial2.print(motor_enabled ? 1 : 0);
  Serial2.print(F(","));
  Serial2.print(estop ? 1 : 0);
  Serial2.print(F(","));
  Serial2.print(target_left_pwm);
  Serial2.print(F(","));
  Serial2.println(target_right_pwm);
}

void runStep(int16_t left_pwm, int16_t right_pwm, uint32_t ms) {
  applyMotorPWM(left_pwm, right_pwm);
  last_cmd_ms = millis();
  uint32_t start = millis();
  while (millis() - start < ms) {
    publishStatus();
    delay(100);
  }
  stopMotors();
  delay(250);
}

void runTestSequence() {
  Serial.println(F("Running test sequence. Keep wheels off the ground first."));
  runStep(DEFAULT_PWM, DEFAULT_PWM, 700);
  runStep(-DEFAULT_PWM, -DEFAULT_PWM, 700);
  runStep(-DEFAULT_PWM, DEFAULT_PWM, 700);
  runStep(DEFAULT_PWM, -DEFAULT_PWM, 700);
  stopMotors();
  Serial.println(F("Test sequence done."));
}

void parseCommand(String line, Stream &out) {
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (line == "HELP" || line == "?") {
    printHelp(out);
    return;
  }

  if (line == "TEST") {
    runTestSequence();
    return;
  }

  if (line.length() == 1) {
    char c = line.charAt(0);
    if (c == 'F') {
      applyMotorPWM(DEFAULT_PWM, DEFAULT_PWM);
    } else if (c == 'B') {
      applyMotorPWM(-DEFAULT_PWM, -DEFAULT_PWM);
    } else if (c == 'L') {
      applyMotorPWM(-DEFAULT_PWM, DEFAULT_PWM);
    } else if (c == 'R') {
      applyMotorPWM(DEFAULT_PWM, -DEFAULT_PWM);
    } else if (c == 'S') {
      stopMotors();
    } else {
      out.println(F("ERR,unknown one-letter command"));
      return;
    }
    last_cmd_ms = millis();
    publishStatus();
    return;
  }

  int p1 = line.indexOf(',');
  String cmd = (p1 < 0) ? line : line.substring(0, p1);

  if (cmd == "PWM") {
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) {
      out.println(F("ERR,use PWM,<left>,<right>"));
      return;
    }
    int left_pwm = line.substring(p1 + 1, p2).toInt();
    int right_pwm = line.substring(p2 + 1).toInt();
    applyMotorPWM(left_pwm, right_pwm);
    last_cmd_ms = millis();
    publishStatus();
    return;
  }

  if (cmd == "MOTOR") {
    motor_enabled = line.substring(p1 + 1).toInt() != 0;
    if (!motor_enabled) stopMotors();
    publishStatus();
    return;
  }

  if (cmd == "ESTOP") {
    estop = line.substring(p1 + 1).toInt() != 0;
    if (estop) stopMotors();
    publishStatus();
    return;
  }

  out.println(F("ERR,unknown command"));
}

void readStream(Stream &stream, String &buffer) {
  while (stream.available() > 0) {
    char c = static_cast<char>(stream.read());
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        parseCommand(buffer, stream);
        buffer = "";
      }
    } else if (buffer.length() < 80) {
      buffer += c;
    }
  }
}

void setup() {
  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  Serial2.begin(9600);
  delay(1000);

  Serial.println(F("Mega motor test ready."));
  printHelp(Serial);
  last_cmd_ms = millis();
}

void loop() {
  readStream(Serial, usb_buffer);
  readStream(Serial2, bt_buffer);

  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (millis() - last_status_ms > 500) {
    last_status_ms = millis();
    publishStatus();
  }
}
