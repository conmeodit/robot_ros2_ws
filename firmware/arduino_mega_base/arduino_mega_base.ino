#include <Arduino.h>
#include <Servo.h>

// =========================
// Hardware Configuration
// =========================
static const uint8_t LEFT_RPWM_PIN = 5;
static const uint8_t LEFT_LPWM_PIN = 6;
static const uint8_t RIGHT_RPWM_PIN = 7;
static const uint8_t RIGHT_LPWM_PIN = 8;

// Optional BTS7960 enable pins. Set to -1 when tied HIGH in hardware.
static const int8_t LEFT_REN_PIN = -1;
static const int8_t LEFT_LEN_PIN = -1;
static const int8_t RIGHT_REN_PIN = -1;
static const int8_t RIGHT_LEN_PIN = -1;

static const uint8_t LEFT_ENC_A_PIN = 2;
static const uint8_t LEFT_ENC_B_PIN = 3;
static const uint8_t RIGHT_ENC_A_PIN = 18;
static const uint8_t RIGHT_ENC_B_PIN = 19;

// Arm servos (2 servos for lifting joint)
static const uint8_t SERVO_1_PIN = 22;
static const uint8_t SERVO_2_LEFT_PIN = 23;
static const uint8_t SERVO_2_RIGHT_PIN = 24;
static const uint8_t SERVO_3_PIN = 25;
static const uint8_t SERVO_4_PIN = 26;
static const uint8_t SERVO_5_PIN = 27;

// =========================
// Robot Parameters (adjust)
// =========================
static const float WHEEL_RADIUS_M = 0.065f;
static const float WHEEL_BASE_M = 0.32f;
static const int32_t TICKS_PER_REV = 600;

// =========================
// Control Parameters (adjust)
// =========================
static const float CONTROL_DT_SEC = 0.02f;   // 50 Hz
static const float TELEMETRY_DT_SEC = 0.05f; // 20 Hz
static const uint32_t CMD_TIMEOUT_MS = 500;

static const float KP = 55.0f;
static const float KI = 30.0f;
static const float KD = 1.0f;

static const float MAX_RPS = 5.0f;

volatile int32_t g_left_ticks = 0;
volatile int32_t g_right_ticks = 0;

float target_left_rps = 0.0f;
float target_right_rps = 0.0f;

float meas_left_rps = 0.0f;
float meas_right_rps = 0.0f;

float left_integral = 0.0f;
float right_integral = 0.0f;
float left_prev_err = 0.0f;
float right_prev_err = 0.0f;

bool estop = false;
bool motor_enabled = true;
bool fault = false;
String fault_text = "OK";
String mode = "AUTO";

uint32_t last_cmd_ms = 0;
uint32_t last_control_us = 0;
uint32_t last_telemetry_us = 0;

String cmd_buffer;
Servo g_servo_1;
Servo g_servo_2_left;
Servo g_servo_2_right;
Servo g_servo_3;
Servo g_servo_4;
Servo g_servo_5;

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

void setMotorPWM(int16_t left_pwm, int16_t right_pwm) {
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

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

void stopMotorsAndResetPID() {
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_prev_err = 0.0f;
  right_prev_err = 0.0f;
  target_left_rps = 0.0f;
  target_right_rps = 0.0f;
  setMotorPWM(0, 0);
}

void setCmdVel(float linear_x, float angular_z) {
  float v_left = linear_x - angular_z * (WHEEL_BASE_M * 0.5f);
  float v_right = linear_x + angular_z * (WHEEL_BASE_M * 0.5f);

  float wheel_circ = 2.0f * PI * WHEEL_RADIUS_M;
  target_left_rps = v_left / wheel_circ;
  target_right_rps = v_right / wheel_circ;

  target_left_rps = constrain(target_left_rps, -MAX_RPS, MAX_RPS);
  target_right_rps = constrain(target_right_rps, -MAX_RPS, MAX_RPS);
}

void parseCommand(const String &line) {
  if (line.length() == 0) {
    return;
  }

  int p1 = line.indexOf(',');
  String cmd = (p1 < 0) ? line : line.substring(0, p1);

  if (cmd == "CMD_VEL") {
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) {
      return;
    }

    float v = line.substring(p1 + 1, p2).toFloat();
    float w = line.substring(p2 + 1).toFloat();
    setCmdVel(v, w);
    last_cmd_ms = millis();
    return;
  }

  if (cmd == "ESTOP") {
    int val = line.substring(p1 + 1).toInt();
    estop = (val != 0);
    if (estop) {
      stopMotorsAndResetPID();
    }
    return;
  }

  if (cmd == "MOTOR") {
    int val = line.substring(p1 + 1).toInt();
    motor_enabled = (val != 0);
    if (!motor_enabled) {
      stopMotorsAndResetPID();
    }
    return;
  }

  if (cmd == "SERVO") {
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) {
      return;
    }

    int id = line.substring(p1 + 1, p2).toInt();
    int deg = line.substring(p2 + 1).toInt();
    deg = constrain(deg, 0, 180);

    switch (id) {
      case 1:
        g_servo_1.write(deg);
        break;
      case 2:
        g_servo_2_left.write(deg);
        g_servo_2_right.write(180 - deg);
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
      default:
        break;
    }
    return;
  }
}

void readCommands() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      if (cmd_buffer.length() > 0) {
        parseCommand(cmd_buffer);
        cmd_buffer = "";
      }
    } else {
      if (cmd_buffer.length() < 120) {
        cmd_buffer += c;
      }
    }
  }
}

int16_t pidStep(float target_rps, float meas_rps, float &integral, float &prev_err, float dt) {
  float err = target_rps - meas_rps;
  integral += err * dt;
  integral = constrain(integral, -8.0f, 8.0f);

  float deriv = (err - prev_err) / dt;
  prev_err = err;

  float u = KP * err + KI * integral + KD * deriv;
  int16_t pwm = (int16_t)constrain((int)lround(u), -255, 255);
  return pwm;
}

void controlLoop() {
  uint32_t now_us = micros();
  float dt = (now_us - last_control_us) * 1e-6f;
  if (dt < CONTROL_DT_SEC) {
    return;
  }
  last_control_us = now_us;

  static int32_t last_left_ticks = 0;
  static int32_t last_right_ticks = 0;

  noInterrupts();
  int32_t left_ticks = g_left_ticks;
  int32_t right_ticks = g_right_ticks;
  interrupts();

  int32_t dleft = left_ticks - last_left_ticks;
  int32_t dright = right_ticks - last_right_ticks;
  last_left_ticks = left_ticks;
  last_right_ticks = right_ticks;

  meas_left_rps = ((float)dleft / (float)TICKS_PER_REV) / dt;
  meas_right_rps = ((float)dright / (float)TICKS_PER_REV) / dt;

  bool cmd_timeout = (millis() - last_cmd_ms) > CMD_TIMEOUT_MS;
  if (cmd_timeout) {
    target_left_rps = 0.0f;
    target_right_rps = 0.0f;
  }

  if (!motor_enabled || estop) {
    stopMotorsAndResetPID();
    return;
  }

  int16_t left_pwm = pidStep(target_left_rps, meas_left_rps, left_integral, left_prev_err, dt);
  int16_t right_pwm = pidStep(target_right_rps, meas_right_rps, right_integral, right_prev_err, dt);
  setMotorPWM(left_pwm, right_pwm);
}

void sendTelemetry() {
  uint32_t now_us = micros();
  float dt = (now_us - last_telemetry_us) * 1e-6f;
  if (dt < TELEMETRY_DT_SEC) {
    return;
  }
  last_telemetry_us = now_us;

  noInterrupts();
  int32_t left_ticks = g_left_ticks;
  int32_t right_ticks = g_right_ticks;
  interrupts();

  // Format expected by Pi bridge:
  // STAT,mode,estop,motor_enabled,left_ticks,right_ticks,batt_v,batt_i,fault,fault_text
  Serial2.print("STAT,");
  Serial2.print(mode);
  Serial2.print(',');
  Serial2.print(estop ? 1 : 0);
  Serial2.print(',');
  Serial2.print(motor_enabled ? 1 : 0);
  Serial2.print(',');
  Serial2.print(left_ticks);
  Serial2.print(',');
  Serial2.print(right_ticks);
  Serial2.print(',');
  Serial2.print(-1.0f, 2); // battery voltage placeholder
  Serial2.print(',');
  Serial2.print(-1.0f, 2); // battery current placeholder
  Serial2.print(',');
  Serial2.print(fault ? 1 : 0);
  Serial2.print(',');
  Serial2.println(fault_text);
}

void setupPins() {
  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);

  if (LEFT_REN_PIN >= 0) {
    pinMode(LEFT_REN_PIN, OUTPUT);
    digitalWrite(LEFT_REN_PIN, HIGH);
  }
  if (LEFT_LEN_PIN >= 0) {
    pinMode(LEFT_LEN_PIN, OUTPUT);
    digitalWrite(LEFT_LEN_PIN, HIGH);
  }
  if (RIGHT_REN_PIN >= 0) {
    pinMode(RIGHT_REN_PIN, OUTPUT);
    digitalWrite(RIGHT_REN_PIN, HIGH);
  }
  if (RIGHT_LEN_PIN >= 0) {
    pinMode(RIGHT_LEN_PIN, OUTPUT);
    digitalWrite(RIGHT_LEN_PIN, HIGH);
  }

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

  // Initial neutral arm pose
  g_servo_1.write(90);
  g_servo_2_left.write(90);
  g_servo_2_right.write(90);
  g_servo_3.write(90);
  g_servo_4.write(90);
  g_servo_5.write(90);

  setMotorPWM(0, 0);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600); // HC-05 on Mega Serial2 (D16/D17) - HC-05 default baud rate

  setupPins();
  cmd_buffer.reserve(128);

  last_cmd_ms = millis();
  last_control_us = micros();
  last_telemetry_us = micros();
}

void loop() {
  readCommands();
  controlLoop();
  sendTelemetry();
}
