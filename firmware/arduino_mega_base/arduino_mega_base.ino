#include <Arduino.h>
#include <Servo.h>
#include <Wire.h> 

// =========================
// Cấu hình phần cứng Động cơ & Encoder
// =========================
static const uint8_t LEFT_RPWM_PIN = 6;  
static const uint8_t LEFT_LPWM_PIN = 5;  
static const uint8_t RIGHT_RPWM_PIN = 8; 
static const uint8_t RIGHT_LPWM_PIN = 7; 

static const uint8_t LEFT_ENC_A_PIN = 2;
static const uint8_t LEFT_ENC_B_PIN = 3;
static const uint8_t RIGHT_ENC_A_PIN = 18;
static const uint8_t RIGHT_ENC_B_PIN = 19;

// =========================
// Cấu hình Arm Servos
// =========================
static const uint8_t SERVO_1_PIN = 22;
static const uint8_t SERVO_2_LEFT_PIN = 23;
static const uint8_t SERVO_2_RIGHT_PIN = 24;
static const uint8_t SERVO_3_PIN = 25;
static const uint8_t SERVO_4_PIN = 26;
static const uint8_t SERVO_5_PIN = 27;

// =========================
// MPU9250 & La Bàn AK8963 Configuration
// =========================
const int MPU_ADDR = 0x68; // Địa chỉ chip Gia tốc + Gyro
const int MAG_ADDR = 0x0C; // Địa chỉ chip La bàn AK8963

int16_t accelX = 0, accelY = 0, accelZ = 0;
int16_t gyroX = 0,  gyroY = 0,  gyroZ = 0;
int16_t magX = 0,   magY = 0,   magZ = 0; // Biến lưu dữ liệu la bàn

// =========================
// System Parameters
// =========================
static const float TELEMETRY_DT_SEC = 0.5f; // 2 lần/giây
static const uint32_t CMD_TIMEOUT_MS = 500;  

volatile int32_t g_left_ticks = 0;
volatile int32_t g_right_ticks = 0;

int16_t target_left_pwm = 0;
int16_t target_right_pwm = 0;

bool estop = false;
bool motor_enabled = true;
bool fault = false;
String fault_text = "OK";
String mode = "MANUAL"; 

uint32_t last_cmd_ms = 0;
uint32_t last_telemetry_us = 0;

String cmd_buffer;
Servo g_servo_1, g_servo_2_left, g_servo_2_right, g_servo_3, g_servo_4, g_servo_5;

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
// MPU9250 & AK8963 Setup & Read
// =========================
void setupMPU() {
  Wire.begin();
  Wire.setWireTimeout(3000, true); 

  // 1. Đánh thức MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Thanh ghi Nguồn
  Wire.write(0x00); // Xóa chế độ Sleep
  Wire.endTransmission(true);
  delay(50);

  // 2. BẬT CHẾ ĐỘ I2C BYPASS (Mở cửa cho Arduino đọc la bàn)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); // Thanh ghi INT_PIN_CFG
  Wire.write(0x02); // Bật bit số 1 (BYPASS_EN)
  Wire.endTransmission(true);
  delay(50);

  // 3. KHỞI TẠO LA BÀN AK8963
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); // Thanh ghi điều khiển CNTL1
  Wire.write(0x16); // Chế độ: Đo liên tục (100Hz), độ phân giải 16-bit
  Wire.endTransmission(true);
  delay(50);
}

void readMPU() {
  // --- 1. ĐỌC GIA TỐC VÀ GÓC NGHIÊNG ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(MPU_ADDR, 14, true); 
    if (Wire.available() == 14) {
      accelX = Wire.read() << 8 | Wire.read();
      accelY = Wire.read() << 8 | Wire.read();
      accelZ = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Bỏ qua nhiệt độ
      gyroX = Wire.read() << 8 | Wire.read();
      gyroY = Wire.read() << 8 | Wire.read();
      gyroZ = Wire.read() << 8 | Wire.read();
    }
  }

  // --- 2. ĐỌC LA BÀN TỪ TRƯỜNG AK8963 ---
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03); // Thanh ghi bắt đầu của La bàn (HXL)
  if (Wire.endTransmission(false) == 0) {
    // Yêu cầu đọc 7 byte (6 byte dữ liệu + 1 byte trạng thái cuối cùng)
    Wire.requestFrom(MAG_ADDR, 7, true); 
    if (Wire.available() == 7) {
      // AK8963 gửi byte THẤP (Low) trước, byte CAO (High) sau
      uint8_t hxl = Wire.read();
      uint8_t hxh = Wire.read();
      uint8_t hyl = Wire.read();
      uint8_t hyh = Wire.read();
      uint8_t hzl = Wire.read();
      uint8_t hzh = Wire.read();
      uint8_t st2 = Wire.read(); // Bắt buộc phải đọc byte ST2 để chốt dữ liệu

      // Ghép 2 byte lại thành số nguyên 16-bit có dấu
      magX = (int16_t)(hxh << 8 | hxl);
      magY = (int16_t)(hyh << 8 | hyl);
      magZ = (int16_t)(hzh << 8 | hzl);
    }
  }
}

// =========================
// Motor Control Functions 
// =========================
void setMotorPWM(int16_t left_pwm, int16_t right_pwm) {
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  if (!motor_enabled || estop) {
    left_pwm = 0; right_pwm = 0;
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
// Bluetooth HC-05 Parsing
// =========================
void parseCommand(const String &line) {
  if (line.length() == 0) return;

  if (line.length() == 1) {
    char c = line[0];
    int16_t speed_pwm = 150; 
    int16_t turn_pwm  = 120; 

    if (c == 'F') { target_left_pwm = speed_pwm;  target_right_pwm = speed_pwm; }
    else if (c == 'B') { target_left_pwm = -speed_pwm; target_right_pwm = -speed_pwm; }
    else if (c == 'L') { target_left_pwm = -turn_pwm;  target_right_pwm = turn_pwm; } 
    else if (c == 'R') { target_left_pwm = turn_pwm;   target_right_pwm = -turn_pwm; } 
    else if (c == 'S') { target_left_pwm = 0;          target_right_pwm = 0; }

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
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
    
    target_left_pwm = constrain((v - w) * 200, -255, 255);
    target_right_pwm = constrain((v + w) * 200, -255, 255);
    
    setMotorPWM(target_left_pwm, target_right_pwm);
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
      case 1: g_servo_1.write(deg); break;
      case 2: setJoint2Angle(deg);  break; 
      case 3: g_servo_3.write(deg); break;
      case 4: g_servo_4.write(deg); break;
      case 5: g_servo_5.write(deg); break;
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
      if (cmd_buffer.length() < 120) cmd_buffer += c;
    }
  }
}

void checkSafetyLoop() {
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_left_pwm = 0;
    target_right_pwm = 0;
    setMotorPWM(0, 0); 
  }
}

// =========================
// Telemetry (Gửi dữ liệu về HC-05)
// =========================
void sendTelemetry() {
  uint32_t now_us = micros();
  if ((now_us - last_telemetry_us) * 1e-6f < TELEMETRY_DT_SEC) return;
  last_telemetry_us = now_us;

  noInterrupts();
  int32_t left_ticks = g_left_ticks;
  int32_t right_ticks = g_right_ticks;
  interrupts();

  readMPU(); 

  Serial2.println("\n=== THONG SO XE ===");
  Serial2.print("Trang thai : ");
  if (estop) Serial2.println("DUNG KHAN CAP");
  else if (!motor_enabled) Serial2.println("TAT DONG CO");
  else Serial2.println("HOAT DONG");

  Serial2.print("Encoder    : Trai = "); Serial2.print(left_ticks); 
  Serial2.print(" | Phai = "); Serial2.println(right_ticks);
  
  Serial2.print("Gia toc    : X="); Serial2.print(accelX);
  Serial2.print(" | Y="); Serial2.print(accelY);
  Serial2.print(" | Z="); Serial2.println(accelZ);
  
  Serial2.print("Goc ngieng : X="); Serial2.print(gyroX);
  Serial2.print(" | Y="); Serial2.print(gyroY);
  Serial2.print(" | Z="); Serial2.println(gyroZ);

  // ĐÃ THÊM: In dữ liệu La Bàn ra màn hình
  Serial2.print("La ban     : X="); Serial2.print(magX);
  Serial2.print(" | Y="); Serial2.print(magY);
  Serial2.print(" | Z="); Serial2.println(magZ);
  
  Serial2.println("===================");
}

// =========================
// Setup & Loop
// =========================
void setupPins() {
  pinMode(LEFT_RPWM_PIN, OUTPUT); pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT); pinMode(RIGHT_LPWM_PIN, OUTPUT);

  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP); pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP); pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

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
  cmd_buffer.reserve(128);

  last_cmd_ms = millis();
  last_telemetry_us = micros();
}

void loop() {
  readCommands();
  checkSafetyLoop(); 
  sendTelemetry();
}