#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); // Đợi Serial Monitor mở
  
  Serial.println("\n--- Bắt đầu quét I2C ---");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Đang quét...");

  nDevices = 0;
  // Quét các địa chỉ từ 1 đến 127
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Tìm thấy thiết bị I2C tại địa chỉ 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Lỗi không xác định tại địa chỉ 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("Không tìm thấy thiết bị I2C nào.\n");
  } else {
    Serial.println("Quét xong.\n");
  }

  delay(5000); // Chờ 5 giây rồi quét lại
}