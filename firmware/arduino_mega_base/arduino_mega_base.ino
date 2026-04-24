#include <Wire.h>

void setup() {
  Wire.begin();
  
  // Khởi tạo Serial máy tính (dự phòng)
  Serial.begin(115200); 
  
  // Khởi tạo giao tiếp với HC-05 qua cổng Serial2 của Mega
  Serial2.begin(9600);  
  
  delay(2000); // Chờ HC-05 khởi động
  
  Serial.println("\n--- Bat dau quet I2C ---");
  Serial2.println("\n--- Bat dau quet I2C ---");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Dang quet...");
  Serial2.println("Dang quet...");

  nDevices = 0;
  // Quét các địa chỉ từ 1 đến 127
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Tim thay I2C tai dia chi 0x");
      Serial2.print("Tim thay I2C tai dia chi 0x");
      
      if (address < 16) {
        Serial.print("0");
        Serial2.print("0");
      }
      
      Serial.println(address, HEX);
      Serial2.println(address, HEX);
      
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Loi khong xac dinh tai 0x");
      Serial2.print("Loi khong xac dinh tai 0x");
      
      if (address < 16) {
        Serial.print("0");
        Serial2.print("0");
      }
      
      Serial.println(address, HEX);
      Serial2.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("Khong tim thay module I2C nao.\n");
    Serial2.println("Khong tim thay module I2C nao.\n");
  } else {
    Serial.println("Quet xong.\n");
    Serial2.println("Quet xong.\n");
  }

  // Chờ 5 giây rồi tiến hành quét lại
  delay(5000); 
}