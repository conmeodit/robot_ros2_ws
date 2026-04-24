#include <Wire.h>

void setup() {
  Wire.begin();
  
  // Bật tính năng chống treo I2C (Timeout 100ms)
  // Nếu mạch bị đơ, nó sẽ tự thoát và báo lỗi
  Wire.setWireTimeout(100000, true); 
  
  Serial.begin(115200); 
  Serial2.begin(9600);  
  
  delay(2000); 
  
  Serial.println("\n--- Bat dau quet I2C ---");
  Serial2.println("\n--- Bat dau quet I2C ---");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Dang quet...");
  Serial2.println("Dang quet...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Tim thay I2C tai: 0x");
      Serial2.print("Tim thay I2C tai: 0x");
      if (address < 16) { Serial.print("0"); Serial2.print("0"); }
      Serial.println(address, HEX);
      Serial2.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Loi tai: 0x");
      Serial2.print("Loi tai: 0x");
      if (address < 16) { Serial.print("0"); Serial2.print("0"); }
      Serial.println(address, HEX);
      Serial2.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("Khong tim thay module nao. Kiem tra lai day!\n");
    Serial2.println("Khong tim thay module nao. Kiem tra lai day!\n");
  } else {
    Serial.println("Quet xong.\n");
    Serial2.println("Quet xong.\n");
  }

  delay(5000); 
}