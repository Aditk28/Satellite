/*
  I2C scanner -- sweeps all addresses on the shared I2C1 bus (PB8=SCL,
  PB9=SDA) and prints what responds. Use this to check whether the
  INA219 is answering at all before chasing the driver library.

  Expected if everything's wired correctly:
    0x68 -> MPU6050
    0x40 -> INA219 (default, A0/A1 unbridged)

  Reading the result:
    - Only 0x68 shows up -> INA219 isn't powered or isn't on the bus.
      Check VCC/GND to 3.3V/GND (separate from VIN+/VIN- !), then
      SDA/SCL wiring.
    - 0x68 + some address other than 0x40 -> A0/A1 got bridged
      somehow, or a different default than expected -- note the address
      shown and pass it into Adafruit_INA219(0xXX) in main.cpp.
    - Nothing at all shows up -> bus-level problem (SDA/SCL swapped or
      unconnected, no common ground, or a bad breadboard row) -- would
      also explain why MPU6050 fails, if it does.
*/

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(921600);
  delay(1000);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  Serial.println("I2C scanner starting (PB8=SCL, PB9=SDA)...");
}

void loop() {
  int found = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found. Check wiring / common ground.");
  } else {
    Serial.print(found);
    Serial.println(" device(s) found.");
  }

  Serial.println("---");
  delay(3000); // rescan every 3 seconds
}