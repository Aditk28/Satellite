/*
  enc_test.cpp — STANDALONE MT6701 SSI encoder test.

  Purpose: prove the encoder + wiring + SCK pin in COMPLETE isolation. No
  FreeRTOS, no SimpleFOC, no motor, no gyro — just a bare-metal SPI read of the
  MT6701 printed continuously over USB serial. This replicates exactly what
  MagneticSensorMT6701SSI::readRawAngleSSI() does (SPI mode 2, 1 MHz, MSB-first,
  16-bit frame, drop 1 LSB -> 14-bit angle), with nothing else in the way.

  Flash:   pio run -e enctest -t upload
  Monitor: pio device monitor -e enctest      (USB / ST-LINK VCP; press RESET)

  READ THE raw16 COLUMN — it tells you what the encoder is actually returning:
    changes smoothly as you hand-turn the wheel   -> encoder + wiring GOOD.
        (the fault is then in the RTOS firmware/config, not the hardware.)
    stuck 0x0000                                  -> DO reads LOW: no data / DO
                                                     disconnected / encoder unpowered.
    stuck 0xFFFF (count 16383, ~359.98 deg)       -> DO idle HIGH but never clocked:
                                                     SCK not toggling / bad clock line.
    random / jumpy at rest                        -> noise, marginal connection, or magnet.

  If it's stuck, try the alternate SCK pin: change ENC_SCK to PB10, move the CLK
  wire from PB13 to PB10 (D6 on the Nucleo header), reflash. PB10 is the other
  SPI2_SCK pin on the F446 and is otherwise free.
*/

#include <Arduino.h>
#include <SPI.h>

// SPI2 pins. SCK is the one under suspicion.
#define ENC_SCK   PB13     // <-- try PB10 (alternate SPI2_SCK) if PB13 is dead
#define ENC_MISO  PB14     // DO  (encoder data out -> MCU)
#define ENC_MOSI  PB15     // unused by SSI, but SPIClass needs a MOSI pin
#define ENC_CS    PB1      // CSN (chip select, active low)

SPIClass    encSPI(ENC_MOSI, ENC_MISO, ENC_SCK);
SPISettings encSettings(1000000, MSBFIRST, SPI_MODE2);

static uint16_t readRaw16() {
  encSPI.beginTransaction(encSettings);
  digitalWrite(ENC_CS, LOW);
  uint16_t v = encSPI.transfer16(0x0000);
  digitalWrite(ENC_CS, HIGH);
  encSPI.endTransaction();
  return v;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(ENC_CS, OUTPUT);
  digitalWrite(ENC_CS, HIGH);
  encSPI.begin();

  Serial.println();
  Serial.println(F("=== MT6701 standalone SSI test ==="));
  Serial.println(F("SCK=PB13 (edit ENC_SCK for PB10)  MISO=PB14  MOSI=PB15  CS=PB1  mode2 @1MHz"));
  Serial.println(F("Hand-turn the wheel. raw16 should sweep. stuck 0x0000/0xFFFF = hardware."));
  Serial.println(F("raw16,count14,angle_deg"));
}

void loop() {
  uint16_t raw   = readRaw16();
  uint16_t count = (raw >> 1) & 0x3FFF;            // 14-bit angle (matches the driver)
  float    deg   = count / 16384.0f * 360.0f;

  Serial.print("0x");
  if (raw < 0x1000) Serial.print('0');
  if (raw < 0x0100) Serial.print('0');
  if (raw < 0x0010) Serial.print('0');
  Serial.print(raw, HEX);
  Serial.print(',');  Serial.print(count);
  Serial.print(',');  Serial.println(deg, 2);

  delay(200);                                      // 5 Hz
}
