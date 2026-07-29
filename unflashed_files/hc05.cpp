/* ============================================================================
 * HC-05 Data-mode verification, at the NEW baud (115200) set via AT+UART.
 *
 * EN is left LOW here (not forced high) so the module boots into normal
 * Data mode rather than AT mode. Power-cycle the HC-05 itself (unplug/
 * replug its VCC wire, or the whole USB cable) before testing -- an MCU
 * reset alone may not be enough since the 5V rail likely stays powered
 * through a reset.
 *
 * How to verify the baud change actually took:
 *   Option A (no phone needed): with this flashed, open the serial
 *   monitor. You shouldn't see garbage/mismatched-baud noise -- silence
 *   is actually expected here since nothing is being sent yet. Type
 *   something in the monitor; it gets forwarded to the HC-05. If the
 *   HC-05 is paired to something on the other end, you should see
 *   whatever it echoes back arrive cleanly (not garbled) -- garbled
 *   text is the classic symptom of a baud mismatch.
 *   Option B (more conclusive): pair the HC-05 with your phone's
 *   Bluetooth settings, connect a serial terminal app to it, and type
 *   back and forth between the phone and this serial monitor.
 * ==========================================================================*/

#include <Arduino.h>

#define EN_PIN     PC12
#define STATE_PIN  PC0

#define HC05_BAUD 115200   // the NEW baud you just set via AT+UART

HardwareSerial hc05Serial(PC11, PC10);   // RX, TX

unsigned long lastStatePrint = 0;

void setup() {
  pinMode(STATE_PIN, INPUT);

  // Leave EN low/floating -- do NOT force AT mode this time.
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  hc05Serial.begin(HC05_BAUD);

  Serial.println("HC-05 Data-mode test. EN held LOW.");
  Serial.println("Power-cycle the HC-05's VCC (or whole USB cable) now");
  Serial.println("if you haven't already, so it boots fresh in Data mode.");
  Serial.print("Talking to it at ");
  Serial.print(HC05_BAUD);
  Serial.println(" baud.");
}

void loop() {
  if (Serial.available()) {
    hc05Serial.write(Serial.read());
  }
  if (hc05Serial.available()) {
    Serial.write(hc05Serial.read());
  }

  if (millis() - lastStatePrint > 2000) {
    lastStatePrint = millis();
    Serial.print("\n[STATE = ");
    Serial.print(digitalRead(STATE_PIN) ? "HIGH (connected)" : "LOW (not connected)");
    Serial.println("]");
  }
}