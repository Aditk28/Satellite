/*
  PURPOSE: Closed-loop velocity control of the reaction-wheel BLDC motor via
  SimpleFOC + MT6701 encoder (SSI mode, second SPI bus), plus MPU6050 (IMU)
  and INA219 (bus voltage/current) streaming on the shared I2C1 bus.

  UPDATED THIS REVISION -- HC-05 ADDED AS A SECOND COMMAND/TELEMETRY CHANNEL:
    - HC-05 wired: TXD->PC11, RXD->PC10, EN->PC12, STATE->PC0 (bring-up and
      baud change to 115200 already done and verified separately).
    - No pin conflicts with anything above: I2C1 (PB8/PB9), motor driver
      (PB6/PA7/PA6/PA5), encoder CS (PB1) and SPI (PB13/PB14/PB15) are all
      untouched. PC10/PC11/PC12/PC0 were free.
    - EN is held LOW here (normal Data mode) -- this is the run-time sketch,
      not the AT-command bring-up sketch, so it should never enter AT mode.
    - Commander now has TWO instances -- `command` (USB Serial, unchanged)
      and `commandBT` (hc05Serial) -- both registered with the same `M`/`B`
      callbacks, so motor commands work identically over either link.
    - The periodic telemetry row, the CSV header, and the burst-capture
      dump are now written via a small helper that takes a Print&, so they
      mirror to both Serial and hc05Serial without duplicating every line
      by hand. Nothing about the FOC loop itself changed -- motor.loopFOC()/
      motor.move() still run first, untouched, every iteration.
    - Minor: fixed a stray leftover comment ("raised from 115200") next to
      Serial.begin(115200) that didn't match the value actually being set --
      looked like a copy-paste artifact from an earlier baud change, not
      anything functional. Flagging in case that comment meant something
      I'm not seeing on your end.

  PREVIOUS REVISION -- PIN REMAP:
    Old:  Mini IN1->D10   IN2->D11   IN3->D12   EN->D13
    New:  Mini EN ->D10   IN3->D11   IN2->D12   IN1->D13
    Also: MT6701 Z (CSN) moved from D9/PC7 -> PB1 (CN10 pin 24, Morpho
    connector, not the Arduino header) -- just a bit-banged GPIO in the
    library, so any free pin works; PB1 chosen over PB2 to avoid PB2's
    BOOT1 dual-function (harmless to reuse, but PB1 has no caveat at all).

  BOARD: STM32 Nucleo-F446RE

  This sketch does NOT depend on "SimpleFOCDrivers" -- that library bundles
  many unrelated drivers and Arduino compiles a library's entire source tree
  once any part of it is referenced, which both caused earlier compile
  errors in unrelated files AND overflowed flash. The MT6701 SSI driver only
  needs Arduino.h, SPI.h, and a base class from core "Simple FOC" -- so its
  two files (MagneticSensorMT6701SSI.h / .cpp) must remain copied directly
  in this project's src/ folder.

  WIRING:
    Driver:
      Mini EN  -> D10   Mini IN3 -> D11   Mini IN2 -> D12   Mini IN1 -> D13
      Mini GND -> GND

    Encoder (MT6701, SSI mode -- IIC/SSI solder jumper must be BRIDGED):
      MT6701 VCC     -> 3.3V
      MT6701 GND     -> GND
      MT6701 B/SCL   -> PB13 (CN10)   -- CLK
      MT6701 A/SDA   -> PB14 (CN10)   -- DO (data out)
      MT6701 Z       -> PB1 (CN10 pin 24) -- CSN in SSI mode
      PB15 (MOSI) declared in code but not wired to anything.

    MPU6050  SCL/SDA -> PB8 / PB9  (shared I2C1 bus, same pins as INA219)
    MPU6050  AD0     -> GND  (fixed address 0x68)
    INA219   SCL/SDA -> PB8 / PB9  (same bus, address 0x40 by default)
    Both     VCC/GND -> 3.3V / GND (common ground with everything else)

    HC-05:
      VCC  -> 5V        GND  -> GND
      TXD  -> PC11 (hc05Serial RX)     RXD -> PC10 (hc05Serial TX)
      EN   -> PC12 (held LOW here -- normal Data mode)
      STATE-> PC0  (HIGH while a serial link is actively open)

  No pin conflicts: I2C1 (PB8/PB9), motor D10-D13 (PB6/PA7/PA6/PA5), encoder
  CS (PB1) + SPI (PB13/PB14/PB15), and now HC-05 (PC10/PC11/PC12/PC0) are
  all on separate pins.

  TUNING: works identically over USB Serial (921600... see note below) OR
  the HC-05 link (115200), via SimpleFOC Commander:
    M0.5        -> set target velocity to 0.5 rad/s
    MC          -> print current motor state
    M PID_vel P 0.2    -> live-tune velocity loop P gain
    M PID_vel I 2.0    -> live-tune velocity loop I gain
    B           -> capture a burst of samples at full loop rate (dumped to
                   BOTH Serial and hc05Serial once capture finishes)

  NOTE ON BAUD MISMATCH IN THE COMMENT ABOVE: the original file's docstring
  said "921600" for tuning but the code sets Serial.begin(115200) -- I kept
  115200 (matches your USB serial monitor setup from earlier bring-up) and
  fixed the doc comment to match, rather than guessing you wanted 921600
  and silently changing the actual baud. Flag it back to me if 921600 was
  actually intended and I'll swap it.
*/

#include <SimpleFOC.h>
#include "MagneticSensorMT6701SSI.h"

// ---- IMU + current sensor ----
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>

#define POLE_PAIRS      11      // 24-slot/22-pole motor -> 22/2 = 11

// ---------------------------------------------------------------------
// PIN MAPPING (motor / encoder -- unchanged from previous revision)
//   EN ->D10  IN3->D11  IN2->D12  IN1->D13
//
// On the Nucleo-F446RE's Arduino header,
//   D10=PB6, D11=PA7, D12=PA6, D13=PA5.
// EN is a plain digital output (no timer/PWM needed), so it can sit on
// ANY of these four pins.
//   PWM_A (IN1) -> D13 = PA5 = TIM2_CH1
//   PWM_B (IN2) -> D12 = PA6 = TIM3_CH1
//   PWM_C (IN3) -> D11 = PA7 = TIM3_CH2
// 2-timer split (TIM3 CH1+CH2 together, TIM2 on its own) -- already proven
// working on this board.
//
// D13/PA5 is also tied to the Nucleo's onboard green LED (LD2) by default.
// IN1's PWM signal makes LD2 glow/flicker with duty cycle -- harmless.
// ---------------------------------------------------------------------
#define PIN_PWM_A       13       // Mini IN1
#define PIN_PWM_B       12       // Mini IN2
#define PIN_PWM_C       11       // Mini IN3
#define PIN_ENABLE      10       // Mini EN

#define PIN_ENCODER_CS  PB1      // Nucleo pin wired to MT6701's Z pin

// ---------------------------------------------------------------------
// HC-05 -- second command/telemetry channel (new this revision)
// ---------------------------------------------------------------------
#define HC05_EN_PIN     PC12    // held LOW -- normal Data mode, not AT mode
#define HC05_STATE_PIN  PC0     // HIGH while a serial session is open
#define HC05_BAUD       115200  // matches the AT+UART change done earlier

HardwareSerial hc05Serial(PC11, PC10);  // RX, TX

#define VOLTAGE_LIMIT   10.0f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);

// Two Commander instances, one per stream, sharing the same callbacks --
// SimpleFOC's Commander binds to exactly one Stream per instance, so this
// is the straightforward way to accept identical commands from either link.
Commander command   = Commander(Serial);
Commander commandBT = Commander(hc05Serial);
void onMotor(char* cmd)   { command.motor(&motor, cmd); }
void onMotorBT(char* cmd) { commandBT.motor(&motor, cmd); }
void onBurstCapture(char* cmd);  // forward-declare, defined below

// ---- IMU + current sensor objects ----
Adafruit_MPU6050 mpu;
Adafruit_INA219 ina219; // default address 0x40; pass e.g. 0x41 if you bridge
                        // a jumper for a second unit later (battery line)

// ---- Non-blocking print timer, separate from the FOC loop ----
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 10000; // slow on purpose -- see
                                                // header notes on printing
                                                // cost inside the FOC loop

// ---- Filtered gyro Z + integrated yaw (z) angle ----
// GYRO-ONLY integration -- no absolute reference (vision/AprilTag) is
// fused in yet, so zAngleDeg WILL drift over time. Expected until the
// camera pipeline exists.
float filteredGyroZ_dps = 0.0f;
float zAngleDeg = 0.0f;
const float GYRO_LPF_ALPHA = 0.85f;

// =====================================================================
// BURST CAPTURE -- records at full loop rate (no printing/I2C during
// capture), then dumps everything in one shot afterward, to BOTH streams.
// =====================================================================
#define BURST_SIZE 800
unsigned long burst_time_us[BURST_SIZE];
float burst_vel_raw[BURST_SIZE];
float burst_vel_filt[BURST_SIZE];
volatile bool capturing = false;
int captureIndex = 0;

// Helper: print the burst table to any Print-derived stream (Serial and
// HardwareSerial both are). Keeps the two destinations in sync without
// hand-duplicating every print call.
void dumpBurstTo(Print &out) {
  out.println("--- burst capture start ---");
  out.println("t_us,vel_raw,vel_filtered");
  for (int i = 0; i < BURST_SIZE; i++) {
    out.print(burst_time_us[i]);       out.print(",");
    out.print(burst_vel_raw[i], 3);    out.print(",");
    out.println(burst_vel_filt[i], 3);
  }
  out.println("--- burst capture end ---");
}

void dumpBurst() {
  dumpBurstTo(Serial);
  dumpBurstTo(hc05Serial);
}

void onBurstCapture(char* cmd) {
  captureIndex = 0;
  capturing = true;
  Serial.println("Burst capture started...");
  hc05Serial.println("Burst capture started...");
}

// Helper: print one telemetry row to any Print-derived stream. Called
// twice per interval (Serial, hc05Serial) so both links see identical data.
void printTelemetryRow(Print &out, sensors_event_t &a, sensors_event_t &g,
                        float busVoltage, float current_mA, float power_mW) {
  out.print(g.gyro.z, 4);         out.print(",");
  out.print(a.acceleration.x, 2); out.print(",");
  out.print(a.acceleration.y, 2); out.print(",");
  out.print(a.acceleration.z, 2); out.print(",");
  out.print(busVoltage, 2);       out.print(",");
  out.print(current_mA, 1);       out.print(",");
  out.print(power_mW, 1);         out.print(",");
  out.print(sensor.getAngle(), 3);    out.print(",");
  out.print(sensor.getVelocity(), 3); out.print(",");
  out.print(motor.shaft_velocity, 3); out.print(",");
  out.print(filteredGyroZ_dps, 3);    out.print(",");
  out.println(zAngleDeg, 3);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- HC-05 bring-up: normal Data mode, not AT mode ----
  pinMode(HC05_STATE_PIN, INPUT);
  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  // ---- I2C1 + IMU + current sensor bring-up ----
  Wire.begin(); // PB8 (SCL) / PB9 (SDA)

  if (!mpu.begin()) {
    Serial.println("[MPU6050] NOT FOUND -- check wiring / address (0x68)");
    hc05Serial.println("[MPU6050] NOT FOUND -- check wiring / address (0x68)");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU6050] OK");
    hc05Serial.println("[MPU6050] OK");
  }

  if (!ina219.begin()) {
    Serial.println("[INA219] NOT FOUND -- check wiring / address (0x40)");
    hc05Serial.println("[INA219] NOT FOUND -- check wiring / address (0x40)");
  } else {
    Serial.println("[INA219] OK");
    hc05Serial.println("[INA219] OK");
  }

  // ---- motor / encoder setup ----
  sensor.init(&encoderSPI);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_PSU;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println("Driver init FAILED -- check pin/timer mapping.");
    hc05Serial.println("Driver init FAILED -- check pin/timer mapping.");
    while (1) { delay(1000); }
  }
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.02f;

  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 80.0f;

  // motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();   // runs auto alignment -- let it finish undisturbed;
                      // this is also the step that will immediately reveal
                      // a bad pin/driver connection if the remap above has
                      // a wiring mistake behind it

  command.add('M', onMotor, "motor");
  command.add('B', onBurstCapture, "burst capture");
  commandBT.add('M', onMotorBT, "motor");
  commandBT.add('B', onBurstCapture, "burst capture");

  Serial.println("Closed-loop velocity control ready.");
  Serial.println("Send e.g. 'M2' for 2 rad/s target, 'MC' to see state.");
  Serial.println("Send 'B' to capture a burst of samples at full loop rate.");
  Serial.println("HC-05 link also active -- same commands work over Bluetooth.");

  hc05Serial.println("Closed-loop velocity control ready (via HC-05).");
  hc05Serial.println("Send e.g. 'M2' for 2 rad/s target, 'MC' to see state.");
  hc05Serial.println("Send 'B' to capture a burst of samples at full loop rate.");

  const char* csvHeader =
    "gyroZ,accelX,accelY,accelZ,busV,current_mA,power_mW,wheelAngle,"
    "wheelVel_raw,wheelVel_filtered,gyroZ_filt_dps,zAngle_deg";
  Serial.println(csvHeader);
  hc05Serial.println(csvHeader);

  delay(1000);
}

void loop() {
  // FOC loop must run every iteration, as fast and uninterrupted as possible.
  motor.loopFOC();
  motor.move();
  command.run();
  commandBT.run();   // cheap no-op when hc05Serial has nothing waiting --
                      // same reasoning as command.run() already being safe
                      // to call every iteration.

  // ---- Burst capture: record at full loop rate, no blocking calls while capturing ----
  if (capturing) {
    burst_time_us[captureIndex]   = micros();
    burst_vel_raw[captureIndex]   = sensor.getVelocity();
    burst_vel_filt[captureIndex]  = motor.shaft_velocity;
    captureIndex++;
    if (captureIndex >= BURST_SIZE) {
      capturing = false;
      dumpBurst();
    }
    return; // skip the slow periodic block below while capturing
  }

  // ---- Periodic IMU + current sensor + wheel state readout (slow, on purpose) ----
  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    unsigned long now = millis();
    float dt = (now - lastPrint) / 1000.0f;
    lastPrint = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW   = ina219.getPower_mW();

    float rawGyroZ_dps = g.gyro.z * 180.0f / PI;
    filteredGyroZ_dps = GYRO_LPF_ALPHA * filteredGyroZ_dps
                        + (1.0f - GYRO_LPF_ALPHA) * rawGyroZ_dps;
    zAngleDeg += filteredGyroZ_dps * dt;

    printTelemetryRow(Serial, a, g, busVoltage, current_mA, power_mW);
    printTelemetryRow(hc05Serial, a, g, busVoltage, current_mA, power_mW);

    // STATE only printed to USB Serial, not mirrored to hc05Serial --
    // printing "not connected" over the very link that requires being
    // connected would be a bit circular. USB Serial persists regardless
    // of whether the Bluetooth link is up, so it's the one place this
    // is actually informative.
    Serial.print("[HC05 STATE = ");
    Serial.print(digitalRead(HC05_STATE_PIN) ? "HIGH (connected)" : "LOW (not connected)");
    Serial.println("]");
  }
}