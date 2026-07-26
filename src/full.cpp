/*
  PURPOSE: Closed-loop velocity control of the reaction-wheel BLDC motor via
  SimpleFOC + MT6701 encoder (SSI mode, second SPI bus), plus MPU6050 (IMU)
  and INA219 (bus voltage/current) streaming on the shared I2C1 bus.

  UPDATED THIS REVISION -- PIN REMAP ONLY:
    Old:  Mini IN1->D10   IN2->D11   IN3->D12   EN->D13
    New:  Mini EN ->D10   IN3->D11   IN2->D12   IN1->D13
    Also: MT6701 Z (CSN) moved from D9/PC7 -> PB1 (CN10 pin 24, Morpho
    connector, not the Arduino header) -- just a bit-banged GPIO in the
    library, so any free pin works; PB1 chosen over PB2 to avoid PB2's
    BOOT1 dual-function (harmless to reuse, but PB1 has no caveat at all).
  Nothing else changed. See the note above PIN_PWM_A/B/C/PIN_ENABLE below
  for why the driver remap is electrically fine on this board.

  BOARD: STM32 Nucleo-F446RE

  This sketch does NOT depend on "SimpleFOCDrivers" -- that library bundles
  many unrelated drivers and Arduino compiles a library's entire source tree
  once any part of it is referenced, which both caused earlier compile
  errors in unrelated files AND overflowed flash. The MT6701 SSI driver only
  needs Arduino.h, SPI.h, and a base class from core "Simple FOC" -- so its
  two files (MagneticSensorMT6701SSI.h / .cpp) must remain copied directly
  in this project's src/ folder.

  WIRING:
    Driver (UPDATED):
      Mini EN  -> D10   Mini IN3 -> D11   Mini IN2 -> D12   Mini IN1 -> D13
      Mini GND -> GND

    Encoder (MT6701, SSI mode -- IIC/SSI solder jumper must be BRIDGED):
      MT6701 VCC     -> 3.3V
      MT6701 GND     -> GND
      MT6701 B/SCL   -> PB13 (CN10)   -- CLK
      MT6701 A/SDA   -> PB14 (CN10)   -- DO (data out)
      MT6701 Z       -> PB1 (CN10 pin 24) -- CSN in SSI mode (moved off D9)
      PB15 (MOSI) declared in code but not wired to anything.

    MPU6050  SCL/SDA -> PB8 / PB9  (shared I2C1 bus, same pins as INA219)
    MPU6050  AD0     -> GND  (fixed address 0x68)
    INA219   SCL/SDA -> PB8 / PB9  (same bus, address 0x40 by default)
    Both     VCC/GND -> 3.3V / GND (common ground with everything else)

  No pin conflicts: I2C1 (PB8/PB9) is separate from the motor's D10-D13
  (PB6/PA7/PA6/PA5), the encoder CS on PB1 (CN10 pin 24), and the encoder
  SPI bus on PB13/PB14/PB15.

  TUNING: Serial Monitor at 921600 baud. Commands via SimpleFOC Commander:
    M0.5        -> set target velocity to 0.5 rad/s
    MC          -> print current motor state
    M PID_vel P 0.2    -> live-tune velocity loop P gain
    M PID_vel I 2.0    -> live-tune velocity loop I gain
    B           -> capture a burst of samples at full loop rate
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
// UPDATED PIN MAPPING (this revision)
//   Old:  IN1->D10  IN2->D11  IN3->D12  EN->D13
//   New:  EN ->D10  IN3->D11  IN2->D12  IN1->D13
//
// Why this is fine: on the Nucleo-F446RE's Arduino header,
//   D10=PB6, D11=PA7, D12=PA6, D13=PA5.
// EN is a plain digital output (no timer/PWM needed), so it can sit on
// ANY of these four pins -- it's simply moving from PA5(D13) to PB6(D10).
// The three PWM channels just shift which physical pins they land on:
//   PWM_A (IN1) -> D13 = PA5 = TIM2_CH1
//   PWM_B (IN2) -> D12 = PA6 = TIM3_CH1
//   PWM_C (IN3) -> D11 = PA7 = TIM3_CH2
// That's still a 2-timer split (TIM3 CH1+CH2 together, TIM2 on its own) --
// the exact same *pattern* the old mapping used (TIM4 alone + TIM3 CH1+CH2),
// which is already proven working on this board. SimpleFOC's STM32 driver
// handles multi-timer PWM groups like this, so this remap does not change
// anything about how the driver initializes.
//
// One cosmetic note: D13/PA5 is also tied to the Nucleo's onboard green
// LED (LD2) by default. IN1's PWM signal will now make LD2 glow/flicker
// with the duty cycle -- harmless, just don't mistake it for a fault.
//
// (Could not run an actual `pio run` compile in this sandbox -- PlatformIO's
// package registry, api.registry.platformio.org, is outside the allowed
// network domains here. The pin/timer reasoning above is a solid check on
// paper, but do a normal `pio run` + bench test yourself before trusting it
// fully.)
// ---------------------------------------------------------------------
#define PIN_PWM_A       13       // Mini IN1  (was D10)
#define PIN_PWM_B       12       // Mini IN2  (unchanged, D12)
#define PIN_PWM_C       11       // Mini IN3  (was D12)
#define PIN_ENABLE      10       // Mini EN   (was D13)

#define PIN_ENCODER_CS  PB1      // Nucleo pin wired to MT6701's Z pin
                                 // (was D9/PC7 -- moved to PB1, CN10 pin 24,
                                 // on the Morpho connector. Picked PB1 over
                                 // PB2 since PB2 doubles as BOOT1 -- safe to
                                 // reuse, but PB1 has no such caveat at all.)

#define VOLTAGE_LIMIT   10.0f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);

Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

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
// capture), then dumps everything in one shot afterward.
// =====================================================================
#define BURST_SIZE 800
unsigned long burst_time_us[BURST_SIZE];
float burst_vel_raw[BURST_SIZE];
float burst_vel_filt[BURST_SIZE];
volatile bool capturing = false;
int captureIndex = 0;

void dumpBurst() {
  Serial.println("--- burst capture start ---");
  Serial.println("t_us,vel_raw,vel_filtered");
  for (int i = 0; i < BURST_SIZE; i++) {
    Serial.print(burst_time_us[i]);       Serial.print(",");
    Serial.print(burst_vel_raw[i], 3);    Serial.print(",");
    Serial.println(burst_vel_filt[i], 3);
  }
  Serial.println("--- burst capture end ---");
}

void onBurstCapture(char* cmd) {
  captureIndex = 0;
  capturing = true;
  Serial.println("Burst capture started...");
}

void setup() {
  Serial.begin(921600); // raised from 115200 -- shrinks blocking time per
                         // print call ~8x, so prints stop stealing
                         // multi-ms chunks from the FOC loop
  delay(1000);

  // ---- I2C1 + IMU + current sensor bring-up ----
  Wire.begin(); // PB8 (SCL) / PB9 (SDA)

  if (!mpu.begin()) {
    Serial.println("[MPU6050] NOT FOUND -- check wiring / address (0x68)");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU6050] OK");
  }

  if (!ina219.begin()) {
    Serial.println("[INA219] NOT FOUND -- check wiring / address (0x40)");
  } else {
    Serial.println("[INA219] OK");
  }

  // ---- motor / encoder setup ----
  sensor.init(&encoderSPI);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_PSU;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println("Driver init FAILED -- check pin/timer mapping.");
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

  Serial.println("Closed-loop velocity control ready.");
  Serial.println("Send e.g. 'M2' for 2 rad/s target, 'MC' to see state.");
  Serial.println("Send 'B' to capture a burst of samples at full loop rate.");

  Serial.println("gyroZ,accelX,accelY,accelZ,busV,current_mA,power_mW,wheelAngle,wheelVel_raw,wheelVel_filtered,gyroZ_filt_dps,zAngle_deg");

  delay(1000);
}

void loop() {
  // FOC loop must run every iteration, as fast and uninterrupted as possible.
  motor.loopFOC();
  motor.move();
  command.run();

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

    Serial.print(g.gyro.z, 4);         Serial.print(",");
    Serial.print(a.acceleration.x, 2); Serial.print(",");
    Serial.print(a.acceleration.y, 2); Serial.print(",");
    Serial.print(a.acceleration.z, 2); Serial.print(",");
    Serial.print(busVoltage, 2);       Serial.print(",");
    Serial.print(current_mA, 1);       Serial.print(",");
    Serial.print(power_mW, 1);         Serial.print(",");
    Serial.print(sensor.getAngle(), 3);    Serial.print(",");
    Serial.print(sensor.getVelocity(), 3); Serial.print(",");
    Serial.print(motor.shaft_velocity, 3); Serial.print(",");
    Serial.print(filteredGyroZ_dps, 3);    Serial.print(",");
    Serial.println(zAngleDeg, 3);
  }
}