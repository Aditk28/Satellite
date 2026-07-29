/*
  PURPOSE: Add the MT6701 magnetic encoder (SSI mode, over a second SPI bus)
  and run closed-loop velocity control. Only run this AFTER confirming
  smooth open-loop rotation with the correct pole pair count.

  BOARD: STM32 Nucleo-F446RE

  This sketch does NOT depend on "SimpleFOCDrivers" -- that library bundles
  many unrelated drivers and Arduino compiles a library's entire source tree
  once any part of it is referenced, which both caused earlier compile
  errors in unrelated files AND overflowed flash. The MT6701 SSI driver only
  needs Arduino.h, SPI.h, and a base class from core "Simple FOC" -- so its
  two files are copied directly into this project's src/ folder instead.

  WIRING:
    Driver:
      Mini IN1 -> D10   Mini IN2 -> D11   Mini IN3 -> D12   Mini EN -> D13
      Mini GND -> GND

    Encoder (MT6701, SSI mode -- IIC/SSI solder jumper must be BRIDGED):
      MT6701 VCC     -> 3.3V
      MT6701 GND     -> GND
      MT6701 B/SCL   -> PB13 (CN10)   -- CLK
      MT6701 A/SDA   -> PB14 (CN10)   -- DO (data out)
      MT6701 Z       -> D9            -- CSN in SSI mode
      PB15 (MOSI) declared in code but not wired to anything.

  TUNING: Serial Monitor at 115200 baud. Commands via SimpleFOC Commander:
    M0.5        -> set target velocity to 0.5 rad/s
    MC          -> print current motor state
    M PID_vel P 0.2    -> live-tune velocity loop P gain
    M PID_vel I 2.0    -> live-tune velocity loop I gain
*/

#include <SimpleFOC.h>
#include "MagneticSensorMT6701SSI.h"

#define POLE_PAIRS      11      // 24-slot/22-pole motor -> 22/2 = 11

#define PIN_PWM_A       10       // Mini IN1
#define PIN_PWM_B       11       // Mini IN2
#define PIN_PWM_C       12       // Mini IN3
#define PIN_ENABLE      13       // Mini EN

#define PIN_ENCODER_CS  9        // Nucleo pin wired to MT6701's Z pin

#define VOLTAGE_LIMIT   16.0f
#define VOLTAGE_PSU     18.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);

Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  delay(1000);

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
  motor.LPF_velocity.Tf = 0.01f;

  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 80.0f;

  // motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  command.add('M', onMotor, "motor");

  Serial.println("Closed-loop velocity control ready.");
  Serial.println("Send e.g. 'M2' for 2 rad/s target, 'MC' to see state.");

  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
}