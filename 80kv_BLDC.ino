#include <SimpleFOC.h>

// BLDC motor instance (14 pole pairs typical for 2208 motors, verify with datasheet)
BLDCMotor motor = BLDCMotor(14);

// 6 PWM driver for TMC6300
BLDCDriver6PWM driver = BLDCDriver6PWM(26, 27, 14, 12, 13, 15); // UH, UL, VH, VL, WH, WL

void setup() {
  // Initialize driver
  driver.voltage_power_supply = 10.36; // Set to your supply voltage
  driver.init();
  motor.linkDriver(&driver);

  // Set control mode to open-loop velocity
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 10.36; // Match supply voltage
  motor.velocity_limit = 50; // Limit max speed (rad/s)

  // Initialize motor
  motor.init();

  Serial.begin(115200);
  Serial.println("Motor ready!");
}

void loop() {
  motor.loopFOC(); // Run commutation logic
  motor.move(20.94); // Target 200 RPM
}