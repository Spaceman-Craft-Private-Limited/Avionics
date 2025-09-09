// // code with tb6600 whole system-------------------------
// #include <Arduino.h> // Core ESP32 functions
// #include <AccelStepper.h> // Library for stepper motor control

// // --- Pin Definitions ---
// #define STEP_PIN 18      // GPIO for stepper motor PUL+ signal (TB6600 PUL pin)
// #define DIR_PIN 19       // GPIO for stepper motor DIR+ signal (TB6600 DIR pin)
// #define ENABLE_PIN 21    // GPIO for TB6600 EN+ pin (active LOW)
// #define RELAY_PIN 22     // GPIO for the relay control signal (connects to NPN transistor base)

// // --- BTS7960 H-Bridge Pins for Eddy Damper Coil ---
// // Connect these to the corresponding pins on your BTS7960 module
// #define RPWM_PIN 15      // Right PWM input (e.g., IN_R or RPWM)
// #define LPWM_PIN 23      // Left PWM input (e.g., IN_L or LPWM)
// // R_EN and L_EN are typically tied HIGH on the module itself or connected to a GPIO if you need to enable/disable
// // For continuous operation, we'll assume they are enabled.

// // --- Motor Configuration ---
// #define MOTOR_INTERFACE_TYPE 1 // Motor interface type: 1 = Driver (STEP/DIR)
// const int STEPS_PER_REVOLUTION = 1600; // STEPS_PER_REVOLUTION for 1/8 micro-stepping (200 steps/rev * 8)

// // Create stepper object: (interface type, step pin, direction pin)
// AccelStepper stepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

// // --- Motor Speed Variables ---
// const float TARGET_MOTOR_RPM = 100.0; // Target RPM for continuous motor movement
// float motorSpeedStepsPerSec;          // Calculated speed in steps/second

// // --- Relay Control Variables (Non-blocking with millis()) ---
// unsigned long previousRelayMillis = 0;    // Stores when the relay last changed state
// unsigned long relayOnDuration = 1000;     // Relay ON for 1 second
// unsigned long relayOffDuration = 2000;    // Relay OFF for 2 seconds
// bool relayState = false;                  // Current state of the relay (false = OFF, true = ON)

// // --- Eddy Damper Bidirectional Toggle Configuration (25 Hz - 20ms in one direction / 20ms in other) ---
// unsigned long previousDamperToggleMillis = 0; // Stores when the eddy damper direction last changed
// const unsigned long damperDirectionDuration = 20; // Time for current in one direction (20ms)
// bool currentDirectionState = false;           // false = Direction 1 (RPWM HIGH), true = Direction 2 (LPWM HIGH)


// void setup() {
//   // --- Start Serial Communication Early for Debugging ---
//   Serial.begin(115200);
//   Serial.println("\n--- System Booting Up ---");

//   // --- Initial Delay to Ensure ESP32 Stability ---
//   delay(100); // Small delay to allow ESP32 to stabilize

//   Serial.println("ESP32 TB6600 Continuous Motor (100 RPM) with Non-Blocking Relay & BTS7960 Eddy Damper Started!");
//   Serial.println("Eddy Damper is driven by BTS7960 with 20ms in one direction, then 20ms in the other (25Hz alternating).");

//   // --- Initialize TB6600 Driver Pins ---
//   pinMode(ENABLE_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, LOW); // Enable the TB6600 motor driver (LOW = enabled)
//   pinMode(DIR_PIN, OUTPUT);
//   digitalWrite(DIR_PIN, HIGH); // Set initial direction to clockwise

//   // --- Initialize Relay Pin (for NPN transistor base) ---
//   pinMode(RELAY_PIN, OUTPUT);
//   digitalWrite(RELAY_PIN, LOW); // Initially turn transistor OFF -> relay OFF

//   // --- Initialize BTS7960 H-Bridge Pins ---
//   pinMode(RPWM_PIN, OUTPUT);
//   pinMode(LPWM_PIN, OUTPUT);

//   // Start with current in one direction (e.g., RPWM HIGH, LPWM LOW)
//   digitalWrite(RPWM_PIN, HIGH);
//   digitalWrite(LPWM_PIN, LOW);
//   currentDirectionState = false; // Set initial state
//   Serial.println("Eddy Damper initialized with current in one direction.");

//   // --- AccelStepper Configuration ---
//   motorSpeedStepsPerSec = (STEPS_PER_REVOLUTION * TARGET_MOTOR_RPM) / 60.0;
//   stepper.setMaxSpeed(motorSpeedStepsPerSec);
//   stepper.setAcceleration(0); // No acceleration for continuous rotation
//   stepper.setSpeed(motorSpeedStepsPerSec);
// }

// void loop() {
//   // --- Stepper Motor Control (Continuous 100 RPM) ---
//   stepper.runSpeed();

//   // --- Non-Blocking Relay Logic ---
//   unsigned long currentMillis = millis();
//   if (relayState) {
//     if (currentMillis - previousRelayMillis >= relayOnDuration) {
//       relayState = false;
//       digitalWrite(RELAY_PIN, LOW); // Turn transistor OFF -> relay OFF
//       previousRelayMillis = currentMillis;
//       Serial.println("Relay OFF");
//     }
//   } else {
//     if (currentMillis - previousRelayMillis >= relayOffDuration) {
//       relayState = true;
//       digitalWrite(RELAY_PIN, HIGH); // Turn transistor ON -> relay ON
//       previousRelayMillis = currentMillis;
//       Serial.println("Relay ON");
//     }
//   }

//   // --- Non-Blocking Eddy Damper Bidirectional Toggle Logic (20ms per direction) ---
//   if (currentMillis - previousDamperToggleMillis >= damperDirectionDuration) {
//     if (currentDirectionState == false) {
//       // Switch to Direction 2: LPWM HIGH, RPWM LOW
//       digitalWrite(RPWM_PIN, LOW);
//       digitalWrite(LPWM_PIN, HIGH);
//       currentDirectionState = true;
//       // Serial.println("Eddy Damper Direction 2"); // Uncomment for verbose debugging
//     } else {
//       // Switch to Direction 1: RPWM HIGH, LPWM LOW
//       digitalWrite(RPWM_PIN, HIGH);
//       digitalWrite(LPWM_PIN, LOW);
//       currentDirectionState = false;
//       // Serial.println("Eddy Damper Direction 1"); // Uncomment for verbose debugging
//     }
//     previousDamperToggleMillis = currentMillis; // Reset the timer
//   }
// }

// code with A4988 driver -----------------------------
#include <Arduino.h>
#include <AccelStepper.h> 

// --- Pin Definitions ---
#define STEP_PIN 18     
#define DIR_PIN 19      
#define ENABLE_PIN 21   

#define RELAY_PIN 22    
#define RPWM_PIN 15      
#define LPWM_PIN 23      

// --- Motor Configuration ---
#define MOTOR_INTERFACE_TYPE 1   // DRIVER mode for A4988
const int STEPS_PER_REVOLUTION = 200; // Full-step mode

AccelStepper stepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

// --- Motor Speed Variables ---
const float TARGET_MOTOR_RPM = 100.0; 
float motorSpeedStepsPerSec;       

// --- Relay Control Variables ---
unsigned long previousRelayMillis = 0;   
unsigned long relayOnDuration = 1000;    // 1 sec ON
unsigned long relayOffDuration = 2000;   // 2 sec OFF
bool relayState = false;             

// --- Eddy Damper Bidirectional Toggle ---
unsigned long previousDamperToggleMillis = 0; 
const unsigned long damperDirectionDuration = 20; 
bool currentDirectionState = false;          

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- System Booting Up ---");
  delay(100); 

  Serial.println("ESP32 A4988 Continuous Motor (100 RPM) with Non-Blocking Relay & BTS7960 Eddy Damper Started!");

  // --- Initialize A4988 Driver Pins ---
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW = enabled
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); 
  
  // --- Initialize Relay & BTS7960 Pins ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); 
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, HIGH);
  digitalWrite(LPWM_PIN, LOW);
  currentDirectionState = false; 
  Serial.println("Eddy Damper initialized with current in one direction.");

  // --- AccelStepper Configuration ---
  motorSpeedStepsPerSec = (STEPS_PER_REVOLUTION * TARGET_MOTOR_RPM) / 60.0; // 200*100/60 = ~333 steps/sec
  stepper.setMaxSpeed(1000);      // allow up to 1000 steps/sec
  stepper.setAcceleration(500);   // smooth acceleration
  stepper.setSpeed(motorSpeedStepsPerSec); // constant speed
}

void loop() {
  // --- Stepper Motor Continuous Rotation ---
  stepper.runSpeed();

  // --- Non-Blocking Relay Logic ---
  unsigned long currentMillis = millis();
  if (relayState) {
    if (currentMillis - previousRelayMillis >= relayOnDuration) {
      relayState = false;
      digitalWrite(RELAY_PIN, LOW); 
      previousRelayMillis = currentMillis;
      Serial.println("Relay OFF");
    }
  } else {
    if (currentMillis - previousRelayMillis >= relayOffDuration) {
      relayState = true;
      digitalWrite(RELAY_PIN, HIGH); 
      previousRelayMillis = currentMillis;
      Serial.println("Relay ON");
    }
  }

  // --- Non-Blocking Eddy Damper Bidirectional Toggle ---
  if (currentMillis - previousDamperToggleMillis >= damperDirectionDuration) {
    if (currentDirectionState == false) {
      digitalWrite(RPWM_PIN, LOW);
      digitalWrite(LPWM_PIN, HIGH);
      currentDirectionState = true;
    } else {
      digitalWrite(RPWM_PIN, HIGH);
      digitalWrite(LPWM_PIN, LOW);
      currentDirectionState = false;
    }
    previousDamperToggleMillis = currentMillis;
  }
}
