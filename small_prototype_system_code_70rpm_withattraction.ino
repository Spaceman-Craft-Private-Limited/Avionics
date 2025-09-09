#define STEP_PIN 3 // Arduino pin connected to STEP
#define DIR_PIN 4 // Arduino pin connected to DIR
#define RELAY_PIN 2 // Arduino pin connected to 5V relay

// Stepper motor variables
// const unsigned long STEP_INTERVAL = 1254; // For ~70 RPM vref= 0.64 V 

// Previous RPM settings: 
// const unsigned long STEP_INTERVAL = 3000; // For ~29.27 RPM (original, 2.05s per revolution) 
// const unsigned long STEP_INTERVAL = 439;  // For ~200 RPM vref= 0.64V (over speed and noise produce) 
// const unsigned long STEP_INTERVAL = 585;  // For ~150 RPM vref=0.64V (Motor stpos inbetween)
// bool stepState = LOW;
// unsigned long lastStepTime = 0;

// --- New variables for Acceleration Ramp ---
// const unsigned long minStepInterval = 1254;     // Target interval for max speed (~70 RPM)
// const unsigned long initialStepInterval = 4000;   // Starting interval (slow speed)
// const unsigned long accelerationStep = 10;      // How much to decrease the interval by each step (controls acceleration rate)
// unsigned long currentStepInterval;              // The current step interval, which will change over time

void setup() {
  // Initialize pins. This part runs only once.
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Initial state for motor pins
  digitalWrite(DIR_PIN, HIGH); // Set direction to clockwise
  digitalWrite(STEP_PIN, LOW); // Initialize step pin
}

void loop() {
  // This entire sequence will now repeat continuously.
  
  // --- New Sequence Logic ---

  // 1. Turn relay ON for 1 second, then OFF
  delay(1000); // after first time relay on & this is from the second cycle
  digitalWrite(RELAY_PIN, LOW); // Turn relay ON (assuming active low)
  delay(1000); // Wait for 1 second
  digitalWrite(RELAY_PIN, HIGH); // Turn relay OFF

  // 2. Wait for 3 seconds
  delay(3000);

  // 3. Run motor for 3.56 full rotations and stop
  // NOTE: A standard stepper motor is 200 steps/revolution.
  // If you use microstepping, you must multiply this number.
  // (e.g., 16x microstepping = 200 * 16 = 3200 steps).
  const int totalSteps = 712; // This is 200 steps/rev * 3.56 revolutions
  // const int stepsPerRevolution = 200;

  // Each step requires a HIGH and a LOW pulse, so we loop twice the number of steps.
  for (int i = 0; i < totalSteps * 2; i++) {
  // for (int i = 0; i < stepsPerRevolution * 2; i++) {
    digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); // Toggle the step pin to create a pulse
    delayMicroseconds(1254); // This delay controls the speed (~70 RPM)
  }

  // After this, the loop() function will start again from the top.
}