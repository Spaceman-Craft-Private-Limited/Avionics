#define STEP_PIN 3 // Arduino pin connected to STEP
#define DIR_PIN 4 // Arduino pin connected to DIR
#define RELAY_PIN 2 // Arduino pin connected to 5V relay

// Stepper motor variables
const unsigned long STEP_INTERVAL = 1254; // For ~70 RPM vref= 0.68 V 

// Previous RPM settings: 
// const unsigned long STEP_INTERVAL = 3000; // For ~29.27 RPM (original, 2.05s per revolution) 
// const unsigned long STEP_INTERVAL = 439;  // For ~200 RPM vref= 0.64V (over speed and noise produce) 
// const unsigned long STEP_INTERVAL = 585;  // For ~150 RPM vref=0.64V (Motor stpos inbetween)
bool stepState = LOW;
unsigned long lastStepTime = 0;

// --- New variables for Acceleration Ramp ---
const unsigned long minStepInterval = 1254;     // Target interval for max speed (~70 RPM)
const unsigned long initialStepInterval = 4000;   // Starting interval (slow speed)
const unsigned long accelerationStep = 10;      // How much to decrease the interval by each step (controls acceleration rate)
unsigned long currentStepInterval;              // The current step interval, which will change over time

void setup() {
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Initial state
  digitalWrite(DIR_PIN, HIGH); // Set direction to clockwise
  digitalWrite(STEP_PIN, LOW); // Initialize step pin
  digitalWrite(RELAY_PIN, LOW); // Turn relay ON (active low)
  
  // Initialize the starting speed
  currentStepInterval = initialStepInterval; 
}

void loop() {
  unsigned long currentMicros = micros();

  // Non-blocking stepper motor control
  if (currentMicros - lastStepTime >= currentStepInterval) { // Check against the current, changing interval
  // if (currentMicros - lastStepTime >= STEP_INTERVAL) {
    stepState = !stepState; // Toggle step state
    digitalWrite(STEP_PIN, stepState);
    lastStepTime = currentMicros;

    // --- New Acceleration Logic ---
    // This part runs every time a step is taken.
    // We check if we are still accelerating (i.e., not at max speed).
    if (currentStepInterval > minStepInterval) {
      // Decrease the interval to speed up the motor for the next step.
      currentStepInterval -= accelerationStep;

      // Make sure we don't accelerate past our target speed.
      if (currentStepInterval < minStepInterval) {
        currentStepInterval = minStepInterval;
      } 
    }
  }
}