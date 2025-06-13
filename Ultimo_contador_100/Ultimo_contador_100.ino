#include <Servo.h>

const int dirPin = 6; // Stepper motor direction
const int stepPin = 7; // Stepper motor step
const int switchPin = 2; // Limit switch for stepper motor (Normalmente Cerrado)

// Servo related pins
const int servoPin = 5;
const int servoSwitchPin = 4; // Switch for servo control (Pin 4)

// Debouncing variables for servoSwitchPin (Pin 4)
int servoSwitch_lastState = HIGH; // Initialized assuming not pressed
int servoSwitch_currentState;     // To be initialized in setup()
unsigned long servoSwitch_lastDebounceTime = 0;
const unsigned long servoSwitch_debounceDelay = 50; // 50ms debounce delay

// Switch 2 (Pin 4) press counter and halt logic
int switch2_pressCount = 0;
const int switch2_maxPressCount = 10;
bool systemHalted_switch2Max = false;

volatile byte triggerState = LOW; // For stepper motor trigger
int movimientos = 0; // For stepper motor logic (used in moverPasoCorregido)
unsigned long lastInterruptTime = 0; // For stepper switch (switchPin) debounce
unsigned long interruptDebounceDelay = 100; // For stepper switch (switchPin) debounce
volatile bool motorIsMoving = false; // For stepper motor state

// Variables for stepper motor timing and state
unsigned long stepper_lastHomeTime = 0; // Timestamp of last home switch activation
const unsigned long startupHoming_timeout = 10000; // 10 seconds for startup homing attempts
const unsigned long normalOp_noHomeTimeout = 20000; // 20 seconds for override in normal operation

// Variables for Startup Homing / 3-Try Recovery System
int startupHoming_attemptCounter = 0;
bool systemHalted_startupHomingFailed = false;
bool systemIsInStartupHomingPhase = true; // System starts in Homing/Recovery phase

Servo myServo; // Declare Servo object

// Variables for servo's timed hold logic
bool servo_isHoldingAt90 = false;
unsigned long servo_reached90Time = 0;
const unsigned long servo_Duration = 5000; // 5 seconds for servo hold (renamed)

// Variables for managing servo active state and NEMA quiet time
// These were from a more complex interaction model. Their effect in the snapshot's loop() is minimal.
bool servo_isActive = false; 
unsigned long servo_finishTime = 0; 
const unsigned long nema_quietTimeAfterServo = 2000;

bool systemActive_afterCom1 = false; // System is initially inactive until "com1" is received

void setup() {
  Serial.begin(115200); // Initialize Serial communication

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); 
  pinMode(servoSwitchPin, INPUT_PULLUP); 
  pinMode(8, OUTPUT);      // Configure Pin 8 as output
  digitalWrite(8, LOW);    // Initialize Pin 8 to LOW

  attachInterrupt(digitalPinToInterrupt(switchPin), switchAction, FALLING); // ISR for switchPin

  myServo.attach(servoPin);
  myServo.write(0);      
  
  servoSwitch_currentState = digitalRead(servoSwitchPin); // Initialize debounced state of servoSwitchPin
  stepper_lastHomeTime = millis() - startupHoming_timeout; // To encourage quick first recovery check
}

void loop() {
  // --- Serial Command Listener ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace
    if (command.equals("com1")) {
      if (!systemActive_afterCom1) { // Optional: only print/reset if not already active
          Serial.println("System activated by com1.");
          systemActive_afterCom1 = true;
          
          // Re-initialize system states for a fresh start / homing sequence
          systemIsInStartupHomingPhase = true;     // Start in Homing/Recovery phase
          systemHalted_startupHomingFailed = false; // Clear any previous halt state
          startupHoming_attemptCounter = 0;        // Reset recovery attempts
          // Use startupHoming_timeout for the initial setup to encourage quick first recovery check
          stepper_lastHomeTime = millis() - startupHoming_timeout; 
          triggerState = LOW;                    // Clear any pending stepper trigger
          motorIsMoving = false;                 // Ensure motor is marked as not moving
          
          // Reset servo states as well
          servo_isActive = false;
          servo_isHoldingAt90 = false;
          myServo.write(0); // Ensure servo is at 0 before homing starts
          // servo_finishTime and servo_reached90Time will be set when servo next operates.
      }
    }
  }
  // --- End Serial Command Listener ---

  if (systemActive_afterCom1) {
    // --- Startup Homing Logic Block ---
    if (systemIsInStartupHomingPhase && !systemHalted_startupHomingFailed && !motorIsMoving) {
      if (triggerState == HIGH) { // Switch 1 (switchPin) was pressed
          systemIsInStartupHomingPhase = false; // Transition to Normal Operation
          stepper_lastHomeTime = millis();      // Mark successful home time
          startupHoming_attemptCounter = 0;    // Reset counter
          // triggerState remains HIGH for the Normal Operation block to handle.
      } else if ((millis() - stepper_lastHomeTime) >= startupHoming_timeout) { // 10s timeout
          if (startupHoming_attemptCounter < 3) {
              startupHoming_attemptCounter++;
              // Optional: Serial.print("Startup Homing Attempt #"); Serial.println(startupHoming_attemptCounter);
              
              motorIsMoving = true;            
              moverPasoCorregido();            
              motorIsMoving = false;           

              if (triggerState == LOW) { // If Switch 1 still not pressed by the recovery move
                  stepper_lastHomeTime = millis(); // Reset timer for the next 10s wait
              }
              // If triggerState is HIGH, the (triggerState == HIGH) block above will catch it next loop.
          } else { // 3 attempts are done
              if (!systemHalted_startupHomingFailed) { // Send "vacio" only once
                  Serial.println("vacio");
                  systemHalted_startupHomingFailed = true; 
              }
              systemIsInStartupHomingPhase = false; // Exit startup homing phase (system is now halted)
          }
      }
  }
  // --- End Startup Homing Logic Block ---

  // --- Normal Operation Phase Logic ---
  // Condition: Not in homing/recovery, not halted, trigger from Switch 1 is HIGH, motor is idle.
  if (!systemIsInStartupHomingPhase && !systemHalted_startupHomingFailed && triggerState == HIGH && !motorIsMoving) {
      
      stepper_lastHomeTime = millis();
      startupHoming_attemptCounter = 0; // Reset recovery attempts because normal cycle ran
      triggerState = LOW; // Consume the trigger for this cycle
      motorIsMoving = true;
      moverPasoCorregido();
      motorIsMoving = false;
      Serial.println("contador"); // Send message after NEMA move from Switch 1

      // Servo Trigger (from snapshot)
      if (digitalRead(servoSwitchPin) == LOW) {
          if (!servo_isHoldingAt90) {
              myServo.write(90);
              digitalWrite(8, HIGH); // Set Pin 8 HIGH
              servo_isHoldingAt90 = true;
              servo_reached90Time = millis();
              servo_isActive = true; // As per snapshot
          }
      }
  }
  // --- End Normal Operation Phase Logic ---

  // --- Normal Operation Stepper Override ---
  // Active only when not in startup homing, system not halted by startup failure,
  // stepper motor is idle, and no primary trigger from switchPin is pending.
  if (!systemIsInStartupHomingPhase && !systemHalted_startupHomingFailed && !motorIsMoving && triggerState == LOW) {
      
      // Check for 20s inactivity of Switch 1 AND Pin 4 (servoSwitchPin) being pressed
      if (((millis() - stepper_lastHomeTime) >= normalOp_noHomeTimeout) && (digitalRead(servoSwitchPin) == LOW)) {
          
          // Perform the override movement
          // Optional: Serial.println("Normal Op Override Triggered");
          motorIsMoving = true;
          moverPasoCorregido(); // Uses the standard movement
          motorIsMoving = false;

          // After the movement, check if Switch 1 (switchPin) was pressed by this override move.
          // The ISR for switchPin would have set triggerState to HIGH if it was pressed.
          if (triggerState == LOW) { // Switch 1 still NOT pressed by the override move
              Serial.println("vacio"); // Send "vacio" as per user's instruction
          }
          // If triggerState is HIGH, the Normal Stepper Cycle logic will handle it in the next loop,
          // which will update stepper_lastHomeTime and reset startupHoming_attemptCounter.
          // If triggerState is LOW (override didn't hit home), update stepper_lastHomeTime here
          // to restart the 20s observation period for this Normal Operation Override path.
          stepper_lastHomeTime = millis(); 
      }
  }
  // --- End Normal Operation Stepper Override ---

  // --- Pin 4 (servoSwitchPin) Debouncing and Press Counting START ---
  int reading_servoSwitch = digitalRead(servoSwitchPin);

  // If the switch changed, due to noise or pressing:
  if (reading_servoSwitch != servoSwitch_lastState) {
    servoSwitch_lastDebounceTime = millis(); // Reset the debouncing timer
  }

  if ((millis() - servoSwitch_lastDebounceTime) > servoSwitch_debounceDelay) {
    // Whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // If the switch state has changed (debounced):
    if (reading_servoSwitch != servoSwitch_currentState) {
      servoSwitch_currentState = reading_servoSwitch; // Update the official debounced state

      if (servoSwitch_currentState == LOW) { // Switch was just pressed (debounced)
        // Only count presses if the system is not halted by other conditions.
        if (!systemHalted_startupHomingFailed && !systemHalted_switch2Max) {
            switch2_pressCount++;
            // Optional: Serial.print("Switch 2 Press Count: "); Serial.println(switch2_pressCount);
        }
      }
    }
  }
  servoSwitch_lastState = reading_servoSwitch; // Save the current reading for the next loop iteration.
  // --- Pin 4 (servoSwitchPin) Debouncing and Press Counting END ---

  // --- Switch 2 (Pin 4) Counter Limit Check and Halt Logic START ---
  if (!systemHalted_switch2Max && (switch2_pressCount >= switch2_maxPressCount)) {
    Serial.println("ter");
    systemHalted_switch2Max = true; // Set the flag to indicate this halt condition is met
  }
  // --- Switch 2 (Pin 4) Counter Limit Check and Halt Logic END ---

  // Servo Timed Return to 0 Logic (Snapshot version)
  if (servo_isHoldingAt90) {
    if ((millis() - servo_reached90Time) >= servo_Duration) { // Use new constant name
      myServo.write(0); 
      digitalWrite(8, LOW); // Set Pin 8 LOW
      servo_isHoldingAt90 = false; 
      servo_isActive = false;      // Set by snapshot servo logic
      servo_finishTime = millis(); // Set by snapshot servo logic
    }
  }
  } // End of if(systemActive_afterCom1)
}

// ISR for switchPin (Snapshot version)
void switchAction() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > interruptDebounceDelay) {
    // Set trigger only if motor is idle AND system is not halted from startup homing failures
    if (!motorIsMoving && !systemHalted_startupHomingFailed) { 
      triggerState = HIGH; 
    }
    lastInterruptTime = currentTime; 
  }
}

// moverPasoCorregido (Snapshot version - uses 1777/1778 steps)
void moverPasoCorregido() {
  movimientos++;
  int pasos;
  if (movimientos % 3 == 0) {
    pasos = 2134; 
  } else {
    pasos = 2133;
  }
  digitalWrite(dirPin, HIGH); 
  for (int i = 0; i < pasos; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}