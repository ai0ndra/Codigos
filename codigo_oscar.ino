#include <Servo.h>

// Serial Command Buffer
const int SERIAL_BUFFER_SIZE = 32; // Max length for serial commands
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// Pin definitions
const int dirPin = 6; // Stepper motor direction pin
const int stepPin = 7; // Stepper motor step pin
const int switchPin = 2; // Limit switch for stepper motor (Normally Closed contact)

// Servo related pins
const int servoPin = 5; // Servo motor control pin
const int servoSwitchPin = 4; // Switch for servo control input (Pin 4)

// Debouncing variables for servoSwitchPin (Pin 4)
int servoSwitch_lastState = HIGH;       // Last stable state of the servo switch
int servoSwitch_currentState;           // Current debounced state of the servo switch
unsigned long servoSwitch_lastDebounceTime = 0; // Last time the servo switch output changed
const unsigned long servoSwitch_debounceDelay = 50; // Debounce time in milliseconds

// Switch 2 (Pin 4) press counter and halt logic
int switch2_pressCount = 0;                 // Counter for presses of servoSwitchPin
const int switch2_maxPressCount = 3;        // Max press count for normal modes (e.g., Com1, Com3)
const int switch2_maxPressCountC = 1000;    // Max press count for continuous modes (e.g., Com2, Com4) - effectively infinite for typical use
bool systemHalted_switch2Max = false;       // Flag: true if system is halted due to max switch 2 presses

// Stepper motor control variables
volatile byte triggerState = LOW;     // State of the stepper limit switch (switchPin), set by ISR
int movimientos = 0;                  // Counter for stepper movements, used for step correction logic
unsigned long lastInterruptTime = 0;  // Last time the switchPin ISR was triggered (for debouncing)
unsigned long interruptDebounceDelay = 100; // Debounce delay for switchPin ISR
volatile bool motorIsMoving = false;  // Flag: true if stepper motor is currently executing a move

// Variables for stepper motor timing and state management
unsigned long stepper_lastHomeTime = 0;         // Timestamp of the last successful homing (switchPin activation)
const unsigned long startupHoming_timeout = 10000; // Timeout (ms) for each startup homing attempt
const unsigned long normalOp_noHomeTimeout = 10000; // 10 seconds for override in normal operation

// Variables for Startup Homing / 3-Try Recovery System
int startupHoming_attemptCounter = 0;     // Counts attempts for startup homing
int nemaOverrideAttemptCounter = 0; // Counter for NEMA override attempts
bool system_startupHomingFailed = false;  // Flag: true if startup homing failed after 3 attempts
bool systemIsInStartupHomingPhase = true; // Flag: true if system is currently in startup homing/recovery phase

Servo myServo; // Servo object declaration

// Variables for servo's timed hold logic
bool servo_isHoldingAt90 = false;      // Flag: true if servo is currently holding at 90 degrees
unsigned long servo_reached90Time = 0; // Timestamp when servo reached 90 degrees
const unsigned long servoHoldDuration_Mode12 = 5000; // Servo hold duration (ms) for Modes 1, 2, & 4 (5 seconds)
const unsigned long servoHoldDuration_Mode34 = 8000; // Servo hold duration (ms) for Mode 3 (8 seconds)

// Variables for managing servo active state
bool servo_isActive = false;           // Flag: true if servo is considered active (e.g., moving to or holding at 90)
unsigned long servo_finishTime = 0;    // Timestamp when servo completed its return to 0 (currently informational)
bool canServoBeActivated = false; // True if servo can be activated (after a NEMA 'contador' event)

// Variables for NEMA motor control relative to servo activity
const unsigned long nema_quietTimeAfterServo = 2000; // Duration NEMA stays quiet after servo finishes (ms)
bool allowNemaMovement = true; // Flag to control NEMA motor movement based on servo activity
unsigned long servoBecameInactiveTime = 0; // Timestamp for when the servo returned to 0/inactive state

// System mode activation flags (set by Serial commands)
bool systemActive_afterCom1 = false; // True if system mode '1' is active
bool systemActive_afterCom2 = false; // True if system mode '2' is active
bool systemActive_afterCom3 = false; // True if system mode '3' is active
bool systemActive_afterCom4 = false; // True if system mode '4' is active

// System pause state flag
bool system_pausa = false; // True if system is paused (starts active/not paused)

// Flag to indicate if servo switch (Pin 4) was just pressed (debounced)
bool servoSwitch_just_pressed_debounced = false;

// Flag to indicate if NEMA quiet time has just elapsed in the current cycle
bool nemaQuietTimeJustElapsed = false; 
// Stores the state of allowNemaMovement from the previous cycle
bool previous_allowNemaMovement_state = false; 
// True if Switch 1 ISR path (Path 1) has processed a NEMA move in the current executeCoreLogic cycle
bool switch1_processed_by_ISR_this_cycle = false; 

// True if Switch 1 was pressed during startup homing, now waiting 2s for a step
bool waiting_for_step_after_homing_press = false; 
// Timestamp when Switch 1 was pressed during startup homing
unsigned long switch1_pressed_during_homing_time = 0; 


/*
 * Executes the core operational logic of the system.
 * 
 * This function is called when the system is active and not paused. It handles:
 * 1. Startup Homing: Ensures the stepper motor is homed before normal operation.
 * 2. Normal Operation: Moves the stepper motor upon limit switch trigger and controls the servo.
 * 3. Stepper Override: Allows manual stepper operation under certain conditions.
 * 4. Servo Switch Debouncing & Counting: Manages input from the servo control switch.
 * 5. Max Press Count Halt: Stops operation if the servo switch is pressed too many times.
 * 6. Servo Timed Return: Returns the servo to its base position after a set duration.
 * 
 * Parameters:
 *   current_switch2_maxPressCount: The maximum number of presses allowed for switch2 (servoSwitchPin)
 *                                  before the system halts for the current operational mode.
 *   current_servo_Duration: The duration (in milliseconds) the servo should hold its position (e.g., 90 degrees)
 *                           before returning to 0 for the current operational mode.
 */
void executeCoreLogic(int current_switch2_maxPressCount, unsigned long current_servo_Duration) {
  // Reset one-cycle flag at the beginning of each logic execution
  switch1_processed_by_ISR_this_cycle = false; 

  // Store the previous state of allowNemaMovement to detect transitions
  previous_allowNemaMovement_state = allowNemaMovement;

  // --- Update NEMA Movement Allowance based on Servo State ---
  if (servo_isActive) {
      allowNemaMovement = false; // Servo is active, NEMA should not move
  } else {
      // Servo is not active, check if quiet time has passed
      if (millis() - servoBecameInactiveTime >= nema_quietTimeAfterServo) {
          allowNemaMovement = true; // Quiet time elapsed, NEMA can move
      } else {
          // Servo is inactive, but quiet time is still in effect
          allowNemaMovement = false; 
      }
  }

  // Check if the NEMA quiet time has just elapsed in this cycle
  if (allowNemaMovement && !previous_allowNemaMovement_state) {
      nemaQuietTimeJustElapsed = true; // NEMA quiet time just ended in this cycle
  } else {
      nemaQuietTimeJustElapsed = false; // Reset if not just elapsed or still not allowed
  }

  // --- NEW: NEMA Trigger based on Switch 1 state AFTER post-servo quiet time ---
  // This runs if:
  // 1. Servo is not currently active.
  // 2. NEMA quiet time just elapsed in this cycle.
  // 3. The ISR for switchPin has NOT already set triggerState.
  // 4. The motor is not already moving.
  // 5. This event was not already processed by ISR path in this cycle.
  if (nemaQuietTimeJustElapsed && triggerState == LOW && !motorIsMoving && !switch1_processed_by_ISR_this_cycle) {
      if (digitalRead(switchPin) == LOW) { // Check current state of Switch 1 (Pin 2)
          canServoBeActivated = true;     // ARM servo activation for this NEMA cycle
          Serial.println(F("NEMA trigger: Switch 1 pressed post-quiet time (Path 2).")); // Debug
          
          motorIsMoving = true;
          moverPasoCorregido();
          motorIsMoving = false;
          
          stepper_lastHomeTime = millis(); // Update last home/active time
          nemaOverrideAttemptCounter = 0;  // Reset override attempts
          nemaQuietTimeJustElapsed = false; // Consume this one-time trigger event

          // Check Switch 1 state upon completion of NEMA move
          if (digitalRead(switchPin) == LOW) {
              // If Switch 1 is (still) pressed, ensure servo is armed and timers reflect Switch 1 activity
              canServoBeActivated = true; 
              stepper_lastHomeTime = millis(); 
              nemaOverrideAttemptCounter = 0;
              Serial.println(F("Normal Op (Path 2): Switch 1 found LOW post-NEMA. Servo re-armed/confirmed.")); // Debug
          }
      }
      // If switchPin was not LOW when nemaQuietTimeJustElapsed was true, 
      // nemaQuietTimeJustElapsed will be reset to false in the next cycle's update block anyway.
  }

  // --- Startup Homing Logic Block ---
  // This block manages the initial homing sequence for the stepper motor.
  if (systemIsInStartupHomingPhase && !system_startupHomingFailed && !motorIsMoving) {
      
      // Path A Part 2: Execute delayed step if Switch 1 was pressed and 2s have passed
      if (waiting_for_step_after_homing_press) {
          if (millis() - switch1_pressed_during_homing_time >= 2000) { // Check if 2 seconds have passed
              if (allowNemaMovement) { // Check if NEMA can move (respects NEMA quiet time)
                  Serial.println(F("Startup Homing: Waited 2s after Switch 1 press, making step...")); // Debug
                  
                  motorIsMoving = true;
                  moverPasoCorregido();
                  motorIsMoving = false;

                  // Now complete the homing success actions
                  systemIsInStartupHomingPhase = false; // Exit homing phase
                  stepper_lastHomeTime = millis();      // Mark successful home time
                  startupHoming_attemptCounter = 0;     // Reset attempt counter
                  canServoBeActivated = true;           // This NEMA move enables servo
                  nemaOverrideAttemptCounter = 0;       // Reset NEMA override counter

                  waiting_for_step_after_homing_press = false; // Reset this flag, delayed step is done

                  // NEW: Immediately check Switch 2 (servoSwitchPin) if it's held, 
                  //      now that servo is armed from the delayed startup step.
                  if (digitalRead(servoSwitchPin) == LOW) { // Check direct state of Switch 2
                      // Condition: Servo isn't already at 90, and it's allowed to activate (which it should be)
                      if (!servo_isHoldingAt90 && canServoBeActivated) { 
                          Serial.println(F("Servo directly activated post-delayed-startup-step by held Switch 2.")); // Debug

                          // --- Replicate Servo Activation Core Logic ---
                          Serial.println(F("contador"));
                          // Increment press count only if not already halted by this counter
                          if (!systemHalted_switch2Max) {
                               switch2_pressCount++;
                          }
                          // Check for "ter" - print if limit met and not already halted
                          if (!systemHalted_switch2Max && (switch2_pressCount >= current_switch2_maxPressCount)) {
                              Serial.println(F("ter"));
                              systemHalted_switch2Max = true;
                          }

                          myServo.write(90);
                          digitalWrite(8, HIGH);
                          servo_isHoldingAt90 = true;
                          servo_reached90Time = millis();
                          servo_isActive = true;
                          allowNemaMovement = false;      // Disallow NEMA now servo is active
                          canServoBeActivated = false;      // Consume the activation permission
                          nemaQuietTimeJustElapsed = false; // Reset this flag too
                          // --- End of Replicated Servo Activation ---
                      }
                  }
              }
              // If NEMA cannot move now (e.g., quiet time), this block will be re-evaluated 
              // in the next cycle as long as waiting_for_step_after_homing_press remains true.
          }
          // While waiting for the 2s or for allowNemaMovement, do nothing else in startup homing.
      } 
      // Path A Part 1: Initial detection of Switch 1 press (sets up the waiting flag)
      else if (triggerState == HIGH) { 
          // This logic was implemented in the previous step:
          switch1_pressed_during_homing_time = millis();
          waiting_for_step_after_homing_press = true;
          triggerState = LOW; // Consume this trigger to prevent immediate normal operation.
          Serial.println(F("Startup Homing: Switch 1 pressed, starting 2s wait...")); // Debug
      } 
      // Path B: Timeout-driven recovery step (only if not waiting for delayed step from Path A)
      else if ((millis() - stepper_lastHomeTime) >= startupHoming_timeout) {
          // This is the existing logic for timeout-based recovery attempts.
          if (startupHoming_attemptCounter < 3) { // Only attempt if counter < 3
              if (allowNemaMovement) { // Respect NEMA quiet time
                  Serial.print(F("Startup Homing Attempt #")); // Debug
                  Serial.println(startupHoming_attemptCounter + 1);

                  motorIsMoving = true;
                  moverPasoCorregido();
                  motorIsMoving = false;

                  // Check if the recovery NEMA move successfully pressed Switch 1
                  if (digitalRead(switchPin) == LOW) { // Switch 1 HELD LOW after recovery move
                      Serial.println(F("Startup Homing: Recovery NEMA step SUCCESSFUL - Switch 1 is HELD LOW.")); // Debug

                      systemIsInStartupHomingPhase = false;    // Exit homing phase
                      stepper_lastHomeTime = millis();          // Mark successful home time
                      startupHoming_attemptCounter = 0;         // Reset attempt counter
                      canServoBeActivated = true;               // ARM SERVO
                      nemaOverrideAttemptCounter = 0;           // Reset NEMA override counter
                      triggerState = LOW;                       // Ensure triggerState is low (as we've handled this press)
                      waiting_for_step_after_homing_press = false; // Ensure we are not waiting for a delayed step
                  } else { 
                      // Switch 1 NOT HELD LOW after recovery move.
                      // A brief click might have set triggerState HIGH via ISR. If so, clear it.
                      if (triggerState == HIGH) {
                          Serial.println(F("Startup Homing: Recovery NEMA step caused brief Switch 1 press (ISR), but not held. Attempt failed.")); // Debug
                          triggerState = LOW; // Consume the brief press from this failed recovery.
                      } else {
                          Serial.println(F("Startup Homing: Recovery NEMA step FAILED to hit Switch 1 at all.")); // Debug
                      }
                      // This attempt failed to secure a HELD LOW on Switch 1.
                      startupHoming_attemptCounter++; 
                      stepper_lastHomeTime = millis(); // Reset timer for the next 10s wait
                  }
              } else { // NEMA movement is NOT allowed right now (e.g., servo quiet time)
                  Serial.println(F("Startup Homing: Recovery NEMA move deferred by allowNemaMovement."));
                  stepper_lastHomeTime = millis(); // Reset timer to start a new 10s observation
                  // DO NOT increment startupHoming_attemptCounter here, as the attempt was deferred.
              }
          } 
          
          // This check is now outside the 'if (allowNemaMovement)' block,
          // so it correctly processes failure if attempts are exhausted,
          // regardless of whether the last attempt was made or deferred.
          if (startupHoming_attemptCounter >= 3) { 
              if (!system_startupHomingFailed) { 
                  Serial.println(F("vacio")); 
                  system_startupHomingFailed = true;
              }
              systemIsInStartupHomingPhase = false; 
          }
      }
  }
  // --- End Startup Homing Logic Block ---

  // --- Normal Operation Phase Logic ---
  // This block runs if not in startup homing, homing hasn't failed, the limit switch (triggerState) is HIGH, and motor is idle.
  if (!systemIsInStartupHomingPhase && !system_startupHomingFailed && triggerState == HIGH && !motorIsMoving) {
    // ISR detected a press (triggerState is HIGH). This path (Path 1) attempts to make a NEMA move.
    // Servo activation is armed regardless of whether NEMA moves immediately or is deferred.
    canServoBeActivated = true; // Armed by Switch 1 detection via ISR/triggerState

    if (allowNemaMovement) {
      // NEMA movement is allowed
      motorIsMoving = true;
      moverPasoCorregido(); // Perform the main stepper motor movement
      motorIsMoving = false;
      
      // Serial.println("contador"); // "contador" is now tied to servo activation

      // Update states after successful NEMA operation triggered by switchPin
      // canServoBeActivated = true; // Moved earlier in this block
      stepper_lastHomeTime = millis(); // Update last home/active time
      nemaOverrideAttemptCounter = 0;  // Reset override attempts as primary mechanism worked
      triggerState = LOW;             // Consume the trigger
      switch1_processed_by_ISR_this_cycle = true; // Mark this Switch 1 event as processed by ISR path

      // Check Switch 1 state upon completion of NEMA move
      if (digitalRead(switchPin) == LOW) {
          // If Switch 1 is (still) pressed, ensure servo is armed and timers reflect Switch 1 activity
          canServoBeActivated = true; 
          stepper_lastHomeTime = millis(); 
          nemaOverrideAttemptCounter = 0;
          Serial.println(F("Normal Op (ISR Path): Switch 1 found LOW post-NEMA. Servo re-armed/confirmed.")); // Debug
      }
    } else {
      // NEMA movement deferred. 
      // Consume the trigger to prevent re-triggering Path 1 continuously while NEMA is disallowed.
      // Servo remains armed by canServoBeActivated = true from above.
      triggerState = LOW; 
      switch1_processed_by_ISR_this_cycle = true; // Indicate Switch 1 event was processed (even if NEMA didn't move)
      // Serial.println(F("NEMA movement deferred in Normal Op (Path 1) - trigger consumed."));
    }

    // Servo Trigger Logic - now uses the debounced flag
    if (servoSwitch_just_pressed_debounced) {
      servoSwitch_just_pressed_debounced = false; // Consume the debounced press event

      // And servo is not already at 90, AND it's allowed to be activated
      if (!servo_isHoldingAt90 && canServoBeActivated) { 
        Serial.println(F("contador")); // "contador" now sent upon successful servo activation
        switch2_pressCount++;          // Increment press count only on successful servo activation

        // Check for "ter" condition immediately after successful activation and count increment
        if (!systemHalted_switch2Max && (switch2_pressCount >= current_switch2_maxPressCount)) {
            Serial.println(F("ter"));
            systemHalted_switch2Max = true;
        }
        
        myServo.write(90);        // Move servo to 90 degrees
        digitalWrite(8, HIGH);    // Set Pin 8 HIGH (indicator for servo active)
        servo_isHoldingAt90 = true; // Update servo state
        servo_reached90Time = millis(); // Record time servo reached 90
        servo_isActive = true;      // Mark servo as active
        allowNemaMovement = false;  // Prevent NEMA movement while servo is active
        canServoBeActivated = false; // Servo activation permission consumed
        nemaQuietTimeJustElapsed = false; // Reset flag as servo is now active (and NEMA movement is disallowed)
      }
    }
  }
  // --- End Normal Operation Phase Logic ---

  // --- Normal Operation Stepper Override ---
  // This block allows a stepper move if the system is in normal operation, not failed homing, motor is idle,
  // limit switch (triggerState) is LOW, and a timeout has occurred with the servo switch pressed.
  // It also now includes the allowNemaMovement check.
  if (!systemIsInStartupHomingPhase && !system_startupHomingFailed && !motorIsMoving && triggerState == LOW && allowNemaMovement) {
    // Check for 10s inactivity of Switch 1 (stepper_lastHomeTime) AND Pin 4 (servoSwitchPin) being pressed
    if (((millis() - stepper_lastHomeTime) >= normalOp_noHomeTimeout) && (digitalRead(servoSwitchPin) == LOW)) {
      
      canServoBeActivated = false; // Servo cannot be activated from an override NEMA move

      // Optional: Serial.println(F("Normal Op Override Triggered"));
      motorIsMoving = true;
      moverPasoCorregido(); 
      motorIsMoving = false;

      // After the movement, check if Switch 1 (switchPin) was pressed by this override move.
      if (triggerState == LOW) { // Switch 1 still NOT pressed by the override move
          nemaOverrideAttemptCounter++; 
          stepper_lastHomeTime = millis(); // Reset timer for the next 10s observation/attempt

          if (nemaOverrideAttemptCounter >= 3) {
              Serial.println(F("vacio")); 
              nemaOverrideAttemptCounter = 0; 
          }
      } else { // Switch 1 WAS pressed (triggerState is HIGH)
          nemaOverrideAttemptCounter = 0; 
          // stepper_lastHomeTime will be updated by the Normal Operation block when it processes triggerState == HIGH
          // and canServoBeActivated will also be set to true there.
      }
    }
  }
  // --- End Normal Operation Stepper Override ---

  // --- Pin 4 (servoSwitchPin) Debouncing and Press Counting START ---
  // This section reads the servoSwitchPin, debounces its input, and counts presses.
  int reading_servoSwitch = digitalRead(servoSwitchPin);

  if (reading_servoSwitch != servoSwitch_lastState) { // If the switch reading has changed
    servoSwitch_lastDebounceTime = millis(); // Reset the debouncing timer
  }

  if ((millis() - servoSwitch_lastDebounceTime) > servoSwitch_debounceDelay) { // If current reading is stable for longer than debounce delay
    if (reading_servoSwitch != servoSwitch_currentState) { // If the debounced state has changed
      servoSwitch_currentState = reading_servoSwitch; // Update the official debounced state

      if (servoSwitch_currentState == LOW) { // If switch was just pressed (debounced LOW signal)
        servoSwitch_just_pressed_debounced = true; // Set flag for processing in Servo Trigger Logic
        // switch2_pressCount is no longer incremented here.
      }
    }
  }
  servoSwitch_lastState = reading_servoSwitch; // Save the current raw reading for the next loop
  // --- Pin 4 (servoSwitchPin) Debouncing and Press Counting END ---

  // --- Switch 2 (Pin 4) Counter Limit Check and Halt Logic START --- (This block is now removed, logic integrated above)
  // --- Switch 2 (Pin 4) Counter Limit Check and Halt Logic END ---

  // --- Servo Timed Return to 0 Logic ---
  // If servo is at 90 degrees, check if its hold duration has elapsed.
  if (servo_isHoldingAt90) {
    if ((millis() - servo_reached90Time) >= current_servo_Duration) { // If hold time is over
      myServo.write(0);        // Return servo to 0 degrees
      digitalWrite(8, LOW);    // Set Pin 8 LOW (indicator for servo inactive)
      servo_isHoldingAt90 = false; // Update servo state
      servo_isActive = false;      // Mark servo as inactive
      servo_finishTime = millis(); // Record time servo returned (informational)
      servoBecameInactiveTime = millis(); // Record time for NEMA quiet period logic
    }
  }
}


/*
 * Main wrapper function for system operations.
 * 
 * This function acts as an entry point to the system's core logic.
 * It checks if the system is paused. If not paused, it determines the active
 * system mode (based on Com1-Com4 flags) and calls executeCoreLogic() 
 * with the appropriate parameters for that mode.
 */
void run_codigo_oscar() {
  if (!system_pausa) { // Only run if the system is not paused
    // Call executeCoreLogic with parameters specific to the active system mode
    if (systemActive_afterCom1) {
      executeCoreLogic(switch2_maxPressCount, servoHoldDuration_Mode12);
    } else if (systemActive_afterCom2) {
      executeCoreLogic(switch2_maxPressCountC, servoHoldDuration_Mode12);
    } else if (systemActive_afterCom3) {
      executeCoreLogic(switch2_maxPressCount, servoHoldDuration_Mode34);
    } else if (systemActive_afterCom4) { 
      executeCoreLogic(switch2_maxPressCount, servoHoldDuration_Mode12); // Assuming Com4 uses parameters similar to Com1/Com2
    }
  }
}

/*
 * Interrupt Service Routine for switchPin (stepper limit switch).
 * 
 * This function is called when switchPin (Pin 2) triggers a FALLING edge.
 * It debounces the switch input. If a valid trigger occurs (motor not moving, 
 * system not failed homing), it sets triggerState to HIGH.
 */
void switchAction() {
  unsigned long currentTime = millis();
  // Debounce: check if enough time has passed since the last interrupt
  if ((currentTime - lastInterruptTime) > interruptDebounceDelay) {
    // Only set triggerState if motor is idle and startup homing hasn't failed,
    // to prevent processing triggers during movement or if system is critically non-operational.
    if (!motorIsMoving && !system_startupHomingFailed) { 
      triggerState = HIGH; 
    }
    lastInterruptTime = currentTime; // Update the last interrupt time
  }
}

/*
 * Controls the stepper motor movement with a correction mechanism.
 * 
 * Moves the stepper motor a specific number of steps.
 * Includes a correction mechanism where every 3rd movement has a slightly
 * different number of steps (2134 vs 2133) to average out positioning over time.
 * The direction is fixed (HIGH).
 */
void moverPasoCorregido() {
  movimientos++; // Increment movement counter
  int pasos;
  // Apply step correction based on the number of movements
  if (movimientos % 3 == 0) {
    pasos = 2134; 
  } else {
    pasos = 2133;
  }

  digitalWrite(dirPin, HIGH); // Set direction (assuming HIGH is the desired direction)
  // Execute step pulses
  for (int i = 0; i < pasos; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

/*
 * Standard Arduino setup function.
 * 
 * Initializes Serial communication, configures pin modes for stepper, servo,
 * and switches. Attaches the servo and sets its initial position.
 * Attaches an interrupt for the stepper limit switch.
 * Initializes various state variables.
 */
void setup() {
  Serial.begin(57600); // Initialize Serial communication
  while (!Serial) {
    ; // Wait for Serial port to connect (needed for some Arduinos like Leonardo)
  }
  Serial.println("Iniciando sistema en Arduino Nano...");

  // Configure pin modes
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);     // Stepper limit switch with internal pull-up
  pinMode(servoSwitchPin, INPUT_PULLUP); // Servo control switch with internal pull-up
  pinMode(8, OUTPUT);                   // Pin 8 as an output (e.g., servo active indicator)
  digitalWrite(8, LOW);                 // Initialize Pin 8 to LOW

  // Initialize servo
  myServo.attach(servoPin); // Attach servo to its pin
  myServo.write(0);         // Set servo to initial position (0 degrees)

  // Attach interrupt for switchPin (stepper limit switch)
  // Triggers on FALLING edge because of INPUT_PULLUP (HIGH normally, LOW when pressed)
  attachInterrupt(digitalPinToInterrupt(switchPin), switchAction, FALLING);

  // Initialize switch states
  servoSwitch_currentState = digitalRead(servoSwitchPin); // Read initial state of servo switch
  // Force immediate check of homing on startup by setting last home time to be older than timeout
  stepper_lastHomeTime = millis() - startupHoming_timeout; 

  // --- Servo Initial Startup Sequence ---
  Serial.println(F("Starting initial servo sequence..."));

  myServo.write(90);      // Move servo to 90 degrees
  digitalWrite(8, HIGH);  // Set Pin 8 HIGH (indicator)
  
  delay(5000);            // Hold for 5 seconds

  myServo.write(0);       // Move servo back to 0 degrees
  digitalWrite(8, LOW);   // Set Pin 8 LOW
  
  Serial.println(F("Initial servo sequence complete."));

  // Set initial servo states after startup sequence
  servo_isHoldingAt90 = false;
  servo_isActive = false;
  // Record the time the servo became inactive to respect NEMA quiet time from startup
  servoBecameInactiveTime = millis(); 
}

/*
 * Standard Arduino loop function.
 * 
 * Continuously checks for Serial commands and runs the main system logic
 * via run_codigo_oscar() if any system mode is active.
 */
void loop() {
  // Check for incoming Serial commands and buffer them
  if (Serial.available()) {
      char receivedChar = Serial.read();

      if (receivedChar == '\n') { // Newline character marks end of command
          serialBuffer[serialBufferIndex] = '\0'; // Null-terminate the string

          if (serialBufferIndex > 0) { // Process only if buffer is not empty
              if (strcmp(serialBuffer, "pausa") == 0) {
                  system_pausa = true;
                  Serial.println(F("System Paused (Command pausa)."));
                  myServo.write(0);
                  digitalWrite(8, LOW); 
                  servo_isHoldingAt90 = false;
                  servo_isActive = false;
                  servoBecameInactiveTime = millis();
              } else if (strcmp(serialBuffer, "despausa") == 0) {
                  system_pausa = false;
                  Serial.println(F("System Resumed (Command despausa)."));
              } else if (strcmp(serialBuffer, "CMD:1") == 0) {
                  if (!systemActive_afterCom1) {
                      Serial.println(F("System activated by command '1'.")); // Message can stay as is or be updated
                      systemActive_afterCom1 = true;
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(8, LOW);
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                  }
                  systemActive_afterCom2 = false;
                  systemActive_afterCom3 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:2") == 0) {
                  if (!systemActive_afterCom2) {
                      Serial.println(F("System activated by command '2'.")); // Message can stay as is or be updated
                      systemActive_afterCom2 = true;
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(8, LOW);
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                  }
                  systemActive_afterCom1 = false;
                  systemActive_afterCom3 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:3") == 0) {
                  if (!systemActive_afterCom3) {
                      Serial.println(F("System activated by command '3'.")); // Message can stay as is or be updated
                      systemActive_afterCom3 = true;
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(8, LOW);
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                  }
                  systemActive_afterCom1 = false;
                  systemActive_afterCom2 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:4") == 0) {
                  if (!systemActive_afterCom4) {
                      Serial.println(F("System activated by command '4'.")); // Message can stay as is or be updated
                      systemActive_afterCom4 = true;
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(8, LOW);
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                  }
                  systemActive_afterCom1 = false;
                  systemActive_afterCom2 = false;
                  systemActive_afterCom3 = false;
              } else {
                  Serial.print(F("Unknown command: ["));
                  Serial.print(serialBuffer);
                  Serial.println(F("]"));
              }
          }
          serialBufferIndex = 0; // Reset buffer index for the next command
      } else if (receivedChar == '\r') {
          // Ignore carriage return character, often sent with newline by Serial Monitor
      } else {
          // Add character to buffer if space is available
          if (serialBufferIndex < (SERIAL_BUFFER_SIZE - 1)) {
              serialBuffer[serialBufferIndex++] = receivedChar;
          } else {
              // Buffer overflow, discard and reset to prevent issues
              Serial.println(F("Error: Command too long. Buffer cleared."));
              serialBufferIndex = 0; 
          }
      }
  }

  // Consolidated call to the main operational logic.
  // This ensures run_codigo_oscar() is called once per loop iteration if any system mode is active.
  // The run_codigo_oscar() function itself handles the system_pausa state.
  if (systemActive_afterCom1 || systemActive_afterCom2 || systemActive_afterCom3 || systemActive_afterCom4) {
    run_codigo_oscar();
  }
}
