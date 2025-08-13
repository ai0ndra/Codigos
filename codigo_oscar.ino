#include <Servo.h>

// Serial Command Buffer
const int SERIAL_BUFFER_SIZE = 32; // Max length for serial commands
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// Pin definitions
const int dirPin = 10; // Stepper motor direction pin
const int stepPin = 11; // Stepper motor step pin
const int switchPin = 27; // Limit switch for stepper motor (Normally Closed contact)

// Servo related pins
const int servoPin = 7; // Servo motor control pin
const int servoSwitchPin = 22; // Switch for servo control input (Pin 4)

const int motorReductorPin = 18;
const int motorVibradorPin = 19;
const int dosificadoraPin = 26;

const int pin12_output_pin = 20;
bool pin12_is_active = false;
unsigned long pin12_start_time = 0;
// Debouncing variables for servoSwitchPin (Pin 4)
int servoSwitch_lastState = HIGH;       // Last stable state of the servo switch
int servoSwitch_currentState;           // Current debounced state of the servo switch
unsigned long servoSwitch_lastDebounceTime = 0; // Last time the servo switch output changed
const unsigned long servoSwitch_debounceDelay = 50; // Debounce time in milliseconds

// Switch 2 (Pin 4) press counter and halt logic
int switch2_pressCount = 0;                 // Counter for presses of servoSwitchPin
const int switch2_maxPressCount = 10;        // Max press count for normal modes (e.g., Com1, Com3)
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

// Variables for Dosificadora timing
unsigned long dosificadora_triggerTime = 0; // Timestamp when servo reached 90, to calculate 200ms delay
unsigned long dosificadora_startTime = 0;   // Timestamp when dosificadora signal went HIGH, for 500ms duration
bool dosificadora_pending = false;          // True if 200ms delay is active before dosificadora pulse
bool dosificadora_active = false;           // True if dosificadora signal is currently HIGH

// Variables for Idle Bottle Search Logic
int idleBottleSearch_attemptCounter = 0;
unsigned long lastSwitch1ActivityOrSearchTime = 0; 


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
      if (digitalRead(switchPin) == LOW) { // Check current state of Switch 1 (Pin 2) initially for this path
          Serial.println(F("NEMA trigger: Switch 1 pressed post-quiet time (Path 2). Making NEMA move.")); // Debug
          
          motorIsMoving = true;
          moverPasoCorregido();
          motorIsMoving = false;
          
          stepper_lastHomeTime = millis(); // Update last home/active time
          nemaOverrideAttemptCounter = 0;  // Reset override attempts
          nemaQuietTimeJustElapsed = false; // Consume this one-time trigger event
          canServoBeActivated = (digitalRead(switchPin) == LOW); // Set based on actual bottle presence post-move
          if(canServoBeActivated) {
              Serial.println(F("Normal Op (Path 2): Switch 1 found LOW post-NEMA. Servo armed.")); // Debug
          } else {
              Serial.println(F("Normal Op (Path 2): Switch 1 NOT LOW post-NEMA. Servo NOT armed.")); // Debug
          }
          lastSwitch1ActivityOrSearchTime = millis();
          idleBottleSearch_attemptCounter = 0;
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
                  canServoBeActivated = (digitalRead(switchPin) == LOW); // Set based on actual bottle presence
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
                          digitalWrite(motorVibradorPin, HIGH);
                          servo_isHoldingAt90 = true;
                          servo_reached90Time = millis();
                          servo_isActive = true;
                          dosificadora_triggerTime = millis(); // Capture time when servo is commanded to 90
                          dosificadora_pending = true;
                          dosificadora_active = false;        // Ensure it's reset if re-triggering
                          digitalWrite(dosificadoraPin, LOW); // Ensure it's initially LOW if re-triggering
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
          if (digitalRead(switchPin) == LOW) { // NEW: Priority check if Switch 1 is HELD LOW on timeout
              Serial.println(F("Startup Homing: Timeout override - Switch 1 HELD. Transitioning to 2s wait.")); // Debug
              switch1_pressed_during_homing_time = millis();
              waiting_for_step_after_homing_press = true;
              triggerState = LOW; // Consume any ISR trigger
              system_startupHomingFailed = false; // Clear previous failure state
              startupHoming_attemptCounter = 0;   // Reset attempts
              // This path effectively bypasses the normal attempt counting and failure for this timeout event.
          } else { // Switch 1 is NOT HELD LOW on timeout, proceed with normal attempt/failure logic
              // Existing logic for attempts < 3 (Path B)
              if (startupHoming_attemptCounter < 3) {
                  if (allowNemaMovement) { // Respect NEMA quiet time
                      Serial.print(F("Startup Homing Attempt #")); // Debug
                      Serial.println(startupHoming_attemptCounter + 1);

                      motorIsMoving = true;
                      moverPasoCorregido();
                      motorIsMoving = false;

                      if (digitalRead(switchPin) == LOW) { // Check if Switch 1 is HELD LOW *after this NEMA move*
                          Serial.println(F("Startup Homing: Recovery NEMA HELD Switch 1. Transitioning to 2s wait."));
                          switch1_pressed_during_homing_time = millis();
                          waiting_for_step_after_homing_press = true;
                          triggerState = LOW; 
                      } else { // Switch 1 was NOT held low after the recovery NEMA move
                          if (triggerState == HIGH) { 
                              Serial.println(F("Startup Homing: Recovery NEMA caused brief Switch 1 press (ISR), but not held. Attempt failed."));
                              triggerState = LOW; 
                          } else {
                              Serial.println(F("Startup Homing: Recovery NEMA FAILED to hit Switch 1 at all."));
                          }
                          startupHoming_attemptCounter++;
                          stepper_lastHomeTime = millis(); 
                      }
                  } else { // NEMA movement is NOT allowed
                      Serial.println(F("Startup Homing: Recovery NEMA move deferred by allowNemaMovement."));
                      stepper_lastHomeTime = millis(); 
                  }
              }
              
              // Existing logic for attempts >= 3 (Failure condition)
              // This check should only be evaluated if the above 'digitalRead(switchPin) == LOW' was false,
              // and after an attempt (if <3) has been made and potentially incremented the counter.
              if (startupHoming_attemptCounter >= 3) { 
                  if (!system_startupHomingFailed) { 
                      Serial.println(F("vacio")); 
                      system_startupHomingFailed = true;
                      digitalWrite(motorReductorPin, LOW); 
                      canServoBeActivated = false; // Ensure servo cannot activate if homing fails
                      digitalWrite(dosificadoraPin, LOW);
                      dosificadora_pending = false;
                      dosificadora_active = false;
                  }
                  systemIsInStartupHomingPhase = false; 
              }
          }
      }
  }
  // --- End Startup Homing Logic Block ---

  // --- Normal NEMA Operation from ISR (Path 1) ---
  // This block runs if not in startup homing, homing hasn't failed, the limit switch (triggerState) is HIGH, and motor is idle.
  if (!systemIsInStartupHomingPhase && !system_startupHomingFailed && triggerState == HIGH && !motorIsMoving) {
    // ISR detected a press (triggerState is HIGH). This path (Path 1) attempts to make a NEMA move.
    // canServoBeActivated is now set *after* the NEMA move based on switchPin state.

    if (allowNemaMovement) {
      // NEMA movement is allowed
      motorIsMoving = true;
      moverPasoCorregido(); // Perform the main stepper motor movement
      motorIsMoving = false;
      
      stepper_lastHomeTime = millis(); // Update last home/active time
      nemaOverrideAttemptCounter = 0;  // Reset override attempts as primary mechanism worked
      triggerState = LOW;             // Consume the trigger
      switch1_processed_by_ISR_this_cycle = true; // Mark this Switch 1 event as processed by ISR path
      canServoBeActivated = (digitalRead(switchPin) == LOW); // Set based on actual bottle presence post-move
      if(canServoBeActivated) {
          Serial.println(F("Normal Op (ISR Path): Switch 1 found LOW post-NEMA. Servo armed.")); // Debug
      } else {
          Serial.println(F("Normal Op (ISR Path): Switch 1 NOT LOW post-NEMA. Servo NOT armed.")); // Debug
      }
      lastSwitch1ActivityOrSearchTime = millis();
      idleBottleSearch_attemptCounter = 0;
      // The old inner if(digitalRead(switchPin)==LOW) that re-confirmed arming is now replaced by the line above.
    } else {
      // NEMA movement deferred. 
      // If NEMA movement is deferred, Switch 1 was pressed (triggerState was HIGH), 
      // so we can assume we want to activate servo once NEMA is allowed again.
      // However, the standard is now to check *after* a NEMA move.
      // For now, if deferred, we don't immediately know the future state of switchPin post-NEMA.
      // Let's assume if deferred, the intent from the ISR trigger still means canServoBeActivated should be true.
      // This is a bit of a dilemma: Path 1 implies an event occurred. If deferred, when should canServoBeActivated be set?
      // For consistency, it should be set after its NEMA move. But Path 2 sets it based on initial check.
      // Given the new rule "set after NEMA move", if Path 1 NEMA is deferred, canServoBeActivated shouldn't be set here.
      // It will be set by Path 2 if switchPin is still LOW when Path 2 eventually runs.
      // Therefore, removing speculative canServoBeActivated = true from here.
      // The original `canServoBeActivated = true;` at the beginning of the block is also removed.
      // If Path 1 NEMA move is deferred, it implies `canServoBeActivated` will be determined by a subsequent Path 2 execution.
      // If the trigger is consumed and NEMA doesn't move, servo shouldn't be armed based on this stale trigger.
      canServoBeActivated = false; // Explicitly false if NEMA move is deferred for this path.
      // Consume the trigger to prevent re-triggering Path 1 continuously while NEMA is disallowed.
      // Servo remains armed by canServoBeActivated = true from above.
      triggerState = LOW; 
      switch1_processed_by_ISR_this_cycle = true; // Indicate Switch 1 event was processed (even if NEMA didn't move)
      // Serial.println(F("NEMA movement deferred in Normal Op (Path 1) - trigger consumed."));
    }
    // (Servo Trigger Logic is GONE from here)
  }
  // --- End Normal NEMA Operation from ISR (Path 1) ---

  // --- Servo Activation Logic ---
  if (!systemIsInStartupHomingPhase && !system_startupHomingFailed) {
    // Check if servoSwitchPin (pin 4) is currently in the active state (LOW, debounced)
    // AND servo is not already at 90 AND servo is allowed to be activated.
    if (servoSwitch_currentState == LOW && !servo_isHoldingAt90 && canServoBeActivated) {
      // Only proceed if system is not already halted by max presses.
      if (!systemHalted_switch2Max) { 
        Serial.println(F("contador")); 
        switch2_pressCount++; // Count this activation
         if (switch2_pressCount > 0 && switch2_pressCount % 20 == 0) {
          if (!pin12_is_active) { // Only activate if not already in an 8-second HIGH period
            digitalWrite(pin12_output_pin, HIGH);
            pin12_is_active = true;
            pin12_start_time = millis();
            // Optional: Serial.println(F("Pin 12 ACTIVATED for 8 seconds."));
          }
        }

        if (switch2_pressCount >= current_switch2_maxPressCount) {
          Serial.println(F("ter"));
          systemHalted_switch2Max = true;
          digitalWrite(motorReductorPin, LOW); 
          digitalWrite(dosificadoraPin, LOW);
          dosificadora_pending = false;
          dosificadora_active = false;
          // canServoBeActivated will be set to false below, covering this halt.
        }
        
        myServo.write(90);        
        digitalWrite(motorVibradorPin, HIGH);
        servo_isHoldingAt90 = true; 
        servo_reached90Time = millis(); 
        servo_isActive = true;     
        dosificadora_triggerTime = millis(); // Capture time when servo is commanded to 90
        dosificadora_pending = true;
        dosificadora_active = false;        // Ensure it's reset if re-triggering
        digitalWrite(dosificadoraPin, LOW); // Ensure it's initially LOW if re-triggering
        allowNemaMovement = false;  
        canServoBeActivated = false; // Consume activation permission
        nemaQuietTimeJustElapsed = false;
      }
    }
  }
  // --- End Servo Activation Logic ---

  // --- Idle Bottle-Searching Logic ---
  if (!systemIsInStartupHomingPhase && !system_startupHomingFailed && !systemHalted_switch2Max &&
      !motorIsMoving && triggerState == LOW && allowNemaMovement) {
    
    if (idleBottleSearch_attemptCounter < 3 && (millis() - lastSwitch1ActivityOrSearchTime >= 5000)) {
      Serial.println(F("Idle Search: 5s elapsed. Attempting NEMA step.")); // Debug
      
      motorIsMoving = true;
      moverPasoCorregido();
      motorIsMoving = false;
      
      lastSwitch1ActivityOrSearchTime = millis(); // Reset timer after the step
      idleBottleSearch_attemptCounter++;

      if (digitalRead(switchPin) == LOW) { // Bottle found after idle search step
        canServoBeActivated = true;
        idleBottleSearch_attemptCounter = 0; // Reset search counter
        // lastSwitch1ActivityOrSearchTime will be updated again if servo activates or by next ISR event
        Serial.println(F("Idle Search: Bottle found and servo armed!")); // Debug
      } else { // Bottle NOT found after idle search step
        canServoBeActivated = false; // Ensure it's false if no bottle
        Serial.print(F("Idle Search: No bottle. Attempt #")); 
        Serial.println(idleBottleSearch_attemptCounter); // Debug
        
        if (idleBottleSearch_attemptCounter >= 3) {
          Serial.println(F("vacio")); // Send "vacio" after 3 failed search attempts
          system_startupHomingFailed = true; // Use existing flag for system halt
          digitalWrite(motorReductorPin, LOW);
          canServoBeActivated = false;
          digitalWrite(dosificadoraPin, LOW); 
          dosificadora_pending = false; 
          dosificadora_active = false;
          // Any other necessary halt actions (e.g., stopping other motors if they were added)
          // This effectively halts the system for this cycle type.
        }
      }
    }
  }
  // --- End Idle Bottle-Searching Logic ---

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

  // --- Dosificadora Logic ---
  // Check if 200ms pending time has elapsed to activate dosificadora
  if (dosificadora_pending && servo_isHoldingAt90 && (millis() - dosificadora_triggerTime >= 500)) {
    digitalWrite(dosificadoraPin, HIGH);
    dosificadora_active = true;
    dosificadora_startTime = millis(); // Record when it actually started
    dosificadora_pending = false;
    Serial.println(F("Dosificadora ACTIVATED (200ms delay passed)")); // Debug
  }

  // Check if 500ms active duration has elapsed to deactivate dosificadora
  if (dosificadora_active && (millis() - dosificadora_startTime >= 3000)) {
    digitalWrite(dosificadoraPin, LOW);
    dosificadora_active = false;
    Serial.println(F("Dosificadora DEACTIVATED (500ms duration passed)")); // Debug
  }
  // --- End Dosificadora Logic ---
  // --- Pin 12 Timed Deactivation Logic ---
  if (pin12_is_active && (millis() - pin12_start_time >= 8000)) {
    digitalWrite(pin12_output_pin, LOW);
    pin12_is_active = false;
    // Optional: Serial.println(F("Pin 12 DEACTIVATED after 8 seconds."));
  }
  // --- Servo Timed Return to 0 Logic ---
  // If servo is at 90 degrees, check if its hold duration has elapsed.
  if (servo_isHoldingAt90) {
    if ((millis() - servo_reached90Time) >= current_servo_Duration) { // If hold time is over
      myServo.write(0);        // Return servo to 0 degrees
      digitalWrite(motorVibradorPin, LOW);
      digitalWrite(dosificadoraPin, LOW);
      dosificadora_pending = false;
      dosificadora_active = false;
      servo_isHoldingAt90 = false; // Update servo state
      servo_isActive = false;      // Mark servo as inactive
      servo_finishTime = millis(); // Record time servo returned (informational)
      servoBecameInactiveTime = millis(); // Record time for NEMA quiet period logic
      lastSwitch1ActivityOrSearchTime = millis(); // Reset timer as a servo cycle just completed
      idleBottleSearch_attemptCounter = 0;
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
  
  pinMode(motorReductorPin, OUTPUT);
  digitalWrite(motorReductorPin, LOW);
  pinMode(motorVibradorPin, OUTPUT);
  digitalWrite(motorVibradorPin, LOW);
  pinMode(dosificadoraPin, OUTPUT);
  digitalWrite(dosificadoraPin, LOW);

  pinMode(pin12_output_pin, OUTPUT);
  digitalWrite(pin12_output_pin, LOW);
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
  digitalWrite(motorVibradorPin, HIGH);
  
  delay(5000);            // Hold for 5 seconds

  myServo.write(0);       // Move servo back to 0 degrees
  digitalWrite(motorVibradorPin, LOW);
  
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
                  digitalWrite(motorReductorPin, LOW);
                  Serial.println(F("System Paused (Command pausa)."));
                  myServo.write(0);
                  digitalWrite(motorVibradorPin, LOW);
                  digitalWrite(dosificadoraPin, LOW);
                  digitalWrite(pin12_output_pin, LOW);
                  pin12_is_active = false;
                  dosificadora_pending = false;
                  dosificadora_active = false;
                  // Ensure pin 8 specific old indicator logic is not present if motorReductorPin handles it
                  servo_isHoldingAt90 = false;
                  servo_isActive = false;
                  servoBecameInactiveTime = millis();
                  canServoBeActivated = false; // System paused, servo cannot be activated
              } else if (strcmp(serialBuffer, "despausa") == 0) {
                  system_pausa = false;
                  if (systemActive_afterCom1 || systemActive_afterCom2 || systemActive_afterCom3 || systemActive_afterCom4) {
                    digitalWrite(motorReductorPin, HIGH);
                  }
                  Serial.println(F("System Resumed (Command despausa)."));
              } else if (strcmp(serialBuffer, "CMD:1") == 0) {
                  if (!systemActive_afterCom1) {
                      Serial.println(F("System activated by command '1'.")); // Message can stay as is or be updated
                      systemActive_afterCom1 = true;
                      digitalWrite(motorReductorPin, HIGH); 
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(motorVibradorPin, LOW); // Ensure vibrator is OFF at CMD start
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      digitalWrite(pin12_output_pin, LOW);
                      pin12_is_active = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                      idleBottleSearch_attemptCounter = 0;
                      lastSwitch1ActivityOrSearchTime = millis();
                  }
                  systemActive_afterCom2 = false;
                  systemActive_afterCom3 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:2") == 0) {
                  if (!systemActive_afterCom2) {
                      Serial.println(F("System activated by command '2'.")); // Message can stay as is or be updated
                      systemActive_afterCom2 = true;
                      digitalWrite(motorReductorPin, HIGH); 
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(motorVibradorPin, LOW); // Ensure vibrator is OFF at CMD start
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      digitalWrite(pin12_output_pin, LOW);
                      pin12_is_active = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                      idleBottleSearch_attemptCounter = 0;
                      lastSwitch1ActivityOrSearchTime = millis();
                  }
                  systemActive_afterCom1 = false;
                  systemActive_afterCom3 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:3") == 0) {
                  if (!systemActive_afterCom3) {
                      Serial.println(F("System activated by command '3'.")); // Message can stay as is or be updated
                      systemActive_afterCom3 = true;
                      digitalWrite(motorReductorPin, HIGH); 
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(motorVibradorPin, LOW); // Ensure vibrator is OFF at CMD start
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      digitalWrite(pin12_output_pin, LOW);
                      pin12_is_active = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                      idleBottleSearch_attemptCounter = 0;
                      lastSwitch1ActivityOrSearchTime = millis();
                  }
                  systemActive_afterCom1 = false;
                  systemActive_afterCom2 = false;
                  systemActive_afterCom4 = false;
              } else if (strcmp(serialBuffer, "CMD:4") == 0) {
                  if (!systemActive_afterCom4) {
                      Serial.println(F("System activated by command '4'.")); // Message can stay as is or be updated
                      systemActive_afterCom4 = true;
                      digitalWrite(motorReductorPin, HIGH); 
                      systemIsInStartupHomingPhase = true;
                      system_startupHomingFailed = false;
                      startupHoming_attemptCounter = 0;
                      stepper_lastHomeTime = millis() - startupHoming_timeout;
                      triggerState = LOW;
                      motorIsMoving = false;
                      servo_isActive = false;
                      servo_isHoldingAt90 = false;
                      myServo.write(0);
                      digitalWrite(motorVibradorPin, LOW); // Ensure vibrator is OFF at CMD start
                      servoBecameInactiveTime = millis();
                      switch2_pressCount = 0;
                      systemHalted_switch2Max = false;
                      canServoBeActivated = false;
                      digitalWrite(pin12_output_pin, LOW);
                      pin12_is_active = false;
                      nemaOverrideAttemptCounter = 0;
                      waiting_for_step_after_homing_press = false;
                      switch1_pressed_during_homing_time = 0;
                      idleBottleSearch_attemptCounter = 0;
                      lastSwitch1ActivityOrSearchTime = millis();
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
