#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

class StepperMotor {
private:
  TMC2209Stepper driver;
  AccelStepper stepper;
  uint8_t enPin;
  uint8_t limitSwitchPin;
  long stepsPerRev;
  float maxSpeed, maxAccel;
  bool movementComplete;
  bool limitSwitchEnabled;
  bool lastLimitState;

public:
  struct Config {
    HardwareSerial &serialPort;
    uint8_t rxPin;
    uint8_t txPin;
    uint8_t enPin;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitSwitchPin;  // 0 = disabled
    float rSense;
    uint8_t serialAddress;
    uint16_t motorCurrentRMS;
    uint16_t microsteps;
  };

  StepperMotor(const Config &cfg)
      : driver(&cfg.serialPort, cfg.rSense, cfg.serialAddress),
        stepper(AccelStepper::DRIVER, cfg.stepPin, cfg.dirPin),
        enPin(cfg.enPin),
        limitSwitchPin(cfg.limitSwitchPin),
        movementComplete(true),
        limitSwitchEnabled(cfg.limitSwitchPin != 0),
        lastLimitState(HIGH) {
    pinMode(cfg.enPin, OUTPUT);
    digitalWrite(cfg.enPin, LOW);

    // Setup limit switch if enabled
    if (limitSwitchEnabled) {
      pinMode(cfg.limitSwitchPin, INPUT_PULLUP);
      Serial.print("Limit switch enabled on pin ");
      Serial.println(cfg.limitSwitchPin);
    }

    cfg.serialPort.begin(115200, SERIAL_8N1, cfg.rxPin, cfg.txPin);
    driver.begin();
    driver.rms_current(cfg.motorCurrentRMS);
    driver.microsteps(cfg.microsteps);
    driver.toff(5);
    driver.pdn_disable(true);

    stepsPerRev = 200 * (cfg.microsteps > 0 ? cfg.microsteps : 1);
    maxSpeed = stepsPerRev * 5;
    maxAccel = maxSpeed / 2;

    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(maxAccel);

    Serial.print("Motor setup complete on EN pin ");
    Serial.println(enPin);
  }

  // ✅ Non-blocking move to target
  void moveTo(long target) {
    stepper.moveTo(target);
    movementComplete = false;
    Serial.print("Moving to target: ");
    Serial.println(target);
  }

  // ✅ Non-blocking relative move
  void move(long steps) {
    stepper.move(steps);
    movementComplete = false;
    Serial.print("Moving relative: ");
    Serial.print(steps);
    Serial.println(" steps");
  }

  // ✅ MUST be called frequently in loop()
  void update() {
    // Check limit switch
    if (limitSwitchEnabled) {
      bool currentState = digitalRead(limitSwitchPin);
      
      // Detect switch pressed (LOW = pressed with INPUT_PULLUP)
      if (currentState == LOW && lastLimitState == HIGH) {
        if (isRunning()) {
          emergencyStop();
          Serial.println("⚠️  LIMIT SWITCH TRIGGERED - EMERGENCY STOP!");
          displayPosition();
        }
      }
      lastLimitState = currentState;
    }

    // Normal motion control
    if (stepper.distanceToGo() != 0) {
      stepper.run();
    } else {
      // Movement just completed
      if (!movementComplete) {
        movementComplete = true;
        Serial.println("Target reached!");
        displayPosition();  // Auto-display when target reached
      }
    }
  }

  // ✅ Check if motor is currently moving
  bool isRunning() {
    return stepper.distanceToGo() != 0;
  }

  // ✅ Check limit switch state
  bool isLimitSwitchPressed() {
    if (!limitSwitchEnabled) return false;
    return digitalRead(limitSwitchPin) == LOW;  // LOW = pressed with pullup
  }

  // ✅ Get limit switch status
  void printLimitSwitchStatus() {
    if (!limitSwitchEnabled) {
      Serial.println("Limit switch: DISABLED");
    } else {
      Serial.print("Limit switch: ");
      Serial.println(isLimitSwitchPressed() ? "PRESSED ⚠️" : "NOT PRESSED ✓");
    }
  }

  // ✅ Stop immediately
  void stop() {
    stepper.stop();
    Serial.println("Motor stopped!");
  }

  // ✅ Emergency stop (no deceleration)
  void emergencyStop() {
    stepper.setCurrentPosition(stepper.currentPosition());
    Serial.println("EMERGENCY STOP!");
  }

  // ✅ Get current position
  long getCurrentPosition() {
    return stepper.currentPosition();
  }

  // ✅ Display current position with formatting
  void displayPosition() {
    long pos = stepper.currentPosition();
    Serial.print("┌─────────────────────────────┐\n");
    Serial.print("│ Current Position: ");
    Serial.print(pos);
    Serial.println(" steps  │");
    Serial.print("│ Revolutions: ");
    Serial.print((float)pos / stepsPerRev, 2);
    Serial.println(" rev    │");
    Serial.print("│ Status: ");
    Serial.print(isRunning() ? "MOVING" : "IDLE");
    Serial.println("          │");
    if (limitSwitchEnabled) {
      Serial.print("│ Limit Switch: ");
      Serial.print(isLimitSwitchPressed() ? "PRESSED ⚠️" : "OK ✓");
      Serial.println("     │");
    }
    Serial.println("└─────────────────────────────┘");
  }

  // ✅ Set home position
  void setHome() {
    stepper.setCurrentPosition(0);
    Serial.println("Home position set");
  }

  // ✅ Enable/disable motor
  void enable() {
    digitalWrite(enPin, LOW);
    Serial.println("Motor enabled");
  }

  void disable() {
    digitalWrite(enPin, HIGH);
    Serial.println("Motor disabled");
  }

  // ✅ Set speed parameters
  void setSpeed(float speed) {
    stepper.setMaxSpeed(speed);
  }

  void setAcceleration(float accel) {
    stepper.setAcceleration(accel);
  }
};

// ====================================================
//               EASY CONFIG SETUP
// ====================================================
StepperMotor::Config M1 = {
  Serial1,   // UART port
  17, 18,    // RX, TX
  21,        // Enable pin
  16, 15,    // Step, Dir
  20,        // Limit switch pin (set to 0 to disable)
  0.11f,     // Rsense
  0,         // UART address
  800,       // Motor current (mA)
  0          // Microsteps (full step)
};

StepperMotor motorX(M1);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println("\n=== Non-Blocking Stepper Controller ===");
  Serial.println("Commands:");
  Serial.println("  number    - Move to absolute position (e.g., 1000)");
  Serial.println("  +number   - Move relative forward (e.g., +500)");
  Serial.println("  -number   - Move relative backward (e.g., -500)");
  Serial.println("  s         - Stop with deceleration");
  Serial.println("  e         - Emergency stop");
  Serial.println("  h         - Set current position as home (0)");
  Serial.println("  p         - Print current position");
  Serial.println("  d         - Display detailed position info");
  Serial.println("  l         - Check limit switch status");
  Serial.println("  on        - Enable motor");
  Serial.println("  off       - Disable motor");
  Serial.println("=======================================\n");
  
  // Display initial position
  motorX.displayPosition();
}

void loop() {
  // ✅ CRITICAL: Update motor continuously
  motorX.update();

  // ✅ Handle serial commands (non-blocking)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      // Command parsing
      if (input.equalsIgnoreCase("s")) {
        motorX.stop();
      }
      else if (input.equalsIgnoreCase("e")) {
        motorX.emergencyStop();
      }
      else if (input.equalsIgnoreCase("h")) {
        motorX.setHome();
      }
      else if (input.equalsIgnoreCase("p")) {
        Serial.print("Current position: ");
        Serial.println(motorX.getCurrentPosition());
      }
      else if (input.equalsIgnoreCase("d")) {
        motorX.displayPosition();
      }
      else if (input.equalsIgnoreCase("l")) {
        motorX.printLimitSwitchStatus();
      }
      else if (input.equalsIgnoreCase("on")) {
        motorX.enable();
      }
      else if (input.equalsIgnoreCase("off")) {
        motorX.disable();
      }
      else if (input.startsWith("+")) {
        // Relative move forward
        long steps = input.substring(1).toInt();
        if (steps != 0) {
          motorX.move(steps);
        }
      }
      else if (input.startsWith("-")) {
        // Relative move backward
        long steps = input.toInt();
        if (steps != 0) {
          motorX.move(steps);
        }
      }
      else {
        // Absolute move
        long target = input.toInt();
        // Allow moving to position 0
        motorX.moveTo(target);
      }
    }
  }

  // ✅ You can add other tasks here without blocking!
  // Example: Blink LED, read sensors, handle other motors, etc.
}