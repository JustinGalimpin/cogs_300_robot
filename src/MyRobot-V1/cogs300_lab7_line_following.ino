
/*
 * COGS 300 - Lab 07
 * Three IR Sensor Line Detection
 *
 * Current implementation: Step 2
 *
 * Features:
 * - Three IR sensors (left, center, right).
 * - Robot moves forward while detecting white tape.
 * - Robot stops when all sensors lose the tape.
 * - Serial Monitor debugging.
 *
 * Uses the original Lab 5 motor configuration.
 */

// ==========================================
// MOTOR PIN CONFIGURATION
// ==========================================

// Motor A (Left Wheel)
const int enA = 5;
const int in1 = 6;
const int in2 = 7;

// Motor B (Right Wheel)
const int enB = 9;
const int in3 = 10;
const int in4 = 11;

// ==========================================
// IR SENSOR CONFIGURATION
// ==========================================

// IR sensors use available analog pins
// configured as digital inputs.
const int leftIRPin = A0;
const int centerIRPin = A1;
const int rightIRPin = A2;

// Sensor output corresponding to white tape.
// Change to HIGH if calibration shows that
// your sensors return HIGH on white.
const int WHITE_DETECTED = LOW;

// Store whether each sensor detects white.
bool leftOnLine = false;
bool centerOnLine = false;
bool rightOnLine = false;

// ==========================================
// MOVEMENT CONFIGURATION
// ==========================================

// Initial forward motor speed (PWM 0-255).
// Lower than Lab 5 for controlled testing.
const int baseSpeed = 110;

// Time between sensor checks (milliseconds).
const unsigned long SENSOR_INTERVAL = 50;

// Track the previous sensor update.
unsigned long lastSensorTime = 0;

// ==========================================
// SETUP
// ==========================================

void setup() {

  // Configure motor pins.
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Configure IR sensors.
  pinMode(leftIRPin, INPUT);
  pinMode(centerIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  // Initialize Serial Monitor.
  Serial.begin(9600);

  // Ensure the robot starts stationary.
  setMotors(0, 0);

  Serial.println("Lab 7 robot ready!");
  Serial.println("Step 2: Start/Stop on white tape.");
}

// ==========================================
// MOTOR CONTROL
// ==========================================

/*
 * Controls both motors independently.
 *
 * Positive speed = forward
 * Negative speed = backward
 * Zero = stop
 *
 * Motor directions match our Lab 5 robot.
 */

void setMotors(int leftSpeed, int rightSpeed) {

  // Restrict speeds to valid PWM values.
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // ------------------------------------------
  // LEFT MOTOR
  // ------------------------------------------

  if (leftSpeed > 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (leftSpeed < 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // ------------------------------------------
  // RIGHT MOTOR
  // ------------------------------------------

  if (rightSpeed > 0) {

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (rightSpeed < 0) {

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Apply PWM speeds.
  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}

// ==========================================
// IR SENSOR READINGS
// ==========================================

/*
 * Reads the three IR sensors.
 *
 * true  = White tape detected
 * false = White tape not detected
 */

void readIRSensors() {

  leftOnLine =
    digitalRead(leftIRPin) == WHITE_DETECTED;

  centerOnLine =
    digitalRead(centerIRPin) == WHITE_DETECTED;

  rightOnLine =
    digitalRead(rightIRPin) == WHITE_DETECTED;
}

// ==========================================
// START/STOP CONTROLLER
// ==========================================

/*
 * Step 2:
 *
 * If any IR sensor detects white tape,
 * move forward.
 *
 * If no IR sensors detect white tape,
 * stop both motors.
 */

void updateLineDetection() {

  // Check whether at least one sensor
  // currently detects white tape.
  bool lineDetected =
    leftOnLine ||
    centerOnLine ||
    rightOnLine;

  if (lineDetected) {

    // Move both wheels forward.
    setMotors(baseSpeed, baseSpeed);
  }
  else {

    // Stop when the tape is no longer detected.
    setMotors(0, 0);
  }
}

// ==========================================
// SERIAL MONITOR OUTPUT
// ==========================================

void printSensorReadings() {

  Serial.print("IR Sensors [L C R]: ");

  Serial.print(leftOnLine);
  Serial.print(" ");

  Serial.print(centerOnLine);
  Serial.print(" ");

  Serial.print(rightOnLine);

  // Display the current movement decision.
  if (leftOnLine || centerOnLine || rightOnLine) {

    Serial.println(" | MOVING FORWARD");
  }
  else {

    Serial.println(" | STOPPED");
  }
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  // Update sensors approximately every 50 ms.
  if (millis() - lastSensorTime >= SENSOR_INTERVAL) {

    // Read all three IR sensors.
    readIRSensors();

    // Determine whether to move or stop.
    updateLineDetection();

    // Display sensor readings.
    printSensorReadings();

    // Record when the update finished.
    lastSensorTime = millis();
  }
}
