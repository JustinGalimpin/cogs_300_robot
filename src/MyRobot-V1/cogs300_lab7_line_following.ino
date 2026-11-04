
/*
 * COGS 300 - Lab 07
 * Three-Sensor Autonomous Line Following
 *
 * Features:
 * - Three IR sensors (left, center, right).
 * - HIGH indicates white tape.
 * - Straight movement when centered.
 * - Gradual left/right steering.
 * - Stronger corrections when off-center.
 * - Line recovery when all sensors lose tape.
 *
 * Motor configuration preserved from Lab 5.
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

const int leftIRPin = A0;
const int centerIRPin = A1;
const int rightIRPin = A2;

// Confirmed calibration:
// HIGH = White tape
// LOW  = Dark floor
const int WHITE_DETECTED = HIGH;

bool leftOnLine = false;
bool centerOnLine = false;
bool rightOnLine = false;

// ==========================================
// MOTOR SPEED CONFIGURATION
// ==========================================

// Normal forward speed.
const int baseSpeed = 110;

// Strong steering correction.
const int turnSpeed = 125;
const int slowSpeed = 65;

// Gentle steering correction.
const int slightFastSpeed = 115;
const int slightSlowSpeed = 85;

// Rotation speed during line recovery.
const int searchSpeed = 85;

// ==========================================
// LINE RECOVERY CONFIGURATION
// ==========================================

// -1 = Last detected on left
//  0 = No known side
//  1 = Last detected on right
int lastDirection = 0;

// Record when the robot first loses the tape.
unsigned long lineLostTime = 0;

// Continue forward briefly before searching.
const unsigned long lineLostDelay = 100;

// Track whether the line is currently lost.
bool lineLost = false;

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

  // Ensure robot starts stationary.
  setMotors(0, 0);

  Serial.println("Lab 7 line-following ready!");
  Serial.println("HIGH = White tape.");
}

// ==========================================
// MOTOR CONTROL
// ==========================================

/*
 * Positive speed = forward
 * Negative speed = backward
 * Zero = stop
 *
 * Motor directions preserved from Lab 5.
 */

void setMotors(int leftSpeed, int rightSpeed) {

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT MOTOR
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

  // RIGHT MOTOR
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

  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}

// ==========================================
// IR SENSOR READINGS
// ==========================================

void readIRSensors() {

  leftOnLine =
    digitalRead(leftIRPin) == WHITE_DETECTED;

  centerOnLine =
    digitalRead(centerIRPin) == WHITE_DETECTED;

  rightOnLine =
    digitalRead(rightIRPin) == WHITE_DETECTED;
}

// ==========================================
// LINE RECOVERY
// ==========================================

void recoverLine() {

  // Record when the tape first disappears.
  if (!lineLost) {
    lineLost = true;
    lineLostTime = millis();
  }

  // Continue forward briefly to avoid
  // reacting to momentary sensor loss.
  if (millis() - lineLostTime < lineLostDelay) {

    setMotors(baseSpeed, baseSpeed);
    return;
  }

  // Search toward the last known tape location.
  if (lastDirection == -1) {

    // Rotate left.
    setMotors(-searchSpeed, searchSpeed);
  }
  else if (lastDirection == 1) {

    // Rotate right.
    setMotors(searchSpeed, -searchSpeed);
  }
  else {

    // No known direction: stop safely.
    setMotors(0, 0);
  }
}

// ==========================================
// LINE-FOLLOWING CONTROLLER
// ==========================================

void updateLineFollowing() {

  // Read all three sensors.
  readIRSensors();

  // CASE 1: No sensors detect white.
  if (!leftOnLine &&
      !centerOnLine &&
      !rightOnLine) {

    recoverLine();
    return;
  }

  // Tape detected: reset recovery tracking.
  lineLost = false;

  // CASE 2: Center sensor detects white.
  if (!leftOnLine &&
      centerOnLine &&
      !rightOnLine) {

    setMotors(baseSpeed, baseSpeed);

    // Do not erase the last known side.
    // This helps with future recovery.
  }

  // CASE 3: Left sensor detects white.
  else if (leftOnLine &&
           !centerOnLine &&
           !rightOnLine) {

    setMotors(slowSpeed, turnSpeed);
    lastDirection = -1;
  }

  // CASE 4: Right sensor detects white.
  else if (!leftOnLine &&
           !centerOnLine &&
           rightOnLine) {

    setMotors(turnSpeed, slowSpeed);
    lastDirection = 1;
  }

  // CASE 5: Left and center detect white.
  else if (leftOnLine &&
           centerOnLine &&
           !rightOnLine) {

    setMotors(slightSlowSpeed, slightFastSpeed);
    lastDirection = -1;
  }

  // CASE 6: Center and right detect white.
  else if (!leftOnLine &&
           centerOnLine &&
           rightOnLine) {

    setMotors(slightFastSpeed, slightSlowSpeed);
    lastDirection = 1;
  }

  // CASE 7: All three sensors detect white.
  else if (leftOnLine &&
           centerOnLine &&
           rightOnLine) {

    setMotors(baseSpeed, baseSpeed);
  }

  // CASE 8: Left and right detect white,
  // but center does not.
  else {

    // Ambiguous pattern: stop.
    setMotors(0, 0);
  }
}

// ==========================================
// SERIAL MONITOR OUTPUT
// ==========================================

void printSensorReadings() {

  Serial.print("IR [L C R]: ");

  Serial.print(leftOnLine);
  Serial.print(" ");

  Serial.print(centerOnLine);
  Serial.print(" ");

  Serial.print(rightOnLine);

  Serial.print(" | Last direction: ");
  Serial.println(lastDirection);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  // Update line-following continuously.
  updateLineFollowing();

  // Print sensor readings every 200 ms.
  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime >= 200) {

    printSensorReadings();
    lastPrintTime = millis();
  }

  // Short delay for stable sensor polling.
  delay(10);
}
