// Original Lab 7 line-following and recovery logic.
// Only conflicting identifiers have been renamed.

// Normal forward speed.
const int lineBaseSpeed = 110;

// Strong steering correction.
const int lineTurnSpeed = 125;
const int lineSlowSpeed = 65;

// Gentle steering correction.
const int lineSlightFastSpeed = 115;
const int lineSlightSlowSpeed = 85;

// Rotation speed during line recovery.
const int lineSearchSpeed = 85;

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

void recoverLine() {

  // Record when the tape first disappears.
  if (!lineLost) {
    lineLost = true;
    lineLostTime = millis();
  }

  // Continue forward briefly to avoid
  // reacting to momentary sensor loss.
  if (millis() - lineLostTime < lineLostDelay) {

    setMotors(lineBaseSpeed, lineBaseSpeed);
    return;
  }

  // Search toward the last known tape location.
  if (lastDirection == -1) {

    // Rotate left.
    setMotors(-lineSearchSpeed, lineSearchSpeed);
  }
  else if (lastDirection == 1) {

    // Rotate right.
    setMotors(lineSearchSpeed, -lineSearchSpeed);
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

    setMotors(lineBaseSpeed, lineBaseSpeed);

    // Do not erase the last known side.
    // This helps with future recovery.
  }

  // CASE 3: Left sensor detects white.
  else if (leftOnLine &&
           !centerOnLine &&
           !rightOnLine) {

    setMotors(lineSlowSpeed, lineTurnSpeed);
    lastDirection = -1;
  }

  // CASE 4: Right sensor detects white.
  else if (!leftOnLine &&
           !centerOnLine &&
           rightOnLine) {

    setMotors(lineTurnSpeed, lineSlowSpeed);
    lastDirection = 1;
  }

  // CASE 5: Left and center detect white.
  else if (leftOnLine &&
           centerOnLine &&
           !rightOnLine) {

    setMotors(lineSlightSlowSpeed, lineSlightFastSpeed);
    lastDirection = -1;
  }

  // CASE 6: Center and right detect white.
  else if (!leftOnLine &&
           centerOnLine &&
           rightOnLine) {

    setMotors(lineSlightFastSpeed, lineSlightSlowSpeed);
    lastDirection = 1;
  }

  // CASE 7: All three sensors detect white.
  else if (leftOnLine &&
           centerOnLine &&
           rightOnLine) {

    setMotors(lineBaseSpeed, lineBaseSpeed);
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

void printLineSensorReadings() {

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
