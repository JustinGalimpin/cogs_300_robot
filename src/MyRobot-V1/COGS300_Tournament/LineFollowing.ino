// IR reading flags are defined in Sensors.ino, which Arduino concatenates later.
extern bool leftOnLine;
extern bool centerOnLine;
extern bool rightOnLine;

// Original Lab 7 line-following and recovery logic.
// Only conflicting identifiers have been renamed.

// Normal forward speed.
const int lineBaseSpeed = 100;

// Strong steering correction.
const int lineTurnSpeed = 105;
const int lineSlowSpeed = 40;

// Gentle steering correction.
const int lineSlightFastSpeed = 90;
const int lineSlightSlowSpeed = 60;

// Rotation speed during line recovery.
const int lineSearchSpeed = 65;

// ==========================================
// LINE RECOVERY CONFIGURATION
// ==========================================

// -1 = Last detected on left
//  0 = No known side
//  1 = Last detected on right
int lastDirection = 0;

// Track whether the line is currently lost.
bool lineLost = false;

// ==========================================
// LINE RECOVERY
// ==========================================

/*
 * If no IR sensor sees white tape, immediately rotate
 * toward the last known side of the tape.
 * This avoids overshooting sharp corners.
 */
void recoverLine() {
  if (!lineLost) {
    lineLost = true;
    Serial.println("Line lost! Searching...");
  }

  if (lastDirection == -1) {
    setMotors(-lineSearchSpeed, lineSearchSpeed);
  }
  else if (lastDirection == 1) {
    setMotors(lineSearchSpeed, -lineSearchSpeed);
  }
  else {
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
  Serial.print(lastDirection);

  Serial.print(" | Line lost: ");
  Serial.println(lineLost);
}

// ==========================================
