
/*
 * COGS 300 - Lab 08
 * ObjectDetection.ino
 *
 * Autonomous Object Detection and Navigation
 *
 * Features:
 * - Servo-based ultrasonic scanning
 * - No Servo.h library required
 * - Scan from 30 to 150 degrees
 * - Measurements every 15 degrees
 * - Group readings into candidate objects
 * - Prefer the largest estimated object
 * - Rotate chassis toward selected target
 * - Verify target before approaching
 * - Drive toward selected target
 * - Avoid white-tape arena boundaries
 *
 * Hardware:
 * - Arduino Uno
 * - Front ultrasonic sensor on servo (A5)
 * - Three IR sensors (A0, A1, A2)
 * - Two DC motors through motor driver
 *
 * Requires:
 * - Motors.ino
 * - Sensors.ino
 *
 * Servo movement is only activated during
 * the object-detection stage.
 */

// ==========================================
// SHARED SENSOR CONFIGURATION
// ==========================================

/*
 * These constants are defined in Sensors.ino.
 *
 * extern allows ObjectDetection.ino to
 * reference them before their definitions
 * appear in Arduino's compilation order.
 *
 * Their definitions in Sensors.ino must
 * have external linkage.
 */

extern const int servoScanStep;
extern const int servoMinAngle;
extern const int servoMaxAngle;
extern const int servoCenterAngle;

extern const int frontTrigPin;
extern const int frontEchoPin;

// ==========================================
// OBJECT DETECTION STATES
// ==========================================

enum ObjectState {
  SCANNING,
  TURN_TO_OBJECT,
  VERIFY_TARGET,
  APPROACHING_OBJECT,
  AVOIDING_BOUNDARY,
  SEARCH_ROTATION,
  OBJECT_REACHED
};

ObjectState objectState = SCANNING;

// ==========================================
// MOTOR CONFIGURATION
// ==========================================

// Reduced speeds for initial testing.
const int objectTurnSpeed = 70;
const int objectApproachSpeed = 75;
const int objectReverseSpeed = 75;

// ==========================================
// DISTANCE CONFIGURATION
// ==========================================

// Minimum valid object distance.
const float objectMinDistance = 4.0;

// Maximum distance considered for detection.
const float objectMaxDistance = 100.0;

// Stop when an object is this close.
const float objectStopDistance = 12.0;

// Maximum difference between consecutive
// readings grouped into the same object.
const float objectGroupingTolerance = 18.0;

// Allowed difference between the original
// target distance and verification distance.
const float objectVerifyTolerance = 25.0;

// ==========================================
// SCAN CONFIGURATION
// ==========================================

// Scan from 30 to 150 degrees,
// taking measurements every 15 degrees.
//
// Total measurement positions:
// 30, 45, 60, 75, 90, 105, 120, 135, 150

const int objectScanCount = 9;

// Time allowed for servo positioning.
const unsigned long objectServoSettleTime = 300;

// ==========================================
// ROTATION CONFIGURATION
// ==========================================

/*
 * Approximate time for one complete
 * chassis rotation.
 *
 * These values MUST be calibrated using
 * the physical robot.
 *
 * They are starting estimates only.
 */

const unsigned long objectClockwise360Time = 3000;
const unsigned long objectCounterClockwise360Time = 3000;

// Rotation duration when searching
// for objects outside the current sweep.
const unsigned long objectSearchTurnTime = 750;

// ==========================================
// BOUNDARY AVOIDANCE CONFIGURATION
// ==========================================

// Time spent reversing from white tape.
const unsigned long objectReverseTime = 450;

// Time spent turning away from boundary.
const unsigned long objectBoundaryTurnTime = 550;

// ==========================================
// APPROACH CONFIGURATION
// ==========================================

// Time between distance measurements.
const unsigned long objectApproachInterval = 100;

// Maximum time spent approaching a target.
const unsigned long objectMaxApproachTime = 5000;

// Number of consecutive invalid readings
// allowed before abandoning the target.
const int objectMaxInvalidReadings = 3;

// ==========================================
// SCAN STORAGE
// ==========================================

// Stores distance measurements from
// each servo scanning position.
float objectScanDistances[objectScanCount];

// Current measurement position.
int objectScanIndex = 0;

// ==========================================
// SELECTED TARGET INFORMATION
// ==========================================

bool objectFound = false;

float objectTargetDistance = -1;
float objectTargetWidth = 0;

int objectTargetAngle = 90;

// ==========================================
// STATE VARIABLES
// ==========================================

unsigned long objectStateStartTime = 0;
unsigned long objectLastApproachTime = 0;
unsigned long objectApproachStartTime = 0;

int objectInvalidReadings = 0;

// Chassis rotation direction.
//
// -1 = left
//  1 = right
int objectTurnDirection = 1;

// Duration of the current turn.
unsigned long objectTurnDuration = 0;

// Boundary recovery phase.
//
// 0 = reverse
// 1 = turn away
int objectBoundaryPhase = 0;

// ==========================================
// HELPER: VALID DISTANCE
// ==========================================

bool isValidObjectDistance(float distance) {

  return distance >= objectMinDistance &&
         distance <= objectMaxDistance;
}

// ==========================================
// HELPER: BOUNDARY DETECTION
// ==========================================

/*
 * The three IR sensors detect white tape.
 *
 * HIGH = dark floor
 * LOW  = white tape
 *
 * readIRSensors() updates:
 * - leftOnLine
 * - centerOnLine
 * - rightOnLine
 */

bool objectBoundaryDetected() {

  readIRSensors();

  return leftOnLine ||
         centerOnLine ||
         rightOnLine;
}

// ==========================================
// START NEW OBJECT SCAN
// ==========================================

void startObjectScan() {

  // Stop chassis movement before scanning.
  setMotors(0, 0);

  objectScanIndex = 0;

  objectFound = false;

  objectTargetDistance = -1;
  objectTargetWidth = 0;
  objectTargetAngle = 90;

  objectInvalidReadings = 0;

  // Clear previous scan measurements.
  for (int i = 0; i < objectScanCount; i++) {

    objectScanDistances[i] = -1;
  }

  objectState = SCANNING;
  objectStateStartTime = millis();

  Serial.println("-------------------------");
  Serial.println("Starting servo object scan...");
}

// ==========================================
// INITIALIZE OBJECT DETECTION
// ==========================================

void initializeObjectDetection() {

  // Ensure robot begins stationary.
  setMotors(0, 0);

  Serial.println("=========================");
  Serial.println("OBJECT DETECTION INITIALIZED");
  Serial.println("Servo scanning enabled");
  Serial.println("=========================");

  startObjectScan();
}

// ==========================================
// ANALYZE OBJECT SCAN
// ==========================================

/*
 * Groups consecutive ultrasonic readings
 * that have similar distances.
 *
 * Each group represents a possible object.
 *
 * We estimate its physical width using:
 *
 * width = 2 * distance * tan(angle / 2)
 *
 * The candidate with the greatest
 * estimated width is selected.
 *
 * IMPORTANT:
 * This is an approximation because the
 * HC-SR04 has a relatively wide beam.
 */

void analyzeObjectScan() {

  int index = 0;

  while (index < objectScanCount) {

    // Skip invalid measurements.
    if (!isValidObjectDistance(
          objectScanDistances[index])) {

      index++;
      continue;
    }

    int groupStart = index;
    int groupEnd = index;

    float distanceSum =
      objectScanDistances[index];

    int sampleCount = 1;

    // Group consecutive readings
    // that have similar distances.
    while (groupEnd + 1 < objectScanCount) {

      float previousDistance =
        objectScanDistances[groupEnd];

      float nextDistance =
        objectScanDistances[groupEnd + 1];

      if (!isValidObjectDistance(nextDistance)) {
        break;
      }

      if (abs(nextDistance - previousDistance) >
          objectGroupingTolerance) {
        break;
      }

      groupEnd++;

      distanceSum += nextDistance;
      sampleCount++;
    }

    // Require at least two readings
    // to form an object candidate.
    if (sampleCount >= 2) {

      float averageDistance =
        distanceSum / sampleCount;

      // Estimate angular coverage.
      float angularWidth =
        (groupEnd - groupStart + 1) *
        servoScanStep;

      // Convert half-angle into radians.
      float halfAngleRadians =
        (angularWidth / 2.0) *
        PI / 180.0;

      // Approximate physical width.
      float estimatedWidth =
        2.0 * averageDistance *
        tan(halfAngleRadians);

      // Determine the group's center.
      int centerIndex =
        (groupStart + groupEnd) / 2;

      int centerAngle =
        servoMinAngle +
        centerIndex * servoScanStep;

      Serial.print("Candidate at ");
      Serial.print(centerAngle);

      Serial.print(" degrees | Distance: ");
      Serial.print(averageDistance);

      Serial.print(" cm | Width estimate: ");
      Serial.print(estimatedWidth);

      Serial.println(" cm");

      // Select the largest candidate.
      if (!objectFound ||
          estimatedWidth > objectTargetWidth) {

        objectFound = true;

        objectTargetWidth = estimatedWidth;
        objectTargetDistance = averageDistance;
        objectTargetAngle = centerAngle;
      }
    }

    index = groupEnd + 1;
  }
}

// ==========================================
// BEGIN CHASSIS ROTATION
// ==========================================

void beginObjectTurn(
  int direction,
  unsigned long duration
) {

  setMotors(0, 0);

  objectTurnDirection = direction;
  objectTurnDuration = duration;

  objectStateStartTime = millis();
  objectState = TURN_TO_OBJECT;
}

// ==========================================
// START BOUNDARY AVOIDANCE
// ==========================================

/*
 * Called whenever white tape is detected.
 *
 * The robot:
 * 1. Stops.
 * 2. Reverses.
 * 3. Turns away from the boundary.
 * 4. Starts scanning again.
 */

void startBoundaryAvoidance() {

  setMotors(0, 0);

  // Update IR readings.
  readIRSensors();

  // White tape on the left:
  // turn toward the right.
  if (leftOnLine && !rightOnLine) {

    objectTurnDirection = 1;
  }

  // White tape on the right:
  // turn toward the left.
  else if (rightOnLine && !leftOnLine) {

    objectTurnDirection = -1;
  }

  // Center or multiple sensors:
  // use a default right turn.
  else {

    objectTurnDirection = 1;
  }

  objectBoundaryPhase = 0;

  objectStateStartTime = millis();
  objectState = AVOIDING_BOUNDARY;

  Serial.println("Boundary detected!");
  Serial.println("Beginning recovery...");
}

// ==========================================
// SCAN FOR OBJECTS
// ==========================================

void scanForObjects() {

  // Robot must remain stationary
  // while the servo is scanning.
  setMotors(0, 0);

  // Check for white tape.
  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  // ======================================
  // SCAN COMPLETE
  // ======================================

  if (objectScanIndex >= objectScanCount) {

    analyzeObjectScan();

    // No valid candidate detected.
    if (!objectFound) {

      Serial.println("No suitable target.");
      Serial.println("Rotating chassis to search...");

      // Center sensor before rotating.
      centerFrontServo();

      objectState = SEARCH_ROTATION;
      objectStateStartTime = millis();

      return;
    }

    // Target candidate found.
    Serial.println("Largest candidate selected.");

    Serial.print("Target angle: ");
    Serial.println(objectTargetAngle);

    Serial.print("Estimated width: ");
    Serial.println(objectTargetWidth);

    // Center the sensor before
    // rotating the chassis.
    centerFrontServo();

    // Calculate difference between
    // target direction and straight ahead.
    int angleDifference =
      objectTargetAngle - servoCenterAngle;

    /*
     * Servo mounting assumption:
     *
     * Greater than 90 degrees = left
     * Less than 90 degrees = right
     *
     * Reverse this mapping if needed.
     */

    int direction =
      angleDifference > 0 ? -1 : 1;

    unsigned long fullTurnTime =
      direction == -1
        ? objectCounterClockwise360Time
        : objectClockwise360Time;

    // Estimate required turning duration.
    unsigned long turnDuration =
      (unsigned long)(
        abs(angleDifference) / 360.0 *
        fullTurnTime
      );

    // Target already near center.
    if (abs(angleDifference) <= 7) {

      objectState = VERIFY_TARGET;
      objectStateStartTime = millis();

      return;
    }

    // Rotate chassis toward target.
    beginObjectTurn(direction, turnDuration);

    return;
  }

  // ======================================
  // MOVE SERVO TO SCANNING POSITION
  // ======================================

  int scanAngle =
    servoMinAngle +
    objectScanIndex * servoScanStep;

  // Move servo and allow it to settle.
  holdServoPosition(
    scanAngle,
    objectServoSettleTime
  );

  // Check boundary after positioning.
  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  // ======================================
  // COLLECT ULTRASONIC MEASUREMENT
  // ======================================

  float distance =
    measureDistance(frontTrigPin, frontEchoPin);

  objectScanDistances[objectScanIndex] =
    distance;

  Serial.print("Angle: ");
  Serial.print(scanAngle);

  Serial.print(" | Distance: ");
  Serial.println(distance);

  objectScanIndex++;
}

// ==========================================
// TURN TOWARD OBJECT
// ==========================================

void turnToObject() {

  // Stop turning if boundary is detected.
  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  unsigned long elapsed =
    millis() - objectStateStartTime;

  // Continue rotating until the
  // estimated turn duration has elapsed.
  if (elapsed < objectTurnDuration) {

    if (objectTurnDirection == -1) {

      // Rotate left.
      setMotors(
        -objectTurnSpeed,
        objectTurnSpeed
      );

    } else {

      // Rotate right.
      setMotors(
        objectTurnSpeed,
        -objectTurnSpeed
      );
    }

    return;
  }

  // Stop after completing rotation.
  setMotors(0, 0);

  objectState = VERIFY_TARGET;
  objectStateStartTime = millis();

  Serial.println("Chassis aligned.");
}

// ==========================================
// VERIFY SELECTED TARGET
// ==========================================

void verifyObjectTarget() {

  setMotors(0, 0);

  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  // Allow chassis to settle after turning.
  if (millis() - objectStateStartTime < 250) {
    return;
  }

  // Point sensor straight ahead.
  centerFrontServo();

  float distance =
    measureDistance(frontTrigPin, frontEchoPin);

  Serial.print("Verification distance: ");
  Serial.println(distance);

  // Invalid measurement.
  if (!isValidObjectDistance(distance)) {

    objectInvalidReadings++;

    if (objectInvalidReadings >=
        objectMaxInvalidReadings) {

      Serial.println("Verification failed.");
      startObjectScan();
    }

    return;
  }

  // Reject readings that differ too much
  // from the selected candidate.
  if (abs(distance - objectTargetDistance) >
      objectVerifyTolerance) {

    Serial.println("Target mismatch. Rescanning...");

    startObjectScan();
    return;
  }

  Serial.println("Target verified!");

  objectInvalidReadings = 0;

  objectApproachStartTime = millis();
  objectLastApproachTime = 0;

  objectState = APPROACHING_OBJECT;
}

// ==========================================
// APPROACH SELECTED TARGET
// ==========================================

void approachObject() {

  // Boundary detection takes priority.
  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  unsigned long currentTime = millis();

  // Stop if approaching takes too long.
  if (currentTime - objectApproachStartTime >=
      objectMaxApproachTime) {

    Serial.println("Approach timeout.");

    startObjectScan();
    return;
  }

  // Only take measurements at the
  // configured interval.
  if (currentTime - objectLastApproachTime <
      objectApproachInterval) {

    return;
  }

  objectLastApproachTime = currentTime;

  float distance =
    measureDistance(frontTrigPin, frontEchoPin);

  // Stop if sensor loses the target.
  if (!isValidObjectDistance(distance)) {

    setMotors(0, 0);

    objectInvalidReadings++;

    if (objectInvalidReadings >=
        objectMaxInvalidReadings) {

      Serial.println("Target lost.");

      startObjectScan();
    }

    return;
  }

  objectInvalidReadings = 0;

  // ======================================
  // OBJECT REACHED
  // ======================================

  if (distance <= objectStopDistance) {

    setMotors(0, 0);

    objectState = OBJECT_REACHED;

    Serial.println("=========================");
    Serial.println("OBJECT REACHED!");
    Serial.println("=========================");

    return;
  }

  // ======================================
  // MOVE FORWARD
  // ======================================

  setMotors(
    objectApproachSpeed,
    objectApproachSpeed
  );

  Serial.print("Approaching | Distance: ");
  Serial.println(distance);
}

// ==========================================
// AVOID WHITE-TAPE BOUNDARY
// ==========================================

void avoidObjectBoundary() {

  unsigned long elapsed =
    millis() - objectStateStartTime;

  // ======================================
  // PHASE 0: REVERSE
  // ======================================

  if (objectBoundaryPhase == 0) {

    if (elapsed < objectReverseTime) {

      setMotors(
        -objectReverseSpeed,
        -objectReverseSpeed
      );

      return;
    }

    // Stop after reversing.
    setMotors(0, 0);

    // Begin turning away.
    objectBoundaryPhase = 1;
    objectStateStartTime = millis();

    return;
  }

  // ======================================
  // PHASE 1: TURN AWAY
  // ======================================

  if (elapsed < objectBoundaryTurnTime) {

    if (objectTurnDirection == 1) {

      // Rotate right.
      setMotors(
        objectTurnSpeed,
        -objectTurnSpeed
      );

    } else {

      // Rotate left.
      setMotors(
        -objectTurnSpeed,
        objectTurnSpeed
      );
    }

    return;
  }

  // Recovery complete.
  setMotors(0, 0);

  Serial.println("Boundary recovery complete.");

  // Scan from the new orientation.
  startObjectScan();
}

// ==========================================
// ROTATE TO SEARCH NEW DIRECTION
// ==========================================

void searchForObjectsByRotation() {

  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  // Rotate clockwise for the
  // configured search duration.
  if (millis() - objectStateStartTime <
      objectSearchTurnTime) {

    setMotors(
      objectTurnSpeed,
      -objectTurnSpeed
    );

    return;
  }

  // Stop before scanning again.
  setMotors(0, 0);

  startObjectScan();
}

// ==========================================
// MAIN OBJECT DETECTION CONTROLLER
// ==========================================

/*
 * Called repeatedly by the tournament
 * controller during OBJECT_DETECTION.
 *
 * Each state manages one part of the
 * autonomous object-detection process.
 */

void updateObjectDetection() {

  switch (objectState) {

    case SCANNING:
      scanForObjects();
      break;

    case TURN_TO_OBJECT:
      turnToObject();
      break;

    case VERIFY_TARGET:
      verifyObjectTarget();
      break;

    case APPROACHING_OBJECT:
      approachObject();
      break;

    case AVOIDING_BOUNDARY:
      avoidObjectBoundary();
      break;

    case SEARCH_ROTATION:
      searchForObjectsByRotation();
      break;

    case OBJECT_REACHED:
      setMotors(0, 0);
      break;
  }
}
