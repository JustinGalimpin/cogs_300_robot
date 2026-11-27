
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
 * - Measurements every 10 degrees
 * - Group readings into candidate objects
 * - Select largest estimated object in current sweep
 * - Rotate chassis toward selected target
 * - Verify target before approaching
 * - Drive in short increments and rescan
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

extern bool leftOnLine, centerOnLine, rightOnLine;
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
const int objectTurnSpeed = 75;
const int objectApproachSpeed = 110;
const int objectNearApproachSpeed = 75;
const int objectReverseSpeed = 130;

// ==========================================
// DISTANCE CONFIGURATION
// ==========================================

// Minimum valid object distance.
const float objectMinDistance = 4.0;

// Maximum distance considered for detection.
const float objectMaxDistance = 240.0;

// Stop when an object is this close.
const float objectStopDistance = 0.5;
const float objectNearDistance = 25.0;
const float objectExploreClearance = 35.0;
const float objectUnexpectedDrop = 12.0;


// Maximum difference between consecutive
// readings grouped into the same object.
const float objectGroupingTolerance = 10.0;

// Allowed difference between the original
// target distance and verification distance.
const float objectVerifyTolerance = 12.0;

// ==========================================
// SCAN CONFIGURATION
// ==========================================

// Local scan settings: no changes needed in Sensors.ino.
const int objectScanMinAngle = 30;
const int objectScanMaxAngle = 150;
const int objectScanStep = 15;
const int objectScanCount = (objectScanMaxAngle - objectScanMinAngle) / objectScanStep + 1;
const unsigned long objectPingGap = 60;
const int objectBeamAllowance = 0; // Calibrate using known objects.

// Time allowed for servo positioning.
const unsigned long objectServoSettleTime = 100;

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
const unsigned long objectSearchTurnTime = 600;

// ==========================================
// BOUNDARY AVOIDANCE CONFIGURATION
// ==========================================

// Time spent reversing from white tape.
const unsigned long objectReverseTime = 350;

// Time spent turning away from boundary.
const unsigned long objectBoundaryTurnTime = 500;
const unsigned long objectWideBoundaryTurnTime = 850;

// ==========================================
// APPROACH CONFIGURATION
// ==========================================

// Time between distance measurements.
const unsigned long objectApproachInterval = 100;

// Maximum time spent approaching a target.
const unsigned long objectForwardStepTime = 600;
const unsigned long objectNearStepTime = 400;
const unsigned long objectExploreStepTime = 700;
const int objectMaxSearchRotations = 5;

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
unsigned long objectRecoveryTurnTime = objectBoundaryTurnTime;
float objectPreviousDistance = -1;

int objectDefaultEscapeDirection = 1;
int objectSearchRotations = 0;
bool objectExploring = false;
unsigned long objectStepDuration = 0;
unsigned long objectLastServoRefresh = 0;

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

// Take three readings at ONE angle. A majority must be valid.
// Keep the servo energized between pings without Servo.h.
float objectMedianAtAngle(int angle) {
  float values[3];
  int count = 0;
  for (int i = 0; i < 3; i++) {
    float d = measureDistance(frontTrigPin, frontEchoPin);
    if (d > 0) values[count++] = d;
    holdServoPosition(angle, objectPingGap);
  }
  if (count < 2) return -1;
  for (int i = 0; i < count; i++)
    for (int j = i + 1; j < count; j++)
      if (values[j] < values[i]) {
        float t = values[i]; values[i] = values[j]; values[j] = t;
      }
  if (count == 2 && fabs(values[0] - values[1]) > objectGroupingTolerance) return -1;
  return count == 3 ? values[1] : (values[0] + values[1]) / 2.0f;
}

// ==========================================
// START NEW OBJECT SCAN
// ==========================================

void startObjectScan() {

  // Stop chassis movement before scanning.
  setMotors(0, 0);

  objectScanIndex = 0;
  objectExploring = false;

  objectFound = false;

  objectTargetDistance = -1;
  objectTargetWidth = 0;
  objectTargetAngle = 90;

  objectInvalidReadings = 0;

  objectPreviousDistance = -1;

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
  objectSearchRotations = 0;

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
 * The candidate with the largest estimated physical width is selected.
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
        (groupEnd - groupStart + 1) * objectScanStep;

      // Convert half-angle into radians.
      float halfAngleRadians =
        (max(1.0f, angularWidth - objectBeamAllowance) / 2.0) *
        PI / 180.0;

      // Approximate physical width.
      float estimatedWidth =
        2.0 * averageDistance *
        tan(halfAngleRadians);

      // Determine the group's center.
      float centerIndex = (groupStart + groupEnd) / 2.0f;

      int centerAngle =
        objectScanMinAngle +
        centerIndex * objectScanStep;

      Serial.print("Candidate at ");
      Serial.print(centerAngle);

      Serial.print(" degrees | Distance: ");
      Serial.print(averageDistance);

      Serial.print(" cm | Width estimate: ");
      Serial.print(estimatedWidth);

      Serial.println(" cm");

      // Rank all detected groups by estimated physical width.
      // Edge groups are allowed, but their width may be underestimated.
      // No fixed target width or acceptance range is required.
      if (!objectFound || estimatedWidth > objectTargetWidth) {

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
  objectSearchRotations = 0;

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
    objectTurnDirection = objectDefaultEscapeDirection;
    objectDefaultEscapeDirection *= -1;
  }
  objectRecoveryTurnTime = (leftOnLine && rightOnLine) || (centerOnLine && (leftOnLine || rightOnLine)) ? objectWideBoundaryTurnTime : objectBoundaryTurnTime;
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

      // After a full approximate search rotation, explore only if
      // the centered sensor confirms a clear path. No echo is not clearance.
      if (objectSearchRotations >= objectMaxSearchRotations) {
        objectSearchRotations = 0;
        float ahead = objectMedianAtAngle(servoCenterAngle);
        if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
        if (ahead >= objectExploreClearance) {
          objectExploring = true;
          objectPreviousDistance = ahead;
          objectStepDuration = objectExploreStepTime;
          objectApproachStartTime = millis();
          objectLastApproachTime = millis() - objectApproachInterval;
          objectState = APPROACHING_OBJECT;
          Serial.println("Exploring one short step");
          return;
        }
      }
      objectSearchRotations++;
      objectState = SEARCH_ROTATION;
      objectStateStartTime = millis();

      return;
    }

    objectSearchRotations = 0;
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
    objectScanMinAngle +
    objectScanIndex * objectScanStep;

  // Move servo and allow it to settle.
  holdServoPosition(
    scanAngle,
    objectScanIndex == 0 ? 300 : objectServoSettleTime
  );

  // Check boundary after positioning.
  if (objectBoundaryDetected()) {

    startBoundaryAvoidance();
    return;
  }

  // ======================================
  // COLLECT ULTRASONIC MEASUREMENT
  // ======================================

  float distance = objectMedianAtAngle(scanAngle);

  // Only a straight-ahead reading can finish the stage during a sweep.
  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  if (scanAngle == servoCenterAngle && distance > 0 &&
      distance <= objectStopDistance) {
    setMotors(0, 0);
    objectState = OBJECT_REACHED;
    Serial.println("OBJECT REACHED!");
    return;
  }

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

void startObjectObstacleRecovery() {
  objectSearchRotations = 0;
  setMotors(0, 0);
  objectTurnDirection = objectDefaultEscapeDirection;
  objectDefaultEscapeDirection *= -1;
  objectRecoveryTurnTime = objectBoundaryTurnTime;
  objectBoundaryPhase = 0;
  objectStateStartTime = millis();
  objectState = AVOIDING_BOUNDARY;
  Serial.println("Unexpected obstacle: reversing and rescanning");
}

void verifyObjectTarget() {
  setMotors(0, 0);
  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  if (millis() - objectStateStartTime < 250) return;
  centerFrontServo();
  float distance = objectMedianAtAngle(servoCenterAngle);
  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  Serial.print("Verification distance: "); Serial.println(distance);
  // Arrival takes priority over distance mismatch or obstacle recovery.
  if (distance > 0 && distance <= objectStopDistance) {
    setMotors(0, 0);
    objectState = OBJECT_REACHED;
    Serial.println("OBJECT REACHED!");
    return;
  }
  if (!isValidObjectDistance(distance) ||
      fabs(distance - objectTargetDistance) > objectVerifyTolerance) {
    if (++objectInvalidReadings >= objectMaxInvalidReadings) startObjectScan();
    return;
  }
  objectPreviousDistance = distance;
  objectInvalidReadings = 0;
  objectStepDuration = distance <= objectNearDistance ?
                       objectNearStepTime : objectForwardStepTime;
  objectApproachStartTime = millis();
  objectLastApproachTime = millis() - objectApproachInterval;
  objectState = APPROACHING_OBJECT;
  Serial.println("Target verified: beginning one forward step");
}

// Move for only one short increment. IR takes priority each loop.
// A valid close front reading ends the stage immediately.
void approachObject() {
  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  unsigned long now = millis();
  if (now - objectApproachStartTime >= objectStepDuration) {
    startObjectScan(); return;
  }
  if (now - objectLastApproachTime >= objectApproachInterval) {
    objectLastApproachTime = now;
    float distance = measureDistance(frontTrigPin, frontEchoPin);
    if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
    // Check arrival before rejecting close readings as obstacles.
    if (distance > 0 && distance <= objectStopDistance) {
      setMotors(0, 0);
      objectState = OBJECT_REACHED;
      Serial.println("OBJECT REACHED!");
      return;
    }
    if (distance > 0 && (distance < objectMinDistance ||
        objectPreviousDistance - distance > objectUnexpectedDrop ||
        (objectExploring && distance <= objectNearDistance))) {
      startObjectObstacleRecovery(); return;
    }
    if (distance <= 0 || (!objectExploring &&
        (!isValidObjectDistance(distance) ||
         distance - objectPreviousDistance > objectVerifyTolerance))) {
      startObjectScan(); return;
    }
    objectPreviousDistance = distance;
    Serial.print("Forward step | Distance: "); Serial.println(distance);
  }
  // Refresh the manual servo without using Timer1. One pulse blocks ~20 ms.
  if (millis() - objectLastServoRefresh >= 20) {
    objectLastServoRefresh = millis();
    sendServoPulse(servoCenterAngle);
  }
  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  if (millis() - objectApproachStartTime >= objectStepDuration) {
    startObjectScan(); return;
  }
  int forwardSpeed = objectPreviousDistance <= objectNearDistance
                     ? objectNearApproachSpeed : objectApproachSpeed;
  setMotors(forwardSpeed, forwardSpeed);
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

  if (objectBoundaryDetected()) { startBoundaryAvoidance(); return; }
  if (elapsed < objectRecoveryTurnTime) {

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
