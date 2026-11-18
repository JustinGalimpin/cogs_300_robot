// Wall-following parameters and the original Lab 5 navigation logic.
// Names are prefixed where necessary to avoid collisions with Lab 7.

// Desired distance from the left wall (cm).
const float targetWallDistance = 5.0;

// Normal forward speed.
const int wallBaseSpeed = 100;

// Proportional controller gain.
const float Kp = 3.0;

// Ignore errors within +/- 2 cm.
const float distanceTolerance = 2.0;

// Maximum motor speed correction.
const int maxCorrection = 80;

// Beyond this distance, the left wall is
// considered missing.
const float maxWallFollowDistance = 35.0;

// ==========================================
// INSIDE-CORNER CONFIGURATION
// ==========================================

// Begin turning right when a front obstacle
// is detected within this distance.
const float frontStopDistance = 25.0;

// Finish turning right when the front path
// appears clear by at least this distance.
const float frontClearDistance = 30.0;

// Speed for rotating right in place.
const int wallTurnSpeed = 80;

// Maximum duration of a right turn.
const unsigned long maxTurnDuration = 5000;

// Consecutive clear readings needed to
// finish the right turn.
const int requiredClearReadings = 1;

int clearReadingCount = 0;

// ==========================================
// OUTSIDE-CORNER CONFIGURATION
// ==========================================

// Number of consecutive missing-left-wall
// readings required to detect an outside corner.
const int requiredLostWallReadings = 2;

int lostWallReadingCount = 0;

// Move forward briefly before turning left.
// This helps the robot clear the wall's endpoint.
const unsigned long outsideAdvanceDuration = 300;

// Gradual left curve:
// Both wheels move forward, but the right
// wheel moves faster than the left.
const int outsideLeftSpeed = 65;
const int outsideRightSpeed = 110;

// Minimum time spent turning left before
// accepting a reacquired wall.
const unsigned long minimumOutsideTurnDuration = 300;

// Maximum distance at which a wall can be
// considered reacquired.
const float outsideReacquireDistance = 30.0;

// Consecutive wall detections required to
// finish the outside-corner turn.
const int requiredReacquireReadings = 2;

int reacquireReadingCount = 0;

// Maximum duration of an outside-corner turn.
const unsigned long maxOutsideTurnDuration = 3500;

// Track when outside-corner actions begin.
unsigned long outsideAdvanceStartTime = 0;
unsigned long outsideTurnStartTime = 0;

// ==========================================
// NAVIGATION STATES
// ==========================================

enum RobotState {
  FOLLOW_WALL,
  TURN_RIGHT,
  ADVANCE_OUTSIDE,
  TURN_LEFT_OUTSIDE,
  STOPPED
};

// Start in normal wall-following mode.
RobotState wallState = FOLLOW_WALL;

// Track when an inside-corner turn begins.
unsigned long turnStartTime = 0;

// ==========================================

// A front wall exists when a valid reading
// is within the stopping distance.
bool frontWallDetected() {

  return frontDistance >= 0 &&
         frontDistance <= frontStopDistance;
}

// A missing left wall may indicate an
// outside corner.
bool leftWallMissing() {

  return leftDistance < 0 ||
         leftDistance > maxWallFollowDistance;
}

// Check whether the left sensor detects
// a wall close enough to reacquire.
bool leftWallReacquired() {

  return leftDistance >= 0 &&
         leftDistance <= outsideReacquireDistance;
}

// ==========================================
// BEGIN INSIDE-CORNER RIGHT TURN
// ==========================================

void beginRightTurn() {

  wallState = TURN_RIGHT;

  // Record when the turn begins.
  turnStartTime = millis();

  // Reset counters from previous manoeuvres.
  clearReadingCount = 0;
  lostWallReadingCount = 0;
  reacquireReadingCount = 0;

  Serial.println("Front wall detected! Turning right.");

  // Rotate clockwise in place.
  setMotors(wallTurnSpeed, -wallTurnSpeed);
}

// ==========================================
// WALL-FOLLOWING CONTROLLER
// ==========================================

void updateWallFollowing() {

  // ------------------------------------------
  // 1. CHECK FOR A MISSING LEFT WALL
  // ------------------------------------------

  if (leftWallMissing()) {

    lostWallReadingCount++;

    // Confirm the wall is actually missing
    // before beginning an outside turn.
    if (lostWallReadingCount >= requiredLostWallReadings) {

      wallState = ADVANCE_OUTSIDE;

      outsideAdvanceStartTime = millis();

      lostWallReadingCount = 0;
      reacquireReadingCount = 0;

      Serial.println("Outside corner detected!");
      Serial.println("Advancing past wall endpoint.");

      // Move forward to clear the corner.
      setMotors(wallBaseSpeed, wallBaseSpeed);

      return;
    }

    // First missing reading:
    // Continue straight while waiting
    // for confirmation.
    setMotors(wallBaseSpeed, wallBaseSpeed);

    return;
  }

  // Left wall is detected again.
  lostWallReadingCount = 0;

  // ------------------------------------------
  // 2. CALCULATE DISTANCE ERROR
  // ------------------------------------------

  float error = leftDistance - targetWallDistance;

  // Ignore small distance errors.
  if (abs(error) <= distanceTolerance) {
    error = 0;
  }

  // ------------------------------------------
  // 3. CALCULATE PROPORTIONAL CORRECTION
  // ------------------------------------------

  int correction = constrain(
    (int)(Kp * error),
    -maxCorrection,
    maxCorrection
  );

  // ------------------------------------------
  // 4. ADJUST MOTOR SPEEDS
  // ------------------------------------------

  /*
   * Too far from left wall:
   *   Left wheel slows down.
   *   Right wheel speeds up.
   *   Robot steers left.
   *
   * Too close to left wall:
   *   Left wheel speeds up.
   *   Right wheel slows down.
   *   Robot steers right.
   */

  int leftSpeed = wallBaseSpeed - correction;
  int rightSpeed = wallBaseSpeed + correction;

  setMotors(leftSpeed, rightSpeed);
}

// ==========================================
// INSIDE-CORNER RIGHT TURN
// ==========================================

void updateRightTurn() {

  // ------------------------------------------
  // 1. CHECK TURN TIMEOUT
  // ------------------------------------------

  if (millis() - turnStartTime >= maxTurnDuration) {

    setMotors(0, 0);
    wallState = STOPPED;

    Serial.println("Right-turn timeout! Robot stopped.");

    return;
  }

  // ------------------------------------------
  // 2. CHECK FRONT CLEARANCE
  // ------------------------------------------

  /*
   * A reading counts as potentially clear if:
   *
   * 1. The front distance is at least
   *    frontClearDistance.
   *
   * OR
   *
   * 2. The sensor returns -1 (no echo).
   *
   * Multiple consecutive readings reduce
   * the effect of isolated sensor errors.
   */

  if (frontDistance >= frontClearDistance ||
      frontDistance < 0) {

    clearReadingCount++;

  }
  else {

    clearReadingCount = 0;
  }

  // ------------------------------------------
  // 3. FINISH RIGHT TURN
  // ------------------------------------------

  if (clearReadingCount >= requiredClearReadings) {

    setMotors(0, 0);

    clearReadingCount = 0;
    lostWallReadingCount = 0;

    wallState = FOLLOW_WALL;

    Serial.println("Right turn complete!");
    Serial.println("Resuming wall following.");

    return;
  }

  // ------------------------------------------
  // 4. CONTINUE ROTATING RIGHT
  // ------------------------------------------

  // Left wheel forward, right wheel backward.
  setMotors(wallTurnSpeed, -wallTurnSpeed);
}

// ==========================================
// OUTSIDE-CORNER FORWARD ADVANCE
// ==========================================

void updateOutsideAdvance() {

  /*
   * Before curving left, move forward briefly.
   *
   * This helps the robot pass the endpoint
   * of the original left wall.
   *
   * Front obstacle detection is handled
   * before this function is called.
   */

  if (millis() - outsideAdvanceStartTime <
      outsideAdvanceDuration) {

    setMotors(wallBaseSpeed, wallBaseSpeed);

    return;
  }

  // Begin the gradual left curve.
  wallState = TURN_LEFT_OUTSIDE;

  outsideTurnStartTime = millis();
  reacquireReadingCount = 0;

  Serial.println("Starting gradual outside left turn.");

  setMotors(outsideLeftSpeed, outsideRightSpeed);
}

// ==========================================
// OUTSIDE-CORNER LEFT TURN
// ==========================================

void updateOutsideLeftTurn() {

  unsigned long elapsedTime =
    millis() - outsideTurnStartTime;

  // ------------------------------------------
  // 1. CHECK TURN TIMEOUT
  // ------------------------------------------

  if (elapsedTime >= maxOutsideTurnDuration) {

    setMotors(0, 0);

    wallState = STOPPED;

    Serial.println("Outside-turn timeout!");
    Serial.println("Robot stopped.");

    return;
  }

  // ------------------------------------------
  // 2. CHECK WHETHER LEFT WALL HAS RETURNED
  // ------------------------------------------

  /*
   * Do not finish the turn immediately.
   *
   * The minimum turning duration helps
   * prevent the robot from accepting the
   * original wall before moving around
   * the outside corner.
   */

  if (elapsedTime >= minimumOutsideTurnDuration) {

    if (leftWallReacquired()) {

      reacquireReadingCount++;

    }
    else {

      reacquireReadingCount = 0;
    }

    // ----------------------------------------
    // 3. FINISH OUTSIDE TURN
    // ----------------------------------------

    if (reacquireReadingCount >=
        requiredReacquireReadings) {

      setMotors(0, 0);

      reacquireReadingCount = 0;
      lostWallReadingCount = 0;

      wallState = FOLLOW_WALL;

      Serial.println("Left wall reacquired!");
      Serial.println("Resuming wall following.");

      return;
    }
  }

  // ------------------------------------------
  // 4. CONTINUE GRADUAL LEFT CURVE
  // ------------------------------------------

  /*
   * Both wheels move forward.
   *
   * Left wheel: slower
   * Right wheel: faster
   *
   * This creates a gradual leftward arc,
   * rather than an in-place rotation.
   */

  setMotors(outsideLeftSpeed, outsideRightSpeed);
}

// ==========================================
// NAVIGATION CONTROLLER
// ==========================================

void updateWallNavigation() {

  // ------------------------------------------
  // PRIORITY 1: FRONT WALL
  // ------------------------------------------

  /*
   * A front obstacle takes priority over
   * wall following and outside-corner turns.
   *
   * If the robot encounters a front wall
   * during an outside-corner manoeuvre,
   * it switches to the right-turn state.
   *
   * While already turning right, we let
   * updateRightTurn() determine when the
   * front path is sufficiently clear.
   */

  if (wallState != STOPPED &&
      wallState != TURN_RIGHT &&
      frontWallDetected()) {

    beginRightTurn();

    return;
  }

  // ------------------------------------------
  // PRIORITY 2: CURRENT NAVIGATION STATE
  // ------------------------------------------

  switch (wallState) {

    case FOLLOW_WALL:

      // Follow the left wall or detect
      // a possible outside corner.
      updateWallFollowing();

      break;

    case TURN_RIGHT:

      // Rotate right until front clearance
      // is confirmed.
      updateRightTurn();

      break;

    case ADVANCE_OUTSIDE:

      // Move forward past the wall endpoint.
      updateOutsideAdvance();

      break;

    case TURN_LEFT_OUTSIDE:

      // Curve left until the wall is reacquired.
      updateOutsideLeftTurn();

      break;

    case STOPPED:

      // Remain stationary until reset.
      setMotors(0, 0);

      break;
  }
}

// ==========================================
