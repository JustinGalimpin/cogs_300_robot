/* COGS 300 - WallFollowing.ino
 * Reactive left-wall following with adaptive right turns and brief reverse recovery.
 * No left recovery rotation or outside-corner search. Uses existing Sensors and Motors tabs.
 */
const float targetWallDistance = 13.0;
const float distanceTolerance = 2.5;
const float Kp = 2;
const int maxCorrection = 15;
const float maxWallFollowDistance = 35.0;
const float wallCautionDistance = 8.0;
const int wallBaseSpeed = 80;
const int wallMinForwardSpeed = 75;
const int wallTurnSpeed = 75;
const int wallMediumTurnSpeed = 80;
const int wallMaxTurnSpeed = 85;
const int wallRecoverySpeed = 100;
const float frontEmergencyDistance = 5.0;
const unsigned long wallStuckTurnDuration = 1200;
const unsigned long wallReverseDuration = 350;
const float frontStopDistance = 12.0;
const float frontClearDistance = 14.0;
const float frontSlowDistance = 20.0;
const unsigned long wallSensorInterval = 60;

// Sensor conditioning and inferred stall detection.
const unsigned long wallSonarGap = 60;
const int wallMedianSize = 5;
const int requiredLostWallReadings = 3;
const int requiredReacquireReadings = 2;
const unsigned long wallFollowStuckDuration = 1000;
const float wallFrontProgressTolerance = 4.0;
const float wallLeftProgressTolerance = 2.0;
const int wallStallCorrection = 10;
const float wallRecurringFraction = 0.60f;
float wallFrontSamples[wallMedianSize], wallLeftSamples[wallMedianSize];
int wallSampleCount = 0, wallSampleIndex = 0;
float wallRawFront = -1, wallRawLeft = -1;
bool wallLeftPresent = false;
int wallLostCount = 0, wallFoundCount = 0;
float wallLastPresentLeft = -1;
bool wallFollowTracking = false;
unsigned long wallFollowStart = 0;
float wallAnchorFront = -1, wallAnchorLeft = -1;
int wallFollowSamples = 0, wallRecurringSamples = 0;

bool wallTurningRight = false;
bool wallReversing = false;
unsigned long wallTurnStart = 0;
unsigned long wallReverseStart = 0;
float wallFilteredLeft = -1;
float wallCorrection = 0;

void wallResetSteering() {
  wallFilteredLeft = -1;
  wallCorrection = 0;
}

// Treat missing echoes as far readings for median ordering, not as zero.
float wallMedian(float *samples, int count) {
  float sorted[wallMedianSize];
  for (int i = 0; i < count; ++i) sorted[i] = samples[i] < 0 ? 10000.0f : samples[i];
  for (int i = 1; i < count; ++i) {
    float v = sorted[i]; int j = i;
    while (j > 0 && sorted[j-1] > v) { sorted[j] = sorted[j-1]; --j; }
    sorted[j] = v;
  }
  float v = sorted[count/2];
  return v >= 10000.0f ? -1 : v;
}

void wallResetProgress() {
  wallFollowTracking = false;
  wallFollowSamples = wallRecurringSamples = 0;
}

void wallResetSamples() {
  wallSampleCount = wallSampleIndex = 0;
  wallLostCount = wallFoundCount = 0;
  wallLeftPresent = false;
  wallLastPresentLeft = -1;
  wallResetProgress();
}

void wallConditionReadings() {
  wallFrontSamples[wallSampleIndex] = wallRawFront;
  wallLeftSamples[wallSampleIndex] = wallRawLeft;
  wallSampleIndex = (wallSampleIndex + 1) % wallMedianSize;
  if (wallSampleCount < wallMedianSize) ++wallSampleCount;
  frontDistance = wallMedian(wallFrontSamples, wallSampleCount);
  float side = wallMedian(wallLeftSamples, wallSampleCount);
  bool seen = side >= 0 && side <= maxWallFollowDistance;
  if (seen) {
    wallLastPresentLeft = side;
    wallLostCount = 0;
    if (wallFoundCount < requiredReacquireReadings) ++wallFoundCount;
    if (wallFoundCount >= requiredReacquireReadings) wallLeftPresent = true;
  } else {
    wallFoundCount = 0;
    if (wallLostCount < requiredLostWallReadings) ++wallLostCount;
    if (wallLostCount >= requiredLostWallReadings) wallLeftPresent = false;
  }
  leftDistance = wallLeftPresent ? wallLastPresentLeft : -1;
}

void initializeWallFollowing() {
  setMotors(0, 0);
  wallTurningRight = false;
  wallReversing = false;
  wallTurnStart = wallReverseStart = 0;
  wallResetSamples();
  wallResetSteering();
  lastSensorTime = millis() - wallSensorInterval;
}

bool leftWallMissing() {
  return !wallLeftPresent;
}

// Reduce forward PWM as a valid front reading approaches the turn threshold.
int wallForwardSpeed() {
  float distance = frontDistance;
  if (wallRawFront >= 0 && (distance < 0 || wallRawFront < distance)) distance = wallRawFront;
  if (distance < 0 || distance >= frontSlowDistance) return wallBaseSpeed;
  float fraction = constrain((distance - frontStopDistance) /
      (frontSlowDistance - frontStopDistance), 0.0f, 1.0f);
  return (int)(wallMinForwardSpeed +
      fraction * (wallBaseSpeed - wallMinForwardSpeed));
}

void wallSteer(int speed) {
  // No left wall: drive straight, rather than searching by turning left.
  if (leftWallMissing()) {
    wallResetSteering();
    setMotors(speed, speed);
    return;
  }
  if (wallFilteredLeft < 0) wallFilteredLeft = leftDistance;
  else wallFilteredLeft = 0.55f * wallFilteredLeft + 0.45f * leftDistance;

  float error = wallFilteredLeft - targetWallDistance;
  if (error > distanceTolerance) error -= distanceTolerance;
  else if (error < -distanceTolerance) error += distanceTolerance;
  else error = 0;

  float desired = constrain(Kp * error, -(float)maxCorrection,
                            (float)maxCorrection);
  // Smooth changes in steering between sensor readings.
  wallCorrection += constrain(desired - wallCorrection, -8.0f, 8.0f);
  // Raw close side reading takes priority over filtered steering.
  if (leftDistance < wallCautionDistance) {
    speed = min(speed, 85);
    wallCorrection = -(float)maxCorrection;
  }
  int correction = (int)wallCorrection;
  // Positive correction turns left; negative correction turns right.
  setMotors(speed - correction, speed + correction);
}

// Start a single timed reverse. Sensor readings cannot restart its timer.
void beginWallReverse() {
  wallReversing = true;
  wallTurningRight = false;
  wallReverseStart = millis();
  wallResetSamples();
  wallResetSteering();
  setMotors(-wallRecoverySpeed, -wallRecoverySpeed);
  Serial.println("RECOVERY: reversing before retrying right turn");
}

// A recurring side distance while steering is evidence of a possible stall.
// A close side wall can be sufficient even if the front echo alternates wildly.
// Evaluate a whole time window instead of resetting on every isolated spike.
bool wallFollowAppearsStuck() {
  bool closeSide = !leftWallMissing() && leftDistance < wallCautionDistance;
  bool steering = !leftWallMissing() && abs(wallCorrection) >= wallStallCorrection;
  bool nearFront = frontDistance >= 0 && frontDistance < frontSlowDistance;
  if (!wallFollowTracking) {
    if (!(closeSide || steering || nearFront)) return false;
    wallFollowTracking = true;
    wallFollowStart = millis();
    wallAnchorFront = frontDistance;
    wallAnchorLeft = leftDistance;
    wallFollowSamples = wallRecurringSamples = 0;
  }
  ++wallFollowSamples;
  bool sameLeft = leftDistance >= 0 && wallAnchorLeft >= 0 &&
      abs(leftDistance - wallAnchorLeft) <= wallLeftProgressTolerance;
  bool sameFront = frontDistance >= 0 && wallAnchorFront >= 0 &&
      abs(frontDistance - wallAnchorFront) <= wallFrontProgressTolerance;
  // Use raw side recurrence as well: a median can temporarily switch echo modes.
  bool sameRawLeft = wallRawLeft >= 0 && wallAnchorLeft >= 0 &&
      abs(wallRawLeft - wallAnchorLeft) <= wallLeftProgressTolerance;
  bool anchoredClose = wallAnchorLeft >= 0 && wallAnchorLeft < wallCautionDistance;
  if ((anchoredClose && (sameLeft || sameRawLeft)) ||
      ((steering || nearFront) && sameFront &&
       (sameLeft || (wallAnchorLeft < 0 && leftWallMissing())))) {
    ++wallRecurringSamples;
  }
  if (millis() - wallFollowStart < wallFollowStuckDuration) return false;
  bool stuck = wallFollowSamples >= 4 &&
      (float)wallRecurringSamples / wallFollowSamples >= wallRecurringFraction;
  wallResetProgress();
  return stuck;
}

void updateWallNavigation() {
  // Extremely close front wall: create clearance before attempting a pivot.
  if ((wallRawFront >= 0 && wallRawFront <= frontEmergencyDistance) ||
      (frontDistance >= 0 && frontDistance <= frontEmergencyDistance)) {
    beginWallReverse();
    return;
  }

  // Hysteresis: start at the stop threshold, finish at the clear threshold.
  if (wallTurningRight) {
    if ((frontDistance < 0 || frontDistance >= frontClearDistance) &&
        (wallRawFront < 0 || wallRawFront >= frontClearDistance)) {
      wallTurningRight = false;
      wallResetSteering();
      wallResetProgress();
    }
  } else if ((frontDistance >= 0 && frontDistance <= frontStopDistance) ||
             (wallRawFront >= 0 && wallRawFront <= frontStopDistance)) {
    wallTurningRight = true;
    wallTurnStart = millis();
    wallResetSteering();
  }

  if (wallTurningRight) {
    wallResetProgress();
    unsigned long elapsed = millis() - wallTurnStart;
    // A failed turn is inferred from elapsed time, not measured wheel motion.
    if (elapsed >= wallStuckTurnDuration) {
      beginWallReverse();
      return;
    }
    int speed = wallTurnSpeed;
    if (elapsed >= 1000) speed = wallMaxTurnSpeed;
    else if (elapsed >= 500) speed = wallMediumTurnSpeed;
    setMotors(speed, -speed);
    return;
  }
  wallSteer(wallForwardSpeed());
  if (wallFollowAppearsStuck()) {
    Serial.println("RECOVERY: possible stall during wall following");
    beginWallReverse();
  }
}

void printWallSensorReadings() {
  Serial.print("Raw F/L: "); Serial.print(wallRawFront);
  Serial.print(" / "); Serial.print(wallRawLeft);
  Serial.print(" | Front: "); Serial.print(frontDistance);
  Serial.print(" | Left: "); Serial.print(leftDistance);
  Serial.print(" | State: ");
  if (wallReversing) Serial.println("REVERSE");
  else if (wallTurningRight) Serial.println("TURN_RIGHT");
  else if (leftWallMissing()) Serial.println("STRAIGHT_NO_LEFT_WALL");
  else Serial.println("FOLLOW_WALL");
}

void runWallFollowingCycle() {
  // Service the reverse timer every loop, independently of sonar delays.
  if (wallReversing) {
    if (millis() - wallReverseStart < wallReverseDuration) return;
    setMotors(0, 0);
    wallReversing = false;
    // Reassess with fresh readings before retrying the right turn.
    wallTurningRight = true;
    wallTurnStart = millis();
    lastSensorTime = millis() - wallSensorInterval;
  }
  if (millis() - lastSensorTime >= wallSensorInterval) {
    wallRawFront = measureDistance(frontTrigPin, frontEchoPin);
    delay(wallSonarGap);
    wallRawLeft = measureDistance(leftTrigPin, leftEchoPin);
    wallConditionReadings();
    updateWallNavigation();
    printWallSensorReadings();
    lastSensorTime = millis();
  }
}