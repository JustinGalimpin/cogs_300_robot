// Shared ultrasonic and IR sensor definitions and readers.

// Front-facing ultrasonic sensor.
const int frontTrigPin = 2;
const int frontEchoPin = 3;

// Left-facing ultrasonic sensor.
const int leftTrigPin = 13;
const int leftEchoPin = 12;

// Latest distance measurements.
// -1 indicates no valid ultrasonic echo.
float frontDistance = -1;
float leftDistance = -1;

// Sensor update interval.
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL = 100;

// ==========================================

const int leftIRPin = A0;
const int centerIRPin = A1;
const int rightIRPin = A2;

// Confirmed calibration:
// LOW = White tape
// HIGH = Dark floor
const int WHITE_DETECTED = LOW;

bool leftOnLine = false;
bool centerOnLine = false;
bool rightOnLine = false;

// ==========================================

/*
 * Measures distance using either sensor.
 *
 * Returns:
 * - Distance in centimetres
 * - -1 if no valid echo was detected
 */

float measureDistance(int trigPin, int echoPin) {

  // Ensure trigger starts LOW.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond trigger pulse.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the returning echo.
  long duration = pulseIn(echoPin, HIGH, 30000);

  // Return -1 if no echo was detected.
  if (duration == 0) {
    return -1;
  }

  // Convert echo duration to centimetres.
  return duration * 0.0343 / 2.0;
}

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
