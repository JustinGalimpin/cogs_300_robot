
/*
 * COGS 300 - Tournament Robot
 * Sensors.ino
 *
 * Shared sensor and servo functionality.
 *
 * Features:
 * - Front ultrasonic distance sensor
 * - Left ultrasonic distance sensor
 * - Three IR line/boundary sensors
 * - Manual servo control (without Servo.h)
 *
 * Servo signal: A5
 *
 * IMPORTANT:
 * The servo only needs to sweep during
 * the OBJECT_DETECTION stage.
 *
 * The servo functions below use blocking
 * pulse generation. During a pulse/hold,
 * other robot logic cannot execute.
 *
 * We will coordinate these functions
 * with ObjectDetection.ino.
 */

// ==========================================
// ULTRASONIC SENSOR CONFIGURATION
// ==========================================

// Front-facing ultrasonic sensor.
// Mounted on the positional servo.
extern const int frontTrigPin = 2;
extern const int frontEchoPin = 3;

// Left-facing ultrasonic sensor.
// Remains fixed to the robot.
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
// IR SENSOR CONFIGURATION
// ==========================================

const int leftIRPin = A0;
const int centerIRPin = A1;
const int rightIRPin = A2;

// Confirmed calibration:
// HIGH = Dark floor
// LOW  = White tape
const int WHITE_DETECTED = LOW;

bool leftOnLine = false;
bool centerOnLine = false;
bool rightOnLine = false;

// ==========================================
// SERVO CONFIGURATION
// ==========================================

// Servo signal pin.
// A5 can be used as a digital output.
const int servoPin = A5;

// Approximate servo control pulse widths.
// These match our successful standalone test.
const int servoMinPulse = 1000;
const int servoMaxPulse = 2000;

// Servo positions.
extern const int servoCenterAngle = 90;
extern const int servoMinAngle = 30;
extern const int servoMaxAngle = 150;

// Servo scan increments.
//
// ObjectDetection.ino can use this value
// when scanning at different angles.
extern const int servoScanStep = 15;

// Servo pulse refresh period.
const unsigned long servoPeriod = 20000UL;

// Current commanded servo angle.
int currentServoAngle = servoCenterAngle;

// ==========================================
// ULTRASONIC DISTANCE MEASUREMENT
// ==========================================

/*
 * Measures distance using either sensor.
 *
 * Returns:
 * - Distance in centimetres
 * - -1 if no valid echo was detected
 *
 * NOTE:
 * pulseIn() can block for up to 30 ms.
 * Servo pulses are not generated while
 * this function is waiting for an echo.
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
// SEND ONE SERVO CONTROL PULSE
// ==========================================

/*
 * Generates one complete servo pulse.
 *
 * The requested angle determines the
 * width of the HIGH signal.
 *
 * This function blocks for approximately
 * 20 milliseconds.
 *
 * It does not use Servo.h or Timer1,
 * preserving PWM on motor pin 9.
 */

void sendServoPulse(int angle) {

  // Constrain angle to the servo's
  // nominal range.
  angle = constrain(angle, 0, 180);

  // Convert angle into pulse width.
  int pulseWidth = map(
    angle,
    0, 180,
    servoMinPulse,
    servoMaxPulse
  );

  // Begin the control pulse.
  digitalWrite(servoPin, HIGH);

  // Maintain HIGH for the desired width.
  delayMicroseconds(pulseWidth);

  // End the control pulse.
  digitalWrite(servoPin, LOW);

  // Wait for the remainder of the
  // approximately 20 ms refresh cycle.
  delayMicroseconds(servoPeriod - pulseWidth);
}

// ==========================================
// HOLD SERVO AT A POSITION
// ==========================================

/*
 * Repeatedly generates servo pulses for
 * the requested duration.
 *
 * Useful for:
 * - Moving to a new scanning position
 * - Allowing the servo to settle
 * - Holding the sensor at a known angle
 *
 * IMPORTANT:
 * This function blocks.
 *
 * In ObjectDetection.ino, we'll use
 * short positioning periods and check
 * arena boundaries between operations.
 */

void holdServoPosition(
  int angle,
  unsigned long durationMs
) {

  // Restrict movement to our tested
  // scanning range.
  angle = constrain(
    angle,
    servoMinAngle,
    servoMaxAngle
  );

  // Remember the commanded position.
  currentServoAngle = angle;

  unsigned long startTime = millis();

  // Keep sending pulses until the
  // requested duration has elapsed.
  while (millis() - startTime < durationMs) {

    sendServoPulse(angle);
  }
}

// ==========================================
// CENTER FRONT SERVO
// ==========================================

/*
 * Point the front ultrasonic sensor
 * straight ahead.
 *
 * This is useful when:
 * - Entering the wall-following stage
 * - Beginning object detection
 * - Returning the sensor to its default
 *   orientation
 */

void centerFrontServo() {

  holdServoPosition(
    servoCenterAngle,
    400
  );
}

// ==========================================
// SENSOR INITIALIZATION
// ==========================================

/*
 * Initialize ultrasonic sensors,
 * IR sensors, and servo signal pin.
 *
 * We do not begin sweeping here.
 * ObjectDetection.ino will control
 * when servo movement is required.
 */

void initializeSensors() {

  // Front ultrasonic sensor.
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);

  // Left ultrasonic sensor.
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  // Initialize trigger signals LOW.
  digitalWrite(frontTrigPin, LOW);
  digitalWrite(leftTrigPin, LOW);

  // IR sensors.
  pinMode(leftIRPin, INPUT);
  pinMode(centerIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  // Servo signal.
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);

  // Record the default servo position.
  // No servo movement is commanded here.
  currentServoAngle = servoCenterAngle;
}
