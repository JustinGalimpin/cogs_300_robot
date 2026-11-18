/*
 * COGS 300 - Tournament Robot (Labs 5 + 7)
 *
 * All five .ino tabs are compiled and uploaded as ONE sketch.
 * For now, choose the navigation mode below and upload once per test.
 * Automatic transitions and Lab 8 object detection will be added later.
 *
 * IMPORTANT: Only one setup() and loop() exist in the entire sketch.
 */

enum TournamentStage {
  LINE_FOLLOWING,
  WALL_FOLLOWING,
  OBJECT_DETECTION, // Reserved for Lab 8; stops safely for now.
  FINISHED
};

// MANUAL CHANGING OF STAGE CODE FOR THE ROBOT
// We'll eventually make this automatic
// CHANGE THIS VALUE to test either of the existing navigation systems.
TournamentStage currentStage = LINE_FOLLOWING;

void setup() {
  // Motor direction and PWM pins (shared by both navigation modes).
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic sensors (Lab 5).
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  digitalWrite(frontTrigPin, LOW);
  digitalWrite(leftTrigPin, LOW);

  // IR sensors (Lab 7): HIGH means white tape.
  pinMode(leftIRPin, INPUT);
  pinMode(centerIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  Serial.begin(9600);
  setMotors(0, 0);
  Serial.println("COGS 300 tournament robot ready.");
  Serial.println("Select LINE_FOLLOWING or WALL_FOLLOWING in the main tab.");
}

void loop() {
  switch (currentStage) {
    case LINE_FOLLOWING: {
      // Preserve Lab 7's continuous IR updates and 10 ms loop delay.
      updateLineFollowing();

      static unsigned long lastLinePrintTime = 0;
      if (millis() - lastLinePrintTime >= 200) {
        printLineSensorReadings();
        lastLinePrintTime = millis();
      }
      delay(10);
      break;
    }

    case WALL_FOLLOWING: {
      // Preserve Lab 5's approximately 100 ms sensor cycle.
      // Both ultrasonic readings are taken before navigation updates.
      if (millis() - lastSensorTime >= SENSOR_INTERVAL) {
        frontDistance = measureDistance(frontTrigPin, frontEchoPin);
        delay(30); // Original Lab 5 inter-sensor settling delay.
        leftDistance = measureDistance(leftTrigPin, leftEchoPin);

        updateWallNavigation();
        printWallSensorReadings();
        lastSensorTime = millis();
      }
      break;
    }

    case OBJECT_DETECTION:
    case FINISHED:
      // Object detection is not implemented yet. Fail safe by stopping.
      setMotors(0, 0);
      break;
  }
}

// Original Lab 5 Serial Monitor output, extracted from its loop().
void printWallSensorReadings() {
  Serial.print("Front: ");
  if (frontDistance < 0) {
    Serial.print("No echo");
  } else {
    Serial.print(frontDistance);
    Serial.print(" cm");
  }

  Serial.print(" | Left: ");
  if (leftDistance < 0) {
    Serial.print("No echo");
  } else {
    Serial.print(leftDistance);
    Serial.print(" cm");
  }

  Serial.print(" | State: ");
  switch (wallState) {
    case FOLLOW_WALL: Serial.println("FOLLOW_WALL"); break;
    case TURN_RIGHT: Serial.println("TURN_RIGHT"); break;
    case ADVANCE_OUTSIDE: Serial.println("ADVANCE_OUTSIDE"); break;
    case TURN_LEFT_OUTSIDE: Serial.println("TURN_LEFT_OUTSIDE"); break;
    case STOPPED: Serial.println("STOPPED"); break;
  }
}
