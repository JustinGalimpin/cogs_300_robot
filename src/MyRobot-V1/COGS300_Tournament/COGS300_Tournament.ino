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

// CHANGE THIS VALUE to test either of the existing navigation systems.
TournamentStage currentStage = LINE_FOLLOWING;

void setup() {
  // Hardware setup lives with the shared motor/sensor implementations.
  initializeMotors();
  initializeSensors();
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

    case WALL_FOLLOWING:
      // Lab 5's original timed sensor cycle is encapsulated in its own tab.
      runWallFollowingCycle();
      break;

    case OBJECT_DETECTION:
    case FINISHED:
      // Object detection is not implemented yet. Fail safe by stopping.
      setMotors(0, 0);
      break;
  }
}

