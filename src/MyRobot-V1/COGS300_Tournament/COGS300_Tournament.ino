/*
 * COGS 300 - Tournament Robot
 * Labs 5, 7, and 8
 *
 * Main Tournament Controller
 *
 * Navigation systems:
 * - Lab 7: Line Following
 * - Lab 5: Wall Following
 * - Lab 8: Object Detection
 *
 * All .ino tabs are compiled and uploaded
 * together as ONE Arduino sketch.
 *
 * Automatic tournament transitions will
 * be implemented in a future update.
 */

// ==========================================
// TOURNAMENT STAGES
// ==========================================

enum TournamentStage {
  LINE_FOLLOWING,
  WALL_FOLLOWING,
  OBJECT_DETECTION,
  FINISHED
};

// ==========================================
// CURRENT TOURNAMENT STAGE
// ==========================================

// Manually select which navigation system
// to test.
//
// Available options:
// LINE_FOLLOWING
// WALL_FOLLOWING
// OBJECT_DETECTION
// FINISHED
//
// Once automatic transitions are implemented,
// this variable will change during navigation.

TournamentStage currentStage = OBJECT_DETECTION;

// Track the previously initialized stage.
//
// This ensures that initialization functions
// only run when entering a new stage.

TournamentStage previousStage = FINISHED;

// ==========================================
// STAGE INITIALIZATION
// ==========================================

/*
 * Initializes the navigation system associated
 * with the currently selected stage.
 *
 * Called once when the stage changes.
 */

void initializeTournamentStage() {

  // Stop the motors before entering a stage.
  setMotors(0, 0);

  switch (currentStage) {

    // ======================================
    // LINE FOLLOWING
    // ======================================

    case LINE_FOLLOWING:

      Serial.println("Stage: LINE FOLLOWING");

      // Existing Lab 7 controller requires
      // no additional initialization.
      break;

    // ======================================
    // WALL FOLLOWING
    // ======================================

    case WALL_FOLLOWING:

      Serial.println("Stage: WALL FOLLOWING");

      // Existing Lab 5 controller maintains
      // its own wall-navigation state.
      break;

    // ======================================
    // OBJECT DETECTION
    // ======================================

    case OBJECT_DETECTION:

      Serial.println("Stage: OBJECT DETECTION");

      // Initialize the Lab 8 scan and
      // reset its target-detection variables.
      initializeObjectDetection();
      break;

    // ======================================
    // TOURNAMENT FINISHED
    // ======================================

    case FINISHED:

      Serial.println("Stage: FINISHED");

      setMotors(0, 0);
      break;
  }

  // Remember which stage was initialized.
  previousStage = currentStage;
}

// ==========================================
// CHANGE TOURNAMENT STAGE
// ==========================================

/*
 * Safely switches between navigation stages.
 *
 * This function will eventually be called
 * by our automatic transition logic.
 *
 * Example:
 * changeTournamentStage(WALL_FOLLOWING);
 */

void changeTournamentStage(TournamentStage newStage) {

  // Do nothing if already in this stage.
  if (currentStage == newStage) {
    return;
  }

  // Stop before changing navigation modes.
  setMotors(0, 0);

  // Update the active stage.
  currentStage = newStage;

  // Initialize the new navigation system.
  initializeTournamentStage();
}

// ==========================================
// ARDUINO SETUP
// ==========================================

void setup() {

  // Initialize shared motor hardware.
  initializeMotors();

  // Initialize ultrasonic and IR sensors.
  initializeSensors();

  // Start Serial Monitor.
  Serial.begin(9600);

  // Ensure robot starts stationary.
  setMotors(0, 0);

  Serial.println("==========================");
  Serial.println("COGS 300 TOURNAMENT ROBOT");
  Serial.println("==========================");

  Serial.println("Initializing navigation...");

  // Initialize the selected starting stage.
  initializeTournamentStage();

  Serial.println("Robot ready!");
}

// ==========================================
// MAIN TOURNAMENT LOOP
// ==========================================

void loop() {

  // Initialize the controller if the stage
  // was changed directly elsewhere.
  if (currentStage != previousStage) {
    initializeTournamentStage();
  }

  // Run the appropriate navigation system.
  switch (currentStage) {

    // ======================================
    // LAB 7: LINE FOLLOWING
    // ======================================

    case LINE_FOLLOWING: {

      // Execute existing line-following logic.
      updateLineFollowing();

      // Print IR sensor information
      // every 200 milliseconds.
      static unsigned long lastLinePrintTime = 0;

      if (millis() - lastLinePrintTime >= 200) {

        printLineSensorReadings();

        lastLinePrintTime = millis();
      }

      // Preserve Lab 7's polling delay.
      delay(10);

      break;
    }

    // ======================================
    // LAB 5: WALL FOLLOWING
    // ======================================

    case WALL_FOLLOWING:

      // Execute the existing wall-following
      // cycle, including inside and outside
      // corner navigation.
      runWallFollowingCycle();

      break;

    // ======================================
    // LAB 8: OBJECT DETECTION
    // ======================================

    case OBJECT_DETECTION:

      // Execute the object-detection
      // state machine.
      //
      // SCANNING
      // TURN_TO_OBJECT
      // APPROACHING_OBJECT
      // OBJECT_REACHED

      updateObjectDetection();

      // Small polling delay.
      delay(10);

      break;

    // ======================================
    // TOURNAMENT FINISHED
    // ======================================

    case FINISHED:

      // Stop all movement.
      setMotors(0, 0);

      break;
  }
}