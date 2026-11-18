// Shared motor pins and original Lab 5 / Lab 7 motor control.

// Motor A (Left Wheel)
const int enA = 5;
const int in1 = 6;
const int in2 = 7;

// Motor B (Right Wheel)
const int enB = 9;
const int in3 = 10;
const int in4 = 11;

// ==========================================

/*
 * Controls both motors independently.
 *
 * Positive speed = forward
 * Negative speed = backward
 * Zero = stop
 *
 * Motor directions match our existing robot.
 */

void setMotors(int leftSpeed, int rightSpeed) {

  // Restrict speeds to valid PWM values.
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // ------------------------------------------
  // LEFT MOTOR
  // ------------------------------------------

  if (leftSpeed > 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

  }
  else if (leftSpeed < 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

  }
  else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // ------------------------------------------
  // RIGHT MOTOR
  // ------------------------------------------

  if (rightSpeed > 0) {

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

  }
  else if (rightSpeed < 0) {

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

  }
  else {

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Apply PWM speeds.
  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}

// ==========================================
