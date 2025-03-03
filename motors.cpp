#include "motors.h"
#include "ultrasonic.h"
#include "config.h"
#include <Arduino.h>

// Define IR sensor threshold
const int THRESHOLD = 500;
bool stopped = true;
bool going = false;
bool justChanged = true;


// Function to move forward
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, 170);
  analogWrite(RIGHT_MOTOR_SPEED, 170);
}

// Function to move backward
void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  analogWrite(LEFT_MOTOR_SPEED, 170);
  analogWrite(RIGHT_MOTOR_SPEED, 170);
}

// Function to turn right
void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  analogWrite(LEFT_MOTOR_SPEED, 170);
  analogWrite(RIGHT_MOTOR_SPEED, 150);
}

// Function to turn left
void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, 150);
  analogWrite(RIGHT_MOTOR_SPEED, 170);
}

// Function to stop motors
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, 0);
  analogWrite(RIGHT_MOTOR_SPEED, 0);
}

// Function to handle IR sensor-based movement
void handleMotorLogic() {
    int leftIR = analogRead(IR_SENSOR_LEFT);
    int rightIR = analogRead(IR_SENSOR_RIGHT);

    bool leftDetected = leftIR > THRESHOLD;
    bool rightDetected = rightIR > THRESHOLD;

    if (leftDetected && rightDetected) {
        moveForward();
        if (stopped){
          justChanged = true;
        }
        if (going){
          justChanged = false;
        }
        going = true;
        stopped = false;
    } else if (leftDetected) {
        turnLeft();
    } else if (rightDetected) {
        turnRight();
    } else {
        stopMotors();
        if (going){
          justChanged = true;
        }
        if (stopped){
          justChanged = false;
        }
        going = false;
        stopped = true;
    }
}
