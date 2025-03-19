#include "motors.h"
#include "ultrasonic.h"
#include "config.h"
#include <Arduino.h>

// Define IR sensor threshold
const int THRESHOLD = 500;
bool stopped = true;
bool going = false;
bool justChanged = true;


void moveForward(double speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  setMotorSpeed(speed, speed);
}

void moveBackward(double speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  setMotorSpeed(speed, speed);
}

void turnRight(double speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  setMotorSpeed(speed, speed * 0.9);  // Reduce right motor speed
}

void turnLeft(double speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  setMotorSpeed(speed * 0.9, speed);  // Reduce left motor speed
}

void stopMotors(){
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, 0);
  analogWrite(RIGHT_MOTOR_SPEED, 0);
}




// Function to handle IR sensor-based movement
/*void handleMotorLogic() {
    int leftIR = analogRead(IR_SENSOR_LEFT);
    int rightIR = analogRead(IR_SENSOR_RIGHT);

    bool leftDetected = leftIR > THRESHOLD;
    bool rightDetected = rightIR > THRESHOLD;

    double targetSpeed = 150;  // Base speed
    double actualSpeed = calculateSpeed();  // Get current speed from encoders
    double newSpeed = speedPID.compute(actualSpeed, targetSpeed);  // Adjust using PID

    Serial.print("Left IR: "); Serial.print(leftIR);
    Serial.print(" Right IR: "); Serial.println(rightIR);

    if (leftDetected && rightDetected) {
        Serial.println("Moving forward");
        moveForward(newSpeed);  
    } else if (leftDetected) {
        Serial.println("Turning left");
        turnLeft(newSpeed);  
    } else if (rightDetected) {
        Serial.println("Turning right");
        turnRight(newSpeed);  
    } else {
        Serial.println("Stopping");
        stopMotors();
    }
}*/

void handleMotorLogic() {
  Serial.println("handle motor logic is called");
    int leftIR = analogRead(IR_SENSOR_LEFT);
    int rightIR = analogRead(IR_SENSOR_RIGHT);

    bool leftDetected = leftIR > THRESHOLD;
    bool rightDetected = rightIR > THRESHOLD;

    double targetSpeed = 170;  // Base target speed
    double actualSpeed = calculateSpeed();  // Get current speed from encoders
    double newSpeed = speedPID.compute(actualSpeed, targetSpeed);  // Adjust speed

    if (leftDetected && rightDetected) {
        moveForward(newSpeed);
        justChanged = (stopped);
        going = true;
        stopped = false;
    } else if (leftDetected) {
        turnLeft(newSpeed);
    } else if (rightDetected) {
        turnRight(newSpeed);
    } else {
        stopMotors();
        justChanged = (going);
        going = false;
        stopped = true;
    }
}

double calculateSpeed() {
  Serial.println("calculate speed gets called");
    static unsigned long lastTime = millis();
    static double lastDistance = 0;

    unsigned long currentTime = millis();
    double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

    if (deltaTime == 0) {
        Serial.println("ERROR: deltaTime is 0! Avoiding division by zero.");
        return 0;
    }

    double distanceTraveled = (((revcountleft + revcountright) / 2)) * PI * 5; // cm

    double speed = (distanceTraveled - lastDistance) / deltaTime; // cm/s

    Serial.print("Delta Time: "); Serial.println(deltaTime);
    Serial.print("Distance Traveled: "); Serial.println(distanceTraveled);
    Serial.print("Speed: "); Serial.println(speed);

    lastTime = currentTime;
    lastDistance = distanceTraveled;

    return speed;
}


void setMotorSpeed(double leftSpeed, double rightSpeed) {
  Serial.println("set motor speed is called");
    if (isnan(leftSpeed) || isnan(rightSpeed)) {
        Serial.println("ERROR: NaN detected in motor speeds! Fixing...");
        leftSpeed = 0;
        rightSpeed = 0;
    }

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    Serial.print("Setting motor speeds - Left: ");
    Serial.print(leftSpeed);
    Serial.print(", Right: ");
    Serial.println(rightSpeed);

    analogWrite(LEFT_MOTOR_SPEED, leftSpeed);
    analogWrite(RIGHT_MOTOR_SPEED, rightSpeed);
}

/*
void setMotorSpeed(double leftSpeed, double rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);  // Ensure within motor range
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(LEFT_MOTOR_SPEED, leftSpeed);
  analogWrite(RIGHT_MOTOR_SPEED, rightSpeed);
}
*/
