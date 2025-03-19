#include "PIDcontroller.h"
#include <Arduino.h>

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0), prevTime(millis()) {}

/*double PIDController::compute(double input, double setpoint) {
    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - prevTime) / 1000.0; // Convert ms to seconds

    double error = setpoint - input;
    integral += error * elapsedTime;
    double derivative = (error - prevError) / elapsedTime;

    double output = (kp * error) + (ki * integral) + (kd * derivative);

    prevError = error;
    prevTime = currentTime;

    // Constrain output to motor limits (0-255)
    output = constrain(output, 0, 255);
    return output;
}*/

double PIDController::compute(double input, double setpoint) {
  Serial.println("Compute is called");
    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - prevTime) / 1000.0; // Convert ms to sec

    if (elapsedTime == 0) {
        Serial.println("ERROR: elapsedTime is 0 in PID compute!");
        return 0;
    }

    double error = setpoint - input;
    integral += error * elapsedTime;
    double derivative = (error - prevError) / elapsedTime;

    double output = (kp * error) + (ki * integral) + (kd * derivative);

    Serial.print("PID Output: ");
    Serial.println(output);
    Serial.println(error);
    Serial.println(integral);
    Serial.println(derivative);

    prevError = error;
    prevTime = currentTime;

    return constrain(output, 0, 255);
}

void PIDController::setTunings(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::reset() {
    prevError = 0;
    integral = 0;
}


