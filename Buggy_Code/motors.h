#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "PIDcontroller.h"  // Ensure this is included

extern PIDController speedPID;  // Declare speedPID from main.cpp
extern int revcountleft, revcountright;  // Declare encoder counts

void moveForward(double speed);
void moveBackward(double speed);
void turnRight(double speed);
void turnLeft(double speed);
void stopMotors();
void handleMotorLogic();
double calculateSpeed();
void setMotorSpeed(double leftSpeed, double rightSpeed);

#endif
