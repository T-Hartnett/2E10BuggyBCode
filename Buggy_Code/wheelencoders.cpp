#include "wheelencoders.h"
#include "config.h"



/*void countLeftRevs() {
    revcountleft++;
}

void countRightRevs() {
    revcountright++;
}*/

volatile int lTicks = 0;
volatile int rTicks = 0;
const int pi = 3.14159;
const float wheelRadius = 3.25;
const float wheelCircumference = 2 * pi * wheelRadius;
float mDistance = 0;
float rDistance = 0;
float lDistance = 0;

void lCount() {
  static bool lastState = LOW;
  bool currentState = digitalRead(ENCODER_LEFT);
  if (lastState == LOW && currentState == HIGH) {
    lTicks++;
  }
  lastState = currentState;
}

void rCount() {
  static bool lastState = LOW;
  bool currentState = digitalRead(ENCODER_RIGHT);
  if (lastState == LOW && currentState == HIGH) {
    rTicks++;
  }
  lastState = currentState;
}

