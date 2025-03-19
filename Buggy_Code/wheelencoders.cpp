#include "wheelencoders.h"
#include "config.h"
//wheelEncoder(uint8_t HALL_SENSOR_LEFT, uint8_t HALL_SENSOR_RIGHT)

int revcountleft = 0;
int revcountright = 0;
int rstate = 0;
int lstate = 0;
float total_Distance = 0;

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

void lCount() {
static bool lastState = LOW;
bool currentState = digitalRead (ENCODER_LEFT);
if (lastState == LOW && currentState == HIGH){
  lTicks++;
}
lastState = currentState;
}

void rCount() {
static bool lastState = LOW;
bool currentState = digitalRead (ENCODER_RIGHT);
if (lastState == LOW && currentState == HIGH){
  rTicks++;
}
lastState = currentState;
}

