#ifndef WHEELENCODERS_H
#define WHEELENCODERS_H

#include <Arduino.h>


/*void countLeftRevs();
void countRightRevs();*/

void lCount();
void rCount();

extern volatile int lTicks, rTicks;
extern const float wheelRadius, wheelCircumference;
extern float lDistance;
extern float rDistance;
extern float mDistance;
extern volatile float currDistance;

//wheelEncoder(uint8_t HALL_SENSOR_LEFT, uint8_t HALL_SENSOR_RIGHT);

#endif 