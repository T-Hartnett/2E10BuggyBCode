#ifndef WHEELENCODERS_H
#define WHEELENCODERS_H

#include <Arduino.h>

//const pin_size_t ENCODER_LEFT = D2;
//const pin_size_t ENCODER_RIGHT = D8;
extern int revcountleft;
extern int revcountright;
extern int rstate;
extern int lstate;
extern float total_Distance;

/*void countLeftRevs();
void countRightRevs();*/

void lCount();
void rCount();

extern volatile int lTicks, rTicks;
extern const float wheelRadius, wheelCircumference;
extern float lDistance;
extern float rDistance;
extern float mDistance;

//wheelEncoder(uint8_t HALL_SENSOR_LEFT, uint8_t HALL_SENSOR_RIGHT);

#endif 