#include <Arduino.h>

const pin_size_t HALL_SENSOR_LEFT = D2;
const pin_size_t HALL_SENSOR_RIGHT = D13;
int revcountleft = 0;
int revcountright = 0;
int rstate = 0;
int lstate = 0;
float total_Distance = 0;