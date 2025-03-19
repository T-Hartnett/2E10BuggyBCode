#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    
    void trigger();
    unsigned long getDistance(); // Returns distance in cm
    static void ISR_Echo(); // Interrupt Service Routine

private:
    uint8_t trigPin;
    uint8_t echoPin;
    static UltrasonicSensor* instance; // Static pointer to the current instance

    volatile unsigned long startTime;
    volatile unsigned long echoDuration;
};

#endif
