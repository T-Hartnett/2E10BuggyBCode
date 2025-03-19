#include "ultrasonic.h"

UltrasonicSensor* UltrasonicSensor::instance = nullptr; // Initialize static instance pointer

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin) 
    : trigPin(trigPin), echoPin(echoPin), startTime(0), echoDuration(0) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    instance = this; // Set the static instance to this object

    attachInterrupt(digitalPinToInterrupt(echoPin), ISR_Echo, CHANGE);
}

void UltrasonicSensor::trigger() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

void UltrasonicSensor::ISR_Echo() {
    if (instance) { // Ensure instance exists
        if (digitalRead(instance->echoPin) == HIGH) {
            instance->startTime = micros();
        } else {
            instance->echoDuration = micros() - instance->startTime;
        }
    }
}

unsigned long UltrasonicSensor::getDistance() {
    return echoDuration * 0.034 / 2; // Convert to cm
}


