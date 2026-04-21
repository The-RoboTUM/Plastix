#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
private:
    uint8_t trigPin;
    uint8_t echoPin;

public:
    // Constructor
    UltrasonicSensor(uint8_t trig, uint8_t echo);
    
    // Initializes the pins
    void begin();
    
    // Triggers a ping and returns the distance in cm. 
    // Returns -1.0 if out of range or disconnected.
    float readDistanceCM();
};

#endif // ULTRASONIC_SENSOR_H