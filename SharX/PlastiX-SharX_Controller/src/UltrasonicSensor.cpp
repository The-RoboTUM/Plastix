#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trig, uint8_t echo) {
    trigPin = trig;
    echoPin = echo;
}

void UltrasonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW); // Ensure trig is low to start
}

float UltrasonicSensor::readDistanceCM() {
    // 1. Send a 10 microsecond pulse to trigger the sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 2. Read the echo pulse. 
    // We use a 30,000 microsecond timeout (roughly 5 meters).
    // If no object is detected within 5m, it returns 0 immediately 
    // rather than locking up the task for a full second.
    long duration = pulseIn(echoPin, HIGH, 30000); 

    // 3. Calculate distance
    if (duration == 0) {
        return -1.0; // Timeout / Out of range
    }
    
    // Speed of sound is 0.034 cm/us. Divide by 2 because the wave travels out and back.
    return (duration * 0.034) / 2.0; 
}