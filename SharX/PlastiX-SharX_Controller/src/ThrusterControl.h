#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include <Arduino.h>

class ThrusterControl {
private:
    uint8_t pwmPin;
    uint8_t pwmChannel;

public:
    // Constructor
    ThrusterControl(uint8_t pin, uint8_t channel);
    
    // Configures the ESP32 hardware timer
    void begin();
    
    // Accepts a speed from -100 (full reverse) to 100 (full forward)
    void setSpeed(int speedPercent); 
};

#endif // THRUSTER_CONTROL_H