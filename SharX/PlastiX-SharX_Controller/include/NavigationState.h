#ifndef NAVIGATION_STATE_H
#define NAVIGATION_STATE_H

#include <Arduino.h>

// This struct acts as the central hub for all sensor data.
// The sensor tasks write to it, and the motor control task reads from it.
struct NavigationState {
    
    // --- Ultrasonic Data ---
    // Distances are stored in centimeters
    float distanceSonar1_CM = -1.0; 
    float distanceSonar2_CM = -1.0; 

    // --- GPS Data ---
    float latitude = 0.0;
    float longitude = 0.0;
    bool  hasFix = false;           // True if the GPS sees enough satellites
    int   satellitesVisible = 0;

    // --- Motor Commands ---
    int targetSpeedLeft = 0;   // Range: -100 to 100
    int targetSpeedRight = 0;  // Range: -100 to 100

    // --- System Status ---
    // Tracking when data was last updated helps the main loop know 
    // if a sensor has frozen or disconnected.
    uint32_t lastSonarUpdateMs = 0;
    uint32_t lastGpsUpdateMs = 0;
};

#endif // NAVIGATION_STATE_H