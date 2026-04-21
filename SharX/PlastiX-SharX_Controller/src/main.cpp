#include <Arduino.h>
#include "config.h"
#include "NavigationState.h"
#include "UltrasonicSensor.h"
#include "ThrusterControl.h"
#include "GPSModule.h"
#include "BluetoothManager.h"

// ==========================================
// Global Instances
// ==========================================
NavigationState navState;

UltrasonicSensor sonarLeft(PIN_SONAR1_TRIG, PIN_SONAR1_ECHO);
UltrasonicSensor sonarRight(PIN_SONAR2_TRIG, PIN_SONAR2_ECHO);

ThrusterControl escLeft(PIN_ESC1_PWM, ESC1_PWM_CHANNEL);
ThrusterControl escRight(PIN_ESC2_PWM, ESC2_PWM_CHANNEL);

GPSModule gps(PIN_GPS_RX, PIN_GPS_TX, Serial1); 

// Pass the global navState by reference into the Bluetooth Manager
BluetoothManager btManager(navState);

// ==========================================
// FreeRTOS Task Definitions
// ==========================================

void TaskSonar(void *pvParameters) {
    for (;;) {
        navState.distanceSonar1_CM = sonarLeft.readDistanceCM();
        navState.distanceSonar2_CM = sonarRight.readDistanceCM();
        navState.lastSonarUpdateMs = millis();
        vTaskDelay(pdMS_TO_TICKS(TASK_SONAR_INTERVAL_MS));
    }
}

void TaskGPS(void *pvParameters) {
    uint32_t lastGpsLog = 0;
    for (;;) {
        gps.update(); 
        if (millis() - lastGpsLog >= TASK_GPS_INTERVAL_MS) {
            navState.hasFix = gps.hasFix();
            navState.latitude = gps.getLatitude();
            navState.longitude = gps.getLongitude();
            navState.satellitesVisible = gps.getSatellites();
            navState.lastGpsUpdateMs = millis();
            lastGpsLog = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void TaskBluetooth(void *pvParameters) {
    for (;;) {
        btManager.processIncomingData();
        // Check frequently (every 20ms) for highly responsive remote control
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ==========================================
// Setup & Loop
// ==========================================

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing Model Boat Systems...");

    sonarLeft.begin();
    sonarRight.begin();
    escLeft.begin();
    escRight.begin();
    gps.begin(GPS_BAUD_RATE);
    
    // Initialize Bluetooth
    btManager.begin(BLUETOOTH_DEVICE_NAME);

    // Create FreeRTOS Tasks
    xTaskCreate(TaskSonar,     "SonarTask", 2048, NULL, 2, NULL);
    xTaskCreate(TaskGPS,       "GPSTask",   2048, NULL, 1, NULL);
    xTaskCreate(TaskBluetooth, "BTTask",    4096, NULL, 3, NULL); // Priority 3 (highest) for fast control
    
    Serial.println("Systems Ready. RTOS Tasks Started.");
}

void loop() {
    // 1. Read the desired speeds from the central state (updated by Bluetooth)
    int leftSpeed = navState.targetSpeedLeft;
    int rightSpeed = navState.targetSpeedRight;

    // 2. Safety Override Example (Optional)
    // If an object is extremely close to the left sonar, prevent moving forward on the left side
    // if (navState.distanceSonar1_CM > 0 && navState.distanceSonar1_CM < 20.0) {
    //     if (leftSpeed > 0) leftSpeed = 0; 
    // }

    // 3. Apply the final speeds to the hardware
    escLeft.setSpeed(leftSpeed);
    escRight.setSpeed(rightSpeed);

    // Run the main control loop at 20Hz (every 50ms)
    delay(TASK_MAIN_INTERVAL_MS); 
}