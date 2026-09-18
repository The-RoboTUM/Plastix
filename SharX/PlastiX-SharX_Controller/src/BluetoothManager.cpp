#include "BluetoothManager.h"

BluetoothManager::BluetoothManager(NavigationState& navState) : stateRef(navState) {}

void BluetoothManager::begin(const char* deviceName) {
    SerialBT.begin(deviceName);
}

void BluetoothManager::processIncomingData() {
    if (SerialBT.available()) {
        // Read the incoming text until a newline character is received
        String incomingString = SerialBT.readStringUntil('\n');
        
        // Remove any carriage returns or extra spaces
        incomingString.trim(); 
        
        if (incomingString.length() > 0) {
            parseCommand(incomingString);
        }
    }
}

void BluetoothManager::parseCommand(String command) {
    String action = command;
    int value = 0;
    bool hasValue = false;

    // Check if there is a space separating a command from a value
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
        action = command.substring(0, spaceIndex);
        value = command.substring(spaceIndex + 1).toInt();
        hasValue = true;
    }

    // ==========================================
    // WRITE Commands (Require a value)
    // ==========================================
    if (action == "set_speed_left" && hasValue) {
        value = constrain(value, -100, 100);
        stateRef.targetSpeedLeft = value;
        SerialBT.printf("Left speed set to: %d\n", value);
    } 
    else if (action == "set_speed_right" && hasValue) {
        value = constrain(value, -100, 100);
        stateRef.targetSpeedRight = value;
        SerialBT.printf("Right speed set to: %d\n", value);
    }

    // ==========================================
    // ACTION Commands (No value required)
    // ==========================================
    else if (action == "stop") {
        stateRef.targetSpeedLeft = 0;
        stateRef.targetSpeedRight = 0;
        SerialBT.println("FULL STOP: Both thrusters halted.");
    }

    // ==========================================
    // READ Commands (No value required)
    // ==========================================
    else if (action == "get_sonar") {
        // We use %.1f to print the floats with 1 decimal place
        SerialBT.printf("Sonar Left: %.1f cm | Sonar Right: %.1f cm\n", 
                        stateRef.distanceSonar1_CM, 
                        stateRef.distanceSonar2_CM);
    }
    else if (action == "get_gps") {
        if (stateRef.hasFix) {
            SerialBT.printf("GPS Fix: YES | Lat: %.6f | Lon: %.6f | Sats: %d\n", 
                            stateRef.latitude, 
                            stateRef.longitude, 
                            stateRef.satellitesVisible);
        } else {
            // Note: If you test this indoors, it will almost certainly say NO FIX
            SerialBT.printf("GPS Fix: NO | Sats Visible: %d\n", 
                            stateRef.satellitesVisible);
        }
    }
    else if (action == "get_status") {
        // A handy all-in-one diagnostic command
        SerialBT.printf("Motors [L:%d R:%d] | Sonar [L:%.1f R:%.1f] | GPS Sats: %d\n",
                        stateRef.targetSpeedLeft, stateRef.targetSpeedRight,
                        stateRef.distanceSonar1_CM, stateRef.distanceSonar2_CM,
                        stateRef.satellitesVisible);
    }
    
    // ==========================================
    // Error Handling
    // ==========================================
    else {
        SerialBT.println("--- Error: Unknown command or missing value ---");
        SerialBT.println("Available Commands:");
        SerialBT.println("  set_speed_left <val>");
        SerialBT.println("  set_speed_right <val>");
        SerialBT.println("  stop");
        SerialBT.println("  get_sonar");
        SerialBT.println("  get_gps");
        SerialBT.println("  get_status");
    }
}