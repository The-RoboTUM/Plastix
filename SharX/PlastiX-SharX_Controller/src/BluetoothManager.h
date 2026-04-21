#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "NavigationState.h"

class BluetoothManager {
private:
    BluetoothSerial SerialBT;
    NavigationState& stateRef; // Reference to our central data hub

    void parseCommand(String command);

public:
    // Constructor requires a reference to the global NavigationState
    BluetoothManager(NavigationState& navState);
    
    void begin(const char* deviceName);
    
    // Checks for incoming data and updates NavigationState if a command is found
    void processIncomingData(); 
};

#endif // BLUETOOTH_MANAGER_H