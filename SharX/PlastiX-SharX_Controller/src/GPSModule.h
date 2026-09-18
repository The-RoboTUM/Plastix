#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPSModule {
private:
    uint8_t rxPin;
    uint8_t txPin;
    HardwareSerial& gpsSerial;
    TinyGPSPlus gps;

public:
    // We pass in the HardwareSerial port we want to use (e.g., Serial1)
    GPSModule(uint8_t rx, uint8_t tx, HardwareSerial& serialPort);
    
    void begin(uint32_t baudRate);
    
    // Feeds incoming serial data into the TinyGPSPlus parser
    void update(); 
    
    // Getters for parsed data
    bool hasFix();
    float getLatitude();
    float getLongitude();
    int getSatellites();
};

#endif // GPS_MODULE_H