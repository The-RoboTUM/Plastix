#include "GPSModule.h"

GPSModule::GPSModule(uint8_t rx, uint8_t tx, HardwareSerial& serialPort) 
    : rxPin(rx), txPin(tx), gpsSerial(serialPort) {}

void GPSModule::begin(uint32_t baudRate) {
    // Initialize the hardware serial port with the pins defined in config.h
    gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void GPSModule::update() {
    // Read all available bytes from the GPS module and feed them to the parser
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
}

bool GPSModule::hasFix() {
    return gps.location.isValid();
}

float GPSModule::getLatitude() {
    return gps.location.lat();
}

float GPSModule::getLongitude() {
    return gps.location.lng();
}

int GPSModule::getSatellites() {
    return gps.satellites.value();
}