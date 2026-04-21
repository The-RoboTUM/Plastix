#include <Arduino.h>

#define GPS_RX 25   // ESP32 RX  <- GPS TX
#define GPS_TX 26   // ESP32 TX  -> GPS RX

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("Esperando datos crudos del GPS...");
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);   // Imprime TODO lo que llega
  }
}
