#include <Arduino.h>

#define trigPin 17
#define echoPin 16

#define TIMEOUT_US 30000   // 30 ms
#define MIN_CM 20.0        // zona ciega real
#define MAX_CM 450.0

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delay(500);

  Serial.println("[INIT] JSN-SR04T stable mode");
}

float readDistanceCm() {

  // Trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t us = pulseIn(echoPin, HIGH, TIMEOUT_US);
  if (us == 0) return -1.0;

  float cm = (us / 2.0) / 29.1;

  // Filtrado básico
  if (cm < MIN_CM || cm > MAX_CM) return -1.0;

  return cm;
}

void loop() {

  float cm = readDistanceCm();

  if (cm > 0) {
    Serial.print("[OK] Distance = ");
    Serial.print(cm, 1);
    Serial.println(" cm");
  }

  // ⬅️ CLAVE: NO disparar rápido
  delay(100);
}