#include <Arduino.h>

// Right side - 2 motors
#define R1_IN1 19
#define R1_IN2 18

#define R2_IN1 14
#define R2_IN2 27

// Left side - 2 motors
#define L1_IN1 25
#define L1_IN2 33

#define L2_IN1 32
#define L2_IN2 26   // GPIO35 no sirve como salida

int last_cmd = 999;

void forward();
void backward();
void stopMotors();

void setup() {
  Serial.begin(115200);

  pinMode(R1_IN1, OUTPUT);
  pinMode(R1_IN2, OUTPUT);
  pinMode(R2_IN1, OUTPUT);
  pinMode(R2_IN2, OUTPUT);

  pinMode(L1_IN1, OUTPUT);
  pinMode(L1_IN2, OUTPUT);
  pinMode(L2_IN1, OUTPUT);
  pinMode(L2_IN2, OUTPUT);

  stopMotors();

  Serial.println("ESP32 ready");
  Serial.println("Commands: 1=FORWARD, -1=BACKWARD, 0=STOP");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int cmd = input.toInt();

    if (cmd == 1) {
      forward();
      Serial.println("FORWARD");
    } 
    else if (cmd == -1) {
      backward();
      Serial.println("BACKWARD");
    } 
    else {
      stopMotors();
      Serial.println("STOP");
    }
  }
}

void forward() {
  // Right motors
  digitalWrite(R1_IN1, HIGH);
  digitalWrite(R1_IN2, LOW);
  digitalWrite(R2_IN1, HIGH);
  digitalWrite(R2_IN2, LOW);

  // Left motors
  digitalWrite(L1_IN1, HIGH);
  digitalWrite(L1_IN2, LOW);
  digitalWrite(L2_IN1, HIGH);
  digitalWrite(L2_IN2, LOW);
}

void backward() {
  // Right motors
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, HIGH);
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, HIGH);

  // Left motors
  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, HIGH);
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, HIGH);
}

void stopMotors() {
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, LOW);
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, LOW);

  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, LOW);
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, LOW);
}