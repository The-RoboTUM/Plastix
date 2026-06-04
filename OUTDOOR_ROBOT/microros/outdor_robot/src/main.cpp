#include <Arduino.h>
#include "motor_controller.hpp"

#define PWM_PIN1      18
#define DIR_PIN1      21
#define ENCODER_PIN1  23

#define PWM_PIN2      36
#define DIR_PIN2      21
#define ENCODER_PIN2  27

MotorController motor1(0, PWM_PIN1, DIR_PIN1, ENCODER_PIN1);
MotorController motor2(1, PWM_PIN2, DIR_PIN2, ENCODER_PIN2);

void setup() {
    Serial.begin(115200);

    motor1.begin();
    motor1.setPID(0.5, 0.35, 0.0);
    motor1.setTargetRPM(60.0);

    motor2.begin();
    motor2.setPID(0.5, 0.35, 0.0);
    motor2.setTargetRPM(60.0);
}

void loop() {
    motor1.update();
    motor2.update();
}