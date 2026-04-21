#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// Pin Definitions
// ==========================================

// Ultrasonic Sensor 1 (e.g., Port / Left)
#define PIN_SONAR1_TRIG 14
#define PIN_SONAR1_ECHO 12

// Ultrasonic Sensor 2 (e.g., Starboard / Right)
#define PIN_SONAR2_TRIG 33
#define PIN_SONAR2_ECHO 32

// GPS Module (GY-NEO6MV2 usually defaults to 9600 baud)
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define GPS_BAUD_RATE 9600

// Electronic Speed Controllers (ESCs)
#define PIN_ESC1_PWM 25
#define PIN_ESC2_PWM 26

// ==========================================
// Bluetooth Configuration
// ==========================================
#define BLUETOOTH_DEVICE_NAME "SharX_ESP32"

// ==========================================
// Timing & Execution Settings
// ==========================================

// FreeRTOS Task intervals (in milliseconds)
#define TASK_SONAR_INTERVAL_MS 100
#define TASK_GPS_INTERVAL_MS   2000
#define TASK_MAIN_INTERVAL_MS  50   // How often the boat makes decisions

// ==========================================
// PWM Configuration for ESCs
// ==========================================

// Standard RC ESCs usually expect a 50Hz signal (20ms period)
#define ESC_PWM_FREQ 50
#define ESC_PWM_RESOLUTION 16       // 16-bit for very smooth throttle control

// ESP32 LEDC (PWM) Channels
#define ESC1_PWM_CHANNEL 0
#define ESC2_PWM_CHANNEL 1

#endif // CONFIG_H