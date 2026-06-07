#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <SCServo.h>

// Use the official library class for the ST3215
SMS_STS st;
const uint8_t SERVO_ID = 1; // Change to your confirmed servo ID

// micro-ROS entity structures
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2 // Onboard LED pin for most ESP32 Dev modules

// Error handling loop macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
    while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

// Subscription Callback: Executes every time a ROS 2 message arrives
void subscription_callback(const void * msvgin) {  
    const std_msgs__msg__Int32 * incoming_msg = (const std_msgs__msg__Int32 *)msvgin;
    
    // Constrain input safely within the ST3215 encoder bounds (0 to 4095)
    uint16_t target_position = constrain(incoming_msg->data, 0, 4095);
    
    // Drive the servo via Serial2! (ID, Position, Speed, Acceleration)
    st.WritePosEx(SERVO_ID, target_position, SERVO_MAX_SPEED/2, SERVO_ACCELERATION);
    
    // Toggle LED to visually acknowledge message arrival
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
    // 1. Initialize the Waveshare Servo on Hardware Serial 2
    // Pins: RX2 = GPIO16, TX2 = GPIO17
    Serial2.begin(SERVO_UART_BAUD, SERIAL_8N1, SERVO_UART_RX_PIN, SERVO_UART_TX_PIN);
    st.pSerial = &Serial2;

    // 2. Initialize micro-ROS transport layer via the default USB Serial
    set_microros_transports();
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  
    delay(2000);

    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create ROS 2 Node named "esp32_servo_node"
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

    // Create Subscriber for topic "/servo_cmd"
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_cmd"
    ));

    // Create micro-ROS Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
    // Spin the executor to handle incoming subscriber callbacks smoothly
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}