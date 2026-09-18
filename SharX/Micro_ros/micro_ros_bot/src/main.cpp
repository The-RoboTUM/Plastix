#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ Serial.print("ERROR RCCHECK at line "); Serial.println(__LINE__); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ Serial.print("Soft error at line "); Serial.println(__LINE__); }}

void error_loop() {
  Serial.println("Entering ERROR LOOP!");
  while(1) {
    Serial.println("Init FAILED");
    delay(1000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;

  msg.data++;

  Serial.print("Publishing: ");
  Serial.println(msg.data);

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== micro-ROS ESP32 START ===");

  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  Serial.println("Init support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("Support OK");

  Serial.println("Init node...");
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
  Serial.println("Node OK");

  Serial.println("Init publisher...");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));
  Serial.println("Publisher OK");

  Serial.println("Init timer...");
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback));
  Serial.println("Timer OK");

  Serial.println("Init executor...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  Serial.println("Executor OK");

  msg.data = 0;

  Serial.println("=== micro-ROS READY ===");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
  delay(50);
}
