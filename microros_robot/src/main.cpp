#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "motor_controller.hpp"
#include "servo_controller.hpp"
#include "imu_bno085.hpp"

// ---------------------------------------------------------------------------
// Layout of robot/cmd/in (Float32MultiArray), 8 elements:
//   [0..3] drive motor velocities  (FL, FR, RL, RR)  normalized [-1, 1]
//   [4..7] steering servo velocities                 normalized [-1, 1]
// ---------------------------------------------------------------------------

static constexpr uint8_t NUM_CMD_VALUES = NUM_DRIVE_MOTORS + NUM_STEER_SERVOS;

static constexpr uint32_t SAFETY_TIMEOUT_MS = 1000;
static constexpr float CMD_DEADBAND = 0.05f;

volatile float drive_cmd[NUM_DRIVE_MOTORS] = {0};
volatile float steer_cmd[NUM_STEER_SERVOS] = {0};
volatile uint32_t last_cmd_ms = 0;

MotorController motors;
ServoController servos;

// ---------------------------------------------------------------------------
// micro-ROS
// ---------------------------------------------------------------------------

rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray inp_msg;
std_msgs__msg__Float32MultiArray out_msg;
bool microros_initialized = false;

unsigned long last_ping = 0;

void steeringLoop(void *parameter);
void driveLoop(void *parameter);

static bool commandsAlive() {
  return (millis() - last_cmd_ms) < SAFETY_TIMEOUT_MS;
}

static float applyDeadband(float v) {
  if (fabsf(v) < CMD_DEADBAND) {
    return 0.0f;
  }
  return v;
}

void cmd_callback(const void *msg_in) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msg_in;

  if (msg->data.size < NUM_CMD_VALUES) {
    return;
  }

  for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
    float v = msg->data.data[i];
    if (v > 1.0f) {
      v = 1.0f;
    } else if (v < -1.0f) {
      v = -1.0f;
    }
    drive_cmd[i] = v;
  }

  for (uint8_t i = 0; i < NUM_STEER_SERVOS; ++i) {
    float v = msg->data.data[NUM_DRIVE_MOTORS + i];
    if (v > 1.0f) {
      v = 1.0f;
    } else if (v < -1.0f) {
      v = -1.0f;
    }
    steer_cmd[i] = v;
  }

  last_cmd_ms = millis();
}

void setup_micro_ros() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    return;
  }
  if (rclc_node_init_default(&node, "outdoor_swerve_node", "", &support) !=
      RCL_RET_OK) {
    return;
  }

  rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "robot/data_out");

  rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "robot/cmd/in");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &inp_msg, cmd_callback,
                                 ON_NEW_DATA);

  inp_msg.data.capacity = NUM_CMD_VALUES;
  inp_msg.data.size = NUM_CMD_VALUES;
  inp_msg.data.data = static_cast<float *>(malloc(inp_msg.data.capacity * sizeof(float)));

  out_msg.data.capacity = NUM_CMD_VALUES;
  out_msg.data.size = NUM_CMD_VALUES;
  out_msg.data.data = static_cast<float *>(malloc(out_msg.data.capacity * sizeof(float)));

  microros_initialized = true;
}

void deinit_micro_ros() {
  rclc_executor_fini(&executor);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&publisher, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  free(inp_msg.data.data);
  free(out_msg.data.data);
  microros_initialized = false;
}

// ---------------------------------------------------------------------------
// FreeRTOS control loops (same pattern as your VESC + RoboClaw example)
// ---------------------------------------------------------------------------

void steeringLoop(void *parameter) {
  (void)parameter;
  uint32_t last_control_time = 0;

  while (true) {
    const uint32_t now = millis();
    if (now - last_control_time < 50) {
      vTaskDelay(1);
      continue;
    }
    last_control_time = now;

    float local_steer[NUM_STEER_SERVOS];
    if (commandsAlive()) {
      for (uint8_t i = 0; i < NUM_STEER_SERVOS; ++i) {
        local_steer[i] = applyDeadband(steer_cmd[i]);
      }
      servos.applyAll(local_steer, NUM_STEER_SERVOS);
    } else {
      servos.stopAll();
    }
  }
}

void driveLoop(void *parameter) {
  (void)parameter;
  uint32_t last_control_time = 0;

  while (true) {
    const uint32_t now = millis();
    if (now - last_control_time < 20) {
      vTaskDelay(1);
      continue;
    }
    last_control_time = now;

    float local_drive[NUM_DRIVE_MOTORS];
    if (commandsAlive()) {
      for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
        local_drive[i] = drive_cmd[i];
      }
      motors.applyAll(local_drive, NUM_DRIVE_MOTORS);
      motors.update(now);
    } else {
      motors.stopAll();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  motors.begin();
  servos.begin();

  set_microros_ethernet_transports(
      IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0),
      IPAddress(192, 168, 1, 116), 8888, "outdoor-swerve", nullptr);

  if (rmw_uros_ping_agent(500, 3) == RMW_RET_OK) {
    setup_micro_ros();
  }

  xTaskCreatePinnedToCore(steeringLoop, "Steering Control Loop", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(driveLoop, "Drive Control Loop", 4096, NULL, 1, NULL, 1);
}

void loop() {
  if (millis() - last_ping >= 2000) {
    last_ping = millis();
    const bool agent_alive = (rmw_uros_ping_agent(20, 1) == RMW_RET_OK);
    if (agent_alive && !microros_initialized) {
      setup_micro_ros();
    } else if (!agent_alive && microros_initialized) {
      deinit_micro_ros();
    }
  }

  if (microros_initialized) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
      out_msg.data.data[i] = drive_cmd[i];
    }
    for (uint8_t i = 0; i < NUM_STEER_SERVOS; ++i) {
      out_msg.data.data[NUM_DRIVE_MOTORS + i] = steer_cmd[i];
    }
    rcl_publish(&publisher, &out_msg, NULL);
  }
}
