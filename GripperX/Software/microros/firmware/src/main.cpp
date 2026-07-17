#include <Arduino.h>
#include "motor_controller.hpp"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>

// Cytron MDD20A — all 4 in PWM+DIR mode
//
// FR/BR rewire switch (Task #19, diagnostic session 2026-07-06):
// FR-PWM is on GPIO5, BR-PWM on GPIO16 — both ESP32 strapping pins.
// GPIO5 is the main suspect for why the FR motor doesn't spin. Target wiring:
// FR → GPIO22, BR → GPIO4 (BR is currently unremarkable, rewire there is not urgent,
// but prepared identically since it can be carried out physically separate from the FR rewire).
//
// ACTIVATE ONLY AFTER PHYSICAL REWIRING ON THE ROBOT ON SITE!
// See docs/FR_MOTOR_REPAIR.md for the sequence. If a define here is set to 1
// without the cable having been physically moved first, the
// firmware drives an unconnected pin — the motor then still won't spin.
#define FR_REWIRE_GPIO22   0   // 0 = FR-PWM on GPIO5 (default/current state) | 1 = GPIO22 (after rewire)
#define BR_REWIRE_GPIO4    0   // 0 = BR-PWM on GPIO16 (default/current state) | 1 = GPIO4 (after rewire)

#define PWM_PIN1      19   // FL
#define DIR_PIN1      13
#define PWM_PIN2      18   // BL
#define DIR_PIN2      21
#if BR_REWIRE_GPIO4
#define PWM_PIN3       4   // BR  (rewired from GPIO16)
#else
#define PWM_PIN3      16   // BR  (strapping pin — rewire to GPIO4 planned)
#endif
#define DIR_PIN3      26
#if FR_REWIRE_GPIO22
#define PWM_PIN4      22   // FR  (rewired from GPIO5)
#else
#define PWM_PIN4       5   // FR  (strapping pin — rewire to GPIO22 planned)
#endif
#define DIR_PIN4      17

#define NUM_JOINTS    8
#define IDX_FL   4
#define IDX_FR   5
#define IDX_BL   6
#define IDX_BR   7

#define STATES_PUBLISH_MS   100
#define CMD_TIMEOUT_MS     1000
#define PING_INTERVAL_MS   1000

// FL and BL physically mirrored → sign negated in cmdCb
MotorController motor_fl(PWM_PIN1, DIR_PIN1);
MotorController motor_bl(PWM_PIN2, DIR_PIN2);
MotorController motor_br(PWM_PIN3, DIR_PIN3);
MotorController motor_fr(PWM_PIN4, DIR_PIN4);

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;

std_msgs__msg__Float64MultiArray cmd_msg;
std_msgs__msg__Float64MultiArray state_msg;
static double cmd_data[NUM_JOINTS];
static double state_data[NUM_JOINTS];

bool ros_ok = false;
bool motors_ok = false;
unsigned long last_cmd_ms = 0;
unsigned long last_ping_ms = 0;

static void initBuffer(std_msgs__msg__Float64MultiArray * msg, double * buf) {
    msg->data.data = buf;
    msg->data.capacity = NUM_JOINTS;
    msg->data.size = NUM_JOINTS;
    msg->layout.dim.data = nullptr;
    msg->layout.dim.capacity = 0;
    msg->layout.dim.size = 0;
    msg->layout.data_offset = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) buf[i] = 0.0;
}

static float radToRpm(float rad) { return rad * (60.0f / (2.0f * PI)); }
static float rpmToRad(float rpm) { return rpm * (2.0f * PI / 60.0f); }

static void stopMotors() {
    motor_fl.setTargetRPM(0);
    motor_bl.setTargetRPM(0);
    motor_br.setTargetRPM(0);
    motor_fr.setTargetRPM(0);
}

void cmdCb(const void * raw) {
    const auto * msg = (const std_msgs__msg__Float64MultiArray *)raw;
    if (msg->data.size < NUM_JOINTS) return;
    motor_fl.setTargetRPM(-radToRpm((float)msg->data.data[IDX_FL]));
    motor_bl.setTargetRPM(-radToRpm((float)msg->data.data[IDX_BL]));
    motor_br.setTargetRPM( radToRpm((float)msg->data.data[IDX_BR]));
    motor_fr.setTargetRPM( radToRpm((float)msg->data.data[IDX_FR]));
    last_cmd_ms = millis();
}

static void destroyEntities() {
    stopMotors();
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&subscriber, &node);
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    ros_ok = false;
    motors_ok = false;
}

static void startMotors() {
    if (motors_ok) return;
    motor_fl.begin(); motor_bl.begin(); motor_br.begin(); motor_fr.begin();
    motor_fl.setFeedForward(0.85f); motor_bl.setFeedForward(0.85f);
    motor_br.setFeedForward(0.85f); motor_fr.setFeedForward(0.85f);
    motors_ok = true;
}

static bool startRos() {
    if (ros_ok) return true;
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "gripperx_firmware", "", &support) != RCL_RET_OK) {
        rclc_support_fini(&support); return false;
    }
    initBuffer(&cmd_msg, cmd_data);
    initBuffer(&state_msg, state_data);
    if (rclc_publisher_init_default(&publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "hw/joint_states") != RCL_RET_OK) {
        rcl_node_fini(&node); rclc_support_fini(&support); return false;
    }
    if (rclc_subscription_init_default(&subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "hw/joint_commands") != RCL_RET_OK) {
        rcl_publisher_fini(&publisher, &node);
        rcl_node_fini(&node); rclc_support_fini(&support); return false;
    }
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        rcl_subscription_fini(&subscriber, &node);
        rcl_publisher_fini(&publisher, &node);
        rcl_node_fini(&node); rclc_support_fini(&support); return false;
    }
    if (rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &cmdCb, ON_NEW_DATA) != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        rcl_subscription_fini(&subscriber, &node);
        rcl_publisher_fini(&publisher, &node);
        rcl_node_fini(&node); rclc_support_fini(&support); return false;
    }
    last_cmd_ms = millis();
    last_ping_ms = millis();
    ros_ok = true;
    startMotors();
    return true;
}

void setup() {
    // Strapping pins and DIR pins LOW early — prevents boot-time motor activation.
    // Deliberately via the PWM_PIN*/DIR_PIN* defines instead of literals, so that when
    // FR_REWIRE_GPIO22/BR_REWIRE_GPIO4 is enabled this automatically captures the
    // correct (new) pin and not the old, now unused one.
    const uint8_t EARLY_PINS[] = {PWM_PIN1, DIR_PIN1, PWM_PIN2, DIR_PIN2,
                                   PWM_PIN3, DIR_PIN3, PWM_PIN4, DIR_PIN4};
    for (auto p : EARLY_PINS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);
}

void loop() {
    if (!ros_ok) {
        // Wait until agent reachable, then connect
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            startRos();
        }
        delay(50);
        return;
    }

    // Periodic ping to detect agent disconnects
    unsigned long now = millis();
    if (now - last_ping_ms > PING_INTERVAL_MS) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
            destroyEntities();
            return;
        }
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Motor timeout: no commands → stop
    if (motors_ok && last_cmd_ms > 0 && (millis() - last_cmd_ms > CMD_TIMEOUT_MS)) {
        stopMotors();
        last_cmd_ms = 0;
    }

    // Publish joint states
    static unsigned long last_pub = 0;
    if (millis() - last_pub < STATES_PUBLISH_MS) return;
    last_pub = millis();
    for (int i = 0; i < NUM_JOINTS; ++i) state_data[i] = 0.0;
    if (motors_ok) {
        state_data[IDX_FL] = rpmToRad(-motor_fl.getRPM());
        state_data[IDX_FR] = rpmToRad( motor_fr.getRPM());
        state_data[IDX_BL] = rpmToRad(-motor_bl.getRPM());
        state_data[IDX_BR] = rpmToRad( motor_br.getRPM());
    }
    state_msg.data.size = NUM_JOINTS;
    rcl_publish(&publisher, &state_msg, nullptr);
}
