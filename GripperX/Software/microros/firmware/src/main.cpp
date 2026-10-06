#include <Arduino.h>
#include "motor_controller.hpp"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>

// Cytron MDD10A drivers — all 4 channels in PWM+DIR mode.
// Board: YD-ESP32-S3-N16R8 (ESP32-S3-WROOM-1). Pin map = WIRING_PLAN.md §1.
// None of these GPIOs is a strapping / flash / octal-PSRAM pin on the S3-N16R8.
#define PWM_PIN1       4   // FL PWM (IO4)
#define DIR_PIN1       5   // FL DIR (IO5)
#define PWM_PIN2      15   // BL PWM (IO15)
#define DIR_PIN2      16   // BL DIR (IO16)
#define PWM_PIN3      17   // BR PWM (IO17)
#define DIR_PIN3      18   // BR DIR (IO18)
#define PWM_PIN4       6   // FR PWM (IO6)
#define DIR_PIN4       7   // FR DIR (IO7)

// Quadrature-encoder A/B primaries (WIRING_PLAN.md §1.1 / §5). 3.3 V push-pull
// Hall, no pull-ups; each encoder is decoded x4 by one HW PCNT unit (the S3 has
// exactly 4 units). PCNT reaches any GPIO via the matrix, so these are free.
#define ENC_FL_A       8   // IO8
#define ENC_FL_B       9   // IO9
#define ENC_FR_A      10   // IO10
#define ENC_FR_B      11   // IO11
#define ENC_BL_A      12   // IO12
#define ENC_BL_B      13   // IO13
#define ENC_BR_A      14   // IO14
#define ENC_BR_B      21   // IO21

// Encoder sign per wheel, in the MOTOR COMMAND frame of that MotorController: a positive
// setTargetRPM() must read back as a positive getRPM(). This is a pure wiring fact (A/B
// order and motor polarity at the driver header) and can only be settled at the bench:
//   1. command a positive RPM for that wheel
//   2. echo hw/joint_states and look at that wheel's velocity index (4..7)
//   3. sign matches  -> leave at +1
//      sign inverted -> set to -1
// Measured on the assembled robot: all four = +1.
//
// NOT the same thing as the FL/BL robot-frame mirroring in cmdCb()/the publish loop —
// that one converts between the motor frame and the robot frame and stays as it is. Do
// not copy one into the other.
//
// EVERY statement above is about the MOTOR frame only, i.e. it relates a motor command
// to that motor's own encoder — it says nothing about which way the ROBOT then moves.
// The robot-frame direction is a separate, later fact, held in ROBOT_FRAME_WHEEL_SIGN
// below; the two are independent and must not be folded into one another.
#define ENC_DIR_FL     1
#define ENC_DIR_FR     1
#define ENC_DIR_BL     1
#define ENC_DIR_BR     1

// Global ROBOT-frame <-> MOTOR-frame sign for the wheel drives.
//
// Observed on the jacked-up robot: a commanded positive linear.x produced a clean,
// uniform command chain (all four wheels within a few percent of each other) but all
// four wheels physically turned BACKWARD. Uniform across all four, so this is a global
// frame error, NOT a per-side one — the FL/BL mirroring in cmdCb()/the publish loop is
// correct and stays untouched. Hence the factor is -1.
//
// The fault was invisible before because command and feedback carried the same
// mirroring, so encoders read POSITIVE while the wheels ran backward — self-consistent,
// and contradicted by nothing but the wheels. That consistency is the property to
// protect: command and feedback MUST carry this factor IDENTICALLY. Applying it on one
// side only would give correct driving with odometry counting backward — strictly worse
// than the unfixed state. If it is ever changed, change it in cmdCb() and in the publish
// loop (velocities AND positions) together, or not at all.
#define ROBOT_FRAME_WHEEL_SIGN  (-1.0f)

// hw/joint_commands: 8 values [4 steer positions, 4 wheel velocities].
#define NUM_CMD_JOINTS    8
// hw/joint_states: 16 values [4 RESERVED steering slots, 4 wheel velocities,
//   4 wheel positions, 4 wheel-feedback PROVENANCE codes].
//   Indices 0..3 are RESERVED AND ALWAYS ZERO from this firmware — structurally, not a
//   gap to fill later: the steering servos hang on the Pi's USB bus, the ESP32 has
//   neither a steering sensor nor any steering input, so it has nothing to measure. The
//   publish loop zeroes the array and writes only 4..11. The only value this firmware
//   could place in 0..3 is an echo of the commands it receives, and an echo is exactly
//   what FR-2 (functional requirement 2) rejects as feedback. Do NOT "fix" this here —
//   the real steering measurement is /hw/steer_states, published by steer_servo_node on
//   the Pi and merged into the steering position state interfaces by
//   gripperx_hardware_interface (FR-10).
//   Indices 4..7 keep the existing contract gripperx_hardware_interface::read() consumes
//   on the Pi (size check is >= 8, extra values ignored). 8..11 is real encoder-position
//   feedback for read() to adopt (HWR-10, hardware rework requirement 10 — odometry
//   integration).
//   Indices 12..15 are the PROVENANCE of 4..7 and 8..11, one EncoderStatus code per wheel
//   in FL, FR, BL, BR order (FR-11 items 5/6, deviation D14): the velocity is either a
//   measurement or a verbatim echo of the command, and this is the only thing that says
//   which. Codes are defined in motor_controller.hpp (EncoderStatus) and mirrored on the
//   Pi in gripperx_interface.cpp. Anything appended must go AFTER 15 — the Pi keys its
//   length guards on 8 / 12 / 16 and a shorter message reads as "provenance unknown",
//   never as "valid".
#define NUM_STATE_JOINTS 16
#define IDX_FL   4
#define IDX_FR   5
#define IDX_BL   6
#define IDX_BR   7
#define IDX_FL_POS   8
#define IDX_FR_POS   9
#define IDX_BL_POS  10
#define IDX_BR_POS  11
#define IDX_FL_ENC  12
#define IDX_FR_ENC  13
#define IDX_BL_ENC  14
#define IDX_BR_ENC  15

// State publish period. 30 Hz, matching controller_manager update_rate on the Pi
// (gripperx_control/config/ros2_controllers.yaml L3), so GripperXInterface::read()
// sees a fresh frame per control cycle instead of one in three or four.
//
// Scheduled on micros() with a fixed phase (see loop()), so the executor's sleep
// cannot drift the period; EXEC_SPIN_MS bounds that sleep well under it.
//
// UART link budget at 115200 8N1: this rate plus the command traffic (Pi -> ESP32,
// unchanged) and the 1 Hz ping use about 75 % of the link in the pessimistic
// (shared-link) reading — a UART is actually full duplex, so the real margin is
// larger, not smaller. 40 Hz would already exceed the shared-link budget. Margin is
// kept wide on purpose because one item is not quantified: the publisher QoS is
// RELIABLE (rclc_publisher_init_default), so every frame is subject to XRCE-level
// (micro-ROS client-server protocol) acknowledgement whose byte cost has never been
// measured here. Raising the baud rate is the real headroom, but it is a COORDINATED
// change — firmware and gripperx-agent.sh (-b 115200) must move together.
#define STATES_PUBLISH_US 33333
// Upper bound on how long the executor may sit in rcl_wait with nothing to do. This
// is NOT a per-message cost: spin_some returns as soon as a command is ready, so
// lowering it cannot delay cmdCb — it only stops the loop from sleeping past the
// publish deadline and past the encoder sampling interval. Command latency is not
// made worse by lowering it: a command already pending when spin_some is entered
// returns from rcl_wait immediately.
#define EXEC_SPIN_MS          5
#define CMD_TIMEOUT_MS     1000
// PING_INTERVAL_MS, the 200 ms rmw_uros_ping_agent() timeout in loop(), and the
// 100 ms spin_some() slice together bound how long this firmware needs to NOTICE
// that the agent is gone: 1000 + 200 + 100 = 1300 ms worst case. The Pi depends on
// that number. HWR-40 / SR-12 (hardware rework requirement 40 / safety requirement
// 12) chose the clean-shutdown path where the Pi does NOT ask for a teardown over a
// dedicated interface — it stops the micro-ROS agent and relies on the ping failure
// below to call destroyEntities() here. Its restart path waits a 1.5 s dwell before
// bringing the agent back, because startRos() opens with `if (ros_ok) return true;`
// -- an agent that reappears before this firmware has noticed the loss leaves a
// stale session that still LOOKS healthy while nothing is delivered.
// Raising PING_INTERVAL_MS or the ping timeout, or changing the reconnect logic,
// breaks that dwell SILENTLY on the Pi side. Change them only together with the
// dwell in the shutdown path.
#define PING_INTERVAL_MS   1000

// DDS domain of the micro-ROS participant. The XRCE client dictates the domain in
// the participant-creation request — the agent's ROS_DOMAIN_ID does NOT override it,
// so this value MUST match ROS_DOMAIN_ID on the Pi. Project rule (SR-8): real robot
// = 20, digital twin = 220. Without this, rclc_support_init() would default to 0 and
// hw/joint_states + hw/joint_commands stay invisible to every node on the robot.
#define ROS_DOMAIN_ID        20

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
static double cmd_data[NUM_CMD_JOINTS];
static double state_data[NUM_STATE_JOINTS];

bool ros_ok = false;
bool motors_ok = false;
unsigned long last_cmd_ms = 0;
unsigned long last_ping_ms = 0;

static void initBuffer(std_msgs__msg__Float64MultiArray * msg, double * buf, size_t n) {
    msg->data.data = buf;
    msg->data.capacity = n;
    msg->data.size = n;
    msg->layout.dim.data = nullptr;
    msg->layout.dim.capacity = 0;
    msg->layout.dim.size = 0;
    msg->layout.data_offset = 0;
    for (size_t i = 0; i < n; ++i) buf[i] = 0.0;
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
    if (msg->data.size < NUM_CMD_JOINTS) return;
    // ROBOT_FRAME_WHEEL_SIGN converts the incoming robot-frame velocity into the motor
    // frame; the leading -/+ per wheel is the unrelated FL/BL mirroring (see above).
    motor_fl.setTargetRPM(ROBOT_FRAME_WHEEL_SIGN * -radToRpm((float)msg->data.data[IDX_FL]));
    motor_bl.setTargetRPM(ROBOT_FRAME_WHEEL_SIGN * -radToRpm((float)msg->data.data[IDX_BL]));
    motor_br.setTargetRPM(ROBOT_FRAME_WHEEL_SIGN *  radToRpm((float)msg->data.data[IDX_BR]));
    motor_fr.setTargetRPM(ROBOT_FRAME_WHEEL_SIGN *  radToRpm((float)msg->data.data[IDX_FR]));
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
    // One PCNT unit per wheel (S3 units 0..3). dirSign passed explicitly from the
    // ENC_DIR_* defines above — BENCH-CONFIRM each one, see the procedure there.
    motor_fl.attachEncoder(PCNT_UNIT_0, ENC_FL_A, ENC_FL_B, ENC_DIR_FL);
    motor_fr.attachEncoder(PCNT_UNIT_1, ENC_FR_A, ENC_FR_B, ENC_DIR_FR);
    motor_bl.attachEncoder(PCNT_UNIT_2, ENC_BL_A, ENC_BL_B, ENC_DIR_BL);
    motor_br.attachEncoder(PCNT_UNIT_3, ENC_BR_A, ENC_BR_B, ENC_DIR_BR);
    motor_fl.begin(); motor_bl.begin(); motor_br.begin(); motor_fr.begin();
    // ---- Open-loop drive feedforward: pwm = FF_OFFSET_PWM + FF_GAIN * |rpm| --------
    // The law, its derivation, the deadband and the surface caveat are documented once,
    // in motor_controller.hpp. Only the two NUMBERS live here. They are ONE calibration
    // in two halves - never change one without re-deriving the other.
    //
    // FF_GAIN (b): PWM counts per output-shaft RPM. Calibrated on the unloaded robot
    // (wheels on blocks, turning freely in the air) so that commanded and measured
    // speed agree to within about 5 % across all four wheels.
    //
    // FF_OFFSET_PWM (a): PWM counts, calibrated on carpet -- the load case the gain
    // above is never checked against. The offset compensates that surface's rolling
    // resistance; it is a FIRST-ORDER COMPENSATION DERIVED FROM A SPEED DEFICIT VIA
    // FF_GAIN, NOT A TORQUE MEASUREMENT -- no current, no torque, no motor constant was
    // measured. TO-VERIFY on hardware and has never been flashed.
    //
    // CONSEQUENCES, recorded so nobody has to rediscover them:
    //  - CARPET ONLY. Unloaded the feedforward is already within about -5 %, so there
    //    the offset is almost pure overspeed. On grass or gravel it will be too small,
    //    on smooth floor too large, by whatever the rolling-resistance difference is.
    //  - It does NOT make the feedforward correct. It makes it correct AT ONE LOAD. The
    //    remainder as the load varies is what a future regulator (FR-11, functional
    //    requirement 11) is for; none is built here, and this stays open-loop (NFR-10,
    //    non-functional requirement 10, acceptance item 10).
    //  - SATURATION / HEADROOM: at max_wheel_angular_speed 12.0 rad/s
    //    (ros2_controllers.yaml) the output is 140 of 255 (55 % duty); headroom
    //    remains, the clamp is not reachable from any legal wheel command.
    //  - THE MINIMUM COMMANDED SPEED THAT PRODUCES MOTION GOES DOWN. Below the deadband
    //    nothing changes, but immediately above it the output steps straight to the
    //    carpet-rolling-resistance duty instead of ramping from 0: small commands that
    //    used to be swallowed by stiction now move the robot, and on a lower-resistance
    //    surface the same step is a larger overspeed.
    static constexpr float FF_GAIN       = 1.0625f;   // b, PWM counts per RPM
    static constexpr float FF_OFFSET_PWM = 19.0f;     // a, PWM counts (CARPET, TO-VERIFY)
    motor_fl.setFeedForward(FF_GAIN); motor_fl.setFeedForwardOffset(FF_OFFSET_PWM);
    motor_bl.setFeedForward(FF_GAIN); motor_bl.setFeedForwardOffset(FF_OFFSET_PWM);
    motor_br.setFeedForward(FF_GAIN); motor_br.setFeedForwardOffset(FF_OFFSET_PWM);
    motor_fr.setFeedForward(FF_GAIN); motor_fr.setFeedForwardOffset(FF_OFFSET_PWM);
    motors_ok = true;
}

static bool startRos() {
    if (ros_ok) return true;
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK) {
        (void)rcl_init_options_fini(&init_options); return false;
    }
    if (rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator) != RCL_RET_OK) {
        (void)rcl_init_options_fini(&init_options); return false;
    }
    (void)rcl_init_options_fini(&init_options);
    if (rclc_node_init_default(&node, "gripperx_firmware", "", &support) != RCL_RET_OK) {
        rclc_support_fini(&support); return false;
    }
    initBuffer(&cmd_msg, cmd_data, NUM_CMD_JOINTS);
    initBuffer(&state_msg, state_data, NUM_STATE_JOINTS);
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
    // All PWM and DIR pins driven OUTPUT/LOW early — prevents boot-time motor activation.
    // Built from the PWM_PIN*/DIR_PIN* defines instead of literals, so the early-LOW set
    // always tracks the active pin map (WIRING_PLAN.md §1) with no separate list to update.
    const uint8_t EARLY_PINS[] = {PWM_PIN1, DIR_PIN1, PWM_PIN2, DIR_PIN2,
                                   PWM_PIN3, DIR_PIN3, PWM_PIN4, DIR_PIN4};
    for (auto p : EARLY_PINS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    // TX ring buffer ahead of begin(): without one, HardwareSerial::write() blocks
    // until the bytes fit in the 128-byte UART FIFO, so publishing a ~176-byte state
    // frame stalls loop() for ~4 ms every cycle - time in which no command is
    // serviced and no encoder sample is taken. 1024 B holds several frames.
    Serial.setTxBufferSize(1024);
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

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXEC_SPIN_MS));

    // Encoder sampling is DECOUPLED from publishing (motor_controller.hpp): it runs on
    // every loop iteration, self-throttled to ENC_SAMPLE_INTERVAL_US; the publish block
    // below only reads the result. Do not sample inside the publish block instead --
    // that ties each measurement to the jittery publish interval rather than a steady one.
    if (motors_ok) {
        motor_fl.sampleEncoder(); motor_fr.sampleEncoder();
        motor_bl.sampleEncoder(); motor_br.sampleEncoder();
    }

    // Motor timeout: no commands → stop
    if (motors_ok && last_cmd_ms > 0 && (millis() - last_cmd_ms > CMD_TIMEOUT_MS)) {
        stopMotors();
        last_cmd_ms = 0;
    }

    // Publish joint states on a FIXED phase: the deadline advances by exactly one
    // period rather than being re-based on the current time, so the publish interval
    // does not accumulate the loop's overshoot. If a cycle is missed entirely the
    // phase is re-based instead of catching up, so a stall can never produce a burst
    // of frames onto a link that is already the binding constraint. The signed
    // comparison is wrap-safe on the 32-bit micros() counter.
    static uint32_t next_pub_us = 0;
    const uint32_t now_us = micros();
    if ((int32_t)(now_us - next_pub_us) < 0) return;
    next_pub_us += STATES_PUBLISH_US;
    if ((int32_t)(now_us - next_pub_us) > 0) next_pub_us = now_us + STATES_PUBLISH_US;

    for (int i = 0; i < NUM_STATE_JOINTS; ++i) state_data[i] = 0.0;
    if (motors_ok) {
        // Wheel velocity (rad/s). FL/BL are physically mirrored -> negate to the
        // robot frame, same convention as the command path in cmdCb().
        // ROBOT_FRAME_WHEEL_SIGN is the same factor cmdCb() applies, so measured motion
        // is reported in the frame the command was given in (odometry direction = drive
        // direction). Never apply it here without applying it there, and vice versa.
        state_data[IDX_FL] = ROBOT_FRAME_WHEEL_SIGN * rpmToRad(-motor_fl.getRPM());
        state_data[IDX_FR] = ROBOT_FRAME_WHEEL_SIGN * rpmToRad( motor_fr.getRPM());
        state_data[IDX_BL] = ROBOT_FRAME_WHEEL_SIGN * rpmToRad(-motor_bl.getRPM());
        state_data[IDX_BR] = ROBOT_FRAME_WHEEL_SIGN * rpmToRad( motor_br.getRPM());

        // Wheel position (rad). Same FL/BL mirroring and the same robot-frame sign,
        // so the integrated position runs with the velocities, not against them.
        state_data[IDX_FL_POS] = ROBOT_FRAME_WHEEL_SIGN * -motor_fl.getPositionRad();
        state_data[IDX_FR_POS] = ROBOT_FRAME_WHEEL_SIGN *  motor_fr.getPositionRad();
        state_data[IDX_BL_POS] = ROBOT_FRAME_WHEEL_SIGN * -motor_bl.getPositionRad();
        state_data[IDX_BR_POS] = ROBOT_FRAME_WHEEL_SIGN *  motor_br.getPositionRad();

        // Provenance of the two blocks above, per wheel. NO sign and NO frame
        // conversion applies here — this is a status code, not a physical quantity;
        // multiplying it by ROBOT_FRAME_WHEEL_SIGN would turn Live into -3.
        state_data[IDX_FL_ENC] = (double)(uint8_t)motor_fl.getEncoderStatus();
        state_data[IDX_FR_ENC] = (double)(uint8_t)motor_fr.getEncoderStatus();
        state_data[IDX_BL_ENC] = (double)(uint8_t)motor_bl.getEncoderStatus();
        state_data[IDX_BR_ENC] = (double)(uint8_t)motor_br.getEncoderStatus();
    }
    // If motors_ok is false the whole array stays at the zero fill above, which reads
    // as EncoderStatus::NoEncoder = "not a measurement". That is the intended safe
    // default: the one code the zero fill can produce is the one that claims least.
    state_msg.data.size = NUM_STATE_JOINTS;
    rcl_publish(&publisher, &state_msg, nullptr);
}
