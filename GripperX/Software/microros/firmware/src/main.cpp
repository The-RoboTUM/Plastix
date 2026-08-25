#include <Arduino.h>
#include "motor_controller.hpp"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>

// Cytron MDD10A drivers — all 4 channels in PWM+DIR mode.
// Board: YD-ESP32-S3-N16R8 (ESP32-S3-WROOM-1). Pin map = WIRING_PLAN.md §1
// primary GPIOs. Legacy ESP32-WROOM-32 numbering and the FR/BR rewire toggles
// (GPIO22/GPIO4) are retired — GPIO22–25 do not exist on the S3.
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
// order and motor polarity at the driver header) and can only be settled at the bench.
//
// NOT the same thing as the FL/BL robot-frame mirroring in cmdCb()/the publish loop —
// that one converts between the motor frame and the robot frame and stays as it is. Do
// not copy one into the other.
//
// BENCH PROCEDURE (per wheel, driver connected, wheel free-standing):
//   1. command a positive RPM for that wheel
//   2. echo hw/joint_states and look at that wheel's velocity index (4..7)
//   3. sign matches  -> leave at +1
//      sign inverted -> set to -1
// CONFIRMED 2026-08-13 on the assembled robot, all four = +1 (so the previous implicit
// defaults were right, but they are now measured rather than assumed):
//   1. Each wheel was rolled forward BY HAND with the pintest firmware counting, giving
//      physical direction <-> count direction:  FL/BL forward = counts UP,
//      FR/BR forward = counts DOWN.
//   2. Each wheel was then driven with `m <wheel> f` (= DIR HIGH) and the count direction
//      read back:  DIR HIGH turns FL/BL BACKWARD and FR/BR FORWARD.
// applyPwmDir() sets DIR LOW for a positive command, i.e. the opposite of the runs above,
// so a positive command turns FL/BL forward (counts up) and FR/BR backward (counts up too).
// Raw counts therefore rise on a positive command on every wheel => dirSign = +1 throughout.
// This chain lives entirely in the MOTOR command frame and does not depend on the URDF.
//
// Note the bench entry in documentation/ENCODER_FEEDBACK.md claimed the opposite count directions;
// it is superseded (miswired encoder cables were found and corrected in this session).
// Its DIR<->physical half, however, was confirmed by step 2 above.
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
// Determined by OBSERVATION on the jacked-up robot, 2026-08-17: a commanded
// linear.x = +0.3 m/s produced a clean, uniform command chain (4.286 rad/s on
// wheel_velocity_controller/commands and hw/joint_commands, ~3200 counts per wheel
// per 2 s, all four wheels within 5.7 %) — but all four wheels physically turned
// BACKWARD. Uniform across all four, so this is a global frame error, NOT a per-side
// one: the FL/BL mirroring in cmdCb()/the publish loop is correct and stays untouched.
// Hence the factor is -1.
//
// Why the error could hide: the fault was previously invisible in the data because
// command and feedback carried the same mirroring, so encoders read POSITIVE while the
// wheels ran backward — self-consistent, and contradicted by nothing but the wheels.
// That consistency is the property worth protecting, therefore:
// command and feedback MUST carry this factor IDENTICALLY. Applying it on one side only
// would give correct driving with odometry counting backward — strictly worse than the
// unfixed state. If it is ever changed, change it in cmdCb() and in the publish loop
// (velocities AND positions) together, or not at all.
#define ROBOT_FRAME_WHEEL_SIGN  (-1.0f)

// hw/joint_commands: 8 values [4 steer positions, 4 wheel velocities].
#define NUM_CMD_JOINTS    8
// hw/joint_states: 16 values [4 RESERVED steering slots, 4 wheel velocities,
//   4 wheel positions, 4 wheel-feedback PROVENANCE codes].
//   Indices 0..3 are RESERVED AND ALWAYS ZERO from this firmware — structurally,
//   not as a gap to be filled later: the steering servos hang on the Pi's USB bus,
//   the ESP32 has neither a steering sensor nor any steering input, so it has
//   nothing to measure. The publish loop zeroes the array and then writes only
//   4..11. The only value this firmware could ever place in 0..3 is an echo of the
//   commands it receives — and an echo is exactly what FR-2 rejects as feedback.
//   Do NOT "fix" this here. The real steering measurement is /hw/steer_states,
//   published by steer_servo_node on the Pi; gripperx_hardware_interface merges it
//   into the steering position state interfaces (FR-10). This comment previously
//   read "4 steer positions", a contract this firmware cannot honour.
//   Indices 4..7 keep the existing contract that the Pi
//   gripperx_hardware_interface::read() consumes (size check is >= 8, extra
//   values ignored). The appended wheel-position block (8..11) is real encoder
//   feedback for the Pi read() to adopt (HWR-10 odometry integration).
//   Indices 12..15 are the PROVENANCE of 4..7 and 8..11, one EncoderStatus code
//   per wheel in the same FL, FR, BL, BR order (FR-11 items 5/6, deviation D14):
//   the velocity is either a measurement or a verbatim echo of the command, and
//   this block is the only thing in the message that says which. Codes and their
//   meaning live in motor_controller.hpp (EncoderStatus) and are mirrored on the
//   Pi in gripperx_interface.cpp. Anything that appends further values must go
//   AFTER 15 — the Pi keys its length guards on 8 / 12 / 16 and a shorter message
//   is read as "provenance unknown", never as "valid".
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
// It was 100 ms (nominal 10 Hz) and MEASURED at 8.72 Hz / 114-121 ms, because the
// executor below was allowed to sleep 100 ms inside the same loop. Both halves are
// fixed here: this period is now scheduled on micros() with a fixed phase, and
// EXEC_SPIN_MS bounds the sleep well below it.
//
// LINK BUDGET, 115200 8N1 = 11520 B/s = 86.806 us per byte. Un-stuffed frame sizes:
// one 16-value state frame = 128 B payload + 35 B XRCE/serial framing = 163 B =
// 14.15 ms, one 8-value command frame = 64 + 35 = 99 B = 8.59 ms.
// Allowance for byte stuffing: the XRCE serial framing escapes 0x7E and 0x7D, which
// IEEE-754 doubles can hit, and the cost is data-dependent and formally unbounded.
// Expected cost is small - only ~64 of the 128 payload bytes are non-zero (indices
// 0-3 and the status codes are zero-heavy), so 64 * 2/256 = 0.5 escaped bytes per
// state frame - so this budget carries +10 % of payload, about 25x the expectation:
//   state frame  163 + 12.8 = 175.8 B = 15.26 ms
//   command frame 99 +  6.4 = 105.4 B =  9.15 ms
// Treating the link as SHARED (it is not - a UART is full duplex, so this is the
// pessimistic reading and the numbers hold either way):
//   states   30 Hz * 15.26 ms = 457.8 ms/s = 45.8 %
//   commands 30 Hz *  9.15 ms = 274.5 ms/s = 27.5 %   (Pi -> ESP32, unchanged)
//   ping     1 Hz, request + reply, allow 200 B/s      =  1.7 %
//   TOTAL 75.0 %, margin 25.0 percentage points.
// Per direction the same traffic is 46 % up / 28 % down.
// Why not higher: 40 Hz gives 61.0 + 27.5 + 1.7 = 90.2 % (9.8 pp margin) and 50 Hz
// gives 105.5 %, i.e. not feasible at all on the shared reading. The margin is kept
// wide on purpose because ONE budget item is not quantified: the publisher QoS is
// RELIABLE (rclc_publisher_init_default), so every frame is subject to XRCE-level
// acknowledgement whose byte cost has never been measured here.
// Raising the baud rate is the real headroom, but that is a COORDINATED change -
// firmware and gripperx-agent.sh (-b 115200) must move together - and it is not
// part of this change.
#define STATES_PUBLISH_US 33333
// Upper bound on how long the executor may sit in rcl_wait with nothing to do. This
// is NOT a per-message cost: spin_some returns as soon as a command is ready, so
// lowering it cannot delay cmdCb - it only stops the loop from sleeping past the
// publish deadline and past the encoder sampling interval. At 100 ms the loop spent
// nearly all of its time inside rcl_wait - fine for the commands, which wake it, but
// it meant everything that is NOT a callback (publishing, encoder sampling, the
// command timeout) could only run after that sleep, which is the direct cause of the
// measured 114-121 ms publish period. Command latency is not made worse by the
// change: a command already pending when spin_some is entered returns from rcl_wait
// immediately, and the work now done between two waits is a few microseconds of
// sampling plus one publish.
#define EXEC_SPIN_MS          5
#define CMD_TIMEOUT_MS     1000
// PING_INTERVAL_MS, the 200 ms rmw_uros_ping_agent() timeout in loop() and the
// 100 ms spin_some() slice together bound how long this firmware needs to NOTICE
// that the agent is gone: 1000 + 200 + 100 = 1300 ms worst case. The Pi depends on
// that number. HWR-40 / SR-12 chose "Option 0" for the clean-shutdown path (user,
// 2026-08-19): the Pi does NOT ask for a teardown over a dedicated interface, it
// stops the micro-ROS agent and relies on the ping failure below to call
// destroyEntities() here. Its restart path therefore waits a 1.5 s dwell before
// bringing the agent back, because startRos() opens with `if (ros_ok) return true;`
// -- an agent that reappears before this firmware has noticed the loss leaves a
// stale session that still LOOKS healthy while nothing is delivered.
// CONSEQUENCE, and the reason this comment exists: raising PING_INTERVAL_MS or the
// ping timeout, or changing the reconnect logic, breaks that dwell SILENTLY on the
// Pi side. Change them only together with the dwell in the shutdown path.
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
    // FF_GAIN (b), PWM counts per output-shaft RPM. Raised 0.85 -> 1.0625 (+25 %) on
    // 2026-08-20 by user decision, against a MEASURED steady-state shortfall: commanded
    // 4.2857 rad/s per wheel with the robot ON BLOCKS and the wheels turning FREELY IN
    // THE AIR, i.e. under no load at all, the four wheels measured 3.2242 / 3.0654 /
    // 3.1792 / 3.1398 rad/s - about 26 % below command, with a 5.2 % spread between
    // wheels. Unloaded, so THAT shortfall was the gain itself, not the load. After the
    // change, still unloaded, the four wheels came in at -3.7 / -7.1 / -4.7 / -4.3 %,
    // i.e. roughly -5 %: the gain is about right for the unloaded machine.
    //
    // FF_OFFSET_PWM (a), PWM counts. Measured 2026-08-20 with the robot DRIVING ON
    // CARPET, i.e. the load case the gain above was never checked in:
    //     commanded 4.286 rad/s -> ~2.44 rad/s   deficit 1.858 rad/s  (-43 %)
    //     commanded 2.857 rad/s -> ~1.00 rad/s   deficit 1.859 rad/s  (-65 %)
    // Same absolute deficit at two operating points => constant load torque, so the
    // correction is an offset, not more gain (see the header for why more gain is the
    // wrong form). Per-wheel at the slow point: 1.8515 / 1.8719 / 1.8478 / 1.8660 rad/s,
    // spread 1.3 %, so ONE constant for all four.
    //
    // ARITHMETIC, in full:
    //     1.858 rad/s * 60 / (2*PI)  = 17.74 RPM at the output shaft
    //     17.74 RPM * 1.0625 PWM/RPM = 18.85 -> 19 PWM counts
    //     19 / 255                   = 7.4 % duty
    // Note what the middle line is and is not: the deficit is converted into PWM VIA
    // THE GAIN. This is a FIRST-ORDER COMPENSATION DERIVED FROM A SPEED DEFICIT, NOT A
    // TORQUE MEASUREMENT - no current, no torque, no motor constant was measured. It is
    // TO-VERIFY on hardware and has never been flashed.
    //
    // CONSEQUENCES, recorded so nobody has to rediscover them:
    //  - CARPET ONLY. The offset compensates rolling resistance, which is a property of
    //    the surface. Unloaded the same firmware is already within about -5 %, so there
    //    the offset is almost pure overspeed. On grass or gravel it will be too small,
    //    on smooth floor too large, by whatever the rolling-resistance difference is.
    //  - It does NOT make the feedforward correct. It makes it correct AT ONE LOAD. The
    //    remainder as the load varies is what a future regulator (FR-11) is for; none is
    //    built here, and this stays open-loop (NFR-10 acceptance item 10).
    //  - SATURATION / HEADROOM: at max_wheel_angular_speed 12.0 rad/s (114.59 RPM,
    //    ros2_controllers.yaml) the output is 19 + 114.59*1.0625 = 140.75 -> 140 of 255
    //    (55 % duty). It was 121 with the gain alone and 97 before that. 115 counts of
    //    headroom remain; the clamp is not reachable from any legal wheel command.
    //  - THE MINIMUM COMMANDED SPEED THAT PRODUCES MOTION GOES DOWN. Below the deadband
    //    nothing changed (the old law truncated to 0 counts there too), but immediately
    //    above it the output steps from 0 to 19 counts instead of ramping from 0 - and
    //    19 counts is by construction the duty that overcomes carpet rolling resistance.
    //    Small commands that used to be swallowed by stiction now move the robot. On a
    //    lower-resistance surface the same step is a large overspeed at small commands.
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

    // Encoder sampling is DECOUPLED from publishing (motor_controller.hpp): it runs
    // on every loop iteration, self-throttled to ENC_SAMPLE_INTERVAL_US, and the
    // publish block below only reads the result. Sampling inside the publish block
    // was the defect: a measurement then existed only once per publish, over whatever
    // jittery interval the publish loop happened to produce.
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
    // does not accumulate the loop's overshoot (that drift is why 100 ms nominal
    // measured 114-121 ms). If a cycle is missed entirely the phase is re-based
    // instead of catching up, so a stall can never produce a burst of frames onto a
    // link that is already the binding constraint. The signed comparison is wrap-safe
    // on the 32-bit micros() counter.
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
