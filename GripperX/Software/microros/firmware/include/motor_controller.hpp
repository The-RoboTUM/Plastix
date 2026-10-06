#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "driver/pcnt.h"   // pcnt_unit_t for the optional encoder attachment
#include "quad_encoder.hpp"

// --- GB37-50 quadrature-encoder scaling (HWR-10) ----------------------------
// Counts of the x4 HW-PCNT decoder per OUTPUT-shaft (wheel) revolution:
//   COUNTS_PER_OUTPUT_REV = ENCODER_CPR_PER_CHANNEL * 4 (x4 decode) * GEAR_RATIO
//
// Measured 2026-08-13 on the assembled robot (HWR-10 / HWA-2): 3200 counts per
// output-shaft revolution, i.e. 16 pulses/channel/motor-rev (64 counts/motor-rev
// after x4 decoding) through the nominal 50:1 gearbox.
//
// Re-measure only if the motors or encoders are replaced:
// `python3 ~/enc_identify.py`, then `rev <wheel> 10`.
#define ENCODER_CPR_PER_CHANNEL   16.0    // measured 2026-08-13
#define GEAR_RATIO                50.0    // GB37-50 50:1 gearbox
#define COUNTS_PER_OUTPUT_REV     (ENCODER_CPR_PER_CHANNEL * 4.0 * GEAR_RATIO)

// --- Velocity estimation: sampling is DECOUPLED from publishing ---------------
// Sampling runs on its own cadence (main.cpp calls sampleEncoder() every loop
// iteration; the call self-throttles to ENC_SAMPLE_INTERVAL_US) and the publish
// loop only READS the estimate.
//
// The estimate is a first difference of the 64-bit PCNT accumulator over a SLIDING
// window of at least ENC_WINDOW_US, with dt taken from micros() and MEASURED rather
// than assumed (a millisecond clock's quantisation is a sizeable fraction of a
// short window; a microsecond one is not). Publish rate and estimation window are
// deliberately NOT the same number: a shorter window trades lower group delay
// (window/2) for more count-quantisation noise, and the two effects cross over well
// below ENC_WINDOW_US. Keep ENC_SAMPLE_INTERVAL_US, ENC_WINDOW_US and
// ENC_SAMPLE_SLOTS together if any one of them is changed.
//
// ENC_SAMPLE_INTERVAL_US only has to be small enough that the window boundary can be
// placed to that resolution; it is NOT needed to avoid losing counts (PCNT counts in
// hardware and rolls over into a 64-bit accumulator via its own ISR).
#define ENC_SAMPLE_INTERVAL_US    5000u     // 200 Hz sampling cadence
#define ENC_WINDOW_US           100000u     // estimation window (see 2. and 3. above)
#define ENC_SAMPLE_SLOTS             32     // >= ENC_WINDOW_US/ENC_SAMPLE_INTERVAL_US + headroom
// A sampling gap longer than this (micro-ROS reconnect: loop() returns early and
// nothing is sampled) invalidates every stored sample as a window boundary - counts
// accumulated across the gap must not be spread over a window that never ran.
#define ENC_MAX_GAP_US          250000u

// --- Open-loop drive feedforward: pwm = a + b*|rpm| --------------------------
// Open-loop only: gain (b) plus a constant offset (a). No error term, no
// integrator, no gain acting on the measured wheel velocity (FR-11 item 2,
// NFR-10 acceptance item 10).
//
// `a` compensates a constant load torque (rolling resistance): a gain-only law
// cannot remove a constant deficit without overspeeding the unloaded case. It
// is a first-order compensation derived from a measured speed deficit, not a
// torque measurement, and is correct only at the load/surface it was measured
// under. Compensating load variation as it changes is the future regulator's
// job (FR-11), deliberately not built here.
//
// SURFACE DEPENDENCE: this constant compensates rolling resistance, a property
// of the SURFACE, not the robot. Wrong (in either direction) on any surface
// other than the one it was measured on — too small on higher-resistance
// ground (still too slow), too large on lower-resistance or unloaded ground
// (now too fast). `TO-VERIFY` on hardware, per surface.
//
// DEADBAND, and why it is not optional: applying the offset unconditionally
// would output a + 0 = a at a commanded zero — a permanent creep whenever the
// robot is idle. The offset therefore applies only from FF_OFFSET_DEADBAND_RPM
// upwards; strictly below it the output is EXACTLY 0, an integer 0 returned
// before any arithmetic, not a product that happens to truncate to zero.
//
// Value chosen ABOVE the wheel command the steering slew brake holds while the
// wheels re-align (NFR-10 acceptance 1) — see steer_alignment_scale() in
// swerve_controller.cpp for the brake's current floor — so that "very nearly
// stopped" during a brake event is never executed as spurious PWM. That
// justification is scoped to the slew brake specifically (safety finding
// F-43): it does not extend to ordinary cornering, which never comes close to
// this deadband.
//
// A real consequence of this offset: the minimum commanded speed that actually
// produces motion is markedly lower than with gain alone. `a` is derived from
// ROLLING resistance on a robot already moving — it is NOT the duty that
// breaks the machine loose from standstill, and static friction is higher, so
// there remains a breakaway/drop-out band with no proportional regime inside
// it; the robot starts or reads zero, it cannot creep. See SAFETY.md F-41/F-42
// for the measured breakaway behaviour and its surface dependence.
#define FF_OFFSET_DEADBAND_RPM   0.5f    // below this the output is exactly 0 (see above)

enum class DriveMode {
    PwmDir,   // PWM + direction pin (motors 1 and 2)
    In1In2    // DBH-1A: IN1=PWM forward, IN2=PWM reverse (motors 3 and 4)
};

// PROVENANCE of what getRPM() returns, per wheel (FR-11 items 5/6, deviation D14).
//
// getRPM() is either a real measurement or a verbatim echo of the command, and until
// now nothing in the data said which. An echo cannot be told from a measurement by
// looking at it — it is plausible, it tracks the setpoint, and a closed loop built on
// it would regulate against its own command with an error of identically zero. This
// enum is the missing bit, published on hw/joint_states[12..15] so the Pi can mark the
// velocity as not-a-measurement instead of guessing.
//
// The order is deliberately MONOTONE IN CONFIDENCE: everything >= LiveUnconfirmed is a
// measurement, everything below it is an echo. Consumers may test the ordering; do not
// renumber. The values are mirrored on the Pi in gripperx_interface.cpp
// (WheelFeedbackProvenance) — change both or neither.
//
// NoEncoder is 0 so that the zero-filled state array (and any older/foreign publisher)
// degrades to "not a measurement" rather than to "valid".
enum class EncoderStatus : uint8_t {
    NoEncoder       = 0,   // attachEncoder() was never called for this wheel
    InitFailed      = 1,   // encoder attached, but the PCNT unit rejected its configuration
    LiveUnconfirmed = 2,   // PCNT configured and running, no count change seen since boot
    Live            = 3    // counts have actually moved -> the decoder is provably working
};

class MotorController {
public:
    MotorController(uint8_t pinA, uint8_t pinB, DriveMode mode = DriveMode::PwmDir);

    void begin();
    void setTargetRPM(float rpm);
    // Feedforward pwm = a + b*|rpm|: b (PWM counts per output-shaft RPM) and a (PWM
    // counts, applied only above FF_OFFSET_DEADBAND_RPM). Two settable halves of ONE
    // law — a is surface-specific, so set both together or neither.
    void setFeedForward(float pwmPerRPM);         // b
    void setFeedForwardOffset(float pwmOffset);   // a

    // Attach an x4 quadrature encoder on its own PCNT unit (WIRING_PLAN §1.1).
    // Call BEFORE begin(). dirSign (+1/-1) aligns the measured sign with the
    // motor COMMAND frame (a positive setTargetRPM must yield a positive
    // measured RPM); the raw A/B-vs-rotation polarity is a wiring fact that can
    // only be settled at the bench -> defaults to +1, BENCH-CONFIRM per wheel
    // (HWR-10 "directionally correct" acceptance).
    void attachEncoder(pcnt_unit_t unit, uint8_t encPinA, uint8_t encPinB, int8_t dirSign = 1);

    // Sample the encoder and refresh measured RPM + accumulated position.
    // Call FROM EVERY LOOP ITERATION - the call throttles itself to
    // ENC_SAMPLE_INTERVAL_US and is a few microseconds when it is not due. Must
    // NOT be tied to the state-publish cycle (see the block at the top of this
    // file for why that coupling is wrong).
    void sampleEncoder();

    float getTargetRPM() const;
    float getRPM() const;              // measured (encoder) if attached, else target
    double getPositionRad() const;     // accumulated wheel position, command frame
    int getPWM() const;

    // Whether the value getRPM()/getPositionRad() just returned is a measurement or an
    // echo of the command. Cheap enough to call every publish cycle.
    EncoderStatus getEncoderStatus() const;

private:
    DriveMode mode_;
    uint8_t pinA_;
    uint8_t pinB_;

    float targetRPM_ = 0.0;

    float pwmPerRPM_ = 0.90;   // b
    // a defaults to 0 => the pure-gain law, i.e. the behaviour before this change. A
    // motor that never got setFeedForwardOffset() must fall back to "too slow", never
    // to an unasked-for surface compensation. The derived value is set in main.cpp.
    float pwmOffset_ = 0.0f;   // a

    int pwmOutput_ = 0;
    int direction_ = 1;

    // --- encoder feedback (optional) ---
    QuadEncoder encoder_;
    bool  hasEncoder_ = false;
    bool  encInited_  = false;
    EncoderStatus encStatus_ = EncoderStatus::NoEncoder;
    pcnt_unit_t encUnit_ = PCNT_UNIT_0;
    uint8_t encPinA_ = 0;
    uint8_t encPinB_ = 0;
    int8_t  encDirSign_ = 1;
    float  measuredRPM_ = 0.0f;
    double positionRad_ = 0.0;

    // Ring of timestamped counter samples; the velocity window is spanned by two of
    // them. Times are micros() (uint32, wraps every 71.6 min) and are only ever used
    // as unsigned DIFFERENCES, which stay correct across that wrap.
    struct EncSample {
        int64_t  count;
        uint32_t tUs;
    };
    EncSample samples_[ENC_SAMPLE_SLOTS];
    uint8_t   sampleHead_  = 0;   // index of the newest valid sample
    uint8_t   sampleCount_ = 0;   // valid samples currently in the ring

    // Single implementation of the feedforward law. BOTH drive paths call it, so
    // PwmDir and In1In2 cannot drift apart.
    int  computePwm(float rpm) const;

    void stopMotor();
    void applyPwmDir(float rpm);
    void applyIn1In2(float rpm);
};

#endif
