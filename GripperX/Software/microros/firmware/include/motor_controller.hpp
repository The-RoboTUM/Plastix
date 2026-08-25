#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "driver/pcnt.h"   // pcnt_unit_t for the optional encoder attachment
#include "quad_encoder.hpp"

// --- GB37-50 quadrature-encoder scaling (HWR-10) ----------------------------
// Counts of the x4 HW-PCNT decoder per OUTPUT-shaft (wheel) revolution:
//   COUNTS_PER_OUTPUT_REV = ENCODER_CPR_PER_CHANNEL * 4 (x4 decode) * GEAR_RATIO
//
// MEASURED 2026-08-13 on the assembled robot (HWR-10 / HWA-2). Two wheels were
// rolled forward by hand through exactly 10 output revolutions with the pintest
// firmware counting; no motion was commanded:
//     FL  32021 counts / 10 rev = 3202.1
//     BL  31989 counts / 10 rev = 3198.9
// Mean 3200.5, the two wheels 0.1 % apart and bracketing 3200 — which factors
// exactly as 16 * 4 * 50. So the encoder is 16 pulses per channel per MOTOR
// revolution (64 counts/motor-rev after x4 decoding, the usual value for this
// motor class) and the nominal 50:1 gearbox is confirmed rather than assumed.
//
// ENCODER_CPR_PER_CHANNEL was 11.0 before — a clearly-flagged placeholder taken
// from what is "most commonly quoted" for GB37 Hall encoders. It gave 2200, and
// dividing real counts by too small a number INFLATES the result: one true wheel
// revolution produces 3200 counts, which the old constant reported as 3200/2200 =
// 1.45 revolutions. Reported speed and distance were therefore ~45 % TOO HIGH —
// the robot actually moved about 31 % less than any odometry from before this
// date claims. Nothing was ever driven on it, so no stored map is affected.
//
// Both constants are now measured. Re-measure only if the motors or encoders are
// replaced: `python3 ~/enc_identify.py`, then `rev <wheel> 10`.
#define ENCODER_CPR_PER_CHANNEL   16.0    // measured 2026-08-13 (was 11.0, guessed)
#define GEAR_RATIO                50.0    // GB37-50 50:1, confirmed by the same measurement
#define COUNTS_PER_OUTPUT_REV     (ENCODER_CPR_PER_CHANNEL * 4.0 * GEAR_RATIO)

// --- Velocity estimation: sampling is DECOUPLED from publishing ---------------
// Until 2026-08-20 the encoder was read once per state publish, inside the publish
// block, so a "measurement" existed only every 114-121 ms and its dt was that same
// jittery interval taken from millis(). Sampling now runs on its own cadence
// (main.cpp calls sampleEncoder() every loop iteration; the call self-throttles to
// ENC_SAMPLE_INTERVAL_US) and the publish loop only READS the estimate.
//
// The estimate is a first difference of the 64-bit PCNT accumulator over a SLIDING
// window of at least ENC_WINDOW_US, with dt taken from micros() and MEASURED rather
// than assumed. Three numbers justify the constants below; keep them together if any
// one of them is changed.
//
//   1. dt quantisation. millis() over a 115 ms window: 1 ms / 115 ms = 0.87 %
//      velocity error from the clock alone. micros() over 100 ms: 1 us / 100 ms =
//      0.001 %, i.e. gone.
//   2. Count quantisation. One count = 2*PI / 3200 = 1.9635e-3 rad, so the velocity
//      step is 1.9635e-3 rad / window. At 100 ms that is 1.96e-2 rad/s = 0.46 % of
//      the 4.286 rad/s reference drive speed. This term scales as 1/window: a 33 ms
//      window would give 5.9e-2 rad/s (1.4 %) and would be NOISIER than the estimator
//      it replaces. Publish rate and estimation window are therefore deliberately
//      NOT the same number.
//   3. Group delay of a rectangular window is window/2 = 50 ms. The old estimator
//      already had a 115 ms window (57 ms group delay) AND delivered it up to 121 ms
//      late.
//
// Net effect, against the 4.286 rad/s reference drive and combining 1. and 2. in
// quadrature. OLD: sqrt(0.0171^2 + 0.0373^2) = 0.0410 rad/s = 0.96 %, delivered with
// up to 57 + 121 = 178 ms of total latency. NEW: 0.0196 rad/s = 0.46 %, with up to
// 50 + 33 = 83 ms. Note honestly that the COUNT term alone got 15 % worse (0.0171 ->
// 0.0196 rad/s) because the window is shorter than the old jittery one; removing the
// dt term more than pays for that, and it is the dt term that made the old estimate
// unusable, because it scaled with the reading instead of being a fixed floor.
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
// The law was pwm = b*|rpm| (gain only). It is now an AFFINE feedforward, gain plus
// a constant offset. This changes the SHAPE of the feedforward and nothing else: it
// is still open-loop, there is still no error term, no integrator and no gain acting
// on the measured wheel velocity (FR-11 item 2, NFR-10 acceptance item 10).
//
// WHY AN OFFSET AND NOT A BIGGER GAIN. Measured on hardware 2026-08-20, robot driving
// ON CARPET, gain already at 1.0625:
//     commanded 4.286 rad/s -> measured ~2.44 rad/s   deficit 1.858 rad/s  (-43 %)
//     commanded 2.857 rad/s -> measured ~1.00 rad/s   deficit 1.859 rad/s  (-65 %)
// Two operating points 1.5x apart, the SAME ABSOLUTE deficit; per wheel at the slow
// point 1.8515 / 1.8719 / 1.8478 / 1.8660 rad/s, a spread of 1.3 %, so ONE constant
// covers all four and four separate ones would be fitting noise. A constant speed
// deficit that does not scale with the setpoint is the signature of a constant load
// torque. Raising b would SCALE the deficit instead of removing it (it would fix one
// operating point and miss every other one) and would make the unloaded case overspeed;
// an offset removes a constant deficit identically at every operating point.
//
// WHAT THIS IS NOT. The offset is a FIRST-ORDER COMPENSATION DERIVED FROM A SPEED
// DEFICIT VIA THE GAIN, not a torque measurement — no current, no torque and no motor
// constant was measured, the deficit was simply converted into the PWM that the gain
// says would have produced it. It does not make the feedforward CORRECT; it makes it
// correct AT ONE LOAD. Whatever remains as the load varies is exactly the job of the
// future regulator (FR-11), which is deliberately not built here.
//
// SURFACE DEPENDENCE — the important caveat. This constant compensates ROLLING
// RESISTANCE and is therefore a property of the SURFACE, not of the robot. The same
// firmware unloaded (wheels in the air) shows only about -5 %, i.e. almost no deficit
// to compensate. On any surface other than the carpet it was measured on, the constant
// is wrong by whatever the rolling-resistance difference is: too small on grass or
// gravel (still too slow), too large on smooth indoor floor or unloaded on blocks
// (now too fast). TO-VERIFY on hardware, per surface.
//
// DEADBAND, and why it is not optional. An offset applied unconditionally would output
// a + 0 = a at a commanded zero, i.e. ~7.4 % duty on a robot that was told to hold
// still — a permanent creep whenever it is idle. The offset therefore applies only
// from FF_OFFSET_DEADBAND_RPM upwards, and strictly below it the output is EXACTLY 0
// (an integer 0
// returned before any arithmetic, not a product that happens to truncate to zero).
//
// Value: 0.5 RPM = 0.0524 rad/s at the wheel. Chosen ABOVE one command that is known
// to occur in normal operation and must NOT be lifted to the offset: the steering slew
// brake holds the wheel command at +-0.04 rad/s (= 0.382 RPM) while the wheels
// re-align (NFR-10 acceptance 1). That command means "very nearly stopped"; with a
// smaller deadband it would instead be executed as 19 PWM counts.
//
// SCOPE OF THAT JUSTIFICATION, narrowed 2026-08-20 by safety finding F-43 - do not
// read it as "the deadband covers the slew brake". The brake is MULTIPLICATIVE WITH A
// FLOOR: angular_speed = (target_speed / wheel_radius) * scale, with
// scale >= steer_alignment_min_scale = 0.45 (swerve_controller.cpp,
// steer_alignment_scale()). The braked command is therefore never below 45 % of the
// command, and it lands inside this deadband ONLY when the command itself is below
// ~0.116 rad/s. The +-0.04 rad/s above is a SINGLE OBSERVED SAMPLE from the NFR-10
// acceptance run, not an upper bound. The deadband still covers that case and the
// value stays right; it simply does not protect ordinary cornering, which never gets
// near it.
//
// CONSEQUENCE ON THE STARTING SPEED, stated because it is a real behaviour change, and
// MEASURED on carpet 2026-08-20 rather than argued: the minimum commanded speed that
// actually produces motion goes DOWN, from roughly 0.17 m/s to roughly 0.05 m/s.
//
// The mechanism needs one distinction that is easy to get wrong. `a` is derived from
// ROLLING resistance, measured on a robot already in motion. It is NOT the duty that
// breaks the machine loose from standstill, and STATIC friction is much higher:
//
//   PWM 19-24  (cmd up to 0.040 m/s)  -> does NOT start. Measured: 0.017 m over 22 s
//                                        across a four-step ladder, i.e. nothing.
//   PWM 25-26  (cmd 0.050 m/s)        -> BREAKAWAY. Starts, tracks to ~62 %.
//   PWM 27     (cmd 0.060 m/s)        -> starts, tracks to ~96 %.
//   PWM 22-23  (cmd 0.030 m/s)        -> DROP-OUT while already rolling; below this it
//                                        stops. ~3 counts of hysteresis.
//
// So `a` alone cannot start the robot; it lowers the command needed to REACH the
// breakaway duty, which is where the 0.17 -> 0.05 m/s improvement comes from. Between
// 0.030 and 0.050 m/s the outcome depends on whether it was already moving, and below
// ~0.06 m/s there is no proportional regime at all: measured speed clusters around
// 0.45-1.0 rad/s or falls to zero. THE MACHINE CANNOT CREEP. See SAFETY.md F-42.
//
// On a lower-resistance surface the same step is a large overspeed at small commands
// instead — breakaway is lower there, so the step is actually reached; see the surface
// note above and SAFETY.md F-41.
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
    // ENC_SAMPLE_INTERVAL_US and is a few microseconds when it is not due. It must
    // NOT be tied to the state-publish cycle: that coupling was the defect (see the
    // block at the top of this file). It was called updateEncoder() while that
    // coupling existed; the rename is deliberate, so no caller keeps the old
    // once-per-publish contract by accident.
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
