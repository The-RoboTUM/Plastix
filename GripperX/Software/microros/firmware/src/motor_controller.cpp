#include "motor_controller.hpp"

MotorController::MotorController(uint8_t pinA, uint8_t pinB, DriveMode mode)
    : mode_(mode),
      pinA_(pinA),
      pinB_(pinB)
{
}

void MotorController::begin() {
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
    stopMotor();

    // Encoder input pins are separate from the motor PWM/DIR pins, so starting
    // the PCNT decoder here never drives a motor. Guarded so PCNT config + the
    // ISR handler are set up exactly once (begin() re-runs on micro-ROS
    // reconnect); position then keeps accumulating across reconnects.
    if (hasEncoder_ && !encInited_) {
        if (encoder_.begin(encUnit_, encPinA_, encPinB_)) {
            sampleHead_   = 0;
            sampleCount_  = 0;
            measuredRPM_  = 0.0f;
            positionRad_  = 0.0;
            encInited_    = true;
            // Configured and counting, but nothing has been seen to move yet — that is a
            // weaker claim than "working", and it is reported as the weaker claim.
            encStatus_    = EncoderStatus::LiveUnconfirmed;
        } else {
            // PCNT refused the configuration. getRPM() will fall back to the target, i.e.
            // echo the command; say so instead of publishing a plausible-looking number.
            encStatus_    = EncoderStatus::InitFailed;
        }
    }
    // No else: without attachEncoder() the status stays NoEncoder. Once Live has been
    // reached it also survives a begin() re-run on micro-ROS reconnect, because the
    // block above is guarded by encInited_ and the counter keeps its accumulator.
}

void MotorController::attachEncoder(pcnt_unit_t unit, uint8_t encPinA, uint8_t encPinB, int8_t dirSign) {
    encUnit_    = unit;
    encPinA_    = encPinA;
    encPinB_    = encPinB;
    encDirSign_ = dirSign;
    hasEncoder_ = true;
}

void MotorController::sampleEncoder() {
    if (!encInited_) return;

    const uint32_t now = micros();
    const uint32_t sinceLast = (sampleCount_ > 0)
                                 ? (uint32_t)(now - samples_[sampleHead_].tUs)
                                 : 0u;
    if (sampleCount_ > 0 && sinceLast < ENC_SAMPLE_INTERVAL_US) return;   // not due yet

    const int64_t c = encoder_.count();

    // Promote on the FIRST observed count change since boot — no count threshold and no
    // time window, both of which would need a calibration that does not exist.
    // DELIBERATELY ONE-WAY: there is no path back from Live. A downgrade would have to
    // mean "commanded to move but not counting", which requires comparing command and
    // count over a window with thresholds nobody has measured; inventing one here would
    // make a stationary robot report a dead encoder every time it stands still. That
    // detection belongs to HWR-30a. Do not add a decay/timeout here.
    if (sampleCount_ > 0 && c != samples_[sampleHead_].count &&
        encStatus_ == EncoderStatus::LiveUnconfirmed) {
        encStatus_ = EncoderStatus::Live;
    }

    // Sampling stopped for a long time (micro-ROS reconnect). Every stored sample is
    // now a window boundary that would spread the counts of the whole gap over a
    // window that never ran -> discard the history and restart the window.
    if (sampleCount_ > 0 && sinceLast > ENC_MAX_GAP_US) {
        sampleCount_ = 0;
        measuredRPM_ = 0.0f;
    }

    sampleHead_ = (uint8_t)((sampleHead_ + 1) % ENC_SAMPLE_SLOTS);
    samples_[sampleHead_].count = c;
    samples_[sampleHead_].tUs   = now;
    if (sampleCount_ < ENC_SAMPLE_SLOTS) ++sampleCount_;

    // Position: derived from the absolute count each sample (no integration drift)
    // -> output-shaft radians, motor COMMAND frame.
    positionRad_ = (((double)encDirSign_ * (double)c) / COUNTS_PER_OUTPUT_REV) * (2.0 * PI);

    // Velocity: walk back to the oldest sample that is at least ENC_WINDOW_US old. If
    // the ring does not reach that far back yet (the first ~100 ms after init, or right
    // after a gap reset) the oldest available sample is used, so the window grows into
    // its nominal length instead of producing a spike. dt is MEASURED between the two
    // samples, so an irregular sample cadence changes the window LENGTH, never the
    // correctness of the estimate.
    uint8_t  idx = sampleHead_;
    uint32_t dt  = 0;
    for (uint8_t i = 1; i < sampleCount_; ++i) {
        idx = (uint8_t)((sampleHead_ + ENC_SAMPLE_SLOTS - i) % ENC_SAMPLE_SLOTS);
        dt  = (uint32_t)(now - samples_[idx].tUs);
        if (dt >= ENC_WINDOW_US) break;
    }
    if (dt == 0) return;   // only one sample so far -> keep the last estimate

    // Output-shaft revolutions over the window -> RPM. dt is in microseconds.
    const double revs = ((double)encDirSign_ * (double)(c - samples_[idx].count))
                        / COUNTS_PER_OUTPUT_REV;
    measuredRPM_ = (float)(revs * 60000000.0 / (double)dt);
}

void MotorController::stopMotor() {
    pwmOutput_ = 0;

    if (mode_ == DriveMode::In1In2) {
        analogWrite(pinA_, 0);
        analogWrite(pinB_, 0);
        return;
    }

    digitalWrite(pinB_, LOW);
    analogWrite(pinA_, 0);
}

void MotorController::setTargetRPM(float rpm) {
    targetRPM_ = rpm;

    // Anything inside the feedforward deadband is a full stop, INCLUDING the commanded
    // zero (|0.0| < FF_OFFSET_DEADBAND_RPM, so this subsumes the old == 0.0 test and no
    // longer depends on an exact float compare). This is the outer of two guards: it
    // drives the pins to a hard stop, while computePwm() independently returns 0 for the
    // same range. Without it the offset would creep the robot whenever it is idle.
    if (fabs(targetRPM_) < FF_OFFSET_DEADBAND_RPM) {
        stopMotor();
        return;
    }

    if (mode_ == DriveMode::PwmDir) {
        applyPwmDir(rpm);
    } else {
        applyIn1In2(rpm);
    }
}

// Feedforward pwm = a + b*|rpm|, clamped to the 8-bit range. Derivation, surface
// dependence and the reason for the deadband are in motor_controller.hpp.
int MotorController::computePwm(float rpm) const {
    const float mag = fabs(rpm);
    // Returned as an integer 0 BEFORE any arithmetic: below the deadband the output is
    // exactly off, it does not depend on a product happening to truncate to zero.
    if (mag < FF_OFFSET_DEADBAND_RPM) return 0;
    return constrain((int)(pwmOffset_ + mag * pwmPerRPM_), 0, 255);
}

void MotorController::applyPwmDir(float rpm) {
    if (rpm > 0.0) {
        direction_ = 1;
        digitalWrite(pinB_, LOW);
    } else {
        direction_ = -1;
        digitalWrite(pinB_, HIGH);
    }

    pwmOutput_ = computePwm(rpm);
    analogWrite(pinA_, pwmOutput_);
}

void MotorController::applyIn1In2(float rpm) {
    pwmOutput_ = computePwm(rpm);

    // Always turn off both with PWM before changing direction.
    // Mixing digitalWrite + analogWrite on ESP32 leaves the DBH with IN1 and IN2 active.
    analogWrite(pinA_, 0);
    analogWrite(pinB_, 0);

    if (rpm > 0.0) {
        direction_ = 1;
        if (pwmOutput_ > 0) {
            analogWrite(pinA_, pwmOutput_);
        }
    } else {
        direction_ = -1;
        if (pwmOutput_ > 0) {
            analogWrite(pinB_, pwmOutput_);
        }
    }
}

void MotorController::setFeedForward(float pwmPerRPM) {
    pwmPerRPM_ = pwmPerRPM;
}

void MotorController::setFeedForwardOffset(float pwmOffset) {
    pwmOffset_ = pwmOffset;
}

float MotorController::getTargetRPM() const {
    return targetRPM_;
}

float MotorController::getRPM() const {
    // Real measured RPM once the encoder is running; falls back to the target
    // (legacy synthetic behaviour) only until the encoder is initialised.
    // The fallback is INDISTINGUISHABLE from a measurement by value — which branch
    // was taken is reported by getEncoderStatus(), and only there.
    return encInited_ ? measuredRPM_ : targetRPM_;
}

double MotorController::getPositionRad() const {
    return positionRad_;
}

int MotorController::getPWM() const {
    return pwmOutput_;
}

EncoderStatus MotorController::getEncoderStatus() const {
    return encStatus_;
}
