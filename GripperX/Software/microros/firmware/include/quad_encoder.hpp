#ifndef QUAD_ENCODER_HPP
#define QUAD_ENCODER_HPP

#include <Arduino.h>
#include "driver/pcnt.h"

// x4 hardware quadrature decoder for one A/B encoder, using ONE PCNT unit
// (both channels) — the exact decode scheme bench-proven in
// Hardware_Test/bench_tests/encoder_test.ino, ported from the classic ESP32 to
// the ESP32-S3. The S3 has 4 PCNT units (PCNT_UNIT_0..3) -> one per drive wheel.
//
// PCNT reaches any GPIO through the GPIO matrix, so the encoder pins are
// unrestricted (WIRING_PLAN.md §1.1). Encoder inputs are 3.3 V push-pull Hall
// -> plain INPUT, NO pull-ups (WIRING_PLAN.md §4/§10; the S3 GPIOs are not
// 5 V-tolerant).
//
// Uses the legacy ESP-IDF PCNT API (driver/pcnt.h). It is still present (and
// deprecated) in the arduino-esp32 3.x / IDF 5.x core; deprecation warnings are
// expected, compilation is not affected.
class QuadEncoder {
public:
    QuadEncoder() = default;

    // Configure the PCNT unit for x4 decode on (pinA, pinB) and start counting.
    // Installs the shared PCNT ISR service once (process-wide) and registers a
    // per-unit overflow handler so POSITION does not wrap after ~30000 counts.
    //
    // Returns TRUE only if every driver call succeeded and the counter is
    // actually running. Every esp_err_t used to be discarded, so a rejected PCNT
    // configuration produced a silently dead decoder that MotorController then
    // reported as an initialised encoder — the count stays 0, the derived RPM
    // stays 0, and nothing anywhere says the number is not a measurement
    // (FR-11 item 5). On the first failure the function returns immediately
    // WITHOUT resuming the counter, so a half-configured unit never counts.
    bool begin(pcnt_unit_t unit, uint8_t pinA, uint8_t pinB);

    // 64-bit accumulated count = 16-bit hardware value + limit-overflow rollovers.
    // Reads are made race-safe against the overflow ISR.
    int64_t count();

private:
    static void overflowIsr(void * arg);   // accumulates H_LIM / L_LIM rollovers

    pcnt_unit_t unit_ = PCNT_UNIT_0;
    volatile int64_t overflow_ = 0;

    // Symmetric software limits well inside the int16 counter range; on hitting
    // a limit the hardware counter auto-resets to 0 and fires the H/L_LIM event.
    static constexpr int16_t kHighLimit = 30000;
    static constexpr int16_t kLowLimit  = -30000;
};

#endif  // QUAD_ENCODER_HPP
