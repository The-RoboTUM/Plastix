#include "quad_encoder.hpp"

namespace {
// pcnt_isr_service_install() must run exactly once for the whole process; each
// unit then adds its own handler via pcnt_isr_handler_add().
bool g_isr_service_installed = false;
}  // namespace

bool QuadEncoder::begin(pcnt_unit_t unit, uint8_t pinA, uint8_t pinB) {
    unit_ = unit;
    overflow_ = 0;

    // 3.3 V push-pull Hall encoder -> plain INPUT, no pull-ups (WIRING_PLAN §10).
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    // Channel 0: pulse=A, ctrl=B -> count BOTH edges of A, B selects direction.
    pcnt_config_t chanA = {};
    chanA.pulse_gpio_num = pinA;
    chanA.ctrl_gpio_num  = pinB;
    chanA.channel        = PCNT_CHANNEL_0;
    chanA.unit           = unit_;
    chanA.pos_mode       = PCNT_COUNT_INC;    // rising edge A: +1
    chanA.neg_mode       = PCNT_COUNT_DEC;    // falling edge A: -1
    chanA.lctrl_mode     = PCNT_MODE_KEEP;    // B low  -> keep direction
    chanA.hctrl_mode     = PCNT_MODE_REVERSE; // B high -> reverse direction
    chanA.counter_h_lim  = kHighLimit;
    chanA.counter_l_lim  = kLowLimit;
    if (pcnt_unit_config(&chanA) != ESP_OK) return false;

    // Channel 1: pulse=B, ctrl=A -> count BOTH edges of B (completes x4 decode).
    pcnt_config_t chanB = {};
    chanB.pulse_gpio_num = pinB;
    chanB.ctrl_gpio_num  = pinA;
    chanB.channel        = PCNT_CHANNEL_1;
    chanB.unit           = unit_;
    chanB.pos_mode       = PCNT_COUNT_INC;
    chanB.neg_mode       = PCNT_COUNT_DEC;
    chanB.lctrl_mode     = PCNT_MODE_REVERSE;
    chanB.hctrl_mode     = PCNT_MODE_KEEP;
    chanB.counter_h_lim  = kHighLimit;
    chanB.counter_l_lim  = kLowLimit;
    if (pcnt_unit_config(&chanB) != ESP_OK) return false;

    // Glitch filter (~12.5 us @ 80 MHz APB) against PWM/EMI-induced bounce, same
    // value as the bench sketch — safely below the shortest real encoder edge.
    if (pcnt_set_filter_value(unit_, 1000) != ESP_OK) return false;
    if (pcnt_filter_enable(unit_) != ESP_OK) return false;

    // Overflow accounting: fire on the symmetric limits and fold them into a
    // 64-bit accumulator so wheel POSITION stays continuous across rollovers.
    if (pcnt_event_enable(unit_, PCNT_EVT_H_LIM) != ESP_OK) return false;
    if (pcnt_event_enable(unit_, PCNT_EVT_L_LIM) != ESP_OK) return false;

    if (pcnt_counter_pause(unit_) != ESP_OK) return false;
    if (pcnt_counter_clear(unit_) != ESP_OK) return false;

    if (!g_isr_service_installed) {
        // ESP_ERR_INVALID_STATE would mean the service is already installed by
        // someone else; the flag is only set on a real success, so a genuine
        // failure is not remembered as done and the next unit retries.
        if (pcnt_isr_service_install(0) != ESP_OK) return false;
        g_isr_service_installed = true;
    }
    if (pcnt_isr_handler_add(unit_, &QuadEncoder::overflowIsr, this) != ESP_OK) return false;

    // Last: only a fully configured unit is ever started, so "counter running"
    // and "begin() returned true" cannot diverge.
    if (pcnt_counter_resume(unit_) != ESP_OK) return false;

    return true;
}

int64_t QuadEncoder::count() {
    // overflow_ is a 64-bit value mutated by the ISR; reads on this 32-bit MCU
    // are non-atomic, so guard each read and re-sample until the accumulator is
    // stable around the (non-atomic) hardware counter read — this rejects the
    // race where a limit rollover lands between the two reads (which would
    // otherwise produce a spurious ±kHighLimit spike in the derived velocity).
    int16_t raw = 0;
    int64_t before, after;
    do {
        noInterrupts(); before = overflow_; interrupts();
        pcnt_get_counter_value(unit_, &raw);
        noInterrupts(); after = overflow_; interrupts();
    } while (before != after);
    return after + (int64_t)raw;
}

void QuadEncoder::overflowIsr(void * arg) {
    QuadEncoder * self = static_cast<QuadEncoder *>(arg);
    uint32_t status = 0;
    pcnt_get_event_status(self->unit_, &status);
    if (status & PCNT_EVT_H_LIM) self->overflow_ += kHighLimit;
    if (status & PCNT_EVT_L_LIM) self->overflow_ += kLowLimit;
}
