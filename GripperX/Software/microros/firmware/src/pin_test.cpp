/*
 * GripperX bench instrument: pin_test
 * ====================================
 * Purpose: verify the ESP32-S3 <-> Cytron MDD10A wiring pin-by-pin at the
 * driver header, WITH THE DRIVER DISCONNECTED, before it is trusted to
 * carry motor current. Also reads the four quadrature encoders live via
 * PCNT so their wiring can be checked by turning each motor shaft by hand.
 *
 * SAFETY:
 *   - Boots fully STOPPED: every PWM/DIR pin is forced OUTPUT/LOW before
 *     Serial even comes up (mirrors the boot-safe init in main.cpp).
 *   - Only ONE PWM/DIR pin (or, for the "m" command below, one wheel's
 *     PWM+DIR pair) is ever driven at a time. Any new "on"/"m" first turns
 *     off whatever was previously active.
 *   - A pin/wheel left on is auto-stopped after ACTIVE_TIMEOUT_MS in case
 *     the operator forgets.
 *   - The "l/list", "<n> on/off" commands are a meter/scope probe with the
 *     driver header DISCONNECTED — they never assert PWM+DIR together, so
 *     they cannot themselves spin a motor even if the driver were connected.
 *   - The "m" command (added for the bench functional-drive test) DOES
 *     assert PWM+DIR together — this is the real motor-drive pattern and
 *     WILL spin a motor if the MDD10A driver is connected and powered.
 *     Only use it with the driver connected when an actual motor run is
 *     intended and approved. DIR-HIGH=forward is a convention only, to be
 *     confirmed by observing actual wheel rotation at the bench.
 *
 * Pin map: WIRING_PLAN.md §1 S3 primaries.
 *   Drive:    FL 4/5, FR 6/7, BL 15/16, BR 17/18  (PWM/DIR pairs)
 *   Encoder:  FL 8/9, FR 10/11, BL 12/13, BR 14/21 (PCNT A/B pairs)
 *
 * PCNT logic ported from Hardware_Test/bench_tests/encoder_test/encoder_test.ino
 * (originally ESP32-WROOM-32). Port notes:
 *   - That sketch picked GPIO34/35 because they are input-only on the classic
 *     ESP32, "ideal for encoder inputs". The S3 has no input-only pins at all
 *     and PCNT is routed through the GPIO matrix to any GPIO, so the
 *     input-only rationale simply does not apply here — plain INPUT works.
 *   - The classic ESP32 exposes 8 PCNT units (0-7); the S3 exposes 4 (0-3).
 *     We need exactly 4 (one per wheel), so each encoder gets its own unit,
 *     same as the original used unit 0 for its single encoder.
 *   - Channel setup (A on channel 0 controlled by B, B on channel 1
 *     controlled by A, x4 quadrature decoding) is unchanged.
 *
 * Serial command syntax @115200:
 *   l | list                    - print the drive-pin/wheel table with indices
 *   h | help | ?                - print this help
 *   <n> on                      - drive pin index n active (DIR: HIGH, PWM: low test duty)
 *   <n> off                     - drive pin index n inactive (LOW / 0)
 *   m <wheel> <f|r> [duty]      - combined per-wheel drive: assert DIR + PWM together
 *                                 (the real motor-drive pattern). wheel = 1-4 or
 *                                 FL/FR/BL/BR; f = forward (DIR HIGH), r = reverse
 *                                 (DIR LOW); duty = optional 0-255, default
 *                                 TEST_PWM_DUTY. Turns everything else off first.
 *   s | stop                    - force ALL drive pins off (safe stop)
 *   r | reset                   - reset all four encoder counters to 0
 *
 * Encoder counts print continuously in the background every PRINT_INTERVAL_MS.
 */

#include <Arduino.h>
#include "driver/pcnt.h"

// ---------------------------------------------------------------------------
// Drive pins under test (WIRING_PLAN.md §1 primaries)
// ---------------------------------------------------------------------------
struct DrivePin {
    const char * name;
    uint8_t gpio;
    bool isPwm;   // true = PWM-capable (LEDC via analogWrite), false = plain DIR GPIO
};

static const DrivePin DRIVE_PINS[] = {
    {"FL_PWM",  4, true},
    {"FL_DIR",  5, false},
    {"FR_PWM",  6, true},
    {"FR_DIR",  7, false},
    {"BL_PWM", 15, true},
    {"BL_DIR", 16, false},
    {"BR_PWM", 17, true},
    {"BR_DIR", 18, false},
};
static const int NUM_DRIVE_PINS = sizeof(DRIVE_PINS) / sizeof(DRIVE_PINS[0]);

#define TEST_PWM_DUTY      40    // ~16% of 255 - enough to meter, nowhere near a drive duty
#define ACTIVE_TIMEOUT_MS  4000  // auto-off safety net if the operator forgets a pin/wheel ON

static int activeIndex = -1;          // index into DRIVE_PINS currently driven, -1 = none
static int activeWheel = -1;          // index into WHEEL_PINS currently driven via "m", -1 = none
static unsigned long activeSinceMs = 0;

// Per-wheel PWM+DIR pair, for the combined "m" drive command (real motor-drive
// pattern). Same pins as DRIVE_PINS, just grouped by wheel instead of by pin.
struct WheelPins {
    const char * name;
    uint8_t pwmGpio;
    uint8_t dirGpio;
};

static const WheelPins WHEEL_PINS[] = {
    {"FL",  4,  5},
    {"FR",  6,  7},
    {"BL", 15, 16},
    {"BR", 17, 18},
};
static const int NUM_WHEELS = sizeof(WHEEL_PINS) / sizeof(WHEEL_PINS[0]);

// ---------------------------------------------------------------------------
// Encoder pins under test (WIRING_PLAN.md §1 primaries), one PCNT unit each
// ---------------------------------------------------------------------------
struct EncoderPins {
    const char * name;
    uint8_t pinA;
    uint8_t pinB;
    pcnt_unit_t unit;
};

static const EncoderPins ENC_PINS[] = {
    {"FL",  8,  9, PCNT_UNIT_0},
    {"FR", 10, 11, PCNT_UNIT_1},
    {"BL", 12, 13, PCNT_UNIT_2},
    {"BR", 14, 21, PCNT_UNIT_3},
};
static const int NUM_ENC_PINS = sizeof(ENC_PINS) / sizeof(ENC_PINS[0]);

#define PRINT_INTERVAL_MS 200

static int16_t lastPcntCount[NUM_ENC_PINS] = {0, 0, 0, 0};

// ---------------------------------------------------------------------------
// Drive-pin control
// ---------------------------------------------------------------------------
static void allPinsOff() {
    for (int i = 0; i < NUM_DRIVE_PINS; ++i) {
        if (DRIVE_PINS[i].isPwm) analogWrite(DRIVE_PINS[i].gpio, 0);
        else digitalWrite(DRIVE_PINS[i].gpio, LOW);
    }
    activeIndex = -1;
    activeWheel = -1;
}

static void printPinList() {
    Serial.println(F("--- Drive pins (index / name / GPIO / type) ---"));
    for (int i = 0; i < NUM_DRIVE_PINS; ++i) {
        Serial.printf("  [%d] %-7s GPIO%-2d  %s\n", i + 1, DRIVE_PINS[i].name,
                      DRIVE_PINS[i].gpio, DRIVE_PINS[i].isPwm ? "PWM" : "DIR (digital)");
    }
    Serial.println(F("--- Wheels (for 'm' combined drive command) ---"));
    for (int i = 0; i < NUM_WHEELS; ++i) {
        Serial.printf("  [%d] %-2s  PWM=GPIO%-2d  DIR=GPIO%-2d\n", i + 1, WHEEL_PINS[i].name,
                      WHEEL_PINS[i].pwmGpio, WHEEL_PINS[i].dirGpio);
    }
    Serial.println(F("--- Encoder pins (PCNT A/B) ---"));
    for (int i = 0; i < NUM_ENC_PINS; ++i) {
        Serial.printf("  %-3s A=GPIO%-2d B=GPIO%-2d\n", ENC_PINS[i].name,
                      ENC_PINS[i].pinA, ENC_PINS[i].pinB);
    }
}

// ---------------------------------------------------------------------------
// Pull-down verification (R30-R37)
// ---------------------------------------------------------------------------
// Safe-start rests entirely on the 10k pull-downs at the MDD10A inputs: during an
// ESP32 reset the drive GPIOs are high-Z for a few ms before setup() pulls them
// LOW, and only R30-R37 keep the driver inputs from floating up. That was never
// verified. This reproduces exactly that state on purpose and reads back what the
// driver input actually sits at.
//
// !!! ACTUATOR POWER MUST BE OFF. If a pull-down is missing, its pin floats during
// this test — the very condition being looked for — and a powered driver could
// start its motor. The command refuses nothing and cannot know; that is on the
// operator, so it prints the warning and requires the explicit "pd yes" form.
//
// Two readings per pin:
//   floating   pinMode(INPUT)         — the actual reset condition. LOW = held.
//   vs pull-up pinMode(INPUT_PULLUP)  — stress test. The ESP32's internal pull-up
//              is ~45k; a 10k pull-down divides it to ~0.6 V and must still read
//              LOW. A pin that reads HIGH here has no effective pull-down.
static void checkPullDowns() {
    Serial.println();
    Serial.println(F("=== PULL-DOWN CHECK (R30-R37) — actuator power must be OFF ==="));
    Serial.println(F("pin        GPIO  floating  vs 45k pull-up  verdict"));

    int missing = 0, weak = 0;
    for (int i = 0; i < NUM_DRIVE_PINS; ++i) {
        const uint8_t g = DRIVE_PINS[i].gpio;

        pinMode(g, INPUT);
        delay(30);
        int lvlFloat = 0;
        for (int k = 0; k < 5; ++k) { lvlFloat += digitalRead(g); delay(4); }

        pinMode(g, INPUT_PULLUP);
        delay(30);
        int lvlPu = 0;
        for (int k = 0; k < 5; ++k) { lvlPu += digitalRead(g); delay(4); }

        pinMode(g, OUTPUT);
        digitalWrite(g, LOW);

        const char * verdict;
        if (lvlFloat == 0 && lvlPu == 0)      verdict = "OK  pull-down holds";
        else if (lvlFloat == 0)               { verdict = "WEAK  floats LOW but loses to 45k"; ++weak; }
        else                                  { verdict = "MISSING  floats HIGH"; ++missing; }

        Serial.printf("  %-8s GPIO%-3d  %d/5 high  %d/5 high      %s\n",
                      DRIVE_PINS[i].name, g, lvlFloat, lvlPu, verdict);
    }

    Serial.println();
    if (missing == 0 && weak == 0) {
        Serial.println(F(">> ALL 8 OK — every drive input is held LOW while the ESP32 is high-Z."));
        Serial.println(F("   Safe-start through R30-R37 is verified."));
    } else {
        Serial.printf(">> %d MISSING, %d WEAK — safe-start is NOT guaranteed.\n", missing, weak);
        Serial.println(F("   Do not power the actuator branch with the ESP32 unprogrammed or resetting."));
    }
    Serial.println(F("   All pins restored to OUTPUT/LOW."));
    Serial.println();
}

static void printHelp() {
    Serial.println();
    Serial.println(F("=== GripperX pin_test ==="));
    Serial.println(F("Driver header must be DISCONNECTED while probing drive pins with"));
    Serial.println(F("'<n> on/off'. The 'm' command asserts PWM+DIR together (real motor"));
    Serial.println(F("drive) - only use it with the driver connected for an approved run."));
    Serial.println(F("Commands:"));
    Serial.println(F("  l | list             - show pin/wheel table"));
    Serial.println(F("  <n> on               - drive pin n (DIR=HIGH, PWM=low test duty)"));
    Serial.println(F("  <n> off              - stop pin n"));
    Serial.println(F("  m <wheel> <f|r> [duty] - combined drive: wheel 1-4|FL/FR/BL/BR,"));
    Serial.println(F("                         f=forward(DIR HIGH)/r=reverse(DIR LOW),"));
    Serial.println(F("                         duty 0-255 optional (default test duty)"));
    Serial.println(F("  s | stop             - stop ALL drive pins (safe stop)"));
    Serial.println(F("  r | reset            - reset encoder counters"));
    Serial.println(F("  pd yes               - verify R30-R37 pull-downs (ACTUATOR POWER OFF)"));
    Serial.println(F("  h | help | ?         - this help"));
    Serial.print(F("Exactly one pin/wheel is ever active; auto-stop after "));
    Serial.print(ACTIVE_TIMEOUT_MS);
    Serial.println(F(" ms."));
    Serial.println();
}

// Splits `line` on spaces into up to maxTokens non-empty tokens. Returns the count.
static int splitTokens(const String & line, String tokens[], int maxTokens) {
    int count = 0;
    int start = 0;
    int len = line.length();
    while (count < maxTokens && start <= len) {
        int sp = line.indexOf(' ', start);
        String tok = (sp < 0) ? line.substring(start) : line.substring(start, sp);
        tok.trim();
        if (tok.length() > 0) tokens[count++] = tok;
        if (sp < 0) break;
        start = sp + 1;
    }
    return count;
}

// "m <wheel> <f|r> [duty]" - combined PWM+DIR drive, the real motor-drive
// pattern. See file header SAFETY note: this WILL spin a motor if the
// MDD10A driver is connected and powered.
static void handleDriveCommand(String tokens[], int n) {
    if (n < 3) {
        Serial.println(F("?? usage: m <wheel 1-4|FL|FR|BL|BR> <f|r> [duty 0-255]"));
        return;
    }

    String wheelTok = tokens[1];
    String wheelUpper = wheelTok;
    wheelUpper.toUpperCase();

    bool numeric = wheelTok.length() > 0;
    for (unsigned int i = 0; i < wheelTok.length(); ++i) {
        if (!isDigit(wheelTok[i])) { numeric = false; break; }
    }

    int wIdx = -1;
    if (numeric) {
        wIdx = wheelTok.toInt() - 1;
        if (wIdx < 0 || wIdx >= NUM_WHEELS) wIdx = -1;
    } else {
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (wheelUpper == WHEEL_PINS[i].name) { wIdx = i; break; }
        }
    }
    if (wIdx < 0) {
        Serial.println(F("?? unknown wheel, use 1-4 or FL/FR/BL/BR"));
        return;
    }

    String dirTok = tokens[2];
    dirTok.toLowerCase();
    bool forward;
    if (dirTok == "f") forward = true;
    else if (dirTok == "r") forward = false;
    else {
        Serial.println(F("?? direction must be 'f' (forward) or 'r' (reverse)"));
        return;
    }

    int duty = TEST_PWM_DUTY;
    if (n >= 4) {
        duty = tokens[3].toInt();
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
    }

    allPinsOff();   // enforce exactly-one-wheel-active, same invariant as "<n> on"

    const WheelPins & w = WHEEL_PINS[wIdx];
    // DIR-HIGH=forward is a convention only - to be confirmed at the bench by
    // observing actual wheel rotation, not assumed correct.
    digitalWrite(w.dirGpio, forward ? HIGH : LOW);
    analogWrite(w.pwmGpio, duty);

    activeWheel = wIdx;
    activeSinceMs = millis();

    Serial.printf(">> DRIVE %-2s  DIR GPIO%-2d=%-4s(%s)  PWM GPIO%-2d duty=%d\n",
                  w.name, w.dirGpio, forward ? "HIGH" : "LOW", forward ? "fwd?" : "rev?",
                  w.pwmGpio, duty);
}

static void handleCommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    String tokens[4];
    int n = splitTokens(line, tokens, 4);
    if (n == 0) return;

    String cmd = tokens[0];
    String cmdLower = cmd;
    cmdLower.toLowerCase();

    if (cmdLower == "h" || cmdLower == "help" || cmdLower == "?") { printHelp(); return; }
    if (cmdLower == "l" || cmdLower == "list") { printPinList(); return; }
    if (cmdLower == "s" || cmdLower == "stop") {
        allPinsOff();
        Serial.println(F(">> STOP: all drive pins LOW/0."));
        return;
    }
    if (cmdLower == "r" || cmdLower == "reset") {
        for (int i = 0; i < NUM_ENC_PINS; ++i) {
            pcnt_counter_clear(ENC_PINS[i].unit);
            lastPcntCount[i] = 0;
        }
        Serial.println(F(">> Encoder counters reset."));
        return;
    }
    if (cmdLower == "pd") {
        if (n < 2 || String(tokens[1]) != "yes") {
            Serial.println(F("?? 'pd' reproduces the ESP32-reset condition: drive pins go high-Z."));
            Serial.println(F("   With actuator power ON and a pull-down missing, a motor can start."));
            Serial.println(F("   Switch the actuator branch OFF, then confirm with:  pd yes"));
            return;
        }
        checkPullDowns();
        return;
    }
    if (cmdLower == "m") { handleDriveCommand(tokens, n); return; }

    if (n < 2) {
        Serial.println(F("?? unknown command, type 'h' for help"));
        return;
    }
    int idx = cmd.toInt() - 1;
    String action = tokens[1];
    action.trim();

    if (idx < 0 || idx >= NUM_DRIVE_PINS) {
        Serial.println(F("?? pin index out of range, type 'l' for list"));
        return;
    }
    const DrivePin & p = DRIVE_PINS[idx];

    if (action == "on") {
        allPinsOff();   // enforce exactly-one-pin-active
        if (p.isPwm) analogWrite(p.gpio, TEST_PWM_DUTY);
        else digitalWrite(p.gpio, HIGH);
        activeIndex = idx;
        activeSinceMs = millis();
        Serial.printf(">> ON  [%d] %-7s GPIO%-2d  %s\n", idx + 1, p.name, p.gpio,
                      p.isPwm ? "PWM test duty" : "DIGITAL HIGH");
    } else if (action == "off") {
        if (p.isPwm) analogWrite(p.gpio, 0);
        else digitalWrite(p.gpio, LOW);
        if (activeIndex == idx) activeIndex = -1;
        activeWheel = -1;
        Serial.printf(">> OFF [%d] %-7s GPIO%-2d\n", idx + 1, p.name, p.gpio);
    } else {
        Serial.println(F("?? unknown action, use 'on' or 'off'"));
    }
}

// ---------------------------------------------------------------------------
// PCNT setup (ported from encoder_test.ino, parameterized per wheel/unit)
// ---------------------------------------------------------------------------
static void setupPcntPair(pcnt_unit_t unit, uint8_t pinA, uint8_t pinB) {
    pcnt_config_t chan0 = {};
    chan0.pulse_gpio_num = pinA;
    chan0.ctrl_gpio_num  = pinB;
    chan0.channel        = PCNT_CHANNEL_0;
    chan0.unit           = unit;
    chan0.pos_mode       = PCNT_COUNT_INC;    // rising edge A: +1
    chan0.neg_mode       = PCNT_COUNT_DEC;    // falling edge A: -1
    chan0.lctrl_mode     = PCNT_MODE_KEEP;    // B low -> keep count direction
    chan0.hctrl_mode     = PCNT_MODE_REVERSE; // B high -> reverse count direction
    chan0.counter_h_lim  = 32767;
    chan0.counter_l_lim  = -32768;
    pcnt_unit_config(&chan0);

    pcnt_config_t chan1 = {};
    chan1.pulse_gpio_num = pinB;
    chan1.ctrl_gpio_num  = pinA;
    chan1.channel        = PCNT_CHANNEL_1;
    chan1.unit           = unit;
    chan1.pos_mode       = PCNT_COUNT_INC;
    chan1.neg_mode       = PCNT_COUNT_DEC;
    chan1.lctrl_mode     = PCNT_MODE_REVERSE;
    chan1.hctrl_mode     = PCNT_MODE_KEEP;
    chan1.counter_h_lim  = 32767;
    chan1.counter_l_lim  = -32768;
    pcnt_unit_config(&chan1);

    // Glitch filter against contact bounce/EMI when turning by hand (~ in APB
    // clock cycles, 1000 corresponds to approx. 12.5us at 80MHz APB).
    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

static void printEncoderCounts() {
    Serial.print(F("ENC "));
    for (int i = 0; i < NUM_ENC_PINS; ++i) {
        int16_t count = 0;
        pcnt_get_counter_value(ENC_PINS[i].unit, &count);
        int16_t delta = count - lastPcntCount[i];
        lastPcntCount[i] = count;
        const char * dir = (delta > 0) ? "fwd" : (delta < 0) ? "rev" : "-";
        Serial.printf("%s=%-6d(%s) ", ENC_PINS[i].name, count, dir);
    }
    Serial.println();
}

// ---------------------------------------------------------------------------
void setup() {
    // Boot-safe: every drive pin OUTPUT/LOW before anything else runs, same
    // pattern as main.cpp:148-158 — no pin may come up active.
    for (int i = 0; i < NUM_DRIVE_PINS; ++i) {
        pinMode(DRIVE_PINS[i].gpio, OUTPUT);
        digitalWrite(DRIVE_PINS[i].gpio, LOW);
    }

    for (int i = 0; i < NUM_ENC_PINS; ++i) {
        pinMode(ENC_PINS[i].pinA, INPUT);
        pinMode(ENC_PINS[i].pinB, INPUT);
        setupPcntPair(ENC_PINS[i].unit, ENC_PINS[i].pinA, ENC_PINS[i].pinB);
    }

    Serial.begin(115200);
    delay(300);
    printHelp();
    printPinList();
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }

    if ((activeIndex >= 0 || activeWheel >= 0) && millis() - activeSinceMs > ACTIVE_TIMEOUT_MS) {
        Serial.println(F(">> AUTO-STOP (timeout): pin/wheel was left ON too long."));
        allPinsOff();
    }

    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();
    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        printEncoderCounts();
    }
}
