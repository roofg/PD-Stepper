#include "homing.h"
#include "encoder.h"
#include "motion_control.h"
#include "pins.h"
#include "step_generator.h"
#include "tmc_driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// AUX UART for debug output (defined in main.cpp)
extern HardwareSerial Serial1;

namespace homing {

// ---------------------------------------------------------------------------
// Shared state (volatile for cross-task reads)
// ---------------------------------------------------------------------------
static volatile HomingState s_state      = HOMING_IDLE;
static volatile bool        s_stall_flag = false;
static char                 s_lastError[64] = "";

// Copy of params for the homing task
static HomingParams s_params;
static TaskHandle_t s_taskHandle = nullptr;

// ---------------------------------------------------------------------------
// DIAG pin ISR — TMC2209 DIAG goes HIGH on StallGuard4 event
// ---------------------------------------------------------------------------
static void IRAM_ATTR diagISR() {
    s_stall_flag = true;
}

// ---------------------------------------------------------------------------
// Restore TMC settings from params (caller fills from main.cpp statics)
// ---------------------------------------------------------------------------
static void restoreTmcSettings(const HomingParams& p) {
    tmc::setRunCurrent(p.restoreCurrent);
    if (p.restoreStealthchop) tmc::enableStealthChop();
    if (p.restoreCoolstep) {
        tmc::enableCoolStep(1, 0);
    } else {
        tmc::disableCoolStep();
    }
    // Restore TCOOLTHRS to original value (2000 from init)
    tmc::setCoolStepDurationThreshold(2000);
    tmc::setStallGuardThreshold(p.restoreSgThresh);
}

// ---------------------------------------------------------------------------
// waitForStall result codes
// ---------------------------------------------------------------------------
static constexpr uint8_t STALL_CONTACT  = 0;
static constexpr uint8_t STALL_TIMEOUT  = 1;
static constexpr uint8_t STALL_INSTANT  = 2;
static constexpr uint8_t STALL_GRINDING = 3;
static constexpr uint8_t STALL_ESTOP    = 4;
static constexpr long    MIN_TRAVEL_COUNTS = 50;

// ---------------------------------------------------------------------------
// Wait for stall or timeout.
// Returns STALL_CONTACT(0), STALL_TIMEOUT(1), STALL_INSTANT(2),
//         STALL_GRINDING(3), or STALL_ESTOP(4).
// Uses DIAG pin interrupt (StallGuard4 works in StealthChop on TMC2209).
// Ignores stalls during the first ignoreMs to let velocity stabilize.
// encAtStart: encoder counts at probe start (for min-travel check).
// sgMinOut:   receives the minimum SG_RESULT seen during this probe.
// sgBaseOut:  receives the average SG_RESULT over first 5 samples after
//             the ignore window (free-running baseline before contact).
// ---------------------------------------------------------------------------
static uint8_t waitForStall(uint32_t timeoutMs, uint32_t ignoreMs, uint8_t sgThresh,
                             long encAtStart, uint16_t* sgMinOut, uint16_t* sgBaseOut) {
    uint32_t start = millis();
    s_stall_flag = false;
    uint32_t lastDiagMs = 0;
    int consecutiveHits = 0;
    const int HITS_NEEDED = 3; // require 3 consecutive polls below threshold
    // Stall threshold: SG_RESULT ≤ 2 * sgThresh (matching TMC2209 DIAG behavior)
    int sgTrigger = 2 * (int)sgThresh;

    uint16_t sgMin = 0xFFFF;

    // Baseline tracking: average of first 5 post-ignore-window samples
    uint32_t sgBaseSum   = 0;
    int      sgBaseCount = 0;

    // Encoder staleness tracking — seeded continuously in the ignore window
    long     encStalenessRef = encoder::getTotalCounts();
    uint32_t encStalenessMs  = millis();

    while ((millis() - start) < timeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(2)); // 2ms poll interval (~0.48ms UART read at 250kbaud)

        // Check E-stop
        if (digitalRead(PIN_SW1) == LOW) {
            *sgMinOut  = sgMin;
            *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
            return STALL_ESTOP;
        }
        if (motion::isEstopFault()) {
            *sgMinOut  = sgMin;
            *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
            return STALL_ESTOP;
        }

        int sg = tmc::getStallGuardResult();

        // Track minimum SG seen
        if ((uint16_t)sg < sgMin) sgMin = (uint16_t)sg;

        // Ignore early stalls while motor ramps up; keep staleness ref fresh
        if ((millis() - start) < ignoreMs) {
            s_stall_flag = false;
            consecutiveHits = 0;
            // Keep staleness window starting fresh so 200ms begins after ramp-up
            encStalenessRef = encoder::getTotalCounts();
            encStalenessMs  = millis();
            continue;
        }

        // Collect baseline: average of first 5 samples after ignore window
        if (sgBaseCount < 5) {
            sgBaseSum += (uint32_t)sg;
            sgBaseCount++;
        }

        // Encoder staleness check — detect grinding (motor stopped, no SG trigger)
        long encNow = encoder::getTotalCounts();
        if (abs(encNow - encStalenessRef) >= 5) {
            // Encoder moved: reset staleness window
            encStalenessRef = encNow;
            encStalenessMs  = millis();
        } else if ((millis() - encStalenessMs) >= 200) {
            // Encoder has not moved ≥5 counts in 200ms — grinding
            Serial1.printf("[HOMING] GRINDING detected: enc stuck at %ld for 200ms, SG=%d\r\n",
                (long)encNow, sg);
            *sgMinOut  = sgMin;
            *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
            return STALL_GRINDING;
        }

        // UART-based stall detection: SG_RESULT ≤ 2*SGTHRS (same as DIAG logic)
        if (sg <= sgTrigger) {
            consecutiveHits++;
        } else {
            consecutiveHits = 0;
        }

        // Diagnostic output every 200ms — includes encoder position
        uint32_t now = millis();
        if ((now - lastDiagMs) >= 200) {
            lastDiagMs = now;
            int diagPin = digitalRead(TMC_DIAG);
            tmc::DriverStatus ds = tmc::getDriverStatus();
            uint32_t tstep = tmc::getInterstepDuration();
            uint16_t pwm = tmc::getPwmScaleSum();
            long encPos = encoder::getTotalCounts();
            Serial1.printf("[HOMING] t=%lums SG=%d/%d DIAG=%d cs=%u tstep=%lu PWM=%u enc=%ld hits=%d\r\n",
                (unsigned long)(now - start), sg, sgTrigger, diagPin,
                ds.current_scaling, (unsigned long)tstep, pwm,
                (long)encPos, consecutiveHits);
        }

        // Check DIAG interrupt (if it works) OR UART polling threshold
        if (s_stall_flag || consecutiveHits >= HITS_NEEDED) {
            Serial1.printf("[HOMING] STALL! SG=%d/%d via %s\r\n",
                sg, sgTrigger, s_stall_flag ? "DIAG" : "UART");
            // DIAG pin guarantees SG_RESULT ≤ sgTrigger at the moment it fired.
            // If UART polling missed the actual dip, cap the reported min to that bound.
            if (s_stall_flag && sgMin > (uint16_t)sgTrigger) sgMin = (uint16_t)sgTrigger;
            // Min-travel check: premature stall if barely moved
            long travel = abs(encoder::getTotalCounts() - encAtStart);
            if (travel < MIN_TRAVEL_COUNTS) {
                Serial1.printf("[HOMING] INSTANT STALL: travel=%ld counts (< %ld)\r\n",
                    (long)travel, (long)MIN_TRAVEL_COUNTS);
                *sgMinOut  = sgMin;
                *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
                return STALL_INSTANT;
            }
            *sgMinOut  = sgMin;
            *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
            return STALL_CONTACT;
        }
    }
    *sgMinOut  = sgMin;
    *sgBaseOut = sgBaseCount ? (uint16_t)(sgBaseSum / sgBaseCount) : 0xFFFF;
    return STALL_TIMEOUT;
}

// ---------------------------------------------------------------------------
// Homing Task — one-shot FreeRTOS task, self-deletes on completion
//
// Uses StealthChop + StallGuard4 (not SpreadCycle).
// TMC2209 SG4 works in StealthChop — unlike TMC2130/2160 which need
// SpreadCycle. Keeping StealthChop + pwm_autoscale gives good SG dynamic
// range (reference: KushagraK7/TMC2209_sensorless_homing_test).
// ---------------------------------------------------------------------------
static void HomingTask(void* pvParameters) {
    HomingParams p = s_params;
    float dirSign = p.directionCW ? 1.0f : -1.0f;

    Serial1.println("[HOMING] Starting — StealthChop + StallGuard4");

    // --- Configure TMC for homing ---
    s_state = HOMING_FAST_APPROACH;
    tmc::setRunCurrent(p.currentPct);
    // Stay in StealthChop — SG4 works here on TMC2209
    tmc::enableStealthChop();
    // Ensure automatic current scaling is on (needed for SG4 in StealthChop)
    tmc::enableAutomaticCurrentScaling();
    tmc::disableCoolStep();          // CoolStep interferes with SG readings
    // TCOOLTHRS must be set for StallGuard and DIAG pin to work.
    // StallGuard only activates when TSTEP < TCOOLTHRS.
    // Use 20-bit max (0xFFFFF) so SG is active at all speeds.
    tmc::setCoolStepDurationThreshold(0xFFFFF);
    tmc::setStallGuardThreshold(p.sgThresh1);
    tmc::enable();
    vTaskDelay(pdMS_TO_TICKS(50)); // let current regulator and PWM autoscale settle

    // Diagnostic: confirm config before starting
    {
        tmc::DriverStatus ds = tmc::getDriverStatus();
        int sg = tmc::getStallGuardResult();
        int diagPin = digitalRead(TMC_DIAG);
        uint32_t tstep = tmc::getInterstepDuration();
        uint16_t pwm = tmc::getPwmScaleSum();
        Serial1.printf("[HOMING] Config: cur=%d%% sgThresh=%d DIAG=%d SC=%d SG=%d cs=%u tstep=%lu PWM=%u\r\n",
            p.currentPct, p.sgThresh1, diagPin,
            ds.stealth_chop ? 1 : 0, sg, ds.current_scaling, (unsigned long)tstep, pwm);
    }

    // --- Pre-approach backoff ---
    // Move away from the endstop before starting the fast approach.
    // Ensures the carriage has a clean run-up for velocity stabilisation,
    // and prevents grinding if homing is triggered while already at the endstop.
    Serial1.printf("[HOMING] Pre-backoff: %.0f steps at fast speed\r\n", p.backoffSteps);
    stepgen::resetStepCount();
    stepgen::setVelocity(-dirSign * p.speed1);
    {
        uint32_t t0 = millis();
        while (abs(stepgen::getStepCount()) < (int32_t)p.backoffSteps) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if ((millis() - t0) > 5000) break;
        }
    }
    stepgen::halt();
    vTaskDelay(pdMS_TO_TICKS(200)); // let motor and belt settle

    // Attach DIAG interrupt
    s_stall_flag = false;
    attachInterrupt(digitalPinToInterrupt(TMC_DIAG), diagISR, RISING);

    // --- Stage 1: Fast approach ---
    long encAtFast = encoder::getTotalCounts();
    uint16_t sgMinFast = 0xFFFF, sgMinSlow = 0xFFFF;
    uint16_t sgBaseFast = 0xFFFF, sgBaseSlow = 0xFFFF;
    stepgen::setVelocity(dirSign * p.speed1);
    Serial1.printf("[HOMING] Fast approach at %.0f steps/s, SG thresh=%d\r\n", p.speed1, p.sgThresh1);

    uint8_t r1 = waitForStall(p.timeoutMs, 300, p.sgThresh1, encAtFast, &sgMinFast, &sgBaseFast);
    if (r1 != STALL_CONTACT) {
        switch (r1) {
            case STALL_TIMEOUT:
                strncpy(s_lastError, "Timeout: no contact detected", sizeof(s_lastError) - 1);
                break;
            case STALL_INSTANT:
                strncpy(s_lastError, "Stall at start: lower SGThresh or check position", sizeof(s_lastError) - 1);
                break;
            case STALL_GRINDING:
                strncpy(s_lastError, "Grinding: SG not triggered \xe2\x80\x94 raise SGThresh", sizeof(s_lastError) - 1);
                break;
            case STALL_ESTOP:
            default:
                strncpy(s_lastError, "E-Stop during homing", sizeof(s_lastError) - 1);
                break;
        }
        motion::sendHomingResult(r1, sgMinFast, 0xFFFF, sgBaseFast, 0xFFFF, encoder::getTotalCounts(), s_lastError);
        stepgen::halt();
        detachInterrupt(digitalPinToInterrupt(TMC_DIAG));
        restoreTmcSettings(p);
        s_state = HOMING_ERROR;
        Serial1.printf("[HOMING] Error: %s\r\n", s_lastError);
        s_taskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    stepgen::halt();
    // Detach interrupt during backoff to prevent spurious re-triggers
    detachInterrupt(digitalPinToInterrupt(TMC_DIAG));
    Serial1.println("[HOMING] Stall detected (fast), backing off");

    // --- Stage 2: Backoff ---
    s_state = HOMING_BACKOFF;
    vTaskDelay(pdMS_TO_TICKS(200)); // settle after stall
    stepgen::resetStepCount();
    stepgen::setVelocity(-dirSign * p.speed1); // reverse at fast speed

    // Wait until half the backoff distance is reached
    uint32_t backoffStart = millis();
    while (abs(stepgen::getStepCount()) < (int32_t)(p.backoffSteps / 2)) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if ((millis() - backoffStart) > 5000) break; // safety timeout for backoff
    }
    stepgen::halt();
    vTaskDelay(pdMS_TO_TICKS(200)); // settle before slow approach

    // --- Stage 3: Slow approach ---
    s_state = HOMING_SLOW_APPROACH;
    // Set new SG threshold BEFORE re-attaching interrupt.
    // At slow speed, SG baseline is much lower (~24-28 vs ~140 at fast speed).
    // sgThresh2 must be low enough that 2*sgThresh2 < slow-speed SG baseline.
    tmc::setStallGuardThreshold(p.sgThresh2);
    vTaskDelay(pdMS_TO_TICKS(50)); // let TMC update DIAG based on new threshold

    // Sample SG baseline at slow speed before starting approach
    stepgen::setVelocity(dirSign * p.speed2);
    vTaskDelay(pdMS_TO_TICKS(300)); // let speed stabilize
    {
        int sgBaseline = tmc::getStallGuardResult();
        int diagNow = digitalRead(TMC_DIAG);
        Serial1.printf("[HOMING] Slow SG baseline=%d trigger<=%d DIAG=%d\r\n",
            sgBaseline, 2 * p.sgThresh2, diagNow);
        if (sgBaseline <= 2 * p.sgThresh2) {
            Serial1.printf("[HOMING] WARNING: SG baseline (%d) already below trigger (%d)! Lower sgThresh2.\r\n",
                sgBaseline, 2 * p.sgThresh2);
        }
    }

    // Re-attach DIAG interrupt fresh — clear any pending state
    s_stall_flag = false;
    attachInterrupt(digitalPinToInterrupt(TMC_DIAG), diagISR, RISING);
    long encAtSlow = encoder::getTotalCounts();
    Serial1.printf("[HOMING] Slow approach at %.0f steps/s, SG thresh=%d\r\n", p.speed2, p.sgThresh2);

    uint8_t r2 = waitForStall(p.timeoutMs, 300, p.sgThresh2, encAtSlow, &sgMinSlow, &sgBaseSlow);
    if (r2 != STALL_CONTACT) {
        switch (r2) {
            case STALL_TIMEOUT:
                strncpy(s_lastError, "Timeout: no contact detected", sizeof(s_lastError) - 1);
                break;
            case STALL_INSTANT:
                strncpy(s_lastError, "Stall at start: lower SGThresh or check position", sizeof(s_lastError) - 1);
                break;
            case STALL_GRINDING:
                strncpy(s_lastError, "Grinding: SG not triggered \xe2\x80\x94 raise SGThresh", sizeof(s_lastError) - 1);
                break;
            case STALL_ESTOP:
            default:
                strncpy(s_lastError, "E-Stop during homing", sizeof(s_lastError) - 1);
                break;
        }
        motion::sendHomingResult(r2, sgMinFast, sgMinSlow, sgBaseFast, sgBaseSlow, encoder::getTotalCounts(), s_lastError);
        stepgen::halt();
        detachInterrupt(digitalPinToInterrupt(TMC_DIAG));
        restoreTmcSettings(p);
        s_state = HOMING_ERROR;
        Serial1.printf("[HOMING] Error: %s\r\n", s_lastError);
        s_taskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    stepgen::halt();
    detachInterrupt(digitalPinToInterrupt(TMC_DIAG));
    Serial1.println("[HOMING] Stall detected (slow), relaxing belt");

    // --- Stage 4: Relaxation backoff — let belt decompress before zeroing ---
    s_state = HOMING_ZEROING;
    vTaskDelay(pdMS_TO_TICKS(100)); // brief settle after stall
    stepgen::resetStepCount();
    stepgen::setVelocity(-dirSign * p.speed2); // back off slowly
    {
        uint32_t relax_start = millis();
        while (abs(stepgen::getStepCount()) < (int32_t)(p.backoffSteps / 4)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if ((millis() - relax_start) > 3000) break;
        }
    }
    stepgen::halt();
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial1.println("[HOMING] Zeroing positions");

    encoder::resetPosition();
    stepgen::resetStepCount();
    motion::resetPositions();

    // --- Done ---
    restoreTmcSettings(p);
    motion::sendHomingResult(STALL_CONTACT, sgMinFast, sgMinSlow, sgBaseFast, sgBaseSlow, 0, "");
    s_state = HOMING_DONE;
    s_lastError[0] = '\0';
    Serial1.println("[HOMING] Complete — positions zeroed");

    // Brief pause so GUI can see DONE state, then go idle
    vTaskDelay(pdMS_TO_TICKS(500));
    s_state = HOMING_IDLE;
    s_taskHandle = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void init() {
    // DIAG pin configured as INPUT in setup().
    // ISR is attached/detached on demand during homing.
}

bool start(const HomingParams& params) {
    if (motion::isRunning() || isActive()) return false;

    s_params     = params;
    s_lastError[0] = '\0';
    // Set state to FAST_APPROACH immediately so isActive() returns true
    // before the task starts — prevents ControlTask hold loop from fighting.
    s_state      = HOMING_FAST_APPROACH;

    BaseType_t ret = xTaskCreatePinnedToCore(
        HomingTask, "HomingTask", 4096, nullptr, 3, &s_taskHandle, 0);

    return (ret == pdPASS);
}

HomingState getState() { return s_state; }

bool isActive() {
    HomingState st = s_state;
    return st != HOMING_IDLE && st != HOMING_DONE && st != HOMING_ERROR;
}

const char* lastError() { return s_lastError; }

} // namespace homing
