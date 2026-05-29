/**
 * @file SensorManager.cpp
 * @brief IR beam-break sensor manager implementation — TSSP77038 + discrete emitter
 *
 * Key differences from E3Z build:
 *   1. init() starts the 38kHz PWM emitter on IR_EMITTER_PIN (GPIO5) via ledcAttach.
 *   2. isBeamIntact() reads LOW = intact (TSSP77038 pulls LOW when carrier seen).
 *      E3Z was HIGH = intact. Every beam-state read in this file reflects this.
 *   3. Member renamed: sensorPin → receiverPin for clarity.
 *
 * Architecture (unchanged from E3Z build):
 *   ISR (onBeamChange)  — fires on CHANGE, timestamps event, sets pending flags.
 *                         Nothing else — no state changes, no Serial.
 *   update()            — consumes ISR flags, applies timing windows, drives state
 *                         machine, sets triggerPending / faultPending for main.cpp.
 *
 * @version 2.0.0  (Stamp S3 / TSSP77038 port)
 * @date 2025
 */

#include "SensorManager.h"

/*===========================================
 * STATIC MEMBER INITIALISATION
 *===========================================*/

uint8_t SensorManager::receiverPin = IR_RECEIVER_PIN;

volatile SensorState SensorManager::state = SENSOR_INITIALIZING;
bool     SensorManager::alignmentComplete  = false;

volatile unsigned long SensorManager::breakStartUs        = 0;
volatile bool          SensorManager::breakEventPending   = false;
volatile bool          SensorManager::restoreEventPending = false;
volatile unsigned long SensorManager::lastBreakDurationUs = 0;

unsigned long SensorManager::alignmentStableStart = 0;
uint8_t       SensorManager::alignmentProgress    = 0;
unsigned long SensorManager::faultClearStart      = 0;
unsigned long SensorManager::triggerTimestamp     = 0;

bool SensorManager::triggerPending = false;
bool SensorManager::faultPending   = false;

/*===========================================
 * ISR — keep this minimal
 *===========================================*/

void IRAM_ATTR SensorManager::onBeamChange() {
    // TSSP77038: LOW = beam intact, HIGH = beam broken
    bool beamBroken = (digitalRead(receiverPin) == HIGH);

    if (beamBroken) {
        breakStartUs        = micros();
        breakEventPending   = true;
        restoreEventPending = false;
    } else {
        // Beam restored
        if (breakStartUs > 0) {
            lastBreakDurationUs = micros() - breakStartUs;
        }
        restoreEventPending = true;
        breakEventPending   = false;
    }
}

/*===========================================
 * PUBLIC — LIFECYCLE
 *===========================================*/

void SensorManager::init(uint8_t rxPin) {
    receiverPin = rxPin;

    // ── Start IR emitter PWM ──────────────────────────────────────────────
    // Arduino core 2.x API: ledcSetup (channel, freq, resolution) + ledcAttachPin
    // Channel 0 reserved for IR emitter — do not use elsewhere
    ledcSetup(0, IR_CARRIER_FREQ_HZ, IR_CARRIER_RES_BITS);
    ledcAttachPin(IR_EMITTER_PIN, 0);
    ledcWrite(0, IR_CARRIER_DUTY);
    Serial.printf("SensorManager: IR emitter started on GPIO%d @ %dHz\n",
                  IR_EMITTER_PIN, IR_CARRIER_FREQ_HZ);

    // ── Configure receiver pin ────────────────────────────────────────────
    pinMode(receiverPin, INPUT_PULLUP);
    delay(10);  // Allow pull-up to charge and TSSP77038 to settle

    // ── Attach interrupt on both edges ────────────────────────────────────
    attachInterrupt(digitalPinToInterrupt(receiverPin), onBeamChange, CHANGE);

    Serial.printf("SensorManager: Receiver on GPIO%d (active LOW = beam intact)\n", receiverPin);
    Serial.printf("SensorManager: Timing windows — valid: %lu-%luµs  fault: >%luµs\n",
                  MIN_BREAK_US, MAX_BREAK_US, MAX_BREAK_US);

    transitionTo(SENSOR_ALIGNMENT);
    alignmentStableStart = 0;
    alignmentProgress    = 0;
}

/*===========================================
 * PUBLIC — MAIN LOOP UPDATE
 *===========================================*/

void SensorManager::update() {

    // Snapshot and clear volatile ISR flags atomically
    bool gotBreak   = false;
    bool gotRestore = false;

    noInterrupts();
    if (breakEventPending) {
        gotBreak          = true;
        breakEventPending = false;
    }
    if (restoreEventPending) {
        gotRestore          = true;
        restoreEventPending = false;
    }
    interrupts();

    if (gotBreak)   handleBreakEvent();
    if (gotRestore) handleRestoreEvent();

    // Per-state polling logic
    unsigned long now = millis();

    switch (state) {

        // ── ALIGNMENT ──────────────────────────────────────────────────────
        case SENSOR_ALIGNMENT: {
            bool intact = isBeamIntact();

            if (!intact) {
                alignmentStableStart = 0;
                alignmentProgress    = 0;
                break;
            }

            if (alignmentStableStart == 0) {
                alignmentStableStart = now;
            }

            unsigned long stableFor = now - alignmentStableStart;

            alignmentProgress = (uint8_t)min(
                (stableFor * 100UL) / ALIGNMENT_STABLE_MS,
                100UL
            );

            if (stableFor >= ALIGNMENT_STABLE_MS) {
                alignmentComplete = true;
                alignmentProgress = 100;
                Serial.println("SensorManager: Alignment confirmed — beam stable 2s");
                transitionTo(SENSOR_STANDBY);
            }
            break;
        }

        // ── STANDBY ────────────────────────────────────────────────────────
        // Not armed — beam status read live via isBeamIntact(), nothing to poll.
        case SENSOR_STANDBY:
            break;

        // ── ACTIVE ─────────────────────────────────────────────────────────
        // Poll for fault: beam broken beyond MAX_BREAK_US with no restore yet.
        case SENSOR_ACTIVE: {
            if (!isBeamIntact()) {
                noInterrupts();
                unsigned long bStart = breakStartUs;
                interrupts();

                if (bStart > 0) {
                    unsigned long breakSoFar = micros() - bStart;
                    if (breakSoFar > MAX_BREAK_US) {
                        Serial.printf("SensorManager: Fault — beam broken for %luµs (>%luµs limit)\n",
                                      breakSoFar, MAX_BREAK_US);
                        faultPending = true;
                        transitionTo(SENSOR_FAULT);
                    }
                }
            }
            break;
        }

        // ── TRIGGERED / RESETTING ──────────────────────────────────────────
        // Driven externally by main.cpp — nothing to poll.
        case SENSOR_TRIGGERED:
        case SENSOR_RESETTING:
            break;

        // ── FAULT ──────────────────────────────────────────────────────────
        // Wait for beam restored and stable for FAULT_CLEAR_MS before re-aligning.
        case SENSOR_FAULT: {
            if (!isBeamIntact()) {
                faultClearStart = 0;
                break;
            }

            if (faultClearStart == 0) {
                faultClearStart = now;
                Serial.println("SensorManager: Beam restored after fault — waiting for stable period");
            }

            if (now - faultClearStart >= FAULT_CLEAR_MS) {
                Serial.println("SensorManager: Fault cleared — returning to alignment");
                faultClearStart      = 0;
                alignmentStableStart = 0;
                alignmentProgress    = 0;
                transitionTo(SENSOR_ALIGNMENT);
            }
            break;
        }

        case SENSOR_ERROR:
        case SENSOR_INITIALIZING:
        default:
            break;
    }
}

/*===========================================
 * PRIVATE — ISR EVENT HANDLERS
 *===========================================*/

void SensorManager::handleBreakEvent() {
    switch (state) {

        case SENSOR_ALIGNMENT:
            alignmentStableStart = 0;
            alignmentProgress    = 0;
            Serial.println("SensorManager: Beam broken during alignment — restarting stable timer");
            break;

        case SENSOR_STANDBY:
            Serial.println("SensorManager: Beam broken in STANDBY (not armed)");
            break;

        case SENSOR_ACTIVE:
            Serial.println("SensorManager: Beam broken — timing window started");
            break;

        default:
            break;
    }
}

void SensorManager::handleRestoreEvent() {
    noInterrupts();
    unsigned long duration = lastBreakDurationUs;
    unsigned long bTs      = breakStartUs;
    interrupts();

    Serial.printf("SensorManager: Beam restored — break duration: %luµs\n", duration);

    switch (state) {

        case SENSOR_ACTIVE: {
            if (duration < MIN_BREAK_US) {
                // Too short — noise or glitch
                Serial.printf("SensorManager: Ignored — %luµs < %luµs minimum\n",
                              duration, MIN_BREAK_US);
                break;
            }

            if (duration <= MAX_BREAK_US) {
                // Valid crossing
                lastBreakDurationUs = duration;
                triggerTimestamp    = bTs;
                triggerPending      = true;
                Serial.printf("SensorManager: Valid crossing — %luµs\n", duration);
                transitionTo(SENSOR_TRIGGERED);
                break;
            }

            // duration > MAX_BREAK_US — fault (belt-and-braces, polling usually catches first)
            Serial.printf("SensorManager: Fault on restore — %luµs > %luµs\n",
                          duration, MAX_BREAK_US);
            faultPending = true;
            transitionTo(SENSOR_FAULT);
            break;
        }

        case SENSOR_FAULT:
            // Polling in update() handles FAULT_CLEAR_MS wait — nothing to do here
            break;

        default:
            break;
    }
}

/*===========================================
 * PRIVATE — STATE TRANSITION
 *===========================================*/

void SensorManager::transitionTo(SensorState newState) {
    if (state == newState) return;

    Serial.printf("SensorManager: %s → %s\n",
                  getStateString(),
                  [](SensorState s) -> const char* {
                      switch(s) {
                          case SENSOR_INITIALIZING: return "INITIALIZING";
                          case SENSOR_ALIGNMENT:    return "ALIGNMENT";
                          case SENSOR_STANDBY:      return "STANDBY";
                          case SENSOR_ACTIVE:       return "ACTIVE";
                          case SENSOR_TRIGGERED:    return "TRIGGERED";
                          case SENSOR_RESETTING:    return "RESETTING";
                          case SENSOR_FAULT:        return "FAULT";
                          case SENSOR_ERROR:        return "ERROR";
                          default:                  return "UNKNOWN";
                      }
                  }(newState));

    state = newState;
}

/*===========================================
 * PUBLIC — STATE QUERIES
 *===========================================*/

SensorState SensorManager::getState() {
    return state;
}

bool SensorManager::isBeamIntact() {
    // TSSP77038: LOW = carrier detected = beam intact
    return (digitalRead(receiverPin) == LOW);
}

bool SensorManager::isArmed() {
    return (state == SENSOR_ACTIVE);
}

bool SensorManager::hasTriggerPending() {
    if (triggerPending) {
        triggerPending = false;
        return true;
    }
    return false;
}

bool SensorManager::hasFaultPending() {
    if (faultPending) {
        faultPending = false;
        return true;
    }
    return false;
}

unsigned long SensorManager::getTriggerTimestamp() {
    return triggerTimestamp;
}

unsigned long SensorManager::getLastBreakDurationUs() {
    noInterrupts();
    unsigned long d = lastBreakDurationUs;
    interrupts();
    return d;
}

bool SensorManager::isAlignmentComplete() {
    return alignmentComplete;
}

uint8_t SensorManager::getAlignmentProgress() {
    return alignmentProgress;
}

/*===========================================
 * PUBLIC — CONTROL
 *===========================================*/

void SensorManager::arm() {
    if (state != SENSOR_STANDBY) {
        Serial.printf("SensorManager: arm() ignored — not in STANDBY (state=%s)\n",
                      getStateString());
        return;
    }
    if (!isBeamIntact()) {
        Serial.println("SensorManager: arm() refused — beam not intact");
        return;
    }

    noInterrupts();
    breakStartUs        = 0;
    breakEventPending   = false;
    restoreEventPending = false;
    interrupts();

    transitionTo(SENSOR_ACTIVE);
    Serial.println("SensorManager: Armed — watching for beam crossing");
}

void SensorManager::reset() {
    if (state != SENSOR_TRIGGERED && state != SENSOR_RESETTING) {
        Serial.printf("SensorManager: reset() ignored — not in TRIGGERED/RESETTING (state=%s)\n",
                      getStateString());
        return;
    }

    noInterrupts();
    breakStartUs        = 0;
    breakEventPending   = false;
    restoreEventPending = false;
    interrupts();

    if (isBeamIntact()) {
        transitionTo(SENSOR_ACTIVE);
        Serial.println("SensorManager: Reset complete — re-armed");
    } else {
        Serial.println("SensorManager: Reset — beam not intact, entering FAULT");
        faultPending = true;
        transitionTo(SENSOR_FAULT);
    }
}

void SensorManager::forceStandby() {
    noInterrupts();
    breakStartUs        = 0;
    breakEventPending   = false;
    restoreEventPending = false;
    interrupts();
    triggerPending = false;
    faultPending   = false;
    transitionTo(SENSOR_STANDBY);
    Serial.println("SensorManager: Forced to STANDBY");
}

void SensorManager::retryInit() {
    Serial.println("SensorManager: Retrying init...");
    alignmentComplete    = false;
    alignmentStableStart = 0;
    alignmentProgress    = 0;
    triggerPending       = false;
    faultPending         = false;

    noInterrupts();
    breakStartUs        = 0;
    breakEventPending   = false;
    restoreEventPending = false;
    interrupts();

    detachInterrupt(digitalPinToInterrupt(receiverPin));
    delay(10);
    attachInterrupt(digitalPinToInterrupt(receiverPin), onBeamChange, CHANGE);

    transitionTo(SENSOR_ALIGNMENT);
    alignmentStableStart = 0;
    Serial.println("SensorManager: Retry — back in ALIGNMENT");
}

/*===========================================
 * PUBLIC — DEBUG
 *===========================================*/

const char* SensorManager::getStateString() {
    switch (state) {
        case SENSOR_INITIALIZING: return "INITIALIZING";
        case SENSOR_ALIGNMENT:    return "ALIGNMENT";
        case SENSOR_STANDBY:      return "STANDBY";
        case SENSOR_ACTIVE:       return "ACTIVE";
        case SENSOR_TRIGGERED:    return "TRIGGERED";
        case SENSOR_RESETTING:    return "RESETTING";
        case SENSOR_FAULT:        return "FAULT";
        case SENSOR_ERROR:        return "ERROR";
        default:                  return "UNKNOWN";
    }
}
