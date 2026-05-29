/**
 * @file SensorManager.h
 * @brief IR beam-break sensor manager — TSSP77038 + discrete emitter
 *
 * Hardware:
 *   Emitter  : IR LED driven at 38kHz via ledcAttach/ledcWrite on GPIO5
 *   Receiver : TSSP77038 demodulated output on GPIO7 (INPUT_PULLUP)
 *
 * Signal logic (TSSP77038 — inverted vs E3Z NPN):
 *   Beam INTACT  = LOW   (demodulator sees 38kHz → pulls output LOW)
 *   Beam BROKEN  = HIGH  (no carrier → pull-up holds line HIGH)
 *
 * Interrupt on CHANGE fires on both beam-break and beam-restore.
 * ISR is kept minimal — timestamps only, all logic in update().
 *
 * Timing windows:
 *   < MIN_BREAK_US        : ignored  (noise / glitch)
 *   MIN_BREAK_US to
 *   MAX_BREAK_US          : valid crossing → fire trigger event
 *   > MAX_BREAK_US        : sustained fault → disarm and alert
 *
 * @version 2.0.0  (Stamp S3 / TSSP77038 port — forked from E3Z v1.0.0)
 * @date 2025
 */

#pragma once
#include <Arduino.h>
#include "pin_config.h"

/*===========================================
 * TIMING CONSTANTS
 *===========================================*/

// Minimum beam-break duration to register as a valid crossing.
// Rejects sub-10ms glitches (electrical noise, insects, fast debris).
// A BMX rider at sprint speed (~10 m/s) crossing a ~20cm body width
// takes ~20ms minimum, so 10ms gives comfortable margin.
#define MIN_BREAK_US        10000UL     // 10ms in microseconds

// Maximum beam-break duration before declaring a fault.
// A rider crossing at walking pace (~1 m/s) over ~50cm takes ~500ms.
// Anything beyond 500ms is almost certainly a post knock or obstruction.
#define MAX_BREAK_US        500000UL    // 500ms in microseconds

// How long the beam must be continuously restored after a fault
// before the system will allow re-arming.
#define FAULT_CLEAR_MS      2000UL      // 2 seconds of clean beam

// How long beam must be stable during alignment before declaring OK.
#define ALIGNMENT_STABLE_MS 2000UL      // 2 seconds stable = aligned

/*===========================================
 * SENSOR STATES
 *===========================================*/

enum SensorState {
    SENSOR_INITIALIZING,    // GPIO/PWM setup, not yet checked
    SENSOR_ALIGNMENT,       // Boot-time beam verification
    SENSOR_STANDBY,         // Beam OK, waiting for arm signal from controller
    SENSOR_ACTIVE,          // Armed, watching for crossing
    SENSOR_TRIGGERED,       // Valid crossing detected, signal sent
    SENSOR_RESETTING,       // Brief post-trigger lockout before re-arming
    SENSOR_FAULT,           // Beam lost too long while armed (or during standby)
    SENSOR_ERROR            // Hardware / init failure
};

/*===========================================
 * SENSOR MANAGER CLASS
 *===========================================*/

class SensorManager {
public:
    // Lifecycle
    static void init(uint8_t receiverPin);  // Also starts emitter PWM on IR_EMITTER_PIN
    static void update();                   // Call every loop iteration

    // State queries
    static SensorState getState();
    static bool isBeamIntact();             // Raw beam status (LOW = intact for TSSP77038)
    static bool isArmed();                  // True only in SENSOR_ACTIVE
    static bool hasTriggerPending();        // True after valid crossing, clears on read
    static bool hasFaultPending();          // True after fault declared, clears on read
    static unsigned long getTriggerTimestamp(); // micros() at moment of beam break

    // Control
    static void arm();                      // Transition STANDBY → ACTIVE
    static void reset();                    // After trigger: back to ACTIVE
    static void forceStandby();             // Emergency de-arm
    static void retryInit();                // From SENSOR_ERROR: retry setup

    // Alignment
    static bool isAlignmentComplete();      // True once boot alignment passed
    static uint8_t getAlignmentProgress();  // 0-100

    // Debug
    static const char* getStateString();
    static unsigned long getLastBreakDurationUs();

private:
    // Pins
    static uint8_t receiverPin;

    // State
    static volatile SensorState state;
    static bool alignmentComplete;

    // Interrupt-captured values (volatile, written in ISR)
    static volatile unsigned long breakStartUs;
    static volatile bool          breakEventPending;
    static volatile bool          restoreEventPending;
    static volatile unsigned long lastBreakDurationUs;

    // Main-loop timing
    static unsigned long alignmentStableStart;
    static uint8_t       alignmentProgress;
    static unsigned long faultClearStart;
    static unsigned long triggerTimestamp;

    // Pending flags consumed by main.cpp
    static bool triggerPending;
    static bool faultPending;

    // ISR
    static void IRAM_ATTR onBeamChange();

    // Internal helpers
    static void handleBreakEvent();
    static void handleRestoreEvent();
    static void transitionTo(SensorState newState);
};
