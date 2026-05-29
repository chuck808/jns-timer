#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @file PowerManager.h
 * @brief Owns the M5PM1 PMIC on the BAT_Carrier board.
 *
 * Deliberately separate from BatteryManager so the *rail* always comes up
 * even when battery *telemetry* is disabled for bench testing.
 *
 * Responsibilities:
 *   - Bring up the I2C bus to the PM1 (G48/G47, addr 0x6E)
 *   - Enable EXT_5V_OUT (SYS_5V) — REQUIRED before the IR emitter will light
 *   - Enable charging (current selectable, see PM1_CHARGE_650MA in the .cpp)
 *   - Arm the WAKE button (PY_G4 / PM1 IO4): internal pull-up + falling edge
 *   - Provide a real soft power-off (shutdown) that wakes on the button
 *   - Expose raw PMIC voltage reads for an optional BatteryManager
 *
 * HARDWARE NOTE: R5 (10K) on WAKE_IN_N must be left UNPOPULATED — it is a
 * pull-down that defeats the PM1 internal pull-up the wake relies on.
 *
 * begin() is non-fatal: if the PM1 is absent (e.g. bench testing on bare
 * hardware) it logs a warning and the device still boots — but with the 5V
 * rail OFF, so the emitter won't work unless you power J1 externally.
 */
class PowerManager {
public:
    static bool begin();                 // bus + 5V rail + charge + wake. true = PM1 found
    static bool isReady();

    static void enable5V(bool on);       // EXT_5V_OUT (IR emitter / J5 supply)
    static void shutdown();              // drop rail + PM1 soft power-off (wake via J6 button)

    // Raw PMIC telemetry (mV); return 0 if PM1 unavailable.
    static uint16_t batteryMillivolts();
    static uint16_t inputMillivolts();   // 5VIN (USB / DC)
    static uint16_t fiveVoutMillivolts();
    static bool     onExternalPower();   // VIN present (> BATTERY_VIN_PRESENT_MV)

private:
    static bool ready;
};
