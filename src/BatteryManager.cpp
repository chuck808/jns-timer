/**
 * @file BatteryManager.cpp
 * @brief SoC / warning logic on top of PowerManager's PMIC reads.
 */

#include "BatteryManager.h"
#include "PowerManager.h"
#include <esp_log.h>

static const char* TAG = "BatteryManager";

bool          BatteryManager::ready        = false;
uint16_t      BatteryManager::lastMv       = 0;
uint16_t      BatteryManager::lastVin      = 0;
uint8_t       BatteryManager::lastPct      = 0;
BatteryState  BatteryManager::state        = BAT_UNKNOWN;
unsigned long BatteryManager::lastSampleMs = 0;

#define BATTERY_SAMPLE_INTERVAL_MS  2000

/*===========================================
 * LiPo discharge curve (open-ish circuit, mV -> %)
 * Coarse piecewise LUT, linearly interpolated.
 *===========================================*/

struct SocPoint { uint16_t mv; uint8_t pct; };
static const SocPoint kSoc[] = {
    {4200, 100}, {4100, 95}, {4000, 88}, {3900, 78}, {3800, 65},
    {3700,  50}, {3600, 35}, {3500, 22}, {3400, 12}, {3300,  5}, {3200, 0}
};

uint8_t BatteryManager::percentFromMv(uint16_t mv) {
    if (mv >= kSoc[0].mv) return 100;
    const uint8_t n = sizeof(kSoc) / sizeof(kSoc[0]);
    if (mv <= kSoc[n - 1].mv) return 0;
    for (uint8_t i = 0; i < n - 1; i++) {
        if (mv <= kSoc[i].mv && mv > kSoc[i + 1].mv) {
            uint16_t hiMv = kSoc[i].mv,     loMv = kSoc[i + 1].mv;
            uint8_t  hiPc = kSoc[i].pct,    loPc = kSoc[i + 1].pct;
            return loPc + (uint8_t)(((uint32_t)(mv - loMv) * (hiPc - loPc)) / (hiMv - loMv));
        }
    }
    return 0;
}

BatteryState BatteryManager::classify(uint16_t mv) {
    // Hysteresis: only leave a low state once we've risen a margin above it,
    // so readings sitting on a threshold don't chatter.
    if (state == BAT_CRITICAL && mv <= BATTERY_VOLTAGE_CRITICAL + BATTERY_HYSTERESIS_MV)
        return BAT_CRITICAL;
    if (state == BAT_LOW && mv <= BATTERY_VOLTAGE_LOW + BATTERY_HYSTERESIS_MV)
        return (mv < BATTERY_VOLTAGE_CRITICAL) ? BAT_CRITICAL : BAT_LOW;

    if (mv < BATTERY_VOLTAGE_CRITICAL) return BAT_CRITICAL;
    if (mv < BATTERY_VOLTAGE_LOW)      return BAT_LOW;
    if (mv >= BATTERY_VOLTAGE_FULL)    return BAT_FULL;
    return BAT_OK;
}

/*===========================================
 * LIFECYCLE
 *===========================================*/

bool BatteryManager::init() {
    ready = PowerManager::isReady();   // PowerManager owns the bus + PM1
    if (!ready) {
        ESP_LOGI(TAG, "PM1 not ready (PowerManager not initialised)");
        return false;
    }
    update();
    ESP_LOGI(TAG, "%u%% (%umV)  VIN=%umV  %s",
                  lastPct, lastMv, lastVin, getChargingString());
    return true;
}

bool BatteryManager::update() {
    if (!ready) return false;

    unsigned long now = millis();
    if (now - lastSampleMs < BATTERY_SAMPLE_INTERVAL_MS && lastSampleMs != 0) return false;
    lastSampleMs = now;

    uint16_t mv  = PowerManager::batteryMillivolts();
    uint16_t vin = PowerManager::inputMillivolts();
    if (mv == 0) return false;   // bad read; keep previous

    BatteryState newState = classify(mv);
    uint8_t      newPct   = percentFromMv(mv);

    bool changed = (newState != state) ||
                   (newPct > lastPct ? newPct - lastPct : lastPct - newPct) >= 2;

    lastMv  = mv;
    lastVin = vin;
    lastPct = newPct;
    state   = newState;

    return changed;
}

/*===========================================
 * QUERIES
 *===========================================*/

uint16_t BatteryManager::getVoltage()      { return lastMv; }
uint16_t BatteryManager::getInputVoltage() { return lastVin; }
uint8_t  BatteryManager::getPercentage()   { return lastPct; }
BatteryState BatteryManager::getState()    { return state; }

bool BatteryManager::isCharging() {
    // No dedicated charge-status read exposed; infer from external power present
    // while the pack isn't already full.
    return PowerManager::onExternalPower() && state != BAT_FULL;
}

const char* BatteryManager::getChargingString() {
    if (!PowerManager::onExternalPower()) return "ON BATTERY";
    return (state == BAT_FULL) ? "EXT (full)" : "CHARGING";
}

bool BatteryManager::isCritical() { return state == BAT_CRITICAL; }

bool BatteryManager::isSafeForRace() {
    // Safe to arm on external power, or when comfortably above the low threshold.
    if (PowerManager::onExternalPower()) return true;
    return (state == BAT_OK || state == BAT_FULL);
}

bool BatteryManager::hasWarning() {
    if (PowerManager::onExternalPower()) return false;
    return lastMv > 0 && lastMv <= BATTERY_VOLTAGE_WARNING;
}

const char* BatteryManager::getWarningMessage() {
    switch (state) {
        case BAT_CRITICAL: return "CRITICAL battery — power off imminent";
        case BAT_LOW:      return "LOW battery — charge soon";
        default:           return (lastMv <= BATTERY_VOLTAGE_WARNING) ? "Battery getting low" : "Battery OK";
    }
}
