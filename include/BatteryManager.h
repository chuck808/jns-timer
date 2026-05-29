#pragma once

#include <Arduino.h>

/**
 * @file BatteryManager.h
 * @brief Battery state-of-charge / warning logic for the BAT_Carrier.
 *
 * Reworked to read voltages through PowerManager (which owns the single PM1
 * instance and the I2C bus) rather than initialising the PM1 itself. This
 * removes the duplicate Wire.begin / pm1.begin and keeps the rail and the
 * telemetry decoupled — you can leave this disabled for bench testing and the
 * 5V rail still comes up via PowerManager.
 *
 * Thresholds are single-cell LiPo, in millivolts.
 */

// Single-cell LiPo thresholds (mV)
#define BATTERY_VOLTAGE_FULL      4150
#define BATTERY_VOLTAGE_WARNING   3600
#define BATTERY_VOLTAGE_LOW       3400
#define BATTERY_VOLTAGE_CRITICAL  3200
#define BATTERY_HYSTERESIS_MV       50    // stops state flicker at the edges

enum BatteryState {
    BAT_UNKNOWN,
    BAT_CRITICAL,
    BAT_LOW,
    BAT_OK,
    BAT_FULL
};

class BatteryManager {
public:
    static bool init();                 // true if the PM1 (via PowerManager) is available
    static bool update();               // re-read; returns true if state/% changed materially

    static uint16_t getVoltage();       // battery mV
    static uint16_t getInputVoltage();  // VIN (USB/DC) mV
    static uint8_t  getPercentage();    // 0..100
    static BatteryState getState();

    static bool isCharging();
    static const char* getChargingString();

    static bool isCritical();
    static bool isSafeForRace();        // refuse to arm if true == false
    static bool hasWarning();
    static const char* getWarningMessage();

private:
    static uint8_t  percentFromMv(uint16_t mv);
    static BatteryState classify(uint16_t mv);

    static bool          ready;
    static uint16_t      lastMv;
    static uint16_t      lastVin;
    static uint8_t       lastPct;
    static BatteryState  state;
    static unsigned long lastSampleMs;
};
