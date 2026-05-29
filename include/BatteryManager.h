#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <M5PM1.h>
#include "pin_config.h"

// -----------------------------------------------------------------------------
// Battery voltage thresholds (millivolts)
// Conservative 1S Li-Ion thresholds.
// -----------------------------------------------------------------------------
#define BATTERY_VOLTAGE_FULL         4180
#define BATTERY_VOLTAGE_HIGH         3950
#define BATTERY_VOLTAGE_NORMAL       3700
#define BATTERY_VOLTAGE_WARNING      3600
#define BATTERY_VOLTAGE_LOW          3400
#define BATTERY_VOLTAGE_CRITICAL     3200
#define BATTERY_VOLTAGE_EMPTY        3000
#define BATTERY_VOLTAGE_SAFE_START   3500

#define BATTERY_HYSTERESIS_MV        40
#define BATTERY_ADC_SAMPLES          16
#define BATTERY_UPDATE_INTERVAL      2000

#ifndef BATTERY_I2C_SDA
#define BATTERY_I2C_SDA 48
#endif

#ifndef BATTERY_I2C_SCL
#define BATTERY_I2C_SCL 47
#endif

#ifndef BATTERY_VIN_PRESENT_MV
#define BATTERY_VIN_PRESENT_MV 4300
#endif

enum BatteryState {
    BAT_UNKNOWN = 0,
    BAT_CRITICAL,
    BAT_LOW,
    BAT_WARNING,
    BAT_NORMAL,
    BAT_HIGH,
    BAT_FULL
};

enum ChargingStatus {
    CHARGE_UNKNOWN = 0,
    CHARGE_NOT_CHARGING,
    CHARGE_CHARGING,
    CHARGE_FULL
};

enum BatteryColor {
    BAT_COLOR_RED,
    BAT_COLOR_YELLOW,
    BAT_COLOR_GREEN,
    BAT_COLOR_BLUE
};

class BatteryManager {
public:
    static bool init();
    static bool update();

    static uint8_t        getPercentage();
    static uint16_t       getVoltage();
    static uint16_t       getSmoothedVoltage();
    static uint16_t       getInputVoltage();
    static BatteryState   getState();
    static ChargingStatus getChargingStatus();
    static BatteryColor   getColorCode();

    static bool isSafeForRace();
    static bool isCritical();
    static bool isCharging();
    static bool isFullyCharged();
    static bool isExternalPowerPresent();
    static bool isReady();

    static const char* getStateString();
    static const char* getChargingString();

    static uint16_t getEstimatedRuntime();

    static void forceUpdate();
    static void setWarningsEnabled(bool enabled);
    static bool hasWarning();
    static const char* getWarningMessage();
    static void resetStats();
    static String getDebugInfo();

private:
    static uint16_t readBatteryMillivolts();
    static uint16_t readInputMillivolts();
    static void     smoothVoltage(uint16_t newVoltage);
    static uint8_t  voltageToPercentage(uint16_t mv);
    static void     updateState();
    static void     updateChargingStatus();
    static void     checkWarnings();

    static M5PM1         pm1;
    static bool          pm1Ready;

    static uint16_t       currentVoltage;
    static uint16_t       smoothedVoltage;
    static uint16_t       inputVoltage;
    static uint8_t        percentage;
    static BatteryState   state;
    static ChargingStatus chargingStatus;

    static unsigned long lastUpdate;
    static bool          warningsEnabled;
    static bool          warningPending;
    static char          warningMessage[64];

    static uint16_t voltageBuffer[BATTERY_ADC_SAMPLES];
    static uint8_t  bufferIndex;
    static bool     bufferFilled;

    static bool criticalWarningShown;
    static bool lowWarningShown;
    static bool warningWarningShown;

    static unsigned long totalRuntime;
    static uint16_t      minVoltage;
    static uint16_t      maxVoltage;
};