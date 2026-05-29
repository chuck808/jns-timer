#include "BatteryManager.h"

M5PM1 BatteryManager::pm1;
bool  BatteryManager::pm1Ready = false;

uint16_t       BatteryManager::currentVoltage  = 0;
uint16_t       BatteryManager::smoothedVoltage = 0;
uint16_t       BatteryManager::inputVoltage    = 0;
uint8_t        BatteryManager::percentage      = 0;
BatteryState   BatteryManager::state           = BAT_UNKNOWN;
ChargingStatus BatteryManager::chargingStatus  = CHARGE_UNKNOWN;

unsigned long  BatteryManager::lastUpdate      = 0;
bool           BatteryManager::warningsEnabled = true;
bool           BatteryManager::warningPending  = false;
char           BatteryManager::warningMessage[64] = {0};

uint16_t BatteryManager::voltageBuffer[BATTERY_ADC_SAMPLES] = {0};
uint8_t  BatteryManager::bufferIndex  = 0;
bool     BatteryManager::bufferFilled = false;

bool BatteryManager::criticalWarningShown = false;
bool BatteryManager::lowWarningShown      = false;
bool BatteryManager::warningWarningShown  = false;

unsigned long BatteryManager::totalRuntime = 0;
uint16_t      BatteryManager::minVoltage   = 65535;
uint16_t      BatteryManager::maxVoltage   = 0;

bool BatteryManager::init() {
    Serial.println("BatteryManager: Initialising PMIC battery backend...");

    Wire.begin(BATTERY_I2C_SDA, BATTERY_I2C_SCL, 400000U);

    auto rc = pm1.begin(&Wire, M5PM1_DEFAULT_ADDR, BATTERY_I2C_SDA, BATTERY_I2C_SCL, M5PM1_I2C_FREQ_400K);
    if (rc != M5PM1_OK) {
        Serial.printf("BatteryManager: M5PM1 init failed (%d)\n", (int)rc);
        pm1Ready = false;
        state = BAT_UNKNOWN;
        chargingStatus = CHARGE_UNKNOWN;
        return false;
    }

    pm1Ready = true;

    for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
        voltageBuffer[i] = readBatteryMillivolts();
        delay(8);
    }

    bufferFilled = true;
    totalRuntime = 0;
    lastUpdate   = millis();

    forceUpdate();

    Serial.printf("BatteryManager: Ready — VBAT=%u mV (%u%%) VIN=%u mV [%s] charge=%s\n",
                  smoothedVoltage,
                  percentage,
                  inputVoltage,
                  getStateString(),
                  getChargingString());

    return true;
}

bool BatteryManager::update() {
    if (!pm1Ready) return false;

    unsigned long now = millis();

    if (lastUpdate != 0 && (now - lastUpdate) < BATTERY_UPDATE_INTERVAL) {
        return false;
    }

    if (lastUpdate != 0) {
        totalRuntime += (now - lastUpdate);
    }
    lastUpdate = now;

    uint8_t        oldPercentage = percentage;
    BatteryState   oldState      = state;
    ChargingStatus oldCharging   = chargingStatus;
    uint16_t       oldVin        = inputVoltage;

    smoothVoltage(readBatteryMillivolts());
    currentVoltage = smoothedVoltage;
    inputVoltage   = readInputMillivolts();
    percentage     = voltageToPercentage(smoothedVoltage);

    updateChargingStatus();
    updateState();
    checkWarnings();

    if (smoothedVoltage < minVoltage) minVoltage = smoothedVoltage;
    if (smoothedVoltage > maxVoltage) maxVoltage = smoothedVoltage;

    return (oldPercentage != percentage) ||
           (oldState      != state) ||
           (oldCharging   != chargingStatus) ||
           (oldVin        != inputVoltage);
}

void BatteryManager::forceUpdate() {
    lastUpdate = millis() - BATTERY_UPDATE_INTERVAL;
    update();
}

uint8_t BatteryManager::getPercentage() { return percentage; }
uint16_t BatteryManager::getVoltage() { return currentVoltage; }
uint16_t BatteryManager::getSmoothedVoltage() { return smoothedVoltage; }
uint16_t BatteryManager::getInputVoltage() { return inputVoltage; }
BatteryState BatteryManager::getState() { return state; }
ChargingStatus BatteryManager::getChargingStatus() { return chargingStatus; }
bool BatteryManager::isReady() { return pm1Ready; }

BatteryColor BatteryManager::getColorCode() {
    switch (state) {
        case BAT_CRITICAL:
        case BAT_LOW:
            return BAT_COLOR_RED;
        case BAT_WARNING:
            return BAT_COLOR_YELLOW;
        default:
            return BAT_COLOR_GREEN;
    }
}

bool BatteryManager::isSafeForRace() {
    return pm1Ready && smoothedVoltage >= BATTERY_VOLTAGE_SAFE_START;
}

bool BatteryManager::isCritical() {
    return pm1Ready && state == BAT_CRITICAL;
}

bool BatteryManager::isCharging() {
    return chargingStatus == CHARGE_CHARGING;
}

bool BatteryManager::isFullyCharged() {
    return chargingStatus == CHARGE_FULL || state == BAT_FULL;
}

bool BatteryManager::isExternalPowerPresent() {
    return inputVoltage >= BATTERY_VIN_PRESENT_MV;
}

const char* BatteryManager::getStateString() {
    switch (state) {
        case BAT_CRITICAL: return "Critical";
        case BAT_LOW:      return "Low";
        case BAT_WARNING:  return "Warning";
        case BAT_NORMAL:   return "Normal";
        case BAT_HIGH:     return "High";
        case BAT_FULL:     return "Full";
        default:           return "Unknown";
    }
}

const char* BatteryManager::getChargingString() {
    switch (chargingStatus) {
        case CHARGE_NOT_CHARGING: return "Not charging";
        case CHARGE_CHARGING:     return "Charging";
        case CHARGE_FULL:         return "Full";
        default:                  return "Unknown";
    }
}

uint16_t BatteryManager::getEstimatedRuntime() {
    if (percentage == 0) return 0;
    return (240U * percentage) / 100U;
}

void BatteryManager::setWarningsEnabled(bool enabled) {
    warningsEnabled = enabled;
}

bool BatteryManager::hasWarning() {
    if (warningPending) {
        warningPending = false;
        return true;
    }
    return false;
}

const char* BatteryManager::getWarningMessage() {
    return warningMessage;
}

void BatteryManager::resetStats() {
    totalRuntime         = 0;
    minVoltage           = smoothedVoltage;
    maxVoltage           = smoothedVoltage;
    criticalWarningShown = false;
    lowWarningShown      = false;
    warningWarningShown  = false;
}

String BatteryManager::getDebugInfo() {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "Battery: %umV (%u%%) VIN:%umV [%s] charge:%s min:%u max:%u runtime:%lum safe:%s ready:%s",
             currentVoltage,
             percentage,
             inputVoltage,
             getStateString(),
             getChargingString(),
             minVoltage,
             maxVoltage,
             totalRuntime / 60000UL,
             isSafeForRace() ? "Y" : "N",
             pm1Ready ? "Y" : "N");
    return String(buf);
}

uint16_t BatteryManager::readBatteryMillivolts() {
    if (!pm1Ready) return 0;

    uint16_t mv = 0;
    if (pm1.readVbat(&mv) == M5PM1_OK) {
        return mv;
    }

    return currentVoltage;
}

uint16_t BatteryManager::readInputMillivolts() {
    if (!pm1Ready) return 0;

    uint16_t mv = 0;
    if (pm1.readVin(&mv) == M5PM1_OK) {
        return mv;
    }

    return 0;
}

void BatteryManager::smoothVoltage(uint16_t newVoltage) {
    voltageBuffer[bufferIndex] = newVoltage;
    bufferIndex = (bufferIndex + 1) % BATTERY_ADC_SAMPLES;

    if (bufferIndex == 0) {
        bufferFilled = true;
    }

    uint32_t sum = 0;
    uint8_t count = bufferFilled ? BATTERY_ADC_SAMPLES : bufferIndex;
    if (count == 0) count = 1;

    for (uint8_t i = 0; i < count; ++i) {
        sum += voltageBuffer[i];
    }

    smoothedVoltage = (uint16_t)(sum / count);
}

uint8_t BatteryManager::voltageToPercentage(uint16_t mv) {
    static const uint16_t voltTable[] = {
        3000, 3200, 3400, 3600, 3700, 3800, 3900, 4000, 4100, 4180
    };
    static const uint8_t percTable[] = {
           0,    3,   10,   20,   35,   50,   68,   82,   94,  100
    };

    constexpr size_t count = sizeof(voltTable) / sizeof(voltTable[0]);

    if (mv <= voltTable[0]) return percTable[0];
    if (mv >= voltTable[count - 1]) return percTable[count - 1];

    for (size_t i = 0; i < count - 1; ++i) {
        if (mv < voltTable[i + 1]) {
            float frac = (float)(mv - voltTable[i]) /
                         (float)(voltTable[i + 1] - voltTable[i]);
            float p = percTable[i] + frac * (percTable[i + 1] - percTable[i]);
            return (uint8_t)(p + 0.5f);
        }
    }

    return 100;
}

void BatteryManager::updateChargingStatus() {
    if (!pm1Ready) {
        chargingStatus = CHARGE_UNKNOWN;
        return;
    }

    if (!isExternalPowerPresent()) {
        chargingStatus = CHARGE_NOT_CHARGING;
        return;
    }

    if (smoothedVoltage >= (BATTERY_VOLTAGE_FULL - 20)) {
        chargingStatus = CHARGE_FULL;
    } else {
        chargingStatus = CHARGE_CHARGING;
    }
}

void BatteryManager::updateState() {
    const uint16_t v = smoothedVoltage;
    BatteryState newState = state;

    if (state == BAT_UNKNOWN) {
        if      (v <= BATTERY_VOLTAGE_CRITICAL) newState = BAT_CRITICAL;
        else if (v <= BATTERY_VOLTAGE_LOW)      newState = BAT_LOW;
        else if (v <= BATTERY_VOLTAGE_WARNING)  newState = BAT_WARNING;
        else if (v >= BATTERY_VOLTAGE_FULL)     newState = BAT_FULL;
        else if (v >= BATTERY_VOLTAGE_HIGH)     newState = BAT_HIGH;
        else                                    newState = BAT_NORMAL;
        state = newState;
        return;
    }

    switch (state) {
        case BAT_CRITICAL:
            if (v >= BATTERY_VOLTAGE_LOW + BATTERY_HYSTERESIS_MV) newState = BAT_LOW;
            break;

        case BAT_LOW:
            if (v <= BATTERY_VOLTAGE_CRITICAL - BATTERY_HYSTERESIS_MV) {
                newState = BAT_CRITICAL;
            } else if (v >= BATTERY_VOLTAGE_WARNING + BATTERY_HYSTERESIS_MV) {
                newState = BAT_WARNING;
            }
            break;

        case BAT_WARNING:
            if (v <= BATTERY_VOLTAGE_LOW - BATTERY_HYSTERESIS_MV) {
                newState = BAT_LOW;
            } else if (v >= BATTERY_VOLTAGE_NORMAL + BATTERY_HYSTERESIS_MV) {
                newState = BAT_NORMAL;
            }
            break;

        case BAT_NORMAL:
            if (v <= BATTERY_VOLTAGE_WARNING - BATTERY_HYSTERESIS_MV) {
                newState = BAT_WARNING;
            } else if (v >= BATTERY_VOLTAGE_HIGH + BATTERY_HYSTERESIS_MV) {
                newState = BAT_HIGH;
            }
            break;

        case BAT_HIGH:
            if (v <= BATTERY_VOLTAGE_NORMAL - BATTERY_HYSTERESIS_MV) {
                newState = BAT_NORMAL;
            } else if (v >= BATTERY_VOLTAGE_FULL + BATTERY_HYSTERESIS_MV) {
                newState = BAT_FULL;
            }
            break;

        case BAT_FULL:
            if (v <= BATTERY_VOLTAGE_HIGH - BATTERY_HYSTERESIS_MV) {
                newState = BAT_HIGH;
            }
            break;

        default:
            break;
    }

    state = newState;
}

void BatteryManager::checkWarnings() {
    if (!warningsEnabled) return;

    if (state == BAT_CRITICAL && !criticalWarningShown) {
        snprintf(warningMessage, sizeof(warningMessage),
                 "Battery critical: %umV (%u%%)", smoothedVoltage, percentage);
        warningPending = true;
        criticalWarningShown = true;
    } else if (state == BAT_LOW && !lowWarningShown) {
        snprintf(warningMessage, sizeof(warningMessage),
                 "Battery low: %umV (%u%%)", smoothedVoltage, percentage);
        warningPending = true;
        lowWarningShown = true;
    } else if (state == BAT_WARNING && !warningWarningShown) {
        snprintf(warningMessage, sizeof(warningMessage),
                 "Battery warning: %umV (%u%%)", smoothedVoltage, percentage);
        warningPending = true;
        warningWarningShown = true;
    }

    if (state == BAT_NORMAL || state == BAT_HIGH || state == BAT_FULL) {
        warningWarningShown = false;
    }
    if (state == BAT_HIGH || state == BAT_FULL) {
        lowWarningShown = false;
        criticalWarningShown = false;
    }
}