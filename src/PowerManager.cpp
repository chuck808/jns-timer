/**
 * @file PowerManager.cpp
 * @brief M5PM1 PMIC control for the BAT_Carrier board.
 *
 * API names follow the M5PM1 Arduino library (m5stack/M5PM1 @ ^1.0.6).
 * If a symbol name differs in your installed version, check src/M5PM1.h —
 * the M5PM1_* enums are the only things likely to vary.
 */

#include "PowerManager.h"
#include "pin_config.h"

#include <Wire.h>
#include <M5PM1.h>

/*===========================================
 * PM1 instance + GPIO role map
 *===========================================*/

static M5PM1 pm1;
bool PowerManager::ready = false;

#define PM1_5V_EN_GPIO     M5PM1_GPIO_NUM_1   // 5VOUT_EN  -> EXT_5V_OUT enable
#define PM1_CHG_PROG_GPIO  M5PM1_GPIO_NUM_3   // PY_G3_CHG_PROG (low = 650mA, float = 200mA)
#define PM1_WAKE_GPIO      M5PM1_GPIO_NUM_4   // PY_G4 / WAKE_IN_N

// Set to 1 ONLY if the cell can safely take 650mA charge. Default 200mA.
#define PM1_CHARGE_650MA   0

/*===========================================
 * LIFECYCLE
 *===========================================*/

bool PowerManager::begin() {
    Wire.end();
    Wire.begin(BATTERY_I2C_SDA, BATTERY_I2C_SCL, 100000U);

    m5pm1_err_t err = pm1.begin(&Wire, M5PM1_DEFAULT_ADDR,
                                BATTERY_I2C_SDA, BATTERY_I2C_SCL,
                                M5PM1_I2C_FREQ_100K);
    if (err != M5PM1_OK) {
        Serial.printf("PowerManager: PM1 init FAILED (%d) — 5V rail OFF, emitter will not work\n", err);
        ready = false;
        return false;
    }
    ready = true;
    Serial.println("PowerManager: PM1 init OK");

    // --- Charging ---------------------------------------------------------
    pm1.setChargeEnable(true);
#if PM1_CHARGE_650MA
    pm1.gpioSetMode(PM1_CHG_PROG_GPIO, M5PM1_GPIO_MODE_OUTPUT);
    pm1.gpioSetPull(PM1_CHG_PROG_GPIO, M5PM1_GPIO_PULL_NONE);
    pm1.gpioSetOutput(PM1_CHG_PROG_GPIO, false);            // low = 650mA
    Serial.println("PowerManager: charge current 650mA");
#else
    pm1.gpioSetMode(PM1_CHG_PROG_GPIO, M5PM1_GPIO_MODE_INPUT);
    pm1.gpioSetPull(PM1_CHG_PROG_GPIO, M5PM1_GPIO_PULL_NONE); // float = 200mA
    Serial.println("PowerManager: charge current 200mA");
#endif

    // --- EXT_5V_OUT on (powers IR emitter J1 + external LED J5) -----------
    enable5V(true);

    // --- WAKE button: PY_G4, internal pull-up, falling edge ---------------
    // Requires R5 (external pull-down) to be UNPOPULATED.
    pm1.irqClearGpioAll();
    pm1.irqClearSysAll();
    pm1.irqClearBtnAll();
    pm1.irqSetGpioMaskAll(M5PM1_IRQ_MASK_ENABLE);
    pm1.irqSetSysMaskAll(M5PM1_IRQ_MASK_ENABLE);
    pm1.irqSetBtnMaskAll(M5PM1_IRQ_MASK_ENABLE);
    pm1.irqSetGpioMask(M5PM1_IRQ_GPIO4, M5PM1_IRQ_MASK_DISABLE);   // unmask wake source

    pm1.gpioSetMode(PM1_WAKE_GPIO, M5PM1_GPIO_MODE_INPUT);
    pm1.gpioSetPull(PM1_WAKE_GPIO, M5PM1_GPIO_PULL_UP);
    pm1.gpioSetWakeEnable(PM1_WAKE_GPIO, true);
    pm1.gpioSetWakeEdge(PM1_WAKE_GPIO, M5PM1_GPIO_WAKE_FALLING);

    Serial.println("PowerManager: 5V rail ON, charging enabled, wake armed on PY_G4");
    return true;
}

bool PowerManager::isReady() { return ready; }

/*===========================================
 * RAILS
 *===========================================*/

void PowerManager::enable5V(bool on) {
    if (!ready) return;
    pm1.gpioSetFunc(PM1_5V_EN_GPIO, M5PM1_GPIO_FUNC_GPIO);
    pm1.gpioSetMode(PM1_5V_EN_GPIO, M5PM1_GPIO_MODE_OUTPUT);
    pm1.gpioSetDrive(PM1_5V_EN_GPIO, M5PM1_GPIO_DRIVE_PUSHPULL);
    pm1.gpioSetOutput(PM1_5V_EN_GPIO, on);
}

/*===========================================
 * SOFT POWER-OFF
 * Drops to PM1 L1 (~13uA). Wakes when J6 pulls PY_G4 low (falling edge),
 * which re-powers the ESP32 and reboots into setup().
 *===========================================*/

void PowerManager::shutdown() {
    Serial.println("PowerManager: shutting down — press the ON button to wake");
    delay(50);
    if (ready) {
        enable5V(false);     // cut emitter / J5 supply first
        delay(20);
        pm1.shutdown();      // ESP32 powers off here; carrier stops with it
    }
    // PM1 absent: nothing can latch us off, so halt visibly.
    delay(100);
}

/*===========================================
 * TELEMETRY  (raw PMIC reads — used by the 'D' debug command and,
 * optionally, by BatteryManager when you re-enable it)
 *===========================================*/

uint16_t PowerManager::batteryMillivolts() {
    uint16_t mv = 0;
    if (ready) pm1.readVbat(&mv);
    return mv;
}

uint16_t PowerManager::inputMillivolts() {
    uint16_t mv = 0;
    if (ready) pm1.readVin(&mv);
    return mv;
}

uint16_t PowerManager::fiveVoutMillivolts() {
    uint16_t mv = 0;
    if (ready) pm1.read5VInOut(&mv);
    return mv;
}

bool PowerManager::onExternalPower() {
    return inputMillivolts() > BATTERY_VIN_PRESENT_MV;
}
