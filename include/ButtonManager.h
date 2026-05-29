/**
 * @file ButtonManager.h
 * @brief Button input manager with multi-tier press detection
 *
 * Hardware:
 *   M5Stack Stamp S3 — onboard button on GPIO0 (active LOW, INPUT_PULLUP)
 *
 * Changes vs Waveshare build:
 *   - Default pin: GPIO40 → GPIO0
 *   - All timing constants sourced from pin_config.h
 *   - Logic unchanged
 *
 * Features:
 *   - Single click
 *   - Double click
 *   - Long press (3s)
 *   - Ultra-long press (6s)
 *   - Debouncing
 *   - Callback-based event dispatch
 *
 * @version 2.0.0  (Stamp S3 port)
 * @date 2025
 */

#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include <Arduino.h>
#include <functional>
#include "pin_config.h"

enum ButtonEvent {
    BTN_NONE,
    BTN_CLICK,
    BTN_DOUBLE_CLICK,
    BTN_LONG_PRESS,
    BTN_LONG_RELEASE,
    BTN_ULTRA_LONG_PRESS,
    BTN_ULTRA_LONG_RELEASE
};

typedef std::function<void(ButtonEvent)> ButtonCallback;

class ButtonManager {
public:
    /**
     * Initialise button manager.
     * Defaults to BTN_INPUT (GPIO0) active LOW — matches Stamp S3 onboard button.
     */
    static void init(uint8_t pin = BTN_INPUT, bool activeLow = true);

    static void update();                               // Call every loop iteration
    static void setCallback(ButtonCallback callback);

    static bool          isPressed();
    static unsigned long getPressDuration();
    static ButtonEvent   getLastEvent();
    static void          clearEvent();

    static void setDebounceDelay(uint32_t ms);
    static void setDoubleClickWindow(uint32_t ms);
    static void setLongPressTime(uint32_t ms);
    static void setUltraLongPressTime(uint32_t ms);

    static void enable();
    static void disable();
    static bool isEnabled();

private:
    static uint8_t buttonPin;
    static bool    activeLevel;
    static bool    enabled;

    static bool currentState;
    static bool lastState;
    static bool stableState;

    static unsigned long lastDebounceTime;
    static unsigned long lastClickTime;
    static unsigned long pressStartTime;

    static uint32_t debounceDelay;
    static uint32_t doubleClickWindow;
    static uint32_t longPressTime;
    static uint32_t ultraLongPressTime;

    static ButtonEvent lastEvent;
    static bool        longPressDetected;
    static bool        ultraLongPressDetected;
    static bool        clickPending;
    static bool        doubleClickPossible;

    static ButtonCallback eventCallback;

    static bool readButton();
    static void processButtonPress();
    static void processButtonRelease();
    static void triggerEvent(ButtonEvent event);
};

#endif // BUTTON_MANAGER_H
