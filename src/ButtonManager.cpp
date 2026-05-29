/**
 * @file ButtonManager.cpp
 * @brief Button input manager implementation — Stamp S3 port
 *
 * Only change from Waveshare build: default pin GPIO40 → GPIO0 (BTN_INPUT).
 * All detection logic, timing, and callback mechanism is identical.
 *
 * @version 2.0.0  (Stamp S3 port)
 * @date 2025
 */

#include "ButtonManager.h"
#include <esp_log.h>

static const char* TAG = "ButtonManager";

// Static member initialisation
uint8_t ButtonManager::buttonPin  = BTN_INPUT;  // GPIO0 — Stamp S3 onboard button
bool    ButtonManager::activeLevel = LOW;
bool    ButtonManager::enabled     = true;

bool ButtonManager::currentState = false;
bool ButtonManager::lastState    = false;
bool ButtonManager::stableState  = false;

unsigned long ButtonManager::lastDebounceTime = 0;
unsigned long ButtonManager::lastClickTime    = 0;
unsigned long ButtonManager::pressStartTime   = 0;

uint32_t ButtonManager::debounceDelay      = DEBOUNCE_DELAY_MS;
uint32_t ButtonManager::doubleClickWindow  = DOUBLE_CLICK_WINDOW_MS;
uint32_t ButtonManager::longPressTime      = LONG_PRESS_TIME_MS;
uint32_t ButtonManager::ultraLongPressTime = ULTRA_LONG_PRESS_TIME_MS;

ButtonEvent   ButtonManager::lastEvent              = BTN_NONE;
bool          ButtonManager::longPressDetected      = false;
bool          ButtonManager::ultraLongPressDetected = false;
bool          ButtonManager::clickPending           = false;
bool          ButtonManager::doubleClickPossible    = false;

ButtonCallback ButtonManager::eventCallback = nullptr;

/*===========================================
 * PUBLIC — LIFECYCLE
 *===========================================*/

void ButtonManager::init(uint8_t pin, bool activeLow) {
    buttonPin   = pin;
    activeLevel = activeLow ? LOW : HIGH;

    pinMode(buttonPin, activeLow ? INPUT_PULLUP : INPUT);

    currentState = readButton();
    lastState    = currentState;
    stableState  = currentState;

    ESP_LOGI(TAG, "Initialised on GPIO%d (active %s)",
                  buttonPin, activeLow ? "LOW" : "HIGH");
}

/*===========================================
 * PUBLIC — UPDATE (call every loop)
 *===========================================*/

void ButtonManager::update() {
    if (!enabled) return;

    unsigned long now     = millis();
    bool          reading = readButton();

    if (reading != lastState) {
        lastDebounceTime = now;
    }

    if ((now - lastDebounceTime) > debounceDelay) {
        if (reading != stableState) {
            stableState = reading;
            if (stableState) processButtonPress();
            else             processButtonRelease();
        }
    }

    // Long press detection while held
    if (stableState && !longPressDetected) {
        if ((now - pressStartTime) >= longPressTime) {
            longPressDetected = true;
            triggerEvent(BTN_LONG_PRESS);
        }
    }

    // Ultra-long press detection while held
    if (stableState && longPressDetected && !ultraLongPressDetected) {
        if ((now - pressStartTime) >= ultraLongPressTime) {
            ultraLongPressDetected = true;
            triggerEvent(BTN_ULTRA_LONG_PRESS);
        }
    }

    // Resolve pending single click after double-click window expires
    if (clickPending && !stableState) {
        if ((now - lastClickTime) >= doubleClickWindow) {
            clickPending           = false;
            doubleClickPossible    = false;
            triggerEvent(BTN_CLICK);
        }
    }

    lastState = reading;
}

/*===========================================
 * PRIVATE — PRESS / RELEASE HANDLERS
 *===========================================*/

void ButtonManager::processButtonPress() {
    unsigned long now = millis();
    pressStartTime         = now;
    longPressDetected      = false;
    ultraLongPressDetected = false;

    if (doubleClickPossible && (now - lastClickTime) < doubleClickWindow) {
        clickPending        = false;
        doubleClickPossible = false;
        triggerEvent(BTN_DOUBLE_CLICK);
    } else {
        clickPending        = true;
        doubleClickPossible = true;
        lastClickTime       = now;
    }

    ESP_LOGI(TAG, "Pressed");
}

void ButtonManager::processButtonRelease() {
    unsigned long pressDuration = millis() - pressStartTime;

    if (ultraLongPressDetected) {
        triggerEvent(BTN_ULTRA_LONG_RELEASE);
        ultraLongPressDetected = false;
        longPressDetected      = false;
        clickPending           = false;
        doubleClickPossible    = false;
    } else if (longPressDetected) {
        triggerEvent(BTN_LONG_RELEASE);
        longPressDetected   = false;
        clickPending        = false;
        doubleClickPossible = false;
    }
    // Single click resolved in update() after double-click window

    ESP_LOGI(TAG, "Released (held %lums)", pressDuration);
}

/*===========================================
 * PRIVATE — HELPERS
 *===========================================*/

bool ButtonManager::readButton() {
    return (digitalRead(buttonPin) == activeLevel);
}

void ButtonManager::triggerEvent(ButtonEvent event) {
    lastEvent = event;

    const char* name;
    switch (event) {
        case BTN_CLICK:              name = "CLICK";              break;
        case BTN_DOUBLE_CLICK:       name = "DOUBLE_CLICK";       break;
        case BTN_LONG_PRESS:         name = "LONG_PRESS";         break;
        case BTN_LONG_RELEASE:       name = "LONG_RELEASE";       break;
        case BTN_ULTRA_LONG_PRESS:   name = "ULTRA_LONG_PRESS";   break;
        case BTN_ULTRA_LONG_RELEASE: name = "ULTRA_LONG_RELEASE"; break;
        default:                     name = "NONE";               break;
    }
    ESP_LOGI(TAG, "%s", name);

    if (eventCallback) eventCallback(event);
}

/*===========================================
 * PUBLIC — ACCESSORS & CONFIGURATION
 *===========================================*/

void ButtonManager::setCallback(ButtonCallback callback) {
    eventCallback = callback;
    ESP_LOGI(TAG, "Callback registered");
}

bool          ButtonManager::isPressed()       { return stableState; }
unsigned long ButtonManager::getPressDuration(){ return stableState ? millis() - pressStartTime : 0; }
ButtonEvent   ButtonManager::getLastEvent()    { return lastEvent; }
void          ButtonManager::clearEvent()      { lastEvent = BTN_NONE; }

void ButtonManager::setDebounceDelay(uint32_t ms)      { debounceDelay      = ms; }
void ButtonManager::setDoubleClickWindow(uint32_t ms)  { doubleClickWindow  = ms; }
void ButtonManager::setLongPressTime(uint32_t ms)      { longPressTime      = ms; }
void ButtonManager::setUltraLongPressTime(uint32_t ms) { ultraLongPressTime = ms; }

void ButtonManager::enable()    { enabled = true;  ESP_LOGI(TAG, "Enabled");  }
void ButtonManager::disable()   { enabled = false; ESP_LOGI(TAG, "Disabled"); }
bool ButtonManager::isEnabled() { return enabled; }
