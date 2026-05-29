/**
 * @file main.cpp
 * @brief JNS Timer — IR Beam-Break Timer for BMX Training
 *
 * Hardware: M5Stack Stamp S3 on StampS3 GroveBreakOut (SKU:A144)
 * Sensor:   TSSP77038 demodulated IR receiver (GPIO7) + discrete 38kHz emitter (GPIO5)
 *
 * Removed vs Waveshare/E3Z build:
 *   - LVGL / display / Arduino_GFX (no screen)
 *   - UIManager (replaced by FastLED LED state machine)
 *   - Buzzer / tone functions (no buzzer on this hardware)
 *   - PIN_OUTPUT / SYS_EN power latch (hardware switch on GroveBreakOut)
 *
 * LED colour convention (WS2812B on GPIO21):
 *   Blue        — booting / settling
 *   White       — alignment (beam not yet stable)
 *   Green       — standby (beam intact, awaiting arm)  slow heartbeat dip
 *   Purple      — armed / active (watching for crossing)
 *   Red         — triggered (beam broken / valid crossing)
 *   Yellow      — fault (beam lost while armed)
 *   Cyan        — BLE discovery / connecting
 *   Orange      — low / warning battery
 *   Blue flash  — ESP-NOW connection timeout
 *
 * Protocol (unchanged — controller compatibility maintained):
 *   processControl from controller:
 *       -1 : Sync / Reset / Activate
 *        1 : Time sync (syncTime)
 *   Messages from timer to controller:
 *       timerControl   =  2 : Breakbeam event
 *       processControl = -2 : Ping response
 *       processControl = -3 : Keep-alive
 *       processControl = -4 : Fault / error condition
 *
 * @version 2.0.0  (Stamp S3 port)
 * @date 2025
 */

#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>

#include "pin_config.h"
#include "ButtonManager.h"
#include "BatteryManager.h"
#include "SensorManager.h"
#include "PowerManager.h"
//#include "version.h"

/*===========================================
 * CONFIGURATION
 *===========================================*/

#define TIMER_NAME  "JNS_Timing"

#define DEBUG_MODE  1
#if DEBUG_MODE
    #define DEBUG_PRINT(x)          Serial.print(x)
    #define DEBUG_PRINTLN(x)        Serial.println(x)
    #define DEBUG_PRINTF(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(fmt, ...)
#endif

// Connection monitoring
#define CONNECTION_CHECK_INTERVAL   5000
#define CONNECTION_TIMEOUT          15000
#define RESET_DELAY_MS              1000

// BLE
#define SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX  "cba1e466-344c-4be3-ab3f-189f80dd7518"
#define MAX_ESP_NOW_INIT_RETRIES 5

// Timing
#define DISCOVERY_TIMEOUT_MS        30000
#define BOOT_SETTLE_MS              2000
#define WATCHDOG_TIMEOUT_SEC        10

// Battery
#define BATTERY_WARNING_CHECK_INTERVAL  5000

// Power-off hold detection
#define POWER_OFF_THRESHOLD_MS      6000
#define POWER_OFF_WARNING_START_MS  4000

// LED
#define LED_HEARTBEAT_INTERVAL_MS   2000    // Pulse period when in standby/green
#define LED_HEARTBEAT_DIP_MS        120     // Duration of heartbeat dip
#define LED_TRIGGER_HOLD_MS         1500    // How long red stays after trigger
#define LED_FAULT_FLASH_INTERVAL_MS 300     // Yellow flash period during fault
#define LED_BLE_FLASH_INTERVAL_MS   400     // Cyan flash during BLE discovery
#define LED_TIMEOUT_FLASH_MS        500     // Blue flash on connection timeout

/*===========================================
 * TYPE DEFINITIONS
 *===========================================*/

typedef struct struct_message {
    unsigned long syncTime;
    int processControl;
    int lightControl;
    int gateControl;
    int timerControl;
    float speedReading;     // Always 0 — retained for protocol compatibility
    int deviceIndex;
} struct_message;

enum DeviceMode {
    MODE_OPERATIONAL,
    MODE_WAITING,
    MODE_DISCOVERING
};

// Internal LED states — drives setLED() in the main loop
enum LedState {
    LED_BOOT,           // Blue — solid during boot
    LED_ALIGNING,       // White — beam not yet stable
    LED_STANDBY,        // Green — beam intact, heartbeat
    LED_ARMED,          // Purple — watching for crossing
    LED_TRIGGERED,      // Red — valid crossing detected
    LED_FAULT,          // Yellow flash — beam lost while armed
    LED_DISCOVERING,    // Cyan flash — BLE scanning
    LED_WAITING,        // Slow blue pulse — not paired
    LED_TIMEOUT,        // Blue flash — ESP-NOW timeout
    LED_BATTERY_LOW,    // Orange — battery warning override
    LED_OFF
};

/*===========================================
 * FASTLED
 *===========================================*/

CRGB leds[RGB_LED_COUNT];

/*===========================================
 * GLOBAL VARIABLES
 *===========================================*/

// ESP-NOW
Preferences        preferences;
esp_now_peer_info_t peerInfo = {};
uint8_t            macController[6];
String             globalDeviceName = "";
struct_message     incomingMessage;
struct_message     outgoingMessage;
unsigned long      lastSuccessfulTransmission = 0;
bool               espnowHandshakeComplete    = false;
unsigned long      syncedTime                 = 0;

// BLE
BLEScan*            pBLEScan   = nullptr;
BLEClient*          pBLEClient = nullptr;
BLEAdvertisedDevice _advertisedDevice;
static BLERemoteCharacteristic* pRemoteCharacteristicTX = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicRX = nullptr;
bool deviceFound     = false;
bool clientConnected = false;

// Application state
DeviceMode    deviceMode         = MODE_OPERATIONAL;
unsigned long discoveryStartTime = 0;

// Session tracking
uint16_t      sessionRuns      = 0;
unsigned long lastBreakbeamTime = 0;

// Post-trigger reset lockout
bool          resetLockoutActive  = false;
unsigned long resetLockoutStart   = 0;

// Power-off sequence
bool          powerOffSequenceActive = false;
unsigned long powerOffStartTime      = 0;

// Pairing progress (button-hold feedback via LED)
bool          pairingProgressActive = false;
unsigned long pairingPressStart     = 0;

// Battery
unsigned long lastBatteryWarningCheck = 0;

// LED state machine
LedState      currentLedState     = LED_BOOT;
LedState      requestedLedState   = LED_BOOT;
unsigned long ledStateEnteredAt   = 0;
unsigned long lastHeartbeat       = 0;
unsigned long triggerLedStart     = 0;
bool          ledFlashPhase       = false;
unsigned long lastFlashToggle     = 0;

// Connection timeout flash tracking
bool          connTimeoutActive   = false;

/*===========================================
 * FORWARD DECLARATIONS
 *===========================================*/

// Sensor
void setupSensor();

// LED
void setLED(CRGB colour, uint8_t brightness = 180);
void applyLedState(LedState state);
void updateLED();
void requestLedState(LedState state);

// Communication
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void sendBreakbeamSignal();
void sendPingResponse(const uint8_t *mac_addr, int deviceIndex);
void sendFaultNotification();
void checkESPNowConnection();

// BLE
void setupBLE();
bool connectToDevice();
void checkAndInitESPNow();

// Preferences
void storeDeviceInfoInPreferences(const uint8_t* macAddress, const String& deviceName);
void clearPreferences();
bool loadStoredPreferences();
void executeUnpair();

// Button
void handleButtonEvent(ButtonEvent event);
void updatePowerOffSequence();
void updatePairingProgress();

// Battery
void checkBatteryWarnings();
bool isBatterySafeForRace();

/*===========================================
 * BLE CALLBACK
 *===========================================*/

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (clientConnected) return;
        DEBUG_PRINTF("BLE: Found device: %s\n", advertisedDevice.getName().c_str());
        if (advertisedDevice.haveServiceUUID()) {
            String serviceUUID = advertisedDevice.getServiceUUID().toString().c_str();
            if (serviceUUID == SERVICE_UUID) {
                DEBUG_PRINTLN("BLE: Match — gate controller found");
                globalDeviceName  = advertisedDevice.getName().c_str();
                _advertisedDevice = advertisedDevice;
                deviceFound       = true;
            }
        }
    }
};

/*===========================================
 * LED FUNCTIONS
 *===========================================*/

void setLED(CRGB colour, uint8_t brightness) {
    FastLED.setBrightness(brightness);
    leds[0] = colour;
    FastLED.show();
}

void requestLedState(LedState state) {
    if (requestedLedState == state) return;
    requestedLedState = state;
    ledStateEnteredAt = millis();
    ledFlashPhase     = false;
    lastFlashToggle   = millis();
}

void updateLED() {
    unsigned long now = millis();

    // Battery warning overrides everything except boot
    // TEMPORARILY DISABLED FOR TESTING
    // if (currentLedState != LED_BOOT) {
    //     BatteryState bs = BatteryManager::getState();
    //     bool batteryWarn =
    //         (bs == BAT_CRITICAL) ||
    //         (bs == BAT_LOW);

    //     if (batteryWarn) {
    //         if (requestedLedState != LED_BATTERY_LOW) {
    //             requestLedState(LED_BATTERY_LOW);
    //         }
    //     }
    // }

    currentLedState = requestedLedState;

    switch (currentLedState) {

        case LED_BOOT:
            setLED(CRGB::Blue, 120);
            break;

        case LED_ALIGNING:
            // White solid — emitter running, waiting for stable beam
            setLED(CRGB::White, 80);
            break;

        case LED_STANDBY: {
            // Green with slow heartbeat dip
            if (now - lastHeartbeat >= LED_HEARTBEAT_INTERVAL_MS) {
                setLED(CRGB::Green, 30);
                if (now - lastHeartbeat >= LED_HEARTBEAT_INTERVAL_MS + LED_HEARTBEAT_DIP_MS) {
                    lastHeartbeat = now;
                }
            } else {
                setLED(CRGB::Green, 160);
            }
            break;
        }

        case LED_ARMED:
            // Purple solid — armed, watching
            setLED(CRGB(128, 0, 128), 160);
            break;

        case LED_TRIGGERED:
            // Red solid — hold for LED_TRIGGER_HOLD_MS then drop to standby
            // (controller drives re-arm via processControl=-1, not the timer)
            setLED(CRGB::Red, 200);
            if (now - triggerLedStart >= LED_TRIGGER_HOLD_MS) {
                requestLedState(LED_STANDBY);
                lastHeartbeat = millis();
            }
            break;

        case LED_FAULT: {
            // Yellow flash
            if (now - lastFlashToggle >= LED_FAULT_FLASH_INTERVAL_MS) {
                ledFlashPhase   = !ledFlashPhase;
                lastFlashToggle = now;
            }
            setLED(ledFlashPhase ? CRGB(255, 165, 0) : CRGB::Black, 180);
            break;
        }

        case LED_DISCOVERING: {
            // Cyan flash — BLE scanning
            if (now - lastFlashToggle >= LED_BLE_FLASH_INTERVAL_MS) {
                ledFlashPhase   = !ledFlashPhase;
                lastFlashToggle = now;
            }
            setLED(ledFlashPhase ? CRGB::Cyan : CRGB::Black, 160);
            break;
        }

        case LED_WAITING: {
            // Slow blue pulse — not paired
            if (now - lastFlashToggle >= 1000) {
                ledFlashPhase   = !ledFlashPhase;
                lastFlashToggle = now;
            }
            setLED(ledFlashPhase ? CRGB::Blue : CRGB::Black, 100);
            break;
        }

        case LED_TIMEOUT: {
            // Fast blue flash — ESP-NOW connection lost
            if (now - lastFlashToggle >= LED_TIMEOUT_FLASH_MS) {
                ledFlashPhase   = !ledFlashPhase;
                lastFlashToggle = now;
            }
            setLED(ledFlashPhase ? CRGB::Blue : CRGB::Black, 160);
            break;
        }

        case LED_BATTERY_LOW:
            // Orange solid — battery warning
            setLED(CRGB(255, 80, 0), 160);
            break;

        case LED_OFF:
            setLED(CRGB::Black, 0);
            break;
    }
}

/*===========================================
 * SENSOR SETUP
 *===========================================*/

void setupSensor() {
    DEBUG_PRINTLN("Initialising IR beam sensor...");
    SensorManager::init(IR_RECEIVER_PIN);
    DEBUG_PRINTF("SensorManager initialised — emitter GPIO%d  receiver GPIO%d\n",
                 IR_EMITTER_PIN, IR_RECEIVER_PIN);
}

/*===========================================
 * ESP-NOW FUNCTIONS
 *===========================================*/

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        lastSuccessfulTransmission = millis();
        connTimeoutActive          = false;
        DEBUG_PRINTLN("ESP-NOW sent OK");
    } else {
        DEBUG_PRINTLN("ESP-NOW send failed");
    }
}

void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len != sizeof(struct_message)) {
        DEBUG_PRINTF("Unexpected data length: %d (expected %d)\n",
                     data_len, sizeof(struct_message));
        return;
    }

    memcpy(&incomingMessage, data, sizeof(incomingMessage));
    lastSuccessfulTransmission = millis();
    connTimeoutActive          = false;

    if (!espnowHandshakeComplete) {
        espnowHandshakeComplete = true;
        DEBUG_PRINTLN("ESP-NOW handshake complete");
    }

    DEBUG_PRINTF("Received — processControl: %d\n", incomingMessage.processControl);

    switch (incomingMessage.processControl) {

        case -1: {
            DEBUG_PRINTLN("Controller: SYNC/RESET received");
            SensorState ss = SensorManager::getState();

            if (ss == SENSOR_STANDBY) {
                if (!SensorManager::isBeamIntact()) {
                    DEBUG_PRINTLN("Cannot arm — beam not intact");
                    sendFaultNotification();
                    break;
                }
                if (!isBatterySafeForRace()) {
                    sendFaultNotification();
                    break;
                }
                SensorManager::arm();
                requestLedState(LED_ARMED);
                DEBUG_PRINTLN("Sensor ARMED");

            } else if (ss == SENSOR_ACTIVE) {
                DEBUG_PRINTLN("Already ACTIVE — re-sync received");

            } else if (ss == SENSOR_TRIGGERED || ss == SENSOR_RESETTING) {
                resetLockoutActive = true;
                resetLockoutStart  = millis();
                DEBUG_PRINTLN("Post-trigger reset lockout started");
            }

            sendPingResponse(mac_addr, incomingMessage.deviceIndex);
            break;
        }

        case 1:
            syncedTime = incomingMessage.syncTime;
            DEBUG_PRINTF("Time sync: %lu\n", syncedTime);
            break;

        default:
            DEBUG_PRINTF("Unknown processControl: %d\n", incomingMessage.processControl);
            break;
    }
}

void sendBreakbeamSignal() {
    memset(&outgoingMessage, 0, sizeof(outgoingMessage));
    outgoingMessage.syncTime       = millis();
    outgoingMessage.processControl = 0;
    outgoingMessage.timerControl   = 2;
    outgoingMessage.speedReading   = 0.0f;
    outgoingMessage.deviceIndex    = 1;

    if (!esp_now_is_peer_exist(macController)) {
        DEBUG_PRINTLN("ESP-NOW peer missing — re-adding");
        memcpy(peerInfo.peer_addr, macController, sizeof(macController));
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            DEBUG_PRINTLN("Failed to re-add peer");
            return;
        }
    }

    esp_err_t result = esp_now_send(macController,
                                    (uint8_t*)&outgoingMessage,
                                    sizeof(outgoingMessage));
    if (result == ESP_OK) {
        DEBUG_PRINTLN("Breakbeam signal sent (timerControl=2)");
        unsigned long now = millis();
        lastBreakbeamTime = now;
        sessionRuns++;

        // Red LED trigger hold
        triggerLedStart = now;
        requestLedState(LED_TRIGGERED);
    } else {
        DEBUG_PRINTF("Breakbeam send error: %d\n", result);
    }
}

void sendPingResponse(const uint8_t *mac_addr, int deviceIndex) {
    memset(&outgoingMessage, 0, sizeof(outgoingMessage));
    outgoingMessage.syncTime       = millis();
    outgoingMessage.processControl = -2;
    outgoingMessage.timerControl   = 0;
    outgoingMessage.deviceIndex    = deviceIndex;

    esp_err_t result = esp_now_send(mac_addr,
                                    (uint8_t*)&outgoingMessage,
                                    sizeof(outgoingMessage));
    if (result == ESP_OK) {
        DEBUG_PRINTLN("Ping response sent");
        return;
    }

    DEBUG_PRINTF("Ping response error: %d\n", result);

    if (!esp_now_is_peer_exist(macController)) {
        memcpy(peerInfo.peer_addr, macController, sizeof(macController));
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            delay(50);
            esp_now_send(macController, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
        }
    }
}

void sendFaultNotification() {
    memset(&outgoingMessage, 0, sizeof(outgoingMessage));
    outgoingMessage.syncTime       = millis();
    outgoingMessage.processControl = -4;
    outgoingMessage.deviceIndex    = 1;
    esp_now_send(macController, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
    DEBUG_PRINTLN("Fault notification sent to controller");
}

void checkESPNowConnection() {
    static unsigned long lastCheckTime  = 0;
    static unsigned long lastKeepAlive  = 0;
    static int           reconnectAttempts = 0;

    unsigned long now = millis();
    if (now - lastCheckTime < CONNECTION_CHECK_INTERVAL) return;
    lastCheckTime = now;

    bool peerExists      = esp_now_is_peer_exist(macController);
    bool connectionTimedOut = (now - lastSuccessfulTransmission > CONNECTION_TIMEOUT);

    if (!peerExists || connectionTimedOut) {
        reconnectAttempts++;
        if (reconnectAttempts >= 3) {
            connTimeoutActive = true;
            requestLedState(LED_TIMEOUT);
        }

        if (peerExists && connectionTimedOut) {
            esp_now_del_peer(macController);
            delay(100);
            peerExists = false;
        }

        if (!peerExists) {
            memcpy(peerInfo.peer_addr, macController, sizeof(macController));
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                memset(&outgoingMessage, 0, sizeof(outgoingMessage));
                outgoingMessage.processControl = -3;
                outgoingMessage.deviceIndex    = 1;
                esp_now_send(macController, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
            }
        }
    } else {
        reconnectAttempts = 0;
        connTimeoutActive = false;

        // Return LED to appropriate operational state if it was showing timeout
        if (currentLedState == LED_TIMEOUT) {
            SensorState ss = SensorManager::getState();
            if      (ss == SENSOR_ACTIVE)    requestLedState(LED_ARMED);
            else if (ss == SENSOR_FAULT)     requestLedState(LED_FAULT);
            else if (ss == SENSOR_ALIGNMENT) requestLedState(LED_ALIGNING);
            else                             requestLedState(LED_STANDBY);
        }

        if (now - lastKeepAlive > 5000) {
            memset(&outgoingMessage, 0, sizeof(outgoingMessage));
            outgoingMessage.processControl = -3;
            outgoingMessage.deviceIndex    = 1;
            esp_now_send(macController, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
            lastKeepAlive = now;
        }
    }
}

/*===========================================
 * BLE FUNCTIONS
 *===========================================*/

void setupBLE() {
    BLEDevice::init(TIMER_NAME);
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    DEBUG_PRINTLN("BLE setup complete");
}

bool connectToDevice() {
    DEBUG_PRINTLN("Connecting to BLE device...");
    pBLEClient = BLEDevice::createClient();

    if (!pBLEClient->connect(&_advertisedDevice)) {
        DEBUG_PRINTLN("BLE connect failed");
        return false;
    }
    clientConnected = true;

    BLERemoteService* pRemoteService = pBLEClient->getService(SERVICE_UUID);
    if (!pRemoteService) {
        pBLEClient->disconnect();
        return false;
    }

    pRemoteCharacteristicTX = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_TX);
    pRemoteCharacteristicRX = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_RX);
    if (!pRemoteCharacteristicTX || !pRemoteCharacteristicRX) {
        pBLEClient->disconnect();
        return false;
    }

    // Send our MAC to controller
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macBuffer[18];
    sprintf(macBuffer, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    pRemoteCharacteristicRX->writeValue((uint8_t*)macBuffer, strlen(macBuffer));

    char nameBuffer[100];
    sprintf(nameBuffer, "%s", TIMER_NAME);
    pRemoteCharacteristicRX->writeValue((uint8_t*)nameBuffer, strlen(nameBuffer));

    // Read controller MAC
    std::string macRaw = pRemoteCharacteristicTX->readValue();
    if (macRaw.size() < 6) {
        DEBUG_PRINTLN("Controller MAC too short");
        pBLEClient->disconnect();
        return false;
    }
    memcpy(macController, macRaw.data(), 6);

    storeDeviceInfoInPreferences(macController, globalDeviceName);

    pBLEClient->disconnect();
    delay(100);
    BLEDevice::deinit();
    delay(100);

    checkAndInitESPNow();
    delay(500);

    setupSensor();

    // Sensor now in ALIGNMENT
    requestLedState(LED_ALIGNING);

    deviceMode = MODE_OPERATIONAL;
    DEBUG_PRINTLN("Device OPERATIONAL after pairing");

    return true;
}

void checkAndInitESPNow() {
    WiFi.mode(WIFI_STA);

    int retryCount = 0;
    while (retryCount < MAX_ESP_NOW_INIT_RETRIES) {
        if (esp_now_init() == ESP_OK) break;
        esp_now_deinit();
        delay(1000 * (1 << retryCount));
        retryCount++;
    }

    if (retryCount >= MAX_ESP_NOW_INIT_RETRIES) {
        DEBUG_PRINTLN("ESP-NOW init failed after retries");
        requestLedState(LED_FAULT);
        return;
    }

    memcpy(peerInfo.peer_addr, macController, sizeof(macController));
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        esp_now_register_send_cb(OnDataSent);
        esp_now_register_recv_cb(onDataReceived);
        lastSuccessfulTransmission = millis();
        DEBUG_PRINTLN("ESP-NOW ready");
    } else {
        DEBUG_PRINTLN("Failed to add ESP-NOW peer");
    }
}

/*===========================================
 * PREFERENCE FUNCTIONS
 *===========================================*/

void storeDeviceInfoInPreferences(const uint8_t* macAddress, const String& deviceName) {
    preferences.begin("timer_app", false);
    preferences.putBytes("macController", macAddress, 6);
    preferences.putString("deviceName", deviceName);
    preferences.end();
    DEBUG_PRINTLN("Preferences stored");
}

void clearPreferences() {
    preferences.begin("timer_app", false);
    preferences.clear();
    preferences.end();
    DEBUG_PRINTLN("Preferences cleared");
}

bool loadStoredPreferences() {
    preferences.begin("timer_app", true);
    size_t readMac    = preferences.getBytes("macController", macController, sizeof(macController));
    String deviceName = preferences.getString("deviceName", "");
    preferences.end();

    if (readMac != sizeof(macController) || deviceName.isEmpty()) {
        DEBUG_PRINTLN("No valid preferences");
        return false;
    }
    globalDeviceName = deviceName;
    DEBUG_PRINTLN("Preferences loaded: " + deviceName);
    return true;
}

void executeUnpair() {
    DEBUG_PRINTLN("Unpairing — clearing preferences and restarting");
    setLED(CRGB::Red, 200);
    delay(300);
    setLED(CRGB::Black, 0);
    delay(200);
    clearPreferences();
    delay(500);
    ESP.restart();
}

/*===========================================
 * BUTTON HANDLER
 *===========================================*/

void handleButtonEvent(ButtonEvent event) {
    DEBUG_PRINTF("Button event: %d  mode: %d  sensor: %s\n",
                 event, deviceMode, SensorManager::getStateString());

    // Ultra-long press (6s) — soft power off
    if (event == BTN_ULTRA_LONG_PRESS) {
        DEBUG_PRINTLN("Ultra-long press — powering off");
        setLED(CRGB::Red, 80);
        delay(500);
        PowerManager::shutdown();   // soft off (~13uA); wakes on the ON button
        return;
    }

    // Cancel power-off sequence on release
    if (event == BTN_LONG_RELEASE || event == BTN_ULTRA_LONG_RELEASE) {
        powerOffSequenceActive = false;
        pairingProgressActive  = false;
        return;
    }

    if (powerOffSequenceActive) return;

    // Suppress LONG_PRESS on most states unless waiting to pair
    if (event == BTN_LONG_PRESS && ButtonManager::isPressed()) {
        if (deviceMode != MODE_WAITING) return;
    }

    switch (deviceMode) {

        case MODE_WAITING:
            // Long press — start BLE discovery
            if (event == BTN_LONG_PRESS) {
                DEBUG_PRINTLN("Long press — starting BLE discovery");
                pairingProgressActive = false;
                deviceMode            = MODE_DISCOVERING;
                discoveryStartTime    = millis();
                requestLedState(LED_DISCOVERING);
            }
            break;

        case MODE_DISCOVERING:
            // Single click — cancel discovery
            if (event == BTN_CLICK) {
                DEBUG_PRINTLN("Discovery cancelled");
                deviceMode         = MODE_WAITING;
                discoveryStartTime = 0;
                requestLedState(LED_WAITING);
            }
            break;

        case MODE_OPERATIONAL: {
            SensorState ss = SensorManager::getState();

            if (ss == SENSOR_ALIGNMENT) {
                // Click to skip alignment (testing only — beam must be intact)
                if (event == BTN_CLICK) {
                    if (SensorManager::isBeamIntact()) {
                        DEBUG_PRINTLN("Alignment skip — beam intact");
                        requestLedState(LED_STANDBY);
                        // Force SensorManager to standby
                        // Note: alignment won't be marked complete but allows testing
                        SensorManager::forceStandby();
                    } else {
                        DEBUG_PRINTLN("Alignment skip refused — beam not intact");
                        // Flash white twice to indicate refusal
                        setLED(CRGB::White, 200); delay(100);
                        setLED(CRGB::Black, 0);   delay(100);
                        setLED(CRGB::White, 200); delay(100);
                        setLED(CRGB::Black, 0);
                    }
                }

            } else if (ss == SENSOR_STANDBY) {
                if (event == BTN_CLICK) {
                    // Manual re-alignment
                    DEBUG_PRINTLN("Click — re-running alignment");
                    SensorManager::retryInit();
                    requestLedState(LED_ALIGNING);
                } else if (event == BTN_DOUBLE_CLICK) {
                    // Go to unpair
                    DEBUG_PRINTLN("Double-click — unpair");
                    SensorManager::forceStandby();
                    executeUnpair();
                }

            } else if (ss == SENSOR_ACTIVE) {
                if (event == BTN_DOUBLE_CLICK) {
                    // De-arm and unpair
                    DEBUG_PRINTLN("Double-click on active — unpair");
                    SensorManager::forceStandby();
                    executeUnpair();
                }
                // Single click ignored while armed — avoid accidental de-arm

            } else if (ss == SENSOR_FAULT) {
                if (event == BTN_CLICK) {
                    DEBUG_PRINTLN("Click — manual fault retry");
                    SensorManager::retryInit();
                    requestLedState(LED_ALIGNING);
                }
            }
            break;
        }

        default:
            break;
    }
}

/*===========================================
 * POWER-OFF SEQUENCE
 * No hardware latch — shows warning via LED flash then restarts.
 *===========================================*/

void updatePowerOffSequence() {
    if (deviceMode == MODE_DISCOVERING) {
        powerOffSequenceActive = false;
        return;
    }

    if (!ButtonManager::isPressed()) {
        powerOffSequenceActive = false;
        return;
    }

    unsigned long pressDuration = ButtonManager::getPressDuration();

    if (pressDuration >= POWER_OFF_WARNING_START_MS &&
        pressDuration <  POWER_OFF_THRESHOLD_MS) {
        if (!powerOffSequenceActive) {
            powerOffSequenceActive = true;
            DEBUG_PRINTLN("Power-off warning — release to cancel, hold to restart");
        }
        // Flash LED red to warn — rate increases as threshold approaches
        unsigned long remaining = POWER_OFF_THRESHOLD_MS - pressDuration;
        unsigned long flashRate = map(remaining, 0, POWER_OFF_THRESHOLD_MS - POWER_OFF_WARNING_START_MS, 100, 500);
        unsigned long now = millis();
        if (now - lastFlashToggle >= flashRate) {
            ledFlashPhase   = !ledFlashPhase;
            lastFlashToggle = now;
            setLED(ledFlashPhase ? CRGB::Red : CRGB::Black, 200);
        }
    }
}

/*===========================================
 * PAIRING PROGRESS
 * Pulses LED brightness while user holds button to pair.
 *===========================================*/

void updatePairingProgress() {
    if (deviceMode != MODE_WAITING) {
        pairingProgressActive = false;
        return;
    }

    if (!ButtonManager::isPressed()) {
        pairingProgressActive = false;
        return;
    }

    unsigned long pressDuration = ButtonManager::getPressDuration();

    if (pressDuration >= 100) {
        if (!pairingProgressActive) {
            pairingProgressActive = true;
            DEBUG_PRINTLN("Pairing progress started");
        }
        // Brightness ramps from 20 → 200 over 3s as a visual cue
        uint8_t brightness = (uint8_t)map(min(pressDuration, 3000UL), 0, 3000, 20, 200);
        FastLED.setBrightness(brightness);
        leds[0] = CRGB::Cyan;
        FastLED.show();
    }
}

/*===========================================
 * BATTERY FUNCTIONS
 *===========================================*/

void checkBatteryWarnings() {
    unsigned long now = millis();
    if (now - lastBatteryWarningCheck < BATTERY_WARNING_CHECK_INTERVAL) return;
    lastBatteryWarningCheck = now;

    if (BatteryManager::hasWarning()) {
        DEBUG_PRINTF("Battery warning: %s\n", BatteryManager::getWarningMessage());
        // Flash orange 3 times
        for (int i = 0; i < 3; i++) {
            setLED(CRGB(255, 80, 0), 200); delay(150);
            setLED(CRGB::Black, 0);        delay(150);
        }
    }
}

bool isBatterySafeForRace() {
    // TEMPORARILY DISABLED FOR TESTING - always return true
    return true;
    
    // if (!BatteryManager::isSafeForRace()) {
    //     DEBUG_PRINTLN("Battery too low to arm");
    //     // Three fast red flashes
    //     for (int i = 0; i < 3; i++) {
    //         setLED(CRGB::Red, 200); delay(100);
    //         setLED(CRGB::Black, 0); delay(100);
    //     }
    //     return false;
    // }
    // return true;
}

/*===========================================
 * SETUP
 *===========================================*/

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=================================");
    Serial.println("JNS Timer — Stamp S3");
    //Serial.println("Version: " APP_VERSION);
    Serial.println("=================================");
    Serial.println("Serial commands: U=unpair  D=debug  R=restart");
    Serial.println("=================================\n");

    // Watchdog
    esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);

    // FastLED
    FastLED.addLeds<SK6812, RGB_LED_PIN, GRB>(leds, RGB_LED_COUNT);
    requestLedState(LED_BOOT);
    updateLED();

    // Power: PM1 rail + wake must come up BEFORE the sensor. The IR emitter
    // runs off SYS_5V (EXT_5V_OUT), which is off until PowerManager enables it.
    if (!PowerManager::begin()) {
        DEBUG_PRINTLN("WARNING: PM1 not found — 5V rail OFF (emitter dead unless J1 externally powered)");
    }

    // Boot settle — blue LED
    DEBUG_PRINTLN("Settling...");
    delay(BOOT_SETTLE_MS);
    esp_task_wdt_reset();

    // Button
    ButtonManager::init(BTN_INPUT, true);
    ButtonManager::setCallback(handleButtonEvent);
    ButtonManager::setLongPressTime(LONG_PRESS_TIME_MS);
    ButtonManager::setUltraLongPressTime(ULTRA_LONG_PRESS_TIME_MS);
    DEBUG_PRINTLN("Button OK");

    // Battery - TEMPORARILY DISABLED FOR TESTING
    DEBUG_PRINTLN("WARNING: Battery manager disabled for testing!");
    bool batteryOk = false;  // Skip battery init
    // bool batteryOk = BatteryManager::init();
    // DEBUG_PRINTF("Battery: %d%% (%dmV) VIN=%dmV %s\n",
    //             BatteryManager::getPercentage(),
    //             BatteryManager::getVoltage(),
    //             BatteryManager::getInputVoltage(),
    //             BatteryManager::getChargingString());
    esp_task_wdt_reset();

    // COMMENTED OUT - allowing boot without battery manager
    // if (!batteryOk) {
    //     DEBUG_PRINTLN("BATTERY PMIC INIT FAILED — cannot boot");
    //     for (;;) {
    //         setLED(CRGB::Yellow, 200); delay(200);
    //         setLED(CRGB::Black, 0);    delay(800);
    //         esp_task_wdt_reset();
    //     }
    // }

    // // Check battery before continuing
    // if (BatteryManager::isCritical()) {
    //     DEBUG_PRINTLN("CRITICAL BATTERY — cannot boot");
    //     for (;;) {
    //         setLED(CRGB::Red, 200); delay(200);
    //         setLED(CRGB::Black, 0); delay(800);
    //         esp_task_wdt_reset();
    //     }
    // }

    // Preferences / pairing
    if (loadStoredPreferences()) {
        DEBUG_PRINTLN("Paired — loading ESP-NOW");
        checkAndInitESPNow();
        esp_task_wdt_reset();

        setupSensor();
        deviceMode = MODE_OPERATIONAL;
        requestLedState(LED_ALIGNING);
        DEBUG_PRINTLN("OPERATIONAL — sensor in alignment");

    } else {
        DEBUG_PRINTLN("Not paired — waiting for BLE pairing");
        setupBLE();
        deviceMode = MODE_WAITING;
        requestLedState(LED_WAITING);
    }

    lastHeartbeat = millis();
    DEBUG_PRINTLN("Setup complete\n");
}

/*===========================================
 * MAIN LOOP
 *===========================================*/

void loop() {
    // Serial debug commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'u' || cmd == 'U') {
            Serial.println("Force unpair");
            clearPreferences();
            delay(500);
            ESP.restart();
        } else if (cmd == 'd' || cmd == 'D') {
            Serial.println("=== DEBUG ===");
            Serial.printf("Mode:         %d\n",  deviceMode);
            Serial.printf("Sensor:       %s\n",  SensorManager::getStateString());
            Serial.printf("Beam:         %s\n",  SensorManager::isBeamIntact() ? "INTACT" : "BROKEN");
            Serial.printf("LED state:    %d\n",  currentLedState);
            Serial.printf("Battery:      %dmV  VIN=%dmV  %s\n",
                PowerManager::batteryMillivolts(),
                PowerManager::inputMillivolts(),
                PowerManager::onExternalPower() ? "EXT-PWR" : "BATTERY");
            Serial.printf("Session runs: %d\n",  sessionRuns);
            Serial.printf("Last break:   %luus\n", SensorManager::getLastBreakDurationUs());
            Serial.println("=============\n");
        } else if (cmd == 'r' || cmd == 'R') {
            Serial.println("Restart");
            delay(500);
            ESP.restart();
        }
    }

    esp_task_wdt_reset();

    ButtonManager::update();
    updatePowerOffSequence();
    updatePairingProgress();

    // TEMPORARILY DISABLED FOR TESTING
    // if (BatteryManager::update()) {
    //     DEBUG_PRINTF("Battery: %d%% (%dmV) VIN=%dmV %s\n",
    //                 BatteryManager::getPercentage(),
    //                 BatteryManager::getVoltage(),
    //                 BatteryManager::getInputVoltage(),
    //                 BatteryManager::getChargingString());
    // }
    // checkBatteryWarnings();

    switch (deviceMode) {

        case MODE_WAITING:
            break;

        case MODE_DISCOVERING: {
            if (discoveryStartTime > 0 &&
                millis() - discoveryStartTime > DISCOVERY_TIMEOUT_MS) {
                DEBUG_PRINTLN("Discovery timeout — no devices found");
                deviceMode         = MODE_WAITING;
                discoveryStartTime = 0;
                requestLedState(LED_WAITING);
                break;
            }

            static unsigned long lastScanStart = 0;
            unsigned long now = millis();
            if (now - lastScanStart < 3500) break;
            lastScanStart = now;

            deviceFound = false;
            esp_task_wdt_reset();
            BLEScanResults foundDevices = pBLEScan->start(3, false);
            esp_task_wdt_reset();
            pBLEScan->clearResults();

            if (deviceFound) {
                if (connectToDevice()) {
                    discoveryStartTime = 0;
                    lastScanStart      = 0;
                }
            }
            break;
        }

        case MODE_OPERATIONAL: {
            checkESPNowConnection();

            SensorManager::update();
            SensorState ss = SensorManager::getState();

            // ── Alignment progress ───────────────────────────────────────
            // White LED stays on during alignment — no per-tick update needed.
            // Transition to standby is detected below.

            // ── Alignment complete → standby ─────────────────────────────
            static bool wasAligning = false;
            if (wasAligning && ss == SENSOR_STANDBY) {
                DEBUG_PRINTLN("Alignment complete — standby");
                requestLedState(LED_STANDBY);
                lastHeartbeat = millis();
            }
            wasAligning = (ss == SENSOR_ALIGNMENT);

            // ── Valid crossing ────────────────────────────────────────────
            if (SensorManager::hasTriggerPending()) {
                DEBUG_PRINTF("Trigger! Duration: %luus\n",
                             SensorManager::getLastBreakDurationUs());
                sendBreakbeamSignal();
                // LED handled inside sendBreakbeamSignal()
            }

            // ── Fault ─────────────────────────────────────────────────────
            if (SensorManager::hasFaultPending()) {
                DEBUG_PRINTLN("Sensor fault");
                requestLedState(LED_FAULT);
                if (ss == SENSOR_ACTIVE || ss == SENSOR_TRIGGERED) {
                    sendFaultNotification();
                }
            }

            // ── Post-trigger reset lockout ────────────────────────────────
            if (resetLockoutActive) {
                if (millis() - resetLockoutStart >= RESET_DELAY_MS) {
                    resetLockoutActive = false;
                    SensorManager::reset();
                    requestLedState(LED_ARMED);
                    DEBUG_PRINTLN("Reset lockout complete — re-armed");
                }
            }

            // ── Keep LED in sync with sensor state ────────────────────────
            // Only correct if no overriding state is active.
            // SENSOR_ACTIVE allows LED_STANDBY — that's the post-trigger
            // "run complete, waiting for re-arm" state.
            if (!resetLockoutActive && !connTimeoutActive) {
                if      (ss == SENSOR_ALIGNMENT && currentLedState != LED_ALIGNING)
                    requestLedState(LED_ALIGNING);
                else if (ss == SENSOR_STANDBY   && currentLedState != LED_STANDBY
                                                && currentLedState != LED_BATTERY_LOW)
                    requestLedState(LED_STANDBY);
                else if (ss == SENSOR_ACTIVE    && currentLedState != LED_ARMED
                                                && currentLedState != LED_TRIGGERED
                                                && currentLedState != LED_STANDBY)
                    requestLedState(LED_ARMED);
                else if (ss == SENSOR_FAULT     && currentLedState != LED_FAULT)
                    requestLedState(LED_FAULT);
            }

            break;
        }

        default:
            break;
    }

    // Drive LED state machine every loop
    updateLED();

    delay(5);
}