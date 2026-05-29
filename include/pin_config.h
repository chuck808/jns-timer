#pragma once

/*===========================================
 * IR BEAM-BREAK CIRCUIT
 *===========================================*/

#define IR_EMITTER_PIN      5
#define IR_RECEIVER_PIN     3

#define IR_CARRIER_FREQ_HZ  38000
#define IR_CARRIER_DUTY     128
#define IR_CARRIER_RES_BITS 8
#define IR_LEDC_CHANNEL     0

/*===========================================
 * BATTERY / POWER MANAGEMENT
 *===========================================*/

#define BATTERY_I2C_SDA         48
#define BATTERY_I2C_SCL         47
#define BATTERY_VIN_PRESENT_MV  4300

/*===========================================
 * LED
 * External SK6812 status LED
 *===========================================*/

#define RGB_LED_PIN             7
#define RGB_LED_COUNT           1

/*===========================================
 * BUZZER
 *===========================================*/

#define BUZZER_PIN              4

/*===========================================
 * USER BUTTON
 *===========================================*/

#define BTN_INPUT               0

/*===========================================
 * HARDWARE CAPABILITIES
 *===========================================*/

#define HAS_IR_EMITTER          1
#define HAS_IR_RECEIVER         1
#define HAS_BATTERY_ADC         0
#define HAS_BATTERY_PMIC        1
#define HAS_RGB_LED             1
#define HAS_BUTTON              1
#define HAS_BUZZER              1
#define HAS_WIFI                1
#define HAS_BLE                 1
#define HAS_PSRAM               1
#define HAS_POWER_CONTROL       0
#define HAS_DISPLAY             0

/*===========================================
 * BUTTON TIMING CONFIGURATION
 *===========================================*/

#define DEBOUNCE_DELAY_MS           50
#define DOUBLE_CLICK_WINDOW_MS      500
#define LONG_PRESS_TIME_MS          3000
#define ULTRA_LONG_PRESS_TIME_MS    6000