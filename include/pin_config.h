#pragma once

/*===========================================
 * JNS Timer — BAT_Carrier pin map
 * Reconciled to BAT_Carrier.kicad_sch (Stamp-S3Bat DIP)
 *
 * Net (schematic)   Module pin / GPIO   Rail
 *   IR_RX_OUT          G3                SYS_3V3 (always-on)
 *   BEEP               G4                SYS_3V3 (always-on)
 *   IR_TX_EN           G5                SYS_5V  (EXT_5V_OUT — must be enabled!)
 *   LED_EXT (D2)       G7                SYS_3V3 (always-on)
 *   USER_BUTTON (J4)   G9                — (internal pull-up)
 *   LED_OUT (J5)       G11               SYS_5V  (external NeoPixel — unused for now)
 *   WAKE_IN_N (J6)     PY_G4 (PM1 IO4)   PM1 always-on domain
 *===========================================*/

/*===========================================
 * IR BEAM-BREAK CIRCUIT
 *===========================================*/

#define IR_EMITTER_PIN      5       // G5  -> IR_TX_EN  (38kHz carrier; LED powered from SYS_5V)
#define IR_RECEIVER_PIN     3       // G3  -> IR_RX_OUT (TSSP77038, powered from SYS_3V3)

#define IR_CARRIER_FREQ_HZ  38000
#define IR_CARRIER_DUTY     128
#define IR_CARRIER_RES_BITS 8
#define IR_LEDC_CHANNEL     0

/*===========================================
 * BATTERY / POWER MANAGEMENT (M5PM1, addr 0x6E)
 *===========================================*/

#define BATTERY_I2C_SDA         48      // G48 -> PM1 SDA (internal bus)
#define BATTERY_I2C_SCL         47      // G47 -> PM1 SCL (internal bus)
#define BATTERY_VIN_PRESENT_MV  4300    // VIN above this == on USB/DC

/*===========================================
 * LED
 * Onboard SK6812 status LED (D2) on G7 via net LED_EXT.
 * NOTE: the *external* NeoPixel connector J5 is a SEPARATE channel on
 *       G11 (net LED_OUT, powered from SYS_5V) and is currently unused.
 *       Enable below only if/when you wire it up.
 *===========================================*/

#define RGB_LED_PIN             7       // G7 -> LED_EXT -> D2 (onboard status LED)
#define RGB_LED_COUNT           1

// External NeoPixel connector J5 on G11 (net LED_OUT, powered from SYS_5V).
// Mirrors the onboard status colour. Bump EXT_LED_COUNT for a strip.
// NOTE: J5 is a 5V part driven by 3.3V logic — data-high margin is tight
//       (SK6812 wants ~0.7*Vdd). If a long/flaky string misbehaves, that's
//       the cause; a level shifter or 3.3V supply on the first pixel fixes it.
#define HAS_EXT_LED             1
#define EXT_LED_PIN             11      // G11 -> LED_OUT -> J5
#define EXT_LED_COUNT           1

/*===========================================
 * BUZZER
 *===========================================*/

#define BUZZER_PIN              4       // G4 -> BEEP (SS8050 driver, buzzer on SYS_3V3)

/*===========================================
 * USER BUTTON
 * FIXED: board wires USER_BUTTON to G9 (pin 14), NOT G0.
 * G0 is the BOOT strapping pin and isn't broken out on this module.
 *===========================================*/

#define BTN_INPUT               9       // G9 -> USER_BUTTON (momentary to GND, internal pull-up)

/*===========================================
 * WAKE / SOFT POWER (handled by PowerManager via the PM1)
 * J6 ON-button -> WAKE_IN_N -> PM1 IO4 (PY_G4).
 *
 * HARDWARE ACTION REQUIRED: do NOT populate R5 (10K) on WAKE_IN_N.
 * It is a pull-DOWN to GND and defeats the PM1 internal pull-up, so the
 * falling-edge wake can never fire. With R5 lifted, the PM1 internal
 * pull-up idles the line high and the button (to GND) makes the edge.
 *===========================================*/

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
#define HAS_POWER_CONTROL       1       // PM1 soft power-off + wake now implemented
#define HAS_DISPLAY             0

/*===========================================
 * BUTTON TIMING CONFIGURATION
 *===========================================*/

#define DEBOUNCE_DELAY_MS           50
#define DOUBLE_CLICK_WINDOW_MS      500
#define LONG_PRESS_TIME_MS          3000
#define ULTRA_LONG_PRESS_TIME_MS    6000
