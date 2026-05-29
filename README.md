# JNS Timer - IR Beam-Break Timer for BMX Training

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-orange.svg)](https://platformio.org/)
[![ESP32-S3](https://img.shields.io/badge/ESP32-S3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A precision wireless timer system designed for BMX training, using IR beam-break technology for accurate lap timing and gate crossing detection.

## 🚴 Features

- **High-Precision Timing**: Infrared beam-break detection using TSSP77038 receiver with 38kHz modulated emitter
- **Wireless Communication**: ESP-NOW protocol for ultra-low latency communication with gate controller
- **BLE Pairing**: Easy device pairing via Bluetooth Low Energy
- **Visual Feedback**: RGB LED status indication with comprehensive state machine
- **Smart Battery Management**: Integrated power monitoring with M5PM1
- **Robust Crossing Detection**: Configurable timing windows to filter noise and validate crossings
- **Multi-Button Interface**: Support for single, double, long, and ultra-long press actions

## 📋 Hardware Requirements

### Main Components
- **MCU**: M5Stack Stamp S3 on StampS3 GroveBreakOut (SKU: A144)
- **Sensor**: TSSP77038 demodulated IR receiver (GPIO3)
- **Emitter**: Discrete 38kHz IR LED (GPIO5)
- **Status LED**: WS2812B/SK6812 RGB LED (GPIO7)
- **Battery**: M5PM1 Power Management IC

### Pin Configuration
| Component | GPIO | Description |
|-----------|------|-------------|
| IR Emitter | 5 | 38kHz modulated output |
| IR Receiver | 3 | TSSP77038 digital input |
| RGB LED | 7 | Status indication |
| Button | 0 | User input (onboard) |
| Buzzer | 4 | Audio feedback |
| I2C SDA | 48 | Battery management |
| I2C SCL | 47 | Battery management |

## 🎨 LED Status Indicators

| Color | State | Description |
|-------|-------|-------------|
| 🔵 Blue (solid) | Booting | System initialization |
| ⚪ White | Alignment | Beam not yet stable |
| 🟢 Green (heartbeat) | Standby | Beam intact, ready to arm |
| 🟣 Purple | Armed | Watching for crossing |
| 🔴 Red | Triggered | Valid crossing detected |
| 🟡 Yellow (flash) | Fault | Beam lost while armed |
| 🔵 Cyan (flash) | Discovering | BLE scanning active |
| 🔵 Blue (pulse) | Waiting | Not paired |
| 🔵 Blue (fast flash) | Timeout | Connection lost |
| 🟠 Orange | Low Battery | Battery warning |

## 📡 Communication Protocol

### ESP-NOW Messages

**From Controller** (`processControl`):
- `-1`: Sync/Reset/Activate timer
- `1`: Time synchronization

**To Controller**:
- `timerControl = 2`: Beam-break event detected
- `processControl = -2`: Ping response
- `processControl = -3`: Keep-alive
- `processControl = -4`: Fault/error condition

## 🚀 Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code
- USB-C cable for programming
- Gate controller device (separate system)

### Installation

1. Clone this repository:
```bash
git clone https://github.com/chuck808/jns-timer.git
cd jns-timer
```

2. Open in VS Code with PlatformIO extension

3. Build and upload:
```bash
pio run -t upload
```

4. Monitor serial output:
```bash
pio device monitor
```

### First Time Setup

1. **Power On**: The device will show a slow blue pulse (waiting mode)
2. **Pairing**: Press and hold the button for 3 seconds to start BLE discovery (cyan flash)
3. **Connect**: The device will automatically pair with the gate controller
4. **Alignment**: White LED indicates beam alignment in progress
5. **Ready**: Green heartbeat indicates system is ready

## ⚙️ Configuration

### Timing Windows
Adjust in `include/SensorManager.h`:
```cpp
#define MIN_BREAK_US        10000UL     // Min crossing time (10ms)
#define MAX_BREAK_US        500000UL    // Max crossing time (500ms)
#define ALIGNMENT_STABLE_MS 2000UL      // Alignment duration
```

### Battery Thresholds
Configure in `include/BatteryManager.h`:
```cpp
#define BATTERY_VOLTAGE_WARNING  3600   // Warning threshold (mV)
#define BATTERY_VOLTAGE_LOW      3400   // Low battery threshold
#define BATTERY_VOLTAGE_CRITICAL 3200   // Critical threshold
```

### LED Behavior
Modify in `src/main.cpp`:
```cpp
#define LED_HEARTBEAT_INTERVAL_MS   2000    // Pulse period
#define LED_TRIGGER_HOLD_MS         1500    // Red LED duration
```

## 🔧 Serial Debug Commands

While connected via serial monitor (115200 baud):
- `U`: Force unpair device
- `D`: Display debug information (battery, sensor state, session stats)
- `R`: Restart device

## 🎯 Usage

### Normal Operation
1. Device automatically arms when controller sends sync signal
2. Purple LED indicates armed state
3. Crossing detection is automatic
4. Red flash confirms valid crossing
5. Returns to standby (green) after trigger

### Manual Operations
- **Single Click** (Standby): Re-run alignment
- **Single Click** (Alignment): Skip alignment (testing only, requires intact beam)
- **Double Click** (Standby/Armed): Unpair device
- **Long Press** (6s): Restart device

## 📊 Technical Specifications

- **Crossing Detection**: 10ms - 500ms window
- **Timing Resolution**: Microsecond precision
- **Communication Range**: ~50m line-of-sight (ESP-NOW)
- **Battery Life**: ~8-12 hours continuous operation
- **Operating Voltage**: 3.3V - 5V
- **IR Carrier Frequency**: 38kHz
- **Wireless Protocol**: ESP-NOW (2.4GHz)

## 🛠️ Development

### Project Structure
```
JNS_Timer_IR/
├── include/
│   ├── pin_config.h          # Hardware pin definitions
│   ├── SensorManager.h       # IR beam sensor control
│   ├── ButtonManager.h       # Input handling
│   └── BatteryManager.h      # Power management
├── src/
│   ├── main.cpp              # Main application logic
│   ├── SensorManager.cpp     # Sensor implementation
│   ├── ButtonManager.cpp     # Button implementation
│   └── BatteryManager.cpp    # Battery implementation
├── lib/                      # Custom libraries
├── test/                     # Unit tests
└── platformio.ini            # PlatformIO config
```

### Dependencies
- **FastLED** (^3.7.0): RGB LED control
- **M5Unified** (^0.2.13): M5Stack hardware support
- **M5PM1** (^1.0.6): Battery management

### Building from Source
```bash
# Clean build
pio run -t clean

# Build
pio run

# Upload and monitor
pio run -t upload && pio device monitor
```

## 🐛 Troubleshooting

### Device won't pair
- Ensure controller is in pairing mode
- Check BLE is enabled on controller
- Restart both devices
- Use `U` command to clear stored pairing

### Beam alignment fails
- Verify IR emitter is powered (check GPIO5)
- Test receiver output (GPIO3 should be LOW when beam intact)
- Ensure proper alignment between emitter and receiver
- Check for obstructions

### Connection timeout
- Fast blue flash indicates lost connection
- Check controller is powered on
- Verify both devices are on same WiFi channel
- Re-pair if persistent

### Battery monitoring enabled
- Full battery monitoring and warnings active
- Orange LED indicates low battery state
- Critical battery prevents boot until charged
- Battery percentage and voltage displayed in debug output (`D` command)

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **JNS Development Team**

## 🙏 Acknowledgments

- Built on ESP32-S3 platform by Espressif
- Uses M5Stack hardware ecosystem
- FastLED library by Daniel Garcia

## 📞 Support

For issues, feature requests, or questions:
- Open an issue on GitHub
- Check existing documentation in `/include/README`

---

**Version**: 2.0.0 (Stamp S3 Port)  
**Last Updated**: 2025
