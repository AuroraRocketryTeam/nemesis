# Nemesis Flight Computer

[![PlatformIO CI](https://github.com/AuroraRocketryTeam/Aurora_Rocketry_SW_24_25/actions/workflows/platformio.yml/badge.svg)](https://github.com/AuroraRocketryTeam/Aurora_Rocketry_SW_24_25/actions/workflows/platformio.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

**Advanced flight computer software for high-power rocketry**, featuring real-time telemetry, sensor fusion, and autonomous flight state management.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware](#hardware)
- [Getting Started](#getting-started)
- [Building & Flashing](#building--flashing)
- [Telemetry System](#telemetry-system)
- [Flight States](#flight-states)
- [Development](#development)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

Nemesis is a sophisticated flight computer designed for the **Aurora Rocket Team** competition rockets. Built on the ESP32 platform with PlatformIO, it provides:

- **Real-time sensor data acquisition** from IMUs, barometers, and GPS
- **Finite State Machine (FSM)** for autonomous flight phase management
- **Binary telemetry protocol** over LoRa for efficient ground station communication
- **Kalman filtering** for accurate altitude and velocity estimation
- **SD card logging** for post-flight analysis
- **FreeRTOS-based** concurrent task management

This system has been developed to meet the demands of high-altitude flights with real-time decision-making capabilities and robust data logging.

---

## Features

### Sensor Suite
- **Inertial Measurement**: BNO055 9-DOF IMU with sensor fusion, LIS3DHTR accelerometer
- **Barometric Pressure**: MS5611 high-precision barometers
- **Navigation**: u-blox GPS module with UBX protocol support

### Flight Management
- **7-phase FSM**: Idle → Armed → Powered Flight → Coasting → Apogee → Descent → Landed
- **State-specific tasks**: Concurrent execution of sensor sampling, data logging, and telemetry
- **Transition detection**: Accelerometer-based launch detection, barometric apogee detection
- **Safety features**: Automated arming sequences, failsafe mechanisms

### Telemetry & Communication
- **Binary protocol**: Custom telemetry protocol for packet fragmentation
- **LoRa radio**: Long-range communication (868 MHz)
- **Real-time metrics**: Altitude, velocity, acceleration, orientation, GPS position, battery status and logging
- **Ground station**: Dedicated receiver with OLED display and serial output

---

## Hardware

### Primary Flight Computer
- **MCU**: ESP32-based board (Arduino Nano ESP32 or similar)
- **Storage**: SD card module (SPI interface)
- **Radio**: SX1262 LoRa transceiver (RadioLib compatible)

### Supported Sensors
| Sensor | Interface | Purpose |
|--------|-----------|---------|
| BNO055 | I2C | 9-DOF IMU with built-in fusion |
| LIS3DHTR | I2C | High-G accelerometer |
| MS5611  | I2C | Precision barometers |
| u-blox GPS | I2C/UART | Global positioning |

### Ground Station Hardware
- **Heltec WiFi LoRa 32 V3**: ESP32-S3 + SX1262 + OLED display
- **Power**: USB or battery (supports remote deployment)

---

### Module Organization

```
lib/
+-- control/          # FSM, state machine, flight logic
|   +-- RocketFSM     # Main FSM implementation
|   +-- states/       # State actions and transitions
|   +-- tasks/        # FreeRTOS tasks per flight phase
+-- BNO055/           # IMU driver
+-- MS5611/           # Barometer driver
+-- GPS/              # GNSS driver
+-- ...
+-- telemetry/        # Binary protocol, packet management
+-- LoRa/             # Radio transmitter/receiver
+-- kalman/           # Kalman filter implementations
+-- logger/           # SD card logging utilities
+-- data/             # Data structures (TelemetryPacket, etc.)
```

### Key Design Patterns
- **Dependency Injection**: Sensors passed as `shared_ptr` to FSM for testability
- **RAII**: Automatic resource management for SD files, I2C devices
- **Thread-safe logging**: Mutex-protected serial output via `Logger` namespace

---

## Getting Started

### Prerequisites

1. **Install PlatformIO**:
   ```bash
   # Via pip
   pip install platformio
   
   # Or install VSCode + PlatformIO IDE extension
   ```

2. **Clone the repository**:
   ```bash
   git clone https://github.com/AuroraRocketryTeam/Aurora_Rocketry_SW_24_25.git
   cd Aurora_Rocketry_SW_24_25
   ```

3. **Install dependencies**:
   ```bash
   pio pkg install
   ```

### Configuration


Key settings are in `lib/global/src/`:
- **`config.h`**: Flight parameters, sensor calibration, thresholds
- **`pins.h`**: GPIO pin assignments for your hardware

Edit these files to match your specific hardware configuration.

---

## Building & Flashing

### Default Environment (Arduino Nano ESP32)

```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

### Custom Environment

Modify `platformio.ini` to add your board. Example for a custom ESP32 target:

```ini
[env:my_custom_board]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags = ${env.build_flags}
lib_deps = ${env.lib_deps}
```

Then build with: `pio run -e my_custom_board`

### Ground Station

The telemetry receiver is a separate Arduino sketch:

```bash
# Navigate to receiver folder
cd lib/LoRa/src/

# Open telemetry_lora_receiver.ino in Arduino IDE or:
pio ci --board=heltec_wifi_lora_32_V3 telemetry_lora_receiver.ino
```

---

## Building & Flashing

### Default Environment (Arduino Nano ESP32)

```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

### Custom Environment

Modify `platformio.ini` to add your board. Example for a custom ESP32 target:

```ini
[env:my_custom_board]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags = ${env.build_flags}
lib_deps = ${env.lib_deps}
```

Then build with: `pio run -e my_custom_board`

### Ground Station

The telemetry receiver is a separate Arduino sketch:

```bash
# Navigate to receiver folder
cd lib/LoRa/src/

# Open telemetry_lora_receiver.ino in Arduino IDE or:
pio ci --board=heltec_wifi_lora_32_V3 telemetry_lora_receiver.ino
```

---

## Telemetry System

### Binary Protocol

Telemetry uses a fixed-size packet structure for reliable LoRa transmission:

```cpp
struct TelemetryPacket {
    uint32_t timestamp;          // Milliseconds since boot
    RocketState state;           // Current flight state
    float altitude;              // Meters ASL
    float vertical_velocity;     // m/s
    float acceleration[3];       // X, Y, Z in m/s²
    float angular_velocity[3];   // Roll, pitch, yaw in deg/s
    float gps_lat, gps_lon;      // Degrees
    uint8_t gps_fix;            // Fix quality
    float battery_voltage;       // Volts
    // ... + CRC checksum
};
```

### Packet Management
- **Fragmentation**: Large packets split into LoRa-compatible chunks
- **Reassembly**: Sequence numbers ensure correct reconstruction
- **CRC validation**: 16-bit CRC for error detection
- **Metrics tracking**: RSSI, SNR, packet loss, throughput

### Transmission Parameters
- **Frequency**: 868 MHz (Europe) / 915 MHz (US)
- **Spreading Factor**: SF7 (fast, shorter range) to SF12 (slow, max range)
- **Bandwidth**: 125 kHz
- **Coding Rate**: 4/7

Airtime for ~64-byte payload at SF7: **~50-80 ms**

---

## Flight States

| State | Entry Condition | Active Tasks | Exit Condition |
|-------|----------------|--------------|----------------|
| **IDLE** | Power-on | Status LED | Arming sequence |
| **ARMED** | User input | All sensors active | Accel > launch threshold |
| **POWERED_FLIGHT** | Launch detected | High-rate logging | Motor burnout (accel < threshold) |
| **COASTING** | Burnout | Altitude tracking | Apogee (velocity < 0) |
| **APOGEE** | Velocity negative | Deploy drogue chute | Altitude dropping |
| **DESCENT** | Post-apogee | GPS tracking | Altitude < 300m (deploy main) |
| **LANDED** | Near-zero velocity | Beeper, data flush | Manual reset |

Transition logic is in `lib/control/src/states/TransitionManager.cpp`.

---

## Development

### Code Style
- **C++17** standard (enforced by `-std=gnu++17`)
- **Doxygen comments** for all public APIs
- **Include guards** and `#pragma once` for headers

### Testing

Unit tests are in `test/`:
```bash
# Run all tests
pio test

# Run specific test
pio test -f test_kalman_filter
```

Manual hardware tests are in `test/test_manual/`.

### Debugging

Enable verbose logging in `config.h`:
```cpp
#define LOG_LEVEL LOG_LEVEL_DEBUG
```

View logs via serial monitor:
```bash
pio device monitor --baud 115200
```

### Adding a New Sensor

1. Create driver in `lib/YourSensor/src/`
2. Implement `ISensor` interface (if applicable)
3. Add to `RocketFSM` initialization
4. Include in relevant state tasks
5. Update `TelemetryPacket` if needed

---

## Documentation
This project's docs are publicly available in the [Github pages](https://aurorarocketryteam.github.io/nemesis/) of this repository.
Docs are generated using Doxygen inside a Github action.
### Generate API Docs

The project includes a Doxygen configuration:

```bash
# Install Doxygen
sudo apt-get install doxygen

# Generate HTML documentation
doxygen Doxyfile

# Open in browser
xdg-open docs/html/index.html
```

### Additional Resources

- **Flight State Patterns**: See `lib/control/docs/STATUS_PATTERNS.md`
- **Telemetry Migration Guide**: See `lib/telemetry/docs/TELEMETRY_MIGRATION.md`
- **Binary Protocol Spec**: See `lib/telemetry/docs/BINARY_TELEMETRY.md`
---

## Contributing

We welcome contributions from the Aurora Rocket Team and the wider rocketry community!

### Workflow

1. **Fork** the repository
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Commit changes**: `git commit -m 'Add amazing feature'`
4. **Push to branch**: `git push origin feature/amazing-feature`
5. **Open a Pull Request**

### Guidelines

- Follow existing code style and naming conventions
- Add Doxygen comments for new public APIs
- Include unit tests for new algorithms
- Update documentation for user-facing changes
- Test on hardware before submitting (if possible)

### Issue Reporting

Found a bug? Have a feature request? Open an issue with:
- Clear description of the problem/feature
- Steps to reproduce (for bugs)
- Expected vs. actual behavior
- Hardware/software versions

---

## License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

### Third-Party Libraries

- **RadioLib**: LGPL-3.0 (LoRa communication)
- **BME680 Library**: BSD (Bosch Sensortec)
- **SparkFun u-blox Library**: MIT
- **Eigen**: MPL2 (linear algebra)
- **TinyEKF**: LGPL (Kalman filtering)

---

## Team

**Aurora Rocket Team** - Università di Bologna

For questions or collaboration opportunities, reach out via GitHub issues or the team's official channels.

---

## Acknowledgments

- **Bosch Sensortec** for excellent sensor documentation
- **RadioLib community** for LoRa protocol support  
- **PlatformIO team** for the best embedded development platform
- **FreeRTOS** for reliable real-time task scheduling

---

<div align="center">

[Back to Top](#nemesis-flight-computer)

</div>
