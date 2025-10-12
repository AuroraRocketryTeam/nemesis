#pragma once

#include <cstdint>

#define __DEBUG__

#define SUFFICIENT_SENSOR_CALIBRATION 2
#define SENSOR_LOOKUP_MAX_ATTEMPTS 5
#define SENSOR_LOOKUP_TIMEOUT 1000

#define MS56_I2C_ADDR_1 0x77
#define MS56_I2C_ADDR_2 0x76
#define BNO055_I2C_ADDR 0x28

/* Legacy sensors */
// #define BME680_I2C_ADDR_1 0x77
// #define BME680_I2C_ADDR_2 0x76
// #define MPRLS_I2C_ADDR 0x18
// #define I2C_MULTIPLEXER_ADDRESS 0x70
// #define I2C_MULTIPLEXER_MPRLS1 1
// #define I2C_MULTIPLEXER_MPRLS2 0

/* LoRa configuration */
#define E220_22       // E220-900T22D
#define FREQUENCY_868 // 868 MHz

#define LORA_SERIAL 1 // Serial port for LoRa (1 = Serial1, 2 = Serial2)

/* Legacy Transmitter antenna address */
#define LORA_ADDH 0x00
#define LORA_ADDL 0x03
/* Legacy Receiver antenna address */
#define LORA_RECEIVER_ADDH 0x00
#define LORA_RECEIVER_ADDL 0x05

#define LORA_CHANNEL 23 // Legacy Communication channel

/* Uncomment the following line to enable RSSI */
#define ENABLE_RSSI

#define SERIAL_BAUD_RATE 115200 // Serial baud rate

// #define TRANSMITTER_CONFIG_MODE_ENABLE // Enable configuration mode for the transmitter (needs to be tested on new hardware)

// Number of log entries to batch before writing to SD card
#define BATCH_SIZE 30

// Maximum log entries before forcing a clear (emergency protection)
#define MAX_LOG_ENTRIES 20

/* Flight parameters configuration */
#define LIFTOFF_ACCELERATION_THRESHOLD GRAVITY * 2.0f // Threshold for the detection of liftoff when relative_acceleration is > 2G in any direction (relative_acceleration = acceleration - gravity)
#define LIFTOFF_TIMEOUT_MS 1000    // Threshold for the detection of liftoff
#define DROGUE_APOGEE_TIMEOUT 2000 // Threshold for opening the drogue parachute after apogee is detected
#define MAIN_ALTITUDE_THRESHOLD 450.0f // Altitude threshold for the deployment of the main parachute (in meters)
#define TOUCHDOWN_VELOCITY_THRESHOLD 2.0f // Vertical velocity threshold for touchdown detection (in m/s)

/* ESP-NOW Telemetry Configuration */
// MAC address of the peer receiver (ESP32 that will relay to LoRa)
// Replace with your actual receiver MAC address
#define ESPNOW_PEER_MAC { 0x34, 0xCD, 0xB0, 0x3C, 0x54, 0xB4 }
#define ESPNOW_CHANNEL 1 // WiFi channel (1-13)
#define TELEMETRY_INTERVAL_MS 700
#define TOUCHDOWN_ALTITUDE_THRESHOLD 5.0f // Altitude threshold for touchdown detection (in meters)

// In the case of the model velocity is extremely wrong, the access to the parachute is denied, and let to the CatsVega, to avoid the opening of it during ascension
#define MIN_ACCECTABLE_VELOCITY -1000
#define MAX_ACCECTABLE_VELOCITY 1000

// Kalman Constants
#define NUM_CALIBRATION_SAMPLES 200
#define STD_THRESHOLD 0.1f
#define SEA_LEVEL 63.0f  // Your launch site altitude above sea level (meters)

// Sea level pressure reference for barometric altitude calculation
// IMPORTANT: This must match your local atmospheric conditions!
// Option 1: Check local weather station for "sea level pressure" (QNH in aviation)
// Option 2: Calculate from known altitude: If at SEA_LEVEL meters reading P hPa,
//           then SEA_LEVEL_PRESSURE_HPA ≈ P / (1 - 0.0065 * SEA_LEVEL / 288.15)^5.255
// Example: At 63m reading 1010.28 hPa → sea level pressure ≈ 1017.5 hPa
#define SEA_LEVEL_PRESSURE_HPA 1017.5f  // Local sea level pressure in hPa

#define H_BIAS_PRESSURE_SENSOR 2.0f
#define GPS_BIAS 3.0f

// Barometer noise filtering
// BAROMETER_FILTER_WINDOW: Size of median filter window for pressure/altitude smoothing
// Smaller = faster response but more noise (1 = no filtering)
// Larger = smoother but more lag (recommended: 3-7)
// At 10Hz sampling: window=5 adds 50ms lag
#define BAROMETER_FILTER_WINDOW 5

#define STATE_INDEX_ALTITUDE 0
#define STATE_INDEX_VELOCITY 1
#define STATE_INDEX_QUAT_W 2
#define STATE_INDEX_QUAT_X 3
#define STATE_INDEX_QUAT_Y 4
#define STATE_INDEX_QUAT_Z 5

// GPS Configuration
#define GPS_FIX_TIMEOUT_MS 180000      // 3 minutes
#define GPS_MIN_FIX 3                  // Minimum fix type for valid GPS data (2 = 2D fix, 3 = 3D fix)
#define GPS_FIX_LOOKUP_INTERVAL_MS 500 // Interval between GPS fix status checks during initialization

// IMU Calibration
#define IMU_MINIMUM_CALIBRATION 3 // Minimum calibration level for IMU sensors (0-3)

#define GRAVITY 9.80665f


// Telemetry configuration
constexpr uint8_t RECEIVER_MAC_ADDRESS[] = { 0x34, 0xCD, 0xB0, 0x3D, 0x97, 0xFC };  // MAC dell'ESP32 ricevente: // 34:CD:B0:3D:97:FC
