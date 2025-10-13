#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "EspNowTransmitter.hpp"
#include "Logger.hpp"
#include <Packet.hpp>
#include <PacketManager.hpp>
#include <memory>
#include <cstdint>

/**
 * @brief Binary telemetry packet structure for efficient transmission.
 * 
 * This structure is tightly packed (no padding) for efficient transmission
 * over ESP-NOW. Total size is approximately 64 bytes.
 * 
 * All multi-byte values are little-endian (ESP32 native).
 */
#pragma pack(push, 1)
struct TelemetryPacket {
    uint32_t timestamp;     ///< Milliseconds since boot
    bool dataValid;         ///< True if sensor data was successfully collected
    
    struct {
        float accel_x, accel_y, accel_z;  ///< Accelerometer (m/s²)
        float gyro_x, gyro_y, gyro_z;     ///< Gyroscope (rad/s)
    } imu;
    
    struct {
        float pressure;     ///< Pressure (hPa)
        float temperature;  ///< Temperature (°C)
    } baro1;
    
    struct {
        float pressure;     ///< Pressure (hPa)
        float temperature;  ///< Temperature (°C)
    } baro2;
    
    struct {
        float latitude;     ///< Latitude (degrees)
        float longitude;    ///< Longitude (degrees)
        float altitude;     ///< GPS altitude (meters)
    } gps;
};
#pragma pack(pop)

/**
 * @brief Task that periodically collects sensor data and transmits it via ESP-NOW.
 * 
 * This task reads from SharedSensorData, serializes it to binary format,
 * divides it into Packet chunks, and transmits them using EspNowTransmitter.
 * 
 * Transmission rate is configurable via constructor.
 */
class TelemetryTask : public BaseTask
{
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    std::shared_ptr<EspNowTransmitter> transmitter;
    
    uint32_t transmitIntervalMs;
    uint32_t lastTransmitTime;
    
    // Statistics
    uint32_t messagesCreated;
    uint32_t packetsSent;
    uint32_t transmitErrors;

public:
    /**
     * @brief Construct a new Telemetry Task.
     * 
     * @param sensorData Shared sensor data to read from.
     * @param mutex Mutex protecting sensor data access.
     * @param espNowTransmitter ESP-NOW transmitter instance.
     * @param intervalMs Interval between transmissions in milliseconds (default 1000ms = 1Hz).
     */
    TelemetryTask(std::shared_ptr<SharedSensorData> sensorData,
                  SemaphoreHandle_t mutex,
                  std::shared_ptr<EspNowTransmitter> espNowTransmitter,
                  uint32_t intervalMs = 1000);

    /**
     * @brief Get transmission statistics.
     * 
     * @param messages Output: number of messages created.
     * @param packets Output: number of packets sent.
     * @param errors Output: number of transmission errors.
     */
    void getStats(uint32_t &messages, uint32_t &packets, uint32_t &errors) const;

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;

private:
    /**
     * @brief Collect current sensor data into a binary telemetry packet.
     * 
     * @param packet Reference to packet structure to fill with sensor data.
     * @return true if data collection successful, false on mutex timeout or error.
     */
    bool collectSensorData(TelemetryPacket &packet);
    
    /**
     * @brief Transmit a message by dividing it into packets and sending them.
     * 
     * @param message The message bytes to transmit.
     * @return true if all packets sent successfully.
     */
    bool transmitMessage(const std::vector<uint8_t> &message);
};
