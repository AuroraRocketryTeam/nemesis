#include "TelemetryTask.hpp"

constexpr float TROPOSPHERE_HEIGHT = 11000.f; // Troposphere height [m]
constexpr float a = 0.0065f;                  // Troposphere temperature gradient [deg/m]
constexpr float R = 287.05f;                  // Air gas constant [J/Kg/K]
#define n (GRAVITY / (R * a))
#define nInv ((R * a) / GRAVITY)

float relAltitude_tele(float pressure, float pressureRef = 99725.0f,
                       float temperatureRef = 291.41f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}

TelemetryTask::TelemetryTask(std::shared_ptr<Nemesis> model,
                             SemaphoreHandle_t modelMutex,
                             std::shared_ptr<EspNowTransmitter> espNowTransmitter,
                             uint32_t intervalMs)
    : BaseTask("TelemetryTask"),
      _model(model),
      _modelMutex(modelMutex),
      _transmitter(espNowTransmitter),
      _transmitIntervalMs(intervalMs),
      _lastTransmitTime(0),
      _messagesCreated(0),
      _packetsSent(0),
      _transmitErrors(0)
{
    LOG_INFO("Telemetry", "Created with transmit interval: %lu ms", _transmitIntervalMs);
    LOG_INFO("Telemetry", "Telemetry packet size: %d bytes", sizeof(TelemetryPacket));
}

void TelemetryTask::onTaskStart()
{
    LOG_INFO("Telemetry", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Telemetry", "Transmitter: %s", _transmitter ? "OK" : "NULL");
    _lastTransmitTime = millis();
}

void TelemetryTask::onTaskStop()
{
    LOG_INFO("Telemetry", "Task stopped - Stats: messages=%lu, packets=%lu, errors=%lu",
             _messagesCreated, _packetsSent, _transmitErrors);
}

void TelemetryTask::taskFunction()
{
    uint32_t loopCount = 0;

    while (running)
    {
        esp_task_wdt_reset();

        // Check early for fast exit
        if (!running)
            break;

        uint32_t now = millis();

        // Check if it's time to transmit
        if (now - _lastTransmitTime >= _transmitIntervalMs)
        {
            _lastTransmitTime = now;

            // Collect sensor data into binary packet
            TelemetryPacket packet;
            if (collectSensorData(packet) && running)
            {
                // Convert packet to byte array
                std::vector<uint8_t> message(sizeof(TelemetryPacket));
                memcpy(message.data(), &packet, sizeof(TelemetryPacket));

                LOG_DEBUG("Telemetry", "Packet size: %d bytes", message.size());

                // Transmit message
                if (transmitMessage(message))
                {
                    _messagesCreated++;
                    LOG_INFO("Telemetry", "Packet %lu transmitted successfully", _messagesCreated);
                }
                else
                {
                    _transmitErrors++;
                    LOG_WARNING("Telemetry", "Failed to transmit packet (errors: %lu)", _transmitErrors);
                }
            }
        }

        // Log stats periodically
        if (loopCount % 10 == 0 && loopCount > 0)
        {
            uint32_t txSent, txFailed;
            if (_transmitter)
            {
                _transmitter->getStats(txSent, txFailed);
                LOG_INFO("Telemetry", "Stats: msgs=%lu, pkts=%lu, tx_ok=%lu, tx_fail=%lu, heap=%u",
                         _messagesCreated, _packetsSent, txSent, txFailed, ESP.getFreeHeap());
            }
        }

        loopCount++;

        // Check running flag frequently during delay (50ms chunks)
        uint32_t delayRemaining = _transmitIntervalMs / 10; // Split into 10 chunks
        if (delayRemaining < 10)
            delayRemaining = 10;

        for (uint32_t i = 0; i < 10 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(delayRemaining));
        }
    }
}

bool TelemetryTask::collectSensorData(TelemetryPacket &packet)
{
    if (!_model || !_modelMutex)
    {
        return false;
    }

    // Take mutex with timeout
    if (xSemaphoreTake(_modelMutex, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        LOG_WARNING("Telemetry", "Failed to acquire data mutex");
        return false;
    }

    try
    {
        // Initialize packet to zeros
        memset(&packet, 0, sizeof(TelemetryPacket));

        // Add timestamp and validity
        packet.timestamp = millis();
        packet.dataValid = true;

        auto bno055Data = _model->getBNO055Data();

        // IMU data - accelerometer
        packet.imu.accel_x = bno055Data->acceleration_x;
        packet.imu.accel_y = bno055Data->acceleration_y;
        packet.imu.accel_z = bno055Data->acceleration_z;
        LOG_DEBUG("Telemetry", "ACC_X: %.2f, ACC_Y: %.2f, ACC_Z: %.2f", packet.imu.accel_x, packet.imu.accel_y, packet.imu.accel_z);

        // IMU data - gyroscope (orientation)
        packet.imu.gyro_x = bno055Data->orientation_x;
        packet.imu.gyro_y = bno055Data->orientation_y;
        packet.imu.gyro_z = bno055Data->orientation_z;

        // Barometer 1
        auto baro1Data = _model->getMS561101BA03Data_1();
        
        packet.baro1.pressure = baro1Data->pressure;
        packet.baro1.temperature = baro1Data->temperature;

        // Barometer 2
        auto baro2Data = _model->getMS561101BA03Data_2();

        packet.baro2.pressure = baro2Data->pressure;
        packet.baro2.temperature = baro2Data->temperature;

        // GPS data
        auto gpsData = _model->getGPSData();

        packet.gps.latitude = gpsData->latitude;
        packet.gps.longitude = gpsData->longitude;
        packet.gps.altitude = gpsData->altitude;

        LOG_DEBUG("TELEMETRY", "GPS ALT: %.2f", packet.gps.altitude);
        LOG_DEBUG("TELEMETRY", "GPS LAT: %.6f", packet.gps.latitude);
        LOG_DEBUG("TELEMETRY", "GPS LON: %.6f", packet.gps.longitude);
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Telemetry", "Exception collecting data: %s", e.what());
        xSemaphoreGive(_modelMutex);
        return false;
    }

    xSemaphoreGive(_modelMutex);

    return true;
}

bool TelemetryTask::transmitMessage(const std::vector<uint8_t> &message)
{
    if (!_transmitter || message.empty())
    {
        return false;
    }

    // Divide message into packets
    std::vector<Packet> packets = PacketManager::divideMessage(message.data(), message.size());

    if (packets.empty())
    {
        LOG_ERROR("Telemetry", "Failed to divide message into packets");
        return false;
    }

    LOG_DEBUG("Telemetry", "Transmitting %d packets", packets.size());

    // Send each packet
    bool allSuccess = true;
    for (size_t i = 0; i < packets.size() && running; i++)
    {
        ResponseStatusContainer result = _transmitter->transmit(packets[i]);

        if (result.getCode() == 0)
        {
            _packetsSent++;
        }
        else
        {
            LOG_WARNING("Telemetry", "Packet %d/%d failed: %s",
                        i + 1, packets.size(), result.getDescription().c_str());
            allSuccess = false;
            // Continue sending remaining packets even if one fails
        }

        // Small delay between packets to avoid overwhelming receiver
        if (i < packets.size() - 1 && running)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    return allSuccess;
}

void TelemetryTask::getStats(uint32_t &messages, uint32_t &packets, uint32_t &errors) const
{
    messages = _messagesCreated;
    packets = _packetsSent;
    errors = _transmitErrors;
}
