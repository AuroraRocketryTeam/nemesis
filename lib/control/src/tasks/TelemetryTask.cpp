#include "TelemetryTask.hpp"
#include "esp_task_wdt.h"
#include <Arduino.h>

TelemetryTask::TelemetryTask(std::shared_ptr<SharedSensorData> sensorData,
                             SemaphoreHandle_t mutex,
                             std::shared_ptr<EspNowTransmitter> espNowTransmitter,
                             uint32_t intervalMs)
    : BaseTask("TelemetryTask"),
      sensorData(sensorData),
      dataMutex(mutex),
      transmitter(espNowTransmitter),
      transmitIntervalMs(intervalMs),
      lastTransmitTime(0),
      messagesCreated(0),
      packetsSent(0),
      transmitErrors(0)
{
    LOG_INFO("Telemetry", "Created with transmit interval: %lu ms", transmitIntervalMs);
    LOG_INFO("Telemetry", "Telemetry packet size: %d bytes", sizeof(TelemetryPacket));
}

void TelemetryTask::onTaskStart()
{
    LOG_INFO("Telemetry", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Telemetry", "Transmitter: %s", transmitter ? "OK" : "NULL");
    lastTransmitTime = millis();
}

void TelemetryTask::onTaskStop()
{
    LOG_INFO("Telemetry", "Task stopped - Stats: messages=%lu, packets=%lu, errors=%lu",
             messagesCreated, packetsSent, transmitErrors);
}

void TelemetryTask::taskFunction()
{
    uint32_t loopCount = 0;

    while (running)
    {
        esp_task_wdt_reset();
        
        // Check early for fast exit
        if (!running) break;
        
        uint32_t now = millis();
        
        // Check if it's time to transmit
        if (now - lastTransmitTime >= transmitIntervalMs)
        {
            lastTransmitTime = now;
            
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
                    messagesCreated++;
                    LOG_INFO("Telemetry", "Packet %lu transmitted successfully", messagesCreated);
                }
                else
                {
                    transmitErrors++;
                    LOG_WARNING("Telemetry", "Failed to transmit packet (errors: %lu)", transmitErrors);
                }
            }
        }
        
        // Log stats periodically
        if (loopCount % 10 == 0 && loopCount > 0)
        {
            uint32_t txSent, txFailed;
            if (transmitter)
            {
                transmitter->getStats(txSent, txFailed);
                LOG_INFO("Telemetry", "Stats: msgs=%lu, pkts=%lu, tx_ok=%lu, tx_fail=%lu, heap=%u",
                         messagesCreated, packetsSent, txSent, txFailed, ESP.getFreeHeap());
            }
        }
        
        loopCount++;
        
        // Check running flag frequently during delay (50ms chunks)
        uint32_t delayRemaining = transmitIntervalMs / 10; // Split into 10 chunks
        if (delayRemaining < 10) delayRemaining = 10;
        
        for (uint32_t i = 0; i < 10 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(delayRemaining));
        }
    }
}

bool TelemetryTask::collectSensorData(TelemetryPacket &packet)
{
    if (!sensorData || !dataMutex)
    {
        return false;
    }
    
    // Take mutex with timeout
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        LOG_WARNING("Telemetry", "Failed to acquire data mutex");
        return false;
    }
    
    try
    {
        // Initialize packet to zeros
        memset(&packet, 0, sizeof(TelemetryPacket));
        
        // Add timestamp and validity
        packet.timestamp = sensorData->timestamp;
        packet.dataValid = sensorData->dataValid;
        
        // IMU data - accelerometer
        auto accelOpt = sensorData->imuData.getData("accelerometer");
        if (accelOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(accelOpt.value()))
        {
            const auto &accelMap = std::get<std::map<std::string, float>>(accelOpt.value());
            packet.imu.accel_x = accelMap.count("x") ? accelMap.at("x") : 0.0f;
            packet.imu.accel_y = accelMap.count("y") ? accelMap.at("y") : 0.0f;
            packet.imu.accel_z = accelMap.count("z") ? accelMap.at("z") : 0.0f;
        }
        
        // IMU data - gyroscope (orientation)
        auto gyroOpt = sensorData->imuData.getData("orientation");
        if (gyroOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(gyroOpt.value()))
        {
            const auto &gyroMap = std::get<std::map<std::string, float>>(gyroOpt.value());
            packet.imu.gyro_x = gyroMap.count("x") ? gyroMap.at("x") : 0.0f;
            packet.imu.gyro_y = gyroMap.count("y") ? gyroMap.at("y") : 0.0f;
            packet.imu.gyro_z = gyroMap.count("z") ? gyroMap.at("z") : 0.0f;
        }
        
        // Barometer 1
        auto baro1PressOpt = sensorData->baroData1.getData("pressure");
        if (baro1PressOpt.has_value() && std::holds_alternative<float>(baro1PressOpt.value()))
        {
            packet.baro1.pressure = std::get<float>(baro1PressOpt.value());
        }
        
        auto baro1TempOpt = sensorData->baroData1.getData("temperature");
        if (baro1TempOpt.has_value() && std::holds_alternative<float>(baro1TempOpt.value()))
        {
            packet.baro1.temperature = std::get<float>(baro1TempOpt.value());
        }
        
        // Barometer 2
        auto baro2PressOpt = sensorData->baroData2.getData("pressure");
        if (baro2PressOpt.has_value() && std::holds_alternative<float>(baro2PressOpt.value()))
        {
            packet.baro2.pressure = std::get<float>(baro2PressOpt.value());
        }
        
        auto baro2TempOpt = sensorData->baroData2.getData("temperature");
        if (baro2TempOpt.has_value() && std::holds_alternative<float>(baro2TempOpt.value()))
        {
            packet.baro2.temperature = std::get<float>(baro2TempOpt.value());
        }
        
        // GPS data
        auto latOpt = sensorData->gpsData.getData("latitude");
        if (latOpt.has_value() && std::holds_alternative<float>(latOpt.value()))
        {
            packet.gps.latitude = std::get<float>(latOpt.value());
        }
        
        auto lonOpt = sensorData->gpsData.getData("longitude");
        if (lonOpt.has_value() && std::holds_alternative<float>(lonOpt.value()))
        {
            packet.gps.longitude = std::get<float>(lonOpt.value());
        }
        
        auto altOpt = sensorData->gpsData.getData("altitude");
        if (altOpt.has_value() && std::holds_alternative<float>(altOpt.value()))
        {
            packet.gps.altitude = std::get<float>(altOpt.value());
        }
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Telemetry", "Exception collecting data: %s", e.what());
        xSemaphoreGive(dataMutex);
        return false;
    }
    
    xSemaphoreGive(dataMutex);
    
    return true;
}

bool TelemetryTask::transmitMessage(const std::vector<uint8_t> &message)
{
    if (!transmitter || message.empty())
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
        ResponseStatusContainer result = transmitter->transmit(packets[i]);
        
        if (result.getCode() == 0)
        {
            packetsSent++;
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
    messages = messagesCreated;
    packets = packetsSent;
    errors = transmitErrors;
}
