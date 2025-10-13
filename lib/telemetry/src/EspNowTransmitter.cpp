#include "EspNowTransmitter.hpp"
#include <PacketManager.hpp>
#include <Logger.hpp>
#include <cstring>

// Static instance pointer for callbacks
EspNowTransmitter* EspNowTransmitter::instance = nullptr;

EspNowTransmitter::EspNowTransmitter(const uint8_t peerMacAddress[6], uint8_t wifiChannel)
    : channel(wifiChannel), initialized(false), packetsSent(0), packetsFailed(0), lastRSSI(0), sendSuccess(false)
{
    std::memcpy(peerMac, peerMacAddress, 6);
    
    // Create synchronization objects
    sendMutex = xSemaphoreCreateMutex();
    sendCompleteSemaphore = xSemaphoreCreateBinary();
    
    // Set static instance for callback access
    instance = this;
    
    LOG_INFO("EspNow", "Created transmitter for peer %02X:%02X:%02X:%02X:%02X:%02X on channel %d",
             peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4], peerMac[5], channel);
}

EspNowTransmitter::~EspNowTransmitter()
{
    if (initialized)
    {
        esp_now_deinit();
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
    }
    
    if (sendMutex)
    {
        vSemaphoreDelete(sendMutex);
    }
    
    if (sendCompleteSemaphore)
    {
        vSemaphoreDelete(sendCompleteSemaphore);
    }
    
    instance = nullptr;
    
    LOG_INFO("EspNow", "Transmitter destroyed");
}

ResponseStatusContainer EspNowTransmitter::init()
{
    if (initialized)
    {
        LOG_WARNING("EspNow", "Already initialized");
        return ResponseStatusContainer(0, "Already initialized");
    }
    
    LOG_INFO("EspNow", "Initializing ESP-NOW transmitter...");
    
    // Set WiFi mode to STA (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Wait for WiFi to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        LOG_ERROR("EspNow", "Failed to initialize ESP-NOW");
        return ResponseStatusContainer(-1, "ESP-NOW init failed");
    }
    
    // Register send callback
    if (esp_now_register_send_cb(onDataSent) != ESP_OK)
    {
        LOG_ERROR("EspNow", "Failed to register send callback");
        esp_now_deinit();
        return ResponseStatusContainer(-2, "Failed to register callback");
    }
    
    // Add peer
    if (!addPeer())
    {
        LOG_ERROR("EspNow", "Failed to add peer");
        esp_now_deinit();
        return ResponseStatusContainer(-3, "Failed to add peer");
    }
    
    initialized = true;
    
    LOG_INFO("EspNow", "ESP-NOW initialized successfully");
    LOG_INFO("EspNow", "Local MAC: %s", WiFi.macAddress().c_str());
    
    return ResponseStatusContainer(0, "Success");
}

ResponseStatusContainer EspNowTransmitter::transmit(Packet packet)
{
    if (!initialized)
    {
        LOG_ERROR("EspNow", "Not initialized");
        return ResponseStatusContainer(-1, "Not initialized");
    }
    
    // Take mutex for thread-safe transmission
    if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        LOG_ERROR("EspNow", "Failed to acquire send mutex");
        return ResponseStatusContainer(-2, "Send mutex timeout");
    }
    
    // Serialize packet
    std::vector<uint8_t> data = PacketManager::serialize(packet);
    
    if (data.empty() || data.size() > ESP_NOW_MAX_DATA_LEN)
    {
        xSemaphoreGive(sendMutex);
        LOG_ERROR("EspNow", "Invalid packet size: %d bytes (max %d)", data.size(), ESP_NOW_MAX_DATA_LEN);
        return ResponseStatusContainer(-3, "Invalid packet size");
    }
    
    LOG_DEBUG("EspNow", "Transmitting packet: msg=%d, chunk=%d/%d, size=%d bytes",
              packet.header.messageId, 
              packet.header.chunkIndex + 1, 
              packet.header.totalChunks,
              data.size());
    
    // Reset semaphore (ensure it's not already given)
    xSemaphoreTake(sendCompleteSemaphore, 0);
    
    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(peerMac, data.data(), data.size());
    
    if (result != ESP_OK)
    {
        xSemaphoreGive(sendMutex);
        packetsFailed++;
        LOG_ERROR("EspNow", "esp_now_send failed: %d", result);
        return ResponseStatusContainer(-4, "ESP-NOW send failed");
    }
    
    // Wait for send callback (timeout 1000ms)
    if (xSemaphoreTake(sendCompleteSemaphore, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        xSemaphoreGive(sendMutex);
        packetsFailed++;
        LOG_ERROR("EspNow", "Send timeout");
        return ResponseStatusContainer(-5, "Send timeout");
    }
    
    // Check send result from callback
    bool success = sendSuccess;
    xSemaphoreGive(sendMutex);
    
    if (success)
    {
        packetsSent++;
        LOG_DEBUG("EspNow", "Packet sent successfully (total: %lu)", packetsSent);
        return ResponseStatusContainer(0, "Success");
    }
    else
    {
        packetsFailed++;
        LOG_WARNING("EspNow", "Packet send failed (total failed: %lu)", packetsFailed);
        return ResponseStatusContainer(-6, "Peer did not receive packet");
    }
}

void EspNowTransmitter::getStats(uint32_t &sent, uint32_t &failed) const
{
    sent = packetsSent;
    failed = packetsFailed;
}

bool EspNowTransmitter::addPeer()
{
    // Check if peer already exists
    if (esp_now_is_peer_exist(peerMac))
    {
        LOG_INFO("EspNow", "Peer already exists, removing old entry");
        esp_now_del_peer(peerMac);
    }
    
    // Prepare peer info
    esp_now_peer_info_t peerInfo = {};
    std::memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false; // No encryption for simplicity
    peerInfo.ifidx = WIFI_IF_STA;
    
    // Add peer
    esp_err_t result = esp_now_add_peer(&peerInfo);
    
    if (result != ESP_OK)
    {
        LOG_ERROR("EspNow", "Failed to add peer: %d", result);
        return false;
    }
    
    LOG_INFO("EspNow", "Peer added successfully");
    return true;
}

void EspNowTransmitter::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (instance == nullptr)
    {
        return;
    }
    
    // Store send result
    instance->sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    
    // Signal completion
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(instance->sendCompleteSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
